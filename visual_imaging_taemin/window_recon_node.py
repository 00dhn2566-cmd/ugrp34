#!/usr/bin/env python3
"""창문 3D 복원 노드 — 태민 파트의 실제 모듈.
/window_detections (§5.1 JSON) + /ov_msckf/poseimu 를 구독해,
포즈 버퍼에서 타임스탬프를 맞춰 corner별 관측 시선을 누적하고,
2초마다 삼각측량 결과(창문 중심·크기)를 로그 및 /window_positions 로 반환한다.
"""
import json
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

# TODO: §6 확정되면 실제 카메라 파라미터로 교체 (시뮬 노드와 동일해야 함)
FX, FY, CX, CY = 600.0, 600.0, 640.0, 360.0

# TODO: 실제 extrinsics 있으면 교체 (EuRoC cam0->IMU)
T_IC = np.array([
    [ 0.0148655429818, -0.999880929698,   0.00414029679422, -0.0216401454975],
    [ 0.999557249008,   0.0149672133247,  0.0257155299480,  -0.0646769867680],
    [-0.0257744366974,  0.00375618835797, 0.999660727178,    0.00981073058949],
    [0, 0, 0, 1]])
R_IC, p_IC = T_IC[:3, :3], T_IC[:3, 3]

DET_CONF_MIN = 0.7        # §5.2 신뢰도 필터
MIN_PARALLAX_DEG = 2.0    # 삼각측량 최소 시차각 문턱
POSE_TOL_NS = 20_000_000  # 검출-포즈 타임스탬프 허용 오차 20ms
REPORT_PERIOD = 2.0       # 결과 갱신 주기 (초)


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]])


class CornerAccumulator:
    """관측 시선을 하나씩 받아 최소자승 행렬을 O(1)로 누적한다. 필요할 때만 3x3 해를 구한다."""
    def __init__(self):
        self.A = np.zeros((3, 3))
        self.b = np.zeros(3)
        self.dirs = []

    def add(self, c, d):
        M = np.eye(3) - np.outer(d, d)
        self.A += M
        self.b += M @ c
        self.dirs.append(d)

    def solve(self):
        if len(self.dirs) < 2:
            return None, 0.0
        p = np.linalg.solve(self.A, self.b)
        D = np.array(self.dirs)
        ang = float(np.degrees(np.arccos(np.clip((D @ D.T).min(), -1, 1))))
        return p, ang


class WindowReconNode(Node):
    def __init__(self):
        super().__init__("window_recon")
        self.pose_buf = deque(maxlen=600)   # (t_ns, R_WI, p_WI) 약 30초 분량
        self.acc = {}                        # (order_index, corner번호) -> CornerAccumulator
        self.colors = {}                     # order_index -> color (디버깅용)
        self.n_det = 0
        self.n_nopose = 0
        self.create_subscription(
            PoseWithCovarianceStamped, "/ov_msckf/poseimu", self.pose_cb, 50)
        self.create_subscription(String, "/window_detections", self.det_cb, 50)
        self.pub = self.create_publisher(String, "/window_positions", 10)
        self.create_timer(REPORT_PERIOD, self.report)
        self.get_logger().info("복원 노드 시작 — 검출/포즈 대기 중")

    def pose_cb(self, msg):
        t_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if self.pose_buf and t_ns < self.pose_buf[-1][0] - 1_000_000_000:
            self.get_logger().warn("시간 역행 감지 (bag 재생?) — 누적 상태 전체 리셋")
            self.pose_buf.clear()
            self.acc.clear()
            self.colors.clear()
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.pose_buf.append((t_ns, quat_to_rot(q.x, q.y, q.z, q.w),
                              np.array([p.x, p.y, p.z])))

    def pose_at(self, t_ns):
        """버퍼에서 가장 가까운 포즈 (허용 오차 내)"""
        best = None
        best_dt = POSE_TOL_NS + 1
        for tb, R, p in self.pose_buf:
            dt = abs(tb - t_ns)
            if dt < best_dt:
                best_dt = dt
                best = (R, p)
        return best

    def det_cb(self, msg):
        det = json.loads(msg.data)
        pose = self.pose_at(det["timestamp"])
        if pose is None:
            self.n_nopose += 1
            if self.n_nopose % 10 == 1:
                self.get_logger().warn(f"타임스탬프 매칭 실패 {self.n_nopose}건 (포즈 버퍼에 없음)")
            return
        R_WI, p_WI = pose
        R_WC = R_WI @ R_IC
        c_W = p_WI + R_WI @ p_IC

        for win in det["windows"]:
            if win["det_conf"] < DET_CONF_MIN:
                continue
            oi = win["order_index"]
            self.colors[oi] = win.get("color", "?")
            for ci in range(4):
                if win["corner_vis"][ci] != 1:
                    continue
                u, v = win["corners"][ci]
                d_C = np.array([(u - CX) / FX, (v - CY) / FY, 1.0])
                d_C /= np.linalg.norm(d_C)
                self.acc.setdefault((oi, ci), CornerAccumulator()).add(c_W, R_WC @ d_C)
        self.n_det += 1

    def report(self):
        if not self.acc:
            return
        results = []
        for oi in sorted({k[0] for k in self.acc}):
            pts, min_ang, n_obs = [], 1e9, 0
            for ci in range(4):
                a = self.acc.get((oi, ci))
                if a is None:
                    break
                p, ang = a.solve()
                if p is None:
                    break
                pts.append(p)
                min_ang = min(min_ang, ang)
                n_obs += len(a.dirs)
            if len(pts) < 4:
                continue
            if min_ang < MIN_PARALLAX_DEG:
                self.get_logger().info(
                    f"W{oi}: 시차각 {min_ang:.1f}도 — 아직 신뢰 불가 (관측 누적 중)")
                continue
            pts = np.array(pts)
            center = pts.mean(axis=0)
            width = float(np.linalg.norm(pts[1] - pts[0]))
            height = float(np.linalg.norm(pts[2] - pts[1]))
            self.get_logger().info(
                f"W{oi}({self.colors.get(oi)}): center=({center[0]:.2f}, {center[1]:.2f}, "
                f"{center[2]:.2f})m, {width:.2f}x{height:.2f}m, "
                f"관측 {n_obs}개, 최소 시차각 {min_ang:.1f}도")
            results.append({
                "order_index": oi, "color": self.colors.get(oi),
                "center_w": [round(float(x), 3) for x in center],
                "corners_w": [[round(float(x), 3) for x in p] for p in pts],
                "width": round(width, 3), "height": round(height, 3),
                "n_obs": n_obs, "min_parallax_deg": round(min_ang, 1)})
        if results:
            m = String()
            m.data = json.dumps({"windows": results})
            self.pub.publish(m)


def main():
    rclpy.init()
    node = WindowReconNode()   # recon 파일이므로 WindowReconNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
