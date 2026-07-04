#!/usr/bin/env python3
"""검출 시뮬레이터 노드 — 길남님 검출기 + 태민 GT 스트림(§4.4)의 대역.
/ov_msckf/poseimu (VIO 포즈)를 구독해 창문을 카메라에 투영하고,
§5.1 메시지(JSON)를 /window_detections 로 반환한다.
실제 파이프라인에서는 이 노드 전체가 실제 검출 노드로 교체된다.
"""
import json
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

# §2: 원본 해상도. TODO: §6 확정되면 intrinsics 교체 (720p 가상 카메라 임시값)
W, H = 1280, 720
FX, FY, CX, CY = 600.0, 600.0, 640.0, 360.0

# TODO: 실제 extrinsics 있으면 교체 (EuRoC cam0->IMU)
T_IC = np.array([
    [ 0.0148655429818, -0.999880929698,   0.00414029679422, -0.0216401454975],
    [ 0.999557249008,   0.0149672133247,  0.0257155299480,  -0.0646769867680],
    [-0.0257744366974,  0.00375618835797, 0.999660727178,    0.00981073058949],
    [0, 0, 0, 1]])
R_IC, p_IC = T_IC[:3, :3], T_IC[:3, 3]

DETECT_PERIOD = 0.5   # 검출 주기 (초) — 초당 2회
PIX_NOISE = 1.0

WINDOW_DEFS = [
    {"order_index": 0, "color": "red",   "center": np.array([11.1, -8.3, -2.0]), "size": 1.0},
    {"order_index": 1, "color": "green", "center": np.array([ 7.6, -6.2, -2.7]), "size": 1.0},
]
FACING_POINT = np.array([4.5, -4.5, -1.5])   # 창문이 궤적 중심부를 향하도록


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]])


def window_corners_3d(wdef):
    """§4.3 순서: 좌상 → 우상 → 우하 → 좌하 (정면 기준)"""
    c = wdef["center"]; s = wdef["size"] / 2.0
    up = np.array([0.0, 0.0, 1.0])
    n = FACING_POINT - c; n[2] = 0.0; n /= np.linalg.norm(n)
    right = np.cross(up, n)
    return np.array([c - right*s + up*s, c + right*s + up*s,
                     c + right*s - up*s, c - right*s - up*s])


class WindowSimNode(Node):
    def __init__(self):
        super().__init__("window_sim")
        for wdef in WINDOW_DEFS:
            wdef["corners3d"] = window_corners_3d(wdef)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, "/ov_msckf/poseimu", self.pose_cb, 10)
        self.pub = self.create_publisher(String, "/window_detections", 10)
        self.rng = np.random.default_rng(42)
        self.last_t = -1e18
        self.frame_id = 0
        self.n_pub = 0
        self.get_logger().info("검출 시뮬레이터 시작 — VIO 포즈 대기 중")

    def pose_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t < self.last_t - 1.0:
            self.get_logger().warn("시간 역행 감지 — 검출 주기 리셋")
            self.last_t = -1e18
        if t - self.last_t < DETECT_PERIOD:
            return
        if t - self.last_t < DETECT_PERIOD:
            return
        self.last_t = t
        self.frame_id += 1

        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        R_WI = quat_to_rot(q.x, q.y, q.z, q.w)
        p_WI = np.array([p.x, p.y, p.z])
        R_WC = R_WI @ R_IC
        c_W = p_WI + R_WI @ p_IC

        frame_windows = []
        for wdef in WINDOW_DEFS:
            corners_px, vis = [], []
            for P in wdef["corners3d"]:
                p_C = R_WC.T @ (P - c_W)
                if p_C[2] < 0.5:
                    corners_px.append([-1.0, -1.0]); vis.append(0)
                    continue
                u = FX * p_C[0] / p_C[2] + CX + self.rng.normal(0, PIX_NOISE)
                v = FY * p_C[1] / p_C[2] + CY + self.rng.normal(0, PIX_NOISE)
                inside = (0 <= u < W) and (0 <= v < H)
                corners_px.append([round(float(u), 2), round(float(v), 2)])
                vis.append(1 if inside else 0)   # §4.2 정책 C
            if sum(vis) < 2:
                continue
            good = [c for c, vf in zip(corners_px, vis) if vf]
            center = [round(sum(x) / len(x), 2) for x in zip(*good)]
            det_conf = round(float(self.rng.uniform(0.3, 0.6)
                                   if self.rng.random() < 0.05
                                   else self.rng.uniform(0.85, 0.99)), 2)
            frame_windows.append({
                "order_index": wdef["order_index"], "color": wdef["color"],
                "corners": corners_px, "corner_vis": vis, "center": center,
                "det_conf": det_conf,
                "color_conf": round(float(self.rng.uniform(0.9, 0.99)), 2)})

        if not frame_windows:
            return
        out = {"timestamp": int(msg.header.stamp.sec) * 1_000_000_000
                            + int(msg.header.stamp.nanosec),   # §5.1: int ns
               "frame_id": self.frame_id,
               "windows": frame_windows}
        m = String(); m.data = json.dumps(out)
        self.pub.publish(m)
        self.n_pub += 1
        if self.n_pub % 20 == 1:
            self.get_logger().info(f"검출 메시지 {self.n_pub}건 발행 (창문 {len(frame_windows)}개 보임)")


def main():
    rclpy.init()
    node = WindowSimNode()   # recon 파일이므로 WindowReconNode()
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
