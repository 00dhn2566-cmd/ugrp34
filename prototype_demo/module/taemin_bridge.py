"""태민의 window_recon_node.py 를 **수정 없이** PyBullet 데모에 연결한다.

두 가지 모드가 같은 데이터 경로를 쓴다.

  offline  ROS2 없이. module/contract.py 가 rclpy/std_msgs/geometry_msgs 를 최소
           스텁으로 채운 뒤 그의 파일을 그대로 import 하고, WindowReconNode 의
           pose_cb / det_cb / report 를 직접 호출한다. 그의 삼각측량 코드가 진짜로
           실행된다. DDS·타이머·QoS 는 검증되지 않는다.

  ros      ROS2 로. 우리가 /ov_msckf/poseimu 와 /window_detections 를 발행하고,
           그의 노드를 별도 프로세스로 띄우면 구독해서 /window_positions 를 낸다.

계약을 맞추기 위해 반드시 지키는 것 두 가지
--------------------------------------------
1. **intrinsics 를 그의 값에 맞춘다.** 그의 노드는 FX/FY/CX/CY 를 파일에 박아두고
   있다. 우리가 다른 화각으로 렌더하면 그의 삼각측량은 에러 없이 *틀린 답*을 낸다.
   그래서 prototype_demo/config/camera.yaml 이 아니라 contract.intrinsics() 를 쓴다.
2. **카메라 pose 가 아니라 IMU pose 를 보낸다.** 그의 노드는 R_WC = R_WI @ R_IC 로
   카메라 자세를 스스로 만든다. 카메라 pose 를 그대로 주면 T_IC 가 두 번 곱해진다.
   contract.camera_pose_to_imu_pose() 로 변환해서 보낸다.
"""
from __future__ import annotations

import json
import os
import sys
from typing import Dict, List, Tuple

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PROTO = os.path.dirname(_HERE)
_TEAM = os.path.dirname(_PROTO)
for _p in (os.path.join(_TEAM, "reinforcement_yunho"),
           os.path.join(_TEAM, "overall_gilnam", "vision")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from module import contract  # noqa: E402


# --------------------------------------------------------------------------- #
# 데이터 생성 — PyBullet 씬을 그의 규격으로 관측한다
# --------------------------------------------------------------------------- #
def _rot_from_quat_xyzw(q) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def _rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    from sim.pybullet_stream import _rot_to_quat_xyzw as f
    return f(R)


def observe(env, model, color_config, poses, conf: float = 0.25,
            fps: float = 30.0, scale: float = 1.0) -> Tuple[List[dict], Dict]:
    """PyBullet 씬을 렌더+검출해서 (IMU pose, §5.1 검출) 쌍의 시퀀스를 만든다.

    반환 항목: {"t_ns", "p_WI", "q_WI_xyzw", "detection"(§5.1 dict or None)}
    """
    import pybullet as p
    from sim import pybullet_stream as pbs
    from infer_stream import infer_frame
    from sim.pybullet_stream import infer_frame_multiclass, is_multiclass

    intr = contract.intrinsics()          # ★ 그의 값
    T_IC = contract.T_imu_cam()
    # 색은 전적으로 모델을 따른다. 학습된 3클래스 헤드가 conf 0.9 대로 맞히는데,
    # HSV 후처리는 뚫린 창문에서 폴리곤 내부가 배경이라 inlier 0.23~0.41 로 떨어져
    # 전부 unknown 드롭시킨다 (green 이 한 번도 복원 안 되던 원인). single_cls
    # 가중치를 쓸 때만 길남의 HSV 경로로 되돌아간다.
    multi = pbs.is_multiclass(model)
    out, n_det = [], 0
    for i, (p_WC, q_WC) in enumerate(poses):
        R_WC = _rot_from_quat_xyzw(q_WC)
        img = pbs.render_frame(p, env.CLIENT, p_WC, p_WC + R_WC[:, 2], intr, scale=scale)
        t_ns = int(round(i * 1e9 / fps))
        msg = (pbs.infer_frame_multiclass(model, img, t_ns, i, conf) if multi
               else infer_frame(model, img, t_ns, i, color_config, conf))
        n_det += len(msg["windows"])
        R_WI, p_WI = contract.camera_pose_to_imu_pose(R_WC, p_WC, T_IC)
        out.append({"t_ns": t_ns,
                    "p_WI": [float(v) for v in p_WI],
                    "q_WI_xyzw": [float(v) for v in _rot_to_quat_xyzw(R_WI)],
                    "detection": msg if msg["windows"] else None})
    return out, {"frames": len(out), "detections": n_det,
                 "colour_from": "model" if multi else "hsv(color_judge)",
                 "intrinsics": intr,
                 "note": "intrinsics taken from window_recon_node.py, not camera.yaml"}


# --------------------------------------------------------------------------- #
# offline: ROS 없이 그의 노드 코드를 직접 구동
# --------------------------------------------------------------------------- #
def run_offline(samples: List[dict], pose_rate_hz: float = 200.0,
                verbose: bool = True, det_conf_min: float | None = None) -> List[dict]:
    """그의 WindowReconNode 를 스텁 ROS 위에서 그대로 돌리고 결과를 돌려준다.

    pose 는 그의 POSE_TOL_NS(20 ms) 안에 들어와야 매칭되므로, 관측 사이를 선형
    보간해 pose_rate_hz 로 채워 넣는다 (실제로는 OpenVINS 가 그 역할).
    """
    mod = contract.load_node_module()
    if det_conf_min is not None:
        # 그의 파일은 그대로 두고 로드된 모듈 속성만 바꾼다 — 문턱을 낮추면 관측이
        # 얼마나 늘고 오차가 어떻게 되는지 재서 그에게 줄 근거를 만들기 위한 실험.
        mod.DET_CONF_MIN = float(det_conf_min)
    node = mod.WindowReconNode()

    # pose 스트림 (관측 시각 사이를 채움)
    ts = [s["t_ns"] for s in samples]
    dt_ns = int(1e9 / pose_rate_hz)
    P = np.array([s["p_WI"] for s in samples])
    Q = np.array([s["q_WI_xyzw"] for s in samples])
    dense_t = np.arange(ts[0], ts[-1] + 1, dt_ns)
    Pi = np.stack([np.interp(dense_t, ts, P[:, k]) for k in range(3)], axis=1)
    Qi = np.stack([np.interp(dense_t, ts, Q[:, k]) for k in range(4)], axis=1)
    Qi /= np.linalg.norm(Qi, axis=1, keepdims=True)

    det_by_t = {s["t_ns"]: s["detection"] for s in samples if s["detection"]}
    di = 0
    for t, pw, qw in zip(dense_t, Pi, Qi):
        node.pose_cb(contract.PoseMsg(int(t), pw, qw))
        while di < len(ts) and ts[di] <= t:
            d = det_by_t.get(ts[di])
            if d is not None:
                node.det_cb(contract.StringMsg(json.dumps(d)))
            di += 1

    node.report()          # 그의 리포트 코드 (시차각 판정 포함)
    published = getattr(node, "published", [])
    if not published:
        if verbose:
            print("  [taemin] 발행된 결과 없음 — 시차각 부족이거나 관측 부족")
        return []
    return json.loads(published[-1].data)["windows"]


# --------------------------------------------------------------------------- #
# ros: 토픽 발행 (그의 노드는 별도 프로세스에서 그대로 실행)
# --------------------------------------------------------------------------- #
def run_ros_publisher(samples: List[dict], pose_rate_hz: float = 200.0,
                      realtime: bool = True) -> None:
    """/ov_msckf/poseimu + /window_detections 를 발행한다. 태민 노드는 따로 띄울 것.

        터미널 1:  python <team>/visual_imaging_taemin/window_recon_node.py
        터미널 2:  bash prototype_demo/scripts/run_taemin.sh --ros
    """
    import time
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String

    rclpy.init()
    node = Node("pybullet_bridge")
    pub_pose = node.create_publisher(PoseWithCovarianceStamped, contract.TOPIC_POSE, 50)
    pub_det = node.create_publisher(String, contract.TOPIC_DETECTIONS, 50)
    node.get_logger().info(
        f"발행 시작 — {contract.TOPIC_POSE} @{pose_rate_hz:.0f}Hz, {contract.TOPIC_DETECTIONS}")

    ts = [s["t_ns"] for s in samples]
    dt_ns = int(1e9 / pose_rate_hz)
    P = np.array([s["p_WI"] for s in samples])
    Q = np.array([s["q_WI_xyzw"] for s in samples])
    dense_t = np.arange(ts[0], ts[-1] + 1, dt_ns)
    Pi = np.stack([np.interp(dense_t, ts, P[:, k]) for k in range(3)], axis=1)
    Qi = np.stack([np.interp(dense_t, ts, Q[:, k]) for k in range(4)], axis=1)
    Qi /= np.linalg.norm(Qi, axis=1, keepdims=True)

    det_by_t = {s["t_ns"]: s["detection"] for s in samples if s["detection"]}
    t0 = time.time()
    di = 0
    for t, pw, qw in zip(dense_t, Pi, Qi):
        m = PoseWithCovarianceStamped()
        m.header.stamp.sec = int(t // 1_000_000_000)
        m.header.stamp.nanosec = int(t % 1_000_000_000)
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = pw
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = qw
        pub_pose.publish(m)
        while di < len(ts) and ts[di] <= t:
            d = det_by_t.get(ts[di])
            if d is not None:
                s = String(); s.data = json.dumps(d)
                pub_det.publish(s)
            di += 1
        if realtime:
            target = t0 + (t - dense_t[0]) / 1e9
            lag = target - time.time()
            if lag > 0:
                time.sleep(lag)
        rclpy.spin_once(node, timeout_sec=0.0)

    node.get_logger().info("발행 완료")
    node.destroy_node()
    rclpy.shutdown()
