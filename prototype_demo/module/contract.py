"""태민 노드의 계약을 그의 소스에서 직접 읽어온다 — 복제하지 않는다.

WHY
---
`visual_imaging_taemin/window_recon_node.py` 는 카메라 intrinsics 와 카메라-IMU
외부파라미터를 파일 안에 상수로 박아두고 있다. 우리가 PyBullet 쪽에 같은 숫자를
다시 적어두면 둘이 조용히 어긋난다 (그가 §6 확정 후 값을 바꾸면 우리만 옛날 값).

그래서 여기서는 그의 소스를 **AST 로 파싱해서 상수를 읽는다**. import 가 아니라
파싱이므로 `rclpy` 가 없어도 되고, 그의 파일은 읽기만 한다.

`sim/pybullet_stream.py` 가 길남의 `infer_stream` 을 그대로 호출하고,
`rl/domain.py` 가 학습 렌더러의 배경 생성기를 그대로 import 하는 것과 같은 규율.

또 하나: 그의 노드는 `rclpy` / `geometry_msgs` / `std_msgs` 를 모듈 최상단에서
import 한다. ROS2 가 없는 기계에서도 그의 *알고리즘* 을 그대로 돌려보기 위해
:func:`load_node_module` 이 그 세 패키지를 최소 스텁으로 채운 뒤 그의 파일을
있는 그대로 실행한다. 그의 코드는 수정되지 않는다.
"""
from __future__ import annotations

import ast
import importlib.util
import os
import sys
import types
from typing import Any, Dict

import numpy as np

_TEAM = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RECON_NODE = os.path.join(_TEAM, "visual_imaging_taemin", "window_recon_node.py")

# 그의 노드가 쓰는 토픽 이름 (계약)
TOPIC_POSE = "/ov_msckf/poseimu"          # OpenVINS 가 내는 IMU pose
TOPIC_DETECTIONS = "/window_detections"   # 우리가 넣어야 하는 §5.1 JSON
TOPIC_POSITIONS = "/window_positions"     # 그가 내보내는 결과


def read_constants(path: str = RECON_NODE) -> Dict[str, Any]:
    """FX/FY/CX/CY, T_IC, 임계값들을 그의 소스에서 파싱해 반환한다."""
    with open(path, encoding="utf-8") as f:
        tree = ast.parse(f.read(), filename=path)
    out: Dict[str, Any] = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        targets = [t.id for t in node.targets if isinstance(t, ast.Name)]
        # FX, FY, CX, CY = 600.0, 600.0, 640.0, 360.0  (튜플 언패킹)
        if not targets and isinstance(node.targets[0], ast.Tuple):
            names = [e.id for e in node.targets[0].elts if isinstance(e, ast.Name)]
            try:
                vals = ast.literal_eval(node.value)
            except ValueError:
                continue
            out.update(dict(zip(names, vals)))
            continue
        for name in targets:
            if name == "T_IC":                      # np.array([...]) 형태
                if isinstance(node.value, ast.Call):
                    try:
                        out["T_IC"] = np.array(ast.literal_eval(node.value.args[0]),
                                               dtype=float)
                    except (ValueError, IndexError):
                        pass
                continue
            try:
                out[name] = ast.literal_eval(node.value)
            except ValueError:
                pass
    missing = [k for k in ("FX", "FY", "CX", "CY", "T_IC") if k not in out]
    if missing:
        raise RuntimeError(f"{path} 에서 상수를 못 읽었습니다: {missing}")
    return out


def intrinsics(path: str = RECON_NODE) -> Dict[str, float]:
    """그의 노드가 가정하는 카메라 규격. **우리가 여기에 맞춰 렌더해야 한다.**

    그가 다른 값을 가정하는데 우리가 다른 화각으로 렌더하면, 그의 삼각측량은
    조용히 틀린 답을 낸다 (에러가 아니라 값이 어긋남). 그래서 PyBullet 카메라는
    prototype/config/camera.yaml 이 아니라 **이 값**으로 세팅한다.
    """
    c = read_constants(path)
    return {"width": 1280.0, "height": 720.0,
            "fx": float(c["FX"]), "fy": float(c["FY"]),
            "cx": float(c["CX"]), "cy": float(c["CY"])}


def T_imu_cam(path: str = RECON_NODE) -> np.ndarray:
    """그의 T_IC (IMU <- camera). 그의 노드는 R_WC = R_WI @ R_IC 로 쓴다.

    즉 그가 구독하는 pose 는 **IMU 의 pose** 이지 카메라의 pose 가 아니다. 우리가
    카메라 pose 를 그대로 발행하면 그의 노드가 T_IC 를 한 번 더 곱해서 틀어진다.
    :func:`camera_pose_to_imu_pose` 로 변환해서 보낼 것.
    """
    return np.asarray(read_constants(path)["T_IC"], dtype=float)


def camera_pose_to_imu_pose(R_WC: np.ndarray, p_WC: np.ndarray,
                            T_IC: np.ndarray | None = None):
    """카메라 pose (world<-cam) -> 그가 기대하는 IMU pose (world<-imu).

    T_WC = T_WI · T_IC  이므로  T_WI = T_WC · T_IC⁻¹.
    이렇게 보내야 그의 `R_WC = R_WI @ R_IC`, `c_W = p_WI + R_WI @ p_IC` 가
    우리가 실제로 렌더한 카메라 위치를 정확히 복원한다.
    """
    T_IC = T_imu_cam() if T_IC is None else T_IC
    R_IC, p_IC = T_IC[:3, :3], T_IC[:3, 3]
    R_WI = R_WC @ R_IC.T
    p_WI = np.asarray(p_WC, dtype=float) - R_WI @ p_IC
    return R_WI, p_WI


# --------------------------------------------------------------------------- #
# ROS 없이 그의 노드 코드를 그대로 실행하기 위한 최소 스텁
# --------------------------------------------------------------------------- #
class _Stamp:
    def __init__(self, t_ns: int):
        self.sec, self.nanosec = divmod(int(t_ns), 1_000_000_000)


class _Header:
    def __init__(self, t_ns: int):
        self.stamp = _Stamp(t_ns)


class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = float(x), float(y), float(z), float(w)


class PoseMsg:
    """`geometry_msgs/PoseWithCovarianceStamped` 와 같은 모양의 오리 타입."""
    def __init__(self, t_ns: int, p_WI, q_WI_xyzw):
        self.header = _Header(t_ns)
        inner = types.SimpleNamespace(position=_Vec(*p_WI),
                                      orientation=_Vec(*q_WI_xyzw))
        self.pose = types.SimpleNamespace(pose=inner)


class StringMsg:
    def __init__(self, data: str = ""):
        self.data = data


def _install_ros_stubs() -> None:
    """rclpy / geometry_msgs / std_msgs 를 최소 스텁으로 sys.modules 에 넣는다."""
    if "rclpy" in sys.modules:
        return

    class _Logger:
        def info(self, m): print(f"  [taemin] {m}")
        def warn(self, m): print(f"  [taemin][warn] {m}")
        def error(self, m): print(f"  [taemin][error] {m}")

    class _Node:
        def __init__(self, name): self._name = name
        def get_logger(self): return _Logger()
        def create_subscription(self, *a, **k): return None
        def create_publisher(self, *a, **k):
            outer = self

            class _Pub:
                def publish(self, msg):
                    outer.published.append(msg)
            outer.published = getattr(outer, "published", [])
            return _Pub()
        def create_timer(self, *a, **k): return None
        def destroy_node(self): pass

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.ok = lambda: False
    rclpy.spin = lambda *a, **k: None
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _Node
    rclpy.node = node_mod

    geo = types.ModuleType("geometry_msgs")
    geo_msg = types.ModuleType("geometry_msgs.msg")
    geo_msg.PoseWithCovarianceStamped = PoseMsg
    geo.msg = geo_msg

    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = StringMsg
    std.msg = std_msg

    sys.modules.update({
        "rclpy": rclpy, "rclpy.node": node_mod,
        "geometry_msgs": geo, "geometry_msgs.msg": geo_msg,
        "std_msgs": std, "std_msgs.msg": std_msg,
    })


def load_node_module(path: str = RECON_NODE):
    """태민의 `window_recon_node.py` 를 **수정 없이** import 해서 모듈을 돌려준다.

    ROS2 가 설치돼 있으면 진짜 rclpy 를 쓰고, 없으면 위 스텁을 끼운다. 어느 쪽이든
    `WindowReconNode` 클래스의 실제 코드(누적·삼각측량·시차각 판정·리포트)가
    그대로 실행된다.
    """
    try:
        import rclpy  # noqa: F401
    except ImportError:
        _install_ros_stubs()
    spec = importlib.util.spec_from_file_location("taemin_window_recon_node", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod
