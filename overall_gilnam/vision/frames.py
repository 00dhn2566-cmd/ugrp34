"""프레임 변환 — spec §6.1 (window_detection_spec_v0.2.md) 카메라-IMU 외부 파라미터와 pose 해석.

표준값의 소유자는 태민 `visual_imaging_taemin/window_recon_node.py` 상수다 (단일 진실).
여기 값은 그 표를 "읽어 따른" 사본이며, 표가 바뀌면 여기와 tests/test_frames.py를 함께 갱신한다.

pose 블록 해석 규칙 (2026-10-21 잠정 확정 — 이의 있으면 회의에서 재론):
- pose["pose_frame"] == "body"  : position/orientation = T_world_body (IMU/body 프레임, §6.1 표준)
- pose["pose_frame"] == "camera": position/orientation = T_world_cam (OpenCV 광학 프레임)
- 필드 없음                     : "camera"로 간주 (2026-10-21 이전 sample_stream 레거시 — body≡camera 선언)
소비 코드는 pose_as_camera()로만 pose를 읽는다. rec["pose"]["orientation"]을 바로 카메라 회전으로 쓰지 말 것.
"""

import numpy as np

# cam→IMU 회전 (§6.1): 광학 z→body x, 광학 x→body −y, 광학 y→body −z
R_IC_SPEC = np.array([[0.0, 0.0, 1.0],
                      [-1.0, 0.0, 0.0],
                      [0.0, -1.0, 0.0]])
# 장착 오프셋 (시뮬 기본 0). 실기 Kalibr 값이 나오면 §6.1 표와 함께 교체.
t_IC_SPEC = np.zeros(3)

POSE_FRAMES = ("body", "camera")
LEGACY_POSE_FRAME = "camera"


def quat_xyzw_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat_xyzw(R):
    """회전행렬 → quaternion (x,y,z,w). Shepperd 방식 (최대 성분 기준, 수치 안정)."""
    R = np.asarray(R, dtype=float)
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2
        w, x, y, z = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2
        w, x, y, z = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2
        w, x, y, z = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    q = np.array([x, y, z, w])
    if w < 0:
        q = -q
    return q / np.linalg.norm(q)


def camera_pose_to_body_pose(position, quat_xyzw):
    """T_world_cam → T_world_body = T_WC · T_IC⁻¹. 반환 (position(3,), quat_xyzw(4,))."""
    R_wc = quat_xyzw_to_rot(quat_xyzw)
    t_wc = np.asarray(position, dtype=float)
    R_wb = R_wc @ R_IC_SPEC.T
    t_wb = t_wc - R_wb @ t_IC_SPEC
    return t_wb, rot_to_quat_xyzw(R_wb)


def body_pose_to_camera_pose(position, quat_xyzw):
    """T_world_body → T_world_cam = T_WB · T_IC. 반환 (position(3,), quat_xyzw(4,))."""
    R_wb = quat_xyzw_to_rot(quat_xyzw)
    t_wb = np.asarray(position, dtype=float)
    R_wc = R_wb @ R_IC_SPEC
    t_wc = t_wb + R_wb @ t_IC_SPEC
    return t_wc, rot_to_quat_xyzw(R_wc)


def pose_frame_of(pose):
    """pose 블록의 프레임 표기. 없으면 레거시(camera)."""
    frame = pose.get("pose_frame", LEGACY_POSE_FRAME)
    if frame not in POSE_FRAMES:
        raise ValueError(f"pose_frame must be one of {POSE_FRAMES}, got {frame!r}")
    return frame


def pose_as_camera(pose):
    """스트림 pose 블록 → (t_wc(3,), R_wc(3,3)) 카메라 pose. 프레임 표기에 따라 변환."""
    if pose_frame_of(pose) == "body":
        t, q = body_pose_to_camera_pose(pose["position"], pose["orientation"])
        return t, quat_xyzw_to_rot(q)
    return np.asarray(pose["position"], dtype=float), quat_xyzw_to_rot(pose["orientation"])
