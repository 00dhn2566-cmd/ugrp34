"""frames.py — spec §6.1 표준 T_IC와 pose_frame 해석 규칙 (2026-10-21 잠정 확정)."""
import json

import numpy as np

from frames import (R_IC_SPEC, body_pose_to_camera_pose, camera_pose_to_body_pose,
                    pose_as_camera, quat_xyzw_to_rot, rot_to_quat_xyzw)
from eval_recon3d import reconstruct_windows
from noisy_stream import load_records
from make_stream import generate_stream


def test_r_ic_matches_spec_table():
    # §6.1: 광학 z→body x, 광학 x→body −y, 광학 y→body −z
    assert np.allclose(R_IC_SPEC @ [0, 0, 1], [1, 0, 0])
    assert np.allclose(R_IC_SPEC @ [1, 0, 0], [0, -1, 0])
    assert np.allclose(R_IC_SPEC @ [0, 1, 0], [0, 0, -1])
    assert np.isclose(np.linalg.det(R_IC_SPEC), 1.0)


def test_quat_roundtrip_and_body_camera_roundtrip():
    rng = np.random.default_rng(7)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        R = quat_xyzw_to_rot(q)
        assert np.allclose(quat_xyzw_to_rot(rot_to_quat_xyzw(R)), R, atol=1e-12)
        t = rng.normal(size=3)
        tb, qb = camera_pose_to_body_pose(t, q)
        tc, qc = body_pose_to_camera_pose(tb, qb)
        assert np.allclose(tc, t, atol=1e-12)
        assert np.allclose(quat_xyzw_to_rot(qc), R, atol=1e-12)


def test_forward_camera_body_x_forward():
    # body가 world +X를 보는 항등 자세면 카메라 광축(+Z_cam)은 world +X를 향한다
    t, q = body_pose_to_camera_pose([0, 0, 1], [0, 0, 0, 1])
    R_wc = quat_xyzw_to_rot(q)
    assert np.allclose(R_wc[:, 2], [1, 0, 0])
    assert np.allclose(t, [0, 0, 1])  # 시뮬 장착 오프셋 0


def test_pose_as_camera_legacy_default_is_camera():
    pose = {"position": [1.0, 2.0, 3.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
    t, R = pose_as_camera(pose)
    assert np.allclose(t, [1, 2, 3]) and np.allclose(R, np.eye(3))
    t2, R2 = pose_as_camera({**pose, "pose_frame": "body"})
    assert np.allclose(R2, R_IC_SPEC)


def test_body_frame_stream_reconstructs_same_as_camera_frame(tmp_path):
    """같은 씬을 body/camera 프레임으로 기록해도 pose_as_camera를 거친 복원은 동일해야 한다."""
    results = {}
    for frame in ("body", "camera"):
        out = tmp_path / frame
        generate_stream(seed=42, out_dir=str(out), pose_frame=frame)
        records = load_records(out / "sample_stream.jsonl")
        assert all(rec["pose"]["pose_frame"] == frame for rec in records)
        scene_gt = json.loads((out / "scene_gt.json").read_text(encoding="utf-8"))
        results[frame] = reconstruct_windows(records, scene_gt)
    for oi in results["body"]:
        a, b = results["body"][oi]["corners_3d_est"], results["camera"][oi]["corners_3d_est"]
        assert a is not None and b is not None
        assert np.abs(np.asarray(a) - np.asarray(b)).max() < 1e-3  # quat 6자리 반올림 차이 < 1mm
