# 합성 씬 생성기 + 샘플 스트림 테스트
# 핵심 보험: 삼각측량 왕복(테스트 1)이 통과하면 관례(투영·pose·corner 순서)가
# 자기일관적임이 증명된다 — 태민이 같은 식으로 복원 가능.
import json
from pathlib import Path

import cv2
import numpy as np

from color_judge import load_color_config
from gt_stream import labels_to_message
from make_stream import generate_stream
from synth_scene import load_intrinsics, make_scene, make_trajectory, project, to_label_lines

VISION_DIR = Path(__file__).resolve().parents[1]
CONFIG = load_color_config(VISION_DIR / "color_order.yaml")
INTR = load_intrinsics(VISION_DIR / "synth_intrinsics.yaml")


def _fully_visible_frames(scene, traj, K, order_index):
    """(pose, projection) 목록 — 해당 창문의 4 corner가 모두 vis=1인 프레임만."""
    out = []
    for pose in traj:
        for w in project(scene, pose, K):
            if w["order_index"] == order_index and all(v == 1 for v in w["vis"]):
                out.append((pose, w))
    return out


def _spec5_corners(proj_window, timestamp, frame_id):
    """생성기 → §4.3 라벨 → 기존 gt_stream 어댑터 → §5 corners (실전 경로 그대로)."""
    lines = to_label_lines([proj_window])
    msg = labels_to_message(lines, timestamp, frame_id, CONFIG)
    return np.array(msg["windows"][0]["corners"], dtype=np.float64)


def _projection_matrix(pose, K):
    # P = K [R_wc^T | -R_wc^T t_wc]  (README_stream.md 계약과 동일식)
    R_t = pose["R_wc"].T
    return K @ np.hstack([R_t, (-R_t @ pose["position"]).reshape(3, 1)])


def test_triangulation_round_trip():
    scene = make_scene(seed=7)
    traj = make_trajectory(scene)
    K = INTR["K"]
    frames = _fully_visible_frames(scene, traj, K, order_index=0)
    assert len(frames) >= 2, "창문0이 완전 가시인 프레임이 2개 이상이어야 함"
    (pose_a, w_a), (pose_b, w_b) = frames[0], frames[-1]  # 시차 최대 쌍
    assert np.linalg.norm(pose_a["position"] - pose_b["position"]) > 0.5, "삼각측량 시차 부족"

    pts_a = _spec5_corners(w_a, pose_a["timestamp"], 0)
    pts_b = _spec5_corners(w_b, pose_b["timestamp"], 1)
    X_h = cv2.triangulatePoints(
        _projection_matrix(pose_a, K), _projection_matrix(pose_b, K), pts_a.T, pts_b.T
    )
    X = (X_h[:3] / X_h[3]).T
    gt = np.array(scene["windows"][0]["corners_3d"], dtype=np.float64)
    assert np.max(np.abs(X - gt)) < 1e-3  # mm 미만 — 생성기 자기일관성


def test_winding_frontal_approach():
    # 정면 접근 첫 프레임: 화면상 좌상(TL)이 실제 좌상 — u_TL < u_TR, v_TL < v_BL
    scene = make_scene(seed=7)
    traj = make_trajectory(scene)
    frames = _fully_visible_frames(scene, traj, INTR["K"], order_index=0)
    corners = frames[0][1]["corners_px"]  # 순서: TL, TR, BR, BL
    tl, tr, _, bl = corners
    assert tl[0] < tr[0]
    assert tl[1] < bl[1]


def test_label_line_format():
    scene = make_scene(seed=7)
    traj = make_trajectory(scene)
    lines = to_label_lines(project(scene, traj[0], INTR["K"]))
    assert lines, "첫 프레임에 라벨이 최소 1개 있어야 함"
    for line in lines:
        fields = line.split()
        assert len(fields) == 17  # §4.3: class + bbox(4) + (u,v,vis)*4
        assert int(fields[0]) in {0, 1, 2}
        assert all(isinstance(float(f), float) for f in fields)


def test_jsonl_schema(tmp_path):
    generate_stream(seed=42, out_dir=tmp_path)
    lines = (tmp_path / "sample_stream.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(lines) >= 200
    for line in lines:
        rec = json.loads(line)
        assert set(rec) == {"vision", "pose"}
        assert rec["vision"]["timestamp"] == rec["pose"]["timestamp"]
        assert isinstance(rec["pose"]["timestamp"], int)
        q = np.array(rec["pose"]["orientation"], dtype=np.float64)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-3
