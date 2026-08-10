"""window_waypoint_planner 테스트 — 기하(접근측)·순서·여유 검사가 핵심."""
import numpy as np
import pytest

from window_waypoint_planner import gate_points, ordered_open_windows

# 접근측 normal = -X (synth_scene 관례와 동일한 예시 창문)
WIN = {
    "order_index": 0, "color": "red",
    "center": [5.0, 0.0, 1.5], "normal": [-1.0, 0.0, 0.0],
    "size_wh": [1.0, 1.2],
}


def test_gate_points_on_correct_sides():
    approach, exit_ = gate_points(WIN, d_app=1.5, d_exit=1.0, clearance_margin=0.35)
    center = np.array(WIN["center"])
    n = np.array(WIN["normal"])
    assert np.dot(approach - center, n) > 0        # 접근점은 접근측
    assert np.dot(exit_ - center, n) < 0           # 이탈점은 반대측
    np.testing.assert_allclose(np.linalg.norm(approach - center), 1.5)
    np.testing.assert_allclose(np.linalg.norm(exit_ - center), 1.0)


def test_gate_points_rejects_narrow_window():
    narrow = dict(WIN, size_wh=[0.6, 1.2])         # min/2 = 0.3 < margin 0.35
    with pytest.raises(ValueError, match="order_index=0"):
        gate_points(narrow, 1.5, 1.0, clearance_margin=0.35)


def test_gate_points_rejects_missing_normal():
    no_normal = {k: v for k, v in WIN.items() if k != "normal"}
    with pytest.raises(ValueError, match="normal"):
        gate_points(no_normal, 1.5, 1.0, 0.35)


def test_ordered_open_windows_filters_and_sorts():
    wmap = {"windows": [
        dict(WIN, order_index=2),
        dict(WIN, order_index=0, passed=True),     # 통과 완료 → 제외
        dict(WIN, order_index=1),                  # passed 부재 → false 취급
    ]}
    out = ordered_open_windows(wmap)
    assert [w["order_index"] for w in out] == [1, 2]
