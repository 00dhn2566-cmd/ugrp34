"""window_waypoint_planner 테스트 — 기하(접근측)·순서·여유 검사가 핵심."""
import json
from pathlib import Path

import numpy as np
import pytest

from window_waypoint_planner import PLANNING_DIR, crossing_warnings, gate_points, load_planner_config, normal_from_corners, ordered_open_windows, plan_waypoints, resolve_normal

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


def test_gate_points_rejects_zero_normal():
    # 영벡터 normal → NaN 전파 대신 즉시 거부 (최종 리뷰 반영)
    zero_n = dict(WIN, normal=[0.0, 0.0, 0.0])
    with pytest.raises(ValueError, match="영벡터"):
        gate_points(zero_n, 1.5, 1.0, 0.35)


def test_ordered_open_windows_filters_and_sorts():
    wmap = {"windows": [
        dict(WIN, order_index=2),
        dict(WIN, order_index=0, passed=True),     # 통과 완료 → 제외
        dict(WIN, order_index=1),                  # passed 부재 → false 취급
    ]}
    out = ordered_open_windows(wmap)
    assert [w["order_index"] for w in out] == [1, 2]


STATE = {"position": [0.0, 0.0, 1.5]}
WMAP3 = {"windows": [
    {"order_index": i, "color": c,
     "center": [4.0 + 5.0 * i, 0.3 * i, 1.5], "normal": [-1.0, 0.0, 0.0],
     "size_wh": [1.0, 1.0]}
    for i, c in enumerate(["red", "green", "blue"])
]}


def _cfg():
    return load_planner_config(PLANNING_DIR / "planner_limits.yaml")


def test_plan_waypoints_sequence_and_schema():
    wc = plan_waypoints(STATE, WMAP3, _cfg())
    wc.validate()                                   # 성진 스키마 통과 (윤호 validate 경유)
    assert len(wc.waypoints) == 1 + 2 * 3           # 시작 + 창문 3개 × (접근·이탈)
    assert wc.waypoints[0] == [0.0, 0.0, 1.5]       # 첫 점 = 드론 현재 위치
    # 접근(-X쪽) < center < 이탈: 창문 0의 x 좌표로 확인
    assert wc.waypoints[1][0] == pytest.approx(4.0 - 1.5)
    assert wc.waypoints[2][0] == pytest.approx(4.0 + 1.0)


def test_plan_waypoints_requires_at_least_one_window():
    with pytest.raises(ValueError, match="열린 창문"):
        plan_waypoints(STATE, {"windows": []}, _cfg())


def test_config_defaults_loaded():
    cfg = _cfg()
    assert cfg["d_app"] == 1.5 and cfg["d_exit"] == 1.0
    assert cfg["clearance_margin"] == 0.35
    assert cfg["limits"]["v_max"] == 1.6 and cfg["dt"] == 0.01  # 물리 한계의 80% (2026-08-11 잠정)


def test_cli_roundtrip(tmp_path):
    import subprocess, sys
    state_p, wmap_p, out_p = tmp_path / "s.json", tmp_path / "m.json", tmp_path / "wp.json"
    state_p.write_text(json.dumps(STATE), encoding="utf-8")
    wmap_p.write_text(json.dumps(WMAP3), encoding="utf-8")
    r = subprocess.run(
        [sys.executable, str(PLANNING_DIR / "window_waypoint_planner.py"),
         "--state", str(state_p), "--window-map", str(wmap_p), "--out", str(out_p)],
        capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    saved = json.loads(out_p.read_text(encoding="utf-8"))
    assert len(saved["waypoints"]) == 7 and "limits" in saved


def test_crossing_warning_outside_opening():
    # 구간이 창문 평면을 개구부 밖(y=2.0, 반폭 0.5-마진 바깥)에서 교차 → 경고 1건
    win = {"order_index": 0, "color": "red", "center": [5.0, 0.0, 1.5],
           "normal": [-1.0, 0.0, 0.0], "size_wh": [1.0, 1.0]}
    seg = [[0.0, 2.0, 1.5], [10.0, 2.0, 1.5]]      # x=5 평면을 y=2.0에서 관통
    warns = crossing_warnings(seg, [win], clearance_margin=0.35)
    assert len(warns) == 1 and warns[0]["order_index"] == 0


def test_no_warning_through_opening():
    win = {"order_index": 0, "color": "red", "center": [5.0, 0.0, 1.5],
           "normal": [-1.0, 0.0, 0.0], "size_wh": [1.0, 1.0]}
    seg = [[0.0, 0.0, 1.5], [10.0, 0.0, 1.5]]      # 정중앙 통과
    assert crossing_warnings(seg, [win], 0.35) == []


def _corners_for(center, n, w, h):
    # synth 관례(README_stream): viewer_right = cross(-n, UP), TL→TR→BR→BL (접근측 기준)
    center = np.asarray(center, dtype=float)
    n = np.asarray(n, dtype=float)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(-n, up)
    right = right / np.linalg.norm(right)
    return [
        (center + h / 2 * up - w / 2 * right).tolist(),
        (center + h / 2 * up + w / 2 * right).tolist(),
        (center - h / 2 * up + w / 2 * right).tolist(),
        (center - h / 2 * up - w / 2 * right).tolist(),
    ]


def test_normal_from_corners_points_to_approach_side():
    # 부호 확정 공식 검증 — yaw 있는 normal 포함
    for n in ([-1.0, 0.0, 0.0], [-0.9, -0.436, 0.0]):
        corners = _corners_for([5.0, 0.0, 1.5], n, 1.0, 1.2)
        derived = normal_from_corners(corners)
        n_unit = np.asarray(n) / np.linalg.norm(n)
        assert float(np.dot(derived, n_unit)) > 0.999


def test_normal_from_corners_rejects_degenerate():
    with pytest.raises(ValueError):
        normal_from_corners([[0, 0, 0]] * 4)          # 퇴화 (모든 점 동일)
    with pytest.raises(ValueError):
        normal_from_corners([[0, 0, 0], [1, 1, 1]])   # (4,3) 아님


def test_gate_points_fallback_to_corners():
    # normal 없이 corners_3d만 있어도 명시 normal과 동일한 게이트 점
    corners = _corners_for(WIN["center"], WIN["normal"], *WIN["size_wh"])
    win_c = {"order_index": 0, "color": "red", "center": WIN["center"],
             "size_wh": WIN["size_wh"], "corners_3d": corners}
    a1, e1 = gate_points(WIN, 1.5, 1.0, 0.35)
    a2, e2 = gate_points(win_c, 1.5, 1.0, 0.35)
    np.testing.assert_allclose(a1, a2, atol=1e-9)
    np.testing.assert_allclose(e1, e2, atol=1e-9)


def test_gate_points_requires_normal_or_corners():
    bare = {"order_index": 0, "color": "red", "center": [5.0, 0.0, 1.5], "size_wh": [1.0, 1.2]}
    with pytest.raises(ValueError, match="corners_3d"):
        gate_points(bare, 1.5, 1.0, 0.35)


def test_resolve_normal_forces_horizontal():
    tilted = {"order_index": 0, "color": "red", "center": [5, 0, 1.5], "size_wh": [1, 1],
              "normal": [-0.8, 0.0, -0.6]}                     # 35°쯤 기울어진 복원 법선
    n = resolve_normal(tilted, force_horizontal=True)
    np.testing.assert_allclose(n, [-1.0, 0.0, 0.0], atol=1e-9)
    n0 = resolve_normal(tilted, force_horizontal=False)
    assert abs(n0[2]) > 0.5                                    # 끄면 원본 유지


def test_resolve_normal_keeps_vertical_when_horizontal_part_vanishes():
    vertical = {"order_index": 0, "color": "red", "center": [5, 0, 1.5], "size_wh": [1, 1],
                "normal": [0.0, 0.0, -1.0]}
    n = resolve_normal(vertical, force_horizontal=True)
    np.testing.assert_allclose(n, [0.0, 0.0, -1.0])           # 판단 불가 → 원본


def test_gate_points_z_clamp():
    low = {"order_index": 0, "color": "red", "center": [5.0, 0.0, 0.3], "size_wh": [1, 1],
           "normal": [-1.0, 0.0, 0.0]}
    a, e = gate_points(low, 1.5, 1.0, 0.35, gate_z=(0.5, 1.9))
    assert a[2] == pytest.approx(0.5) and e[2] == pytest.approx(0.5)
    a2, _ = gate_points(low, 1.5, 1.0, 0.35)                   # 미지정이면 클램프 없음
    assert a2[2] == pytest.approx(0.3)


from window_waypoint_planner import assemble_window_map, format_warning


def test_crossing_warnings_structured():
    win = {"order_index": 0, "color": "red", "center": [5.0, 0.0, 1.5],
           "normal": [-1.0, 0.0, 0.0], "size_wh": [1.0, 1.0]}
    seg = [[0.0, 2.0, 1.5], [10.0, 2.0, 1.5]]
    warns = crossing_warnings(seg, [win], 0.35)
    assert len(warns) == 1
    w = warns[0]
    assert w["order_index"] == 0 and w["seg_index"] == 0
    # width_axis = cross(UP, n) — n=(-1,0,0) → width_axis=(0,-1,0), 실제 부호는 -2.0 (기존 기하 불변)
    assert w["u"] == pytest.approx(-2.0, abs=1e-6) and abs(w["v"]) < 1e-9
    assert w["half_w"] == pytest.approx(0.15) and w["half_h"] == pytest.approx(0.15)
    s = format_warning(w)
    assert "order_index=0" in s and "개구부 밖" in s


def test_assemble_window_map_moved_to_planning():
    recon = {
        1: {"color": "green", "n_pairs": 5, "corners_3d_est": np.array(
            [[4.0, 0.6, 2.0], [4.0, -0.6, 2.0], [4.0, -0.6, 1.0], [4.0, 0.6, 1.0]])},
        0: {"color": "red", "corners_3d_est": None, "n_pairs": 0},
    }
    wmap, failed = assemble_window_map(recon)
    assert failed == [0] and [w["order_index"] for w in wmap["windows"]] == [1]
    assert np.dot(wmap["windows"][0]["normal"], [-1.0, 0.0, 0.0]) > 0.999  # 부호 확정 공식
