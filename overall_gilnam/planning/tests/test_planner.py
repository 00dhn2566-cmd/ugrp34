"""window_waypoint_planner 테스트 — 기하(접근측)·순서·여유 검사가 핵심."""
import json
from pathlib import Path

import numpy as np
import pytest

from window_waypoint_planner import PLANNING_DIR, Plan, crossing_warnings, gate_points, load_planner_config, normal_from_corners, ordered_open_windows, plan_waypoints, plan_waypoints_v2, resolve_normal

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
    # 실 설정 파일에 v2 키(stop_ahead 등)가 있어 v2 경로 — 시작 + 창문 3개 × (접근·이탈) + 정지점
    assert len(wc.waypoints) == 1 + 2 * 3 + 1
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
    assert len(saved["waypoints"]) == 8 and "limits" in saved  # v2 경로 — +정지점


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


V2 = {"d_app": 1.5, "d_exit": 1.0, "clearance_margin": 0.35,
      "limits": {"v_max": 1.6, "a_max": 1.6, "j_max": 8.0, "snap_max": 40.0}, "dt": 0.01,
      "force_horizontal_normal": True, "gate_z": [0.5, 1.9],
      "stop_ahead": 0.6, "align_back": 0.45, "max_passes": 4, "shrink": [1.0, 0.75, 0.55, 0.4]}


def _win(i, x, y=0.0, n=(-1.0, 0.0, 0.0), wh=(1.0, 1.0)):
    return {"order_index": i, "color": ["red", "green", "blue"][i], "center": [x, y, 1.5],
            "normal": list(n), "size_wh": list(wh)}


def test_v2_labels_stop_point_and_ok():
    wmap = {"windows": [_win(0, 4.0), _win(1, 9.0), _win(2, 14.0)]}   # 간격 5m — 후진 없음
    p = plan_waypoints_v2(STATE, wmap, V2)
    assert isinstance(p, Plan) and p.ok and p.passes == 1 and p.shrink == 1.0
    assert p.labels == ["start", "approach0", "exit0", "approach1", "exit1", "approach2", "exit2", "stop"]
    np.testing.assert_allclose(p.waypoints[-1], [14.0 + 1.0 + 0.6, 0.0, 1.5])  # exit2 − stop_ahead·n


def test_v2_backtrack_detected_and_relaxed():
    wmap = {"windows": [_win(0, 4.0), _win(1, 5.5)]}    # 간격 1.5m < d_app+d_exit=2.5 → 후진
    p = plan_waypoints_v2(STATE, wmap, V2)
    assert p.passes > 1                                  # 재계획 발생
    assert p.shrink < 1.0
    # 완화 후 후진 없거나, 못 없앴으면 정직 보고
    assert p.ok or p.backtrack_m > 0


def test_v2_align_point_inserted():
    # 창문1이 창문0 이탈점보다 앞쪽(가까운 depth)이면서 옆으로 크게 비켜 있어
    # exit0→approach1 직선이 이미 창문1 평면을 넘어 개구부 밖에서 뚫음.
    # (교차 여부는 depth 겹침에만 좌우되므로 재계획 루프 영향을 배제하고자 단일 패스로 검증)
    wmap = {"windows": [_win(0, 4.0, 0.0), _win(1, 4.5, 3.0)]}
    p = plan_waypoints_v2(STATE, wmap, {**V2, "max_passes": 1})
    assert "align1" in p.labels
    i = p.labels.index("align1")
    assert p.labels[i + 1] == "approach1"
    # 정렬점 = approach1 + align_back·n̂
    ap = np.asarray(p.waypoints[i + 1]); al = np.asarray(p.waypoints[i])
    np.testing.assert_allclose(al - ap, np.array([-1.0, 0.0, 0.0]) * 0.45, atol=1e-9)


def test_plan_waypoints_v1_path_unchanged_without_v2_keys():
    cfg_v1 = {k: V2[k] for k in ("d_app", "d_exit", "clearance_margin", "limits", "dt")}
    wc = plan_waypoints(STATE, WMAP3, cfg_v1)
    assert len(wc.waypoints) == 7                          # v1과 동일: start + 2N, 정지점 없음


def test_plan_waypoints_wraps_v2_when_keys_present():
    wc = plan_waypoints(STATE, {"windows": [_win(0, 4.0), _win(1, 9.0), _win(2, 14.0)]}, V2)
    wc.validate()
    assert len(wc.waypoints) == 8                          # + stop


def test_v2_matches_yunho_wrapper_reference():
    # 윤호 prototype_demo/planner.py._build를 동일 입력·동일 상수(core cfg: d_app/d_exit/
    # clearance_margin)로 실행한 결과를 고정 (2026-08-18). 두 씬 모두 라벨·좌표 완전 일치 —
    # 구현 수정 불필요.
    wmap = {"windows": [_win(0, 4.0, 0.0), _win(1, 8.0, 3.0), _win(2, 13.0, 3.0)]}
    p = plan_waypoints_v2({"position": [0.0, 0.0, 1.0]}, wmap, V2)
    assert p.labels == ["start", "approach0", "exit0", "approach1", "exit1",
                         "approach2", "exit2", "stop"]
    np.testing.assert_allclose(p.waypoints, [
        [0.0, 0.0, 1.0],
        [2.5, 0.0, 1.5],
        [5.0, 0.0, 1.5],
        [6.5, 3.0, 1.5],
        [9.0, 3.0, 1.5],
        [11.5, 3.0, 1.5],
        [14.0, 3.0, 1.5],
        [14.6, 3.0, 1.5],
    ], atol=1e-6)
    assert p.ok and p.passes == 1 and p.shrink == 1.0

    # 2번째 참조 씬 — align+backtrack 유발 간격 (창문1이 창문0 이탈점 앞쪽·옆으로 크게 비켜
    # 있음). max_passes=4(윤호 기본 MAX_PASSES)로 재계획해도 경고 2건이 남아 정직하게 보고.
    wmap2 = {"windows": [_win(0, 4.0, 0.0), _win(1, 4.5, 3.0)]}
    p2 = plan_waypoints_v2({"position": [0.0, 0.0, 1.0]}, wmap2, V2)
    assert p2.labels == ["start", "approach0", "exit0", "approach1", "exit1", "stop"]
    np.testing.assert_allclose(p2.waypoints, [
        [0.0, 0.0, 1.0],
        [3.4, 0.0, 1.5],
        [4.4, 0.0, 1.5],
        [3.9, 3.0, 1.5],
        [4.9, 3.0, 1.5],
        [5.5, 3.0, 1.5],
    ], atol=1e-6)
    assert p2.passes == 4 and p2.shrink == pytest.approx(0.4)
    assert not p2.ok and p2.backtrack_m == pytest.approx(0.5) and len(p2.warnings) == 2
