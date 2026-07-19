"""A-2 금지 구역 테스트 (§9) — 이격 기하/게이트 연동/회피 재계획/CLI."""

import json
import os
import sys
from datetime import datetime, timedelta

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import traj_pipeline as tp                      # noqa: E402
from traj_shaping import (                      # noqa: E402
    KeepOutViolation,
    keep_out_avoid_waypoints,
    keep_out_check,
    keep_out_clearance,
)

SPHERE = {"shape": "sphere", "center": [3.0, 0.0, 2.0], "radius_m": 1.0}
BOX = {"shape": "box", "min": [2.0, -1.0, 1.0], "max": [4.0, 1.0, 3.0]}


# -- 이격 기하 ----------------------------------------------------------------

def test_sphere_clearance_outside_inside():
    c, _, _ = keep_out_clearance([[6.0, 0.0, 2.0]], [SPHERE], inflate_m=0.5)
    assert abs(c - 1.5) < 1e-9                  # 3 - 1(r) - 0.5(inflate)
    c, _, _ = keep_out_clearance([[3.0, 0.0, 2.0]], [SPHERE], inflate_m=0.5)
    assert abs(c - (-1.5)) < 1e-9               # 중심: -r - inflate


def test_box_clearance_faces_and_interior():
    c, _, _ = keep_out_clearance([[5.0, 0.0, 2.0]], [BOX], inflate_m=0.0)
    assert abs(c - 1.0) < 1e-9                  # x=4 면에서 1m
    c, _, _ = keep_out_clearance([[3.0, 0.0, 2.0]], [BOX], inflate_m=0.0)
    assert abs(c - (-1.0)) < 1e-9               # 내부 중심: 최근접 면 깊이 1m
    c, _, _ = keep_out_clearance([[4.5, 1.5, 2.0]], [BOX], inflate_m=0.0)
    assert abs(c - np.hypot(0.5, 0.5)) < 1e-9   # 모서리 대각 거리


def test_check_no_zones_passes():
    rep = keep_out_check(np.zeros((5, 3)), None)
    assert rep["violated"] is False
    rep = keep_out_check(np.zeros((5, 3)), {"zones": []})
    assert rep["violated"] is False


def test_check_violation_raises_with_stable_code():
    with pytest.raises(KeepOutViolation) as ex:
        keep_out_check([[3.0, 0.0, 2.0]], {"zones": [SPHERE]})
    assert ex.value.reject_code == "KEEP_OUT_VIOLATION"


# -- 파이프라인 게이트 연동 ---------------------------------------------------

def _mission(waypoints, keep_out=None):
    cfg = {"waypoints": waypoints,
           "limits": {"v_max": 1.0, "a_max": 0.8, "j_max": 2.0,
                      "snap_max": 30.0},
           "shaper": {"mode": "none"}}
    if keep_out:
        cfg["keep_out"] = keep_out
    return cfg


def test_build_trajectory_rejects_crossing():
    cfg = _mission([[0, 0, 2], [6, 0, 2]], {"zones": [SPHERE],
                                            "inflate_m": 0.5})
    cfg["controller_profile"] = "precision"
    with pytest.raises(KeepOutViolation):
        tp.build_trajectory(cfg, np.asarray(cfg["waypoints"], float), 1.8)


def test_build_trajectory_clear_zone_reports_clearance():
    cfg = _mission([[0, 0, 2], [6, 0, 2]],
                   {"zones": [{"shape": "sphere", "center": [3.0, 5.0, 2.0],
                               "radius_m": 1.0}], "inflate_m": 0.5})
    cfg["controller_profile"] = "precision"
    res = tp.build_trajectory(cfg, np.asarray(cfg["waypoints"], float), 1.8)
    rep = res["keep_out_report"]
    assert rep["violated"] is False
    assert rep["min_clearance_m"] > 3.0          # 5 - 1 - 0.5 - 경로폭 여유


def test_cli_plan_rejects_with_keep_out_code(tmp_path, capsys):
    m = _mission([[0, 0, 2], [6, 0, 2]], {"zones": [SPHERE],
                                          "inflate_m": 0.5})
    mp = tmp_path / "m.json"
    with open(mp, "w", encoding="utf-8") as f:
        json.dump(m, f)
    with pytest.raises(SystemExit) as ex:
        tp.main(["plan", "--input", str(mp), "--out-dir", str(tmp_path)])
    assert ex.value.code == tp.EXIT_REJECTED
    last = capsys.readouterr().out.strip().splitlines()[-1]
    assert json.loads(last)["reject_codes"][0]["code"] == "KEEP_OUT_VIOLATION"


# -- 회피 재계획 --------------------------------------------------------------

def _polyline_clear(wp, zones, inflate):
    """waypoint 폴리라인을 조밀 샘플해 최소 이격 반환 (검증용)."""
    pts = [wp[0]]
    for a, b in zip(wp[:-1], wp[1:]):
        n = max(int(np.ceil(np.linalg.norm(b - a) / 0.02)), 1)
        for k in range(1, n + 1):
            pts.append(a + (b - a) * k / n)
    c, _, _ = keep_out_clearance(np.asarray(pts), zones, inflate)
    return c


@pytest.mark.parametrize("zone", [SPHERE, BOX])
def test_avoid_reroutes_clear_of_zone(zone):
    ko = {"zones": [zone], "inflate_m": 0.5}
    wp = np.array([[0.0, 0.0, 2.0], [6.0, 0.0, 2.0]])
    new_wp, moved = keep_out_avoid_waypoints(wp, ko)
    assert moved is True
    assert np.allclose(new_wp[0], wp[0]) and np.allclose(new_wp[-1], wp[-1])
    assert _polyline_clear(new_wp, [zone], 0.5) >= 0.0   # 전 구간 비침범


def test_avoid_noop_when_clear():
    ko = {"zones": [SPHERE], "inflate_m": 0.5}
    wp = np.array([[0.0, 5.0, 2.0], [6.0, 5.0, 2.0]])
    new_wp, moved = keep_out_avoid_waypoints(wp, ko)
    assert moved is False
    assert np.allclose(new_wp, wp)


def test_avoid_unavoidable_endpoint_inside():
    ko = {"zones": [SPHERE], "inflate_m": 0.5}
    wp = np.array([[3.0, 0.0, 2.0], [6.0, 0.0, 2.0]])   # 시작점이 구역 안
    with pytest.raises(KeepOutViolation) as ex:
        keep_out_avoid_waypoints(wp, ko)
    assert getattr(ex.value, "unavoidable", False) is True


def test_avoided_route_survives_pipeline_gate():
    """회피 경로가 실제로 계획-성형-게이트-구역검사 전 체인을 통과 (검증 ②의
    파이썬 절반 — MATLAB 실비행 전 단계)."""
    ko = {"zones": [SPHERE], "inflate_m": 0.5}
    wp = np.array([[0.0, 0.0, 2.0], [6.0, 0.0, 2.0]])
    new_wp, _ = keep_out_avoid_waypoints(wp, ko)
    cfg = _mission(new_wp.tolist(), ko)
    cfg["controller_profile"] = "precision"
    res = tp.build_trajectory(cfg, new_wp, 1.8)   # 위반 시 여기서 raise
    assert res["keep_out_report"]["violated"] is False
    assert res["gate_ok"] is True


# -- emergency 동사 x keep_out (불가피 보고) ----------------------------------

def _fresh_state_file(tmp_path, vel):
    # +5s 미래 타임스탬프: 느린 머신/AV 스캔 지연으로 0.5s 신선도 초과하는
    # 간헐 실패 방지 (실측 1.49s 지연 사례)
    ts = (datetime.now() + timedelta(seconds=5.0))
    st = {"pos": [1.0, 0.0, 2.0], "vel": list(vel), "acc": [0, 0, 0],
          "att": {"roll_rad": 0, "pitch_rad": 0, "yaw_rad": 0},
          "timestamp": ts.strftime("%Y-%m-%dT%H-%M-%S.%f")[:-3],
          "ref_state": {"pos": [1.0, 0.0, 2.0], "vel": list(vel),
                        "acc": [0, 0, 0], "traj_hash": "x",
                        "t_on_traj_s": 0.0}}
    p = tmp_path / "current_state.json"
    with open(p, "w", encoding="utf-8") as f:
        json.dump(st, f)
    return str(p)


def test_cli_emergency_reports_unavoidable(tmp_path, capsys, monkeypatch):
    """정지 제동 경로가 구역 관통 - 거부 대신 KEEP_OUT_UNAVOIDABLE 보고 +
    원장 기록 (§9: 정지가 관통 회피보다 우선)."""
    monkeypatch.setattr(tp, "LEDGER_PATH", str(tmp_path / "ledger.jsonl"))
    state_p = _fresh_state_file(tmp_path, vel=(1.5, 0, 0))   # +x로 고속
    ko_p = tmp_path / "keep_out.json"
    with open(ko_p, "w", encoding="utf-8") as f:
        json.dump({"zones": [{"shape": "sphere", "center": [1.6, 0.0, 2.0],
                              "radius_m": 0.3}], "inflate_m": 0.2}, f)
    with pytest.raises(SystemExit) as ex:
        tp.main(["emergency", "--state", state_p, "--out-dir", str(tmp_path),
                 "--keep-out", str(ko_p)])
    assert ex.value.code == tp.EXIT_OK          # 정지는 거부되지 않는다
    last = capsys.readouterr().out.strip().splitlines()[-1]
    msg = json.loads(last)
    assert msg["keep_out"]["code"] == "KEEP_OUT_UNAVOIDABLE"
    with open(tmp_path / "ledger.jsonl", encoding="utf-8") as f:
        events = [json.loads(ln)["event"] for ln in f if ln.strip()]
    assert "keep_out_unavoidable" in events


def test_cli_emergency_clear_zone_no_flag(tmp_path, capsys):
    state_p = _fresh_state_file(tmp_path, vel=(1.0, 0, 0))
    ko_p = tmp_path / "keep_out.json"
    with open(ko_p, "w", encoding="utf-8") as f:
        json.dump({"zones": [{"shape": "sphere", "center": [0.0, 5.0, 2.0],
                              "radius_m": 0.5}], "inflate_m": 0.2}, f)
    with pytest.raises(SystemExit) as ex:
        tp.main(["emergency", "--state", state_p, "--out-dir", str(tmp_path),
                 "--keep-out", str(ko_p)])
    assert ex.value.code == tp.EXIT_OK
    last = capsys.readouterr().out.strip().splitlines()[-1]
    assert json.loads(last)["keep_out"]["violated"] is False
