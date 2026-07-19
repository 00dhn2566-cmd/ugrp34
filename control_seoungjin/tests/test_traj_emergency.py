"""traj_emergency.py 단위테스트 — A-1 비상 정지 (§9 비상 레짐)."""

import json
import os
import sys
from datetime import datetime, timedelta

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import traj_emergency as te                     # noqa: E402
import traj_pipeline as tp                      # noqa: E402
from traj_shaping import _stop_dist, traj_gate  # noqa: E402

DT = 0.01


def _state(pos=(0, 0, 2.0), vel=(0, 0, 0), acc=(0, 0, 0), yaw=0.3):
    return {"pos": list(pos), "vel": list(vel), "acc": list(acc),
            "att": {"roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": yaw}}


def _diffs(t, pos):
    dv = np.diff(pos, axis=0) / DT
    da = np.diff(dv, axis=0) / DT
    dj = np.diff(da, axis=0) / DT
    return dv, da, dj


# -- 정지 물리 ---------------------------------------------------------------

def test_high_speed_stop_gate_and_geometry():
    """고속(1.5m/s) 정지: 게이트 통과 + 후퇴 없음(스냅백 금지) + 정지.

    이산 착지 잔차로 종단에 mm급 미세 왕복은 허용 (합격선 10cm 대비 100배
    여유) — 스냅백 금지가 막는 것은 구 setpoint로의 복귀 기동이지 mm 잔차가
    아님. 단 실질 후퇴(>5mm)는 불합격.
    """
    res = te.build_emergency_stop(_state(vel=(1.5, 0, 0)), dt=DT)
    t, pos = res["t"], res["shaped"]
    assert res["gate_ok"] is True
    x = pos[:, 0]
    back = np.diff(x)
    assert -np.sum(back[back < 0]) < 5e-3       # 누적 후퇴 < 5mm (스냅백 금지)
    assert x.max() - x[-1] < 5e-3               # 정지점 지나침 후 복귀 < 5mm
    dv, _, _ = _diffs(t, pos)
    assert abs(dv[0, 0] - 1.5) < 0.05           # 첫 샘플 속도 = 실측 v0 (연속 승계)
    assert np.all(np.abs(dv[-10:]) < 1e-9)      # 종단 완전 정지 (래치 호버 기준)


def test_stop_distance_near_exact_formula():
    """정지 거리가 2단 정확식과 같은 자릿수 (소프트 저크 0.9x만큼 약간 김)."""
    v0 = 1.5
    res = te.build_emergency_stop(_state(vel=(v0, 0, 0)), dt=DT)
    ds = res["emergency"]["stop_dist_m"]
    ds_ref = _stop_dist(v0, 0.0, te.BRAKE_SHARE * tp.PHYS_AMAX, tp.PHYS_JMAX)
    assert 0.85 * ds_ref < ds < 1.35 * ds_ref   # 실측: 0.819 vs 정확식 0.821


def test_diagonal_stop_xy_share_and_norm_gate():
    """대각 기동 정지: xy x0.7 축배분 적용 + 노름 게이트 통과."""
    res = te.build_emergency_stop(_state(vel=(1.2, 1.2, 0)), dt=DT)
    assert res["smoother_info"]["xy_share_applied"] == te.XY_SHARE
    assert res["gate_ok"] is True


def test_stop_with_initial_accel():
    """가속 중(a0>0) 정지 명령: 저크 한계 내에서 a 스윙 후 정지."""
    res = te.build_emergency_stop(
        _state(vel=(1.0, 0, 0), acc=(1.5, 0, 0)), dt=DT)
    assert res["gate_ok"] is True
    dv, _, _ = _diffs(res["t"], res["shaped"])
    assert np.all(np.abs(dv[-10:]) < 1e-9)


def test_descent_stop_z_axis():
    """하강 중 정지 (z축 전한계): 정지 + z 단조 하강."""
    res = te.build_emergency_stop(_state(vel=(0, 0, -1.0)), dt=DT)
    z = res["shaped"][:, 2]
    up = np.diff(z)
    assert np.sum(up[up > 0]) < 5e-3            # 재상승 누적 < 5mm (미세 잔차만)
    assert res["gate_ok"] is True


def test_already_at_rest_is_pure_hold():
    """정지 상태에서 정지 명령: 현재 자리 hold만 (래치 호버와 동일)."""
    res = te.build_emergency_stop(_state(pos=(1, 2, 3)), dt=DT, hold_s=1.0)
    assert res["emergency"]["stop_dist_m"] < 1e-9
    assert np.allclose(res["shaped"], [1, 2, 3])
    assert abs(res["t"][-1] - 1.0) < 2 * DT


# -- 비상 레짐 규칙 -----------------------------------------------------------

def test_emergency_regime_no_shaper():
    res = te.build_emergency_stop(_state(vel=(1.0, 0, 0)), dt=DT)
    assert res["shaper_mode"] == "none"          # ZVD 생략
    assert np.all(res["delta"] == 0.0)           # 상쇄 레이어 없음
    assert res["limits_effective"]["v_max"] == tp.PHYS_VMAX   # 마진 반납


def test_yaw_frozen_at_measured():
    res = te.build_emergency_stop(_state(vel=(1.0, 0, 0), yaw=0.77), dt=DT)
    assert np.all(res["yaw"] == 0.77)


def test_missing_vel_dies():
    with pytest.raises(KeyError):
        te.build_emergency_stop({"pos": [0, 0, 1]})


# -- 축별 정지 프로파일 (내부) ------------------------------------------------

@pytest.mark.parametrize("v0,a0", [(2.0, 0.0), (-2.0, 0.0), (0.5, -1.6),
                                   (0.0, 1.0), (0.05, 0.0), (1.0, 2.0)])
def test_axis_stop_limits_all_regimes(v0, a0):
    """어떤 초기 상태든 v/a/j 한계 내에서 (0,0) 도달."""
    ab, jmax = 1.6, 10.0
    inc = te._axis_stop(v0, a0, ab, jmax, DT)
    p = np.concatenate([[0.0], np.cumsum(inc), ])
    p = np.concatenate([p, np.tile(p[-1], 50)])  # hold 부착 후 미분 검사
    t = np.arange(len(p)) * DT
    ok, rep = traj_gate(t, np.column_stack([p, 0 * p, 0 * p]),
                        2.05, 2.05, do_error=False, jmax=10.0)
    assert ok, rep
    v_end = (p[-1] - p[-2]) / DT
    assert abs(v_end) < 1e-9


# -- CLI (§8 emergency 동사) --------------------------------------------------

def _fresh_state_file(tmp_path, **kw):
    st = _state(**kw)
    # +5s 미래 타임스탬프: AV 스캔/느린 머신의 0.5s 신선도 간헐 초과 방지
    st["timestamp"] = (datetime.now() + timedelta(seconds=5.0)) \
        .strftime("%Y-%m-%dT%H-%M-%S.%f")[:-3]
    st["ref_state"] = {"pos": st["pos"], "vel": st["vel"], "acc": st["acc"],
                       "traj_hash": "x", "t_on_traj_s": 0.0}
    p = tmp_path / "current_state.json"
    with open(p, "w", encoding="utf-8") as f:
        json.dump(st, f)
    return str(p)


def test_cli_emergency_ok(tmp_path, capsys):
    state_p = _fresh_state_file(tmp_path, vel=(1.2, 0, 0))
    with pytest.raises(SystemExit) as ex:
        tp.main(["emergency", "--state", state_p,
                 "--out-dir", str(tmp_path)])
    assert ex.value.code == tp.EXIT_OK
    last = capsys.readouterr().out.strip().splitlines()[-1]
    msg = json.loads(last)
    assert msg["verdict"] == "accepted"
    assert msg["emergency"]["type"] == "stop"
    assert os.path.isfile(tmp_path / "trajectory.mat")
    with open(tmp_path / "trajectory.json", encoding="utf-8") as f:
        tj = json.load(f)
    assert tj["trajectory_hash"] == msg["trajectory_hash"]


def test_cli_emergency_stale_state_rejected(tmp_path, capsys):
    state_p = _fresh_state_file(tmp_path)
    with open(state_p, encoding="utf-8") as f:
        st = json.load(f)
    st["timestamp"] = "2026-07-19T00-00-00.000"   # 낡음
    with open(state_p, "w", encoding="utf-8") as f:
        json.dump(st, f)
    with pytest.raises(SystemExit) as ex:
        tp.main(["emergency", "--state", state_p,
                 "--out-dir", str(tmp_path)])
    assert ex.value.code == tp.EXIT_REJECTED
    last = capsys.readouterr().out.strip().splitlines()[-1]
    assert json.loads(last)["reject_codes"][0]["code"] == "STATE_STALE"
