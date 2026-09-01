"""energy.py — 사용 전력량 추정 / 실측 피드백 / 남은 전력 예산."""
from __future__ import annotations

import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import energy as E


M_1KG = 2.2726      # 짐 1 kg 포함 총질량 (qc_phys)
M_0KG = 1.2726


# ── 눈금: 모델이 물리적으로 말이 되는가 ──────────────────────────────────

def test_hover_power_is_in_real_multirotor_band():
    """1 kg 짐 호버가 소형 멀티로터 실측 대역(8~10 g/W) 안이어야 한다.

    이 눈금이 깨지면 상수(FM/eta/디스크 면적)가 틀어진 것이다. 08-26 에 이 값으로
    Simscape 배터리(25 g/W)와 qc_motor Cq(3.6 g/W)를 둘 다 기각했다.
    """
    p = E.DEFAULT_MODEL.hover_power(M_1KG)
    g_per_w = M_1KG * 1000.0 / p
    assert 7.5 < g_per_w < 11.0, "호버 %.0f W = %.1f g/W" % (p, g_per_w)


def test_lighter_craft_is_more_efficient():
    """가벼우면 g/W 가 좋아진다 (유도동력이 T^1.5 이므로)."""
    e1 = M_1KG * 1000.0 / E.DEFAULT_MODEL.hover_power(M_1KG)
    e0 = M_0KG * 1000.0 / E.DEFAULT_MODEL.hover_power(M_0KG)
    assert e0 > e1


def test_thrust_from_accel_hover_equals_weight():
    assert E.thrust_from_accel([0, 0, 0], M_1KG) == pytest.approx(M_1KG * E.G)


def test_thrust_grows_with_horizontal_accel():
    """기울면 추력이 커진다 — cos 보정을 따로 안 해도 되는 이유."""
    t0 = E.thrust_from_accel([0, 0, 0], M_1KG)
    t1 = E.thrust_from_accel([4.0, 0, 0], M_1KG)
    assert t1 > t0
    # tan(theta) = a/g 이므로 T = m*g/cos(theta) 와 같아야 한다
    theta = math.atan2(4.0, E.G)
    assert t1 == pytest.approx(M_1KG * E.G / math.cos(theta), rel=1e-9)


def test_zero_thrust_gives_zero_power():
    assert E.electrical_power(0.0) == 0.0
    assert E.electrical_power(-5.0) == 0.0


# ── 적분 ────────────────────────────────────────────────────────────────

def test_constant_hover_energy_matches_power_times_time():
    t = [i * 0.01 for i in range(1001)]          # 10 s
    acc = [[0.0, 0.0, 0.0]] * len(t)
    est = E.estimate_energy(t, acc, M_1KG)
    p = E.DEFAULT_MODEL.hover_power(M_1KG)
    assert est.wh == pytest.approx(p * 10.0 / 3600.0, rel=1e-9)
    assert est.p_mean_w == pytest.approx(p, rel=1e-9)
    assert est.p_peak_w == pytest.approx(p, rel=1e-9)
    assert est.duration_s == pytest.approx(10.0)


def test_distance_gives_wh_per_m():
    t = [0.0, 1.0, 2.0]
    acc = [[0, 0, 0]] * 3
    pos = [[0, 0, 1], [1, 0, 1], [2, 0, 1]]
    est = E.estimate_energy(t, acc, M_1KG, pos=pos)
    assert est.distance_m == pytest.approx(2.0)
    assert est.wh_per_m == pytest.approx(est.wh / 2.0)


def test_no_pos_leaves_wh_per_m_nan():
    est = E.estimate_energy([0.0, 1.0], [[0, 0, 0]] * 2, M_1KG)
    assert math.isnan(est.wh_per_m)


def test_mismatched_lengths_raise():
    with pytest.raises(ValueError):
        E.estimate_energy([0.0, 1.0], [[0, 0, 0]], M_1KG)


def test_trajectory_without_acc_uses_second_difference():
    """가속이 없는 궤적도 위치에서 되뽑는다. 등속이면 가속 0 -> 호버 전력."""
    t = [i * 0.05 for i in range(41)]
    traj = {"t": t, "pos": [[0.5 * x, 0.0, 1.0] for x in t]}
    est = E.estimate_energy_for_trajectory(traj, M_1KG)
    assert est.p_mean_w == pytest.approx(E.DEFAULT_MODEL.hover_power(M_1KG), rel=0.02)


# ── 피드백 교정 ─────────────────────────────────────────────────────────

def test_calibration_needs_enough_samples():
    m = E.DEFAULT_MODEL.calibrated_with(1.5, E.MIN_SAMPLES - 1)
    assert not m.calibrated
    assert m.eff_cal == 1.0


def test_calibration_step_is_ramped():
    """한 번에 CAL_STEP_MAX 이상 못 움직인다 — 튀는 측정 한 번에 끌려가지 않게."""
    m = E.DEFAULT_MODEL.calibrated_with(3.0, 10)
    assert m.calibrated
    assert m.eff_cal == pytest.approx(1.0 + E.CAL_STEP_MAX)


def test_calibration_ratio_direction():
    """실제로 더 썼으면(ratio>1) 추정 전력이 올라가야 한다."""
    base = E.DEFAULT_MODEL.hover_power(M_1KG)
    up = E.DEFAULT_MODEL.calibrated_with(1.1, 10).hover_power(M_1KG)
    down = E.DEFAULT_MODEL.calibrated_with(0.9, 10).hover_power(M_1KG)
    assert up > base > down


def test_calibration_rejects_nonsense():
    for bad in (0.0, -1.0, float("nan"), float("inf")):
        assert E.DEFAULT_MODEL.calibrated_with(bad, 10).eff_cal == 1.0


def test_calibration_clamped_to_bounds():
    m = E.DEFAULT_MODEL
    for _ in range(50):
        m = m.calibrated_with(10.0, 10)
    assert m.eff_cal <= E.CAL_MAX
    m2 = E.DEFAULT_MODEL
    for _ in range(50):
        m2 = m2.calibrated_with(0.01, 10)
    assert m2.eff_cal >= E.CAL_MIN


# ── 출처 신뢰도 (08-26: 시뮬 배터리로 교정하면 계획기가 3배 낙관한다) ─────

def test_sim_battery_is_not_trusted():
    assert not E.source_is_trusted("sim_battery")
    assert not E.source_is_trusted("simscape_worstcase")
    assert E.source_is_trusted("power_module")
    assert E.source_is_trusted("bms")


def test_untrusted_feedback_is_recorded_but_not_applied(tmp_path):
    p = str(tmp_path / E.FEEDBACK_NAME)
    E.write_feedback(p, 0.335, 9, "sim_battery")
    d = json.load(open(p, encoding="utf-8"))
    assert d["trusted"] is False
    m = E.load_feedback(p)
    assert not m.calibrated
    assert m.eff_cal == 1.0          # 교정 안 됨
    assert m.n_samples == 9          # 봤다는 사실은 남는다


def test_trusted_feedback_is_applied(tmp_path):
    p = str(tmp_path / E.FEEDBACK_NAME)
    E.write_feedback(p, 1.10, 9, "power_module")
    m = E.load_feedback(p)
    assert m.calibrated
    assert m.eff_cal == pytest.approx(1.10)


def test_missing_feedback_leaves_model_alone(tmp_path):
    m = E.load_feedback(str(tmp_path / "nope.json"))
    assert m == E.DEFAULT_MODEL


# ── 남은 전력 예산 ──────────────────────────────────────────────────────

def test_usable_applies_reserve():
    b = E.EnergyBudget(remaining_wh=100.0, reserve_frac=0.2)
    assert b.usable_wh == pytest.approx(80.0)


def test_untrusted_budget_doubles_reserve():
    """출처를 못 믿으면 예비를 더 잡는다 — 모자란 쪽으로 틀리는 게 안전하다."""
    b = E.EnergyBudget(remaining_wh=100.0, reserve_frac=0.2, trusted=False)
    assert b.usable_wh == pytest.approx(60.0)


def test_uncalibrated_estimate_gets_safety_factor():
    """미교정 추정치는 1.3배로 보고 판정한다."""
    t = [i * 0.1 for i in range(101)]            # 10 s 호버
    est = E.estimate_energy(t, [[0, 0, 0]] * len(t), M_1KG)
    assert not est.calibrated
    need = est.wh * 1.3
    b_tight = E.EnergyBudget(remaining_wh=need / 0.8 * 0.99, reserve_frac=0.2)
    b_ok = E.EnergyBudget(remaining_wh=need / 0.8 * 1.01, reserve_frac=0.2)
    assert not b_tight.can_afford(est)
    assert b_ok.can_afford(est)


def test_headroom_sign():
    t = [i * 0.1 for i in range(101)]
    est = E.estimate_energy(t, [[0, 0, 0]] * len(t), M_1KG)
    assert E.EnergyBudget(remaining_wh=1000.0).headroom_wh(est) > 0
    assert E.EnergyBudget(remaining_wh=0.01).headroom_wh(est) < 0


def test_energy_block_shape():
    t = [i * 0.1 for i in range(101)]
    est = E.estimate_energy(t, [[0, 0, 0]] * len(t), M_1KG,
                            pos=[[0, 0, 1]] * len(t))
    blk = E.energy_block(est, E.EnergyBudget(remaining_wh=50.0, source="bms"))
    assert set(blk) >= {"estimate", "budget", "affordable", "headroom_wh", "repeats_possible"}
    assert blk["estimate"]["calibrated"] is False
    assert blk["budget"]["source"] == "bms"
    assert isinstance(blk["repeats_possible"], int)


def test_energy_block_without_budget_has_no_verdict():
    est = E.estimate_energy([0.0, 1.0], [[0, 0, 0]] * 2, M_1KG)
    blk = E.energy_block(est)
    assert "affordable" not in blk


# ── 모터 관측 (교차검증용, 잔량 추정용 아님) ────────────────────────────

def test_power_from_motors_positive_and_grows():
    p1 = E.power_from_motors([400.0] * 4)
    p2 = E.power_from_motors([600.0] * 4)
    assert 0 < p1 < p2


def test_power_from_motors_scales_as_omega_cubed():
    """Q ~ w^2 이고 P = Q*w 이므로 P ~ w^3."""
    p1 = E.power_from_motors([300.0] * 4)
    p2 = E.power_from_motors([600.0] * 4)
    assert p2 / p1 == pytest.approx(8.0, rel=1e-9)


def test_motor_observation_disagrees_with_momentum_theory():
    """08-26 실측 기록: 두 모델이 호버에서 2배 이상 벌어진다.

    이건 버그가 아니라 **기록**이다 — qc_motor.hpp 의 Cq 가 프로펠러 공력이 아니라
    토크 클램프에 맞춰 역산된 값이라서 그렇다. 나중에 누가 Cq 를 실측으로 갈면
    이 시험이 깨지고, 그때 이 주석을 지우면 된다.
    """
    p_motor = E.power_from_motors([634.5] * 4)
    p_mom = E.DEFAULT_MODEL.hover_power(M_1KG)
    assert p_motor / p_mom > 2.0
