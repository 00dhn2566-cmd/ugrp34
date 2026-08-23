"""traj_bridge — 재계획 인터벌 다리 궤적 시험.

지키려는 성질 (이게 깨지면 다리가 위험해진다)
  1. 물리 한계를 절대 안 넘는다 — 넘으면 기체가 못 따라가 다리가 무의미
  2. 위치가 연속이고 옛 기준 위에 있다 — 금지구역 재검사를 생략하는 근거
  3. 계획이 안 오면 정지로 끝난다 — 무한정 날아가지 않는다
  4. 해석적 연쇄법칙 == 수치미분 (매끄러운 구간)
  5. 인계 한계(planner_limits)는 인계 상태를 담는다 — 안 담으면 새 계획이 못 만들어진다
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from traj_bridge import (  # noqa: E402
    BASE_LIMITS,
    PHYS,
    _chain_derivs,
    _derivs,
    _smoothstep7,
    plan_bridge,
    scale_for_limits,
)

DT = 0.01


def make_ref(dx=6.0, v0=1.6, tail=8.0, dt=DT):
    """7차 스무드스텝 위치 프로파일 — 4차 도함수까지 유계라 스플라인 링잉이 없다."""
    tmove = 2.1875 * dx / v0          # 스무드스텝 피크속도 = 2.1875·dx/T
    t = np.arange(0.0, tmove + tail + dt, dt)
    xs = dx * _smoothstep7(t / tmove)
    return t, np.column_stack([xs, np.zeros_like(t), np.ones_like(t)]), tmove


def derate(frac, blim=None):
    b = blim or BASE_LIMITS
    return {k: b[k] * frac ** p for k, p in (("v", 1), ("a", 2), ("j", 3), ("snap", 4))}


@pytest.mark.parametrize("frac", [0.75, 0.50, 0.30, 0.18])
@pytest.mark.parametrize("blim,label", [(BASE_LIMITS, "1kg"),
                                        (dict(v=1.2, a=1.0, j=8.0, snap=64.0), "0kg")])
def test_never_exceeds_physical(frac, blim, label):
    """1) 어떤 감쇄폭에서도 물리 한계를 넘지 않는다."""
    t, base, tm = make_ref(v0=blim["v"])
    br = plan_bridge(t, base, t_now=tm / 2, limits_new=derate(frac, blim),
                     replan_budget_s=0.25, base_limits=blim)
    worst = max(br.phys_use.values())
    assert worst <= 1.0, f"{label} frac={frac}: 물리 한계 {worst:.2f}배 초과 {br.phys_use}"


def test_position_continuous_and_on_path():
    """2) 다리는 옛 기준 위를 지난다 (기하 불변) — 금지구역 재검사 생략의 근거."""
    t, base, tm = make_ref()
    t_now = round(tm / 2 / DT) * DT       # 격자에 맞춰야 시작점 비교가 의미 있다
    br = plan_bridge(t, base, t_now=t_now, limits_new=derate(0.5), replan_budget_s=0.25)
    # 시작점 일치
    i0 = int(round(t_now / DT))
    assert np.allclose(br.pos[0], base[i0], atol=2e-3)
    # 모든 점이 옛 기준 곡선 위 (y=0, z=1 유지)
    assert np.max(np.abs(br.pos[:, 1])) < 1e-9
    assert np.max(np.abs(br.pos[:, 2] - 1.0)) < 1e-9
    # x 는 단조 증가 (뒤로 가지 않는다 — 스냅백 금지)
    assert np.all(np.diff(br.pos[:, 0]) >= -1e-9)
    # 위치 연속 (한 스텝 점프가 v_max·dt 의 2배 이내)
    assert np.max(np.abs(np.diff(br.pos[:, 0]))) < 2 * PHYS["v"] * DT


def test_failsafe_ends_stopped():
    """3) 계획이 안 오면 정지로 끝난다."""
    t, base, tm = make_ref()
    br = plan_bridge(t, base, t_now=tm / 2, limits_new=derate(0.5), replan_budget_s=0.25)
    assert br.stopped, "비상 갈래가 정지로 끝나지 않음"
    assert br.s_of_t[-1] == pytest.approx(0.0, abs=1e-9)
    tail = br.pos[br.t >= br.failsafe_from_s + max(br.t_ramp, 0.5)]
    assert np.max(tail[:, 0]) - np.min(tail[:, 0]) < 1e-3, "정지 후에도 움직인다"


def test_chain_rule_matches_numeric():
    """4) 해석적 연쇄법칙이 수치미분과 일치 (s=1, 매끄러운 구간)."""
    from scipy.interpolate import make_interp_spline
    t, base, _ = make_ref()
    spl = make_interp_spline(t, base, k=5)
    n = len(t)
    ana = _chain_derivs(spl, t, np.ones(n), np.zeros(n), np.zeros(n), np.zeros(n))
    num = _derivs(t, base)
    # 이동 구간 내부에서만 비교한다. 이동이 끝나는 지점은 스무드스텝의 4차 도함수가
    # 계단으로 끊기는 곳이라, 스플라인(링잉)과 중심차분(평활) 둘 다 그 자리에서 다른
    # 근사를 낸다 — 어느 쪽도 '정답'이 아니라 대조 대상이 못 된다.
    _, _, tmove = make_ref()
    m = (t > 0.5) & (t < tmove - 0.5)
    for k, (a, b) in zip(("v", "a", "j", "snap"), zip(ana, num)):
        scale = max(np.max(np.abs(b[m])), 1e-9)
        rel = np.max(np.abs(a[m] - b[m])) / scale
        assert rel < 5e-3, f"{k} 상대오차 {rel:.2e}"


def test_planner_limits_admit_handoff():
    """5) 인계 한계는 인계 상태를 담는다.

    계획기는 v(0)=v0 를 고정 초기조건으로 받는다. v0 > v_max 인 한계로는 **어떤 T
    로도** 실행 가능한 세그먼트를 못 만들어 min-time 탐색이 상한까지 밀린다.
    """
    t, base, tm = make_ref()
    for frac in (0.75, 0.5, 0.3):
        br = plan_bridge(t, base, t_now=tm / 2, limits_new=derate(frac),
                         replan_budget_s=0.25)
        p, v, a, j = br.handoff
        for key, val in (("v", v), ("a", a), ("j", j)):
            mag = float(np.linalg.norm(val))
            assert mag <= br.planner_limits[key] + 1e-9, (
                f"frac={frac}: 인계 {key} {mag:.3f} > planner_limits {br.planner_limits[key]:.3f}")
        # 물리 한계는 넘지 않는다
        for key in br.planner_limits:
            assert br.planner_limits[key] <= PHYS[key] + 1e-9


def test_deeper_derate_needs_longer_lead():
    """감쇄가 깊을수록 새 한계 안으로 들어가는 데 오래 걸린다 (= 선행 경보가 더 필요)."""
    t, base, tm = make_ref()
    lead = [plan_bridge(t, base, t_now=tm / 2, limits_new=derate(f),
                        replan_budget_s=0.25).compliant_after_s
            for f in (0.75, 0.50, 0.30)]
    assert lead == sorted(lead), f"단조 증가하지 않음: {lead}"
    assert lead[0] > 0.25, "얕은 감쇄조차 예산 안에 못 들어오는지 확인 (설계 전제)"


def test_identity_when_no_derate():
    """감쇄가 없으면(새 한계 = 기저) 다리는 사실상 옛 기준 그대로."""
    t, base, tm = make_ref()
    t_now = tm / 2
    br = plan_bridge(t, base, t_now=t_now, limits_new=dict(BASE_LIMITS),
                     replan_budget_s=0.25)
    # S_AIM 여유(3%)만큼만 느려진다
    assert 0.9 < br.s_target <= 1.0
    n = int(round(0.25 / DT))
    i0 = int(round(t_now / DT))
    assert np.max(np.abs(br.pos[:n, 0] - base[i0:i0 + n, 0])) < 0.02


def test_scale_for_limits_takes_tightest():
    """s 환산은 v/a/j/snap 중 **가장 빡빡한 것**을 따른다."""
    assert scale_for_limits(derate(0.5)) == pytest.approx(0.5, abs=1e-9)
    lim = dict(BASE_LIMITS)
    lim["j"] = BASE_LIMITS["j"] * 0.2 ** 3        # j 만 크게 깎음
    assert scale_for_limits(lim) == pytest.approx(0.2, abs=1e-9)


def test_rejects_bad_input():
    t, base, tm = make_ref()
    with pytest.raises(ValueError):
        plan_bridge(t, base[:, :2], t_now=1.0, limits_new=derate(0.5), replan_budget_s=0.25)
    with pytest.raises(ValueError):
        plan_bridge(t, base, t_now=t[-1] + 1.0, limits_new=derate(0.5), replan_budget_s=0.25)
