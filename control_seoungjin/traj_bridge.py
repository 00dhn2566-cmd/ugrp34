"""재계획 인터벌을 잇는 다리 궤적 — 스펙을 깎기로 한 순간부터 새 계획이 올 때까지.

2026-08-23 신설. 사용자 요구:
    "중간 time_path 실행하는 동안 인터벌로 이을 계획도 같이 작성하고."

## 왜 필요한가 — `replan_splice` 가 못 메우는 구멍

`traj_pipeline.replan_splice(res1, tau_s, ...)` 는 **τ 시점까지는 옛 궤적 res1 이
그대로 유효하다**고 가정한다. waypoint 만 바뀌는 경우엔 맞다. 그런데 지연/외란
때문에 **한계(limits) 자체를 깎는** 경우엔 틀린다 — 깎기로 결정한 그 순간부터
옛 궤적은 이미 새 한계를 넘는 물건이고, 계획기가 도는 τ_plan 초 동안 드론은
그 위반 궤적을 계속 따라간다. 스펙을 깎은 이유가 바로 그걸 막으려던 것인데.

## 다리의 구성 — 기하는 그대로, 시계만 늦춘다

경로 기하를 새로 만들지 않는다. 가상 시계 배율 s 하나만 내린다:

    dτ/dt = s(t),     p(t) = p_ref(τ(t))

이유 세 가지
  1. **즉시 계산된다** — 스플라인 평가뿐이라 계획기(세그먼트당 5.3 ms)보다
     훨씬 빠르다. 다리가 계획보다 느리면 다리의 존재 의미가 없다.
  2. **금지구역 재검사가 필요 없다** — 지나는 점의 집합이 옛 궤적과 동일하다.
     기하를 새로 만들면 keep-out 을 다시 다 봐야 하고, 그게 또 부하다.
  3. **상위가 규칙을 하나만 외운다** — `capability.degraded.time_scale` 과 같은
     대수 (v∝s, a∝s², j∝s³, snap∝s⁴). MATLAB `qc_clock_gov_apply.m`,
     C++ `SpeedGovernor` 와 동일 물건의 계획측 짝이다.

s(t) 는 **7차 스무드스텝**으로 내린다 — 양끝에서 ṡ=s̈=s⃛=0 이라 이음매에서
저크·스냅이 튀지 않는다. 5차로 하면 s⃛ 이 불연속이라 스냅 스파이크가 남는다.

## 정직하게 적어 두는 한계

램프 초반에는 **새 한계를 넘는다.** 순항 중 v=1.6 m/s 로 날고 있는데 새 한계가
0.8 m/s 라면, 그 자리에서 0.8 로 순간이동할 방법은 없다 (그러려면 무한 가속).
그래서 이 모듈은 통과/실패를 내지 않고 **얼마나 넘고 언제 들어오는지**
(`excess_ratio`, `compliant_after_s`) 를 낸다.

여기서 나오는 결론이 설계로 되먹임된다 — **스펙 감쇄는 선행이어야 한다.**
지연을 재고 나서 깎으면 이미 램프 길이만큼 늦다. 그래서 `compute_load` 의
예측(모델) 경로가 실측 경로와 함께 있는 것이다.

## 계획이 끝내 안 오면 (비상 갈래)

`replan_budget_s` 를 지나도 새 계획이 없으면 s → 0 으로 마저 내려 **경로 위에서**
정지한다. 옆으로 새지 않으므로 금지구역 판정이 그대로 유효하다. 정지 후에는
그 점을 래치한다 (`traj_emergency` 의 A-1 과 같은 종점 규약).


## 플랜트 검증 결과 — **외란 강건성 수단이 아니다** (2026-08-23 실측)

`diagnose/verify_bridge_sim.m`, 위치 지연 40 ms, 감쇄(배율 0.53) 직후 0.3 N·m 펄스:

| 구성 | 종단[cm] | 오버[cm] | 외란 횡이탈[cm] | 복귀[s] |
|---|---|---|---|---|
| A 전속 유지 | 0.18 | 6.66 | **4.03** | 9.93 |
| B 다리 | 0.14 | **1.87** | 5.45 | 9.92 |

**가설이 틀렸다.** "다리로 갈아타면 외란에 더 강해진다" 를 기대했는데 횡이탈이
오히려 +35% 나빴다 (복귀는 동률). 이유: 외란은 펄스가 들어온 **그 순간의 자세 권한**이
결정하는데, 다리는 그때 아직 램프 중이라(새 한계 진입까지 1.43 s) 감쇄가 안 붙었고,
램프 중에는 ṡ 항이 추가 감속을 만들어 권한을 **더** 쓴다.

**다리가 실제로 하는 일** — 이것만 주장할 것:
  · 재계획 인터벌 동안 **물리적으로 실행 가능한** 기준을 준다 (물리 한계 사용 ≤0.80)
  · 기하를 안 바꾸므로 금지구역 재검사가 필요 없다
  · 감속하므로 오버슈트·종단이 좋아진다 (6.66 -> 1.87 cm, -72%)
  · 계획이 안 오면 경로 위에서 정지한다

외란 대응은 **rho 조속기와 돌풍 표**의 몫이지 다리의 몫이 아니다.

사용:
    br = plan_bridge(t, base, t_now=4.0,
                     limits_new=dict(v=0.8, a=0.4, j=1.0, snap=4.0),
                     replan_budget_s=0.25)
    br.pos          # 다리 궤적 (N x 3)
    br.handoff      # (p, v, a, j) — replan_splice 의 초기조건으로 그대로 넘긴다
    br.compliant_after_s
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import make_interp_spline

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 램프 길이 탐색 범위 [s]. 하한은 "이보다 짧으면 시계 변화가 사실상 계단" 이라
# 두는 값이고, 상한은 그 이상 끌면 위반 상태가 너무 오래 간다는 선.
T_RAMP_MIN = 0.06
T_RAMP_MAX = 4.0
T_RAMP_STEPS = 14           # 국소 삼분 탐색 횟수
EVAL_W = T_RAMP_MAX + 1.5   # 램프 길이 비교용 **고정** 평가창 [s].
                            # 창을 램프 길이에 맞춰 늘리면 긴 램프가 부당하게 유리해진다
                            # (위반이 창 밖으로 밀려나 안 보인다). 고정해야 짧은 램프의
                            # '높은 피크' 와 긴 램프의 '오래 감' 이 같은 저울에 올라간다.

# 물리 한계 (traj_pipeline.PHYS_*). 다리는 새 한계는 넘을 수 있어도 **이건 못 넘는다** —
# 넘으면 기체가 못 따라가서 다리 자체가 무의미해진다.
PHYS = dict(v=2.0, a=2.0, j=10.0, snap=80.0)
FAILSAFE_MARGIN = 1.5       # 예산의 이 배까지 기다렸다가 비상 정지로 넘어간다
S_AIM   = 0.97              # 목표 배율 여유 (한계에 정확히 걸치지 않게)
S_FLOOR = 0.05              # 시계 배율 하한 (0 이면 정지 — 램프 목표로는 쓰지 않음)

# 기저 한계 (capability._ANCHORS 1 kg). s 환산의 기준점.
BASE_LIMITS = dict(v=1.6, a=1.6, j=8.0, snap=64.0)


def _smoothstep7(u):
    """7차 스무드스텝 S(u): S(0)=0, S(1)=1, 1~3차 도함수가 양끝에서 0."""
    u = np.clip(u, 0.0, 1.0)
    return u**4 * (35.0 - 84.0 * u + 70.0 * u**2 - 20.0 * u**3)


def _smoothstep7_d(u):
    """S 의 1~3차 도함수. 구간 밖은 0 (양끝이 평평하다는 것이 이 함수를 쓰는 이유)."""
    inb = (u > 0.0) & (u < 1.0)
    uu = np.clip(u, 0.0, 1.0)
    d1 = 140.0 * uu**3 * (1.0 - uu)**3
    d2 = 420.0*uu**2 - 1680.0*uu**3 + 2100.0*uu**4 - 840.0*uu**5
    d3 = 840.0*uu - 5040.0*uu**2 + 8400.0*uu**3 - 4200.0*uu**4
    return d1 * inb, d2 * inb, d3 * inb


def _chain_derivs(spl, tau, s, sd1, sd2, sd3):
    """시간 스케일링 p(t) = p_ref(tau(t)), dtau/dt = s 의 v/a/j/snap — **해석적** 연쇄법칙.

    수치미분 4회는 여기서 못 쓴다. dt=0.01 격자에서 4차까지 반복 차분하면 스플라인
    보간 오차가 (2/dt)^4 로 증폭돼 스냅이 실제의 3~6배로 나온다 (2026-08-23 실측:
    같은 램프가 수치 6.31배 -> 해석 1.8배). 아래는 Faa di Bruno 전개를 4차까지 편 것
    (p1..p4 = 기준 궤적의 tau 에 대한 1~4차 도함수, s1..s3 = s 의 t 에 대한 도함수):

        v    = s*p1
        a    = s^2*p2 + s1*p1
        j    = s^3*p3 + 3*s*s1*p2 + s2*p1
        snap = s^4*p4 + 6*s^2*s1*p3 + (4*s*s2 + 3*s1^2)*p2 + s3*p1
    """
    p1, p2, p3, p4 = [spl.derivative(k)(tau) for k in (1, 2, 3, 4)]
    S = s[:, None]
    D1 = sd1[:, None]
    D2 = sd2[:, None]
    D3 = sd3[:, None]
    v = S * p1
    a = S**2 * p2 + D1 * p1
    j = S**3 * p3 + 3.0 * S * D1 * p2 + D2 * p1
    sn = (S**4 * p4 + 6.0 * S**2 * D1 * p3
          + (4.0 * S * D2 + 3.0 * D1**2) * p2 + D3 * p1)
    return v, a, j, sn


def _derivs(t, pos):
    """시계열 (t, pos) 의 v/a/j/snap — 중심차분(np.gradient) 4회.

    비상 정지 갈래처럼 s 가 두 램프의 곱이라 해석식이 지저분한 곳에서만 쓴다.
    지표 판정에는 쓰지 않는다 (`_chain_derivs` 주석 참조 — 4차에서 잡음이 폭증).
    """
    v = np.gradient(pos, t, axis=0)
    a = np.gradient(v, t, axis=0)
    j = np.gradient(a, t, axis=0)
    s = np.gradient(j, t, axis=0)
    return v, a, j, s


def _peak(arr):
    return float(np.max(np.linalg.norm(arr, axis=1))) if len(arr) else 0.0


def scale_for_limits(lim, base_limits=None):
    """새 한계가 기저의 몇 배인지 -> 시계 배율 s.

    v∝s, a∝s², j∝s³, snap∝s⁴ 이므로 각 축이 요구하는 s 는
        s_v = v/v0,  s_a = √(a/a0),  s_j = (j/j0)^{1/3},  s_snap = (snap/snap0)^{1/4}
    이고 **가장 빡빡한 것**을 따라야 한다.
    """
    b = base_limits or BASE_LIMITS
    cands = []
    for key, p in (("v", 1), ("a", 2), ("j", 3), ("snap", 4)):
        if b.get(key, 0) > 0 and float(lim.get(key, 0)) >= 0:
            cands.append((float(lim[key]) / b[key]) ** (1.0 / p))
    return min(cands) if cands else 1.0


@dataclass
class BridgePlan:
    """다리 한 장. `pos` 를 그대로 기준으로 내보내고, `handoff` 를 계획기에 넘긴다."""
    t: np.ndarray                 # 절대 시각 [s] (t_now 부터)
    pos: np.ndarray               # N x 3 기준 위치
    s_of_t: np.ndarray            # 적용된 시계 배율
    tau: np.ndarray               # 가상 시각 (옛 궤적 어디를 보고 있나)
    t_ramp: float                 # 채택된 램프 길이 [s]
    s_target: float
    t_handoff: float              # 이 시각의 상태를 replan_splice 초기조건으로
    handoff: tuple                # (p, v, a, j) — 각 (3,)
    peak: dict                    # 다리 전체 v/a/j/snap 최대
    excess_ratio: dict            # 램프 전체에서 새 한계 대비 최대 비율 (1.0 이하면 안 넘음)
    handoff_excess: dict          # 인계 시점 상태의 새 한계 대비 비율 (계획기 초기조건)
    planner_limits: dict          # 인계 상태를 받아들일 수 있게 푼 한계 — 새 계획은 이걸 써야 한다
    phys_use: dict                # 물리 한계 대비 최대 비율 — 1.0 넘으면 다리가 성립 안 함
    compliant_after_s: float      # t_now 로부터 이만큼 지나면 새 한계 안으로 들어옴
    failsafe_from_s: float        # 이 시각부터는 정지 갈래 (계획 미도착 대비)
    stopped: bool                 # 다리 끝에서 정지했나
    notes: list = field(default_factory=list)


def plan_bridge(t, base, t_now, limits_new, replan_budget_s,
                s_now=1.0, dt=None, failsafe_margin=FAILSAFE_MARGIN,
                base_limits=None):
    """옛 기준 궤적 위에서 시계를 늦춰 재계획 인터벌을 메운다.

    t, base        : 지금 따르고 있는 기준 (base = N x 3, t 균일 격자)
    t_now          : 감쇄를 시작하는 절대 시각 [s]
    limits_new     : 새 한계 dict(v, a, j, snap) — capability.json 의 limits 그대로
    replan_budget_s: 계획기가 걸릴 것으로 예측되는 시간 [s]
                     (compute_load.LoadGovernor.applied_s 또는 plan_segment 비용)
    s_now          : 지금 적용 중인 시계 배율 (연쇄 감쇄 시 1 이 아니다)
    base_limits    : s 환산 기준 (질량 앵커가 다르면 바꿔 넣는다 — 0 kg 은 v 1.2/a 1.0)

    램프 길이는 **새 한계를 넘는 양(초과 면적)이 최소가 되는 값**을 국소 탐색으로
    고른다. 너무 짧으면 ṡ 항이 가속을 튀게 하고, 너무 길면 위반이 오래 간다 —
    둘 다 나쁘고 그 사이에 최소가 있다.
    """
    t = np.asarray(t, float)
    base = np.asarray(base, float)
    if base.ndim != 2 or base.shape[1] != 3:
        raise ValueError("plan_bridge: base 는 N x 3 이어야 함")
    if len(t) != len(base):
        raise ValueError("plan_bridge: t 와 base 길이 불일치")
    if not (t[0] <= t_now < t[-1]):
        raise ValueError(f"plan_bridge: t_now={t_now} 가 기준 구간 밖")
    if dt is None:
        dt = float(t[1] - t[0])
    lim = {k: float(limits_new[k]) for k in ("v", "a", "j", "snap")}
    budget = max(float(replan_budget_s), dt)

    # 5차 스플라인 — 4차 도함수(스냅)까지 살아 있는 최소 차수. 3차면 스냅이 0 이 된다.
    k = 5 if len(t) > 5 else max(len(t) - 1, 1)
    spl = make_interp_spline(t, base, k=k)
    t_end_ref = float(t[-1])

    # 비상 정지 시점. **램프가 끝난 뒤**여야 한다 —
    # 예산x여유(예: 0.375 s)만 쓰면 램프(2.2 s)가 채 시작하기도 전에 정지 갈래가 켜져
    # 다리가 그냥 급정지가 된다 (2026-08-23 그림에서 드러남). 램프 길이는 아래에서
    # 정해지므로 여기서는 탐색용 임시값만 두고, 채택 후 max() 로 다시 잡는다.
    t_fail = t_now + budget * failsafe_margin
    # 목표 배율은 한계가 허용하는 값보다 살짝 아래로 잡는다. 정확히 맞추면 순항 구간이
    # 새 한계 위에 '올라앉아' 있게 되고, 그러면 다리가 끝나도 여전히 경계라 여유가 0 이다.
    s_tgt = float(np.clip(scale_for_limits(lim, base_limits) * S_AIM, S_FLOOR, 1.0))

    def build(t_ramp, stop_after=None, span=None):
        """램프 하나 만들어 시계열 전체를 돌려준다."""
        if span is None:
            span = (t_fail - t_now) + (stop_after or 0.0) + 2.0
        n = int(np.ceil(span / dt)) + 1
        tb = t_now + dt * np.arange(n)
        u = (tb - t_now) / max(t_ramp, 1e-9)
        s = s_now + (s_tgt - s_now) * _smoothstep7(u)
        if stop_after is not None:
            # 비상 갈래: t_fail 부터 stop_after 동안 s -> 0 (같은 7차 램프)
            u2 = (tb - t_fail) / max(stop_after, 1e-9)
            s = np.where(tb >= t_fail, s * (1.0 - _smoothstep7(u2)), s)
        # τ 적분 (사다리꼴). 기준 끝을 넘으면 끝점에 고정 = 그 자리 정지.
        tau = t_now + np.concatenate([[0.0], np.cumsum(0.5 * (s[1:] + s[:-1]) * dt)])
        tau = np.minimum(tau, t_end_ref)
        pos = spl(np.clip(tau, t[0], t_end_ref))
        return tb, pos, s, tau

    def ramp_kin(t_ramp):
        """램프 갈래(정지 꼬리 없음)의 상태량 — 해석적 연쇄법칙으로."""
        tb, pos, s, tau = build(t_ramp, span=EVAL_W)
        tr = max(t_ramp, 1e-9)
        d1, d2, d3 = _smoothstep7_d((tb - t_now) / tr)
        ds = s_tgt - s_now
        kin = _chain_derivs(spl, np.clip(tau, t[0], t_end_ref), s,
                            ds * d1 / tr, ds * d2 / tr**2, ds * d3 / tr**3)
        return tb, pos, s, tau, kin

    def ratios(kin, ref):
        r = np.zeros(len(kin[0]))
        for arr, key in zip(kin, ("v", "a", "j", "snap")):
            if ref[key] > 0:
                r = np.maximum(r, np.linalg.norm(arr, axis=1) / ref[key])
        return r

    def cost(t_ramp):
        # 램프 갈래만 본다 (비상 정지 꼬리는 다른 갈래라 여기 섞으면 판단이 오염된다).
        *_, kin = ramp_kin(t_ramp)
        c = float(np.sum(np.maximum(ratios(kin, lim) - 1.0, 0.0)) * dt)   # 새 한계 초과 '면적'
        # 물리 한계 위반은 타협 불가 — 큰 가중치로 사실상 배제한다
        c += 1e3 * float(np.sum(np.maximum(ratios(kin, PHYS) - 1.0, 0.0)) * dt)
        return c

    lo, hi = T_RAMP_MIN, T_RAMP_MAX
    grid = np.geomspace(lo, hi, 13)
    costs = [cost(x) for x in grid]
    i = int(np.argmin(costs))
    lo2, hi2 = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
    for _ in range(T_RAMP_STEPS):          # 삼분 탐색 (목적함수가 U 자)
        m1 = lo2 + (hi2 - lo2) / 3.0
        m2 = hi2 - (hi2 - lo2) / 3.0
        if cost(m1) <= cost(m2):
            hi2 = m2
        else:
            lo2 = m1
    t_ramp = 0.5 * (lo2 + hi2)

    # ── 지표는 '순수 램프'(정지 꼬리 없음) 위에서 잰다 ──────────────────
    # 정지 꼬리는 계획이 안 왔을 때만 타는 다른 갈래라, 섞으면 램프 평가가 오염된다.
    tbR, posR, sR, _tauR, (vR, aR, jR, snR) = ramp_kin(t_ramp)

    excess, phys_use = {}, {}
    for arr, key in ((vR, "v"), (aR, "a"), (jR, "j"), (snR, "snap")):
        pk = float(np.max(np.linalg.norm(arr, axis=1)))
        excess[key] = pk / lim[key] if lim[key] > 0 else 0.0
        phys_use[key] = pk / PHYS[key]

    over = np.zeros(len(tbR), bool)
    for arr, key in ((vR, "v"), (aR, "a"), (jR, "j"), (snR, "snap")):
        if lim[key] > 0:
            over |= np.linalg.norm(arr, axis=1) > lim[key] * 1.001
    idx = np.where(over)[0]
    compliant_after = float(tbR[idx[-1]] - t_now) if len(idx) else 0.0
    if len(idx) and idx[-1] >= len(tbR) - 2:
        compliant_after = float("inf")      # 평가창 끝까지 위반 = 이 램프로는 못 들어옴

    # 내보낼 궤적은 램프 + 비상 정지 꼬리 (계획 미도착 대비)
    # 램프를 다 내려간 **뒤에** 정지 갈래로 넘어간다. 그전까지는 옛 기하 위를 느리게
    # 날 뿐이고, 그건 그 자체로 안전하다 (경로가 바뀐 게 아니라 한계가 바뀐 것이므로
    # 금지구역 판정이 그대로 유효하다). 계획이 계속 안 오면 그때 경로 위에서 선다.
    t_fail = t_now + max(budget * failsafe_margin, t_ramp)
    stop_len = max(t_ramp, 0.5)            # 정지 꼬리도 같은 규약 (급정지 금지)
    tb, pos, s, tau = build(t_ramp, stop_after=stop_len)
    v, a, j, sn = _derivs(tb, pos)

    ih = int(np.clip(np.round(budget / dt), 1, len(tbR) - 2))
    handoff = (posR[ih].copy(), vR[ih].copy(), aR[ih].copy(), jR[ih].copy())
    # 인계 시점의 상태가 새 한계를 얼마나 넘는가 — 이게 새 계획의 초기조건이 된다.
    # 계획기는 v(0)=v0 를 고정으로 받으므로, v0 > v_max 인 한계로는 **어떤 T 로도**
    # 실행 가능한 세그먼트를 못 만든다 (min-time 탐색이 상한까지 밀린다).
    # 그래서 상위는 인계용 한계를 이 비율만큼 풀어 줘야 한다 -> `planner_limits`.
    hand_excess = {}
    for val, key in ((handoff[1], "v"), (handoff[2], "a"), (handoff[3], "j")):
        hand_excess[key] = (float(np.linalg.norm(val)) / lim[key]) if lim[key] > 0 else 0.0

    # 계획기에 넘길 한계 — 인계 상태를 못 담으면 min-time 탐색이 수렴하지 않는다.
    # 새 한계와 '인계 상태 x 5% 여유' 중 큰 쪽. 물리 한계는 넘지 않는다.
    planner_limits = {}
    for key in ("v", "a", "j", "snap"):
        # snap 은 hand_excess 에 없다 — 계획기가 받는 초기조건은 (p, v, a, j) 뿐이고
        # snap 은 세그먼트 내부에서만 제약되므로 풀어 줄 이유가 없다. `.get` 의 0 은
        # 누락이 아니라 "이 축은 초기조건이 아니다" 라는 뜻이다.
        need = hand_excess.get(key, 0.0) * lim[key] * 1.05
        planner_limits[key] = float(min(max(lim[key], need), PHYS[key]))

    notes = []
    if compliant_after > budget:
        notes.append(
            f"새 한계 진입 {compliant_after:.2f}s > 재계획 예산 {budget:.2f}s — "
            "감쇄를 더 일찍(예측 기반) 시작해야 한다")
    if tau[-1] >= t_end_ref - 1e-9:
        notes.append("옛 기준의 끝에 도달 — 다리가 종점 래치로 끝난다")
    worst_phys = max(phys_use, key=phys_use.get)
    if phys_use[worst_phys] > 1.0:
        notes.append(
            f"물리 한계 초과 ({worst_phys} {phys_use[worst_phys]:.2f}배) — 이 감쇄폭은 "
            "시계 감속만으로 다리를 놓을 수 없다. 기하까지 바꾸는 비상 정지로 가야 한다")

    return BridgePlan(
        t=tb, pos=pos, s_of_t=s, tau=tau, t_ramp=float(t_ramp), s_target=s_tgt,
        t_handoff=float(tb[ih]), handoff=handoff,
        peak=dict(v=_peak(v), a=_peak(a), j=_peak(j), snap=_peak(sn)),
        excess_ratio=excess, handoff_excess=hand_excess,
        planner_limits=planner_limits, phys_use=phys_use,
        compliant_after_s=compliant_after,
        failsafe_from_s=float(t_fail), stopped=bool(abs(s[-1]) < 1e-6), notes=notes)


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    import time as _time

    DT = 0.01

    def make_ref(dx, tmove, dt=DT, tail=6.0):
        """시험용 기준 궤적 — **7차 스무드스텝** 위치 프로파일.

        레이즈드 코사인을 쓰면 안 된다. 종점에서 x'' 가 0 이 아닌 채로 끊겨 C² 꺾임이
        생기고, 그 자리에서 5차 스플라인의 4차 도함수가 링잉하며 폭발한다
        (2026-08-23 실측: 스냅이 물리 한계의 38배로 나옴 — 다리가 아니라 시험 기준의 탈).
        스무드스텝은 3차까지 양끝이 평평해 그 인공물이 없다.
        """
        tt = np.arange(0.0, tmove + tail + dt, dt)
        xs = dx * _smoothstep7(tt / tmove)
        return tt, np.column_stack([xs, np.zeros_like(tt), np.ones_like(tt)])

    # 해석식 vs 수치미분 대조 (매끄러운 구간에서만 — 둘이 맞아야 연쇄법칙이 맞는 것)
    _tt, _bb = make_ref(6.0, 6.0)
    _spl = make_interp_spline(_tt, _bb, k=5)
    _n = len(_tt)
    _one = np.ones(_n)
    _z = np.zeros(_n)
    _va = _chain_derivs(_spl, _tt, _one, _z, _z, _z)
    _vn = _derivs(_tt, _bb)
    _m = (_tt > 0.5) & (_tt < 5.5)
    print("해석 vs 수치 (s=1 구간, 상대오차):", ", ".join(
        f"{k} {np.max(np.abs(_va[i][_m] - _vn[i][_m])) / max(np.max(np.abs(_vn[i][_m])), 1e-9):.1e}"
        for i, k in enumerate(("v", "a", "j", "snap"))))

    for label, blim in (("1 kg", BASE_LIMITS), ("0 kg", dict(v=1.2, a=1.0, j=8.0, snap=64.0))):
        v0 = blim["v"]
        # 스무드스텝의 피크 속도 = 2.1875·dx/T  ->  T 를 v0 에 맞춘다
        TM = 2.1875 * 6.0 / v0
        tt, bb = make_ref(6.0, TM)
        print(f"\n== 기저 {label} (v0 {v0} m/s), 순항 중간에서 감쇄 시작, 예산 0.25 s ==")
        print(f"{'새 v':>7}{'s*':>7}{'램프[s]':>9}{'초과v':>7}{'초과a':>7}"
              f"{'초과j':>7}{'물리사용':>9}{'진입[s]':>9}{'인계v':>8}{'계산ms':>9}")
        for frac in (0.75, 0.50, 0.30, 0.18):
            s = frac
            lim = {k: blim[k] * s ** p for k, p in
                   (("v", 1), ("a", 2), ("j", 3), ("snap", 4))}
            t0 = _time.perf_counter()
            br = plan_bridge(tt, bb, t_now=TM / 2, limits_new=lim,
                             replan_budget_s=0.25, base_limits=blim)
            ms = (_time.perf_counter() - t0) * 1e3
            print(f"{lim['v']:>7.2f}{br.s_target:>7.2f}{br.t_ramp:>9.2f}"
                  f"{br.excess_ratio['v']:>7.2f}{br.excess_ratio['a']:>7.2f}"
                  f"{br.excess_ratio['j']:>7.2f}{br.phys_use[max(br.phys_use, key=br.phys_use.get)]:>9.2f}"
                  f"{br.compliant_after_s:>9.2f}{br.handoff_excess['v']:>8.2f}{ms:>9.1f}")
            for n in br.notes:
                print(f"        · {n}")
