"""회복 감시 — 실측표가 틀렸을 때 스펙을 폐루프로 교정한다.

2026-08-23 신설. 사용자 설계:
    "스펙 나누는 거 그것도 제어 루프 돌리는 거 적당히 보다가 만약 회복이 너무 더디면
     스펙 깎고 그러는 방식으로"

## 왜 표만으로는 부족한가

`capability._LAT_POS_ANCHORS` 는 특정 조합(1 kg, 자세 5 ms, 3 m 이동, 0.3 N·m)에서
잰 것이다. 실제 운용이 그 격자 위에 정확히 놓일 이유가 없다. 오늘만 해도 구멍이 셋:
질량별 표 미구현(0 kg 은 1 kg 과 딴판) / 2 kg 앵커가 1 kg 복사본 / 자세 지연이
구성값이라 실기에서 틀릴 수 있음.

그래서 **표 = 피드포워드, 이 감시 = 피드백**. 지연을 예측(부하 모델) + 실측 둘 다로
잡은 것과 같은 구조다. 표는 처음부터 배우지 않아도 되게 시작점을 주고, 감시는 그
시작점이 틀렸을 때 교정한다.

## 무엇을 "회복이 더디다" 로 보나

비행 중에는 "외란이 끝났다" 는 이벤트가 없다. 대신 **밴드 초과 지속시간**을 쓴다:

    e(t)    = |측정 - 기준|              (제어 주기에 이미 있는 값)
    t_above = e 가 track 밴드를 **연속으로** 넘고 있는 시간

이게 MATLAB 게이트로 쓴 "외란 복귀 시간" 의 온라인 대응물이다. 넘은 채로 settle 을
지나면 회복이 더딘 것. 에피소드가 끝나기를 기다리지 않고 **넘고 있는 동안** 반응한다.

## 두 가지 안전장치

### ① 판단 주기 > 다리 수렴 시간  (하드 제약)

2026-08-23 실측: 감쇄를 결정한 **뒤에도** 새 한계 안으로 들어가는 데 0.66~3.94 s 가
걸린다 (`traj_bridge`, 깊이 깎을수록 길다). 그보다 짧은 주기로 다시 판단하면 앞선
결정이 반영되기 전에 또 결정하게 되고, 그게 곧 발진이다. 그래서 판단 주기는
`max(min_period_s, 직전 다리의 compliant_after_s x lead_margin)` 이상으로 둔다.

### ② 기준이 한계 안에 있을 때만 계상

기준 궤적 자체가 한계를 넘는 물건이면(상위가 잘못 준 경우) 추종 오차가 커지는데,
그건 **스펙을 깎아서 고칠 문제가 아니다** — 계획 쪽 문제다. 그걸로 스펙을 깎으면
잘못된 계획이 기체 능력을 갉아먹는 되먹임이 된다. `ref_ok=False` 인 표본은 버린다.

사용:
    w = RecoveryWatcher(track_band_m=0.04, settle_s=2.2)
    ...  # 제어 주기마다
    w.observe(err_m=0.06, ref_ok=True, dt=0.001)
    ...  # 판단 주기마다 (또는 매번 불러도 내부에서 주기를 지킨다)
    s_rec = w.decide(bridge_lead_s=br.compliant_after_s)
"""
from __future__ import annotations

from dataclasses import dataclass, field

# 다리 수렴 시간에 이만큼 여유를 곱해 판단 주기 하한으로 쓴다.
LEAD_MARGIN = 1.5
# 감시가 낼 수 있는 최저 배율. 0 까지 내리는 것은 정지 판단이라 감시의 권한이 아니다
# (비상 정지는 감독자 §9 의 몫). 감시는 "느리게" 까지만 한다.
S_FLOOR = 0.15


@dataclass
class RecoveryWatcher:
    """추종 오차의 밴드 초과 지속시간을 보고 스펙 배율을 교정한다.

    track_band_m : 이 위로 벗어나면 '회복 중' 으로 본다 (capability.budget.track)
    settle_s     : 이만큼 넘게 못 돌아오면 '더디다' (capability.budget.settle)
    cut_gain     : 초과 비율 1당 깎는 양. 0.5 면 settle 의 2배를 끌 때 배율 -0.5
    max_cut      : **한 번의 판단**에서 깎을 수 있는 최대량. 이게 없으면 오차가 계속
                   밴드 위에 있을 때 비율이 무한히 커져 두세 번 만에 바닥을 친다.
                   판단 주기가 다리 수렴 시간보다 길게 잡혀 있으므로, 한 주기에 한
                   걸음씩 내려가야 그 걸음의 효과를 보고 다음을 정할 수 있다.
    min_period_s : 판단 주기 하한 [s] — 다리 수렴 시간과 함께 max 로 쓴다
    clean_hold_s : 복귀 시작 전 밴드 아래로 유지돼야 하는 시간
    restore_tau_s: 복귀 지수 시정수
    """

    track_band_m: float = 0.04
    settle_s: float = 2.2
    cut_gain: float = 0.5
    max_cut: float = 0.25
    min_period_s: float = 4.0
    clean_hold_s: float = 3.0
    restore_tau_s: float = 6.0

    # ── 상태 ──
    s: float = field(default=1.0, init=False)          # 현재 교정 배율
    t_above: float = field(default=0.0, init=False)    # 지금 연속 초과 시간
    worst_above: float = field(default=0.0, init=False)  # 이번 판단 창의 최악
    t_clean: float = field(default=0.0, init=False)    # 연속 정상 시간
    t_since: float = field(default=0.0, init=False)    # 마지막 판단 이후
    n_obs: int = field(default=0, init=False)
    n_skipped: int = field(default=0, init=False)      # ref 가 한계 밖이라 버린 표본
    cuts: int = field(default=0, init=False)
    restoring: bool = field(default=False, init=False)
    last_ratio: float = field(default=0.0, init=False)

    def reset(self) -> None:
        self.s = 1.0
        self.t_above = self.worst_above = self.t_clean = self.t_since = 0.0
        self.n_obs = self.n_skipped = self.cuts = 0
        self.restoring = False
        self.last_ratio = 0.0

    # ── 제어 주기 ────────────────────────────────────────────────────────
    def observe(self, err_m: float, ref_ok: bool, dt: float) -> None:
        """추종 오차 한 표본. `ref_ok` 는 '지금 기준이 현재 limits 안인가'.

        `ref_ok=False` 는 **버린다** — 계획이 과한 것을 제어기 탓으로 돌려 스펙을
        깎으면, 잘못된 계획이 기체 능력을 갉아먹는 되먹임이 생긴다.
        시간 누적(`t_since`)은 계속 돌린다 — 판단 주기는 실제 시간으로 세야 한다.
        """
        dt = max(float(dt), 0.0)
        self.t_since += dt
        if not ref_ok:
            self.n_skipped += 1
            return
        self.n_obs += 1
        if abs(float(err_m)) > self.track_band_m:
            self.t_above += dt
            self.t_clean = 0.0
            if self.t_above > self.worst_above:
                self.worst_above = self.t_above
        else:
            self.t_above = 0.0
            self.t_clean += dt

    # ── 판단 주기 ────────────────────────────────────────────────────────
    def period_s(self, bridge_lead_s: float | None = None) -> float:
        """이번 판단까지 기다려야 하는 시간. 다리 수렴보다 짧으면 안 된다."""
        p = self.min_period_s
        if bridge_lead_s is not None and bridge_lead_s == bridge_lead_s:  # NaN 방어
            p = max(p, float(bridge_lead_s) * LEAD_MARGIN)
        return p

    def due(self, bridge_lead_s: float | None = None) -> bool:
        return self.t_since >= self.period_s(bridge_lead_s)

    def decide(self, bridge_lead_s: float | None = None) -> float:
        """주기가 됐으면 교정 배율을 갱신하고 돌려준다. 아니면 현재 값 그대로.

        깎기는 즉시, 되돌리기는 확인 후 천천히 — `LoadGovernor` 와 같은 비대칭.
        경계에서 대칭으로 두면 스펙이 요동치고, 그게 재계획을 유발해 부하가 또
        오르는 양의 되먹임이 된다.
        """
        if not self.due(bridge_lead_s):
            return self.s
        elapsed = self.t_since
        self.t_since = 0.0

        ratio = self.worst_above / max(self.settle_s, 1e-9)
        self.last_ratio = ratio
        self.worst_above = self.t_above      # 아직 넘고 있으면 그 시간은 이월한다

        if ratio > 1.0:
            # 더디다 -> 즉시 깎는다. 초과분에 비례하되 한 걸음 폭은 제한한다.
            step = min(self.cut_gain * (ratio - 1.0), self.max_cut)
            self.s = max(S_FLOOR, self.s - step)
            self.cuts += 1
            self.restoring = False
        elif self.t_clean >= self.clean_hold_s and self.s < 1.0:
            a = min(max(elapsed / max(self.restore_tau_s, 1e-9), 0.0), 1.0)
            self.s += a * (1.0 - self.s)
            self.restoring = True
            if 1.0 - self.s < 1e-6:
                self.s = 1.0
                self.restoring = False
        else:
            self.restoring = False
        return self.s

    def snapshot(self) -> dict:
        """`capability.observed.recovery` 에 그대로 넣을 블록."""
        return {
            "scale": round(self.s, 4),
            "t_above_s": round(self.t_above, 3),
            "last_ratio": round(self.last_ratio, 3),
            "cuts": self.cuts,
            "restoring": self.restoring,
            "samples": self.n_obs,
            "skipped_ref_over_limits": self.n_skipped,
        }


if __name__ == "__main__":
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    DT = 0.01
    w = RecoveryWatcher()
    print(f"{'t[s]':>6}{'오차[cm]':>10}{'초과[s]':>9}{'배율':>7}  비고")
    t = 0.0
    for step in range(9000):
        # 0~20 s 평온 / 20~45 s 회복이 더딘 구간 / 45 s~ 정상
        if 20.0 <= t < 50.0:
            err = 0.09          # 밴드(4 cm) 위에 계속 머문다 = 안 잦아듦
        else:
            err = 0.01
        w.observe(err, ref_ok=True, dt=DT)
        before = w.s
        s = w.decide(bridge_lead_s=2.0)
        if s != before:
            tag = "깎음" if s < before else "복귀"
            print(f"{t:>6.1f}{err*100:>10.1f}{w.t_above:>9.1f}{s:>7.2f}  {tag}"
                  f" (비율 {w.last_ratio:.2f})")
        t += DT
    print(f"\n최종 배율 {w.s:.2f}, 깎은 횟수 {w.cuts}, 판단 주기 "
          f"{w.period_s(2.0):.1f} s (다리 수렴 2.0 s x {LEAD_MARGIN})")
