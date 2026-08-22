"""연산 부하 추정 → 지연 예측 → 스펙 감쇄.

2026-08-22 신설. 사용자 요구: "현재 연산 부하의 양을 추정하고 그에 맞춰 spec 을 감소".

왜 부하를 보나 (지연만 재면 늦다)
  `latency_tracker` 는 **이미 일어난** 지연을 잰다. 그건 사후 지표라, 부하가 올라가는
  중에는 아직 지연이 안 나타나서 스펙이 안 깎이고, 깎일 즈음엔 이미 궤적이 틀어져 있다.
  부하는 **선행 지표**다 — 무엇을 얼마나 자주 돌릴지는 미리 알 수 있으므로,
  지연이 나타나기 전에 예측해서 깎을 수 있다.

근거는 **예상량과 실측값 둘 다** (사용자 지시)
  · 모델(예상) = 선행 지표 — 부하 스케줄에서 미리 계산, 지연이 나타나기 전에 깎는다
  · 실측       = 백스톱   — 모델이 모르는 원인(OneDrive 잠금·GC·타 프로세스)까지 잡는다
  `LoadGovernor.fuse()` 가 둘 중 나쁜 쪽을 취하고, 차이(실측−예상)를 bias 로 들고 있는다.

모델
  1. 작업별 비용    cost(n) = fixed + per_unit · n        (n = 샘플수/세그먼트수)
     이 노트북 실측(2026-08-22):
       traj_smoother  25.1 us/샘플     (R2 0.9995, 고정항 ~0)
       traj_zv(zvd)    0.04 ms 고정    (무시 가능)
       traj_gate       0.1 ms 고정 + 미미한 선형
       plan_waypoints  5.3 ms/세그먼트
  2. 점유율(duty)   duty = sum(cost_i * rate_i) / 1초      [0,1)
  3. 대기 지연      M/D/1 근사:  W = duty * S / (2 * (1 - duty))
     체류 지연      R = S + W        <- 이 값을 capability 의 latency 로 먹인다
     duty -> 1 에서 발산하는 것이 핵심 — "조금만 더 넣으면 갑자기 늦어진다" 를 재현한다.

한계: 단일 코어·선입선출 가정. 병렬 실행이나 우선순위 선점은 반영하지 않는다.
      실측 표본이 들어오면 `observe()` 로 상수를 온라인 보정한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

# 이 개발 머신 실측 기저 (Python, 2026-08-22). 다른 하드웨어면 observe() 로 덮인다.
DEFAULT_COSTS = {
    "smoother":      (0.0,     25.1e-6),   # (고정 [s], 샘플당 [s])
    "zv":            (4.0e-5,   0.0),
    "gate":          (1.0e-4,   5.0e-8),
    "plan_segment":  (0.0,      5.3e-3),   # 세그먼트당
    "capability":    (5.0e-5,   0.0),
}

DUTY_CAP = 0.95        # 이 이상은 포화로 보고 잘라 쓴다 (M/D/1 이 발산)


@dataclass
class TaskCost:
    """단일 작업의 비용 모델. 실측이 들어오면 최소제곱으로 온라인 보정."""

    fixed_s: float
    per_unit_s: float
    # 온라인 최소제곱 누적 (n, t) — 정규방정식용
    _n: int = field(default=0, init=False)
    _sn: float = field(default=0.0, init=False)
    _snn: float = field(default=0.0, init=False)
    _st: float = field(default=0.0, init=False)
    _snt: float = field(default=0.0, init=False)

    def predict(self, units: float) -> float:
        return max(self.fixed_s + self.per_unit_s * max(float(units), 0.0), 0.0)

    def observe(self, units: float, elapsed_s: float) -> None:
        """실측 한 점 반영. 점이 2개 이상이고 분산이 있으면 (fixed, per_unit) 갱신."""
        n, t = float(units), max(float(elapsed_s), 0.0)
        self._n += 1
        self._sn += n
        self._snn += n * n
        self._st += t
        self._snt += n * t
        if self._n < 2:
            return
        den = self._n * self._snn - self._sn ** 2
        if abs(den) < 1e-12:          # 전부 같은 크기 — 기울기 못 뽑음, 고정항만 보정
            self.fixed_s = max(self._st / self._n - self.per_unit_s * (self._sn / self._n), 0.0)
            return
        slope = (self._n * self._snt - self._sn * self._st) / den
        inter = (self._st - slope * self._sn) / self._n
        self.per_unit_s = max(slope, 0.0)
        self.fixed_s = max(inter, 0.0)


@dataclass
class LoadEstimator:
    """등록된 작업 스케줄로 점유율과 예측 지연을 낸다.

    schedule : {작업명: (단위수, 초당 실행횟수)}  — 예 {"smoother": (1000, 1.0)}
    """

    costs: dict = field(default_factory=lambda: {k: TaskCost(*v)
                                                 for k, v in DEFAULT_COSTS.items()})
    schedule: dict = field(default_factory=dict)

    # ── 등록 / 보정 ──────────────────────────────────────────────────────
    def set_task(self, name: str, units: float, rate_hz: float) -> None:
        if name not in self.costs:
            self.costs[name] = TaskCost(0.0, 0.0)
        self.schedule[name] = (float(units), float(rate_hz))

    def clear_task(self, name: str) -> None:
        self.schedule.pop(name, None)

    def observe(self, name: str, units: float, elapsed_s: float) -> None:
        if name not in self.costs:
            self.costs[name] = TaskCost(0.0, 0.0)
        self.costs[name].observe(units, elapsed_s)

    # ── 산출 ─────────────────────────────────────────────────────────────
    def per_task(self) -> dict:
        """작업별 {비용[s], 초당점유[s/s]}."""
        out = {}
        for name, (units, rate) in self.schedule.items():
            c = self.costs[name].predict(units)
            out[name] = {"cost_s": c, "duty": c * rate}
        return out

    def duty(self) -> float:
        """총 점유율 [0, DUTY_CAP]. 1 을 넘으면 이미 실시간을 못 지키는 것."""
        raw = sum(v["duty"] for v in self.per_task().values())
        return min(raw, DUTY_CAP)

    def raw_duty(self) -> float:
        """자르기 전 원값 — 1 초과 여부를 상위가 알 수 있게."""
        return sum(v["duty"] for v in self.per_task().values())

    def mean_service_s(self) -> float:
        """실행 1회당 평균 서비스 시간 (실행 빈도 가중)."""
        pt = self.per_task()
        tot_rate = sum(self.schedule[n][1] for n in pt)
        if tot_rate <= 0.0:
            return 0.0
        return sum(v["cost_s"] * self.schedule[n][1] for n, v in pt.items()) / tot_rate

    def predicted_latency_s(self) -> float:
        """체류 지연 R = S + W,  W = duty·S / (2(1−duty))   (M/D/1 근사).

        duty -> 1 에서 발산 — "조금만 더 얹으면 갑자기 늦어진다" 를 재현한다.
        이 값을 `capability.build_capability(latency_s=...)` 에 그대로 먹인다.
        """
        s = self.mean_service_s()
        if s <= 0.0:
            return 0.0
        d = self.duty()
        w = d * s / (2.0 * (1.0 - d)) if d < 1.0 else float("inf")
        return s + w

    def snapshot(self) -> dict:
        """capability.json `observed.load` 에 넣을 블록."""
        return {
            "duty": round(self.duty(), 4),
            "raw_duty": round(self.raw_duty(), 4),
            "saturated": self.raw_duty() >= DUTY_CAP,
            "service_s": round(self.mean_service_s(), 6),
            "predicted_latency_s": round(self.predicted_latency_s(), 6),
            "tasks": {n: round(v["duty"], 5) for n, v in self.per_task().items()},
        }


@dataclass
class LoadGovernor:
    """예측 지연을 **적용값**으로 바꾸는 양방향 조속 — 올릴 땐 즉시, 내릴 땐 확인 후 천천히.

    사용자 요구: "부하가 줄면 다시 향상시키는 것도 포함". 다만 그대로 되돌리면
    부하가 경계에서 오르내릴 때 스펙이 초 단위로 요동친다(그러면 계획기가 계속
    재계획 -> 부하가 또 오르는 양의 되먹임). 그래서 비대칭으로 간다:

      상승(스펙 감소) : 즉시  — 늦으면 이미 궤적이 틀어진다
      하강(스펙 복귀) : `hold_n` 표본 연속으로 낮아야 시작, 이후 `fall_tau_s` 로 지수 감쇠

    조속기(SPEED_GOVERNOR §5.2)·지연 추적기와 같은 규약이라 상위가 규칙을 하나만 외우면 된다.
    """

    fall_tau_s: float = 3.0      # 복귀 시정수 [s]
    hold_n: int = 10             # 복귀 시작 전 연속 정상 표본수
    bias_n: int = 30             # 모델 오차(실측−예상) EMA 유효 표본수
    applied_s: float = field(default=0.0, init=False)
    low_run: int = field(default=0, init=False)
    restoring: bool = field(default=False, init=False)
    source: str = field(default="model", init=False)      # 무엇이 지배했나
    bias_s: float = field(default=0.0, init=False)        # 실측 − 예상 (EMA)
    _bias_n: int = field(default=0, init=False)

    def reset(self) -> None:
        self.applied_s = 0.0
        self.low_run = 0
        self.restoring = False
        self.source = "model"
        self.bias_s = 0.0
        self._bias_n = 0

    def fuse(self, predicted_s: float, measured_s: float = 0.0) -> float:
        """**예상량과 실측값 둘 다**를 근거로 하나의 지연을 만든다.

        규칙: 둘 중 나쁜 쪽(큰 쪽)을 취한다.
          · 모델(예상)은 **선행** — 부하가 올라가는 순간 지연이 나타나기 전에 잡는다.
          · 실측은 **백스톱** — 모델이 모르는 원인(OneDrive 잠금, GC, 다른 프로세스)까지 잡는다.
        같이 모델 오차 `bias = 실측 − 예상` 을 EMA 로 들고 있어, 모델이 계속 과소예측하면
        그 편차가 드러난다 (상위·사람이 보정 판단에 쓴다).
        """
        p = max(float(predicted_s), 0.0)
        m = max(float(measured_s), 0.0)
        if m > 0.0:
            self._bias_n += 1
            a = 2.0 / (min(self._bias_n, self.bias_n) + 1.0)
            self.bias_s += a * ((m - p) - self.bias_s)
        self.source = "measured" if m > p else "model"
        return max(p, m)

    def update(self, predicted_s: float, measured_s: float = 0.0,
               dt: float = 0.2) -> float:
        """예상+실측 한 표본 반영 -> 지금 적용할 지연 [s]."""
        p = self.fuse(predicted_s, measured_s)
        if p >= self.applied_s:                     # 부하 증가: 즉시 반영
            self.applied_s = p
            self.low_run = 0
            self.restoring = False
            return self.applied_s
        # 부하 감소: 연속으로 낮게 유지될 때만 복귀 시작
        self.low_run += 1
        if self.low_run < self.hold_n:
            self.restoring = False
            return self.applied_s
        self.restoring = True
        a = min(max(float(dt) / max(self.fall_tau_s, 1e-9), 0.0), 1.0)
        self.applied_s += a * (p - self.applied_s)
        if abs(self.applied_s - p) < 1e-9:
            self.applied_s = p
            self.restoring = False
        return self.applied_s

    def snapshot(self) -> dict:
        return {
            "applied_latency_s": round(self.applied_s, 6),
            "source": self.source,               # model | measured (무엇이 지배했나)
            "model_bias_s": round(self.bias_s, 6),   # 실측 − 예상 (양수면 모델이 과소예측)
            "restoring": self.restoring,
            "low_run": self.low_run,
        }


def horizon_for_budget(est: LoadEstimator, budget_s: float, dt: float = 0.01,
                       rate_hz: float = 1.0, task: str = "smoother") -> float:
    """주어진 시간 예산 안에 드는 **재계획 지평**[s] — 부하를 거꾸로 푸는 쪽.

    스펙을 깎는 대신 '한 번에 다시 만드는 구간'을 줄여도 부하가 준다
    (SPEED_GOVERNOR §10.3 리시딩 호라이즌). 상위가 둘 중 고를 수 있게 같이 제공한다.
    """
    c = est.costs.get(task)
    if c is None or c.per_unit_s <= 0.0:
        return float("inf")
    avail = max(float(budget_s) * max(rate_hz, 1e-9), 0.0)
    n = max((avail - c.fixed_s) / c.per_unit_s, 0.0)
    return n * float(dt)


if __name__ == "__main__":
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print(f"{'구성':<44}{'duty':>8}{'service':>10}{'지연예측':>12}")
    for label, tasks in (
        ("평시: 5 s 지평 재성형 1 Hz",        {"smoother": (500, 1.0), "gate": (1, 5.0)}),
        ("보통: 10 s 지평 2 Hz",              {"smoother": (1000, 2.0), "gate": (1, 5.0)}),
        ("과중: 20 s 지평 2 Hz + 계획 8세그 1 Hz",
         {"smoother": (2000, 2.0), "plan_segment": (8, 1.0), "gate": (1, 5.0)}),
        ("포화: 20 s 지평 5 Hz",              {"smoother": (2000, 5.0), "gate": (1, 10.0)}),
    ):
        e = LoadEstimator()
        for n, (u, r) in tasks.items():
            e.set_task(n, u, r)
        print(f"{label:<44}{e.duty():>8.3f}{e.mean_service_s()*1e3:>9.1f}ms"
              f"{e.predicted_latency_s()*1e3:>10.1f}ms")
