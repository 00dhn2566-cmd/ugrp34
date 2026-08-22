"""시간 지연 추적기 — 관측 → 평균 예측 → 스펙 감쇄값 산출.

2026-08-22 신설. 사용자 요구: "현재 시간 지연 양 트래킹해서, 지연 걸렸다 하면
평균 시간 지연 예측해서 그에 맞춰 스펙 조정".

설계
  · 지연은 **한 샘플이 아니라 평균**으로 판단한다. 단발 스파이크(스케줄러 지터,
    파일 잠금)로 스펙을 깎으면 순항 속도가 계속 요동친다.
  · 그래서 EMA 두 개를 쓴다 — 빠른 EMA 로 '걸렸다'를 감지하고, 느린 EMA 를
    '예측 지연'으로 내보낸다. 감지는 빠르게, 반영은 안정적으로.
  · 해제는 회복까지 유지 (`hold_until_recovered`) — 지연이 사라져도 T_hold 동안
    깨끗해야 정상으로 돌아간다. 조속기(SPEED_GOVERNOR §5.2)와 같은 규약.

무엇을 넣나 (호출자가 고르는 표본)
  · `current_state.json` 의 timestamp 나이 (INTERFACE_SPEC §8c T8)
  · 명령→응답 등가 지연 (§8c T4)
  · 계획 동사 왕복 시간 (§8c T7)
셋 다 "상위가 본 세상이 얼마나 낡았나" 라는 같은 단위(초)라 한 추적기에 넣어도 된다.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class LatencyTracker:
    """지연 표본을 받아 예측 지연과 감지 상태를 낸다.

    baseline_s : 정상 지연 (이 아래는 '지연 없음' 취급). 기본 30 Hz 상태 주기의 절반.
    trigger_s  : 이만큼 넘으면 '지연 걸림' 판정 (빠른 EMA 기준)
    tau_fast_n : 빠른 EMA 유효 표본수 (감지용)
    tau_slow_n : 느린 EMA 유효 표본수 (예측용 — 스펙에 들어가는 값)
    arm_n      : 감지 진입에 필요한 '연속' 초과 표본수 (단발 스파이크 방어)
    hold_n     : 해제 전 연속으로 깨끗해야 하는 표본수
    """

    baseline_s: float = 0.017
    trigger_s: float = 0.040
    tau_fast_n: int = 8
    tau_slow_n: int = 60
    arm_n: int = 3
    hold_n: int = 30

    ema_fast: float = field(default=0.0, init=False)
    ema_slow: float = field(default=0.0, init=False)
    n: int = field(default=0, init=False)
    detected: bool = field(default=False, init=False)
    clean_run: int = field(default=0, init=False)
    over_run: int = field(default=0, init=False)
    peak_s: float = field(default=0.0, init=False)

    def reset(self) -> None:
        self.ema_fast = self.ema_slow = 0.0
        self.n = 0
        self.detected = False
        self.clean_run = 0
        self.over_run = 0
        self.peak_s = 0.0

    def update(self, sample_s: float) -> float:
        """표본 하나 반영하고 **예측 지연**(느린 EMA)을 돌려준다."""
        x = max(float(sample_s), 0.0)
        self.n += 1
        self.peak_s = max(self.peak_s, x)
        if self.n == 1:
            self.ema_fast = self.ema_slow = x
        else:
            af = 2.0 / (self.tau_fast_n + 1.0)
            as_ = 2.0 / (self.tau_slow_n + 1.0)
            self.ema_fast += af * (x - self.ema_fast)
            self.ema_slow += as_ * (x - self.ema_slow)

        # 감지 진입은 '연속 초과' 를 요구한다 — 단발 스파이크(스케줄러 지터, 파일
        # 잠금) 하나로 스펙을 깎으면 순항 속도가 계속 요동친다.
        if x > self.trigger_s:
            self.over_run += 1
        else:
            self.over_run = 0

        if self.ema_fast > self.trigger_s and self.over_run >= self.arm_n:
            self.detected = True
            self.clean_run = 0
        elif self.detected:
            # 회복까지 유지 — 빠른 EMA 가 baseline 아래로 hold_n 표본 연속 유지돼야 해제
            if self.ema_fast <= self.baseline_s:
                self.clean_run += 1
                if self.clean_run >= self.hold_n:
                    self.detected = False
                    self.clean_run = 0
            else:
                self.clean_run = 0
        return self.ema_slow

    @property
    def predicted_s(self) -> float:
        """스펙 감쇄에 쓸 지연. 감지 중이면 느린 EMA, 아니면 0(무보정)."""
        return self.ema_slow if self.detected else 0.0

    def snapshot(self) -> dict:
        """capability.json `observed.latency` 에 그대로 넣을 블록."""
        return {
            "samples": self.n,
            "ema_fast_s": round(self.ema_fast, 5),
            "ema_slow_s": round(self.ema_slow, 5),
            "peak_s": round(self.peak_s, 5),
            "detected": self.detected,
            "predicted_s": round(self.predicted_s, 5),
        }
