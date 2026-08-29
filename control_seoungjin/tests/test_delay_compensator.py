"""측정 지연 보상 예측기 시험.

두 가지를 본다:
  ① 보상이 실제로 뒤처짐을 줄이는가 (안 줄이면 존재 이유가 없다)
  ② 모델이 틀렸을 때 **발산하지 않는가** (이게 더 중요하다 — 보상은 모델 오차를
     새 오차로 집어넣는 거래라, 손해가 유계인지가 채택 조건이다)

기준선은 `enabled=False` 다. 그때 예측기는 측정을 그대로 돌려주므로 곧 무보상이다.
"""
from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from delay_compensator import (  # noqa: E402
    AxisPredictor,
    DelayCompensator,
    PredictorConfig,
    lateral_accel_from_attitude,
)

DT = 0.001
MEAS_HZ = 30.0          # VIO 갱신률 (§8c T8 권장 30 Hz)


def simulate(tau_s, enabled, accel_fn, v0=0.0, T=3.0,
             accel_scale=1.0, obs_w=12.0, every_n=10):
    """진실 궤적 vs 지연 측정 vs 보상 결과. 반환 (정착 구간 RMS 오차, 예측기).

    accel_scale 로 예측기에 넘기는 가속도를 일부러 틀리게 만든다 (모델 오차 모사).
    """
    cfg = PredictorConfig(dt=DT, enabled=enabled, obs_w=obs_w, every_n=every_n)
    pred = AxisPredictor(cfg=cfg)
    pred.reset(0.0)
    pred.correct(0.0, 0.0)   # priming

    p, v, t = 0.0, v0, 0.0
    truth_hist = [0.0]
    lag_steps = int(round(tau_s / DT))
    meas_every = max(1, int(round((1.0 / MEAS_HZ) / DT)))
    n = int(T / DT)
    errs = []
    for k in range(n):
        a = accel_fn(t)
        p += v * DT + 0.5 * a * DT * DT
        v += a * DT
        t += DT
        truth_hist.append(p)

        meas = None
        if k % meas_every == 0:
            idx = len(truth_hist) - 1 - lag_steps
            if idx >= 0:
                meas = truth_hist[idx]
        out = pred.tick(a * accel_scale, meas, tau_s, DT)
        if t > 1.0:                      # 초기 정렬 구간은 뺀다
            errs.append(out - p)
    rms = math.sqrt(sum(e * e for e in errs) / max(1, len(errs)))
    return rms, pred


class TestIdentityWhenOff:
    def test_disabled_returns_measurement(self):
        """꺼져 있으면 측정을 그대로 돌려준다 — 골든 트레이스 불변의 근거."""
        pred = AxisPredictor(cfg=PredictorConfig(dt=DT, enabled=False))
        pred.reset(0.0)
        pred.correct(0.0, 0.0)           # priming
        for m in (0.10, 0.25, -0.4):
            pred.step(3.0, DT)
            pred.correct(m, 0.05)
            assert pred.position == pytest.approx(m)

    def test_default_is_off(self):
        """기본값이 켜져 있으면 안 된다 — 추정기가 이미 전파하면 이중 보상이다."""
        assert PredictorConfig().enabled is False


class TestLagRemoval:
    def test_constant_velocity_lag_removed(self):
        """등속 이동의 무보상 뒤처짐은 v·(τ + T/2) 다. 보상은 그걸 지워야 한다.

        τ 만이 아닌 이유: 측정이 30 Hz 라 그 사이(T=33 ms)에는 값이 얼어 있다.
        무보상 제어기가 실제로 겪는 것은 **전달지연 + 그 굳음** 둘 다이고,
        예측기는 둘 다 지운다 (사이 구간을 속도로 채우므로).

        가속도가 0 이라 모델이 알려 주는 게 없다 — 속도는 순전히 혁신에서 배운다.
        알파-베타의 beta 항이 일하는지 보는 시험이다.
        """
        tau, v = 0.06, 1.0
        off, _ = simulate(tau, False, lambda t: 0.0, v0=v)
        on, _ = simulate(tau, True, lambda t: 0.0, v0=v)
        expect = v * (tau + 0.5 / MEAS_HZ)
        assert off == pytest.approx(expect, rel=0.15)
        assert on < off * 0.2                               # 최소 5 배 개선

    def test_accelerating_motion(self):
        """가속 구간에서도 이득이 있어야 한다 (여기서는 모델이 도와준다)."""
        tau = 0.06
        off, _ = simulate(tau, False, lambda t: 0.8 * math.sin(2 * math.pi * 0.5 * t))
        on, _ = simulate(tau, True, lambda t: 0.8 * math.sin(2 * math.pi * 0.5 * t))
        assert on < off * 0.5

    def test_gain_grows_with_delay(self):
        """지연이 클수록 보상이 벌어들이는 양도 커야 한다 (그게 존재 이유다)."""
        ratios = []
        for tau in (0.02, 0.06, 0.10):
            off, _ = simulate(tau, False, lambda t: 0.0, v0=1.0)
            on, _ = simulate(tau, True, lambda t: 0.0, v0=1.0)
            ratios.append(off - on)
        assert ratios == sorted(ratios)


class TestFailsSafe:
    def test_stale_measurement_falls_back(self):
        """max_age 를 넘으면 보상을 포기하고 측정을 그대로 쓴다.

        모델 오차는 τ² 로 커진다. 오래된 측정에 보상을 밀어붙이면 벌이보다 손해가
        커지므로, 그 지점에서는 **아무것도 안 하는 것**이 옳다.
        """
        cfg = PredictorConfig(dt=DT, enabled=True, max_age_s=0.05)
        pred = AxisPredictor(cfg=cfg)
        pred.reset(0.0)
        pred.correct(0.0, 0.0)
        for _ in range(50):
            pred.step(1.0, DT)
        pred.correct(0.3, 0.2)                # 200 ms — 한도 초과
        assert pred.n_fallback == 1
        assert pred.position == pytest.approx(0.3)

    def test_history_too_short_falls_back(self):
        """이력이 그 시각까지 없으면(막 켰을 때 등) 보상하지 않는다."""
        cfg = PredictorConfig(dt=DT, enabled=True, hist_s=0.01)
        pred = AxisPredictor(cfg=cfg)
        pred.reset(0.0)
        pred.correct(0.0, 0.0)
        for _ in range(100):
            pred.step(1.0, DT)
        pred.correct(0.05, 0.08)              # 이력(10 ms)보다 오래된 시각
        assert pred.n_fallback == 1

    def test_model_error_stays_bounded(self):
        """가속도 모델이 30% 틀려도 발산하지 않는다.

        채택 조건이 "완벽하면 좋다" 가 아니라 "틀려도 손해가 유계다" 인 이유:
        실기에서 모델은 반드시 틀린다 (질량 오차, 항력, 바람).
        """
        tau = 0.06
        acc = lambda t: 0.8 * math.sin(2 * math.pi * 0.5 * t)   # noqa: E731
        off, _ = simulate(tau, False, acc)
        bad, pred = simulate(tau, True, acc, accel_scale=1.3)
        assert math.isfinite(bad)
        assert bad < off                       # 30% 틀려도 무보상보다는 낫다
        assert abs(pred.velocity) < 100.0      # 상태가 튀지 않았다

    def test_dt_is_clamped(self):
        """이상한 dt 로 적분하면 상태가 튄다 — SwingDamper 규약과 같은 방어."""
        pred = AxisPredictor(cfg=PredictorConfig(dt=DT, enabled=True))
        pred.reset(0.0)
        pred.step(1.0, 10.0)                  # 10 초짜리 스텝 (있을 수 없는 값)
        assert abs(pred.position) < 1.0       # DT_MAX_S 로 잘렸다


class TestAlphaBeta:
    def test_dimensions(self):
        """beta 를 T 로 나누지 않으면 측정 주기가 바뀔 때 실효 이득이 달라진다.

        같은 물리 상황(등속)에서 측정률만 30 Hz -> 100 Hz 로 바꿔도 정상상태
        속도 추정이 비슷해야 한다.
        """
        cfg = PredictorConfig(obs_w=12.0)
        a30, b30 = cfg.alpha_beta(1 / 30.0)
        a100, b100 = cfg.alpha_beta(1 / 100.0)
        assert 0.0 < a100 < a30 < 1.0          # 자주 재면 한 번에 덜 반영
        assert b30 > b100                       # beta/T 는 주기가 길수록 크다

    def test_zero_bandwidth_is_no_correction(self):
        cfg = PredictorConfig(obs_w=0.0)
        alpha, beta_over_t = cfg.alpha_beta(1 / 30.0)
        assert alpha == pytest.approx(0.0)
        assert beta_over_t == pytest.approx(0.0)


class TestHelpers:
    def test_lateral_accel_zero_attitude(self):
        assert lateral_accel_from_attitude(0.0, 0.0, 0.0) == (0.0, 0.0)

    def test_three_axis_wrapper(self):
        comp = DelayCompensator(cfg=PredictorConfig(dt=DT, enabled=True))
        comp.reset((1.0, 2.0, 3.0))
        comp.correct((1.0, 2.0, 3.0), 0.0)
        comp.step((0.0, 0.0, 0.0), DT)
        assert comp.position == pytest.approx((1.0, 2.0, 3.0))
        assert comp.n_fallback == 0


class TestDecimation:
    """무거운 쪽을 N 스텝마다 돌려도 출력은 현재 시각이어야 한다.

    사용자 설계 2026-08-28: "이거 돌리는 것도 지연이니까." 그런데 사이 구간에
    출력을 붙잡아 두면(ZOH) 최대 N·dt 의 새 지연이 생겨, 지연을 줄이자고 만든
    물건이 지연을 만든다. 그래서 사이는 속도로 외삽한다.
    """

    def test_output_advances_between_heavy_runs(self):
        cfg = PredictorConfig(dt=DT, enabled=True, every_n=10)
        pred = AxisPredictor(cfg=cfg)
        pred.reset(0.0)
        pred.correct(0.0, 0.0)
        pred.velocity = 1.0
        outs = [pred.tick(0.0, None, 0.0, DT) for _ in range(9)]
        assert outs == sorted(outs)
        assert outs[-1] > outs[0]          # ZOH 였다면 전부 같았을 것

    def test_decimation_costs_little_accuracy(self):
        """N=1 과 N=10 의 정확도 차이가 크지 않아야 데시메이션이 정당하다."""
        tau = 0.06
        acc = lambda t: 0.8 * math.sin(2 * math.pi * 0.5 * t)   # noqa: E731
        e1, _ = simulate(tau, True, acc, every_n=1)
        e10, _ = simulate(tau, True, acc, every_n=10)
        off, _ = simulate(tau, False, acc)
        assert e10 < off * 0.6             # 데시메이션해도 이득은 남는다
        assert e10 < e1 * 3.0              # 그리고 매 스텝 대비 크게 나빠지지 않는다

    def test_only_latest_measurement_kept(self):
        """차례를 기다리는 동안 측정이 여러 번 오면 마지막 것만 쓴다."""
        cfg = PredictorConfig(dt=DT, enabled=True, every_n=50)
        pred = AxisPredictor(cfg=cfg)
        pred.reset(0.0)
        pred.correct(0.0, 0.0)
        for _ in range(200):
            pred.step(0.0, DT)
        for m in (0.1, 0.2, 0.3):
            pred.tick(0.0, m, 0.05, DT)
        for _ in range(60):
            pred.tick(0.0, None, 0.0, DT)
        assert pred.position == pytest.approx(0.3, abs=0.2)
