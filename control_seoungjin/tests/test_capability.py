"""실시간 능력 표 + 지연 추적기 테스트 (2026-08-22)."""
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from capability import (  # noqa: E402
    build_capability, scale_from_rho, v_cap_from_latency, write_capability, S_MIN,
)
from latency_tracker import LatencyTracker  # noqa: E402
from compute_load import LoadEstimator, LoadGovernor, TaskCost  # noqa: E402


class TestScale:
    def test_no_disturbance_is_full_scale(self):
        assert scale_from_rho(0.0) == pytest.approx(1.0)

    def test_monotone_decreasing(self):
        prev = 1.1
        for r in (0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0):
            s = scale_from_rho(r)
            assert s <= prev + 1e-12
            prev = s

    def test_saturates_at_s_min(self):
        assert scale_from_rho(0.95) == pytest.approx(S_MIN)
        assert scale_from_rho(1.0) == pytest.approx(S_MIN)

    def test_clamped_outside_unit_range(self):
        assert scale_from_rho(-5.0) == pytest.approx(1.0)
        assert scale_from_rho(5.0) == pytest.approx(S_MIN)


class TestLimits:
    def test_nominal_1kg_matches_capability_card(self):
        c = build_capability(pkg_kg=1.0)
        assert c["limits"]["v"] == pytest.approx(1.6 * 0.75)     # precision limit_scale
        assert c["degraded"]["active"] is False

    def test_mass_interpolation_between_anchors(self):
        c0 = build_capability(pkg_kg=0.0)
        c1 = build_capability(pkg_kg=1.0)
        ch = build_capability(pkg_kg=0.5)
        for k in ("v", "a"):
            lo, hi, mid = c0["limits"][k], c1["limits"][k], ch["limits"][k]
            assert min(lo, hi) <= mid <= max(lo, hi)
            assert mid == pytest.approx(0.5 * (lo + hi), rel=1e-6)

    def test_mass_clamped_outside_anchor_range(self):
        assert build_capability(pkg_kg=-1.0)["limits"] == build_capability(pkg_kg=0.0)["limits"]
        assert build_capability(pkg_kg=9.0)["limits"] == build_capability(pkg_kg=2.0)["limits"]

    def test_time_scale_powers(self):
        """감쇄는 시계 배율 s 하나로: v∝s, a∝s², j∝s³, snap∝s⁴."""
        base = build_capability(pkg_kg=1.0)
        deg = build_capability(pkg_kg=1.0, rho=0.45)
        s = deg["degraded"]["time_scale"]
        assert 0.0 < s < 1.0
        for k, p in (("v", 1), ("a", 2), ("j", 3), ("snap", 4)):
            assert deg["limits"][k] == pytest.approx(base["limits"][k] * s ** p, rel=1e-4)

    def test_agile_is_looser_than_precision(self):
        p = build_capability(pkg_kg=1.0, profile="precision")["limits"]
        a = build_capability(pkg_kg=1.0, profile="agile")["limits"]
        for k in ("v", "a", "j", "snap"):
            assert a[k] > p[k]

    def test_unknown_profile_raises(self):
        with pytest.raises(ValueError):
            build_capability(pkg_kg=1.0, profile="nope")


class TestYawErrorHold:
    def test_yaw_error_alone_degrades(self):
        """외란이 멎어도(rho=0) yaw 가 틀어져 있으면 스펙을 되돌리지 않는다."""
        c = build_capability(pkg_kg=1.0, rho=0.0, yaw_err_rad=math.radians(30))
        assert c["degraded"]["active"] is True
        assert c["degraded"]["time_scale"] < 1.0
        assert "disturbance" in c["degraded"]["reasons"]

    def test_recovered_yaw_restores_full_spec(self):
        c = build_capability(pkg_kg=1.0, rho=0.0, yaw_err_rad=math.radians(0.5))
        assert c["degraded"]["time_scale"] == pytest.approx(1.0, abs=0.02)


class TestLatency:
    def test_zero_latency_is_unbounded(self):
        assert v_cap_from_latency(0.0, 0.04) == math.inf

    def test_v_cap_shrinks_with_latency(self):
        assert v_cap_from_latency(0.10, 0.04) < v_cap_from_latency(0.02, 0.04)

    def test_latency_binds_and_is_reported(self):
        c = build_capability(pkg_kg=1.0, latency_s=0.5)   # 0.5 s -> v ≤ 0.5·0.04/0.5 = 0.04
        # 지연은 이제 **배율 하나**로 반영된다 (v 만 자르면 a/j/snap 이 안 따라와
        # "느린데 급격한" 궤적이 나온다 — capability.latency_track_scale 주석).
        assert c["limits"]["v"] == pytest.approx(0.04, rel=1e-6)
        # degraded.time_scale 은 4자리로 반올림돼 나가므로 재계산 기준으로 쓰면 안 된다.
        # v 에서 역산한 배율로 본다 (v = v0·ls·s 이므로 s = v/(v0·ls)).
        s = c["limits"]["v"] / (1.6 * 0.75)
        assert c["limits"]["a"] == pytest.approx(1.6 * 0.75 * s ** 2, rel=1e-3)
        assert c["limits"]["j"] == pytest.approx(8.0 * 0.75 * s ** 3, rel=1e-3)
        assert "latency_severe" in c["degraded"]["reasons"]
        assert "latency" in c["degraded"]["reasons"]
        assert c["degraded"]["active"] is True

    def test_small_latency_does_not_bind(self):
        c = build_capability(pkg_kg=1.0, latency_s=0.005)
        assert "latency" not in c["degraded"]["reasons"]


class TestTracker:
    def test_quiet_stream_never_detects(self):
        tr = LatencyTracker()
        for _ in range(200):
            tr.update(0.015)
        assert tr.detected is False
        assert tr.predicted_s == 0.0

    def test_sustained_delay_detected_and_predicted(self):
        tr = LatencyTracker()
        for _ in range(200):
            tr.update(0.12)
        assert tr.detected is True
        assert tr.predicted_s == pytest.approx(0.12, rel=0.05)

    def test_single_spike_does_not_trip(self):
        """단발 스파이크로 스펙을 깎으면 안 된다 (빠른 EMA 가 흡수)."""
        tr = LatencyTracker()
        for _ in range(100):
            tr.update(0.010)
        tr.update(0.5)
        assert tr.detected is False

    def test_hold_until_recovered(self):
        tr = LatencyTracker(hold_n=30)
        for _ in range(100):
            tr.update(0.15)
        assert tr.detected is True
        for _ in range(5):          # 잠깐 깨끗해져도 즉시 해제 안 됨
            tr.update(0.001)
        assert tr.detected is True
        for _ in range(200):
            tr.update(0.001)
        assert tr.detected is False

    def test_snapshot_shape(self):
        tr = LatencyTracker()
        tr.update(0.02)
        s = tr.snapshot()
        assert set(s) == {"samples", "ema_fast_s", "ema_slow_s", "peak_s",
                          "detected", "predicted_s"}

    def test_reset(self):
        tr = LatencyTracker()
        for _ in range(50):
            tr.update(0.2)
        tr.reset()
        assert tr.n == 0 and tr.detected is False and tr.peak_s == 0.0


class TestSerialization:
    def test_json_round_trip(self, tmp_path):
        cap = build_capability(pkg_kg=0.5, rho=0.2, latency_s=0.03)
        p = write_capability(cap, tmp_path / "capability.json")
        assert p.exists()
        back = json.loads(p.read_text(encoding="utf-8"))
        assert back == cap

    def test_required_top_level_keys(self):
        cap = build_capability(pkg_kg=1.0)
        for k in ("schema_version", "timestamp", "basis", "limits", "budget",
                  "yaw", "observed", "degraded", "guarantees", "valid_for_s"):
            assert k in cap

    def test_limits_are_plain_floats(self):
        """상위가 json 으로 그대로 먹을 수 있어야 한다 (numpy 타입 금지)."""
        cap = build_capability(pkg_kg=1.0, rho=0.3)
        for v in cap["limits"].values():
            assert type(v) is float


class TestEndToEnd:
    def test_tracker_feeds_capability(self):
        tr = LatencyTracker()
        for _ in range(200):
            tr.update(0.20)
        cap = build_capability(pkg_kg=1.0, rho=0.0, latency_s=tr.predicted_s)
        assert "latency" in cap["degraded"]["reasons"]
        assert cap["limits"]["v"] == pytest.approx(0.5 * 0.04 / tr.predicted_s, rel=0.05)
        # 실측표 배율을 밖에서 넘기면 그쪽이 이긴다 (잰 구간에서는 실측이 진실)
        cap2 = build_capability(pkg_kg=1.0, rho=0.0, latency_s=tr.predicted_s,
                                latency_scale=1.0)
        assert cap2["limits"]["v"] == pytest.approx(1.6 * 0.75, rel=1e-6)
        assert cap2["observed"]["scale_latency_source"] == "measured_table"

    def test_worst_case_exhausts_the_budget(self):
        """전부 나쁘면 한계가 0 이 된다 — 그게 맞는 답이다.

        2026-08-23 이전에는 "0 이 되면 안 된다"고 시험했다. 배율을 min 으로 합치던
        때의 규약이다. 지금은 **여유라는 하나의 자원을 속도·외란·지연이 나눠 쓰는**
        모델이고(사용자 정의), 깎인 양을 더한다. 셋이 다 먹으면 속도 몫이 0 이 되는
        것이 물리적으로 옳은 결론이고, 상위는 그걸 "이동 계획을 내지 말라"로 읽는다.
        0 을 억지로 바닥으로 올리면 **없는 여유를 있다고 보고하는 것**이 된다.
        """
        cap = build_capability(pkg_kg=0.0, rho=1.0, latency_s=0.3,
                               yaw_err_rad=math.radians(90))
        assert cap["limits"]["v"] == 0.0
        assert cap["degraded"]["active"] is True
        assert "latency_severe" in cap["degraded"]["reasons"]

    def test_moderate_case_stays_positive(self):
        """반대로, 적당히 나쁜 조건에서는 여유가 남아야 한다 (전부 0 으로 만들면 무용)."""
        cap = build_capability(pkg_kg=1.0, rho=0.2, latency_s=0.0)
        for k in ("v", "a", "j", "snap"):
            assert cap["limits"][k] > 0.0


class TestComputeLoad:
    def test_idle_is_zero_duty(self):
        assert LoadEstimator().duty() == 0.0
        assert LoadEstimator().predicted_latency_s() == 0.0

    def test_duty_grows_with_rate_and_size(self):
        e = LoadEstimator()
        e.set_task("smoother", 1000, 1.0)
        d1 = e.duty()
        e.set_task("smoother", 1000, 2.0)
        assert e.duty() > d1
        e.set_task("smoother", 4000, 2.0)
        assert e.duty() > 2 * d1

    def test_latency_blows_up_near_saturation(self):
        """duty -> 1 에서 발산해야 '조금만 더 얹으면 갑자기 늦어진다' 가 재현된다."""
        def lat(rate):
            e = LoadEstimator()
            e.set_task("smoother", 1000, rate)
            return e.predicted_latency_s(), e.duty()
        l_lo, d_lo = lat(1.0)      # duty ~0.025
        l_mid, d_mid = lat(20.0)   # duty ~0.50
        l_hi, d_hi = lat(35.0)     # duty ~0.88
        assert d_lo < d_mid < d_hi
        assert l_lo < l_mid < l_hi
        # 초선형: duty 가 0.5 -> 0.88 (1.8배) 인데 지연은 3배 이상
        assert (l_hi / l_mid) > 2.5 * (d_hi / d_mid) / 1.8

    def test_duty_capped_but_raw_reported(self):
        e = LoadEstimator()
        e.set_task("smoother", 4000, 50.0)           # 물리적으로 불가능한 부하
        assert e.duty() <= 0.95
        assert e.raw_duty() > 1.0
        assert e.snapshot()["saturated"] is True

    def test_observe_recalibrates_cost(self):
        c = TaskCost(0.0, 1e-6)
        for n, t in ((100, 0.010), (200, 0.020), (400, 0.040)):
            c.observe(n, t)
        assert c.per_unit_s == pytest.approx(1e-4, rel=1e-3)
        assert c.predict(300) == pytest.approx(0.030, rel=1e-3)

    def test_observe_same_size_only_shifts_fixed(self):
        c = TaskCost(0.0, 1e-4)
        for _ in range(5):
            c.observe(100, 0.020)                    # 예상 0.010, 실측 0.020
        assert c.per_unit_s == pytest.approx(1e-4)
        assert c.predict(100) == pytest.approx(0.020, rel=1e-6)

    def test_horizon_for_budget_shrinks_with_budget(self):
        from compute_load import horizon_for_budget
        e = LoadEstimator()
        assert horizon_for_budget(e, 0.100) > horizon_for_budget(e, 0.020)


class TestLoadGovernor:
    def test_fuse_takes_worse_of_model_and_measured(self):
        g = LoadGovernor()
        assert g.fuse(0.010, 0.050) == pytest.approx(0.050)
        assert g.source == "measured"
        assert g.fuse(0.080, 0.050) == pytest.approx(0.080)
        assert g.source == "model"

    def test_bias_reveals_model_underprediction(self):
        g = LoadGovernor()
        for _ in range(50):
            g.fuse(0.010, 0.030)
        assert g.bias_s == pytest.approx(0.020, rel=0.1)

    def test_rise_is_immediate(self):
        g = LoadGovernor()
        g.update(0.005, 0.005)
        a = g.update(0.200, 0.0)
        assert a == pytest.approx(0.200)

    def test_fall_waits_then_decays(self):
        g = LoadGovernor(hold_n=5, fall_tau_s=1.0)
        g.update(0.200, 0.0)
        for _ in range(4):                            # dwell 중에는 안 내려감
            assert g.update(0.010, 0.0) == pytest.approx(0.200)
        prev = g.applied_s
        for _ in range(10):
            cur = g.update(0.010, 0.0, dt=0.2)
            assert cur < prev + 1e-12
            prev = cur
        assert g.restoring is True
        for _ in range(200):
            g.update(0.010, 0.0, dt=0.2)
        assert g.applied_s == pytest.approx(0.010, abs=1e-4)

    def test_reset(self):
        g = LoadGovernor()
        g.update(0.5, 0.5)
        g.reset()
        assert g.applied_s == 0.0 and g.bias_s == 0.0

    def test_snapshot_keys(self):
        g = LoadGovernor()
        g.update(0.01, 0.02)
        assert set(g.snapshot()) == {"applied_latency_s", "source", "model_bias_s",
                                     "restoring", "low_run"}


class TestLoadIntoCapability:
    def test_saturated_load_is_reported(self):
        e = LoadEstimator()
        e.set_task("smoother", 4000, 50.0)
        cap = build_capability(pkg_kg=1.0, latency_s=0.2, load=e.snapshot())
        assert "load_saturated" in cap["degraded"]["reasons"]
        assert cap["observed"]["load"]["saturated"] is True

    def test_light_load_does_not_degrade(self):
        e = LoadEstimator()
        e.set_task("smoother", 500, 1.0)
        cap = build_capability(pkg_kg=1.0, latency_s=e.predicted_latency_s(),
                               load=e.snapshot())
        assert cap["degraded"]["reasons"] == []

    def test_end_to_end_model_plus_measurement(self):
        """부하 모델(예상) + 실측 -> 조속 -> capability limits 감쇄, 그리고 복귀."""
        e = LoadEstimator()
        e.set_task("smoother", 2000, 2.0)
        g = LoadGovernor(hold_n=3, fall_tau_s=0.5)
        for _ in range(5):
            g.update(e.predicted_latency_s(), 0.150)      # 실측이 훨씬 나쁨
        busy = build_capability(pkg_kg=1.0, latency_s=g.applied_s, load=e.snapshot())
        assert g.snapshot()["source"] == "measured"
        assert "latency" in busy["degraded"]["reasons"]

        e.set_task("smoother", 500, 1.0)                  # 부하 감소
        for _ in range(300):
            g.update(e.predicted_latency_s(), 0.002, dt=0.2)
        calm = build_capability(pkg_kg=1.0, latency_s=g.applied_s, load=e.snapshot())
        assert calm["limits"]["v"] > busy["limits"]["v"]  # 다시 향상됐다
        assert calm["degraded"]["reasons"] == []
