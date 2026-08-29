"""spec_governor — 지연 -> 상위 보고 스펙 조속기 시험.

지키려는 성질
  1. 자세 경로 지연은 **게이트** — 한계 초과 시 임무 거부 (한계값도 0)
  2. 위치 경로 지연은 **감쇄** — 배율 하나로 v/a/j/snap 이 함께 내려간다
  3. 비대칭: 부하 증가는 즉시 반영, 감소는 확인 후 천천히 복귀
  4. 히스테리시스: 미세 변동으로 상위에 재계획을 쏟아붓지 않는다
  5. 표 밖 지연은 외삽하지 않고 플래그를 세운다
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import capability as cap  # noqa: E402
from recovery_watcher import RecoveryWatcher  # noqa: E402
from spec_governor import (  # noqa: E402
    REPUBLISH_EPS,
    SpecGovernor,
    att_delay_verdict,
    scale_from_latency_pos,
)


class TestRecoveryWatcher:
    """표가 틀렸을 때 폐루프로 교정 (사용자 설계 08-23)."""

    def _run(self, w, err, secs, dt=0.01, lead=2.0, ref_ok=True):
        for _ in range(int(secs / dt)):
            w.observe(err, ref_ok, dt)
            w.decide(lead)
        return w.s

    def test_quiet_never_cuts(self):
        w = RecoveryWatcher()
        assert self._run(w, 0.01, 30.0) == 1.0
        assert w.cuts == 0

    def test_slow_recovery_cuts_gradually(self):
        """한 판단에 한 걸음씩. 한 번에 바닥으로 떨어지면 다리가 따라올 수 없다."""
        w = RecoveryWatcher()
        seen = []
        for _ in range(int(30.0 / 0.01)):
            w.observe(0.09, True, 0.01)
            before = w.s
            if w.decide(2.0) != before:
                seen.append(round(w.s, 3))
        assert len(seen) >= 3, f"걸음이 너무 적다: {seen}"
        assert all(a > b for a, b in zip(seen, seen[1:])), f"단조 감소 아님: {seen}"
        assert max(a - b for a, b in zip([1.0] + seen, seen)) <= w.max_cut + 1e-9

    def test_restores_after_clean(self):
        w = RecoveryWatcher()
        self._run(w, 0.09, 25.0)
        low = w.s
        assert low < 1.0
        assert self._run(w, 0.01, 60.0) == pytest.approx(1.0, abs=1e-3)
        assert w.s > low

    def test_period_never_below_bridge_convergence(self):
        """판단 주기 < 다리 수렴 시간이면, 앞 결정이 반영되기 전에 또 결정한다 = 발진."""
        w = RecoveryWatcher(min_period_s=1.0)
        assert w.period_s(4.0) >= 4.0
        assert w.period_s(None) == 1.0
        assert w.period_s(float("nan")) == 1.0

    def test_reference_over_limits_is_not_charged(self):
        """기준이 한계 밖이면 계획 문제다 — 그걸로 스펙을 깎으면 안 된다."""
        w = RecoveryWatcher()
        assert self._run(w, 0.30, 30.0, ref_ok=False) == 1.0
        assert w.cuts == 0 and w.n_skipped > 0

    def test_floor_is_slow_not_stop(self):
        """감시는 '느리게' 까지만. 정지 판단은 감독자(§9) 몫이다."""
        w = RecoveryWatcher()
        assert self._run(w, 1.0, 200.0) >= 0.15 - 1e-9


def gov(**kw):
    kw.setdefault("write", False)
    return SpecGovernor(**kw)


class TestAttitudeGate:
    def test_clean_region_no_derate(self):
        for att in (0.0, 0.005, cap.LAT_ATT_CLEAN_S):
            s, reason = att_delay_verdict(att)
            assert s == 1.0 and reason is None

    def test_margin_region(self):
        s, reason = att_delay_verdict((cap.LAT_ATT_CLEAN_S + cap.LAT_ATT_MAX_S) / 2)
        assert s == cap.LAT_ATT_MARGIN_SCALE
        assert reason == "att_latency_margin"

    def test_unflyable_refuses_mission(self):
        """20 ms 실측 호버 RMS 2.44° — 느리게 가도 안 고쳐지므로 거부해야 한다."""
        g = gov(latency_att_s=cap.LAT_ATT_MAX_S + 0.004)
        out = g.tick()
        assert out["mission_allowed"] is False
        c = out["capability"]
        assert "att_latency_unflyable" in c["degraded"]["reasons"]
        # 플래그만 두면 상위가 놓친다 — 값으로도 막는다
        assert all(v == 0.0 for v in c["limits"].values())
        assert c["degraded"]["time_scale"] == 0.0

    def test_margin_scales_all_axes(self):
        g = gov(latency_att_s=(cap.LAT_ATT_CLEAN_S + cap.LAT_ATT_MAX_S) / 2)
        c = g.tick()["capability"]
        s = c["degraded"]["time_scale"]
        assert s == pytest.approx(cap.LAT_ATT_MARGIN_SCALE, abs=1e-6)
        base = cap._ANCHORS[1.0]
        ls = cap._PROFILE["precision"]["limit_scale"]
        for key, p in (("v", 1), ("a", 2), ("j", 3), ("snap", 4)):
            assert c["limits"][key] == pytest.approx(base["limits"][key] * ls * s ** p,
                                                     rel=1e-3)


class TestPositionDerate:
    def test_table_lookup_endpoints(self):
        keys = sorted(cap._LAT_POS_ANCHORS)
        assert scale_from_latency_pos(0.0) == cap._LAT_POS_ANCHORS[keys[0]]
        assert scale_from_latency_pos(keys[-1]) == cap._LAT_POS_ANCHORS[keys[-1]]

    def test_no_extrapolation_beyond_table(self):
        """표 밖은 외삽하지 않는다 — 안 재본 구간의 추정치는 그 자체가 위험."""
        last = max(cap._LAT_POS_ANCHORS)
        assert scale_from_latency_pos(last + 1.0) == cap._LAT_POS_ANCHORS[last]

    def test_extrapolated_flag_set(self):
        g = gov()
        for _ in range(80):
            g.observe_latency(max(cap._LAT_POS_ANCHORS) + 0.05)
        c = g.tick()["capability"]
        assert c["degraded"]["latency_extrapolated"] is True
        # 표 밖에서는 해석 규칙(보수적)이 쓰인다
        assert c["observed"]["scale_latency_source"] == "analytic_track_budget"

    def test_monotone_in_table(self):
        keys = sorted(cap._LAT_POS_ANCHORS)
        xs = [keys[0] + (keys[-1] - keys[0]) * i / 10 for i in range(11)]
        ss = [scale_from_latency_pos(x) for x in xs]
        assert ss == sorted(ss, reverse=True), f"지연이 커지는데 배율이 올라감: {ss}"


class TestPositionUnflyable:
    def test_zero_anchor_refuses_mission(self):
        """실측표에 0.00 이 있으면 = 그 지연에서 어떤 배율로도 통과 못함 -> 임무 거부.

        '안 재봤다'와 '못 한다'는 다르다. 후자를 감쇄로 처리하면 상위가 아주 느린
        스펙을 받고 임무를 강행하게 된다.
        """
        saved = dict(cap._LAT_POS_ANCHORS)
        try:
            cap._LAT_POS_ANCHORS.clear()
            cap._LAT_POS_ANCHORS.update({0.0: 1.0, 0.05: 0.30, 0.09: 0.0})
            g = gov()
            for _ in range(120):
                g.observe_latency(0.095)
            out = g.tick()
            assert out["mission_allowed"] is False
            c = out["capability"]
            assert "pos_latency_unflyable" in c["degraded"]["reasons"]
            assert all(v == 0.0 for v in c["limits"].values())
        finally:
            cap._LAT_POS_ANCHORS.clear()
            cap._LAT_POS_ANCHORS.update(saved)


class TestAsymmetry:
    def test_rise_immediate_fall_slow(self):
        """부하 증가는 즉시, 감소는 확인 후 천천히 (경계 요동 -> 재계획 폭주 방지)."""
        g = gov()
        for _ in range(30):
            g.observe_latency(0.010)
            g.tick(dt=0.2)
        low = g.gov.applied_s

        for _ in range(6):
            g.observe_latency(0.090)
            g.tick(dt=0.2)
        high = g.gov.applied_s
        assert high > low + 0.03, "지연 급증이 즉시 반영되지 않음"

        g.observe_latency(0.010)
        g.tick(dt=0.2)
        assert g.gov.applied_s == pytest.approx(high, abs=1e-9), "한 표본에 바로 복귀했다"

        for _ in range(60):
            g.observe_latency(0.010)
            g.tick(dt=0.2)
        assert g.gov.applied_s < high, "확인 후에도 복귀하지 않음"


class TestHysteresis:
    def test_small_wobble_does_not_republish(self):
        g = gov()
        for _ in range(40):
            g.observe_latency(0.012)
            g.tick(dt=0.2)
        n0 = g.republishes
        for k in range(40):
            g.observe_latency(0.012 + 0.0005 * (k % 2))
            g.tick(dt=0.2)
        assert g.republishes == n0, "미세 변동에 재발행"

    def test_real_change_republishes(self):
        g = gov()
        for _ in range(20):
            g.observe_latency(0.010)
            g.tick(dt=0.2)
        n0 = g.republishes
        fired = False
        for _ in range(10):
            g.observe_latency(0.30)          # 표 밖 -> 해석 규칙으로 크게 깎임
            if g.tick(dt=0.2)["replan_needed"]:
                fired = True
        assert g.republishes > n0 and fired

    def test_republish_threshold_matches_constant(self):
        g = gov()
        g.tick()
        s0 = g.last_scale
        g.observe_rho(0.02)
        g.tick()
        assert abs(g.last_scale - s0) < REPUBLISH_EPS


class TestBridgeHandoff:
    def test_bridge_uses_current_limits(self):
        import numpy as np

        from traj_bridge import _smoothstep7
        g = gov()
        for _ in range(20):
            g.observe_latency(0.25)          # 크게 깎이는 조건
            g.tick(dt=0.2)
        dt = 0.01
        tmove = 2.1875 * 6.0 / 1.6
        t = np.arange(0.0, tmove + 8.0 + dt, dt)
        base = np.column_stack([6.0 * _smoothstep7(t / tmove),
                                np.zeros_like(t), np.ones_like(t)])
        br = g.plan_bridge_for(t, base, t_now=tmove / 2)
        assert max(br.phys_use.values()) <= 1.0
        assert br.stopped
        # 예산이 하한 아래로 내려가지 않는다
        assert g.replan_budget_s() >= 0.10

    def test_bridge_before_tick_raises(self):
        import numpy as np
        g = gov()
        t = np.arange(0.0, 5.0, 0.01)
        base = np.column_stack([t * 0.1, t * 0, t * 0 + 1])
        with pytest.raises(RuntimeError):
            g.plan_bridge_for(t, base, t_now=1.0)


class TestReportShape:
    def test_fields_present(self):
        g = gov()
        c = g.tick()["capability"]
        for k in ("scale_sources", "mission_allowed", "latency_extrapolated"):
            assert k in c["degraded"], k
        for k in ("latency_att_s", "latency_pos_applied_s"):
            assert k in c["observed"], k
        assert set(c["degraded"]["scale_sources"]) == {
            "disturbance", "latency_pos", "latency_att", "recovery"}
        assert "recovery" in c["observed"]

    def test_disturbance_and_latency_take_min_not_product(self):
        """둘을 곱하면 겹칠 때 필요 이상으로 깎여 임무가 서지 않는다."""
        g = gov(latency_att_s=(cap.LAT_ATT_CLEAN_S + cap.LAT_ATT_MAX_S) / 2)
        g.observe_rho(0.45)
        c = g.tick()["capability"]
        src = c["degraded"]["scale_sources"]
        expect = min(src["disturbance"], src["latency_pos"], src["latency_att"])
        assert c["degraded"]["time_scale"] == pytest.approx(expect, abs=2e-3)
        prod = src["disturbance"] * src["latency_att"]
        assert c["degraded"]["time_scale"] > prod + 1e-6

    def test_yaw_error_counts_as_reserved_authority(self):
        g = gov()
        g.observe_rho(0.0, yaw_err_rad=math.radians(30))
        c = g.tick()["capability"]
        assert c["degraded"]["time_scale"] < 1.0
        assert "disturbance" in c["degraded"]["reasons"]


class TestLatencyMassAxis:
    """위치-지연 표의 질량 축 (0/1 kg 실측, 사이는 선형).

    2026-08-28. 그전까지 표는 1 kg 전용이었고, 0 kg 에 대해서는 보수적으로 틀렸다.
    0 kg 실측이 들어오면서 두 앵커가 생겼다 — 그런데 두 표가 크게 벌어져 있어
    (120 ms: 0 kg 0.55 vs 1 kg 운용 불가) 어떻게 잇느냐가 그냥 나오지 않는다.
    """

    def test_endpoints_reproduce_measured_tables(self):
        """앵커에서는 실측표를 정확히 재현해야 한다 — 보간이 끝점을 흔들면 안 된다."""
        for m in (0.0, 1.0):
            assert cap._lat_table_for_pkg(m) == cap._LAT_POS_ANCHORS_BY_PKG[m]

    def test_one_kg_default_unchanged(self):
        """호출부가 질량을 안 넘기면 예전과 같은 값 (1 kg 표)."""
        for tau in (0.0, 0.02, 0.04, 0.06, 0.08):
            assert (scale_from_latency_pos(tau)
                    == pytest.approx(cap._LAT_POS_ANCHORS[tau]))

    def test_monotone_in_mass(self):
        """무거울수록 지연에 약하다 — 80 ms 에서 0 kg 0.75 -> 1 kg 0.37."""
        ss = [scale_from_latency_pos(0.08, pkg_kg=m)
              for m in (0.0, 0.25, 0.5, 0.75, 1.0)]
        assert ss == sorted(ss, reverse=True)
        assert ss[0] == pytest.approx(0.75)
        assert ss[-1] == pytest.approx(0.37)

    def test_inoperable_is_absorbed_not_interpolated(self):
        """0.00 은 '작은 배율이면 된다' 가 아니라 '어떤 배율로도 안 된다' 다.

        1 kg 120 ms 는 실제로 더 깎을수록 나빠졌다 (0.55 에서 종단 6.3 cm 였다가
        0.40 에서 25 m 발산). 0.55 와 0.00 을 이어 0.275 를 내주면 상위는 그 배율이
        통과한다고 읽는데, 그 값은 아무도 재지 않았다.
        """
        for m in (0.01, 0.25, 0.5, 0.99):
            for tau in (0.120, 0.160):
                assert cap._lat_table_for_pkg(m)[tau] == 0.0
        # 0 kg 자신은 실측이므로 살아 있어야 한다
        assert cap._lat_table_for_pkg(0.0)[0.120] == pytest.approx(0.55)

    def test_out_of_range_clamps(self):
        """범위 밖은 외삽하지 않는다 — 안 재본 질량에 값을 지어내면 그게 사고다."""
        assert cap._lat_table_for_pkg(-1.0) == cap._LAT_POS_ANCHORS_BY_PKG[0.0]
        assert cap._lat_table_for_pkg(9.0) == cap._LAT_POS_ANCHORS_BY_PKG[1.0]

    def test_only_common_taus_emitted(self):
        """한쪽 질량에만 있는 지연 점은 내지 않는다 (그 질량에서도 쟀다고 읽힌다)."""
        common = set(cap._LAT_POS_ANCHORS_BY_PKG[0.0]) & set(cap._LAT_POS_ANCHORS_BY_PKG[1.0])
        assert set(cap._lat_table_for_pkg(0.5)) == common

    def test_gust_table_has_no_mass_axis_yet(self):
        """돌풍 표는 질량마다 **다른 복귀 게이트**로 쟀다 — 이으면 기준이 섞인다."""
        assert not hasattr(cap, "_LAT_POS_ANCHORS_GUST_BY_PKG")
        a = scale_from_latency_pos(0.06, gust=True, pkg_kg=0.0)
        b = scale_from_latency_pos(0.06, gust=True, pkg_kg=1.0)
        assert a == pytest.approx(b)


class TestDerateIneffective:
    """깎아도 안 나아지면 멈추고 알린다 (2026-08-28).

    이 감시기는 지금까지 "더디면 깎는다" 만 했고 **깎아서 나아졌는지는 안 봤다.**
    감쇄가 듣는 영역에서는 문제가 없지만, 실측에서 안 듣는 영역이 나왔다 —
    0 kg / 토크 0.3 N*m / 20 ms 에서 배율 1.00 -> 0.37 로 순항을 2.7배 낮췄는데
    복귀가 9.87 -> 9.79 s 로 꿈쩍도 안 했다 (이탈은 위치오차 클램프 포화가 정하므로
    속도와 무관하다). 그 영역에서 계속 깎으면 에너지만 1/s 로 늘고 회복은 그대로다.
    """

    @staticmethod
    def _watcher(**kw):
        from recovery_watcher import RecoveryWatcher
        return RecoveryWatcher(**kw)

    def _run(self, w, err, seconds, dt=0.01):
        for _ in range(int(seconds / dt)):
            w.observe(err, True, dt)
            w.decide(2.0)

    def test_stops_cutting_when_it_does_not_help(self):
        """오차가 배율과 무관하게 고정이면 = 깎아도 안 나아지는 경우."""
        w = self._watcher()
        self._run(w, 0.09, 120.0)            # 밴드(0.04) 위로 계속
        assert w.derate_ineffective is True
        assert w.futile_cuts >= w.futile_n
        # 바닥까지 안 갔다 — 동결됐다는 뜻
        from recovery_watcher import S_FLOOR
        assert w.s > S_FLOOR + 1e-9

    def test_reports_it_outward(self):
        """말없이 멈추면 안 된다 — 상위가 재계획/축소/착륙을 고를 수 있어야 한다."""
        w = self._watcher()
        self._run(w, 0.09, 120.0)
        snap = w.snapshot()
        assert snap["derate_ineffective"] is True
        assert snap["futile_cuts"] >= w.futile_n

    def test_keeps_cutting_while_it_helps(self):
        """나아지고 있는 동안에는 계속 깎아야 한다 (가드가 정상 동작을 막으면 안 된다).

        "깎기가 듣는다" 를 모사하려면 오차가 **배율에 연동**돼야 한다. 오차를 그냥
        조금씩 줄이면 밴드 위에 머무는 한 초과 시간이 계속 쌓여 ratio 가 오히려
        커진다 — 그건 듣는 상황이 아니라 안 듣는 상황이다.
        """
        w = self._watcher()
        dt = 0.01
        for _ in range(12000):
            err = 0.20 * w.s          # 깎으면 오차가 준다 = 감쇄가 듣는 영역
            w.observe(err, True, dt)
            w.decide(2.0)
        assert w.derate_ineffective is False
        assert w.cuts >= 3
        assert w.s < 1.0              # 실제로 깎았다

    def test_clears_after_recovery(self):
        """회복되면 무효 판정도 없던 일로 — 다음 에피소드는 다를 수 있다."""
        w = self._watcher()
        self._run(w, 0.09, 120.0)
        assert w.derate_ineffective is True
        self._run(w, 0.001, 60.0)            # 밴드 아래로 충분히
        assert w.derate_ineffective is False
        assert w.futile_cuts == 0
