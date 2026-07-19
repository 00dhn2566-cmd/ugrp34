"""command_fidelity 순수 계산 함수 테스트 (INTERFACE_SPEC §7 확정사항 3건).

mat 로더 래퍼는 MATLAB 왕복에서 검증 — 여기서는 배열 단위 로직만.
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from analyze_flight_log import (               # noqa: E402
    pointing_rms_deg,
    scan_coverage_actual,
    waypoint_hits_cm,
    zone_min_clearance_m,
)


class TestWaypointHits:
    def test_basic_hit_measured(self):
        t = np.linspace(0, 10, 1001)
        plan = np.column_stack([t * 0.4, np.zeros_like(t), np.ones_like(t)])
        act = plan + [0.0, 0.01, 0.0]          # 횡방향 1cm 상시 오프셋
        # (진행방향 오프셋은 같은 직선 위라 통과 오차 0 - 횡이어야 측정됨)
        hits = waypoint_hits_cm(t, act, t, plan, [[2.0, 0.0, 1.0]])
        assert hits[0] == pytest.approx(1.0, abs=0.1)

    def test_time_window_excludes_return_pass(self):
        """§7 확정 2: 왕복 경로에서 '돌아올 때 스친 것'을 통과로 오인 금지.

        계획: 0->4m 직진 (wp=[2,0,1]를 t=5s에 통과 예정).
        실제: 갈 때는 y=0.1m 비켜 가고(오차 10cm), t=15s에 같은 점을
        정확히 재통과. 시간창이 없으면 0cm로 속고, 있으면 10cm가 정답.
        """
        t_plan = np.linspace(0, 10, 1001)
        plan = np.column_stack([t_plan * 0.4, np.zeros_like(t_plan),
                                np.ones_like(t_plan)])
        t_act = np.linspace(0, 20, 2001)
        x = np.where(t_act <= 10, t_act * 0.4, 4.0 - (t_act - 10) * 0.4)
        y = np.where(t_act <= 10, 0.1, 0.0)    # 갈 때만 10cm 비킴
        act = np.column_stack([x, y, np.ones_like(t_act)])
        hits = waypoint_hits_cm(t_act, act, t_plan, plan, [[2.0, 0.0, 1.0]])
        assert hits[0] == pytest.approx(10.0, abs=0.5), \
            "복귀 통과(t=15s)를 집계했다면 시간창 미작동"

    def test_no_samples_in_window_is_none(self):
        t_plan = np.linspace(0, 10, 101)
        plan = np.column_stack([t_plan, np.zeros_like(t_plan),
                                np.ones_like(t_plan)])
        t_act = np.linspace(50, 60, 101)       # 창 밖 로그
        act = plan.copy()
        hits = waypoint_hits_cm(t_act, act, t_plan, plan, [[5.0, 0.0, 1.0]])
        assert hits[0] is None


class TestZoneClearance:
    def test_sphere_outside_positive(self):
        pos = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]])
        z = [{"shape": "sphere", "center": [3.0, 0.0, 1.0], "radius_m": 1.0}]
        c = zone_min_clearance_m(pos, z)
        assert c == pytest.approx(1.0, abs=1e-6)    # 최근접 1m

    def test_box_penetration_negative(self):
        pos = np.array([[0.5, 0.5, 1.0]])
        z = [{"shape": "box", "min": [0.0, 0.0, 0.0], "max": [1.0, 1.0, 2.0]}]
        c = zone_min_clearance_m(pos, z)
        assert c < 0, "구역 내부는 음수(침범 깊이)여야 함"

    def test_inflate_shrinks_clearance(self):
        pos = np.array([[0.0, 0.0, 1.0]])
        z = [{"shape": "sphere", "center": [2.0, 0.0, 1.0], "radius_m": 1.0}]
        assert (zone_min_clearance_m(pos, z, inflate_m=0.5)
                == pytest.approx(0.5, abs=1e-6))

    def test_no_zones_none(self):
        assert zone_min_clearance_m(np.zeros((3, 3)), None) is None


class TestPointingAndScan:
    def test_perfect_pointing_zero_rms(self):
        t = np.linspace(0, 5, 501)
        pos = np.column_stack([t, np.zeros_like(t), np.ones_like(t)])
        tgt = [10.0, 5.0, 1.0]
        yaw = np.arctan2(tgt[1] - pos[:, 1], tgt[0] - pos[:, 0])
        assert pointing_rms_deg(yaw, pos, tgt) == pytest.approx(0.0, abs=1e-6)

    def test_freeze_radius_excluded(self):
        """목표 관통 구간(동결 반경)의 발산 각도는 집계 제외."""
        pos = np.array([[0.0, 0.0, 1.0], [1.99, 0.0, 1.0], [4.0, 0.0, 1.0]])
        tgt = [2.0, 0.0, 1.0]
        yaw = np.array([0.0, 2.0, np.pi])      # 근접 샘플의 엉터리 각 2.0rad
        rms = pointing_rms_deg(yaw, pos, tgt)
        assert rms == pytest.approx(0.0, abs=1e-6), \
            "동결 반경(0.3m) 내 샘플이 집계에 섞임"

    def test_wrap_around_error(self):
        """±π 랩어라운드에서 오차 2π로 오측 금지."""
        pos = np.array([[1.0, 0.0, 1.0]])
        tgt = [0.0, 0.001, 1.0]                # 목표각 ~ +pi
        yaw = np.array([-np.pi + 0.01])        # 실측 ~ -pi (같은 방향)
        rms = pointing_rms_deg(yaw, pos, tgt)
        assert rms < 2.0                       # 랩 처리 안 되면 ~360도

    def test_scan_coverage_full_and_partial(self):
        sc = {"from_rad": -1.0, "to_rad": 1.0}
        full = np.linspace(-1.0, 1.0, 100)
        half = np.linspace(-1.0, 0.0, 100)
        assert scan_coverage_actual(full, sc) == pytest.approx(1.0)
        assert scan_coverage_actual(half, sc) == pytest.approx(0.5, abs=0.01)
