"""traj_shaping.py 단위테스트 (HANDOFF_PATHTIME_PIPELINE.md 착수 1 요구사항).

핵심 검증 2종:
  - 정상 궤적(path_time 정품)은 스무더가 무개입 통과 (< 2mm)
  - 살인 궤적(스텝/저크 폭탄)은 성형 후 게이트 통과, 원본은 게이트 차단
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from path_time import plan_waypoints            # noqa: E402
from traj_shaping import (                      # noqa: E402
    counter_swing_offset,
    smooth_with_axis_sharing,
    traj_gate,
    traj_smoother,
    traj_zv,
)

# 성형 한계 (envelope 실측 2.5보다 깎은 값 — 핸드오프 확정 상수)
VMAX, AMAX, JMAX = 2.0, 2.0, 10.0
F_MODE = 1.80
DT = 0.01


def _clean_trajectory():
    """path_time 정품 궤적 (한계를 성형 한계보다 여유 있게)."""
    waypoints = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 3.0],
        [2.0, 0.0, 3.0],
        [2.0, 2.0, 3.0],
    ])
    t, pos, *_ = plan_waypoints(waypoints, 1.0, 0.8, 2.0, 10.0, dt=DT)
    return t, pos.T          # (N, 3)


def _killer_step(axis=0, jump=1.0, T=10.0):
    """한 샘플 만에 jump[m] 점프하는 발산 확정 궤적."""
    t = np.arange(0.0, T, DT)
    pos = np.zeros((len(t), 3))
    pos[t >= 0.5, axis] = jump
    return t, pos


def _jerk_bomb():
    """v/a는 온건하지만 저크만 한계 초과하는 궤적 (15차 옆문 사건 재현).

    가속도가 한 샘플 만에 +1 -> -1로 반전 -> 저크 200 (v 0.5, a 1.0은 통과 수준).
    """
    t = np.arange(0.0, 2.0, DT)
    acc = np.where((t >= 0.5) & (t < 1.5), -1.0, 1.0)
    vel = np.cumsum(acc) * DT
    x = np.cumsum(vel) * DT
    pos = np.zeros((len(t), 3))
    pos[:, 0] = x
    return t, pos


# ---------------------------------------------------------------------------
# traj_smoother
# ---------------------------------------------------------------------------

class TestSmoother:
    def test_clean_trajectory_no_intervention(self):
        t, pos = _clean_trajectory()
        pos_s, info = traj_smoother(t, pos, VMAX, AMAX, JMAX)
        assert np.max(info["maxDev"]) < 0.002, \
            f"정상 궤적에 개입 {np.max(info['maxDev'])*1000:.1f}mm (< 2mm 요구)"

    def test_clean_trajectory_passes_gate(self):
        t, pos = _clean_trajectory()
        pos_s, _ = traj_smoother(t, pos, VMAX, AMAX, JMAX)
        ok, rep = traj_gate(t, pos_s, VMAX, AMAX, do_error=True, jmax=JMAX)
        assert ok

    def test_killer_step_shaped_passes_gate(self):
        t, pos = _killer_step()
        ok_raw, _ = traj_gate(t, pos, VMAX, AMAX, do_error=False, jmax=JMAX)
        assert not ok_raw, "살인 궤적이 원본 그대로 게이트를 통과하면 안 됨"
        pos_s, _ = traj_smoother(t, pos, VMAX, AMAX, JMAX)
        ok, rep = traj_gate(t, pos_s, VMAX, AMAX, do_error=True, jmax=JMAX)
        assert ok

    def test_killer_step_reaches_target_without_overshoot(self):
        t, pos = _killer_step(jump=1.0)
        pos_s, _ = traj_smoother(t, pos, VMAX, AMAX, JMAX)
        x = pos_s[:, 0]
        assert abs(x[-1] - 1.0) < 0.01, "종점 미도달"
        assert np.max(x) < 1.0 + 0.05, \
            f"오버슈트 {(np.max(x)-1.0)*100:.1f}cm (정확 정지거리 공식이면 < 5cm)"

    def test_jerk_bomb_shaped_passes_gate(self):
        t, pos = _jerk_bomb()
        ok_raw, rep_raw = traj_gate(t, pos, VMAX, AMAX, do_error=False, jmax=JMAX)
        assert not ok_raw and rep_raw["jxyPk"] > JMAX, \
            "저크 폭탄은 저크 검사로 걸려야 함 (v/a는 온건)"
        assert rep_raw["vxyPk"] <= VMAX and rep_raw["axyPk"] <= AMAX
        pos_s, _ = traj_smoother(t, pos, VMAX, AMAX, JMAX)
        ok, _ = traj_gate(t, pos_s, VMAX, AMAX, do_error=True, jmax=JMAX)
        assert ok

    def test_state_is_backward_difference_of_output(self):
        """출력의 후방차분 v/a/j가 한계 이내 (원칙 1의 관측 가능한 귀결)."""
        t, pos = _killer_step()
        _, info = traj_smoother(t, pos, VMAX, AMAX, JMAX)
        tol = 1.001
        assert np.all(info["vPk"] <= VMAX * tol)
        assert np.all(info["aPk"] <= AMAX * tol)
        assert np.all(info["jPk"] <= JMAX * tol)

    def test_length_mismatch_raises(self):
        with pytest.raises(ValueError):
            traj_smoother(np.arange(10) * DT, np.zeros((5, 3)), VMAX, AMAX, JMAX)


# ---------------------------------------------------------------------------
# xy 축배분 (원칙 3)
# ---------------------------------------------------------------------------

class TestAxisSharing:
    def _diagonal_step(self):
        """xy 동시 기동 (대각 이동) — 축별 독립 전한계 성형 시 노름 √2 초과."""
        t = np.arange(0.0, 12.0, DT)
        pos = np.zeros((len(t), 3))
        pos[t >= 0.5, 0] = 3.0
        pos[t >= 0.5, 1] = 3.0
        return t, pos

    def test_full_limits_diagonal_fails_gate(self):
        t, pos = self._diagonal_step()
        pos_s, _ = traj_smoother(t, pos, VMAX, AMAX, JMAX)
        ok, rep = traj_gate(t, pos_s, VMAX, AMAX, do_error=False, jmax=JMAX)
        assert not ok, "박스 투어 실증(§W) 재현 실패 — 대각 노름이 걸렸어야 함"

    def test_axis_sharing_diagonal_passes_gate(self):
        t, pos = self._diagonal_step()
        pos_s, info = smooth_with_axis_sharing(t, pos, VMAX, AMAX, JMAX)
        assert info["xy_share_applied"] == pytest.approx(0.7)
        # xy는 v/a/j 전부 0.7 배분이라 노름 <= 0.7*√2*한계 < 한계
        ok, rep = traj_gate(t, pos_s, VMAX, AMAX, do_error=True, jmax=JMAX)
        assert ok

    def test_single_axis_motion_keeps_full_limits(self):
        t, pos = _killer_step(axis=0)
        _, info = smooth_with_axis_sharing(t, pos, VMAX, AMAX, JMAX)
        assert info["xy_share_applied"] == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# traj_zv
# ---------------------------------------------------------------------------

class TestZV:
    def test_zv_is_half_plus_half_delayed(self):
        t, pos = _killer_step()
        pos_sm, _ = traj_smoother(t, pos, VMAX, AMAX, JMAX)
        pos_zv = traj_zv(t, pos_sm, F_MODE, "zv")
        d_half = int(round(1.0 / (2.0 * F_MODE) / DT))
        k = len(t) // 2
        expected = 0.5 * pos_sm[k] + 0.5 * pos_sm[k - d_half]
        np.testing.assert_allclose(pos_zv[k], expected, atol=1e-12)

    @pytest.mark.parametrize("mode", ["zv", "zvd"])
    def test_zv_preserves_limits_after_smoother(self, mode):
        """볼록 결합이라 스무더의 v/a/j 한계 보존 — 파이프라인 순서의 근거."""
        t, pos = _killer_step()
        pos_sm, _ = traj_smoother(t, pos, VMAX, AMAX, JMAX)
        pos_zv = traj_zv(t, pos_sm, F_MODE, mode)
        ok, _ = traj_gate(t, pos_zv, VMAX, AMAX, do_error=True, jmax=JMAX)
        assert ok

    def test_zvd_delay_is_full_period(self):
        t, pos = _killer_step()
        pos_sm, _ = traj_smoother(t, pos, VMAX, AMAX, JMAX)
        pos_zvd = traj_zv(t, pos_sm, F_MODE, "zvd")
        d_half = int(round(1.0 / (2.0 * F_MODE) / DT))
        k = len(t) // 2
        expected = (0.25 * pos_sm[k] + 0.5 * pos_sm[k - d_half]
                    + 0.25 * pos_sm[k - 2 * d_half])
        np.testing.assert_allclose(pos_zvd[k], expected, atol=1e-12)

    def test_nonuniform_time_raises(self):
        t = np.array([0.0, 0.01, 0.03, 0.04, 0.05])
        with pytest.raises(ValueError, match="균일"):
            traj_zv(t, np.zeros((5, 3)), F_MODE)

    def test_coarse_sampling_raises(self):
        t = np.arange(0.0, 5.0, 0.7)     # round(반주기 0.278s / 0.7) = 0 -> 성김
        with pytest.raises(ValueError, match="성김"):
            traj_zv(t, np.zeros((len(t), 3)), F_MODE)

    def test_bad_mode_raises(self):
        t = np.arange(0.0, 1.0, DT)
        with pytest.raises(ValueError, match="mode"):
            traj_zv(t, np.zeros((len(t), 3)), F_MODE, "zvdd")


# ---------------------------------------------------------------------------
# counter_swing_offset (지터 소거 2호기 — 역위상 카운터 가속)
# ---------------------------------------------------------------------------

class TestCounterSwing:
    def test_amplitude_clamped_by_jerk_budget(self):
        t = np.arange(0.0, 20.0, DT)
        jerk_budget = 2.0        # 물리 10의 지터 예산 20%
        w = 2.0 * np.pi * F_MODE
        off, a_used = counter_swing_offset(
            t, amp_pos_m=0.1, phase_rad=0.0, t_ref_s=5.0,
            f_mode=F_MODE, jerk_budget=jerk_budget)
        assert a_used == pytest.approx(jerk_budget / w**3)
        assert np.max(np.abs(off)) <= a_used * 1.0001

    def test_offset_added_trajectory_passes_gate(self):
        """호버 유지 궤적 + 카운터 오프셋이 게이트 3종 검사 이내."""
        t = np.arange(0.0, 20.0, DT)
        pos = np.zeros((len(t), 3))
        pos[:, 2] = 2.0          # 정지 호버
        off, a_used = counter_swing_offset(
            t, amp_pos_m=0.1, phase_rad=1.0, t_ref_s=5.0,
            f_mode=F_MODE, jerk_budget=2.0)
        assert a_used > 0
        pos2 = pos.copy()
        pos2[:, 0] += off        # 피치 지터 → x축 주입
        ok, rep = traj_gate(t, pos2, VMAX, AMAX, do_error=True, jmax=JMAX)
        assert ok

    def test_antiphase(self):
        """오프셋은 측정 진동과 역위상 (측정 위상 + π)."""
        t = np.arange(0.0, 40.0, DT)
        w = 2.0 * np.pi * F_MODE
        off, a_used = counter_swing_offset(
            t, amp_pos_m=1e-3, phase_rad=0.7, t_ref_s=0.0,
            f_mode=F_MODE, jerk_budget=2.0, ramp_cycles=2.0)
        measured = np.sin(w * t + 0.7)
        mid = slice(len(t) // 4, len(t) // 2)   # 램프 밖 정상 구간
        corr = np.corrcoef(off[mid], measured[mid])[0, 1]
        assert corr < -0.99, "역위상(상관 -1)이어야 함"

    def test_inactive_before_t_ref_and_zero_amp(self):
        t = np.arange(0.0, 10.0, DT)
        off, _ = counter_swing_offset(t, 0.01, 0.0, 6.0, F_MODE, 2.0)
        assert np.all(off[t < 6.0] == 0.0)
        off0, a0 = counter_swing_offset(t, 0.0, 0.0, 0.0, F_MODE, 2.0)
        assert a0 == 0.0 and np.all(off0 == 0.0)


# ---------------------------------------------------------------------------
# traj_gate
# ---------------------------------------------------------------------------

class TestGate:
    def test_error_mode_raises_with_report(self):
        t, pos = _killer_step()
        with pytest.raises(ValueError, match="물리 한계 초과"):
            traj_gate(t, pos, VMAX, AMAX, do_error=True, jmax=JMAX)

    def test_report_mode_returns_false(self):
        t, pos = _killer_step()
        ok, rep = traj_gate(t, pos, VMAX, AMAX, do_error=False, jmax=JMAX)
        assert not ok
        assert rep["vxyPk"] > VMAX

    def test_xy_is_norm_not_per_axis(self):
        """축별로는 한계 이내지만 노름은 초과하는 대각 궤적을 잡는다."""
        t = np.arange(0.0, 8.0, DT)
        # 각 축 v=1.8 (< 2.0)로 동시 등속 -> 노름 2.55 > 2.0
        ramp = np.clip(t - 1.0, 0.0, 4.0) * 1.8
        pos = np.zeros((len(t), 3))
        pos[:, 0] = ramp
        pos[:, 1] = ramp
        ok, rep = traj_gate(t, pos, VMAX, AMAX, do_error=False, jmax=1e9)
        assert not ok
        assert rep["vxyPk"] == pytest.approx(1.8 * np.sqrt(2.0), rel=1e-3)

    def test_too_few_samples_raises(self):
        with pytest.raises(ValueError):
            traj_gate(np.array([0.0, 0.01, 0.02]), np.zeros((3, 3)), VMAX, AMAX)

    def test_nonmonotonic_time_raises(self):
        t = np.array([0.0, 0.01, 0.01, 0.02, 0.03])
        with pytest.raises(ValueError, match="단조증가"):
            traj_gate(t, np.zeros((5, 3)), VMAX, AMAX)
