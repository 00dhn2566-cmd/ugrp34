"""isaacsim_export.py — 성진 산출물 → 윤호(Isaac Sim) 스키마 변환기 테스트.

계약 (reinforcement_yunho/interface/isaacsim_motor_commands.schema.json,
CONVENTIONS.md "Control→sim boundary is the per-motor angular-velocity setpoint"):
`motor_cmd_w`는 로터별 각속도 **setpoint**, **unsigned**. 플랜트 실측 아님.
"""

import os
import sys

import numpy as np
import pytest
from scipy.io import savemat

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import isaacsim_export as ie                    # noqa: E402


T = np.arange(0.0, 0.05, 0.01)
SETPOINT = np.array([600.0, 610.0, 620.0, 630.0])          # 제어기 setpoint (양수)
MEASURED = np.array([590.0, -605.0, -615.0, 625.0])        # 실측: 모터 2·3 내장 역회전


def _mat(tmp_path, setpoint=True, measured=True):
    d = {"sim_time": T}
    for i in range(4):
        if setpoint:
            d[f"motor_cmd_w{i+1}"] = np.full_like(T, SETPOINT[i])
        if measured:
            d[f"prop{i+1}_w"] = np.full_like(T, MEASURED[i])
    p = tmp_path / "sim_result.mat"
    savemat(str(p), d)
    return str(p)


class TestMotorCommands:
    def test_prefers_setpoint_channel_over_measured(self, tmp_path):
        info = {}
        out = ie.flight_mat_to_motor_commands(_mat(tmp_path), info=info)
        assert info["source"] == "setpoint"
        assert len(out["frames"]) == len(T)
        np.testing.assert_allclose(out["frames"][0]["motor_cmd_w"], SETPOINT)
        assert out["frames"][0]["time"] == pytest.approx(T[0])

    def test_measured_only_rejected_by_default(self, tmp_path):
        with pytest.raises(KeyError, match="setpoint"):
            ie.flight_mat_to_motor_commands(_mat(tmp_path, setpoint=False))

    def test_measured_fallback_is_explicit_and_unsigned(self, tmp_path):
        info = {}
        out = ie.flight_mat_to_motor_commands(
            _mat(tmp_path, setpoint=False), allow_measured=True, info=info)
        assert info["source"] == "measured"
        w = out["frames"][0]["motor_cmd_w"]
        np.testing.assert_allclose(w, np.abs(MEASURED))
        assert all(v >= 0 for v in w)          # 부호는 Isaac 로터 설정 몫

    def test_rotor_order_reorders(self, tmp_path):
        out = ie.flight_mat_to_motor_commands(_mat(tmp_path), rotor_order=(4, 3, 2, 1))
        np.testing.assert_allclose(out["frames"][0]["motor_cmd_w"], SETPOINT[::-1])

    def test_schema_shape_only(self, tmp_path):
        out = ie.flight_mat_to_motor_commands(_mat(tmp_path))
        assert set(out) == {"fps", "frames"}
        assert set(out["frames"][0]) == {"time", "motor_cmd_w"}
        assert out["fps"] == pytest.approx(100.0)


class TestTrajectory:
    def test_quat_wxyz_yaw_only(self):
        traj = {"dt": 0.01, "t": [0.0, 0.01], "pos": [[0, 0, 1], [0.01, 0, 1]],
                "yaw_rad": [0.0, np.pi / 2]}
        out = ie.trajectory_to_isaacsim(traj)
        assert out["fps"] == pytest.approx(100.0)
        q = out["frames"][1]["orientation_quat_wxyz"]
        np.testing.assert_allclose(q, [np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)])
        assert set(out["frames"][0]) == {"time", "position", "yaw_rad",
                                         "orientation_quat_wxyz"}
