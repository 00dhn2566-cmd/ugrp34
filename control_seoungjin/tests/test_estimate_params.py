"""estimate_params.py 단위테스트 — 합성 물리 데이터에서 상수 복원 검증."""

import os
import sys

import numpy as np
import pytest
from scipy.io import savemat

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import estimate_params as ep                    # noqa: E402

G = ep.G


def _ts(t, v):
    """StructureWithTime 형태로 저장되는 dict (loadmat round-trip 호환)."""
    return {"time": t.reshape(-1, 1), "signals": {"values": v.reshape(-1, 1)}}


@pytest.fixture
def synthetic_mat(tmp_path):
    """참값을 아는 합성 비행: m=1.5, kT=1e-5, L/Ixx 기지, kd/Izz 기지."""
    dt = 0.005
    t = np.arange(0.0, 12.0, dt)

    m_true, kt_true = 1.5, 1e-5
    L, ixx_true = ep.ARM_LENGTH_M, 0.02
    izz_true = ep.IZZ_NOMINAL              # Izz 공칭 = 참값으로 두고 kd 복원 확인
    kd_true = 0.5

    # w = 호버 + 공통 모드(z 가진) + 채널별 미소 차동 (각도 드리프트 방지 수준)
    w_h = np.sqrt(m_true * G / (4 * kt_true))
    # 변조 진폭: 각도 드리프트/샘플당 yaw 회전이 물리적 범위에 머물게 미소로
    common = 0.05 * np.sin(2 * np.pi * 0.4 * t)
    d_roll = 2e-4 * np.sin(2 * np.pi * 0.7 * t)     # 부호 (+,+,-,-)
    d_pitch = 2e-4 * np.cos(2 * np.pi * 0.6 * t)    # 부호 (+,-,+,-)
    d_yaw = 5e-6 * np.sin(2 * np.pi * 0.5 * t)      # 부호 (+,-,-,+)
    signs = [(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)]
    w = [w_h * (1 + common + sr * d_roll + sp * d_pitch + sy * d_yaw)
         for sr, sp, sy in signs]
    T = [kt_true * wi**2 for wi in w]

    # 병진 z: m·(z̈+g) = ΣT
    zdd = (sum(T) / m_true) - G
    vz = np.cumsum(zdd) * dt

    def _int2(acc):
        """평균 제거(드리프트 방지 — 실기의 자세루프 구속 흉내) 후 2중 적분."""
        acc = acc - np.mean(acc)
        vel = np.cumsum(acc) * dt
        vel = vel - np.mean(vel)
        return np.cumsum(vel) * dt

    # roll/pitch 각가속 = (L/I)·ΔT
    roll = _int2((L / ixx_true) * (T[0] + T[1] - T[2] - T[3]))
    pitch = _int2((L / ixx_true) * (T[0] - T[1] + T[2] - T[3]))   # Iyy = Ixx

    # yaw 각가속 = (kd/Izz)·(w1²-w2²-w3²+w4²)
    dw2 = w[0]**2 - w[1]**2 - w[2]**2 + w[3]**2
    yaw = _int2((kd_true / izz_true) * dw2)

    data = {"real_roll": _ts(t, roll), "real_pitch": _ts(t, pitch),
            "real_yaw": _ts(t, yaw), "real_vz": _ts(t, vz)}
    for i in range(4):
        data[f"prop{i+1}_T"] = _ts(t, T[i])
        data[f"prop{i+1}_w"] = _ts(t, w[i])
    p = str(tmp_path / "synth.mat")
    savemat(p, data)
    return p, {"m": m_true, "kt": kt_true, "ixx": ixx_true, "kd": kd_true}


class TestEstimator:
    def test_recovers_constants(self, synthetic_mat):
        path, truth = synthetic_mat
        est = ep.estimate(path)
        assert est["mass_kg"]["value"] == pytest.approx(truth["m"], rel=0.05)
        assert est["mass_kg"]["confident"]
        assert est["k_thrust_lumped"]["value"] == pytest.approx(
            truth["kt"], rel=0.02)
        assert est["k_thrust_lumped"]["confident"]
        assert est["inertia"]["Ixx"]["value"] == pytest.approx(
            truth["ixx"], rel=0.10)
        assert est["inertia"]["Iyy"]["value"] == pytest.approx(
            truth["ixx"], rel=0.10)
        assert est["k_drag_lumped"]["value"] == pytest.approx(
            truth["kd"], rel=0.10)

    def test_missing_signals_die_loudly(self, tmp_path):
        p = str(tmp_path / "empty.mat")
        savemat(p, {"real_roll": _ts(np.arange(3.0), np.zeros(3))})
        with pytest.raises(KeyError, match="로그 변수 없음"):
            ep.estimate(p)

    def test_write_estimate_atomic(self, synthetic_mat, tmp_path):
        path, _ = synthetic_mat
        est = ep.estimate(path)
        out = str(tmp_path / "param_estimate.json")
        doc = ep.write_estimate(est, path=out, meta_path=str(tmp_path / "x"))
        assert os.path.isfile(out)
        assert doc["estimates"]["mass_kg"]["confident"] in (True, False)
