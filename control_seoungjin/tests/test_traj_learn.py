# -*- coding: utf-8 -*-
"""traj_learn (사후 학습 ILC) 단위 테스트 — 합성 1차 지연 플랜트에서 반복 학습이 추종 오차를 줄이고, 보정이 한계·해시 규약을 지키는지."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import traj_learn as tl  # noqa: E402


def _s5(u):
    u = np.clip(u, 0, 1)
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def _plant_lag(t, ref, tau=0.06):
    """1차 지연 플랜트 (추종 지연 모사): y' = (ref - y)/tau."""
    dt = t[1] - t[0]
    y = np.zeros_like(ref)
    for i in range(1, len(t)):
        y[i] = y[i - 1] + dt * (ref[i - 1] - y[i - 1]) / tau
    return y


def test_ilc_converges_on_lag_plant():
    dt = 0.01
    t = np.arange(0, 6.0 + 1e-9, dt)
    target = np.column_stack([_s5((t - 2.0) / 1.5), np.zeros_like(t), np.ones_like(t)])
    prev = None
    rms_hist = []
    for k in range(4):
        c = tl.correction_on_grid(prev, t) if prev is not None else np.zeros_like(target)
        act = np.column_stack([_plant_lag(t, target[:, i] + c[:, i]) for i in range(3)])
        act[:, 2] = 1.0   # z 완벽 추종 가정
        c_new, st = tl.learn(t, target, act, prev=prev, gain=0.6, lpf_hz=1.0)
        rms_hist.append(st["rms_before_cm"]["3d"])
        prev = {"t": t, "c": c_new, "iter": k + 1, "gain": 0.6, "lpf_hz": 1.0}
    assert rms_hist[1] < rms_hist[0] * 0.8, rms_hist
    assert rms_hist[-1] < rms_hist[0] * 0.5, rms_hist
    assert np.abs(prev["c"]).max() <= tl.C_MAX_M + 1e-12
    # 시작·끝 테이퍼: 이륙점/종점 불변
    assert abs(prev["c"][0]).max() < 1e-9 and abs(prev["c"][-1]).max() < 1e-9


def test_apply_correction_hash_guard(tmp_path=None):
    t = np.arange(0, 3.0, 0.01)
    shaped = np.column_stack([t / 3.0, 0 * t, 1 + 0 * t])
    corr = {"base_trajectory_hash": "deadbeef", "iter": 1, "gain": 0.6, "lpf_hz": 1.0,
            "t": t, "c": np.zeros((len(t), 3)), "hold_ext_s": 1.5}
    try:
        tl.apply_correction(t, shaped, corr, "cafebabe")
        raise AssertionError("해시 불일치인데 통과")
    except ValueError:
        pass
    t2, sh2, cor, meta = tl.apply_correction(t, shaped, corr, "deadbeef")
    assert len(t2) == len(t) + 150 and np.allclose(sh2[-1], shaped[-1]) and meta["correction_iter"] == 1


def test_taper_is_c2_smooth():
    t = np.arange(0, 5.0, 0.01)
    w = tl._taper(t)
    d3 = np.gradient(np.gradient(np.gradient(w, 0.01), 0.01), 0.01)
    assert np.isfinite(d3).all() and w[0] == 0 and w[-1] == 0 and w[len(t) // 2] == 1.0


if __name__ == "__main__":
    test_ilc_converges_on_lag_plant()
    test_apply_correction_hash_guard()
    test_taper_is_c2_smooth()
    print("traj_learn tests ok")
