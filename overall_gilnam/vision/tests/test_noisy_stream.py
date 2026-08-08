"""noisy_stream 테스트 — 캘리브레이션 정합·주입 스키마·재현성."""
import numpy as np

from noisy_stream import P_TAIL, calibrate_mixture

MEAN_PX, P95_PX = 8.87, 36.6


def _sample_radial(sigma_core, sigma_tail, p_tail, n, seed=0):
    """혼합 모델에서 2D 반경 오차 표본 추출 (테스트 전용 몬테카를로)."""
    rng = np.random.default_rng(seed)
    sigmas = np.where(rng.random(n) < p_tail, sigma_tail, sigma_core)
    xy = rng.normal(0.0, 1.0, (n, 2)) * sigmas[:, None]
    return np.linalg.norm(xy, axis=1)


def test_calibration_matches_measured_stats():
    sc, st = calibrate_mixture(MEAN_PX, P95_PX)
    assert 0.0 < sc < st  # 코어보다 꼬리가 넓어야 함
    r = _sample_radial(sc, st, P_TAIL, 200_000)
    assert abs(r.mean() - MEAN_PX) / MEAN_PX < 0.03
    assert abs(np.percentile(r, 95) - P95_PX) / P95_PX < 0.05
