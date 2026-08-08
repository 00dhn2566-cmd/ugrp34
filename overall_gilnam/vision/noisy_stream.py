"""GT 스트림에 실측 프로파일 노이즈 주입 (설계: overall_gilnam/docs/superpowers/specs/2026-08-08-*.md).

실측(2026-08-02 본 판정): corner 반경 오차 평균 8.87px / p95 36.6px / 미검출 31/802.
p95/평균 ≈ 4.1로 가우시안(≈1.95)보다 꼬리가 무거워, 2성분 가우시안 혼합
(per-axis, 확률 1-p 코어 σc / p 꼬리 σt, p=0.1 고정)을 쓴다.
(σc, σt)는 혼합 Rayleigh의 평균·p95가 실측치와 일치하도록 이분법으로 캘리브레이션.
"""

import math

P_TAIL = 0.1
DEFAULT_MEAN_PX = 8.87
DEFAULT_P95_PX = 36.6
DEFAULT_DROP = 31.0 / 802.0
RADIAL_MEAN_COEF = math.sqrt(math.pi / 2.0)  # E[r] = σ·√(π/2) (2D 가우시안 반경 = Rayleigh)


def _mixture_p95(sigma_core, sigma_tail, p_tail):
    """혼합 Rayleigh CDF의 95% 분위 (이분법)."""

    def cdf(r):
        core = 1.0 - math.exp(-r * r / (2.0 * sigma_core * sigma_core)) if sigma_core > 0 else 1.0
        tail = 1.0 - math.exp(-r * r / (2.0 * sigma_tail * sigma_tail))
        return (1.0 - p_tail) * core + p_tail * tail

    lo, hi = 0.0, 20.0 * max(sigma_core, sigma_tail)
    for _ in range(200):
        mid = (lo + hi) / 2.0
        if cdf(mid) < 0.95:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def calibrate_mixture(mean_px, p95_px, p_tail=P_TAIL):
    """(σc, σt) 캘리브레이션. σt를 이분 탐색, σc는 평균 제약에서 종속 결정.

    탐색 구간: σt = σ_all(단일 가우시안, p95 하한)부터 σc=0이 되는 상한까지.
    이 구간에서 p95는 σt에 단조 증가 — 실측 p95가 구간 밖이면 ValueError.
    """
    sigma_all = mean_px / RADIAL_MEAN_COEF
    lo, hi = sigma_all, mean_px / (RADIAL_MEAN_COEF * p_tail)
    if not _mixture_p95(sigma_all, sigma_all, p_tail) < p95_px < _mixture_p95(0.0, hi, p_tail):
        raise ValueError(f"p95={p95_px}는 혼합 모델 표현 범위 밖 (mean={mean_px}, p={p_tail})")
    for _ in range(100):
        sigma_tail = (lo + hi) / 2.0
        sigma_core = (mean_px / RADIAL_MEAN_COEF - p_tail * sigma_tail) / (1.0 - p_tail)
        if _mixture_p95(sigma_core, sigma_tail, p_tail) < p95_px:
            lo = sigma_tail
        else:
            hi = sigma_tail
    sigma_tail = (lo + hi) / 2.0
    sigma_core = (mean_px / RADIAL_MEAN_COEF - p_tail * sigma_tail) / (1.0 - p_tail)
    return sigma_core, sigma_tail
