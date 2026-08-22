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


import argparse
import json
from pathlib import Path

import numpy as np

from vision_msg import build_frame_message, build_window

PX_DECIMALS = 2  # make_stream.py와 동일한 기록 자릿수


def _noisy_windows(windows, rng, sigma_core, sigma_tail, p_tail, drop_prob):
    """§5 windows[] → 노이즈 주입본. 기하(corners·center)만 변경, 드롭은 창문 단위."""
    out = []
    for w in windows:
        if rng.random() < drop_prob:
            continue
        corners = np.asarray(w["corners"], dtype=float)
        sigmas = np.where(rng.random(len(corners)) < p_tail, sigma_tail, sigma_core)
        corners = corners + rng.normal(0.0, 1.0, corners.shape) * sigmas[:, None]
        nw = build_window(
            w["order_index"], w["color"],
            [[round(float(u), PX_DECIMALS), round(float(v), PX_DECIMALS)] for u, v in corners],
            w["corner_vis"], w["det_conf"], w["color_conf"],
        )
        nw["center"] = [round(c, PX_DECIMALS) for c in nw["center"]]
        out.append(nw)
    return out


def make_noisy_records(records, scale, seed, mean_px, p95_px, p_tail, drop_prob):
    """스트림 레코드 리스트 → 배율 scale 노이즈 주입본 (pose 불변, 결정적)."""
    sigma_core, sigma_tail = calibrate_mixture(mean_px, p95_px, p_tail)
    rng = np.random.default_rng([seed, int(round(scale * 100))])  # 배율별 독립 시드
    out = []
    for rec in records:
        msg = rec["vision"]
        windows = _noisy_windows(msg["windows"], rng,
                                 sigma_core * scale, sigma_tail * scale, p_tail, drop_prob)
        out.append({"vision": build_frame_message(msg["timestamp"], msg["frame_id"], windows),
                    "pose": rec["pose"]})
    return out


def load_records(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def write_records(records, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for rec in records:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def main():
    ap = argparse.ArgumentParser(description="GT 스트림 → 실측 프로파일 노이즈 주입 jsonl (배율별)")
    ap.add_argument("--stream", required=True, help="입력 §5+pose jsonl (make_stream.py 산출물)")
    ap.add_argument("--out", required=True, help="출력 디렉터리")
    ap.add_argument("--scales", default="0.25,0.5,1,1.5,2,3")
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--mean", type=float, default=DEFAULT_MEAN_PX)
    ap.add_argument("--p95", type=float, default=DEFAULT_P95_PX)
    ap.add_argument("--drop", type=float, default=DEFAULT_DROP)
    args = ap.parse_args()

    records = load_records(args.stream)
    sc, st = calibrate_mixture(args.mean, args.p95)
    print(f"calibrated: sigma_core={sc:.3f}px sigma_tail={st:.3f}px (p_tail={P_TAIL})")
    for scale in [float(s) for s in args.scales.split(",")]:
        noisy = make_noisy_records(records, scale, args.seed, args.mean, args.p95, P_TAIL, args.drop)
        path = Path(args.out) / f"noisy_x{scale:g}.jsonl"
        write_records(noisy, path)
        n_win = sum(len(r["vision"]["windows"]) for r in noisy)
        print(f"x{scale:g}: {len(noisy)} frames, {n_win} windows → {path}")


if __name__ == "__main__":
    main()
