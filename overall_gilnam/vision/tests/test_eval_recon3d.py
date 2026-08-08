"""eval_recon3d 테스트 — 무노이즈 정합 게이트(≤1mm)가 핵심."""
import json
from pathlib import Path

import numpy as np

from eval_recon3d import evaluate_records, _finite_median, summarize
from noisy_stream import load_records, P_TAIL, make_noisy_records

VISION_DIR = Path(__file__).resolve().parents[1]


def _load_sample():
    records = load_records(VISION_DIR / "sample_stream" / "sample_stream.jsonl")
    scene_gt = json.loads((VISION_DIR / "sample_stream" / "scene_gt.json").read_text(encoding="utf-8"))
    return records, scene_gt


def test_noiseless_reconstruction_within_1mm():
    """태민 7/4 결과표(0.01~0.07mm)와 자릿수 정합 — 구현 검증 게이트."""
    records, scene_gt = _load_sample()
    results = evaluate_records(records, scene_gt)
    assert len(results) == 3  # 창문 3개 전부 복원돼야 함
    for r in results:
        assert r["n_pairs"] > 0
        assert r["corner_err_max_mm"] < 1.0
        assert r["center_err_mm"] < 1.0
        assert all(abs(e) < 2.0 for e in r["size_err_mm"])


def test_window_without_valid_pairs_reports_zero():
    """유효 쌍 0인 창문 → 스텁 항목(n_pairs=0, 오차 필드 없음) 보고 확인."""
    # 최소 레코드: 프레임 1개, 창문 1개(4 corner vis=1) — 쌍 불가능
    records = [
        {
            "vision": {
                "timestamp": 1000,
                "frame_id": 0,
                "windows": [
                    {
                        "order_index": 0,
                        "color": "red",
                        "corners": [[100.0, 100.0], [200.0, 100.0], [200.0, 200.0], [100.0, 200.0]],
                        "corner_vis": [1, 1, 1, 1],
                    }
                ]
            },
            "pose": {
                "position": [0.0, 0.0, 0.0],
                "orientation": [0.0, 0.0, 0.0, 1.0],
            }
        }
    ]
    # 최소 scene_gt: intrinsics + 하나의 창문
    scene_gt = {
        "intrinsics": {
            "fx": 800.0, "fy": 800.0, "cx": 320.0, "cy": 240.0
        },
        "windows": [
            {
                "order_index": 0,
                "color": "red",
                "corners_3d": [[0.0, 0.0, 2.0], [1.0, 0.0, 2.0], [1.0, 1.0, 2.0], [0.0, 1.0, 2.0]],
                "center": [0.5, 0.5, 2.0],
                "size_wh": [1.0, 1.0],
            }
        ]
    }
    results = evaluate_records(records, scene_gt)
    assert len(results) == 1
    assert results[0]["order_index"] == 0
    assert results[0]["color"] == "red"
    assert results[0]["n_pairs"] == 0
    # 오차 필드 없음 — 다운스트림에서 n_pairs > 0으로 필터
    assert "corner_err_mm" not in results[0]
    assert "corner_err_max_mm" not in results[0]
    assert "center_err_mm" not in results[0]


def test_nonfinite_estimates_are_excluded():
    """비유한값(NaN/Inf) 추정값 → _finite_median에서 제외, 유효 개수만 보고."""
    # 케이스 1: 유효·비유한 혼합 → 유효한 것만 중앙값
    finite_est = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0], [10.0, 11.0, 12.0]])  # (4,3)
    nan_est = np.array([[np.nan, 1.0, 1.0], [2.0, 2.0, 2.0], [3.0, 3.0, 3.0], [4.0, 4.0, 4.0]])  # (4,3)
    inf_est = np.array([[1.0, np.inf, 1.0], [2.0, 2.0, 2.0], [3.0, 3.0, 3.0], [4.0, 4.0, 4.0]])  # (4,3)

    estimates = [finite_est, nan_est, inf_est, finite_est]
    result, n_finite = _finite_median(estimates)

    assert n_finite == 2, f"expected 2 finite estimates, got {n_finite}"
    assert result is not None
    np.testing.assert_array_almost_equal(result, np.median(np.stack([finite_est, finite_est]), axis=0))

    # 케이스 2: 모두 비유한값 → (None, 0)
    estimates_all_nan = [nan_est, inf_est]
    result, n_finite = _finite_median(estimates_all_nan)
    assert result is None
    assert n_finite == 0

    # 케이스 3: 빈 리스트 → (None, 0)
    result, n_finite = _finite_median([])
    assert result is None
    assert n_finite == 0


def test_error_grows_with_noise_scale():
    records, scene_gt = _load_sample()
    err = {}
    for scale in (0.0, 0.5, 2.0):
        noisy = make_noisy_records(records, scale, 1234, 8.87, 36.6, P_TAIL, drop_prob=0.0)
        s = summarize(f"x{scale}", evaluate_records(noisy, scene_gt))
        err[scale] = s["center_err_mean_mm"]
    assert err[0.0] < err[0.5] < err[2.0]
