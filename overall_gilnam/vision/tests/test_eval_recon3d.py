"""eval_recon3d 테스트 — 무노이즈 정합 게이트(≤1mm)가 핵심."""
import json
from pathlib import Path

from eval_recon3d import evaluate_records
from noisy_stream import load_records

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
