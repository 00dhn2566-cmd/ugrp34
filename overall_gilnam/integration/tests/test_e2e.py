"""E2E 리허설 테스트 — scale 0 전 구간 게이트(≤1mm)가 핵심."""
import numpy as np
import pytest

from e2e_rehearsal import SAMPLE, assemble_window_map, load_inputs, run_scale


def test_scale0_full_pipeline_gate():
    # 무노이즈: 복원 창문 맵 계획이 GT 창문 계획과 게이트점 1mm 이내 일치
    records, scene_gt, cfg = load_inputs()
    result = run_scale(records, scene_gt, cfg, scale=0.0)
    assert result["failed"] == [] and result["n_warnings"] == 0
    for w in result["windows"]:
        assert w["approach_err_mm"] < 1.0 and w["exit_err_mm"] < 1.0
        assert w["margin_left_mm"] > 0


def test_assembled_map_matches_gt():
    # 이음새 검증: 복원 맵의 center·size·normal(부호 포함)이 GT와 정합
    records, scene_gt, cfg = load_inputs()
    from eval_recon3d import reconstruct_windows
    wmap, failed = assemble_window_map(reconstruct_windows(records, scene_gt))
    assert failed == [] and len(wmap["windows"]) == 3
    for w, gt in zip(wmap["windows"], scene_gt["windows"]):
        assert w["order_index"] == gt["order_index"]
        assert np.linalg.norm(np.asarray(w["center"]) - np.asarray(gt["center"])) * 1000 < 1.0
        assert float(np.dot(w["normal"], np.asarray(gt["normal"]))) > 0.999  # 부호 확정 공식
        np.testing.assert_allclose(w["size_wh"], gt["size_wh"], atol=2e-3)


def test_failed_window_excluded_and_reported():
    recon = {
        0: {"color": "red", "corners_3d_est": None, "n_pairs": 0},
        1: {"color": "green", "n_pairs": 5, "corners_3d_est": np.array(
            [[4.0, 0.6, 2.0], [4.0, -0.6, 2.0], [4.0, -0.6, 1.0], [4.0, 0.6, 1.0]])},
    }
    wmap, failed = assemble_window_map(recon)
    assert failed == [0]
    assert [w["order_index"] for w in wmap["windows"]] == [1]
