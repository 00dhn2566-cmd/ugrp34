"""파이프라인 E2E 리허설: 비전 GT → 노이즈 → 삼각측량 복원 → 창문 맵 → 웨이포인트 계획.

설계: overall_gilnam/docs/superpowers/specs/2026-08-08-e2e-rehearsal-design.md.
체크리스트 4번 "전체 파이프라인 통합 검증 주도"의 실행 — 태민(융합)·성진(궤적) 대역은
각각 eval_recon3d 재현 삼각측량·waypoints_config 스키마 검증으로 대신한다.

지표: GT 창문 계획 대비 게이트점 오차(mm), 계획 경로의 GT 창문 평면 통과점이
개구부 중심에서 벗어난 거리와 잔여 여유(mm — margin 논의 직결), 안전 경고 수.
scale 0은 노이즈·드롭 없는 원본 스트림 그대로 (전 구간 정합 게이트).
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

INTEGRATION_DIR = Path(__file__).resolve().parent
GILNAM = INTEGRATION_DIR.parent
for _sub in ("vision", "planning"):
    sys.path.insert(0, str(GILNAM / _sub))

from noisy_stream import DEFAULT_DROP, DEFAULT_MEAN_PX, DEFAULT_P95_PX, P_TAIL, load_records, make_noisy_records  # noqa: E402
from eval_recon3d import reconstruct_windows  # noqa: E402
from window_waypoint_planner import (  # noqa: E402
    PLANNING_DIR, UP, crossing_warnings, gate_points, load_planner_config,
    normal_from_corners, plan_waypoints,
)

SAMPLE = GILNAM / "vision" / "sample_stream"
SCALES = [0.0, 0.5, 1.0, 1.5, 2.0]


def load_inputs():
    """샘플 스트림·scene_gt·계획기 설정 로드."""
    records = load_records(SAMPLE / "sample_stream.jsonl")
    scene_gt = json.loads((SAMPLE / "scene_gt.json").read_text(encoding="utf-8"))
    cfg = load_planner_config(PLANNING_DIR / "planner_limits.yaml")
    return records, scene_gt, cfg


def assemble_window_map(recon):
    """reconstruct_windows 결과 → §6.2 창문 맵. 복원 불가 창문은 제외하고 failed로 보고."""
    windows, failed = [], []
    for order_index in sorted(recon):
        r = recon[order_index]
        est = r["corners_3d_est"]
        if est is None:
            failed.append(order_index)
            continue
        est = np.asarray(est, dtype=float)
        tl, tr, br, bl = est
        w = (np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2.0
        h = (np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2.0
        windows.append({
            "order_index": order_index,
            "color": r["color"],
            "corners_3d": est.tolist(),
            "center": est.mean(axis=0).tolist(),
            "normal": normal_from_corners(est).tolist(),  # 부호 확정 공식 (spec §3.1)
            "size_wh": [float(w), float(h)],
        })
    return {"windows": windows}, failed


def _pass_point(a, b, center, n):
    """구간 a→b가 평면(center, n)을 지나는 점 (crossing_warnings와 동일 보간)."""
    da, db = float(np.dot(a - center, n)), float(np.dot(b - center, n))
    return a + (b - a) * (da / (da - db))


def run_scale(records, scene_gt, cfg, scale, seed=1234):
    """스케일 1개 실행 → 창문별 지표 dict. scale 0은 원본 그대로 (게이트)."""
    if scale == 0.0:
        stream = records
    else:
        stream = make_noisy_records(records, scale, seed, DEFAULT_MEAN_PX, DEFAULT_P95_PX,
                                    P_TAIL, DEFAULT_DROP)
    recon = reconstruct_windows(stream, scene_gt)
    wmap, failed = assemble_window_map(recon)

    start = records[0]["pose"]["position"]
    warnings = []
    wc = plan_waypoints({"position": start}, wmap, cfg, warn=warnings.append)
    warnings += crossing_warnings(wc.waypoints, scene_gt["windows"], cfg["clearance_margin"])

    gt_by_order = {w["order_index"]: w for w in scene_gt["windows"]}
    rows = []
    for i, w in enumerate(wmap["windows"]):
        gt = gt_by_order[w["order_index"]]
        approach_gt, exit_gt = gate_points(gt, cfg["d_app"], cfg["d_exit"], cfg["clearance_margin"])
        a = np.asarray(wc.waypoints[1 + 2 * i], dtype=float)
        b = np.asarray(wc.waypoints[2 + 2 * i], dtype=float)
        center = np.asarray(gt["center"], dtype=float)
        n = np.asarray(gt["normal"], dtype=float)
        n = n / np.linalg.norm(n)
        p = _pass_point(a, b, center, n)
        width_axis = np.cross(UP, n)
        width_axis = width_axis / np.linalg.norm(width_axis)
        u = abs(float(np.dot(p - center, width_axis)))
        v = abs(float(np.dot(p - center, UP)))
        margin_left = min(gt["size_wh"][0] / 2.0 - u, gt["size_wh"][1] / 2.0 - v)
        rows.append({
            "order_index": w["order_index"],
            "color": w["color"],
            "n_pairs": recon[w["order_index"]]["n_pairs"],
            "approach_err_mm": float(np.linalg.norm(a - approach_gt)) * 1000.0,
            "exit_err_mm": float(np.linalg.norm(b - exit_gt)) * 1000.0,
            "pass_u_mm": u * 1000.0,
            "pass_v_mm": v * 1000.0,
            "margin_left_mm": margin_left * 1000.0,
        })
    return {"scale": scale, "windows": rows, "failed": failed, "n_warnings": len(warnings),
            "warnings": warnings}
