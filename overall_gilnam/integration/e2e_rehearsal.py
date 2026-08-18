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
    PLANNING_DIR, UP, assemble_window_map, crossing_warnings, format_warning,
    gate_points, load_planner_config, plan_waypoints, plan_waypoints_v2,
)

SAMPLE = GILNAM / "vision" / "sample_stream"
SCALES = [0.0, 0.5, 1.0, 1.5, 2.0]


def load_inputs():
    """샘플 스트림·scene_gt·계획기 설정 로드."""
    records = load_records(SAMPLE / "sample_stream.jsonl")
    scene_gt = json.loads((SAMPLE / "scene_gt.json").read_text(encoding="utf-8"))
    cfg = load_planner_config(PLANNING_DIR / "planner_limits.yaml")
    return records, scene_gt, cfg


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
    # 라벨 기반 게이트점 조회 — align{k} 삽입(v2)이 있어도 인덱스 산수 없이 안전 (final-review Fix 1)
    plan = plan_waypoints_v2({"position": start}, wmap, cfg)
    wc = plan_waypoints({"position": start}, wmap, cfg, warn=lambda m: None)
    warnings = [format_warning(x) for x in plan.warnings]
    warnings += [format_warning(x) for x in
                 crossing_warnings(wc.waypoints, scene_gt["windows"], cfg["clearance_margin"])]

    gt_by_order = {w["order_index"]: w for w in scene_gt["windows"]}
    rows = []
    for i, w in enumerate(wmap["windows"]):
        gt = gt_by_order[w["order_index"]]
        approach_gt, exit_gt = gate_points(gt, cfg["d_app"], cfg["d_exit"], cfg["clearance_margin"])
        a = np.asarray(plan.waypoints[plan.labels.index(f"approach{i}")], dtype=float)
        b = np.asarray(plan.waypoints[plan.labels.index(f"exit{i}")], dtype=float)
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


def main():
    ap = argparse.ArgumentParser(description="E2E 리허설: 스케일별 복원→계획 품질 표 (markdown)")
    ap.add_argument("--scales", default=",".join(str(s) for s in SCALES))
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--json", help="기계 판독 결과 저장 경로 (선택)")
    args = ap.parse_args()

    records, scene_gt, cfg = load_inputs()
    results = []
    print("| scale | 창문 | n_pairs | 게이트 오차 접근/이탈 (mm) | 통과점 u/v (mm) | 잔여 여유 (mm) |")
    print("|---|---|---|---|---|---|")
    for scale in [float(s) for s in args.scales.split(",")]:
        res = run_scale(records, scene_gt, cfg, scale, args.seed)
        results.append(res)
        for w in res["windows"]:
            print(f"| x{scale:g} | {w['order_index']} ({w['color']}) | {w['n_pairs']} "
                  f"| {w['approach_err_mm']:.1f} / {w['exit_err_mm']:.1f} "
                  f"| {w['pass_u_mm']:.1f} / {w['pass_v_mm']:.1f} | {w['margin_left_mm']:.1f} |")
        worst = min((w["margin_left_mm"] for w in res["windows"]), default=float("nan"))
        note = f"경고 {res['n_warnings']}건" + (f", 복원 불가 {res['failed']}" if res["failed"] else "")
        print(f"| **x{scale:g} 요약** | 창문 {len(res['windows'])}개 | — | — | — "
              f"| **최소 {worst:.1f}** ({note}) |")
    for res in results:
        for msg in res["warnings"]:
            print(f"  ! x{res['scale']:g}: {msg}", file=sys.stderr)
    if args.json:
        Path(args.json).write_text(json.dumps(results, ensure_ascii=False, indent=2),
                                   encoding="utf-8")


if __name__ == "__main__":
    main()
