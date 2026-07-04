"""corner 픽셀 오차 평가 — model_decisions #7 목표치(평균 ≤3px, 원거리 ≤5px 가안) 검증.

코어(evaluate/report)는 ultralytics 없이 동작한다. records는 이미 GT↔예측이
짝지어진 상태로 받는다(매칭은 호출측 책임) — 코어는 계산만 한다.
CLI 모드(--model --dataset)만 ultralytics를 임포트한다 (실행 단계 전용 의존성).

    python eval_corners.py --model runs/pose/train/weights/best.pt --dataset window_dataset/toy/
"""

import argparse
import json
from pathlib import Path

import numpy as np

from gt_stream import parse_label_line

# CLI에서 GT↔예측 매칭 시 center 거리 상한 (720p px).
# single_cls 학습이라 예측에 order_index가 없어 center 최근접으로 짝짓는다.
MATCH_MAX_PX = 150.0


def _bin_key(dist, lo, hi):
    if dist < lo:
        return f"<{lo:g}m"
    if dist <= hi:
        return f"{lo:g}-{hi:g}m"
    return f">{hi:g}m"


def _stats(errors, n_windows):
    if not errors:
        return {"n": n_windows, "mean_px": None, "p95_px": None}
    arr = np.asarray(errors)
    return {"n": n_windows, "mean_px": float(arr.mean()), "p95_px": float(np.percentile(arr, 95))}


def evaluate(records, bins=(3.0, 6.0)):
    """records = [{gt_corners, pred_corners(각 4x2, 720p px), distance_m}].

    pred_corners가 None이면 매칭 실패로 집계하고 통계에서 제외.
    반환: {"overall": {n, mean_px, p95_px}, "bins": {구간: 동일}, "unmatched": int}
    오차 = corner별 유클리드 거리 (창문당 4개를 모두 풀링), n = 창문 수.
    """
    lo, hi = bins
    keys = [f"<{lo:g}m", f"{lo:g}-{hi:g}m", f">{hi:g}m"]
    pooled = {k: {"errors": [], "n": 0} for k in ["overall"] + keys}
    unmatched = 0
    for rec in records:
        if rec["pred_corners"] is None:
            unmatched += 1
            continue
        err = np.linalg.norm(
            np.asarray(rec["pred_corners"], float) - np.asarray(rec["gt_corners"], float), axis=1
        )
        for k in ("overall", _bin_key(rec["distance_m"], lo, hi)):
            pooled[k]["errors"].extend(err.tolist())
            pooled[k]["n"] += 1
    result = {k: _stats(v["errors"], v["n"]) for k, v in pooled.items()}
    return {"overall": result["overall"], "bins": {k: result[k] for k in keys}, "unmatched": unmatched}


def report(result, mean_target=3.0, far_target=5.0):
    """목표치 대비 PASS/FAIL 리포트 문자열 (목표치 가안 — model_decisions #7)."""
    lines = ["구간       n    mean_px   p95_px"]
    rows = [("전체", result["overall"])] + list(result["bins"].items())
    far_key = list(result["bins"])[-1]
    for name, s in rows:
        mean = "-" if s["mean_px"] is None else f"{s['mean_px']:.2f}"
        p95 = "-" if s["p95_px"] is None else f"{s['p95_px']:.2f}"
        lines.append(f"{name:<9} {s['n']:>3}  {mean:>8}  {p95:>7}")
    lines.append(f"매칭 실패: {result['unmatched']}")

    overall_mean = result["overall"]["mean_px"]
    far_mean = result["bins"][far_key]["mean_px"]
    ok_mean = overall_mean is not None and overall_mean <= mean_target
    ok_far = far_mean is None or far_mean <= far_target  # 원거리 표본 없으면 판정 제외
    lines.append(f"목표 mean<={mean_target:g}px: {'PASS' if ok_mean else 'FAIL'}")
    lines.append(f"목표 원거리({far_key})<={far_target:g}px: {'PASS' if ok_far else 'FAIL'}")
    return "\n".join(lines)


def _load_meta(dataset_dir):
    """meta.jsonl → {이미지 stem: {order_index: distance_m}}"""
    meta = {}
    with open(dataset_dir / "meta.jsonl", encoding="utf-8") as f:
        for line in f:
            rec = json.loads(line)
            meta[Path(rec["image"]).stem] = {
                w["order_index"]: w["distance_m"] for w in rec["windows"]
            }
    return meta


def _collect_records(model, dataset_dir, split="val"):
    """val 이미지 예측 → GT(라벨)와 center 최근접 매칭 → evaluate용 records.

    decision #6: 원본 720p 프레임을 그대로 입력하면 라이브러리가 letterbox
    역변환 후 원본 좌표로 keypoints를 반환한다.
    """
    meta = _load_meta(dataset_dir)
    records = []
    for img_path in sorted((dataset_dir / "images" / split).glob("*.png")):
        label_path = dataset_dir / "labels" / split / (img_path.stem + ".txt")
        gts = [parse_label_line(l) for l in label_path.read_text().splitlines() if l.strip()]
        r = model(str(img_path), verbose=False)[0]
        preds = []  # [(center(2,), corners(4,2))]
        if r.keypoints is not None and len(r.keypoints) > 0:
            for kp in r.keypoints.xy.cpu().numpy():
                preds.append((kp.mean(axis=0), kp))
        used = set()
        for gt in gts:
            gt_c = np.asarray(gt["corners"], float)
            center = gt_c.mean(axis=0)
            best_i, best_d = None, MATCH_MAX_PX
            for i, (pc, _) in enumerate(preds):
                d = float(np.linalg.norm(pc - center))
                if i not in used and d < best_d:
                    best_i, best_d = i, d
            pred_corners = None
            if best_i is not None:
                used.add(best_i)
                pred_corners = preds[best_i][1]
            records.append(
                {
                    "gt_corners": gt_c,
                    "pred_corners": pred_corners,
                    "distance_m": meta[img_path.stem][gt["order_index"]],
                }
            )
    return records


def main():
    parser = argparse.ArgumentParser(description="corner 픽셀 오차 평가 (val split)")
    parser.add_argument("--model", required=True, help="학습된 .pt 가중치")
    parser.add_argument("--dataset", required=True, help="meta.jsonl 포함 데이터셋 루트")
    args = parser.parse_args()
    from ultralytics import YOLO  # 실행 단계 전용 의존성 — 코어·테스트는 불필요

    records = _collect_records(YOLO(args.model), Path(args.dataset))
    print(report(evaluate(records)))


if __name__ == "__main__":
    main()
