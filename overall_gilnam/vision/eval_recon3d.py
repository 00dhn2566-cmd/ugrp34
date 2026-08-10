"""§5+pose 스트림 → 삼각측량 3D 복원 → scene_gt 대조 오차표.

README_stream.md 계약 레시피의 재현 구현 (태민 원본 코드는 리포에 없음):
- P = K·[R_wcᵀ | −R_wcᵀ·t_wc], cv2.triangulatePoints (2-프레임).
- 프레임쌍: 창문 4 corner 모두 vis=1 & 카메라 위치 차 ≥ 0.5m인 모든 쌍
  (max_pairs 초과 시 균등 서브샘플 — 결과에 n_pairs 기록).
- 집계: 쌍별 결과의 corner별 성분 중앙값 (꼬리 노이즈 강건 — 태민 집계 방식과
  다를 수 있음, 결과 문서에 가정으로 명시).
무노이즈 스트림에서 corner ≤ 1mm 정합 게이트를 통과해야 스윕 결과를 신뢰한다.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from noisy_stream import load_records


def quat_xyzw_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def projection_matrix(K, R_wc, t_wc):
    return K @ np.hstack([R_wc.T, (-R_wc.T @ t_wc).reshape(3, 1)])


def _intrinsics_K(scene_gt):
    i = scene_gt["intrinsics"]
    return np.array([[i["fx"], 0.0, i["cx"]], [0.0, i["fy"], i["cy"]], [0.0, 0.0, 1.0]])


def _observations(records, K, order_index):
    """해당 창문이 4 corner 모두 vis=1인 프레임 → (position, P, corners(4,2)) 리스트."""
    obs = []
    for rec in records:
        for w in rec["vision"]["windows"]:
            if w["order_index"] == order_index and all(v == 1 for v in w["corner_vis"]):
                t = np.asarray(rec["pose"]["position"], dtype=float)
                R = quat_xyzw_to_rot(rec["pose"]["orientation"])
                obs.append((t, projection_matrix(K, R, t), np.asarray(w["corners"], dtype=float)))
    return obs


def _finite_median(estimates):
    """유효 쌍 추정값 → 비유한값 제외·중앙값 집계. 반환: (중앙값(4,3) or None, 유효 개수)."""
    if not estimates:
        return None, 0
    stacked = np.stack(estimates)  # (n_pairs, 4, 3)
    finite_mask = np.isfinite(stacked).all(axis=(1, 2))  # (n_pairs,) bool
    finite_estimates = stacked[finite_mask]  # (n_finite, 4, 3)
    if len(finite_estimates) == 0:
        return None, 0
    return np.median(finite_estimates, axis=0), len(finite_estimates)


def _triangulate(obs, min_baseline_m, max_pairs):
    """모든 유효 쌍 삼각측량 → corner별 성분 중앙값 (4,3). 비유한값 제외. 반환: (estimate, n_pairs)."""
    pairs = [
        (i, j)
        for i in range(len(obs))
        for j in range(i + 1, len(obs))
        if np.linalg.norm(obs[i][0] - obs[j][0]) >= min_baseline_m
    ]
    if not pairs:
        return None, 0
    if len(pairs) > max_pairs:
        idx = np.linspace(0, len(pairs) - 1, max_pairs).astype(int)
        pairs = [pairs[k] for k in idx]
    estimates = []
    for i, j in pairs:
        X = cv2.triangulatePoints(obs[i][1], obs[j][1], obs[i][2].T, obs[j][2].T)
        estimates.append((X[:3] / X[3]).T)  # (4,3)
    return _finite_median(estimates)


def reconstruct_windows(records, scene_gt, min_baseline_m=0.5, max_pairs=2000):
    """창문별 삼각측량 복원 원본 (E2E 리허설 등 소비용).

    반환: {order_index: {"color", "corners_3d_est"(4,3) | None(복원 불가), "n_pairs"}}.
    evaluate_records는 이 결과에서 오차만 계산한다 — 수치 경로는 동일.
    """
    K = _intrinsics_K(scene_gt)
    out = {}
    for gt in scene_gt["windows"]:
        est, n_pairs = _triangulate(
            _observations(records, K, gt["order_index"]), min_baseline_m, max_pairs)
        out[gt["order_index"]] = {"color": gt["color"], "corners_3d_est": est, "n_pairs": n_pairs}
    return out


def evaluate_records(records, scene_gt, min_baseline_m=0.5, max_pairs=2000):
    recon = reconstruct_windows(records, scene_gt, min_baseline_m, max_pairs)
    results = []
    for gt in scene_gt["windows"]:
        r = recon[gt["order_index"]]
        est, n_pairs = r["corners_3d_est"], r["n_pairs"]
        if est is None:
            # 유효 쌍 없음 → 스텁 항목 (다운스트림에서 n_pairs > 0으로 필터링)
            results.append({
                "order_index": gt["order_index"],
                "color": gt["color"],
                "n_pairs": 0,
            })
            continue
        gt_corners = np.asarray(gt["corners_3d"], dtype=float)
        corner_err = np.linalg.norm(est - gt_corners, axis=1) * 1000.0  # mm
        center_err = float(np.linalg.norm(est.mean(axis=0) - np.asarray(gt["center"]))) * 1000.0
        tl, tr, br, bl = est
        w_est = (np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2.0
        h_est = (np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2.0
        results.append({
            "order_index": gt["order_index"],
            "color": gt["color"],
            "n_pairs": n_pairs,
            "corner_err_mm": [round(float(e), 3) for e in corner_err],
            "corner_err_mean_mm": round(float(corner_err.mean()), 3),
            "corner_err_max_mm": round(float(corner_err.max()), 3),
            "center_err_mm": round(center_err, 3),
            "size_err_mm": [round((w_est - gt["size_wh"][0]) * 1000.0, 3),
                            round((h_est - gt["size_wh"][1]) * 1000.0, 3)],
        })
    return results


def summarize(label, results):
    """창문별 결과 → 스트림 1개 집계 행."""
    valid = [r for r in results if r["n_pairs"] > 0]
    n_windows_failed = len(results) - len(valid)

    if not valid:
        return {
            "label": label,
            "corner_err_mean_mm": None,
            "corner_err_max_mm": None,
            "center_err_mean_mm": None,
            "size_err_mean_abs_mm": None,
            "n_windows_failed": n_windows_failed,
        }

    return {
        "label": label,
        "corner_err_mean_mm": round(float(np.mean([r["corner_err_mean_mm"] for r in valid])), 3),
        "corner_err_max_mm": round(max(r["corner_err_max_mm"] for r in valid), 3),
        "center_err_mean_mm": round(float(np.mean([r["center_err_mm"] for r in valid])), 3),
        "size_err_mean_abs_mm": round(
            float(np.mean([abs(e) for r in valid for e in r["size_err_mm"]])), 3),
        "n_windows_failed": n_windows_failed,
    }


def main():
    ap = argparse.ArgumentParser(description="스트림(들) → 삼각측량 3D 복원 오차표 (markdown)")
    ap.add_argument("--scene-gt", required=True)
    ap.add_argument("--streams", required=True, nargs="+")
    ap.add_argument("--json", help="기계 판독 결과 저장 경로 (선택)")
    args = ap.parse_args()

    scene_gt = json.loads(Path(args.scene_gt).read_text(encoding="utf-8"))
    all_out = []
    print("| 스트림 | 창문 | n_pairs | corner 평균/최대 (mm) | 중심 (mm) | 크기 w/h (mm) |")
    print("|---|---|---|---|---|---|")
    for path in args.streams:
        label = Path(path).stem
        results = evaluate_records(load_records(path), scene_gt)
        for r in results:
            if r["n_pairs"] == 0:
                # 스텁 행 — 복원 불가
                print(f"| {label} | {r['order_index']} ({r['color']}) | 0 | 복원 불가 | — | — |")
            else:
                print(f"| {label} | {r['order_index']} ({r['color']}) | {r['n_pairs']} "
                      f"| {r['corner_err_mean_mm']} / {r['corner_err_max_mm']} "
                      f"| {r['center_err_mm']} | {r['size_err_mm'][0]} / {r['size_err_mm'][1]} |")
        summary = summarize(label, results)
        all_out.append({"summary": summary, "windows": results})
        print(f"| **{label} 집계** | 창문 {len(results)}개 | — "
              f"| **{summary['corner_err_mean_mm']} / {summary['corner_err_max_mm']}** "
              f"| **{summary['center_err_mean_mm']}** | 평균절대 {summary['size_err_mean_abs_mm']} |")
    if args.json:
        Path(args.json).write_text(json.dumps(all_out, ensure_ascii=False, indent=2),
                                   encoding="utf-8")


if __name__ == "__main__":
    main()
