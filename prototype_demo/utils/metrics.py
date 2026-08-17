"""복원 결과 채점 — taemin_demo / render_status / compare_weights 에 흩어져 있던 계산.

    from utils import metrics
    rows = metrics.score(results, layout)
    metrics.print_rows(rows)

results 는 태민 노드(또는 overrides.recon_rays)가 내는 dict 리스트를 그대로 받는다:
``{order_index, color, center_w, corners_w, width, height, n_obs, min_parallax_deg}``
"""
from __future__ import annotations

from typing import Dict, List, Sequence

import numpy as np

COLORS = ("red", "green", "blue")


def score(results: Sequence[dict], layout: Sequence[dict]) -> List[dict]:
    """복원 결과 × GT → 창문별 오차 행. 미복원 창문도 ok=False 행으로 남긴다."""
    by_oi = {r["order_index"]: r for r in results}
    rows = []
    for w in layout:
        oi = w["order_index"]
        r = by_oi.get(oi)
        if r is None:
            rows.append({"order_index": oi, "color": w["color"], "ok": False,
                         "center_mm": None, "w_mm": None, "h_mm": None,
                         "size_mm": None, "n_obs": 0, "parallax": None})
            continue
        cen = float(np.linalg.norm(np.asarray(r["center_w"], float)
                                   - np.asarray(w["center"], float))) * 1000
        we = abs(float(r["width"]) - float(w["ow"])) * 1000
        he = abs(float(r["height"]) - float(w["oh"])) * 1000
        rows.append({"order_index": oi, "color": w["color"], "ok": True,
                     "center_mm": cen, "w_mm": we, "h_mm": he,
                     "size_mm": max(we, he), "n_obs": int(r.get("n_obs", 0)),
                     "parallax": r.get("min_parallax_deg")})
    return rows


def print_rows(rows: Sequence[dict], indent: str = "  ") -> int:
    """행 출력. 반환: 복원 성공 개수."""
    ok = 0
    for r in rows:
        if not r["ok"]:
            print(f"{indent}#{r['order_index']} {r['color']:6s}  복원 실패")
            continue
        ok += 1
        par = f"{r['parallax']:.1f}" if r["parallax"] is not None else "  ? "
        print(f"{indent}#{r['order_index']} {r['color']:6s}  "
              f"center 오차 {r['center_mm']:7.1f} mm   "
              f"크기 오차 {r['w_mm']:6.1f} x {r['h_mm']:6.1f} mm   "
              f"관측 {r['n_obs']:4d}  시차각 {par} deg")
    print(f"{indent}{ok}/{len(rows)} 창문 복원")
    return ok


def aggregate(all_rows: Sequence[Sequence[dict]]) -> Dict:
    """여러 시드 결과 묶음 → 요약 통계.

    center 오차는 평균보다 **중앙값**을 먼저 본다. 한 창문이 색 오분류로 1.5m 튀면
    평균이 통째로 끌려가서 모델 개선이 안 보인다.
    """
    flat = [r for rows in all_rows for r in rows]
    got = [r for r in flat if r["ok"]]
    cen = np.array([r["center_mm"] for r in got]) if got else np.array([np.nan])
    siz = np.array([r["size_mm"] for r in got]) if got else np.array([np.nan])
    return {"n_windows": len(flat), "n_ok": len(got),
            "center_median": float(np.median(cen)), "center_mean": float(np.mean(cen)),
            "center_p90": float(np.percentile(cen, 90)),
            "size_median": float(np.median(siz)),
            "n_gross": int(np.sum(cen > 500))}     # 500mm 초과 = 사실상 다른 창문


def print_summary(named: Dict[str, Dict]) -> None:
    """{'v1': agg, 'v2': agg} → 비교표. 350mm 는 planner clearance."""
    w = max(12, max((len(t) for t in named), default=12) + 1)
    print(f"{'':{w}s}{'복원':>9s}{'center 중앙값':>15s}{'center p90':>13s}"
          f"{'center 평균':>13s}{'size 중앙값':>13s}{'>500mm':>9s}")
    for tag, d in named.items():
        print(f"{tag:{w}s}{d['n_ok']:4d}/{d['n_windows']:<4d}"
              f"{d['center_median']:12.0f} mm{d['center_p90']:10.0f} mm"
              f"{d['center_mean']:10.0f} mm{d['size_median']:10.0f} mm"
              f"{d['n_gross']:9d}")


# --------------------------------------------------------------------------- #
# 색 혼동행렬 — 복원 오차의 최대 원인이 색 오분류라서 따로 잰다
# --------------------------------------------------------------------------- #
def confusion(samples: Sequence[dict], layout: Sequence[dict], intr: Dict,
              iou_min: float = 0.3) -> Dict:
    """검출 박스를 GT 투영 박스와 IoU 로 맞춰 색 혼동행렬을 만든다.

    반환: {"mat": {gt색: {pred색: n}}, "missed": {gt색: n}, "spurious": n, "acc": float}
    """
    from sim.pybullet_stream import window_corners_gt
    mat = {g: {p: 0 for p in COLORS} for g in COLORS}
    missed = {g: 0 for g in COLORS}
    spurious = 0

    for s in samples:
        det = s.get("detection")
        R = _rot(s["q_WI_xyzw"])
        c = np.asarray(s["p_WI"], float)
        gt_boxes = []
        for w in layout:
            uv = _project(window_corners_gt(w["center"], w["ow"], w["oh"]), R, c, intr)
            if uv is None:
                continue
            gt_boxes.append((w["color"], _bbox(uv)))
        used = set()
        preds = det["windows"] if det else []
        for pw in preds:
            pb = _bbox(np.asarray(pw["corners"], float))
            best, best_iou = None, iou_min
            for k, (gc, gb) in enumerate(gt_boxes):
                if k in used:
                    continue
                v = _iou(pb, gb)
                if v > best_iou:
                    best, best_iou = k, v
            if best is None:
                spurious += 1
            else:
                used.add(best)
                mat[gt_boxes[best][0]][pw["color"]] += 1
        for k, (gc, _gb) in enumerate(gt_boxes):
            if k not in used:
                missed[gc] += 1

    hit = sum(mat[g][g] for g in COLORS)
    tot = sum(sum(mat[g].values()) for g in COLORS) + sum(missed.values())
    return {"mat": mat, "missed": missed, "spurious": spurious,
            "acc": hit / tot if tot else 0.0, "total": tot}


def print_confusion(cm: Dict, title: str = "") -> None:
    if title:
        print(title)
    print(f"{'':8s}" + "".join(f"{'→'+p:>8s}" for p in COLORS) + f"{'미검출':>9s}")
    for g in COLORS:
        print(f"{g:8s}" + "".join(f"{cm['mat'][g][p]:8d}" for p in COLORS)
              + f"{cm['missed'][g]:8d}")
    wrong = sum(cm["mat"][g][p] for g in COLORS for p in COLORS if g != p)
    print(f"정확도 {cm['acc']*100:5.1f}%   오분류 {wrong:3d}   "
          f"미검출 {sum(cm['missed'].values()):3d} / {cm['total']}   "
          f"허위검출 {cm['spurious']}")


def _rot(q) -> np.ndarray:
    from sim.pybullet_stream import _rot_to_quat_xyzw   # noqa: F401  (형식 확인용)
    from eval_recon3d import quat_xyzw_to_rot
    return quat_xyzw_to_rot(np.asarray(q, float))


def _project(pts_w: np.ndarray, R_WI: np.ndarray, p_WI: np.ndarray, intr: Dict):
    """월드 점 → 픽셀. 하나라도 카메라 뒤면 None (부분 가림은 여기서 안 다룸)."""
    from module import contract
    T_IC = contract.T_imu_cam()
    R_WC = R_WI @ T_IC[:3, :3]
    c_W = p_WI + R_WI @ T_IC[:3, 3]
    pc = (pts_w - c_W) @ R_WC
    if np.any(pc[:, 2] <= 1e-6):
        return None
    u = intr["fx"] * pc[:, 0] / pc[:, 2] + intr["cx"]
    v = intr["fy"] * pc[:, 1] / pc[:, 2] + intr["cy"]
    uv = np.stack([u, v], axis=1)
    if uv[:, 0].max() < 0 or uv[:, 0].min() > intr["width"] or \
       uv[:, 1].max() < 0 or uv[:, 1].min() > intr["height"]:
        return None
    return uv


def _bbox(uv: np.ndarray):
    return (uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max())


def _iou(a, b) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2]-a[0]) * (a[3]-a[1]) + (b[2]-b[0]) * (b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0
