"""창문 통과 웨이포인트 계획기 (고전, 비학습) — 설계: docs/superpowers/specs/2026-08-08-window-waypoint-planner-design.md.

(드론 상태, 창문 3D 맵) → 창문 법선 정렬 접근·이탈점 열 → 성진 waypoints_config.
궤적 스무딩은 하류(성진 plan_waypoints) 몫 — 여기서는 웨이포인트 선정만.

입력 스키마는 state_window_interface_spec_v0_1 §6.1/§6.2 **미확정 후보안** 기준:
- 드론 상태에서 position만 사용.
- 창문 맵에서 order_index/center/normal/size_wh/passed 사용.
- normal은 접근측을 향하는 단위벡터(§3.1 관례).
- normal 부재 시 corners_3d에서 유도한다 (§3.1 잠정 확정 공식 — normal_from_corners). 둘 다 없으면 에러.
- passed 부재 시 false 취급 (소유권 미결, spec §7).

v2 (2026-08-18, plan_waypoints_v2): 윤호 prototype_demo/planner.py가 얹으려던 통과 후 거동
A~H(수평 법선 강제·게이트 z 클램프·정렬점 삽입·후진 감지·정지점·완화 재계획·구조화 경고·
호환 위임)를 전부 cfg 키(force_horizontal_normal/gate_z/align_back/stop_ahead/max_passes/
shrink)로 흡수했다 — 기본은 꺼짐(v1 동작 불변), stop_ahead가 cfg에 있으면 plan_waypoints가
자동으로 v2 경로에 위임한다. 동치는 test_v2_matches_yunho_wrapper_reference로 고정 검증됨 —
윤호 prototype_demo/planner.py는 이제 plan_waypoints_v2로 대체 가능하다.
"""

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

PLANNING_DIR = Path(__file__).resolve().parent
_REPO_ROOT = PLANNING_DIR.parents[1]
# 윤호 interface 모듈 import 경로 (수정 없음 — WaypointsConfig 조립·검증 경유용)
sys.path.insert(0, str(_REPO_ROOT / "reinforcement_yunho"))

from interface.schemas import WaypointsConfig, save_json  # noqa: E402

UP = np.array([0.0, 0.0, 1.0])


def normal_from_corners(corners_3d):
    """corner 4점(접근측에서 본 TL→TR→BR→BL) → 접근측을 향하는 단위 법선.

    winding 계약(v0.2 §4.3)의 따름정리 (spec §3.1 잠정 확정 2026-08-08):
    n̂ = normalize(cross(BL−TL, TR−TL)). 인자 순서 주의 — cross(TR−TL, BL−TL)은
    반대 방향 (reinforcement_yunho/rl/README.md의 antiparallel 지적 참조).
    """
    c = np.asarray(corners_3d, dtype=float)
    if c.shape != (4, 3):
        raise ValueError(f"corners_3d는 (4,3)이어야 함 — got {c.shape}")
    n = np.cross(c[3] - c[0], c[1] - c[0])
    n_len = float(np.linalg.norm(n))
    if n_len < 1e-9:
        raise ValueError("퇴화 corner — 법선 유도 불가")
    return n / n_len


def resolve_normal(window, force_horizontal=False):
    """창문 법선 결정: 명시 normal → corners_3d 유도 → 에러. force_horizontal이면 수평 성분만 남겨
    정규화(창문 수직 씬 가정 — 복원 법선 기울어짐으로 게이트가 지면 아래로 가던 실측 사고 방지,
    윤호 프로토타입 2026-08-18). 수평 성분이 거의 0이면 판단 불가 → 원본 유지."""
    ident = f"order_index={window.get('order_index')}({window.get('color', '?')})"
    if "normal" in window:
        n = np.asarray(window["normal"], dtype=float)
        n_len = float(np.linalg.norm(n))
        if n_len < 1e-9:
            raise ValueError(f"창문 {ident}: normal이 영벡터 — 접근측 판정 불가")
        n = n / n_len
    elif "corners_3d" in window:
        n = normal_from_corners(window["corners_3d"])
    else:
        raise ValueError(f"창문 {ident}: normal·corners_3d 모두 부재 — 접근측 판정 불가")
    if force_horizontal:
        h = np.array([n[0], n[1], 0.0])
        if np.linalg.norm(h) >= 1e-9:
            n = h / np.linalg.norm(h)
    return n


def gate_points(window, d_app, d_exit, clearance_margin, force_horizontal=False, gate_z=None):
    """창문 1개 → (접근점, 이탈점). 접근점 = center + d_app·n̂, 이탈점 = center − d_exit·n̂.

    두 점을 잇는 직선이 center를 지나므로 center 웨이포인트는 별도로 두지 않는다
    (웨이포인트 최소화 → 하류 최소시간 계획이 더 부드러움).
    force_horizontal: 법선을 수평으로 강제(윤호 실측 — 기울어진 복원 법선으로 접근점 지면 아래 → 추락).
    gate_z: (lo, hi) 지정 시 두 게이트 점의 z를 클램프.
    """
    ident = f"order_index={window.get('order_index')}({window.get('color', '?')})"
    w, h = window["size_wh"]
    if min(w, h) / 2.0 - clearance_margin <= 0:
        raise ValueError(
            f"창문 {ident}: 통과 여유 부족 — min(w,h)/2={min(w, h) / 2.0:.3f}m < margin={clearance_margin}m"
        )
    center = np.asarray(window["center"], dtype=float)
    n = resolve_normal(window, force_horizontal)
    approach, exit_ = center + d_app * n, center - d_exit * n
    if gate_z is not None:
        lo, hi = gate_z
        approach[2] = np.clip(approach[2], lo, hi)
        exit_[2] = np.clip(exit_[2], lo, hi)
    return approach, exit_


def crossing_warnings(waypoints, windows, clearance_margin):
    """연속 웨이포인트 구간이 창문 벽 평면을 개구부 밖에서 교차하면 구조화 경고 dict 리스트.

    벽의 실제 범위는 스펙에 없어 거부 판단이 불가 — v1은 경고만 (설계 §알고리즘 5).
    개구부 내부 판정: 평면 교차점을 창문 폭축(cross(UP, n̂))·높이축(UP)에 투영,
    |u| ≤ w/2−margin ∧ |v| ≤ h/2−margin.
    높이축은 world UP 사용 — 수직 창문(pitch 0) 가정, 기울어진 창문에선 세로 오프셋을 과소평가.
    사람용 문자열이 필요하면 format_warning으로 변환.
    """
    warns = []
    for w in windows:
        center = np.asarray(w["center"], dtype=float)
        try:
            n = resolve_normal(w)
        except ValueError:
            continue  # 판정 불가 — 경고 전용 기능이므로 건너뜀
        width_axis = np.cross(UP, n)
        wa_len = float(np.linalg.norm(width_axis))
        if wa_len < 1e-9:  # normal ∥ UP — 수직 창문 가정 밖, 판정 불가 → 건너뜀
            continue
        width_axis = width_axis / wa_len
        half_w = w["size_wh"][0] / 2.0 - clearance_margin
        half_h = w["size_wh"][1] / 2.0 - clearance_margin
        for seg_i, (a, b) in enumerate(zip(waypoints, waypoints[1:])):
            a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
            da, db = np.dot(a - center, n), np.dot(b - center, n)
            if da * db >= 0:  # 평면을 안 가로지름 (한 점이 평면 위인 경우 포함)
                continue
            p = a + (b - a) * (da / (da - db))  # 평면 교차점
            u, v = np.dot(p - center, width_axis), np.dot(p - center, UP)
            if abs(u) > half_w or abs(v) > half_h:
                warns.append({
                    "order_index": w["order_index"], "color": w.get("color", "?"),
                    "seg_index": seg_i, "a": a.tolist(), "b": b.tolist(),
                    "u": float(u), "v": float(v), "half_w": float(half_w), "half_h": float(half_h),
                })
    return warns


def format_warning(w):
    """구조화 경고 → 사람용 문자열 (v1 문자열 형식 유지)."""
    return (f"경고: 구간 {np.round(w['a'], 2).tolist()}→{np.round(w['b'], 2).tolist()}가 "
            f"창문 order_index={w['order_index']}({w['color']}) 평면을 "
            f"개구부 밖(u={w['u']:.2f}, v={w['v']:.2f})에서 교차")


def ordered_open_windows(window_map):
    """passed=false 창문만 order_index 오름차순으로."""
    return sorted(
        (w for w in window_map["windows"] if not w.get("passed", False)),
        key=lambda w: w["order_index"],
    )


def assemble_window_map(recon):
    """reconstruct_windows 결과 → §6.2 창문 맵. 복원 불가 창문은 제외하고 failed로 보고.

    실전 소비처(윤호 pipeline_demo) 생겨 planning으로 승격 (2026-08-18).
    """
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


def load_planner_config(path):
    """planner_limits.yaml → dict (d_app/d_exit/clearance_margin/limits/dt)."""
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


@dataclass
class Plan:
    """v2 계획 결과. ok = 경고 없음 ∧ 후진 없음."""
    waypoints: list = field(default_factory=list)
    labels: list = field(default_factory=list)
    warnings: list = field(default_factory=list)   # crossing_warnings dict
    passes: int = 0
    shrink: float = 1.0
    backtrack_m: float = 0.0

    @property
    def ok(self):
        return not self.warnings and self.backtrack_m <= 1e-6


def _inside_opening(pt, window, n, margin):
    center = np.asarray(window["center"], dtype=float)
    wa = np.cross(UP, n)
    if np.linalg.norm(wa) < 1e-9:
        return True
    wa = wa / np.linalg.norm(wa)
    d = pt - center
    return (abs(d @ wa) <= window["size_wh"][0] / 2.0 - margin
            and abs(d @ UP) <= window["size_wh"][1] / 2.0 - margin)


def _build_v2(windows, start, d_app, d_exit, margin, force_h, gate_z, align_back, stop_ahead):
    pts, labels, backtrack, prev_exit = [np.asarray(start, dtype=float)], ["start"], 0.0, None
    for k, w in enumerate(windows):
        ap, ex = gate_points(w, d_app, d_exit, margin, force_horizontal=force_h, gate_z=gate_z)
        n = resolve_normal(w, force_h)
        if prev_exit is not None:
            center = np.asarray(w["center"], dtype=float)
            da, db = float((prev_exit - center) @ n), float((ap - center) @ n)
            if da * db < 0:                                     # 이미 평면을 가로지름
                hit = prev_exit + (ap - prev_exit) * (da / (da - db))
                if not _inside_opening(hit, w, n, margin):
                    pts.append(ap + align_back * n); labels.append(f"align{k}")
            back = float((prev_exit - ap) @ (-n))               # 진행방향 = −n
            if back > 0:
                backtrack = max(backtrack, back)
        pts.append(ap); labels.append(f"approach{k}")
        pts.append(ex); labels.append(f"exit{k}")
        prev_exit = ex
    n_last = resolve_normal(windows[-1], force_h)
    pts.append(prev_exit - stop_ahead * n_last); labels.append("stop")
    return pts, labels, backtrack


def plan_waypoints_v2(drone_state, window_map, cfg):
    """v2: 후진 완화·정렬점·정지점·재계획 (윤호 프로토타입 래퍼 요구사항 흡수, 2026-08-18)."""
    windows = ordered_open_windows(window_map)
    if not windows:
        raise ValueError("열린 창문이 없음 — 계획할 대상 없음")
    force_h = bool(cfg.get("force_horizontal_normal", False))
    gate_z = tuple(cfg["gate_z"]) if cfg.get("gate_z") else None
    shrinks = list(cfg.get("shrink", [1.0]))
    best = None
    for i in range(int(cfg.get("max_passes", 1))):
        s = shrinks[min(i, len(shrinks) - 1)]
        pts, labels, back = _build_v2(windows, drone_state["position"], cfg["d_app"] * s, cfg["d_exit"] * s,
                                      cfg["clearance_margin"], force_h, gate_z,
                                      cfg.get("align_back", 0.45), cfg.get("stop_ahead", 0.6))
        warns = crossing_warnings([p.tolist() for p in pts], windows, cfg["clearance_margin"])
        plan = Plan([[float(v) for v in p] for p in pts], labels, warns, i + 1, s, back)
        if best is None or (len(plan.warnings), plan.backtrack_m) < (len(best.warnings), best.backtrack_m):
            best = plan
        if plan.ok:
            break
    return best


def plan_waypoints(drone_state, window_map, cfg, warn=print):
    """(드론 상태, 창문 맵) → WaypointsConfig. 웨이포인트 = [현 위치] + [접근ᵢ, 이탈ᵢ]…

    cfg에 stop_ahead가 있으면 v2(plan_waypoints_v2)로 위임 — 정지점·정렬점·재계획 포함.
    없으면 v1 경로 그대로(호환 규칙).
    """
    if "stop_ahead" in cfg:
        plan = plan_waypoints_v2(drone_state, window_map, cfg)
        points = plan.waypoints
        for w in plan.warnings:
            warn(format_warning(w))
    else:
        windows = ordered_open_windows(window_map)
        if not windows:
            raise ValueError("열린 창문이 없음 — 계획할 대상 없음")
        points = [[float(c) for c in drone_state["position"]]]
        for w in windows:
            approach, exit_ = gate_points(w, cfg["d_app"], cfg["d_exit"], cfg["clearance_margin"])
            points.append([float(c) for c in approach])
            points.append([float(c) for c in exit_])
        for msg in crossing_warnings(points, windows, cfg["clearance_margin"]):
            warn(format_warning(msg))
    wc = WaypointsConfig(waypoints=points, limits=dict(cfg["limits"]), dt=float(cfg["dt"]))
    wc.validate()  # 성진 스키마 검증 — 실패 시 여기서 즉시 드러남
    return wc


def main():
    ap = argparse.ArgumentParser(description="(드론 상태, 창문 맵) JSON → 성진 waypoints_config JSON")
    ap.add_argument("--state", required=True, help="§6.1 드론 상태 JSON (position 사용)")
    ap.add_argument("--window-map", required=True, help="§6.2 창문 맵 JSON")
    ap.add_argument("--out", required=True, help="출력 waypoints_config JSON 경로")
    ap.add_argument("--config", default=str(PLANNING_DIR / "planner_limits.yaml"))
    args = ap.parse_args()

    with open(args.state, encoding="utf-8") as f:
        drone_state = json.load(f)
    with open(args.window_map, encoding="utf-8") as f:
        window_map = json.load(f)
    wc = plan_waypoints(drone_state, window_map, load_planner_config(args.config), warn=lambda m: print(m, file=sys.stderr))
    save_json(wc, args.out)
    print(f"waypoints={len(wc.waypoints)} -> {args.out}")


if __name__ == "__main__":
    main()
