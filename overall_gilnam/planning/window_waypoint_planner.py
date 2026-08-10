"""창문 통과 웨이포인트 계획기 (고전, 비학습) — 설계: docs/superpowers/specs/2026-08-08-window-waypoint-planner-design.md.

(드론 상태, 창문 3D 맵) → 창문 법선 정렬 접근·이탈점 열 → 성진 waypoints_config.
궤적 스무딩은 하류(성진 plan_waypoints) 몫 — 여기서는 웨이포인트 선정만.

입력 스키마는 state_window_interface_spec_v0_1 §6.1/§6.2 **미확정 후보안** 기준:
- 드론 상태에서 position만 사용.
- 창문 맵에서 order_index/center/normal/size_wh/passed 사용.
- normal은 접근측을 향하는 단위벡터(§3.1 관례).
- normal 부재 시 corners_3d에서 유도한다 (§3.1 잠정 확정 공식 — normal_from_corners). 둘 다 없으면 에러.
- passed 부재 시 false 취급 (소유권 미결, spec §7).
"""

import argparse
import json
import sys
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


def gate_points(window, d_app, d_exit, clearance_margin):
    """창문 1개 → (접근점, 이탈점). 접근점 = center + d_app·n̂, 이탈점 = center − d_exit·n̂.

    두 점을 잇는 직선이 center를 지나므로 center 웨이포인트는 별도로 두지 않는다
    (웨이포인트 최소화 → 하류 최소시간 계획이 더 부드러움).
    """
    ident = f"order_index={window.get('order_index')}({window.get('color', '?')})"
    w, h = window["size_wh"]
    if min(w, h) / 2.0 - clearance_margin <= 0:
        raise ValueError(
            f"창문 {ident}: 통과 여유 부족 — min(w,h)/2={min(w, h) / 2.0:.3f}m < margin={clearance_margin}m"
        )
    center = np.asarray(window["center"], dtype=float)
    if "normal" in window:
        n = np.asarray(window["normal"], dtype=float)
        n_len = float(np.linalg.norm(n))
        if n_len < 1e-9:
            raise ValueError(f"창문 {ident}: normal이 영벡터 — 접근측 판정 불가")
        n = n / n_len
    elif "corners_3d" in window:
        n = normal_from_corners(window["corners_3d"])  # 부호 확정 공식 폴백
    else:
        raise ValueError(f"창문 {ident}: normal·corners_3d 모두 부재 — 접근측 판정 불가")
    return center + d_app * n, center - d_exit * n


def crossing_warnings(waypoints, windows, clearance_margin):
    """연속 웨이포인트 구간이 창문 벽 평면을 개구부 밖에서 교차하면 경고 문자열 리스트.

    벽의 실제 범위는 스펙에 없어 거부 판단이 불가 — v1은 경고만 (설계 §알고리즘 5).
    개구부 내부 판정: 평면 교차점을 창문 폭축(cross(UP, n̂))·높이축(UP)에 투영,
    |u| ≤ w/2−margin ∧ |v| ≤ h/2−margin.
    높이축은 world UP 사용 — 수직 창문(pitch 0) 가정, 기울어진 창문에선 세로 오프셋을 과소평가.
    """
    warns = []
    for w in windows:
        center = np.asarray(w["center"], dtype=float)
        if "normal" in w:
            n = np.asarray(w["normal"], dtype=float)
        elif "corners_3d" in w:
            try:
                n = normal_from_corners(w["corners_3d"])
            except ValueError:
                continue  # 판정 불가 — 경고 전용 기능이므로 건너뜀
        else:
            continue
        n_len = float(np.linalg.norm(n))
        if n_len < 1e-9:
            continue
        n = n / n_len
        width_axis = np.cross(UP, n)
        wa_len = float(np.linalg.norm(width_axis))
        if wa_len < 1e-9:  # normal ∥ UP — 수직 창문 가정 밖, 판정 불가 → 건너뜀
            continue
        width_axis = width_axis / wa_len
        half_w = w["size_wh"][0] / 2.0 - clearance_margin
        half_h = w["size_wh"][1] / 2.0 - clearance_margin
        for a, b in zip(waypoints, waypoints[1:]):
            a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
            da, db = np.dot(a - center, n), np.dot(b - center, n)
            if da * db >= 0:  # 평면을 안 가로지름 (한 점이 평면 위인 경우 포함)
                continue
            p = a + (b - a) * (da / (da - db))  # 평면 교차점
            u, v = np.dot(p - center, width_axis), np.dot(p - center, UP)
            if abs(u) > half_w or abs(v) > half_h:
                warns.append(
                    f"경고: 구간 {np.round(a, 2).tolist()}→{np.round(b, 2).tolist()}가 "
                    f"창문 order_index={w['order_index']}({w.get('color', '?')}) 평면을 "
                    f"개구부 밖(u={u:.2f}, v={v:.2f})에서 교차"
                )
    return warns


def ordered_open_windows(window_map):
    """passed=false 창문만 order_index 오름차순으로."""
    return sorted(
        (w for w in window_map["windows"] if not w.get("passed", False)),
        key=lambda w: w["order_index"],
    )


def load_planner_config(path):
    """planner_limits.yaml → dict (d_app/d_exit/clearance_margin/limits/dt)."""
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def plan_waypoints(drone_state, window_map, cfg, warn=print):
    """(드론 상태, 창문 맵) → WaypointsConfig. 웨이포인트 = [현 위치] + [접근ᵢ, 이탈ᵢ]…"""
    windows = ordered_open_windows(window_map)
    if not windows:
        raise ValueError("열린 창문이 없음 — 계획할 대상 없음")
    points = [[float(c) for c in drone_state["position"]]]
    for w in windows:
        approach, exit_ = gate_points(w, cfg["d_app"], cfg["d_exit"], cfg["clearance_margin"])
        points.append([float(c) for c in approach])
        points.append([float(c) for c in exit_])
    for msg in crossing_warnings(points, windows, cfg["clearance_margin"]):
        warn(msg)
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
