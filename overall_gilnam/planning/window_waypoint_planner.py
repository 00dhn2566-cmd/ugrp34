"""창문 통과 웨이포인트 계획기 (고전, 비학습) — 설계: docs/superpowers/specs/2026-08-08-window-waypoint-planner-design.md.

(드론 상태, 창문 3D 맵) → 창문 법선 정렬 접근·이탈점 열 → 성진 waypoints_config.
궤적 스무딩은 하류(성진 plan_waypoints) 몫 — 여기서는 웨이포인트 선정만.

입력 스키마는 state_window_interface_spec_v0_1 §6.1/§6.2 **미확정 후보안** 기준:
- 드론 상태에서 position만 사용.
- 창문 맵에서 order_index/center/normal/size_wh/passed 사용.
- normal은 접근측을 향하는 단위벡터(§3.1 관례). 부재 시 에러 — corner 유도
  법선은 ± 방향 관례 미확정이라 접근측 판정 불가 (rl/README.md 동일 지적).
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


def gate_points(window, d_app, d_exit, clearance_margin):
    """창문 1개 → (접근점, 이탈점). 접근점 = center + d_app·n̂, 이탈점 = center − d_exit·n̂.

    두 점을 잇는 직선이 center를 지나므로 center 웨이포인트는 별도로 두지 않는다
    (웨이포인트 최소화 → 하류 최소시간 계획이 더 부드러움).
    """
    ident = f"order_index={window.get('order_index')}({window.get('color', '?')})"
    if "normal" not in window:
        raise ValueError(f"창문 {ident}: normal 부재 — 접근측 판정 불가 (spec §3.1 관례 미확정)")
    w, h = window["size_wh"]
    if min(w, h) / 2.0 - clearance_margin < 0:
        raise ValueError(
            f"창문 {ident}: 통과 여유 부족 — min(w,h)/2={min(w, h) / 2.0:.3f}m < margin={clearance_margin}m"
        )
    center = np.asarray(window["center"], dtype=float)
    n = np.asarray(window["normal"], dtype=float)
    n = n / np.linalg.norm(n)
    return center + d_app * n, center - d_exit * n


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


def plan_waypoints(drone_state, window_map, cfg):
    """(드론 상태, 창문 맵) → WaypointsConfig. 웨이포인트 = [현 위치] + [접근ᵢ, 이탈ᵢ]…"""
    windows = ordered_open_windows(window_map)
    if not windows:
        raise ValueError("열린 창문이 없음 — 계획할 대상 없음")
    points = [[float(c) for c in drone_state["position"]]]
    for w in windows:
        approach, exit_ = gate_points(w, cfg["d_app"], cfg["d_exit"], cfg["clearance_margin"])
        points.append([float(c) for c in approach])
        points.append([float(c) for c in exit_])
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
    wc = plan_waypoints(drone_state, window_map, load_planner_config(args.config))
    save_json(wc, args.out)
    print(f"waypoints={len(wc.waypoints)} -> {args.out}")


if __name__ == "__main__":
    main()
