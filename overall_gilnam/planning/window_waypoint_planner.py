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

import numpy as np

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
