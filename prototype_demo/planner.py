"""통과 후 거동까지 정하는 간단 플래너 — 길남 `plan_waypoints` 가 안 하는 부분.

    from planner import plan
    res = plan(windows, start=(0,0,1), cfg=load_planner_config(...))

원본이 하는 일 (window_waypoint_planner.py:141-145)
--------------------------------------------------
    points = [start]
    for w in windows:
        points += [approach, exit]

창문마다 게이트점 2개를 이어붙이는 게 전부다. ``exit_k`` 와 ``approach_{k+1}`` 사이에
아무것도 없고, ``crossing_warnings`` 는 위반을 **경고만** 한다 (그 파일 주석에
"v1은 경고만" 이라고 적혀 있다). 즉 통과 후 거동이 결정돼 있지 않다.

여기서 정한 것 4가지
--------------------
1. **standoff 유지**  d_app=1.5 / d_exit=1.0 을 그대로 쓴다. 다만 인접 간격이
   2.5 m 미만이면 이탈점이 다음 접근점을 지나쳐 후진이 생긴다. 그때는 그 쌍에
   한해 standoff 를 줄여 재계획한다 (4번).
2. **창문 사이는 통과축 정렬**  ``exit_k -> approach_{k+1}`` 직선이 다음 창문
   평면을 개구부 밖에서 뚫으면, 다음 창문의 법선 위에 정렬점을 하나 끼워 넣어
   법선 방향으로 들어가게 만든다.
3. **마지막 창문 통과 후 정지**  마지막 이탈점에서 법선 방향으로 STOP_AHEAD_M
   더 나간 뒤 그 자리를 종점으로 둔다. 종점이 곧 정지점이다.
4. **위반 시 재계획**  ``crossing_warnings`` 가 비지 않으면 완화 단계를 한 칸
   올려 다시 푼다. MAX_PASSES 번 안에 못 없애면 남은 경고를 그대로 보고한다
   (조용히 성공한 척하지 않는다).

**이건 임시 플래너다.** 제대로 하려면 창문 통과 자체를 최적화 문제로 놓고 풀거나
강화학습으로 정책을 뽑아야 한다. 여기서는 게이트점을 잇는 절충안까지만 간다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Sequence

import numpy as np

STOP_AHEAD_M = 0.6      # 마지막 이탈점에서 더 나간 뒤 정지할 거리
ALIGN_BACK_M = 0.45     # 통과축 정렬점을 접근점보다 얼마나 더 뒤에 둘지
MAX_PASSES = 4          # 재계획 최대 시도
SHRINK = (1.0, 0.75, 0.55, 0.4)   # 패스별 standoff 축소 계수


@dataclass
class Plan:
    waypoints: List[List[float]] = field(default_factory=list)
    labels: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    passes: int = 0
    shrink: float = 1.0
    backtrack_m: float = 0.0

    @property
    def ok(self) -> bool:
        return not self.warnings and self.backtrack_m <= 1e-6


#: 창문이 전부 수직이라는 씬 사전지식을 쓴다 (법선의 z 성분 = 0).
#: 복원된 코너에서 뽑은 법선은 코너 하나만 틀어져도 크게 기운다. 실측: 중심 오차가
#: 167~178 mm 로 멀쩡한데 법선 z 성분이 −0.57 (35° 아래) 이라 1.5 m 뻗은 접근
#: 게이트가 z = −0.02 (지면 아래) 로 갔고, 드론이 그리로 향하다 추락했다.
#: 수평으로 눕히면 그 오차원이 통째로 사라진다.
FORCE_HORIZONTAL_NORMAL = True

#: 게이트 z 안전대역 [m] — 법선을 눕혀도 남는 오차에 대한 안전망.
GATE_Z_MIN, GATE_Z_MAX = 0.5, 1.9


def _normal(w: Dict) -> np.ndarray:
    from window_waypoint_planner import normal_from_corners
    if "normal" in w:
        n = np.asarray(w["normal"], float)
    else:
        n = normal_from_corners(np.asarray(w["corners_3d"], float))
    if FORCE_HORIZONTAL_NORMAL:
        n = np.array([n[0], n[1], 0.0])
        if np.linalg.norm(n) < 1e-9:      # 법선이 거의 수직이면 판단 불가 — 원본 유지
            n = normal_from_corners(np.asarray(w["corners_3d"], float))
    return n / np.linalg.norm(n)


def _gates(w: Dict, d_app: float, d_exit: float) -> tuple:
    c = np.asarray(w["center"], float)
    n = _normal(w)
    ap, ex = c + d_app * n, c - d_exit * n
    # 안전망: 법선을 눕혀도 중심 z 가 틀리면 게이트가 위험한 높이로 갈 수 있다.
    for g in (ap, ex):
        g[2] = float(np.clip(g[2], GATE_Z_MIN, GATE_Z_MAX))
    return ap, ex, n


def _inside_opening(pt: np.ndarray, w: Dict, n: np.ndarray, margin: float) -> bool:
    """점이 창문 개구부(여유 뺀) 안쪽을 지나는지 — crossing_warnings 와 같은 판정."""
    UP = np.array([0.0, 0.0, 1.0])
    c = np.asarray(w["center"], float)
    wa = np.cross(UP, n)
    if np.linalg.norm(wa) < 1e-9:
        return True
    wa /= np.linalg.norm(wa)
    d = pt - c
    hw = w["size_wh"][0] / 2.0 - margin
    hh = w["size_wh"][1] / 2.0 - margin
    return abs(d @ wa) <= hw and abs(d @ UP) <= hh


def _build(windows: Sequence[Dict], start, d_app: float, d_exit: float,
           margin: float, align: bool) -> tuple:
    """게이트점 시퀀스 생성. 반환 (points, labels, backtrack_m)."""
    pts = [np.asarray(start, float)]
    labels = ["start"]
    backtrack = 0.0
    prev_exit = None
    for k, w in enumerate(windows):
        ap, ex, n = _gates(w, d_app, d_exit)

        # 통과축 정렬점: 이전 이탈점에서 이 창문 접근점으로 곧장 가면 개구부 밖을
        # 뚫는 경우, 법선 위에 점을 하나 더 둬서 정면으로 들어가게 한다.
        if align and prev_exit is not None:
            seg = ap - prev_exit
            L = np.linalg.norm(seg)
            if L > 1e-9:
                c = np.asarray(w["center"], float)
                da, db = (prev_exit - c) @ n, (ap - c) @ n
                if da * db < 0:            # 이미 평면을 가로지름
                    t = da / (da - db)
                    hit = prev_exit + t * (ap - prev_exit)
                    if not _inside_opening(hit, w, n, margin):
                        pts.append(ap + ALIGN_BACK_M * n)
                        labels.append(f"align{k}")

        # 후진량 측정: 이전 이탈점보다 접근점이 뒤(법선 반대편)에 있으면 후진이다
        if prev_exit is not None:
            fwd = -n                       # 진행 방향 (법선 반대)
            back = (prev_exit - ap) @ fwd
            if back > 0:
                backtrack = max(backtrack, float(back))

        pts.append(ap);  labels.append(f"approach{k}")
        pts.append(ex);  labels.append(f"exit{k}")
        prev_exit = ex

    # 마지막 창문 통과 후 정지점
    n_last = _normal(windows[-1])
    pts.append(prev_exit - STOP_AHEAD_M * n_last)
    labels.append("stop")
    return pts, labels, backtrack


def plan(windows: Sequence[Dict], start=(0.0, 0.0, 1.0), cfg: Dict | None = None,
         align: bool = True, verbose: bool = False) -> Plan:
    """창문 리스트 → Plan. windows 는 center/size_wh/normal|corners_3d 를 가져야 한다.

    order_index 순으로 정렬해서 쓴다. cfg 는 길남 planner_limits.yaml 형식.
    """
    from window_waypoint_planner import crossing_warnings
    if cfg is None:
        cfg = {"d_app": 1.5, "d_exit": 1.0, "clearance_margin": 0.35}
    d_app0, d_exit0 = float(cfg["d_app"]), float(cfg["d_exit"])
    margin = float(cfg["clearance_margin"])
    ws = sorted(windows, key=lambda w: w.get("order_index", 0))
    if not ws:
        raise ValueError("창문이 없음 — 계획할 대상 없음")

    best = None
    for i in range(MAX_PASSES):
        s = SHRINK[min(i, len(SHRINK) - 1)]
        pts, labels, back = _build(ws, start, d_app0 * s, d_exit0 * s, margin, align)
        warns = crossing_warnings([p.tolist() for p in pts], ws, margin)
        p = Plan([[float(v) for v in q] for q in pts], labels, list(warns),
                 passes=i + 1, shrink=s, backtrack_m=back)
        if verbose:
            print(f"  pass {i+1}: standoff x{s:.2f}  경고 {len(warns)}  "
                  f"후진 {back*1000:.0f} mm")
        if best is None or (len(p.warnings), p.backtrack_m) < (len(best.warnings),
                                                               best.backtrack_m):
            best = p
        if p.ok:
            break
    return best


def describe(p: Plan) -> str:
    head = (f"웨이포인트 {len(p.waypoints)}개  (재계획 {p.passes}회, "
            f"standoff x{p.shrink:.2f})")
    if p.ok:
        return head + "  — 위반 없음"
    bits = []
    if p.warnings:
        bits.append(f"경고 {len(p.warnings)}")
    if p.backtrack_m > 1e-6:
        bits.append(f"후진 {p.backtrack_m*1000:.0f} mm")
    return head + "  — " + ", ".join(bits)
