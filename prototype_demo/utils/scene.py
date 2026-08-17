"""씬 생성과 관측 경로 — 데모 스크립트마다 복붙돼 있던 부분.

    from utils import scene
    env, layout = scene.make(seed=5, n_windows=3)
    poses, name = scene.path(layout, mode="xy", n_per_window=32)

여기 박혀 있는 상수 두 개는 리포 어디에도 안 적혀 있던 실측값이다.
  TRAIN_STEP = 0.3   PPO 학습에 쓴 값. rl/train_pybullet.py 기본값 0.6 으로 평가하면
                     성공률이 95% → 5% 로 보인다.
  domain_match=True  학습 렌더러의 배경·색 테이블을 PyBullet 씬에 그대로 입힌다.
                     이거 없으면 검출 conf 가 0.95 → 0.28 로 떨어진다.
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np

TRAIN_STEP = 0.3
PATH_MODES = ("xy", "sweep", "scan")


def make(seed: int = 5, n_windows: int = 3, clutter: int = 18, walls: bool = False,
         pane: bool = False, opening: float = 1.0, step: float = TRAIN_STEP,
         spacing: float | None = None, spacing_jitter: float = 0.15,
         min_gap: bool = True):
    """WindowTraversalAviary 생성 + reset. 반환 (env, layout).

    spacing 을 안 주면 env 기본값(1.2 m)을 쓴다 — 학습된 PPO 정책이 본 씬 그대로.
    플래너 데모는 2.0 이상을 준다 (GATE_STANDOFF 주석 참고).
    """
    from rl.pybullet_window_env import WindowTraversalAviary
    kw = {} if spacing is None else {"spacing": float(spacing),
                                     "spacing_jitter": float(spacing_jitter),
                                     "min_gap": bool(min_gap)}
    env = WindowTraversalAviary(n_windows=n_windows, seed=seed, step=step,
                                opening=opening, pane=pane, domain_match=True,
                                clutter=clutter, walls=walls, **kw)
    env.reset(seed=seed)
    return env, env.window_layout


#: d_exit(1.0) + d_app(1.5). 인접 창문 간격이 이보다 좁으면 이탈점이 다음 접근점을
#: 지나쳐서 드론이 뒤로 날아야 한다. planner.py 가 이 경우를 재계획으로 처리한다.
GATE_STANDOFF_M = 2.5


def path(layout: Sequence[dict], mode: str = "xy", n_per_window: int = 32,
         span_deg: float = 110.0, radius: float = 2.0) -> Tuple[List, str]:
    """관측 경로 생성. 반환 (poses, 사람이 읽을 이름).

    xy     창문마다 z 고정 원호 스윕 — 시차각을 크게 벌면서 크기 오차가 제일 작다
    sweep  창문마다 횡스윕 (원래 기본)
    scan   복도를 전진하며 yaw 로 훑기 — 실제 임무 경로에 가깝지만 시차가 작다
    """
    from sim import pybullet_stream as pbs
    if mode == "xy":
        return (pbs.per_window_xy_sweep(layout, n_per_window=n_per_window,
                                        radius=radius, span_deg=span_deg),
                f"xy 원호 스윕 (z 고정, ±{span_deg/2:.0f}°, r={radius}m)")
    if mode == "scan":
        return (pbs.corridor_yaw_scan(layout, n=n_per_window * len(layout)),
                "전진+yaw 스캔")
    if mode == "sweep":
        return (pbs.per_window_sweep(layout, n_per_window=n_per_window),
                "창문별 횡스윕")
    raise ValueError(f"mode 는 {PATH_MODES} 중 하나 — 받은 값: {mode!r}")


def path_label_en(mode: str = "xy", span_deg: float = 110.0,
                  radius: float = 2.0) -> str:
    """경로 이름의 **영문판** — 그림 제목용.

    ``path()`` 가 돌려주는 한글 이름을 matplotlib 에 그대로 넣으면 두부(□□□)가 된다
    (claude.md 규칙 3). 콘솔은 한글, 그림은 영문으로 갈라 쓴다.
    """
    if mode == "xy":
        return f"xy arc sweep (fixed z, +/-{span_deg/2:.0f} deg, r={radius} m)"
    if mode == "scan":
        return "forward + yaw scan"
    if mode == "sweep":
        return "per-window lateral sweep"
    raise ValueError(f"mode 는 {PATH_MODES} 중 하나 — 받은 값: {mode!r}")


def print_layout(layout: Sequence[dict], title: str = "scene (ground truth)") -> None:
    print(title + ":")
    for w in layout:
        c = w["center"]
        print(f"  #{w['order_index']} {w['color']:6s} "
              f"center=({c[0]:5.2f},{c[1]:5.2f},{c[2]:5.2f})  "
              f"{w['ow']:.2f} x {w['oh']:.2f} m")


def gt_by_index(layout: Sequence[dict]) -> dict:
    return {w["order_index"]: w for w in layout}


def corners_world(w: dict) -> np.ndarray:
    """레이아웃 창문 → 월드 코너 (4,3), TL→TR→BR→BL (§4.3 라벨 순서).

    직접 구현하지 않고 렌더러 것을 그대로 쓴다. 접근 방향이 −x 라 image-right 가
    world −y 인데, 여기서 부호를 한 번 틀리면 코너가 좌우로 뒤집힌 채 오차만
    조용히 커진다 (에러는 안 난다).
    """
    from sim.pybullet_stream import window_corners_gt
    return window_corners_gt(w["center"], float(w["ow"]), float(w["oh"]))
