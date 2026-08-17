"""그림 공통 — 팔레트와 반복되던 플롯 조각.

claude.md 규칙 3: **그림에 한글 쓰지 말 것.** matplotlib 기본 폰트에 한글 글리프가
없어서 두부(□□□)로 나온다. 여기 헬퍼들의 라벨은 전부 영문으로 고정한다.
"""
from __future__ import annotations

import os
from typing import Dict, Sequence

import numpy as np

COL = {"red": "#e33", "green": "#1b1", "blue": "#37d"}
GT_STYLE = dict(lw=3.2, ls="-")
EST_STYLE = dict(lw=2.0, ls="--")
CLEARANCE_MM = 350.0    # planner_limits.yaml 의 clearance_margin


def use_agg():
    import matplotlib
    matplotlib.use("Agg")


def save(fig, out_dir: str, name: str, dpi: int = 115) -> str:
    import matplotlib.pyplot as plt
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name)
    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    print(f"wrote {name}")
    return path


def draw_window_3d(ax, corners: np.ndarray, color: str, est: bool = False,
                   label: str | None = None) -> None:
    """월드 코너 (4,3) → 닫힌 사각형."""
    c = np.asarray(corners, float)
    style = EST_STYLE if est else GT_STYLE
    ax.plot(np.r_[c[:, 0], c[0, 0]], np.r_[c[:, 1], c[0, 1]], np.r_[c[:, 2], c[0, 2]],
            color=COL.get(color, "#888"), label=label, **style)


def draw_window_top(ax, corners: np.ndarray, color: str, est: bool = False) -> None:
    """탑뷰 — 창문을 x 위치의 세로 선분으로."""
    c = np.asarray(corners, float)
    style = dict(lw=2.5, ls="--") if est else dict(lw=5, ls="-")
    ax.plot([c[:, 0].mean()] * 2, [c[:, 1].min(), c[:, 1].max()],
            color=COL.get(color, "#888"), **style)


def error_bars(ax, rows: Sequence[Dict]) -> None:
    """metrics.score 행 → center 오차 막대 + clearance 기준선."""
    got = [r for r in rows if r["ok"]]
    names = [r["color"] for r in got]
    vals = [r["center_mm"] for r in got]
    ax.bar(names, vals, color=[COL.get(n, "#888") for n in names])
    for i, v in enumerate(vals):
        ax.text(i, v + max(vals) * 0.02, f"{v:.0f} mm", ha="center", fontsize=10)
    ax.axhline(CLEARANCE_MM, ls="--", color="#666", lw=1.3)
    ax.text(len(names) - 0.5, CLEARANCE_MM * 1.03, "planner clearance 350 mm",
            ha="right", fontsize=9, color="#666")
    ax.set_ylabel("centre error [mm]")
    ax.grid(alpha=.3, axis="y")


def legend_once(ax, seen: set, key: str, label: str) -> str | None:
    """같은 라벨이 범례에 여러 번 안 들어가게."""
    if key in seen:
        return None
    seen.add(key)
    return label
