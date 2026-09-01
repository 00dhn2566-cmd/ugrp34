"""외란 종류 비교 — 토크 / 힘 / 복합 (위치 지연 20 ms 고정).

2026-08-26. 사용자 요청: "힘이 걸린 외란 상황도", "토크와 힘 여러 방향으로".

입력: diagnose/results/worstcase/ts_{K,L,M,N}.csv  (verify_worstcase.m)
출력: figure/16_wrench/*.png

    python make_wrench_figure.py

왜 나눠 보나: 토크는 기체를 **돌려서** 간접적으로 밀고, 힘은 **직접** 민다. 힘은
자세 루프가 개입하기 전에 이미 위치를 밀어 놓기 때문에 오차가 나타나는 시점과
복귀 경로가 다르다. 능력 카드에 "외란 강건성"을 한 줄로 적으려면 둘 다 재야 한다.

케이스 (전부 위치 지연 20 ms / 자세 5 ms / s=1.00 / 3 m 이동 한복판):
  K  토크 x 0.3 N*m                  — 순수 토크
  L  힘 y 2 N                        — 순수 힘 (옆바람 돌풍급)
  M  힘 y 5 N                        — 강한 돌풍
  N  토크 x 0.3 + 힘 y 2, z -2 N     — 복합 (비스듬히 맞고 아래로 눌림)
"""
from __future__ import annotations

import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:      # noqa: BLE001
    pass

OUT = os.path.join("figure", "16_wrench")
RES = os.path.join("controller", "Quadcopter-Drone-Model-Simscape",
                   "diagnose", "results", "worstcase")

for cand in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
    try:
        matplotlib.font_manager.findfont(cand, fallback_to_default=False)
        plt.rcParams["font.family"] = cand
        break
    except Exception:
        continue
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["mathtext.fontset"] = "dejavusans"
plt.rcParams["figure.dpi"] = 130
plt.rcParams["savefig.bbox"] = "tight"

R2D = 180.0 / math.pi
C_REF, C_GRAY = "#222222", "#999999"
CASES = [("K", "토크 x 0.3 N·m", "#1f77b4"),
         ("L", "힘 y 2 N", "#2ca02c"),
         ("M", "힘 y 5 N", "#ff7f0e"),
         ("N", "복합 (토크 x + 힘 y,z)", "#d62728")]
Z_REF = 1.0
T_PULSE = 3.0 + (3.0 * math.pi / (2 * 1.6)) / 2.0     # T0 + TM/2, s=1.00
DUR = 0.3


def load_ts(label):
    p = os.path.join(RES, "ts_%s.csv" % label)
    if not os.path.isfile(p):
        return None
    with open(p, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows or "yaw" not in rows[0]:
        return None
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def recover_s(d):
    idx = np.where(d["t"] > T_PULSE + DUR)[0]
    ok = np.abs(d["y"]) < 0.02
    for j, k in enumerate(idx):
        if ok[idx[j:]].all():
            return d["t"][k] - (T_PULSE + DUR)
    return None


def main():
    have = [(l, t, c, d) for l, t, c in CASES if (d := load_ts(l)) is not None]
    missing = [l for l, _, _ in CASES if load_ts(l) is None]
    if len(have) < 2:
        print("시계열이 부족하다 (없음: %s)" % ", ".join(missing))
        print("  MATLAB 에서 WC_ONLY='L,M,N' 로 verify_worstcase 를 돌릴 것.")
        return 1
    if missing:
        print("주의 — 빠진 케이스: %s" % ", ".join(missing))

    fig, ax = plt.subplots(6, 1, figsize=(9.5, 12.5), sharex=True)
    panels = [("x", "x [m]", 1.0), ("y", "y [cm]", 100.0), ("z", "z [m]", 1.0),
              ("roll", "roll [deg]", R2D), ("pitch", "pitch [deg]", R2D),
              ("yaw", "yaw [deg]", R2D)]

    d0 = have[0][3]
    ax[0].plot(d0["t"], d0["xref"], color=C_REF, lw=1.2, ls="--",
               label="기준 (궤적이 명령한 값)", zorder=1)
    ax[1].axhline(0.0, color=C_REF, lw=1.2, ls="--", zorder=1)
    ax[2].axhline(Z_REF, color=C_REF, lw=1.2, ls="--", zorder=1)
    ax[5].axhline(0.0, color=C_REF, lw=1.2, ls="--", zorder=1)

    # 외란 밖의 구간에서는 네 곡선이 거의 포개진다. 같은 굵기로 순서대로 그리면
    # 나중에 그린 색이 앞의 색을 통째로 덮어 "그 선이 없다" 로 보인다 (15 번 그림에서
    # 실제로 겪었다). 자극이 센 쪽을 굵게 깔고 약한 쪽을 가늘게 위에 얹는다.
    order = list(reversed(have))
    lws = [3.0, 2.2, 1.6, 1.0][-len(order):]
    for i, (key, ylab, scale) in enumerate(panels):
        for (lab, ttl, col, d), lw in zip(order, lws):
            ax[i].plot(d["t"], scale * d[key], color=col, lw=lw, zorder=2,
                       label=ttl if i == 0 else None)
        ax[i].set_ylabel(ylab)
        ax[i].grid(alpha=0.3)
        ax[i].axvspan(T_PULSE, T_PULSE + DUR, color=C_GRAY, alpha=0.20, lw=0, zorder=0)

    ax[0].set_title("외란 종류 비교 — 토크 / 힘 / 복합 (위치 지연 20 ms 고정)\n"
                    "3 m 이동 한복판, 0.3 s 인가 (회색 띠)", fontsize=11)
    hs, ls = ax[0].get_legend_handles_labels()
    idx = [0] + list(range(len(hs) - 1, 0, -1))   # 역순으로 그렸으니 범례는 되돌린다
    ax[0].legend([hs[i] for i in idx], [ls[i] for i in idx],
                 loc="lower right", fontsize=8.5)
    ax[1].axhline(2, color=C_GRAY, lw=0.7, ls=":")
    ax[1].axhline(-2, color=C_GRAY, lw=0.7, ls=":")
    ax[1].text(0.012, 0.90, "점선 = 복귀 판정 밴드 ±2 cm", transform=ax[1].transAxes,
               fontsize=8, color=C_GRAY, va="top")
    ax[5].set_xlabel("시간 [s]")
    fig.tight_layout()

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "fig_wrench_states.png")
    fig.savefig(path)
    plt.close(fig)
    print("  " + path)

    # 확대 그림: 펄스 전후 4초만 — 응답 모양의 차이가 여기서 보인다
    fig2, ax2 = plt.subplots(3, 1, figsize=(9.0, 6.6), sharex=True)
    for (lab, ttl, col, d), lw in zip(order, lws):
        m = (d["t"] > T_PULSE - 1.0) & (d["t"] < T_PULSE + 4.0)
        ax2[0].plot(d["t"][m], 100 * d["y"][m], color=col, lw=lw, label=ttl)
        ax2[1].plot(d["t"][m], 100 * (d["z"][m] - Z_REF), color=col, lw=lw)
        ax2[2].plot(d["t"][m], R2D * d["roll"][m], color=col, lw=lw)
    for a, yl in zip(ax2, ("y [cm]", "z 오차 [cm]", "roll [deg]")):
        a.set_ylabel(yl)
        a.grid(alpha=0.3)
        a.axvspan(T_PULSE, T_PULSE + DUR, color=C_GRAY, alpha=0.20, lw=0)
    ax2[0].axhline(2, color=C_GRAY, lw=0.7, ls=":")
    ax2[0].axhline(-2, color=C_GRAY, lw=0.7, ls=":")
    h2, l2 = ax2[0].get_legend_handles_labels()
    ax2[0].legend(h2[::-1], l2[::-1], fontsize=8.5)
    ax2[0].set_title("펄스 전후 확대 — 토크는 돌려서 밀고, 힘은 직접 민다", fontsize=11)
    ax2[2].set_xlabel("시간 [s]")
    fig2.tight_layout()
    p2 = os.path.join(OUT, "fig_wrench_zoom.png")
    fig2.savefig(p2)
    plt.close(fig2)
    print("  " + p2)

    print()
    print("%-24s %9s %9s %9s %9s %9s %8s"
          % ("외란", "y최대[cm]", "z최대[cm]", "roll[deg]", "pitch[deg]", "yaw[deg]", "복귀[s]"))
    for lab, ttl, col, d in have:
        m = d["t"] >= T_PULSE
        rec = recover_s(d)
        print("%-24s %9.2f %9.2f %9.2f %9.2f %9.2f %8s"
              % (ttl,
                 100 * np.abs(d["y"][m]).max(),
                 100 * np.abs(d["z"][m] - Z_REF).max(),
                 R2D * np.abs(d["roll"][m]).max(),
                 R2D * np.abs(d["pitch"][m]).max(),
                 R2D * np.abs(d["yaw"][m]).max(),
                 "-" if rec is None else "%.2f" % rec))
    return 0


if __name__ == "__main__":
    sys.exit(main())
