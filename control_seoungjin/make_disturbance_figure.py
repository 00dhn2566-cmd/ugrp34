"""이동 중 외란이 걸렸을 때의 위치(x,y,z)와 자세(roll,pitch,yaw) 시계열.

2026-08-26. 입력: diagnose/results/worstcase/ts_{A,B}.csv (verify_worstcase.m)
출력: figure/14_disturbance/*.png

    python make_disturbance_figure.py

자극: 3 m 직선 이동 한복판에 roll 축 토크 펄스 0.3 N·m x 0.3 s (능력 카드 R1).
  A = 무지연 기준선,  B = 위치 60 ms / 자세 12 ms 지연, 스펙 안 깎음
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
except Exception:      # noqa: BLE001 - 출력 인코딩 실패가 본 작업을 막지 않게
    pass

OUT = os.path.join("figure", "14_disturbance")
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

C_A, C_B, C_GRAY = "#1f77b4", "#d62728", "#888888"
R2D = 180.0 / math.pi


def load_ts(label):
    p = os.path.join(RES, "ts_%s.csv" % label)
    if not os.path.isfile(p):
        return None
    with open(p, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows or "yaw" not in rows[0]:
        return None            # yaw 없는 옛 파일 — 다시 뽑아야 한다
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def pulse_window(label):
    """summary.csv 에서 펄스 시각을 읽는다. 없으면 (None, None)."""
    p = os.path.join(RES, "summary.csv")
    if not os.path.isfile(p):
        return None, None
    with open(p, newline="", encoding="utf-8") as fh:
        raw = list(csv.reader(fh))
    head = raw[0]
    for parts in raw[1:]:
        if not parts or parts[0] != label:
            continue
        extra = len(parts) - len(head)
        if extra > 0:
            parts = [parts[0], ",".join(parts[1:2 + extra])] + parts[2 + extra:]
        d = dict(zip(head, parts))
        try:
            return float(d["t_pulse"]), 0.3
        except (KeyError, ValueError):
            return None, None
    return None, None


def main():
    cases = []
    for lab, ttl, col in (("A", "무지연 (기준선)", C_A),
                          ("B", "위치 60 ms / 자세 12 ms 지연", C_B)):
        d = load_ts(lab)
        if d is not None:
            cases.append((lab, ttl, col, d))
    if not cases:
        print("시계열이 없거나 yaw 열이 없다: %s" % RES)
        print("  MATLAB 에서 WC_ONLY='A,B' 로 verify_worstcase 를 다시 돌릴 것.")
        return 1

    tp, dur = pulse_window(cases[0][0])

    fig, ax = plt.subplots(6, 1, figsize=(9.5, 12.5), sharex=True)
    rows = [("x", "x [m]", 1.0, "xref"),
            ("y", "y [cm]", 100.0, None),
            ("z", "z [m]", 1.0, None),
            ("roll", "roll [deg]", R2D, None),
            ("pitch", "pitch [deg]", R2D, None),
            ("yaw", "yaw [deg]", R2D, None)]

    for i, (key, ylab, scale, refkey) in enumerate(rows):
        for lab, ttl, col, d in cases:
            ax[i].plot(d["t"], scale * d[key], color=col, lw=1.5,
                       label=ttl if i == 0 else None)
            if refkey and refkey in d:
                ax[i].plot(d["t"], scale * d[refkey], color=col, lw=0.8,
                           ls="--", alpha=0.45,
                           label=("%s 기준" % ttl) if i == 0 else None)
        ax[i].set_ylabel(ylab)
        ax[i].grid(alpha=0.3)
        if tp is not None:
            ax[i].axvspan(tp, tp + dur, color=C_GRAY, alpha=0.18, lw=0)

    ax[0].set_title("이동 중 외란: 3 m 직선 이동 한복판에 roll 축 토크 0.3 N·m x 0.3 s\n"
                    "(회색 띠 = 외란 인가 구간, 파선 = 기준 궤적)", fontsize=11)
    ax[0].legend(loc="lower right", fontsize=8.5)
    ax[1].axhline(2, color=C_GRAY, lw=0.7, ls=":")
    ax[1].axhline(-2, color=C_GRAY, lw=0.7, ls=":")
    ax[2].axhline(1.0, color=C_GRAY, lw=0.7, ls=":")
    ax[5].set_xlabel("시간 [s]")
    fig.tight_layout()

    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, "fig_disturbance_states.png")
    fig.savefig(p)
    plt.close(fig)
    print("  " + p)

    # 숫자 요약 — 그림만 보고 눈대중하지 않게
    print()
    print("외란 이후 최대 이탈 (펄스 시작 이후 구간)")
    print("  %-30s %8s %8s %8s %8s %8s %8s"
          % ("케이스", "y[cm]", "z[cm]", "roll[deg]", "pitch[deg]", "yaw[deg]", "복귀[s]"))
    for lab, ttl, col, d in cases:
        m = d["t"] >= (tp if tp is not None else 0.0)
        z0 = float(np.median(d["z"][d["t"] < (tp or 1e9)]))
        lat = np.abs(d["y"][m])
        rec = None
        if tp is not None:
            post = d["t"] > tp + dur
            ok = np.abs(d["y"]) < 0.02
            idx = np.where(post)[0]
            for j, k in enumerate(idx):
                if ok[idx[j:]].all():
                    rec = d["t"][k] - (tp + dur)
                    break
        print("  %-30s %8.2f %8.2f %8.2f %8.2f %8.3f %8s"
              % (ttl, 100 * lat.max(), 100 * np.abs(d["z"][m] - z0).max(),
                 R2D * np.abs(d["roll"][m]).max(), R2D * np.abs(d["pitch"][m]).max(),
                 R2D * np.abs(d["yaw"][m]).max(),
                 "-" if rec is None else "%.2f" % rec))
    return 0


if __name__ == "__main__":
    sys.exit(main())
