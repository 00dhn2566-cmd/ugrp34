"""현실 대역 지연 비교 — 0 / 10 / 20 ms + 기준(reference).

2026-08-26. 사용자 지적: "60~80 ms 는 흔치 않다, 많으면 40 ms 수준".
그래서 실제로 마주칠 법한 대역만 놓고 다시 본다.

입력: diagnose/results/worstcase/ts_{A,J,K}.csv  (verify_worstcase.m)
출력: figure/15_realband/*.png

    python make_realband_figure.py

조건: 짐 1 kg / precision / 3 m 직선 이동. 이동 한복판에 roll 축 토크 0.3 N*m x 0.3 s.
      **위치 경로 지연만 변수** (자세는 5 ms 고정) — 한 번에 한 가지만 바꾼다.

기준(reference)에 대해:
  x 는 궤적이 명령한 값(`xref`)을 그대로 겹쳐 그린다.
  y / z 는 이 임무에서 상수다 (y = 0, z = 1.0 m) — 그 선을 그린다.
  자세(roll/pitch/yaw)에는 **임무 수준의 기준이 없다.** 자세 명령은 위치 루프가
  그때그때 만들어 내는 내부 값이라, 여기서는 실측만 그린다 (yaw 만 0 이 목표).
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

OUT = os.path.join("figure", "15_realband")
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
# 지연이 커질수록 짙어지는 순서 (파랑 -> 주황 -> 빨강)
# 라벨 주의: A 는 "무지연" 이 아니다 — 위치 경로만 0 이고 자세 경로는 5 ms 가 걸려
# 있다 (verify_worstcase.m 케이스표). 자세를 5 ms 로 고정한 채 위치 지연만 바꾸는
# 실험이므로 범례도 그렇게 적는다.
CASES = [("A", "위치 지연 0 ms", "#1f77b4"),
         ("J", "위치 지연 10 ms", "#ff7f0e"),
         ("K", "위치 지연 20 ms", "#d62728")]
Z_REF = 1.0        # 이 임무의 고도 기준 (verify_worstcase.m 의 Z0)
# s=1.00 일 때의 펄스 시각 — T0 + TM/2, TM = 3 m / v_ref(1.6) 의 사다리꼴 시간.
# summary.csv 는 마지막 실행분만 남기므로 (WC_ONLY 로 골라 돌리면 그 케이스만),
# A/J/K 가 거기 없을 때 쓰는 폴백이다. verify_worstcase.m 의 정의와 같아야 한다.
T_PULSE_S1 = 3.0 + (3.0 * math.pi / (2 * 1.6)) / 2.0
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


def pulse_t(label):
    p = os.path.join(RES, "summary.csv")
    if not os.path.isfile(p):
        return None
    with open(p, newline="", encoding="utf-8") as fh:
        raw = list(csv.reader(fh))
    head = raw[0]
    for parts in raw[1:]:
        if not parts or parts[0] != label:
            continue
        extra = len(parts) - len(head)
        if extra > 0:
            parts = [parts[0], ",".join(parts[1:2 + extra])] + parts[2 + extra:]
        try:
            return float(dict(zip(head, parts))["t_pulse"])
        except (KeyError, ValueError):
            return None
    return None


def main():
    have = [(l, t, c, d) for l, t, c in CASES if (d := load_ts(l)) is not None]
    missing = [l for l, _, _ in CASES if load_ts(l) is None]
    if len(have) < 2:
        print("시계열이 부족하다 (없음: %s)" % ", ".join(missing))
        print("  MATLAB 에서 WC_ONLY='J,K' 로 verify_worstcase 를 돌릴 것.")
        return 1
    if missing:
        print("주의 — 빠진 케이스: %s" % ", ".join(missing))

    # summary.csv 는 마지막 실행분만 담고 있을 수 있다 (WC_ONLY 로 골라 돌리면
    # 그 케이스만 쓴다). 그래서 있는 케이스 아무거나에서 펄스 시각을 얻는다 —
    # s=1.00 인 케이스들은 t_pulse 가 같다 (T0 + TM/2).
    tp = None
    for lab, _, _, _ in have:
        tp = pulse_t(lab)
        if tp is not None:
            break
    if tp is None:
        tp = T_PULSE_S1
        print("주의 — summary.csv 에 A/J/K 가 없다 (다른 케이스가 덮어씀). "
              "s=1.00 공식값 t=%.3f s 로 대체한다." % tp)

    fig, ax = plt.subplots(6, 1, figsize=(9.5, 12.5), sharex=True)
    panels = [("x", "x [m]", 1.0), ("y", "y [cm]", 100.0), ("z", "z [m]", 1.0),
              ("roll", "roll [deg]", R2D), ("pitch", "pitch [deg]", R2D),
              ("yaw", "yaw [deg]", R2D)]

    # 기준선 먼저 (뒤에 깔리게)
    d0 = have[0][3]
    ax[0].plot(d0["t"], d0["xref"], color=C_REF, lw=1.2, ls="--",
               label="기준 (궤적이 명령한 값)", zorder=1)
    ax[1].axhline(0.0, color=C_REF, lw=1.2, ls="--", zorder=1)
    ax[2].axhline(Z_REF, color=C_REF, lw=1.2, ls="--", zorder=1)
    ax[5].axhline(0.0, color=C_REF, lw=1.2, ls="--", zorder=1)

    # 세 곡선은 거의 포개진다 (roll 은 진폭 2.4 deg 에 0/20 ms 차이가 최대 1.2 deg).
    # 그냥 같은 굵기로 순서대로 그리면 나중에 그린 색이 앞의 색을 통째로 덮어
    # "파란 선이 안 보인다" 가 된다. 지연이 큰 쪽을 굵게 깔고 작은 쪽을 가늘게
    # 위에 얹어, 겹치는 구간에서도 세 색이 다 남게 한다.
    order = list(reversed(have))
    lws = [3.0, 1.9, 1.1][-len(order):]
    for i, (key, ylab, scale) in enumerate(panels):
        for (lab, ttl, col, d), lw in zip(order, lws):
            ax[i].plot(d["t"], scale * d[key], color=col, lw=lw, zorder=2,
                       label=ttl if i == 0 else None)
        ax[i].set_ylabel(ylab)
        ax[i].grid(alpha=0.3)
        if tp is not None:
            ax[i].axvspan(tp, tp + DUR, color=C_GRAY, alpha=0.20, lw=0, zorder=0)

    ax[0].set_title("현실 대역 지연 비교 — 위치 경로 0 / 10 / 20 ms (자세 5 ms 고정)\n"
                    "3 m 이동 한복판에 roll 축 토크 0.3 N·m × 0.3 s (회색 띠)",
                    fontsize=11)
    # 위에서 역순으로 그렸으므로 범례는 다시 지연 오름차순으로 돌려 놓는다
    hs, ls = ax[0].get_legend_handles_labels()
    idx = [0] + list(range(len(hs) - 1, 0, -1))
    ax[0].legend([hs[i] for i in idx], [ls[i] for i in idx],
                 loc="lower right", fontsize=8.5)
    ax[1].axhline(2, color=C_GRAY, lw=0.7, ls=":")
    ax[1].axhline(-2, color=C_GRAY, lw=0.7, ls=":")
    ax[1].text(0.012, 0.90, "점선 = 복귀 판정 밴드 ±2 cm", transform=ax[1].transAxes,
               fontsize=8, color=C_GRAY, va="top")
    ax[3].text(0.012, 0.90, "자세에는 임무 수준 기준이 없다 (위치 루프의 내부 명령)\n"
                            "선이 굵을수록 큰 지연 — 겹치면 얇은 선(작은 지연)이 위",
               transform=ax[3].transAxes, fontsize=8, color=C_GRAY, va="top")
    ax[5].set_xlabel("시간 [s]")
    fig.tight_layout()

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "fig_realband_states.png")
    fig.savefig(path)
    plt.close(fig)
    print("  " + path)

    # 숫자 요약
    print()
    print("%-16s %9s %9s %9s %9s %9s %9s %8s"
          % ("케이스", "추종x[cm]", "외란y[cm]", "z진동[cm]",
             "roll[deg]", "pitch[deg]", "yaw[deg]", "복귀[s]"))
    for lab, ttl, col, d in have:
        m = d["t"] >= (tp if tp is not None else 0.0)
        settled = d["t"] > 6.0
        rec = None
        if tp is not None:
            idx = np.where(d["t"] > tp + DUR)[0]
            ok = np.abs(d["y"]) < 0.02
            for j, k in enumerate(idx):
                if ok[idx[j:]].all():
                    rec = d["t"][k] - (tp + DUR)
                    break
        zr = 100.0 * (d["z"][settled].max() - d["z"][settled].min()) if settled.any() else float("nan")
        print("%-16s %9.2f %9.2f %9.2f %9.2f %9.2f %9.2f %8s"
              % (ttl,
                 100 * np.abs(d["x"] - d["xref"]).max(),
                 100 * np.abs(d["y"][m]).max(),
                 zr,
                 R2D * np.abs(d["roll"][m]).max(),
                 R2D * np.abs(d["pitch"][m]).max(),
                 R2D * np.abs(d["yaw"][m]).max(),
                 "-" if rec is None else "%.2f" % rec))
    return 0


if __name__ == "__main__":
    sys.exit(main())
