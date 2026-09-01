"""질량 1차식 재앵커 그림 — 현행 스케줄 vs 두 실측 앵커 보간.

2026-08-23. 출력: figure/12_mass_lerp/*.png

사용자 지시: "무게에 따라서 중요 튜닝 값들 선형 보간해서 넣어보고 검증해봐"

사용:
    python make_mass_lerp_figure.py
"""
from __future__ import annotations

import json
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = os.path.join("figure", "12_mass_lerp")
RES = os.path.join("controller", "Quadcopter-Drone-Model-Simscape", "diagnose", "results")
PROG = os.path.join(RES, "verify_mass_lerp_progress.txt")

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

C_OLD, C_NEW, C_BAD = "#888888", "#1f77b4", "#d62728"

# 1차식 앵커 (qc_mass_lerp_apply.m / qc::qc_mass_lerp 와 같은 값)
ANCH = {
    "sA":        (0.35, 1.00),
    "kd:kp":     (0.60, 1.50),
    "limit_att": (100., 800.),
    "kp_pos":    (5.0,  8.0),
    "sZ":        (0.56, 1.00),
    "biasChassis": (75.5, 56.5),
    "nl_gmax":   (2.1,  1.0),
}
ROW = re.compile(r"\s*([\d.]+)\s+(sched|lerp)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+"
                 r"([\d.]+)\s+(\S+)")


def read_rows():
    if not os.path.exists(PROG):
        return []
    out = []
    with open(PROG, encoding="utf-8", errors="replace") as f:
        for ln in f:
            m = ROW.match(ln)
            if m:
                rec = float(m.group(7)) if m.group(7) != "NaN" else float("nan")
                out.append(dict(m=float(m.group(1)), cfg=m.group(2),
                                hov=float(m.group(3)), end_cm=float(m.group(4)),
                                trk_cm=float(m.group(5)), devy_cm=float(m.group(6)),
                                rec_s=rec))
    return out


def main():
    rows = read_rows()
    fig, ax = plt.subplots(1, 3, figsize=(12.6, 3.6))

    # ① 게인 법칙 자체
    mm = np.linspace(0, 1, 101)
    ax[0].plot(mm, 0.75 + 0.25 * mm, "--", color=C_OLD, label="현행 sA (07-19 앵커)")
    ax[0].plot(mm, ANCH["sA"][0] + (ANCH["sA"][1] - ANCH["sA"][0]) * mm,
               "-", color=C_NEW, label="재앵커 sA (08-18 실측)")
    ax[0].scatter([0, 1], list(ANCH["sA"]), color=C_NEW, zorder=3, s=36)
    ax[0].scatter([0], [0.75], facecolors="none", edgecolors=C_OLD, zorder=3, s=36)
    ax[0].annotate("08-18 재튜닝\n(0.75 는 5 Hz 한계사이클 ±8°)", xy=(0, 0.35),
                   xytext=(0.16, 0.44), fontsize=7.5, color=C_NEW,
                   arrowprops=dict(arrowstyle="->", color=C_NEW, lw=.9))
    ax[0].set_xlabel("짐 질량 [kg]"); ax[0].set_ylabel("자세 게인 배율 sA")
    ax[0].set_title("1 kg 에서 두 법칙이 만난다 (골든 불변)")
    ax[0].legend(fontsize=8, loc="lower right"); ax[0].grid(alpha=.3)

    if rows:
        for k, cfg, color, mk, lab in ((0, "sched", C_BAD, "x--", "현행"),
                                       (1, "lerp", C_NEW, "o-", "보간")):
            sel = sorted([r for r in rows if r["cfg"] == cfg], key=lambda r: r["m"])
            if not sel:
                continue
            xs = [r["m"] for r in sel]
            ax[1].semilogy(xs, [max(r["hov"], 1e-4) for r in sel], mk,
                           color=color, label=lab)
            ax[2].plot(xs, [r["end_cm"] for r in sel], mk, color=color, label=lab)
        ax[1].axhline(0.25, color=C_OLD, ls=":", lw=1.0)
        ax[1].text(0.02, 0.29, "판정선 0.25°", color=C_OLD, fontsize=8)
        ax[1].set_xlabel("짐 질량 [kg]"); ax[1].set_ylabel("호버 자세 RMS [deg]")
        ax[1].set_title("호버 안정성 — 현행은 저질량에서 무너진다")
        ax[1].legend(fontsize=8); ax[1].grid(alpha=.3, which="both")
        # 한글 폰트에 위첨자/수학 마이너스 글리프가 없어 로그 기본 눈금이 깨진다 -> 평문
        tk = [0.005, 0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0]
        ax[1].set_yticks(tk); ax[1].set_yticklabels(["%g" % v for v in tk])
        ax[1].minorticks_off(); ax[1].set_ylim(0.004, 20)
        ax[2].set_xlabel("짐 질량 [kg]"); ax[2].set_ylabel("종단 오차 [cm]")
        ax[2].set_title("1 m 이동 목표 도달")
        ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
    else:
        for a in ax[1:]:
            a.text(.5, .5, "검증 결과 없음\n(verify_mass_lerp.m 먼저 실행)",
                   ha="center", va="center", fontsize=9, color=C_OLD)
            a.set_xticks([]); a.set_yticks([])

    fig.suptitle("질량 1차식 재앵커 — 0 kg 실측 앵커를 08-18 채택값으로 교체", fontsize=10)
    fig.tight_layout()
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, "fig_mass_lerp.png")
    fig.savefig(p); plt.close(fig)
    print(f"  {p}")

    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"anchors": {k: list(v) for k, v in ANCH.items()},
                   "old_law": {"sA": [0.75, 1.0], "sZ": [0.56, 1.0]},
                   "verify": rows}, f, ensure_ascii=False, indent=2)
    print(f"  {os.path.join(OUT, 'summary.json')}")


if __name__ == "__main__":
    main()
