# -*- coding: utf-8 -*-
"""0 kg 재튜닝 전/후 비교 figure (2026-08-18).

perf_raw_*  : 구 앵커 (sA 0.75, kd:kp 1.5, limit_att 800, kp_pos 8, filtPz 0.01, FF 선형)
perf_*      : 신 스케줄 (sA 0.40, kd:kp 0.6, limit_att 100, kp_pos 5, filtPz 0.005, FF √질량) — 배포 구성
출력: figure/fig_0kg_before_after.png, figure/fig_0kg_mass_sweep_before_after.png, figure/summary_0kg_compare.md
사용: cd control_seoungjin && python perf_0kg_compare.py
"""
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "controller", "Quadcopter-Drone-Model-Simscape", "diagnose", "results")
OUT = os.path.join(HERE, "figure")
plt.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
C_OLD, C_NEW = "#b0413e", "#1f77b4"


def load(name):
    p = os.path.join(RES, name)
    if not os.path.exists(p):
        raise FileNotFoundError(p)
    with open(p, encoding="utf-8") as f:
        r = csv.DictReader(f)
        rows = list(r)
    d = {k: np.array([float(x[k]) for x in rows]) for k in rows[0].keys()}
    return d


def att(d):
    return np.hypot(d["roll_deg"], d["pitch_deg"])


def stats(d, t0=2.0):
    m = d["t"] > t0
    a = att(d)
    xy = np.hypot(d["x"] - d["x_ref"], d["y"] - d["y_ref"])
    return dict(att_rms=float(np.sqrt(np.mean(a[m] ** 2))), att_peak=float(a[m].max()),
                sag_cm=float((d["z_ref"] - d["z"]).max() * 100), xy_max_cm=float(xy.max() * 100),
                track_rms_cm=float(np.sqrt(np.mean(xy ** 2)) * 100))


def main():
    os.makedirs(OUT, exist_ok=True)
    pairs = [("hover_0kg", "12 s 호버"), ("torque_pulse_0kg", "외란 토크 펄스 0.3 N·m"),
             ("diag_move_2m_0kg", "대각 2 m×2 m 이동"), ("wind5_hover7m_0kg", "바람 5 m/s 호버")]
    fig, axes = plt.subplots(4, 2, figsize=(14, 14))
    lines = []
    for i, (case, title) in enumerate(pairs):
        old, new = load(f"perf_raw_{case}.csv"), load(f"perf_{case}.csv")
        so, sn = stats(old), stats(new)
        ax = axes[i, 0]
        ax.plot(old["t"], att(old), color=C_OLD, lw=1.2, label=f"구 앵커 (RMS {so['att_rms']:.2f}°, 피크 {so['att_peak']:.1f}°)")
        ax.plot(new["t"], att(new), color=C_NEW, lw=1.2, label=f"신 스케줄 (RMS {sn['att_rms']:.2f}°, 피크 {sn['att_peak']:.1f}°)")
        ax.set_title(f"{title} — 자세 편차 |roll,pitch| [deg]", loc="left", fontsize=10)
        ax.set_ylabel("deg"); ax.grid(alpha=.3); ax.legend(fontsize=8)
        ax = axes[i, 1]
        for d, c, lab, s in ((old, C_OLD, "구", so), (new, C_NEW, "신", sn)):
            xy = np.hypot(d["x"] - d["x_ref"], d["y"] - d["y_ref"]) * 100
            ax.plot(d["t"], xy, color=c, lw=1.2, label=f"{lab}: 수평오차 최대 {s['xy_max_cm']:.1f} cm")
        ax.set_title(f"{title} — 수평 위치 오차 [cm]", loc="left", fontsize=10)
        ax.set_ylabel("cm"); ax.grid(alpha=.3); ax.legend(fontsize=8)
        if i == 3:
            axes[i, 0].set_xlabel("시간 [s]"); axes[i, 1].set_xlabel("시간 [s]")
        lines.append((title, so, sn))
    fig.suptitle("0 kg(무하중) 재튜닝 전/후 — 구 앵커(sA 0.75·kd/kp 1.5·limit 800·kp_pos 8·FF 선형) vs 신 스케줄(0.40·0.6·100·5·FF √m)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    p1 = os.path.join(OUT, "fig_0kg_before_after.png"); fig.savefig(p1, dpi=130); plt.close(fig)

    # 질량 스윕 전/후 (1 m 이동)
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    rows = []
    for tag, c, lab in (("perf_raw_", C_OLD, "구 앵커"), ("perf_", C_NEW, "신 스케줄")):
        ms, tr, ov, ap = [], [], [], []
        for mp in (0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0):
            fn = f"{tag}move1m_{mp:.1f}kg.csv"
            if not os.path.exists(os.path.join(RES, fn)):
                continue
            d = load(fn)
            xy = np.hypot(d["x"] - d["x_ref"], d["y"] - d["y_ref"]) * 100
            x_end = d["x_ref"][-1]
            ovs = max(0.0, (d["x"].max() - x_end) * 100) if x_end > 0.5 else 0.0
            ms.append(mp); tr.append(math.sqrt(np.mean(xy ** 2))); ov.append(ovs)
            m = d["t"] > 1.0
            ap.append(att(d)[m].max())
            rows.append((lab, mp, tr[-1], ov[-1], ap[-1]))
        axes[0].plot(ms, tr, "o-", color=c, label=lab)
        axes[1].plot(ms, ov, "o-", color=c, label=lab)
        axes[2].plot(ms, ap, "o-", color=c, label=lab)
    for ax, t, sp in zip(axes, ("1 m 이동 추종 RMS [cm]", "오버슈트 [cm]", "이동 중 자세 피크 [deg]"), (10, 10, None)):
        ax.set_title(t, loc="left", fontsize=10); ax.set_xlabel("적재 질량 [kg]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
        if sp:
            ax.axhline(sp, color="gray", ls="--", lw=.8)
    fig.suptitle("질량 스윕 1 m 이동 (precision, ZVD) — 구 앵커 vs 신 스케줄 (0~1 kg 선형 보간, 1 kg 이상 동일)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    p2 = os.path.join(OUT, "fig_0kg_mass_sweep_before_after.png"); fig.savefig(p2, dpi=130); plt.close(fig)

    md = ["# 0 kg 재튜닝 전/후 요약 (perf_0kg_compare.py, 2026-08-18)", "",
          "| 케이스 | 구 자세 RMS/피크 [°] | 신 자세 RMS/피크 [°] | 구 수평오차 최대 [cm] | 신 수평오차 최대 [cm] |", "|---|---|---|---|---|"]
    for title, so, sn in lines:
        md.append(f"| {title} | {so['att_rms']:.2f} / {so['att_peak']:.1f} | {sn['att_rms']:.2f} / {sn['att_peak']:.1f} | {so['xy_max_cm']:.1f} | {sn['xy_max_cm']:.1f} |")
    md += ["", "| 구성 | m_pkg | 추종 RMS [cm] | 오버슈트 [cm] | 자세 피크 [°] |", "|---|---|---|---|---|"]
    for lab, mp, t, o, a in rows:
        md.append(f"| {lab} | {mp:.2f} | {t:.1f} | {o:.1f} | {a:.1f} |")
    md += ["", f"![](fig_0kg_before_after.png)", f"![](fig_0kg_mass_sweep_before_after.png)"]
    with open(os.path.join(OUT, "summary_0kg_compare.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    print("\n".join(md[:20]))
    import perf_metrics
    perf_metrics.organize(OUT)
    print("->", p1, p2)


if __name__ == "__main__":
    main()
