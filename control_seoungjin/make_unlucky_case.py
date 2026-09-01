# -*- coding: utf-8 -*-
"""운수 된통 나쁜 단 하나의 상황 — x, y, z, roll, pitch, yaw 시계열 (사용자 08-18).

케이스: perf_battery `worst_combo_0kg` = 짐 투하 직후 무하중 0 kg × 정상풍 5 m/s(7 m) × 창문 코스 지그재그 5구간(허용 한계 v 1.2/a 1.0)
       × 비행 중 돌풍 토크 펄스 0.3 N·m×0.3 s @14 s. 배포 게인 스케줄(0 kg 앵커 + 비선형 자세 게인), 스무더+ZVD, 구운 모델, 이상 센서.
그림: 왼쪽 x/y/z (기준 vs 실측 [m] + 오차 [cm] 보조축), 오른쪽 roll/pitch/yaw [deg] (yaw 는 기준 0° 대비 오차). 위쪽 경로 톱뷰.
출력: figure/09_headline/fig_unlucky_case_<case>.png
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from perf_metrics import C, AXIS_C, _style, _rms   # noqa: E402
from perf_battery_plots import _read_perf           # noqa: E402

OUT = os.path.join(HERE, "figure", "09_headline")
EVENTS = {"worst_combo_0kg": [(1.0, 6.0, "상승 1→7 m"), (6.0, 11.0, "구간1 3 m"), (11.5, 15.0, "구간2 2 m+0.5↑"), (15.5, 20.5, "구간3 3 m"), (21.0, 24.5, "구간4 2 m")]}
PULSE = {"worst_combo_0kg": (14.0, 14.3)}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="worst_combo_0kg")
    a = ap.parse_args(argv)
    d = _read_perf(a.case)
    if d is None:
        raise SystemExit(f"CSV 없음: perf_{a.case}.csv")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style()
    os.makedirs(OUT, exist_ok=True)
    t = d["t"]
    err = {k: (d[k] - d[f"{k}_ref"]) * 100 for k in ("x", "y", "z")}
    e3 = np.sqrt(sum(v ** 2 for v in err.values()))
    yaw_err = (d["yaw_deg"] - np.degrees(d["yaw_ref_rad"]) + 180) % 360 - 180
    fly = t >= 1.0
    tail = t >= t[-1] - 4.0
    stats = dict(rms=_rms(e3[fly]), emax=float(e3[fly].max()), end=float(e3[-1]),
                 roll=float(np.abs(d["roll_deg"]).max()), pitch=float(np.abs(d["pitch_deg"]).max()), yaw=float(np.abs(yaw_err).max()),
                 tail_att=_rms(np.concatenate([d["roll_deg"][tail] - d["roll_deg"][tail].mean(), d["pitch_deg"][tail] - d["pitch_deg"][tail].mean()])),
                 tail_pos=float(e3[tail].max()), z=float(np.abs(err["z"][fly]).max()))
    pulse = PULSE.get(a.case)
    if pulse:
        w = (t >= pulse[0]) & (t <= pulse[0] + 4.0)
        stats["pulse_att"] = float(np.hypot(d["roll_deg"][w] - d["roll_deg"][w][0], d["pitch_deg"][w] - d["pitch_deg"][w][0]).max())
        stats["pulse_pos"] = float(e3[w].max())

    fig = plt.figure(figsize=(17, 10))
    gs = fig.add_gridspec(4, 2, height_ratios=[1.15, 1, 1, 1], hspace=0.55, wspace=0.18)
    axp = fig.add_subplot(gs[0, 0]); axinfo = fig.add_subplot(gs[0, 1]); axinfo.axis("off")
    axes_l = [fig.add_subplot(gs[i, 0]) for i in (1, 2, 3)]
    axes_r = [fig.add_subplot(gs[i, 1]) for i in (1, 2, 3)]
    # 경로
    axp.plot(d["x_ref"], d["y_ref"], color=C["muted"], ls=":", lw=1.6, label="기준 (스무더+ZVD)")
    axp.plot(d["x"], d["y"], color=C["blue"], lw=1.6, label="실측")
    axp.plot(d["x"][0], d["y"][0], "o", color=C["ink"], ms=6)
    if pulse:
        i0 = int(np.argmax(t >= pulse[0])); axp.plot(d["x"][i0], d["y"][i0], "X", color=C["red"], ms=11, label="돌풍 펄스 시점")
    axp.set_aspect("equal"); axp.set_xlabel("x [m]"); axp.set_ylabel("y [m]"); axp.legend(fontsize=8, loc="best")
    axp.set_title("경로 톱뷰 (z: 1 → 7 → 7.5 → 7 m)", loc="left", fontsize=9.5)
    # 정보 텍스트
    lines = ["이 한 상황에서 지키는 것",
             f"• 위치 오차 RMS {stats['rms']:.1f} cm · 최대 {stats['emax']:.1f} cm (바람 5 m/s 중, 0 kg 부록 W1 ≤25)",
             f"• 종점 오차 {stats['end']:.1f} cm · 마지막 4 s 위치 잔류 ≤{stats['tail_pos']:.1f} cm · z 이탈 ≤{stats['z']:.1f} cm",
             f"• 기울기 최대 roll {stats['roll']:.1f}° / pitch {stats['pitch']:.1f}° · yaw 오차 최대 {stats['yaw']:.1f}°",
             f"• 도착 후 자세 잔류 RMS {stats['tail_att']:.2f}°"]
    if pulse:
        lines.append(f"• 돌풍 펄스(0.3 N·m×0.3 s, 이동 중) 후 4 s: 자세 이탈 최대 {stats['pulse_att']:.1f}°, 위치 이탈 최대 {stats['pulse_pos']:.1f} cm — 발산/전복 없음")
    axinfo.text(0.0, 1.0, "\n".join(lines), va="top", ha="left", fontsize=10.5, linespacing=1.6,
                bbox=dict(boxstyle="round,pad=0.6", fc="#FFF7E6", ec=C["orange"], lw=1.3))
    # x/y/z
    for ax, k, col in zip(axes_l, ("x", "y", "z"), (AXIS_C["x"], AXIS_C["y"], AXIS_C["z"])):
        ax.plot(t, d[f"{k}_ref"], color=C["muted"], ls=":", lw=1.3, label=f"{k} 기준")
        ax.plot(t, d[k], color=col, lw=1.5, label=f"{k} 실측")
        ax.set_ylabel(f"{k} [m]")
        ax2 = ax.twinx()
        ax2.plot(t, err[k], color=C["red"], lw=0.9, alpha=0.85, label="오차 [cm]")
        lim = max(5.0, np.abs(err[k][fly]).max() * 1.3); ax2.set_ylim(-lim, lim); ax2.set_ylabel("오차 [cm]", color=C["red"]); ax2.tick_params(axis="y", colors=C["red"]); ax2.grid(False)
        ax.set_title(f"{k} — 실측 vs 기준, 빨강 = 오차 (max |e| {np.abs(err[k][fly]).max():.1f} cm)", loc="left", fontsize=9.5)
        for (a0, a1, lab) in EVENTS.get(a.case, []):
            ax.axvspan(a0, a1, color=C["yellow"], alpha=0.12, lw=0)
        if pulse:
            ax.axvspan(*pulse, color=C["red"], alpha=0.3, lw=0)
        ax.legend(loc="upper left", fontsize=7.5, ncol=2)
    # roll/pitch/yaw
    for ax, k, col, lab in zip(axes_r, ("roll_deg", "pitch_deg", "yaw"), (C["magenta"], C["violet"], C["yellow"]), ("roll", "pitch", "yaw 오차 (yaw − yaw_ref)")):
        y = yaw_err if k == "yaw" else d[k]
        ax.plot(t, y, color=col, lw=1.4)
        ax.axhline(0, color=C["ink2"], lw=0.6)
        ax.set_ylabel("[deg]")
        ax.set_title(f"{lab} — max {np.abs(y[fly]).max():.1f}°", loc="left", fontsize=9.5)
        for (a0, a1, lab2) in EVENTS.get(a.case, []):
            ax.axvspan(a0, a1, color=C["yellow"], alpha=0.12, lw=0)
        if pulse:
            ax.axvspan(*pulse, color=C["red"], alpha=0.3, lw=0)
        lim = max(1.0, np.abs(y[fly]).max() * 1.25); ax.set_ylim(-lim, lim)
    for ev in EVENTS.get(a.case, []):
        axes_l[0].text((ev[0] + ev[1]) / 2, axes_l[0].get_ylim()[1], ev[2], ha="center", va="top", fontsize=7, color=C["ink2"])
    axes_l[-1].set_xlabel("시간 [s]  (노랑 = 이동 구간, 빨강 = 돌풍 토크 펄스)"); axes_r[-1].set_xlabel("시간 [s]")
    fig.suptitle("운수 된통 나쁜 하루 한 장면 — 짐 투하 직후 무하중 0 kg, 정상풍 5 m/s, 창문 코스 지그재그(허용 한계 v 1.2/a 1.0), 이동 중 돌풍 토크 펄스 0.3 N·m\n"
                 "배포 게인 스케줄(0 kg 앵커 sA 0.35 + 비선형 자세 게인 gmax 2.1) · 스무더+ZVD · 구운 Simscape 모델 무수정 · 이상 센서 · perf_battery worst_combo_0kg",
                 x=0.01, ha="left", fontsize=11, weight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = os.path.join(OUT, f"fig_unlucky_case_{a.case}.png")
    fig.savefig(out, dpi=160)
    plt.close(fig)
    print("->", out)


if __name__ == "__main__":
    main()
