# -*- coding: utf-8 -*-
"""최악 조건 시나리오의 절대 오차 시계열 (사용자 08-18: "측정값 말고 x,y,z·roll/pitch/yaw 절대 오차를 시간에 따라").

각 행 = 최악 조건 한 시나리오 (대표 지표 §0.6 의 '그 조건'), 왼쪽 = 위치 오차 e = 실측 − 기준 [cm] (x/y/z),
오른쪽 = 자세 [deg]: roll/pitch 는 수평(0°) 기준 절대값(자세 명령은 로그에 없어 '오차'가 아니라 기울기), yaw 는 yaw − yaw_ref.
회색 점선 = 같은 시나리오의 1 kg 설계점 (비교용). 입력: diagnose/results/perf_*.csv (배터리 최신).
출력: figure/09_headline/fig_worst_timeseries.png
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from perf_metrics import C, AXIS_C, RESULTS_DIR, _style   # noqa: E402
from perf_battery_plots import _read_perf                # noqa: E402

OUT = os.path.join(HERE, "figure", "09_headline")

# (제목, 최악 케이스 CSV 이름, 비교 1 kg CSV 이름, 지표 설명, xlim)
SCEN = [
    ("추종 최악 — 0 kg 대각 2 m×2 m 1.6 m/s", "diag_move_2m_0kg", "diag_move_2m_1kg", "추종 RMS 13.3 cm (부록 P3 ≤15) / 1 kg 6.1", (2, 12)),
    ("오버슈트 최악 — 0 kg 1 m 이동", "move1m_0.0kg", "move1m_1.0kg", "오버슈트 14.1 cm (부록 P2 ≤15) / 1 kg 4.9", (2, 14)),
    ("잔류 자세 최악 — 0.5 kg 1 m 이동 (도착 후 8 s)", "move1m_0.5kg", "move1m_1.0kg", "tail RMS 0.68° (R11 ≤0.25 ❌) — 짐 잔류 스윙", (2, 14)),
    ("바람 최악 — 0 kg 정상풍 5 m/s 7 m 호버", "wind5_hover7m_0kg", "wind5_hover7m_1kg", "수평 유지 11 cm (부록 W1 ≤25; 선형 게인만 23.3) / 1 kg 1.5", (0, 20)),
    ("외란 최악 — 0 kg 토크 펄스 0.3 N·m×0.3 s @4 s (비선형 자세 게인 배포)", "torque_pulse_0kg", "torque_pulse_1kg", "이탈 10° / 밀림 0.26 m (선형 게인만: 34° / 10 m) — 0 kg 목표 ≤20°·≤1 m", (2, 12)),
    ("새그 최악 — 2 kg 이륙 (1 m 이동 케이스 초반)", "move1m_2.0kg", "move1m_1.0kg", "이륙 새그 7.9 cm (Z3 ≤5 ❌, 모터 스핀업 과도)", (0, 6)),
]


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style()
    os.makedirs(OUT, exist_ok=True)
    rows = [(t, w, r, d, xl) for (t, w, r, d, xl) in SCEN if _read_perf(w) is not None]
    fig, axs = plt.subplots(len(rows), 2, figsize=(16, 3.1 * len(rows)))
    for (title, wn, rn, desc, xl), (axp, axa) in zip(rows, axs):
        d = _read_perf(wn)
        r = _read_perf(rn)
        t = d["t"]
        ex, ey, ez = (d["x"] - d["x_ref"]) * 100, (d["y"] - d["y_ref"]) * 100, (d["z"] - d["z_ref"]) * 100
        eyaw = _wrap_deg(d["yaw_deg"] - np.degrees(d["yaw_ref_rad"]))
        m = (t >= xl[0]) & (t <= xl[1])
        if r is not None:
            for k, col in (("x", AXIS_C["x"]), ("y", AXIS_C["y"]), ("z", AXIS_C["z"])):
                axp.plot(r["t"], (r[k] - r[f"{k}_ref"]) * 100, color=C["muted"], lw=0.8, ls=":", alpha=0.9)
            for k in ("roll_deg", "pitch_deg"):
                axa.plot(r["t"], r[k], color=C["muted"], lw=0.8, ls=":", alpha=0.9)
        axp.plot(t, ex, color=AXIS_C["x"], label=f"x (max |e| {np.max(np.abs(ex[m])):.1f})")
        axp.plot(t, ey, color=AXIS_C["y"], label=f"y (max |e| {np.max(np.abs(ey[m])):.1f})")
        axp.plot(t, ez, color=AXIS_C["z"], label=f"z (max |e| {np.max(np.abs(ez[m])):.1f})")
        axp.axhline(0, color=C["ink2"], lw=0.6)
        axp.set_xlim(*xl)
        axp.set_ylabel("위치 오차 e = 실측 − 기준 [cm]")
        axp.set_title(f"{title}\n{desc}", loc="left", fontsize=9.5)
        axp.legend(loc="upper right", fontsize=8, ncol=3)
        axa.plot(t, d["roll_deg"], color=C["magenta"], label=f"roll (max {np.max(np.abs(d['roll_deg'][m])):.1f}°)")
        axa.plot(t, d["pitch_deg"], color=C["violet"], label=f"pitch (max {np.max(np.abs(d['pitch_deg'][m])):.1f}°)")
        axa.plot(t, eyaw, color=C["yellow"], label=f"yaw − yaw_ref (max {np.max(np.abs(eyaw[m])):.1f}°)")
        axa.axhline(0, color=C["ink2"], lw=0.6)
        axa.set_xlim(*xl)
        axa.set_ylabel("자세 [deg] (roll/pitch 수평 기준, yaw 오차)")
        axa.set_title("자세 — 회색 점선 = 1 kg 설계점 같은 시나리오", loc="left", fontsize=9.5)
        axa.legend(loc="upper right", fontsize=8, ncol=3)
        # 대칭 y 범위 (0 중심)
        for ax, arr in ((axp, np.concatenate([ex[m], ey[m], ez[m]])), (axa, np.concatenate([d["roll_deg"][m], d["pitch_deg"][m], eyaw[m]]))):
            lim = max(1e-3, np.max(np.abs(arr)) * 1.25)
            ax.set_ylim(-lim, lim)
    for ax in axs[-1]:
        ax.set_xlabel("시간 [s]")
    fig.suptitle("최악 조건 시나리오 — 절대 오차 시계열 (측정값이 아니라 기준 대비 오차; 색 = 최악 조건, 회색 점선 = 1 kg 설계점)", x=0.01, ha="left", fontsize=11.5)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    out = os.path.join(OUT, "fig_worst_timeseries.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print("->", out)


if __name__ == "__main__":
    main()
