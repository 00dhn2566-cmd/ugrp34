# -*- coding: utf-8 -*-
"""0 kg 최악 조건 한 장 (사용자 08-18: "무지막지한 상황에서도 이 performance 는 지킨다").

시나리오: 무하중 0 kg (진자 복원 없음·관성 절반의 가장 불리한 기체) + 공격 기동 대각 2 m×2 m 1.8 s (벡터 1.6 m/s, 기울기 15°)
         — perf_battery `diag_move_2m_0kg` (스무더+ZVD, 배포 게인 스케줄, 이상 센서).
패널: ① x–y 경로 (기준 vs 실측) ② 위치 오차 x/y/z [cm] + 스펙 띠 ③ roll/pitch [deg] + 정착 후 잔류.
옵션 --case torque_pulse_0kg (chain h 후: 비선형 게인 외란 케이스) / --case wind5_hover7m_0kg.
출력: figure/09_headline/fig_0kg_showcase_<case>.png
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from perf_metrics import C, AXIS_C, _style, _rms      # noqa: E402
from perf_battery_plots import _read_perf              # noqa: E402

OUT = os.path.join(HERE, "figure", "09_headline")

CASES = {
    "diag_move_2m_0kg": dict(
        title="0 kg 무하중 + 공격 기동 (대각 2 m×2 m, 1.8 s, 벡터 1.6 m/s)",
        why="가장 불리한 기체(진자 복원 모멘트 없음·자세 관성 절반) × 게이트 상한 속도 기동",
        move=(3.0, 4.8), spec_cm=15.0, spec_lab="0 kg 부록 P3 ≤15 cm (1 kg 스펙 10)"),
    "torque_pulse_0kg": dict(
        title="0 kg 무하중 + 외란 토크 펄스 0.3 N·m × 0.3 s (각가속 32 rad/s², 1 kg의 2배)",
        why="복귀 보장 없는 기체 — 이탈 크기를 20°·1 m 안에 가둔다 (비선형 자세 게인)",
        move=(4.0, 4.3), spec_cm=100.0, spec_lab="0 kg 목표 밀림 ≤1 m"),
    "worst_combo_0kg": dict(
        title="있을 법한 최악 — 짐 투하 직후 무하중 0 kg × 정상풍 5 m/s × 창문 코스 지그재그(허용 한계 v 1.2/a 1.0) × 비행 중 돌풍 토크 펄스 0.3 N·m @14 s",
        why="일어날 수 있는 조건을 전부 겹친 경우: 가장 불리한 기체 + 바람 + 실제 임무 궤적 + 돌풍",
        move=(6.0, 25.0), spec_cm=25.0, spec_lab="0 kg 부록 W1 ≤25 cm (바람 중 위치 유지)", pulse=(14.0, 14.3)),
    "wind5_hover7m_0kg": dict(
        title="0 kg 무하중 + 정상풍 5 m/s (7 m 호버)",
        why="드래그 트림을 위치 I항만으로 버티는 기체",
        move=(1.0, 6.0), spec_cm=25.0, spec_lab="0 kg 부록 W1 ≤25 cm (1 kg 5)"),
}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="diag_move_2m_0kg", choices=list(CASES))
    a = ap.parse_args(argv)
    cfg = CASES[a.case]
    d = _read_perf(a.case)
    if d is None:
        raise SystemExit(f"CSV 없음: perf_{a.case}.csv")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style()
    os.makedirs(OUT, exist_ok=True)
    t = d["t"]
    ex, ey, ez = (d["x"] - d["x_ref"]) * 100, (d["y"] - d["y_ref"]) * 100, (d["z"] - d["z_ref"]) * 100
    e3 = np.sqrt(ex ** 2 + ey ** 2 + ez ** 2)
    t0, t1 = cfg["move"]
    mv = (t >= t0) & (t <= t1 + 3.0)
    rms = _rms(e3[mv]); emax = float(e3.max())
    end_err = float(e3[-1])
    # 정착 시간: 기동 시작 후 |e3| < 5 cm 유지 시작
    ok = e3 < 5.0
    t_settle = float("nan")
    for i in range(int(np.argmax(t >= t1)), len(t)):
        if ok[i:].all():
            t_settle = float(t[i] - t1); break
    tail = t >= t[-1] - 6.0
    att = np.hypot(d["roll_deg"], d["pitch_deg"])
    att_peak = float(att.max())
    tail_rms = _rms(np.concatenate([d["roll_deg"][tail] - d["roll_deg"][tail].mean(), d["pitch_deg"][tail] - d["pitch_deg"][tail].mean()]))
    tail_pos = float(np.max(e3[tail]))
    z_dev = float(np.max(np.abs(ez)))

    fig = plt.figure(figsize=(16, 6.2))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.0, 1.6, 1.6], height_ratios=[1, 1], wspace=0.28, hspace=0.42)
    ax0 = fig.add_subplot(gs[:, 0]); ax1 = fig.add_subplot(gs[0, 1:]); ax2 = fig.add_subplot(gs[1, 1:], sharex=ax1)
    # ① 경로
    ax0.plot(d["x_ref"], d["y_ref"], color=C["muted"], ls="--", lw=1.4, label="기준 경로 (ZVD 성형)")
    ax0.plot(d["x"], d["y"], color=C["blue"], lw=1.8, label="실측 경로")
    ax0.plot(d["x"][0], d["y"][0], "o", color=C["ink"], ms=6); ax0.plot(d["x_ref"][-1], d["y_ref"][-1], "*", color=C["red"], ms=12, label="목표점")
    ax0.set_aspect("equal"); ax0.set_xlabel("x [m]"); ax0.set_ylabel("y [m]")
    ax0.set_title(f"경로 — 종점 오차 {end_err:.1f} cm", loc="left")
    ax0.legend(loc="upper left", fontsize=8)
    # ② 위치 오차
    ax1.axvspan(t0, t1, color=C["yellow"], alpha=0.18, lw=0, label="기동/외란 구간")
    if cfg.get("pulse"):
        ax1.axvspan(*cfg["pulse"], color=C["red"], alpha=0.25, lw=0, label="돌풍 토크 펄스")
        ax2.axvspan(*cfg["pulse"], color=C["red"], alpha=0.25, lw=0)
    ax1.axhspan(-cfg["spec_cm"], cfg["spec_cm"], color=C["grid"], alpha=0.7, lw=0, label=cfg["spec_lab"])
    ax1.plot(t, ex, color=AXIS_C["x"], label="x"); ax1.plot(t, ey, color=AXIS_C["y"], label="y"); ax1.plot(t, ez, color=AXIS_C["z"], label="z")
    ax1.plot(t, e3, color=C["ink"], lw=1.0, ls=":", label="|e| 3D")
    ax1.axhline(0, color=C["ink2"], lw=0.6)
    ax1.set_ylabel("위치 오차 실측−기준 [cm]")
    ax1.set_title(f"위치 오차 — 기동 중 RMS {rms:.1f} cm · 최대 {emax:.1f} cm · 기동 후 ±5 cm 정착 {t_settle:.1f} s · 종점 {end_err:.1f} cm · z 이탈 {z_dev:.1f} cm", loc="left", fontsize=9.5)
    ax1.legend(loc="upper right", fontsize=8, ncol=6)
    lim = max(cfg["spec_cm"] * 1.15, emax * 1.15); ax1.set_ylim(-lim, lim)
    # ③ 자세
    ax2.axvspan(t0, t1, color=C["yellow"], alpha=0.18, lw=0)
    ax2.plot(t, d["roll_deg"], color=C["magenta"], label="roll"); ax2.plot(t, d["pitch_deg"], color=C["violet"], label="pitch")
    ax2.axhline(0, color=C["ink2"], lw=0.6)
    ax2.set_ylabel("자세 [deg]"); ax2.set_xlabel("시간 [s]")
    ax2.set_title(f"자세 — 기동 중 최대 기울기 {att_peak:.1f}° → 정착 후 잔류 RMS {tail_rms:.3f}° (마지막 6 s), 위치 잔류 최대 {tail_pos:.1f} cm", loc="left", fontsize=9.5)
    ax2.legend(loc="upper right", fontsize=8, ncol=2)
    al = max(1.0, att_peak * 1.2); ax2.set_ylim(-al, al)
    # 우상단 결론 박스
    verdict = (f"이 조건에서도 지키는 것:  추종 RMS {rms:.1f} cm ≤ {cfg['spec_cm']:g}  ·  종점 {end_err:.1f} cm  ·  정착 {t_settle:.1f} s  ·  "
               f"잔류 자세 {tail_rms:.2f}°  ·  z 이탈(이륙 새그) {z_dev:.1f} cm  ·  발산/전복 없음")
    fig.text(0.5, 0.012, verdict, ha="center", va="bottom", fontsize=10.5, color=C["ink"], weight="bold",
             bbox=dict(boxstyle="round,pad=0.4", fc="#FFF7E6", ec=C["orange"], lw=1.2))
    fig.suptitle(f"{cfg['title']}\n{cfg['why']} — 배포 게인 스케줄(0 kg 앵커), 스무더+ZVD, 구운 Simscape 모델 무수정, 이상 센서", x=0.01, ha="left", fontsize=11, weight="bold")
    fig.tight_layout(rect=(0, 0.06, 1, 0.92))
    out = os.path.join(OUT, f"fig_0kg_showcase_{a.case}.png")
    fig.savefig(out, dpi=160)
    plt.close(fig)
    print("->", out)


if __name__ == "__main__":
    main()
