# -*- coding: utf-8 -*-
"""전시 한 장: 무하중 0 kg + 말도 안 되는 명령(v 8 m/s / a 30 m/s² 지그재그) → 파이프라인이 허용 한계로 깎고(adjusted) 컨트롤러가 지킨다.

입력: input/absurd_mission.json (요청), controller/.../sim_result_absurd_0kg.mat (실비행, verify_pipeline --only absurd --tag _0kg),
      output/pipeline_meta.json 은 쓰지 않고 요청 limits 와 allowed_limits(payload) 를 직접 대조.
출력: figure/09_headline/fig_absurd_showcase_0kg.png
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from perf_metrics import C, AXIS_C, _style, _rms, load_mission   # noqa: E402
import traj_pipeline as tp                                       # noqa: E402

SUB = os.path.join(HERE, "controller", "Quadcopter-Drone-Model-Simscape")
OUT = os.path.join(HERE, "figure", "09_headline")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--mission", default=os.path.join(HERE, "input", "absurd_mission.json"))
    ap.add_argument("--mat", default=os.path.join(SUB, "sim_result_absurd_0kg.mat"))
    ap.add_argument("--payload", type=float, default=0.0)
    ap.add_argument("--tag", default="0kg")
    a = ap.parse_args(argv)
    mission = json.load(open(a.mission, encoding="utf-8"))
    wps = np.asarray(mission["waypoints"], float)
    req = mission["limits"]
    allowed = tp.allowed_limits(a.payload)
    d = load_mission(a.mat)
    t, des, act = d["t"], d["des"], d["act"]
    T = d["t_traj"]
    err = (act - des) * 100
    e3 = np.linalg.norm(err, axis=1)
    fly = (t >= 0) & (t <= T)
    rms = _rms(e3[fly]); emax = float(e3[fly].max())
    end_err = float(np.linalg.norm(act[np.argmax(t >= T)] - wps[-1]) * 100)
    tp_, pitch = d["real_pitch"]; tr_, roll = d["real_roll"]; ty_, yaw = d["real_yaw"]
    pitch_d, roll_d = np.degrees(pitch), np.degrees(roll)
    att_peak = float(max(np.abs(pitch_d).max(), np.abs(roll_d).max()))
    tail = tp_ >= T + 1.0
    tail_rms = _rms(np.concatenate([pitch_d[tail] - pitch_d[tail].mean(), roll_d[tail] - roll_d[tail].mean()])) if tail.any() else float("nan")
    sag = float(max(0.0, (des[0, 2] - act[t < 2.0, 2].min()) * 100))
    zdev = float(np.max(np.abs(err[fly, 2])))
    seg = np.linalg.norm(np.diff(wps, axis=0), axis=1)
    naive_T = float(seg.sum() / req["v_max"])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style()
    os.makedirs(OUT, exist_ok=True)
    fig = plt.figure(figsize=(17, 8.2))
    gs = fig.add_gridspec(3, 3, width_ratios=[1.05, 1.7, 1.7], height_ratios=[1, 1, 1], wspace=0.25, hspace=0.5)
    ax0 = fig.add_subplot(gs[0:2, 0]); axl = fig.add_subplot(gs[2, 0])
    ax1 = fig.add_subplot(gs[0, 1:]); ax2 = fig.add_subplot(gs[1, 1:], sharex=ax1); ax3 = fig.add_subplot(gs[2, 1:], sharex=ax1)
    # ① 경로 (top view) + 요청 웨이포인트
    ax0.plot(wps[:, 0], wps[:, 1], color=C["red"], lw=1.0, ls="--", alpha=0.7, label="요청 (웨이포인트 직선, 8 m/s 면 %.1f s)" % naive_T)
    ax0.plot(wps[:, 0], wps[:, 1], "*", color=C["red"], ms=12)
    for i, w in enumerate(wps):
        ax0.annotate(f"{i}", (w[0], w[1]), xytext=(4, 4), textcoords="offset points", fontsize=8, color=C["red"])
    ax0.plot(des[:, 0], des[:, 1], color=C["muted"], lw=1.6, ls=":", label="조정 기준 (v 1.2/a 1.0 클램프, ZVD, fly_through, %.1f s)" % T)
    ax0.plot(act[:, 0], act[:, 1], color=C["blue"], lw=1.6, label="실비행 (0 kg)")
    ax0.set_aspect("equal"); ax0.set_xlabel("x [m]"); ax0.set_ylabel("y [m]")
    ax0.set_title("경로 (위에서 본 x–y; z 는 1→2.5→1→2→1 m 오르내림)", loc="left", fontsize=9.5)
    ax0.legend(loc="lower right", fontsize=7.5)
    # ② 요청 vs 허용 한계 (log 막대)
    keys = ["v_max", "a_max", "j_max", "snap_max"]; labs = ["v [m/s]", "a [m/s²]", "j [m/s³]", "snap [m/s⁴]"]
    xk = np.arange(4)
    axl.bar(xk - 0.2, [req[k] for k in keys], width=0.4, color=C["red"], label="요청 (RL 이 준 limits)")
    axl.bar(xk + 0.2, [allowed[k] for k in keys], width=0.4, color=C["blue"], label=f"허용 = 질량별 실측 상한 ({a.payload:g} kg)")
    axl.set_yscale("log"); axl.set_xticks(xk); axl.set_xticklabels(labs, fontsize=8)
    for i, k in enumerate(keys):
        axl.text(i, max(req[k], allowed[k]) * 1.3, f"×{req[k] / allowed[k]:.0f}", ha="center", fontsize=8, color=C["red"])
    axl.set_title("limits: 요청 → 클램프 (verdict = adjusted, RL 에 회신)", loc="left", fontsize=9.5)
    axl.legend(fontsize=7.5, loc="upper left")
    # ③ 기준/실측 z 와 위치
    for k, col, lab in ((0, AXIS_C["x"], "x"), (1, AXIS_C["y"], "y"), (2, AXIS_C["z"], "z")):
        ax1.plot(t, des[:, k], color=col, ls=":", lw=1.2)
        ax1.plot(t, act[:, k], color=col, lw=1.6, label=f"{lab} 실측 (점선 = 조정 기준)")
    ax1.axvline(T, color=C["muted"], lw=0.8, ls="--")
    ax1.set_ylabel("위치 [m]"); ax1.set_title("위치 — 조정 기준(점선) vs 실비행(실선)", loc="left", fontsize=9.5)
    ax1.legend(loc="upper right", fontsize=8, ncol=3)
    # ④ 오차
    ax2.axhspan(-10, 10, color=C["grid"], alpha=0.7, lw=0, label="스펙 ±10 cm")
    for k, col, lab in ((0, AXIS_C["x"], "x"), (1, AXIS_C["y"], "y"), (2, AXIS_C["z"], "z")):
        ax2.plot(t, err[:, k], color=col, label=f"{lab} (max |e| {np.abs(err[fly, k]).max():.1f})")
    ax2.plot(t, e3, color=C["ink"], lw=0.9, ls=":", label="|e| 3D")
    ax2.axvline(T, color=C["muted"], lw=0.8, ls="--"); ax2.axhline(0, color=C["ink2"], lw=0.6)
    ax2.set_ylabel("오차 실측−기준 [cm]")
    ax2.set_title(f"위치 오차 — 비행 중 RMS {rms:.1f} cm · 최대 {emax:.1f} cm · 종점 {end_err:.1f} cm · z 이탈 {zdev:.1f} cm (새그 {sag:.1f})", loc="left", fontsize=9.5)
    ax2.legend(loc="upper right", fontsize=8, ncol=5)
    lim = max(12.0, emax * 1.2); ax2.set_ylim(-lim, lim)
    # ⑤ 자세
    ax3.plot(tr_, roll_d, color=C["magenta"], label=f"roll (max {np.abs(roll_d).max():.1f}°)")
    ax3.plot(tp_, pitch_d, color=C["violet"], label=f"pitch (max {np.abs(pitch_d).max():.1f}°)")
    ax3.plot(ty_, np.degrees(yaw), color=C["yellow"], label="yaw")
    ax3.axvline(T, color=C["muted"], lw=0.8, ls="--"); ax3.axhline(0, color=C["ink2"], lw=0.6)
    ax3.set_ylabel("자세 [deg]"); ax3.set_xlabel("시간 [s]  (점선 = 궤적 종료 T)")
    ax3.set_title(f"자세 — 최대 기울기 {att_peak:.1f}° · 도착 후 잔류 RMS {tail_rms:.3f}°", loc="left", fontsize=9.5)
    ax3.legend(loc="upper right", fontsize=8, ncol=3)
    verdict = (f"결과:  요청 v {req['v_max']:g}/a {req['a_max']:g} → 허용 v {allowed['v_max']:g}/a {allowed['a_max']:g} 로 클램프(adjusted 회신)  ·  "
               f"0 kg 실비행 추종 RMS {rms:.1f} cm · 최대 {emax:.1f} cm · 종점 {end_err:.1f} cm · 기울기 ≤{att_peak:.0f}° · 잔류 {tail_rms:.2f}°  ·  발산/전복/게이트 위반 없음")
    fig.text(0.5, 0.012, verdict, ha="center", va="bottom", fontsize=10.5, weight="bold", color=C["ink"],
             bbox=dict(boxstyle="round,pad=0.4", fc="#FFF7E6", ec=C["orange"], lw=1.2))
    fig.suptitle(f"무하중 0 kg + 말도 안 되는 명령 (지그재그 6점, v {req['v_max']:g} m/s / a {req['a_max']:g} m/s² / j {req['j_max']:g} / snap {req['snap_max']:g} — 물리 한계의 {req['v_max'] / allowed['v_max']:.0f}~{req['snap_max'] / allowed['snap_max']:.0f}배)\n"
                 "파이프라인: 스키마 → 질량별 허용 한계 클램프 → 시간 부여(fly_through) → 스무더 → ZVD → 게이트 → 구운 Simscape 모델 실비행 (배포 게인 스케줄 0 kg 앵커, 이상 센서)",
                 x=0.01, ha="left", fontsize=11, weight="bold")
    fig.tight_layout(rect=(0, 0.06, 1, 0.92))
    out = os.path.join(OUT, f"fig_absurd_showcase_{a.tag}.png")
    fig.savefig(out, dpi=160)
    plt.close(fig)
    print("->", out)


if __name__ == "__main__":
    main()
