"""성능 배터리(diagnose/perf_battery.m) 시계열 CSV → 지표 + 그림. perf_metrics.py가 import.

각 함수는 (out_dir, tag) 를 받아 그림을 저장하고 지표 dict를 돌려준다. CSV가 없으면 None.
tag = 케이스 이름 접미(예: '0kg' / '1kg') — perf_battery.m 의 PKG 환경변수와 짝.
"""
from __future__ import annotations

import csv
import os
import sys

import numpy as np

from perf_metrics import C, AXIS_C, SPEC, RESULTS_DIR, _rms


PREFIX = "perf_"      # 'perf_' = 배포 구성(ZVD 셰이퍼 포함) / 'perf_raw_' = 스무더만 (튜닝 하네스 동일)
FIG_SUFFIX = ""       # 그림 파일명 접미 (raw 는 '_raw')


def _read_perf(name):
    path = os.path.join(RESULTS_DIR, f"{PREFIX}{name}.csv")
    if not os.path.exists(path):
        return None
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()}


def _win(d, t0, t1):
    return (d["t"] >= t0) & (d["t"] < t1)


def _att_dev(d, w):
    r = d["roll_deg"][w] - d["roll_deg"][w].mean()
    p = d["pitch_deg"][w] - d["pitch_deg"][w].mean()
    return r, p


def hover(out, tag):
    d = _read_perf(f"hover_{tag}")
    if d is None:
        return None
    import matplotlib.pyplot as plt
    w = _win(d, 2.0, d["t"][-1])
    r, p = _att_dev(d, w)
    m = {"att_rms_deg": _rms(np.concatenate([r, p])), "att_peak_deg": float(np.max(np.hypot(r, p))),
         "z_min_m": float(d["z"][w].min()), "z_max_m": float(d["z"][w].max()),
         "z_sag_cm": float((1.0 - d["z"][_win(d, 0, 2.0)].min()) * 100),
         "drift_cm": float(np.max(np.hypot(d["x"][w] - d["x"][w].mean(), d["y"][w] - d["y"][w].mean())) * 100),
         "yaw_wander_deg": float(np.max(np.abs(d["yaw_deg"][w] - d["yaw_deg"][w].mean())))}
    fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axs[0].plot(d["t"], d["roll_deg"], color=C["magenta"], label="roll")
    axs[0].plot(d["t"], d["pitch_deg"], color=C["violet"], label="pitch")
    axs[0].axhspan(-SPEC["hover_att_rms_deg"], SPEC["hover_att_rms_deg"], color=C["grid"], alpha=0.6, lw=0)
    axs[0].set_ylabel("자세 [deg]")
    axs[0].set_title(f"12 s 호버 ({tag}) — 자세 지터 RMS {m['att_rms_deg']:.3f}° / 피크 {m['att_peak_deg']:.3f}° "
                     f"(R4 ≤0.25° / R5 ≤0.8°, 회색 띠 = ±0.25°)", loc="left")
    axs[0].legend(loc="upper right")
    axs[1].plot(d["t"], d["z"], color=C["aqua"], label="z 실측")
    axs[1].plot(d["t"], d["z_ref"], color=C["muted"], ls="--", label="z 목표")
    axs[1].axhspan(0.97, 1.03, color=C["grid"], alpha=0.6, lw=0)
    axs[1].set_ylabel("고도 [m]")
    axs[1].set_title(f"고도 유지 {m['z_min_m']:.3f}~{m['z_max_m']:.3f} m (Z2 0.97~1.03 = 회색 띠), 이륙 새그 {m['z_sag_cm']:.1f} cm (Z3 ≤5)", loc="left")
    axs[1].legend(loc="lower right")
    axs[2].plot(d["t"], (d["x"] - d["x_ref"]) * 100, color=AXIS_C["x"], label="x")
    axs[2].plot(d["t"], (d["y"] - d["y_ref"]) * 100, color=AXIS_C["y"], label="y")
    axs[2].set_ylabel("수평 위치 오차 [cm]")
    axs[2].set_xlabel("시간 [s]")
    axs[2].set_title(f"위치 드리프트 최대 {m['drift_cm']:.2f} cm (스펙 ≤5 cm)", loc="left")
    axs[2].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_bat_hover_{tag}{FIG_SUFFIX}.png"), dpi=150)
    plt.close(fig)
    return m


def pulse(out, tag):
    import matplotlib.pyplot as plt
    res = {}
    fig, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for nm, lab, col in ((f"torque_pulse_{tag}", "precision", C["blue"]), (f"torque_pulse_agile_{tag}", "agile", C["orange"])):
        d = _read_perf(nm)
        if d is None:
            continue
        pre = _win(d, 2.0, 4.0)
        r0, p0 = d["roll_deg"][pre].mean(), d["pitch_deg"][pre].mean()
        post = _win(d, 4.0, d["t"][-1])
        dev = np.hypot(d["roll_deg"] - r0, d["pitch_deg"] - p0)
        peak = float(dev[post].max())
        ok = dev < 1.0
        t_rec = float("nan")
        for i in range(int(np.argmax(d["t"] >= 4.3)), len(ok)):
            if ok[i:].all():
                t_rec = float(d["t"][i] - 4.0)
                break
        zdev = float(np.max(np.abs(d["z"][post] - 1.0)) * 100)
        xy = float(np.max(np.hypot(d["x"][post], d["y"][post])) * 100)
        ws = np.column_stack([np.abs(d[f"w{i}"]) for i in range(1, 5)])
        res[lab] = {"peak_dev_deg": peak, "t_recover_s": t_rec, "z_dev_cm": zdev, "xy_excursion_cm": xy,
                    "motor_diff_max_rad_s": float(np.max(ws[post].max(axis=1) - ws[post].min(axis=1))),
                    "motor_max_rad_s": float(ws[post].max())}
        axs[0].plot(d["t"], d["roll_deg"], color=col, label=f"{lab} roll (피크 {peak:.2f}°, 회복 {t_rec:.2f} s)")
        axs[1].plot(d["t"], (d["z"] - 1.0) * 100, color=col, label=f"{lab} (최대 {zdev:.1f} cm)")
        for i in range(1, 5):
            axs[2].plot(d["t"], np.abs(d[f"w{i}"]), color=col, lw=0.8, alpha=0.9 if i == 1 else 0.5,
                        label=f"{lab} 모터1~4" if i == 1 else None)
    axs[0].axvspan(4.0, 4.3, color=C["yellow"], alpha=0.25, lw=0)
    for y in (5, -5):
        axs[0].axhline(y, color=C["red"], lw=0.8, ls="--")
    axs[0].set_ylabel("roll [deg]")
    axs[0].set_title(f"외란 토크 펄스 0.3 N·m × 0.3 s @ 4 s ({tag}, 노란 띠) — 최대 이탈 R6 ≤5° (빨간 선), 회복 R7 ≤1.5 s", loc="left")
    axs[0].legend(loc="upper right")
    for y in (10, -10):
        axs[1].axhline(y, color=C["red"], lw=0.8, ls="--")
    axs[1].set_ylabel("고도 이탈 [cm]")
    axs[1].set_title("고도 이탈 (Z4 ≤10 cm)", loc="left")
    axs[1].legend(loc="upper right")
    axs[2].set_ylabel("|ω| [rad/s]")
    axs[2].set_xlabel("시간 [s]")
    axs[2].set_title("모터 각속도 — 포화 없음 확인 (R10)", loc="left")
    axs[2].legend(loc="upper right")
    axs[0].set_xlim(2, 10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_bat_torque_pulse_{tag}{FIG_SUFFIX}.png"), dpi=150)
    plt.close(fig)
    return res or None


def alt_step(out, tag):
    d = _read_perf(f"alt_step_1m_{tag}")
    if d is None:
        return None
    import matplotlib.pyplot as plt
    z, zr, t = d["z"], d["z_ref"], d["t"]
    y = z - 1.0
    i10 = int(np.argmax(y >= 0.1))
    i90 = int(np.argmax(y >= 0.9))
    m = {"rise_s": float(t[i90] - t[i10]), "overshoot_cm": float(max(0.0, z.max() - 2.0) * 100),
         "sse_mm": float((z[-1] - zr[-1]) * 1000),
         "att_peak_deg": float(np.max(np.hypot(d["roll_deg"], d["pitch_deg"])[_win(d, 2.5, t[-1])])),
         "xy_excursion_cm": float(np.max(np.hypot(d["x"], d["y"])) * 100)}
    fig, axs = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axs[0].plot(t, zr, color=C["muted"], ls="--", label="목표 (스무더 통과)")
    axs[0].plot(t, z, color=C["aqua"], label=f"실측 (rise {m['rise_s']:.2f} s, 오버슈트 {m['overshoot_cm']:.1f} cm)")
    axs[0].axhline(2.05, color=C["red"], lw=0.8, ls="--")
    axs[0].text(t[-1], 2.05, " Z1 스펙 +5 cm", color=C["red"], fontsize=8, va="bottom", ha="right")
    axs[0].set_ylabel("고도 [m]")
    axs[0].set_title(f"고도 스텝 1 → 2 m ({tag}) — Z0 rise 1~1.5 s 임계감쇠형 / Z1 오버슈트 ≤5 cm", loc="left")
    axs[0].legend(loc="lower right")
    axs[1].plot(t, (z - zr) * 100, color=C["aqua"], label="z 오차")
    axs[1].plot(t, np.hypot(d["x"], d["y"]) * 100, color=C["blue"], label="수평 이탈 |xy|")
    axs[1].set_ylabel("[cm]")
    axs[1].set_xlabel("시간 [s]")
    axs[1].set_title(f"고도 스텝 중 수평 이탈 최대 {m['xy_excursion_cm']:.2f} cm, 자세 피크 {m['att_peak_deg']:.2f}° (축간 커플링)", loc="left")
    axs[1].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_bat_alt_step_{tag}{FIG_SUFFIX}.png"), dpi=150)
    plt.close(fig)
    return m


def yaw_step(out, tag):
    d = _read_perf(f"yaw_step_90_{tag}")
    if d is None:
        return None
    import matplotlib.pyplot as plt
    t = d["t"]
    yr = np.degrees(d["yaw_ref_rad"])
    y = d["yaw_deg"]
    e = (y - yr + 180) % 360 - 180
    i10 = int(np.argmax(y >= 9))
    i90 = int(np.argmax(y >= 81))
    m = {"rise_s": float(t[i90] - t[i10]), "overshoot_deg": float(max(0.0, y.max() - 90.0)),
         "lag_at_90pct_s": float(t[i90] - t[int(np.argmax(yr >= 81))]),
         "err_peak_deg": float(np.max(np.abs(e[_win(d, 3.0, t[-1])]))),
         "settle_2deg_s": float("nan"),
         "xy_excursion_cm": float(np.max(np.hypot(d["x"], d["y"])) * 100),
         "z_dev_cm": float(np.max(np.abs(d["z"] - 1.0)[_win(d, 2.5, t[-1])]) * 100)}
    band = np.abs(e) <= 2.0
    for i in range(int(np.argmax(t >= 3.0)), len(t)):
        if band[i:].all():
            m["settle_2deg_s"] = float(t[i] - 3.0)
            break
    fig, axs = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axs[0].plot(t, yr, color=C["muted"], ls="--", label="목표 (1.5 s S-램프)")
    axs[0].plot(t, y, color=C["green"], label=f"실측 (rise {m['rise_s']:.2f} s, 오버슈트 {m['overshoot_deg']:.1f}°, ±2° 정착 {m['settle_2deg_s']:.2f} s)")
    axs[0].set_ylabel("yaw [deg]")
    axs[0].set_title(f"yaw 스텝 90° ({tag}) — 스펙 §2 rise ~1.5 s (0.5~1.5 허용)", loc="left")
    axs[0].legend(loc="lower right")
    axs[1].plot(t, e, color=C["green"], label="yaw 오차 [deg]")
    axs[1].plot(t, np.hypot(d["x"], d["y"]) * 100, color=C["blue"], label="수평 이탈 |xy| [cm]")
    axs[1].plot(t, (d["z"] - 1.0) * 100, color=C["aqua"], label="z 이탈 [cm]")
    axs[1].set_ylabel("[deg] / [cm]")
    axs[1].set_xlabel("시간 [s]")
    axs[1].set_title(f"yaw 기동 중 위치 커플링: 수평 {m['xy_excursion_cm']:.2f} cm / z {m['z_dev_cm']:.2f} cm", loc="left")
    axs[1].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_bat_yaw_step_{tag}{FIG_SUFFIX}.png"), dpi=150)
    plt.close(fig)
    return m


def mass_sweep(out, masses=(0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0)):
    import matplotlib.pyplot as plt
    seq = [C["violet"], C["magenta"], C["blue"], C["green"], C["aqua"], C["yellow"], C["orange"]]
    res = []
    fig, axs = plt.subplots(1, 3, figsize=(13, 4))
    d = None
    for i, mp in enumerate(masses):
        d0 = _read_perf(f"move1m_{mp:.1f}kg")   # MATLAB sprintf %.1f: 0.25->0.2, 0.75->0.8
        if d0 is None:
            continue
        d = d0
        t = d["t"]
        ex = (d["x"] - d["x_ref"]) * 100
        mv = _win(d, 3.0, 7.0)
        tl = _win(d, 8, 14)
        m = {"m_pkg": mp, "track_rms_cm": _rms(ex[mv]), "overshoot_cm": float(max(0.0, d["x"].max() - 1.0) * 100),
             "z_dev_cm": float(np.max(np.abs(d["z"] - 1.0)[_win(d, 2.0, t[-1])]) * 100),
             "att_peak_deg": float(np.max(np.hypot(d["roll_deg"], d["pitch_deg"]))),
             "tail_att_rms_deg": _rms(d["pitch_deg"][tl] - d["pitch_deg"][tl].mean()),
             "hover_drift_cm": float(np.max(np.abs(ex[tl])))}
        res.append(m)
        axs[0].plot(t, d["x"], color=seq[i], label=f"{mp:.1f} kg (추종 {m['track_rms_cm']:.1f} cm, 오버 {m['overshoot_cm']:.1f} cm)")
        axs[1].plot(t, (d["z"] - 1.0) * 100, color=seq[i], label=f"{mp:.1f} kg (최대 {m['z_dev_cm']:.1f} cm)")
        axs[2].plot(t, d["pitch_deg"], color=seq[i], label=f"{mp:.1f} kg (피크 {m['att_peak_deg']:.1f}°)")
    if d is not None:
        axs[0].plot(d["t"], d["x_ref"], color=C["muted"], ls="--", label="목표")
    axs[0].set_title("1 m 이동 — 짐 질량 스윕 (precision, 게인 1차식 스케줄)", loc="left")
    axs[0].set_ylabel("x [m]")
    axs[1].set_title("이동 중 고도 이탈 (Z4 ≤10 cm)", loc="left")
    axs[1].set_ylabel("[cm]")
    for y in (10, -10):
        axs[1].axhline(y, color=C["red"], lw=0.8, ls="--")
    axs[2].set_title("pitch", loc="left")
    axs[2].set_ylabel("[deg]")
    for ax in axs:
        ax.set_xlabel("시간 [s]")
        ax.legend(loc="best", fontsize=7)
        ax.set_xlim(2, 12)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_bat_mass_sweep.png"), dpi=150)
    plt.close(fig)
    # agile 질량 스윕 (perf_battery 케이스 8, 있을 때만): precision vs agile 추종/오버/자세피크 대 질량 — 0~1 kg 선형 agile 스케일 검증
    ag = []
    for mp in masses:
        da = _read_perf(f"move1m_agile_{mp:.1f}kg")
        if da is None:
            continue
        ex = (da["x"] - da["x_ref"]) * 100
        mv = _win(da, 3.0, 7.0); tl = _win(da, 8, 14)
        ag.append({"m_pkg": mp, "track_rms_cm": _rms(ex[mv]), "overshoot_cm": float(max(0.0, da["x"].max() - 1.0) * 100),
                   "z_dev_cm": float(np.max(np.abs(da["z"] - 1.0)[_win(da, 2.0, da["t"][-1])]) * 100),
                   "att_peak_deg": float(np.max(np.hypot(da["roll_deg"], da["pitch_deg"]))),
                   "tail_att_rms_deg": _rms(da["pitch_deg"][tl] - da["pitch_deg"][tl].mean()),
                   "hover_drift_cm": float(np.max(np.abs(ex[tl])))})
    if ag and res:
        fig, axs = plt.subplots(1, 3, figsize=(13, 4))
        for key, ax, title, sp in (("track_rms_cm", axs[0], "1 m 이동 추종 RMS [cm]", 10), ("overshoot_cm", axs[1], "오버슈트 [cm]", 10),
                                   ("att_peak_deg", axs[2], "이동 중 자세 피크 [deg]", None)):
            ax.plot([r["m_pkg"] for r in res], [r[key] for r in res], "o-", color=C["blue"], label="precision")
            ax.plot([r["m_pkg"] for r in ag], [r[key] for r in ag], "s-", color=C["orange"], label="agile (0~1 kg: 5/2 ↔ 24/10.8 선형, 1~2 kg 삼각)")
            ax.set_title(title, loc="left"); ax.set_xlabel("적재 질량 [kg]"); ax.legend(fontsize=7)
            if sp:
                ax.axhline(sp, color=C["red"], lw=0.8, ls="--")
        fig.suptitle("질량 스윕 1 m 이동 — precision vs agile 프로파일 (신 자세 스케줄, ZVD)", x=0.01, ha="left")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "fig_bat_mass_sweep_agile.png"), dpi=150)
        plt.close(fig)
    mass_sweep.last_agile = ag or None   # run() 이 b["mass_agile"] 로 수거 (반환형은 precision 리스트 유지)
    return res or None


def diag(out, tag):
    d = _read_perf(f"diag_move_2m_{tag}")
    if d is None:
        return None
    import matplotlib.pyplot as plt
    t = d["t"]
    err = np.column_stack([d["x"] - d["x_ref"], d["y"] - d["y_ref"], d["z"] - d["z_ref"]]) * 100
    mv = _win(d, 3.0, 8.0)
    m = {"track_rms_3d_cm": _rms(np.linalg.norm(err[mv], axis=1)),
         "track_max_cm": float(np.linalg.norm(err[mv], axis=1).max()),
         "z_dev_cm": float(np.max(np.abs(err[_win(d, 2, t[-1]), 2]))),
         "end_err_cm": float(np.linalg.norm(err[-1])),
         "att_peak_deg": float(np.max(np.hypot(d["roll_deg"], d["pitch_deg"]))),
         "roll_pitch_ratio": float(np.max(np.abs(d["roll_deg"])) / max(1e-9, np.max(np.abs(d["pitch_deg"]))))}
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    axs[0].plot(d["x_ref"], d["y_ref"], color=C["muted"], ls="--", label="계획")
    axs[0].plot(d["x"], d["y"], color=C["blue"], label=f"실측 (RMS {m['track_rms_3d_cm']:.1f} cm, 종점 {m['end_err_cm']:.2f} cm)")
    axs[0].set_aspect("equal")
    axs[0].set_xlabel("x [m]")
    axs[0].set_ylabel("y [m]")
    axs[0].set_title(f"대각 이동 2 m × 2 m ({tag}, x·y 동시 기동)", loc="left")
    axs[0].legend(loc="lower right")
    for i, a in enumerate("xyz"):
        axs[1].plot(t, err[:, i], color=AXIS_C[a], label=f"{a} 오차")
    axs[1].set_xlim(2, 12)
    axs[1].set_xlabel("시간 [s]")
    axs[1].set_ylabel("[cm]")
    axs[1].set_title(f"축별 오차 — z 이탈 {m['z_dev_cm']:.1f} cm, roll/pitch 피크비 {m['roll_pitch_ratio']:.2f} (대칭 = 1)", loc="left")
    axs[1].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_bat_diag_move_{tag}{FIG_SUFFIX}.png"), dpi=150)
    plt.close(fig)
    return m


def wind(out, tag):
    import matplotlib.pyplot as plt
    res = {}
    fig, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for nm, lab, col in ((f"wind0_hover7m_{tag}", "바람 0 m/s", C["blue"]), (f"wind5_hover7m_{tag}", "바람 5 m/s", C["orange"])):
        d = _read_perf(nm)
        if d is None:
            continue
        t = d["t"]
        st = _win(d, 9.0, t[-1])
        r, p = _att_dev(d, st)
        m = {"xy_hold_cm": float(np.max(np.hypot(d["x"][st] - d["x_ref"][st], d["y"][st] - d["y_ref"][st])) * 100),
             "z_hold_cm": float(np.max(np.abs(d["z"][st] - d["z_ref"][st])) * 100),
             "att_mean_deg": float(np.hypot(d["roll_deg"][st].mean(), d["pitch_deg"][st].mean())),
             "att_rms_deg": _rms(np.concatenate([r, p]))}
        res[lab] = m
        axs[0].plot(t, np.hypot(d["x"] - d["x_ref"], d["y"] - d["y_ref"]) * 100, color=col, label=f"{lab} (정착 후 최대 {m['xy_hold_cm']:.1f} cm)")
        axs[1].plot(t, (d["z"] - d["z_ref"]) * 100, color=col, label=f"{lab} (정착 후 최대 {m['z_hold_cm']:.1f} cm)")
        axs[2].plot(t, d["pitch_deg"], color=col, label=f"{lab} pitch (평균 트림 {m['att_mean_deg']:.2f}°)")
    axs[0].set_ylabel("수평 이탈 [cm]")
    axs[0].set_title(f"7 m 호버 ({tag}, 바람 프로파일 최대 고도) — 지속 바람 하 위치 유지", loc="left")
    axs[1].set_ylabel("z 오차 [cm]")
    axs[1].set_title("고도 유지", loc="left")
    axs[2].set_ylabel("[deg]")
    axs[2].set_xlabel("시간 [s]")
    axs[2].set_title("바람 상쇄 자세 트림 (I항이 상수 외란 소거)", loc="left")
    for ax in axs:
        ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_bat_wind_{tag}{FIG_SUFFIX}.png"), dpi=150)
    plt.close(fig)
    return res or None


def run(out, tags=("0kg", "1kg"), prefix="perf_", fig_suffix=""):
    global PREFIX, FIG_SUFFIX
    PREFIX, FIG_SUFFIX = prefix, fig_suffix
    b = {}
    for tag in tags:
        for key, fn in (("hover", hover), ("pulse", pulse), ("alt_step", alt_step),
                        ("yaw_step", yaw_step), ("diag", diag), ("wind", wind)):
            try:
                r = fn(out, tag)
            except Exception as e:  # 한 케이스 실패가 전체를 막지 않게 — 조용히 넘기진 않는다
                print(f"[battery:{key}:{tag}] 실패: {e}", file=sys.stderr)
                r = None
            if r is not None:
                b[f"{key}_{tag}"] = r
    try:
        r = mass_sweep(out)
    except Exception as e:
        print(f"[battery:mass] 실패: {e}", file=sys.stderr)
        r = None
    if r is not None:
        b["mass"] = r
        if getattr(mass_sweep, "last_agile", None):
            b["mass_agile"] = mass_sweep.last_agile
    return b


def write_md(b, out, label="배포 구성: 스무더+ZVD"):
    if not b:
        return
    L = [f"\n## 성능 배터리 — {label} (perf_battery.m 실비행, 이 세션 실측)\n"]
    for tag in ("0kg", "1kg"):
        if not any(k.endswith(tag) for k in b):
            continue
        L.append(f"### 짐 {tag}\n")
        h = b.get(f"hover_{tag}")
        if h:
            L.append(f"- **호버 12 s**: 자세 지터 RMS {h['att_rms_deg']:.3f}° / 피크 {h['att_peak_deg']:.3f}° (R4 ≤0.25 / R5 ≤0.8), "
                     f"고도 {h['z_min_m']:.3f}~{h['z_max_m']:.3f} m (Z2 0.97~1.03), 이륙 새그 {h['z_sag_cm']:.1f} cm (Z3 ≤5), "
                     f"드리프트 {h['drift_cm']:.2f} cm (≤5), yaw 배회 {h['yaw_wander_deg']:.2f}° (≤3)")
        for lab, m in (b.get(f"pulse_{tag}") or {}).items():
            L.append(f"- **외란 펄스 0.3 N·m×0.3 s ({lab})**: 최대 이탈 {m['peak_dev_deg']:.2f}° (R6 ≤5), 회복 {m['t_recover_s']:.2f} s (R7 ≤1.5), "
                     f"고도 이탈 {m['z_dev_cm']:.1f} cm, 수평 이탈 {m['xy_excursion_cm']:.1f} cm, 모터 차동 최대 {m['motor_diff_max_rad_s']:.0f} rad/s")
        m = b.get(f"alt_step_{tag}")
        if m:
            L.append(f"- **고도 스텝 1 m**: rise {m['rise_s']:.2f} s (Z0 1~1.5), 오버슈트 {m['overshoot_cm']:.1f} cm (Z1 ≤5), SSE {m['sse_mm']:.1f} mm, 수평 이탈 {m['xy_excursion_cm']:.2f} cm")
        m = b.get(f"yaw_step_{tag}")
        if m:
            L.append(f"- **yaw 스텝 90°**: rise {m['rise_s']:.2f} s (§2 ~1.5), 오버슈트 {m['overshoot_deg']:.1f}°, 최대 오차 {m['err_peak_deg']:.1f}°, ±2° 정착 {m['settle_2deg_s']:.2f} s, "
                     f"위치 커플링 수평 {m['xy_excursion_cm']:.2f} cm / z {m['z_dev_cm']:.2f} cm")
        m = b.get(f"diag_{tag}")
        if m:
            L.append(f"- **대각 이동 2 m×2 m**: 추종 RMS {m['track_rms_3d_cm']:.2f} cm / 최대 {m['track_max_cm']:.1f} cm, 종점 {m['end_err_cm']:.2f} cm, z 이탈 {m['z_dev_cm']:.1f} cm, roll/pitch 피크비 {m['roll_pitch_ratio']:.2f}")
        for lab, m in (b.get(f"wind_{tag}") or {}).items():
            L.append(f"- **7 m 호버 {lab}**: 수평 유지 {m['xy_hold_cm']:.1f} cm, 고도 유지 {m['z_hold_cm']:.1f} cm, 자세 트림 {m['att_mean_deg']:.2f}° / 지터 {m['att_rms_deg']:.3f}°")
        L.append("")
    if "mass" in b:
        L.append("### 질량 스윕 1 m 이동 (precision, 1차식 게인)\n")
        L.append("| 짐 [kg] | 추종 RMS [cm] | 오버슈트 [cm] | z 이탈 [cm] | 자세 피크 [°] | tail 잔류 [°] | 도착 후 드리프트 [cm] |\n|---|---|---|---|---|---|---|")
        for m in b["mass"]:
            L.append(f"| {m['m_pkg']:.1f} | {m['track_rms_cm']:.2f} | {m['overshoot_cm']:.1f} | {m['z_dev_cm']:.1f} | {m['att_peak_deg']:.1f} | {m['tail_att_rms_deg']:.3f} | {m['hover_drift_cm']:.2f} |")
        L.append("")
    with open(os.path.join(out, "summary_missions.md"), "a", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
