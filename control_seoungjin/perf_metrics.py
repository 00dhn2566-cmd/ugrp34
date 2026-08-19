"""컨트롤러 성능 지표 + 그래프 생성기 (성능 지표 세션, 2026-08-18).

MATLAB 없이 기존 비행 로그만으로 돌아간다:
  - 모델 폴더의 sim_result_*.mat  (run_traj_baked / verify_pipeline 실비행 로그)
  - diagnose/results/{step,ramp,jitctr}_ts_*.csv  (튜닝 세션의 스텝/램프/지터 시계열)

지표 정의는 PERFORMANCE_SPEC.md(자세/고도/위치 채점표)와 analyze_flight_log.py의
tail 정의(궤적 종료 T 이후 잔류 = 자세제어가 못 없애는 지터 본체)를 그대로 따른다.
새 지표를 발명하지 않는다 — 스펙에 측정 방법이 있는 것만.

사용:
    python perf_metrics.py [--out figure] [--model-dir <Simscape 폴더>]

출력 (--out 아래):
    summary_missions.csv / .md   미션별 지표표 (스펙 대비 합격 여부 포함)
    fig_mission_<name>.png       미션별 4단 그림: 위치 추종 / 추종 오차 / 자세 / 모터
    fig_path_<name>.png          xy 평면 경로 (계획 vs 실측)
    fig_summary_tracking.png     미션 횡단 추종 RMS·자세 RMS·종점 오차 막대
    fig_step_response.png        위치 스텝 응답 precision vs agile (0.1 m / 1 m)
    fig_ramp_lag.png             등속 램프 추종 지연 precision vs agile
    fig_hover_jitter.png         잔류 지터 대조 (jitctr)
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(HERE, "controller", "Quadcopter-Drone-Model-Simscape")
RESULTS_DIR = os.path.join(MODEL_DIR, "diagnose", "results")
OUT_DIR = os.path.join(HERE, "figure")
TAG = ""   # --tag: 로그/그림/표 접미 (예 _0kg -> sim_result_<name>_0kg.mat, fig_mission_<name>_0kg.png)

# 미션 로그: 이름 -> 파일. sim_result_baked == look_at(08-01 마지막 굽기 비행)이라 중복 제외.
MISSIONS = {
    "step": "sim_result_step.mat",
    "jitter_a": "sim_result_jitter_a.mat",
    "jitter_b": "sim_result_jitter_b.mat",
    "fly_through": "sim_result_fly_through.mat",
    "look_at": "sim_result_look_at.mat",
    "scan": "sim_result_scan.mat",
    "stop_batch": "sim_result_stop_batch.mat",
}

# PERFORMANCE_SPEC.md 채점 기준 (측정 방법이 있는 것만)
SPEC = {
    "track_rms_cm": 10.0,      # §3.5 추종 오차 RMS ≤ 10 cm
    "overshoot_cm": 10.0,      # §3.5 오버슈트 ≤ 10 cm
    "z_dev_cm": 10.0,          # Z4 기동 중 고도 이탈 ≤ 10 cm
    "hover_att_rms_deg": 0.25, # R4 (회귀 기준값 0.48)
    "hover_att_peak_deg": 0.8, # R5 (회귀 기준값 1.05)
    "hover_drift_cm": 5.0,     # §3.5 10 s 호버 드리프트 ≤ 5 cm
}

# dataviz 기준 팔레트 (고정 순서, 순환 금지)
C = {
    "blue": "#2a78d6", "orange": "#eb6834", "aqua": "#1baf7a", "yellow": "#eda100",
    "magenta": "#e87ba4", "green": "#008300", "violet": "#4a3aa7", "red": "#e34948",
    "ink": "#0b0b0b", "ink2": "#52514e", "muted": "#898781", "grid": "#e6e5e1",
    "surface": "#fcfcfb",
}
AXIS_C = {"x": C["blue"], "y": C["orange"], "z": C["aqua"]}


# ---------------------------------------------------------------- 로딩
def _ts(m, name):
    s = m[name]
    return np.ravel(s.time).astype(float), np.ravel(s.signals.values).astype(float)


def load_mission(path):
    import scipy.io as sio
    m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    t = np.ravel(m["sim_time"]).astype(float)
    des = np.column_stack([np.ravel(m[k]).astype(float) for k in ("des_x1", "des_y1", "des_z1")])
    act = np.column_stack([np.ravel(m[k]).astype(float) for k in ("act_x1", "act_y1", "act_z1")])
    d = {
        "t": t, "des": des, "act": act,
        "t_traj": float(np.ravel(m["timespot_spl"])[-1]),
        "plan_t": np.ravel(m["timespot_spl"]).astype(float),
        "plan_xyz": np.asarray(m["spline_data"], float),
        "plan_yaw": np.ravel(m["spline_yaw"]).astype(float),
    }
    for k in ("real_pitch", "real_roll", "real_yaw", "real_vz"):
        d[k] = _ts(m, k)
    d["prop_w"] = [_ts(m, f"prop{i}_w") for i in range(1, 5)]
    return d


# ---------------------------------------------------------------- 지표
def _rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x ** 2))) if x.size else float("nan")


def mission_metrics(d):
    t, des, act = d["t"], d["des"], d["act"]
    T = d["t_traj"]
    fly = t <= T
    tail = t > T
    err = act - des
    # 계획 구간 추종 (스펙 §3.5: 완만 궤적 RMS)
    m = {
        "T_traj_s": round(T, 2), "T_sim_s": round(float(t[-1]), 2),
        "track_rms_x_cm": _rms(err[fly, 0]) * 100, "track_rms_y_cm": _rms(err[fly, 1]) * 100,
        "track_rms_z_cm": _rms(err[fly, 2]) * 100,
        "track_rms_3d_cm": _rms(np.linalg.norm(err[fly], axis=1)) * 100,
        "track_max_3d_cm": float(np.max(np.linalg.norm(err[fly], axis=1))) * 100,
        # 종점 오차: 궤적 끝점 대비 로그 마지막 실측
        "endpoint_err_cm": float(np.linalg.norm(act[-1] - des[-1])) * 100,
        # z: 이륙 새그(Z3, t<2 s) 와 기동 중 이탈(Z4, t>=2 s) 분리
        "z_sag_cm": float(np.max(np.abs(err[(t < 2.0), 2]))) * 100 if (t < 2.0).any() else float("nan"),
        "z_dev_max_cm": float(np.max(np.abs(err[fly & (t >= 2.0), 2]))) * 100,
        # 오버슈트: 최종 목표를 지나 진행방향으로 얼마나 더 갔나 (xy 평면, 도착 이후 창)
        "overshoot_cm": _overshoot_cm(t, des, act, T),
    }
    # 자세: 비행 중 RMS/피크, tail(도착 후) 잔류 RMS = analyze_flight_log 정의
    for nm, key in (("pitch", "real_pitch"), ("roll", "real_roll")):
        ta, va = d[key]
        deg = np.degrees(va)
        f = ta <= T
        tl = ta > T
        m[f"{nm}_rms_fly_deg"] = _rms(deg[f])
        m[f"{nm}_peak_deg"] = float(np.max(np.abs(deg))) if deg.size else float("nan")
        m[f"{nm}_tail_rms_deg"] = _rms(deg[tl] - np.mean(deg[tl])) if tl.any() else float("nan")
    m["att_tail_rms_deg"] = float(np.hypot(m["pitch_tail_rms_deg"], m["roll_tail_rms_deg"]) / np.sqrt(2))
    # 도착 후 위치 드리프트 (§3.5 10 s 호버 ≤ 5 cm): tail 창에서 실측 위치 최대 편차
    if tail.any():
        p = act[tail]
        m["hover_drift_cm"] = float(np.max(np.linalg.norm(p - p.mean(axis=0), axis=1))) * 100
    else:
        m["hover_drift_cm"] = float("nan")
    # yaw 추종 (spline_yaw 대비, 비행 중 랩 처리)
    ty, yaw = d["real_yaw"]
    yref = np.interp(ty, d["plan_t"], np.unwrap(d["plan_yaw"]))
    yerr = np.degrees((yaw - yref + np.pi) % (2 * np.pi) - np.pi)
    m["yaw_rms_fly_deg"] = _rms(yerr[ty <= T])
    # 모터: 각속도 범위, 포화 여부(TUNING 정상대역 ~±825 rad/s 기준 참고만)
    ws = np.concatenate([np.abs(v) for _, v in d["prop_w"]])
    m["prop_w_mean_rad_s"] = float(np.mean(ws[ws > 1]))
    m["prop_w_max_rad_s"] = float(np.max(ws))
    # 합격 판정
    m["pass_track"] = m["track_rms_3d_cm"] <= SPEC["track_rms_cm"]
    m["pass_overshoot"] = m["overshoot_cm"] <= SPEC["overshoot_cm"]
    m["pass_zdev"] = m["z_dev_max_cm"] <= SPEC["z_dev_cm"]
    m["pass_drift"] = m["hover_drift_cm"] <= SPEC["hover_drift_cm"]
    return m


def _overshoot_cm(t, des, act, T):
    """도착 목표(궤적 끝점) 기준, 마지막 접근 방향으로의 초과 진행량 최대 [cm].
    접근 방향 = 마지막 1 s 계획 변위 방향(xy). 정지 미션이 아니거나 변위 0이면 3D 종점 초과로 대체."""
    goal = des[-1]
    fly = t <= T
    if fly.sum() < 2:
        return float("nan")
    idx = np.where(fly)[0]
    i0 = idx[np.searchsorted(t[idx], max(t[idx][0], T - 1.0))]
    v = goal[:2] - des[i0, :2]
    n = np.linalg.norm(v)
    after = t >= T - 0.5
    if n < 1e-3:
        return float(np.max(np.linalg.norm(act[after] - goal, axis=1))) * 100
    u = v / n
    proj = (act[after, :2] - goal[:2]) @ u
    return float(max(0.0, np.max(proj))) * 100


# ---------------------------------------------------------------- 그림 공통
def _style():
    import matplotlib as mpl
    mpl.rcParams.update({
        "figure.facecolor": C["surface"], "axes.facecolor": C["surface"],
        "axes.edgecolor": C["muted"], "axes.labelcolor": C["ink2"],
        "xtick.color": C["ink2"], "ytick.color": C["ink2"],
        "axes.grid": True, "grid.color": C["grid"], "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False,
        "lines.linewidth": 1.6, "font.size": 9, "axes.titlesize": 10,
        "legend.frameon": False, "legend.fontsize": 8,
        "font.family": ["Malgun Gothic", "DejaVu Sans"], "axes.unicode_minus": False,
    })


def fig_mission(name, d, m, out):
    import matplotlib.pyplot as plt
    t, des, act, T = d["t"], d["des"], d["act"], d["t_traj"]
    fig, axs = plt.subplots(4, 1, figsize=(9, 10), sharex=True,
                            gridspec_kw={"height_ratios": [3, 2, 2, 2]})
    ax = axs[0]
    for i, a in enumerate("xyz"):
        ax.plot(t, des[:, i], color=AXIS_C[a], ls="--", lw=1.1, label=f"{a} 목표")
        ax.plot(t, act[:, i], color=AXIS_C[a], lw=1.6, label=f"{a} 실측")
    ax.axvline(T, color=C["muted"], lw=0.8, ls=":")
    ax.text(T, ax.get_ylim()[1], " 궤적 종료", color=C["muted"], va="top", fontsize=8)
    ax.set_ylabel("위치 [m]")
    ax.set_title(f"미션 '{name}' — 위치 추종 (추종 RMS {m['track_rms_3d_cm']:.1f} cm, "
                 f"종점 오차 {m['endpoint_err_cm']:.1f} cm)", loc="left", color=C["ink"])
    ax.legend(ncol=3, loc="upper left", bbox_to_anchor=(0, -0.02))

    ax = axs[1]
    err = (act - des) * 100
    for i, a in enumerate("xyz"):
        ax.plot(t, err[:, i], color=AXIS_C[a], label=f"{a} 오차")
    ax.axhspan(-SPEC["track_rms_cm"], SPEC["track_rms_cm"], color=C["grid"], alpha=0.5, lw=0)
    ax.axvline(T, color=C["muted"], lw=0.8, ls=":")
    ax.set_ylabel("추종 오차 [cm]")
    ax.legend(ncol=3, loc="upper right")
    ax.text(0.01, 0.95, "회색 띠 = 스펙 ±10 cm", transform=ax.transAxes, va="top",
            fontsize=8, color=C["ink2"])

    ax = axs[2]
    tp, p = d["real_pitch"]
    tr, r = d["real_roll"]
    ax.plot(tp, np.degrees(p), color=C["violet"], label="pitch")
    ax.plot(tr, np.degrees(r), color=C["magenta"], label="roll")
    ax.axvline(T, color=C["muted"], lw=0.8, ls=":")
    ax.set_ylabel("자세 [deg]")
    ax.legend(ncol=2, loc="upper right")
    ax.text(0.01, 0.95, f"비행 중 RMS pitch {m['pitch_rms_fly_deg']:.2f}° / roll "
            f"{m['roll_rms_fly_deg']:.2f}°, tail 잔류 {m['att_tail_rms_deg']:.3f}°",
            transform=ax.transAxes, va="top", fontsize=8, color=C["ink2"])

    ax = axs[3]
    cols = [C["blue"], C["orange"], C["aqua"], C["yellow"]]
    for i, (tw, w) in enumerate(d["prop_w"]):
        ax.plot(tw, np.abs(w), color=cols[i], lw=1.0, label=f"모터 {i+1}")
    ax.axvline(T, color=C["muted"], lw=0.8, ls=":")
    ax.set_ylabel("|ω| [rad/s]")
    ax.set_xlabel("시간 [s]")
    ax.legend(ncol=4, loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_mission_{name}{TAG}.png"), dpi=150)
    plt.close(fig)


def fig_path(name, d, m, out):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 6))
    P, A = d["plan_xyz"], d["act"]
    ax.plot(P[:, 0], P[:, 1], color=C["muted"], ls="--", lw=1.2, label="계획 경로")
    ax.plot(A[:, 0], A[:, 1], color=C["blue"], lw=1.6, label="실측 경로")
    ax.plot(A[0, 0], A[0, 1], "o", color=C["aqua"], ms=7, label="출발")
    ax.plot(P[-1, 0], P[-1, 1], "s", color=C["orange"], ms=7, label="목표 종점")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"'{name}' xy 경로 — 최대 이탈 {m['track_max_3d_cm']:.1f} cm", loc="left")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_path_{name}{TAG}.png"), dpi=150)
    plt.close(fig)


def fig_summary(rows, out):
    import matplotlib.pyplot as plt
    names = [r["mission"] for r in rows]
    x = np.arange(len(names))
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.8))
    w = 0.26
    ax = axs[0]
    for i, a in enumerate("xyz"):
        ax.bar(x + (i - 1) * w, [r[f"track_rms_{a}_cm"] for r in rows], w,
               color=AXIS_C[a], label=a)
    ax.axhline(SPEC["track_rms_cm"], color=C["red"], lw=1, ls="--")
    ax.text(len(x) - 0.5, SPEC["track_rms_cm"], " 스펙 10 cm", color=C["red"], fontsize=8, va="bottom", ha="right")
    ax.set_title("비행 중 추종 RMS [cm]", loc="left")
    ax.legend(ncol=3, loc="upper left")
    ax = axs[1]
    ax.bar(x - w / 2, [r["pitch_rms_fly_deg"] for r in rows], w, color=C["violet"], label="pitch (비행 중)")
    ax.bar(x + w / 2, [r["roll_rms_fly_deg"] for r in rows], w, color=C["magenta"], label="roll (비행 중)")
    ax.plot(x, [r["att_tail_rms_deg"] for r in rows], "o", color=C["ink"], ms=5, label="tail 잔류 RMS")
    ax.set_title("자세 RMS [deg]", loc="left")
    ax.legend()
    ax = axs[2]
    ax.bar(x - w / 2, [r["endpoint_err_cm"] for r in rows], w, color=C["blue"], label="종점 오차")
    ax.bar(x + w / 2, [r["z_dev_max_cm"] for r in rows], w, color=C["aqua"], label="z 최대 이탈")
    ax.axhline(SPEC["z_dev_cm"], color=C["red"], lw=1, ls="--")
    ax.set_title("종점 오차 / 기동 중 z 이탈 [cm]", loc="left")
    ax.legend(loc="upper left")
    for ax in axs:
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=25, ha="right")
    fig.suptitle("미션 횡단 요약 (구운 모델, precision 프로파일 기본)", x=0.01, ha="left", color=C["ink"])
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_summary_tracking{TAG}.png"), dpi=150)
    plt.close(fig)


def fig_yaw(missions, out):
    """yaw 모드 미션의 yaw 추종 (알려진 약점 정량화): 목표 vs 실측 + 오차."""
    import matplotlib.pyplot as plt
    names = [n for n in ("scan", "look_at", "fly_through", "stop_batch") if n in missions]
    if not names:
        return
    fig, axs = plt.subplots(2, len(names), figsize=(4.2 * len(names), 6), sharex="col")
    axs = np.atleast_2d(axs)
    if axs.shape[0] == 1:
        axs = axs.T
    for j, n in enumerate(names):
        d, m = missions[n]
        ty, yaw = d["real_yaw"]
        yref = np.interp(ty, d["plan_t"], np.unwrap(d["plan_yaw"]))
        yr_deg = np.degrees(np.unwrap(yref)); y_deg = np.degrees(np.unwrap(yaw))
        e = np.degrees((yaw - yref + np.pi) % (2 * np.pi) - np.pi)
        T = d["t_traj"]
        axs[0, j].plot(ty, yr_deg, color=C["muted"], ls="--", label="목표 yaw")
        axs[0, j].plot(ty, y_deg, color=C["green"], label="실측 yaw")
        axs[0, j].axvline(T, color=C["muted"], lw=0.8, ls=":")
        axs[0, j].set_title(f"'{n}' yaw (비행 중 오차 RMS {m['yaw_rms_fly_deg']:.1f}°)", loc="left")
        axs[0, j].legend(loc="best")
        axs[1, j].plot(ty, e, color=C["green"])
        axs[1, j].axvline(T, color=C["muted"], lw=0.8, ls=":")
        axs[1, j].set_xlabel("시간 [s]")
        axs[1, j].set_title(f"yaw 오차 (최대 {np.max(np.abs(e[ty <= T])):.0f}°)", loc="left")
    axs[0, 0].set_ylabel("yaw [deg]")
    axs[1, 0].set_ylabel("[deg]")
    fig.suptitle("yaw 루프 추종 — 알려진 약점(대역폭·오버슈트, 08-01 ★튜닝 세션 이관). 위치 추종은 영향 없음(§1 표)", x=0.01, ha="left", color=C["ink"])
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_yaw_missions{TAG}.png"), dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------- CSV 시계열 (튜닝 세션)
def _read_ts(path):
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        rows = list(rd)
    t = np.array([float(r["t"]) for r in rows])
    ref = np.array([float(r["x_ref"]) for r in rows])
    meas = np.array([float(r["x_meas"]) for r in rows])
    pitch = np.array([float(r["pitch_deg"]) for r in rows])
    return t, ref, meas, pitch


def step_metrics(t, ref, meas):
    """10-90% rise, 오버슈트 %, ±2 % 정착, 정상상태 오차 — 스텝 시작은 ref가 처음 변하는 순간."""
    A = ref[-1] - ref[0]
    if abs(A) < 1e-9:
        return {}
    i0 = int(np.argmax(np.abs(ref - ref[0]) > 1e-9))
    t0 = t[i0]
    y = (meas - ref[0]) / A
    i10 = int(np.argmax(y >= 0.1))
    i90 = int(np.argmax(y >= 0.9))
    rise = t[i90] - t[i10] if i90 > i10 else float("nan")
    over = float(max(0.0, (np.max(y) - 1.0) * 100))
    band = np.abs(y - 1.0) <= 0.02
    settle = float("nan")
    for i in range(len(y)):
        if band[i:].all():
            settle = t[i] - t0
            break
    sse = float((meas[-1] - ref[-1]) * 1000)
    return {"rise_s": rise, "overshoot_pct": over, "settle2pct_s": settle, "sse_mm": sse, "t0": t0}


def fig_step(out):
    import matplotlib.pyplot as plt
    cases = [("1m", "1 m 스텝"), ("0.1m", "0.1 m 스텝 (기준 자체가 스무더 오용 진동 — 참고)")]
    fig, axs = plt.subplots(2, 2, figsize=(11, 7), sharex="col")
    res = []
    for j, (tag, title) in enumerate(cases):
        for prof, col in (("precision", C["blue"]), ("agile", C["orange"])):
            p = os.path.join(RESULTS_DIR, f"step_ts_{prof}_{tag}.csv")
            if not os.path.exists(p):
                continue
            t, ref, meas, pitch = _read_ts(p)
            mm = step_metrics(t, ref, meas)
            res.append({"case": tag, "profile": prof, **mm})
            axs[0, j].plot(t, meas, color=col, label=f"{prof} (rise {mm['rise_s']:.2f}s, "
                           f"오버 {mm['overshoot_pct']:.1f}%)")
            axs[1, j].plot(t, pitch, color=col, lw=1.2, label=prof)
        axs[0, j].plot(t, ref, color=C["muted"], ls="--", lw=1.1, label="목표")
        axs[0, j].set_title(f"위치 스텝 응답 — {title}", loc="left", fontsize=9 if j else 10)
        axs[0, j].set_ylabel("x [m]")
        axs[0, j].legend(loc="lower right")
        axs[1, j].set_ylabel("pitch [deg]")
        axs[1, j].set_xlabel("시간 [s]")
        axs[1, j].legend(loc="upper right")
        axs[0, j].set_xlim(0, min(12, t[-1]))
    fig.suptitle("컨트롤러 프로파일별 스텝 응답 (파이프라인 통과 궤적: 스무더+게이트 후 준-스텝)",
                 x=0.01, ha="left", color=C["ink"])
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_step_response.png"), dpi=150)
    plt.close(fig)
    return res


def fig_ramp(out):
    import matplotlib.pyplot as plt
    speeds = ["0.5mps", "1.5mps", "2mps"]
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.8), sharey=True)
    res = []
    for j, sp in enumerate(speeds):
        ax = axs[j]
        for prof, col in (("precision", C["blue"]), ("agile", C["orange"])):
            p = os.path.join(RESULTS_DIR, f"ramp_ts_{prof}_{sp}.csv")
            if not os.path.exists(p):
                continue
            t, ref, meas, _ = _read_ts(p)
            lag = (ref - meas) * 100
            moving = np.abs(np.gradient(ref, t)) > 0.05
            lag_rms = _rms(lag[moving]) if moving.any() else float("nan")
            lag_pk = float(np.max(np.abs(lag[moving]))) if moving.any() else float("nan")
            settle = lag[t > t[moving].max() + 2.0] if moving.any() else lag[-100:]
            res.append({"speed": sp, "profile": prof, "lag_rms_cm": lag_rms, "lag_peak_cm": lag_pk,
                        "resid_rms_cm": _rms(settle)})
            ax.plot(t, lag, color=col, label=f"{prof} (이동 중 RMS {lag_rms:.1f} / 피크 {lag_pk:.1f} cm)")
        ax.set_title(f"등속 램프 {sp.replace('mps', ' m/s')}", loc="left")
        ax.set_xlabel("시간 [s]")
        ax.axhline(0, color=C["muted"], lw=0.8)
        ax.legend(loc="upper right")
    axs[0].set_ylabel("추종 지연 (목표−실측) [cm]")
    fig.suptitle("등속 추종 지연 — 프로파일 대조", x=0.01, ha="left", color=C["ink"])
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_ramp_lag.png"), dpi=150)
    plt.close(fig)
    return res


def fig_jitter(out):
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.8))
    res = []
    for prof, col in (("precision", C["blue"]), ("agile", C["orange"])):
        p = os.path.join(RESULTS_DIR, f"jitctr_ts_{prof}_0.1m.csv")
        if not os.path.exists(p):
            continue
        t, ref, meas, pitch = _read_ts(p)
        # 도착 후 창(마지막 4 s) 잔류 지터
        w = t >= t[-1] - 4.0
        rms = _rms(pitch[w] - pitch[w].mean())
        pk = float(np.max(np.abs(pitch[w] - pitch[w].mean())))
        res.append({"profile": prof, "tail_pitch_rms_deg": rms, "tail_pitch_peak_deg": pk})
        axs[0].plot(t, pitch, color=col, lw=1.0, label=f"{prof} (tail RMS {rms:.3f}°)")
        axs[1].plot(t, (meas - ref) * 100, color=col, lw=1.0, label=prof)
    axs[0].axhspan(-SPEC["hover_att_rms_deg"], SPEC["hover_att_rms_deg"], color=C["grid"], alpha=0.6, lw=0)
    axs[0].set_title("0.1 m 이동 후 잔류 자세 지터 (회색 = R4 스펙 ±0.25°)", loc="left")
    axs[0].set_ylabel("pitch [deg]")
    axs[0].set_xlabel("시간 [s]")
    axs[0].legend(loc="upper right")
    axs[1].set_title("위치 오차", loc="left")
    axs[1].set_ylabel("x 오차 [cm]")
    axs[1].set_xlabel("시간 [s]")
    axs[1].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_hover_jitter.png"), dpi=150)
    plt.close(fig)
    return res


# ---------------------------------------------------------------- 표
def write_tables(rows, extra, out):
    keys = ["mission", "T_traj_s", "track_rms_x_cm", "track_rms_y_cm", "track_rms_z_cm",
            "track_rms_3d_cm", "track_max_3d_cm", "endpoint_err_cm", "overshoot_cm",
            "z_sag_cm", "z_dev_max_cm", "hover_drift_cm", "pitch_rms_fly_deg", "roll_rms_fly_deg",
            "pitch_peak_deg", "roll_peak_deg", "att_tail_rms_deg", "yaw_rms_fly_deg",
            "prop_w_mean_rad_s", "prop_w_max_rad_s",
            "pass_track", "pass_overshoot", "pass_zdev", "pass_drift"]
    with open(os.path.join(out, f"summary_missions{TAG}.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=keys)
        wr.writeheader()
        for r in rows:
            wr.writerow({k: (f"{r[k]:.3f}" if isinstance(r[k], float) else r[k]) for k in keys})
    with open(os.path.join(out, f"summary_missions{TAG}.md"), "w", encoding="utf-8") as f:
        f.write("# 컨트롤러 성능 지표 요약 (perf_metrics.py 자동 생성)\n\n")
        f.write("스펙: PERFORMANCE_SPEC.md — 추종 RMS ≤ 10 cm, 오버슈트 ≤ 10 cm, z 이탈 ≤ 10 cm, "
                "도착 후 드리프트 ≤ 5 cm. 자세 R4/R5(≤0.25°/0.8°)는 회귀 기준값 0.48°/1.05° 병기.\n\n")
        f.write("## 미션 실비행 (sim_result_*.mat)\n\n")
        hdr = ["미션", "T[s]", "추종RMS 3D[cm]", "최대[cm]", "종점[cm]", "오버슈트[cm]", "이륙새그[cm]", "z이탈[cm]",
               "드리프트[cm]", "pitch/roll RMS[°]", "피크[°]", "tail 잔류[°]", "yaw RMS[°]", "판정"]
        f.write("| " + " | ".join(hdr) + " |\n|" + "---|" * len(hdr) + "\n")
        for r in rows:
            verdict = "".join("✅" if r[k] else "❌" for k in ("pass_track", "pass_overshoot", "pass_zdev", "pass_drift"))
            f.write(f"| {r['mission']} | {r['T_traj_s']:.1f} | {r['track_rms_3d_cm']:.2f} | "
                    f"{r['track_max_3d_cm']:.1f} | {r['endpoint_err_cm']:.2f} | {r['overshoot_cm']:.1f} | "
                    f"{r['z_sag_cm']:.1f} | {r['z_dev_max_cm']:.1f} | {r['hover_drift_cm']:.2f} | "
                    f"{r['pitch_rms_fly_deg']:.2f}/{r['roll_rms_fly_deg']:.2f} | "
                    f"{max(r['pitch_peak_deg'], r['roll_peak_deg']):.2f} | {r['att_tail_rms_deg']:.3f} | "
                    f"{r['yaw_rms_fly_deg']:.2f} | {verdict} |\n")
        f.write("\n판정 순서: 추종RMS / 오버슈트 / z이탈 / 드리프트\n")
        if extra.get("step"):
            f.write("\n## 위치 스텝 응답 (diagnose/results/step_ts_*.csv)\n\n")
            f.write("| 케이스 | 프로파일 | rise 10-90% [s] | 오버슈트 [%] | ±2% 정착 [s] | SSE [mm] |\n|---|---|---|---|---|---|\n")
            for r in extra["step"]:
                f.write(f"| {r['case']} | {r['profile']} | {r['rise_s']:.2f} | {r['overshoot_pct']:.1f} | "
                        f"{r['settle2pct_s']:.2f} | {r['sse_mm']:.1f} |\n")
        if extra.get("ramp"):
            f.write("\n## 등속 램프 추종 지연 (ramp_ts_*.csv)\n\n| 속도 | 프로파일 | 이동 중 RMS [cm] | 피크 [cm] | 정지 후 잔류 RMS [cm] |\n|---|---|---|---|---|\n")
            for r in extra["ramp"]:
                f.write(f"| {r['speed']} | {r['profile']} | {r['lag_rms_cm']:.2f} | {r['lag_peak_cm']:.2f} | {r['resid_rms_cm']:.2f} |\n")
        if extra.get("jitter"):
            f.write("\n## 잔류 지터 (jitctr_ts_*.csv, 도착 후 4 s)\n\n| 프로파일 | pitch RMS [°] | 피크 [°] |\n|---|---|---|\n")
            for r in extra["jitter"]:
                f.write(f"| {r['profile']} | {r['tail_pitch_rms_deg']:.3f} | {r['tail_pitch_peak_deg']:.3f} |\n")
    with open(os.path.join(out, f"summary{TAG}.json"), "w", encoding="utf-8") as f:
        json.dump({"missions": rows, **extra}, f, ensure_ascii=False, indent=1, default=float)


# ---------------------------------------------------------------- main
# ---------------------------------------------------------------- 폴더 정리 (경우별 하위 폴더)
# figure/ 평면에 생성된 산출물을 경우별 폴더로 옮긴다 (README 링크는 이 구조 기준). 규칙 순서 = 우선순위.
ORGANIZE_RULES = [
    (r"^fig_(mission|path)_.*_0kg\.png$|^fig_summary_tracking_0kg\.png$|^fig_yaw_missions_0kg\.png$|^summary(_missions)?_0kg\.(md|csv|json)$", "02_missions_0kg"),
    (r"^fig_(mission|path)_.*\.png$|^fig_summary_tracking\.png$|^fig_yaw_missions\.png$|^summary_missions\.(md|csv)$", "01_missions_1kg"),
    (r"^fig_(step_response|ramp_lag|hover_jitter)\.png$", "03_tuning_timeseries"),
    (r"^fig_bat_mass_sweep.*\.png$|^fig_0kg_mass_sweep_before_after\.png$", "06_mass_sweep"),
    (r"^fig_bat_.*_1kg(_raw)?\.png$", "04_battery_1kg"),
    (r"^fig_bat_.*_0kg(_raw)?\.png$", "05_battery_0kg"),
    (r"^fig_tune0kg_.*\.png$|^fig_0kg_before_after\.png$|^summary_0kg_compare\.md$|^fig_bat_.*_0kg_tuned.*\.png$", "07_retune_0kg"),   # _tuned*: r3b 후보 대조 배터리(sA 0.40 / 0.5)
]


def organize(out):
    """figure/ 최상위의 산출물을 ORGANIZE_RULES 대로 하위 폴더로 이동 (README.md, summary.json 은 최상위 유지)."""
    import re
    import shutil
    moved = 0
    for fn in os.listdir(out):
        src = os.path.join(out, fn)
        if not os.path.isfile(src):
            continue
        for pat, sub in ORGANIZE_RULES:
            if re.match(pat, fn):
                d = os.path.join(out, sub)
                os.makedirs(d, exist_ok=True)
                shutil.move(src, os.path.join(d, fn))
                moved += 1
                break
    return moved


def main(argv=None):
    ap = argparse.ArgumentParser(description="컨트롤러 성능 지표/그래프")
    ap.add_argument("--out", default=OUT_DIR)
    ap.add_argument("--model-dir", default=MODEL_DIR)
    ap.add_argument("--only", default=None, help="쉼표 구분 미션 이름")
    ap.add_argument("--tag", default="", help="접미 (예 _0kg): sim_result_<name><tag>.mat 만 처리, 그림/표도 접미 — 배터리·CSV 계열은 생략")
    args = ap.parse_args(argv)
    global TAG
    TAG = args.tag
    os.makedirs(args.out, exist_ok=True)
    _style()
    rows = []
    loaded = {}
    want = set(args.only.split(",")) if args.only else None
    for name, fn in MISSIONS.items():
        if want and name not in want:
            continue
        p = os.path.join(args.model_dir, fn.replace(".mat", f"{TAG}.mat"))
        if not os.path.exists(p):
            print(f"[skip] {name}: {p} 없음", file=sys.stderr)
            continue
        d = load_mission(p)
        m = mission_metrics(d)
        m["mission"] = name
        rows.append(m)
        loaded[name] = (d, m)
        fig_mission(name, d, m, args.out)
        fig_path(name, d, m, args.out)
        print(f"{name:12s} track {m['track_rms_3d_cm']:5.2f} cm  end {m['endpoint_err_cm']:5.2f} cm  "
              f"att {m['pitch_rms_fly_deg']:.2f}/{m['roll_rms_fly_deg']:.2f}°  tail {m['att_tail_rms_deg']:.3f}°  "
              f"zdev {m['z_dev_max_cm']:.1f} cm")
    if rows:
        fig_summary(rows, args.out)
        fig_yaw(loaded, args.out)
    extra = {}
    if TAG:
        write_tables(rows, extra, args.out)
        organize(args.out)
        print(f"→ {args.out} (tag {TAG})")
        return 0
    if os.path.isdir(RESULTS_DIR):
        extra["step"] = fig_step(args.out)
        extra["ramp"] = fig_ramp(args.out)
        extra["jitter"] = fig_jitter(args.out)
    write_tables(rows, extra, args.out)
    import perf_battery_plots
    bat = perf_battery_plots.run(args.out, prefix="perf_", fig_suffix="")
    perf_battery_plots.write_md(bat, args.out, "배포 구성: 스무더 + ZVD 셰이퍼")
    bat_raw = perf_battery_plots.run(args.out, prefix="perf_raw_", fig_suffix="_raw")
    perf_battery_plots.write_md(bat_raw, args.out, "셰이퍼 없음(스무더만) — 튜닝 하네스와 동일 구성")
    extra["battery"] = bat
    extra["battery_raw"] = bat_raw
    with open(os.path.join(args.out, f"summary{TAG}.json"), "w", encoding="utf-8") as f:
        json.dump({"missions": rows, **extra}, f, ensure_ascii=False, indent=1, default=float)
    print(f"정리: {organize(args.out)}개 파일 → 하위 폴더")
    print(f"→ {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
