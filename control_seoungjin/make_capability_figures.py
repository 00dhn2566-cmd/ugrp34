"""실시간 능력 표 / 연산 부하 / 쉐이퍼 정지 거동 그림 생성.

2026-08-22. 출력: figure/10_capability/*.png  (기존 figure/ 번호 규칙 이어받음)

사용:
    python make_capability_figures.py
"""
from __future__ import annotations

import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from capability import build_capability, scale_from_rho
from compute_load import DEFAULT_COSTS, LoadEstimator, LoadGovernor
from traj_shaping import traj_smoother

OUT = os.path.join("figure", "10_capability")

for cand in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
    try:
        matplotlib.font_manager.findfont(cand, fallback_to_default=False)
        plt.rcParams["font.family"] = cand
        break
    except Exception:
        continue
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 130
plt.rcParams["savefig.bbox"] = "tight"

C_OLD, C_NEW, C_ACC = "#888888", "#1f77b4", "#d62728"


def _save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p)
    plt.close(fig)
    print(f"  {p}")
    return p


# ─────────────────────────────────────────────── ① 연산 비용 모델 (실측 + 적합)
def fig_cost_model():
    # 2026-08-22 이 노트북 실측
    N = np.array([251, 501, 1001, 2001, 4001])
    t_sm = np.array([6.33, 11.51, 22.77, 48.94, 99.65])       # ms
    nseg = np.array([1, 2, 4, 8])
    t_pl = np.array([5.2, 12.5, 20.5, 42.2])                  # ms

    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    a = np.vstack([np.ones_like(N, float), N.astype(float)]).T
    coef, *_ = np.linalg.lstsq(a, t_sm, rcond=None)
    xs = np.linspace(0, 4200, 50)
    ax[0].plot(N, t_sm, "o", color=C_NEW, label="실측")
    ax[0].plot(xs, coef[0] + coef[1] * xs, "--", color=C_ACC,
               label=f"적합 {coef[1]*1000:.1f} µs/샘플")
    ax[0].set_xlabel("샘플 수 N"); ax[0].set_ylabel("소요 [ms]")
    ax[0].set_title("traj_smoother 비용")
    ax[0].legend(); ax[0].grid(alpha=.3)

    cp, *_ = np.linalg.lstsq(np.vstack([np.ones_like(nseg, float),
                                        nseg.astype(float)]).T, t_pl, rcond=None)
    xs2 = np.linspace(0, 9, 20)
    ax[1].plot(nseg, t_pl, "o", color=C_NEW, label="실측")
    ax[1].plot(xs2, cp[0] + cp[1] * xs2, "--", color=C_ACC,
               label=f"적합 {cp[1]:.1f} ms/세그")
    ax[1].set_xlabel("세그먼트 수"); ax[1].set_ylabel("소요 [ms]")
    ax[1].set_title("plan_waypoints 비용")
    ax[1].legend(); ax[1].grid(alpha=.3)
    fig.suptitle("연산 비용 모델 — 실측 기반 (2026-08-22)", y=1.02)
    return _save(fig, "fig_cost_model.png"), {"smoother_us_per_sample": coef[1] * 1000,
                                              "plan_ms_per_seg": cp[1]}


# ─────────────────────────────────────────── ② 점유율 → 지연 (포화 발산)
def fig_load_latency():
    rates = np.linspace(0.5, 37.0, 200)
    duty, lat = [], []
    for r in rates:
        e = LoadEstimator()
        e.set_task("smoother", 1000, r)
        duty.append(e.duty())
        lat.append(e.predicted_latency_s() * 1e3)
    duty, lat = np.array(duty), np.array(lat)

    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    ax.plot(duty, lat, color=C_NEW, lw=2)
    ax.axvline(0.5, ls=":", color="gray")
    ax.axvline(0.9, ls=":", color=C_ACC)
    ax.text(0.51, ax.get_ylim()[1] * .55, "duty 0.5", fontsize=8, color="gray")
    ax.text(0.83, ax.get_ylim()[1] * .75, "0.9", fontsize=8, color=C_ACC)
    ax.set_xlabel("연산 점유율 duty"); ax.set_ylabel("예측 체류 지연 [ms]")
    ax.set_title("부하 → 지연 (M/D/1):  조금만 더 얹으면 갑자기 늦어진다")
    ax.grid(alpha=.3)
    return _save(fig, "fig_load_latency.png"), None


# ───────────────────────────────── ③ 조속기 시계열 (예상 + 실측 + 적용, 복귀 포함)
def fig_governor_timeline():
    g = LoadGovernor(hold_n=5, fall_tau_s=1.0)
    dt = 0.2
    pred, meas, appl = [], [], []
    seq = ([(19.4, 20.0)] * 10 + [(19.4, 120.0)] * 8 + [(19.4, 20.0)] * 20
           + [(2.2, 2.5)] * 40)
    for p, m in seq:
        appl.append(g.update(p / 1e3, m / 1e3, dt=dt) * 1e3)
        pred.append(p); meas.append(m)
    t = np.arange(len(seq)) * dt

    fig, ax = plt.subplots(figsize=(7.6, 3.8))
    ax.step(t, pred, where="post", color="#2ca02c", lw=1.2, label="예상 (부하 모델)")
    ax.step(t, meas, where="post", color=C_OLD, lw=1.2, label="실측")
    ax.plot(t, appl, color=C_ACC, lw=2.2, label="적용 (융합 + 비대칭 복귀)")
    ax.axvspan(2.0, 3.6, color="#ffcccc", alpha=.35)
    ax.axvspan(7.6, 16.0, color="#cce5ff", alpha=.35)
    ax.set_ylim(-6, 148)
    ax.annotate("실측 급증", xy=(2.8, 132), fontsize=8, color=C_ACC, ha="center")
    ax.annotate("부하 감소 → 확인 후 천천히 복귀", xy=(11.5, 132), fontsize=8,
                color=C_NEW, ha="center")
    ax.set_xlabel("시간 [s]"); ax.set_ylabel("지연 [ms]")
    ax.set_title("지연 조속 — 올릴 땐 즉시, 내릴 땐 dwell 후 감쇠")
    ax.legend(fontsize=8); ax.grid(alpha=.3)
    return _save(fig, "fig_governor_timeline.png"), None


# ────────────────────────────────────── ④ 능력 표 감쇄 (외란·지연·질량)
def fig_capability_derate():
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.6))

    rho = np.linspace(0, 1, 200)
    base = build_capability(pkg_kg=1.0)["limits"]
    for k, c, lbl in (("v", "#1f77b4", "v ∝ s"), ("a", "#ff7f0e", "a ∝ s²"),
                      ("j", "#2ca02c", "j ∝ s³"), ("snap", "#9467bd", "snap ∝ s⁴")):
        y = [build_capability(pkg_kg=1.0, rho=r)["limits"][k] / base[k] for r in rho]
        ax[0].plot(rho, y, color=c, label=lbl)
    ax[0].plot(rho, [scale_from_rho(r) for r in rho], "k--", lw=1, label="시계 배율 s")
    ax[0].set_xlabel("외란 권한 점유율 rho"); ax[0].set_ylabel("기저 대비 비율")
    ax[0].set_title("외란 → 한계 감쇄"); ax[0].legend(fontsize=7); ax[0].grid(alpha=.3)

    lat = np.logspace(-3, -0.3, 200)
    for pkg, c in ((0.0, "#d62728"), (1.0, "#1f77b4")):
        y = [build_capability(pkg_kg=pkg, latency_s=L)["limits"]["v"] for L in lat]
        ax[1].plot(lat * 1e3, y, color=c, label=f"{pkg:g} kg")
    ax[1].set_xscale("log")
    ax[1].set_xlabel("지연 [ms]"); ax[1].set_ylabel("허용 v [m/s]")
    ax[1].set_title("지연 → 속도 상한  (v ≤ ½·track/τ)")
    ax[1].legend(fontsize=8); ax[1].grid(alpha=.3, which="both")

    pkgs = np.linspace(0, 2, 100)
    for k, c in (("v", "#1f77b4"), ("a", "#ff7f0e")):
        ax[2].plot(pkgs, [build_capability(pkg_kg=m)["limits"][k] for m in pkgs],
                   color=c, label=k)
    ax[2].set_xlabel("짐 질량 [kg]"); ax[2].set_ylabel("한계")
    ax[2].set_title("질량 → 기저 한계 (앵커 보간)")
    ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
    fig.suptitle("capability.json — 지금 줘도 되는 스펙이 무엇으로 깎이나", y=1.03)
    return _save(fig, "fig_capability_derate.png"), None


# ───────────────────────────────────── ⑤ 쉐이퍼 정지 거동 (구판 vs 신판)
def fig_shaper_stop():
    DT = 0.01
    cases = [("계단 3 m", 12.0, 3.0), ("계단 0.5 m", 8.0, 0.5)]
    fig, ax = plt.subplots(2, 2, figsize=(10, 6))
    ov = {"구판": [], "신판": [], "name": []}
    for i, (name, T, amp) in enumerate(cases):
        t = np.arange(0, T + DT / 2, DT)
        p = np.where(t >= 1.0, amp, 0.0)
        for tag, flag, col in (("구판", False, C_OLD), ("신판", True, C_NEW)):
            o, _ = traj_smoother(t, p, 2.0, 2.0, 10.0, smooth_stop=flag)
            ax[0, i].plot(t, o, color=col, lw=1.8, label=tag)
            ax[1, i].plot(t[1:], np.diff(o) / DT, color=col, lw=1.5, label=tag)
            ov[tag].append(100 * max(o.max() - p.max(), 0.0))
        ov["name"].append(name)
        ax[0, i].plot(t, p, "k:", lw=1, label="입력")
        ax[0, i].axhline(amp, color=C_ACC, ls="--", lw=.8)
        ax[0, i].set_title(name); ax[0, i].set_ylabel("위치 [m]")
        ax[0, i].legend(fontsize=8); ax[0, i].grid(alpha=.3)
        ax[1, i].set_xlabel("시간 [s]"); ax[1, i].set_ylabel("속도 [m/s]")
        ax[1, i].grid(alpha=.3)
    fig.suptitle("쉐이퍼 정지 거동 — 뱅뱅 제동(구판) vs 거리 연동 포락선(신판)", y=1.0)
    p1 = _save(fig, "fig_shaper_stop.png")

    fig2, ax2 = plt.subplots(figsize=(5.2, 3.4))
    x = np.arange(len(ov["name"]))
    ax2.bar(x - .18, ov["구판"], .36, color=C_OLD, label="구판")
    ax2.bar(x + .18, ov["신판"], .36, color=C_NEW, label="신판")
    for xi, (a, b) in enumerate(zip(ov["구판"], ov["신판"])):
        ax2.text(xi - .18, a, f"{a:.2f}", ha="center", va="bottom", fontsize=8)
        ax2.text(xi + .18, b, f"{b:.2f}", ha="center", va="bottom", fontsize=8)
    ax2.set_xticks(x); ax2.set_xticklabels(ov["name"])
    ax2.set_ylabel("오버슈트 [cm]"); ax2.set_title("정지 오버슈트")
    ax2.legend(fontsize=8); ax2.grid(alpha=.3, axis="y")
    return (p1, _save(fig2, "fig_shaper_overshoot.png")), ov


def main():
    print(f"그림 출력 -> {OUT}")
    _, cost = fig_cost_model()
    fig_load_latency()
    fig_governor_timeline()
    fig_capability_derate()
    _, ov = fig_shaper_stop()

    e = LoadEstimator(); e.set_task("smoother", 2000, 2.0); e.set_task("plan_segment", 8, 1.0)
    summary = {
        "generated": "2026-08-22",
        "cost_model": {k: round(v, 3) for k, v in cost.items()},
        "cost_defaults": {k: list(v) for k, v in DEFAULT_COSTS.items()},
        "load_example_heavy": e.snapshot(),
        "shaper_overshoot_cm": {n: {"old": round(a, 3), "new": round(b, 3)}
                                for n, a, b in zip(ov["name"], ov["구판"], ov["신판"])},
        "capability_examples": {
            "1kg_nominal": build_capability(pkg_kg=1.0)["limits"],
            "1kg_rho031": build_capability(pkg_kg=1.0, rho=0.31)["limits"],
            "1kg_rho031_lat40ms": build_capability(pkg_kg=1.0, rho=0.31,
                                                   latency_s=0.04)["limits"],
            "0kg_rho07_yaw30": build_capability(pkg_kg=0.0, rho=0.7,
                                                yaw_err_rad=math.radians(30))["limits"],
        },
    }
    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(f"  {os.path.join(OUT, 'summary.json')}")


if __name__ == "__main__":
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
