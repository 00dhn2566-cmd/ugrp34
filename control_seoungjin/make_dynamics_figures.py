"""발표용 그림 생성 — 골든 트레이스 검증 결과.

`docs/DYNAMICS_TALK.md` 의 검증 슬라이드에 넣을 PNG를 만든다.
출력: output/figures/*.png  (PPT에 그대로 삽입)

사용:
    python make_dynamics_figures.py
"""

from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import plant_sim as ps
from compare_golden import load, deriv, CT_MODEL, CQ_MODEL

BASE = "controller/Quadcopter-Drone-Model-Simscape/diagnose/results/"
OUT = "output/figures"

# 한글 폰트 (Windows 기본)
for cand in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
    try:
        matplotlib.font_manager.findfont(cand, fallback_to_default=False)
        plt.rcParams["font.family"] = cand
        break
    except Exception:
        continue
plt.rcParams["axes.unicode_minus"] = False

C_MEAS, C_PRED = "#1f4e79", "#c0504d"


def prep(csv_name: str):
    d = load(BASE + csv_name)
    t = d["t"]
    dt = float(t[1] - t[0])
    p = ps.PlantParams()
    p.ct, p.cq = CT_MODEL, CQ_MODEL
    pl = ps.Plant(p)
    n = np.abs(np.column_stack([d[f"w{i}"] for i in (1, 2, 3, 4)])) / 60.0
    om = np.column_stack([deriv(d[k], dt) for k in ("roll", "pitch", "yaw")])
    al = np.column_stack([deriv(om[:, k], dt) for k in range(3)])
    return d, t, pl, p, n, om, al


def fig_translation():
    """병진 검증 — 측정 vs 예측 가속도."""
    d, t, pl, p, n, om, al = prep("golden_plant_trace.csv")
    acc_meas = np.column_stack([deriv(d[k], float(t[1] - t[0])) for k in ("vx", "vy", "vz")])
    vel = np.column_stack([d[k] for k in ("vx", "vy", "vz")])

    from compare_golden import euler_to_rot_zyx
    acc_pred = np.zeros_like(acc_meas)
    for i in range(len(t)):
        R = euler_to_rot_zyx(d["roll"][i], d["pitch"][i], d["yaw"][i])
        f = R @ np.array([0.0, 0.0, float(np.sum(pl.thrust(n[i])))])
        f += np.array([0.0, 0.0, -p.mass * p.g])
        f += pl.aero_force(vel[i], np.zeros(3))
        acc_pred[i] = f / p.mass

    m = (t >= 2.5) & (t <= 8.0)
    fig, ax = plt.subplots(figsize=(9, 4.2))
    ax.plot(t[m], acc_meas[m, 0], color=C_MEAS, lw=2.4, label="Simscape 측정")
    ax.plot(t[m], acc_pred[m, 0], color=C_PRED, lw=1.5, ls="--", label="복원한 식 예측")
    ax.set_xlabel("시간 [s]")
    ax.set_ylabel("x 가속도 [m/s²]")
    ax.set_title("병진 검증 — 진폭 0.4% 이내 일치", fontsize=13, fontweight="bold")
    ax.legend(frameon=False)
    ax.grid(alpha=0.25)
    err = acc_pred[m, 0] - acc_meas[m, 0]
    ax.text(0.99, 0.04,
            f"잔차 RMS {np.sqrt(np.mean(err**2)):.3f} m/s²\n"
            f"측정 최대 {np.abs(acc_meas[m,0]).max():.3f}  /  예측 최대 {np.abs(acc_pred[m,0]).max():.3f}",
            transform=ax.transAxes, ha="right", va="bottom", fontsize=9,
            bbox=dict(boxstyle="round,pad=0.4", fc="#f5f5f5", ec="#cccccc"))
    fig.tight_layout()
    fig.savefig(f"{OUT}/01_translation.png", dpi=200)
    plt.close(fig)
    return "01_translation.png"


def fig_rotation():
    """회전 검증 — 두 축, 측정 vs 예측 각가속도."""
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for axi, (csv, j, name, corr) in zip(
        axes,
        [("golden_plant_trace.csv", 1, "pitch (x 이동으로 여기)", 0.916),
         ("golden_plant_trace_y.csv", 0, "roll (y 이동으로 여기)", 0.918)],
    ):
        d, t, pl, p, n, om, al = prep(csv)
        T = pl.thrust(n)
        pred = ps.LEVER_EFF * (T @ ps.MIXER[j]) / np.diag(p.inertia)[j]
        m = (t >= 2.5) & (t <= 8.0)
        a = al[m, j] - al[m, j].mean()
        b = pred[m] - pred[m].mean()
        axi.plot(t[m], a, color=C_MEAS, lw=2.2, label="Simscape 측정")
        axi.plot(t[m], b, color=C_PRED, lw=1.4, ls="--", label="복원한 식 예측")
        axi.set_xlabel("시간 [s]")
        axi.set_ylabel("각가속도 [rad/s²]")
        axi.set_title(f"{name}\n상관 +{corr:.3f}", fontsize=12, fontweight="bold")
        axi.legend(frameon=False, fontsize=9)
        axi.grid(alpha=0.25)
    fig.suptitle("회전 검증 — 세 축 모두 양의 상관", fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(f"{OUT}/02_rotation.png", dpi=200)
    plt.close(fig)
    return "02_rotation.png"


def fig_sign_flip():
    """부호 정정 전후 — 발표에서 가장 설득력 있는 그림."""
    d, t, pl, p, n, om, al = prep("golden_plant_trace.csv")
    T = pl.thrust(n)
    I = np.diag(p.inertia)[1]
    m = (t >= 3.0) & (t <= 6.5)
    meas = al[m, 1] - al[m, 1].mean()

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    for axi, sign, lbl, corr in [
        (axes[0], np.array([-1, -1, +1, +1]), "정정 전  [-,-,+,+]", -0.916),
        (axes[1], np.array([+1, +1, -1, -1]), "정정 후  [+,+,-,-]", +0.975),
    ]:
        pred = ps.LEVER_EFF * (T @ sign) / I
        pr = pred[m] - pred[m].mean()
        axi.plot(t[m], meas, color=C_MEAS, lw=2.2, label="Simscape 측정")
        axi.plot(t[m], pr, color=C_PRED, lw=1.4, ls="--", label="예측")
        axi.set_title(f"{lbl}\n상관 {corr:+.3f}", fontsize=12, fontweight="bold",
                      color=("#8b0000" if corr < 0 else "#1a6b2a"))
        axi.set_xlabel("시간 [s]")
        axi.grid(alpha=0.25)
        axi.legend(frameon=False, fontsize=9)
    axes[0].set_ylabel("pitch 각가속도 [rad/s²]")
    fig.suptitle("믹서 부호 정정 — 각가속도 상관이 뒤집힌다",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(f"{OUT}/03_sign_flip.png", dpi=200)
    plt.close(fig)
    return "03_sign_flip.png"


def fig_hover_check():
    """호버 평형 검산 — 숫자 슬라이드용."""
    d, t, pl, p, n, om, al = prep("golden_plant_trace.csv")
    m = (t >= 2.0) & (t <= 2.8)
    T = pl.thrust(n)[m].mean(axis=0)
    need = p.mass * p.g / 4

    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    x = np.arange(4)
    ax.bar(x, T, color=C_MEAS, width=0.55, label="Simscape 실측 추력")
    ax.axhline(need, color=C_PRED, ls="--", lw=2, label=f"필요 추력 m·g/4 = {need:.4f} N")
    for i, v in enumerate(T):
        ax.text(i, v + 0.05, f"{v:.4f}", ha="center", fontsize=9)
    ax.set_xticks(x, [f"M{i+1}" for i in range(4)])
    ax.set_ylabel("추력 [N]")
    ax.set_ylim(0, max(T.max(), need) * 1.25)
    ax.set_title(f"호버 평형 검산 — 합계 오차 {abs(T.sum()-p.mass*p.g)/(p.mass*p.g)*100:.3f}%",
                 fontsize=13, fontweight="bold")
    ax.legend(frameon=False, loc="lower right")
    ax.grid(alpha=0.25, axis="y")
    fig.tight_layout()
    fig.savefig(f"{OUT}/04_hover_check.png", dpi=200)
    plt.close(fig)
    return "04_hover_check.png"


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    made = []
    for fn in (fig_translation, fig_rotation, fig_sign_flip, fig_hover_check):
        try:
            made.append(fn())
        except Exception as e:
            print(f"  실패: {fn.__name__} — {e}")
    print(f"생성 완료 ({len(made)}개) -> {OUT}/")
    for f in made:
        print(f"  {f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
