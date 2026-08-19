"""골든 트레이스 대조 — 복원한 식 vs Simscape (DYNAMICS.md §9 / §11.1).

방법: 잔차(residual) 대조
-------------------------
처음에는 Simscape가 기록한 n(t)를 독립 적분기에 먹여 **궤적을 길게 재생**해
비교하려 했으나 실패했다. 쿼드로터의 자세 동역학은 감쇠가 없는 이중적분기라,
호버 로그의 미세한 추력 불균형(0.08 N)만으로도 개루프 재생이 수십 초 만에
눕는다. 폐루프에서는 제어기가 그걸 계속 잡아주지만 재생에는 제어기가 없다.

그래서 **매 시점 가속도를 직접 대조**한다. 이것이 동역학 모델 검증의 표준 방법이며,
불안정성을 적분하지 않으므로 식 자체만 시험한다.

    측정 가속도 = d(v_Simscape)/dt          (로그를 수치미분)
    예측 가속도 = ( R [0,0,sum T] + [0,0,-mg] + F_drag ) / m     (복원한 식)

두 값이 같으면 §10.1이 맞다는 뜻이다. 회전도 같은 방식으로 자세를 두 번
미분해 각가속도를 얻고 §10.2와 대조한다.

계수 주의
--------
Simscape 는 kT = 9.79 로 633.7 rpm 에서 5.55 N 을 만든다. plant_sim 기본값
Ct = 0.1072 는 6057 rpm 에서 같은 힘을 내는 물리 현실 쌍이다. 재생에서 둘을
섞으면 추력이 91배 틀린다 (DYNAMICS.md §5.3). 따라서 **모델 계수를 그대로** 쓴다.

사용
----
    python compare_golden.py [csv경로]
"""

from __future__ import annotations

import math
import sys

import numpy as np

import plant_sim as ps

DEFAULT_CSV = (
    "controller/Quadcopter-Drone-Model-Simscape/diagnose/results/golden_plant_trace.csv"
)

# 대조 구간 — 이륙 과도 이후부터 기동 종료까지
T_START, T_END = 2.5, 8.0

# 모델이 실제로 쓰는 계수 (물리 현실 쌍이 아님 — 위 "계수 주의" 참조)
CT_MODEL = 9.79
CQ_MODEL = 0.597


def load(path: str) -> dict:
    raw = np.loadtxt(path, delimiter=",", skiprows=1)
    cols = "t w1 w2 w3 w4 px py pz vx vy vz roll pitch yaw".split()
    return {c: raw[:, i] for i, c in enumerate(cols)}


def smooth(x: np.ndarray, w: int = 15) -> np.ndarray:
    if w < 3:
        return x
    k = np.ones(w) / w
    return np.convolve(x, k, mode="same")


def deriv(x: np.ndarray, dt: float, w: int = 15) -> np.ndarray:
    return smooth(np.gradient(smooth(x, w), dt), w)


def euler_to_rot_zyx(roll, pitch, yaw) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
    try:
        d = load(path)
    except OSError:
        print(f"CSV를 찾을 수 없다: {path}")
        print("먼저 diagnose/golden_plant_export.m 을 실행할 것.")
        return 1

    t = d["t"]
    dt = float(t[1] - t[0])
    m = (t >= T_START) & (t <= T_END)

    print("=" * 70)
    print("골든 트레이스 대조 — 복원한 식 vs Simscape (잔차 방식)")
    print("=" * 70)
    print(f"입력 : {path}")
    print(f"구간 : t = {T_START} ~ {T_END} s   ({m.sum()}점, dt = {dt*1000:.1f} ms)")
    print()

    params = ps.PlantParams()
    params.ct, params.cq = CT_MODEL, CQ_MODEL
    plant = ps.Plant(params)

    n_tab = np.abs(np.column_stack([d[f"w{i}"] for i in (1, 2, 3, 4)])) / 60.0

    # ---------------- 병진 (§10.1) ----------------
    acc_meas = np.column_stack([deriv(d[k], dt) for k in ("vx", "vy", "vz")])
    vel = np.column_stack([d[k] for k in ("vx", "vy", "vz")])

    acc_pred = np.zeros_like(acc_meas)
    for i in range(len(t)):
        R = euler_to_rot_zyx(d["roll"][i], d["pitch"][i], d["yaw"][i])
        T_tot = float(np.sum(plant.thrust(n_tab[i])))
        f = R @ np.array([0.0, 0.0, T_tot])
        f = f + np.array([0.0, 0.0, -params.mass * params.g])
        f = f + plant.aero_force(vel[i], np.zeros(3))
        acc_pred[i] = f / params.mass

    res = acc_pred[m] - acc_meas[m]
    scale = np.abs(acc_meas[m]).max(axis=0)
    print("[병진 §10.1]  단위 m/s^2")
    print(f"{'축':<6}{'측정 최대':>12}{'예측 최대':>12}{'잔차 RMS':>12}{'잔차 최대':>12}{'상대':>9}")
    for j, ax in enumerate("xyz"):
        rel = np.sqrt(np.mean(res[:, j] ** 2)) / max(scale[j], 1e-9) * 100
        print(f"{ax:<6}{np.abs(acc_meas[m,j]).max():12.4f}"
              f"{np.abs(acc_pred[m,j]).max():12.4f}"
              f"{np.sqrt(np.mean(res[:,j]**2)):12.4f}"
              f"{np.abs(res[:,j]).max():12.4f}{rel:8.1f}%")

    # z축 정적 균형 (호버 구간) — 계수·질량 검증
    hov = (t >= 2.0) & (t <= 2.8)
    print(f"\n  호버 구간 z 잔차 평균 : {res[:, 2][: hov.sum()].mean():+.5f} m/s^2"
          f"  (0 이면 추력·질량·중력 정합)")

    # ---------------- 회전 (§10.2) ----------------
    # 각속도는 버스에 없으므로 오일러각을 미분해 근사한다 (소각도 구간이라 유효)
    omega = np.column_stack([deriv(d[k], dt) for k in ("roll", "pitch", "yaw")])
    alpha_meas = np.column_stack([deriv(omega[:, j], dt) for j in range(3)])

    alpha_pred = np.zeros_like(alpha_meas)
    for i in range(len(t)):
        tau = plant.body_torque(n_tab[i]) + plant.aero_torque(omega[i])
        gyro = np.cross(omega[i], params.inertia @ omega[i])
        alpha_pred[i] = plant.inv_inertia @ (tau - gyro)

    resr = alpha_pred[m] - alpha_meas[m]
    print("\n[회전 §10.2]  단위 rad/s^2   (각속도는 오일러각 미분 근사)")
    print(f"{'축':<8}{'측정 최대':>12}{'예측 최대':>12}{'잔차 RMS':>12}{'상관':>9}")
    for j, ax in enumerate(("roll", "pitch", "yaw")):
        a, b = alpha_meas[m, j], alpha_pred[m, j]
        corr = float(np.corrcoef(a, b)[0, 1]) if a.std() > 1e-12 and b.std() > 1e-12 else float("nan")
        print(f"{ax:<8}{np.abs(a).max():12.4f}{np.abs(b).max():12.4f}"
              f"{np.sqrt(np.mean(resr[:,j]**2)):12.4f}{corr:9.3f}")

    print("\n" + "=" * 70)
    print("판정: 병진 잔차가 측정 진폭 대비 수 % 이내면 §10.1 검증됨.")
    print("      회전은 상관계수가 높으면 부호·구조가 맞다는 뜻 (크기는 모멘트 암 의존).")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
