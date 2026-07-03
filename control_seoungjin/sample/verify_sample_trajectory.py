"""
샘플 waypoint로 plan_waypoints()를 돌려서 v_max/a_max/j_max 제약을
실제로 만족하는지 확인하고, 3D 경로 + 속도/가속도/저크 그래프를 저장한다.

주의: plan_waypoints()의 v_max/a_max/j_max/snap_max는 축별(x,y,z)로 각각
적용되는 제약이라, 대각선 방향 이동에서는 벡터 크기(|v|=sqrt(vx^2+vy^2+vz^2))가
축별 한계보다 커질 수 있다 (예: vx=vy=v_max이면 |v|=v_max*sqrt(2)).
축별 값과 벡터 크기를 둘 다 출력해서 이 차이를 확인할 수 있게 했다.
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from path_time import plan_waypoints  # noqa: E402  (control_seoungjin/path_time.py)


def main():
    waypoints = np.array([
        [-2.0, -2.0, 0.15],
        [-2.0, -2.0, 6.0],
        [0.0,   0.0, 6.0],
        [2.0,   2.0, 6.0],
        [5.0,   0.0, 0.15],
    ])

    V_MAX, A_MAX, J_MAX, SNAP_MAX = 1.0, 0.8, 2.0, 10.0

    t, pos, vel, acc, jerk, T = plan_waypoints(
        waypoints, V_MAX, A_MAX, J_MAX, SNAP_MAX, dt=0.01,
    )

    speed = np.linalg.norm(vel, axis=0)
    accel = np.linalg.norm(acc, axis=0)
    jerk_n = np.linalg.norm(jerk, axis=0)

    print(f"총 시간: {T:.3f}s, 샘플 수: {len(t)}")
    print(f"speed(벡터 크기) max: {speed.max():.4f}  (축별 한계 {V_MAX})")
    print(f"accel(벡터 크기) max: {accel.max():.4f}  (축별 한계 {A_MAX})")
    print(f"jerk (벡터 크기) max: {jerk_n.max():.4f}  (축별 한계 {J_MAX})")
    print(f"축별 vx/vy/vz max: "
          f"{np.abs(vel[0]).max():.4f} / {np.abs(vel[1]).max():.4f} / {np.abs(vel[2]).max():.4f}")

    fig = plt.figure(figsize=(14, 8))

    ax3d = fig.add_subplot(2, 3, 1, projection="3d")
    ax3d.plot(pos[0], pos[1], pos[2], lw=1.5)
    ax3d.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], color="r", label="waypoints")
    ax3d.set_title("3D path")
    ax3d.legend()

    ax1 = fig.add_subplot(2, 3, 2)
    ax1.plot(t, speed)
    ax1.axhline(V_MAX, color="r", ls="--", label="축별 한계")
    ax1.set_title("speed |v(t)| (벡터 크기)")
    ax1.set_xlabel("t [s]")
    ax1.legend()

    ax2 = fig.add_subplot(2, 3, 3)
    ax2.plot(t, accel)
    ax2.axhline(A_MAX, color="r", ls="--")
    ax2.set_title("accel |a(t)| (벡터 크기)")
    ax2.set_xlabel("t [s]")

    ax3 = fig.add_subplot(2, 3, 4)
    ax3.plot(t, jerk_n)
    ax3.axhline(J_MAX, color="r", ls="--")
    ax3.set_title("jerk |j(t)| (벡터 크기)")
    ax3.set_xlabel("t [s]")

    ax4 = fig.add_subplot(2, 3, 5)
    ax4.plot(t, pos[0], label="x")
    ax4.plot(t, pos[1], label="y")
    ax4.plot(t, pos[2], label="z")
    ax4.set_title("position vs time")
    ax4.set_xlabel("t [s]")
    ax4.legend()

    plt.tight_layout()
    plt.savefig("sample_trajectory.png", dpi=110)
    print("저장 완료: sample_trajectory.png")


if __name__ == "__main__":
    main()
