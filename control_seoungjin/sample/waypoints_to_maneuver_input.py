"""
Waypoint 목록 -> Quadcopter-Drone-Model-Simscape (Maneuver Controller)용 입력 생성.

path_time.py 의 plan_waypoints() (waypoint 하나씩 잡아 7차 다항식으로
잇는 최소시간 궤적 계획)를 재사용해서, quadcopter_package_delivery.slx
안의 4개 1-D Lookup Table 블록이 요구하는 형식으로 저장한다.

    timespot_spl : (N,)   시간 breakpoint
    spline_data  : (N, 3) x, y, z 위치
    spline_yaw   : (N,)   yaw(rad), 진행 방향(heading) 기준

MATLAB에서 사용:
    >> load('trajectory.mat')   % timespot_spl, spline_data, spline_yaw 로드
    >> sim('quadcopter_package_delivery')
"""

import os
import sys

import numpy as np
from scipy.io import savemat

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from path_time import plan_waypoints  # noqa: E402  (control_seoungjin/path_time.py)


def _heading_yaw(vel, speed_eps=1e-4):
    """속도 벡터의 진행 방향(atan2(vy, vx))으로 yaw 계산. 정지 구간은 직전 yaw 유지."""
    speed_xy = np.hypot(vel[0], vel[1])
    yaw = np.arctan2(vel[1], vel[0])

    # 정지(속도 거의 0) 구간은 방향이 정의되지 않으므로 직전 값으로 고정
    for i in range(1, len(yaw)):
        if speed_xy[i] < speed_eps:
            yaw[i] = yaw[i - 1]

    return np.unwrap(yaw)


def waypoints_to_maneuver_input(waypoints, v_max, a_max, j_max, snap_max,
                                 dt=0.01, v0=None, a0=None, j0=None):
    """
    Returns
    -------
    timespot_spl : (N,)   시간 [s]
    spline_data  : (N, 3) x, y, z 위치 [m]
    spline_yaw   : (N,)   yaw [rad]
    """
    t_out, pos_out, vel_out, acc_out, jerk_out, T_total = plan_waypoints(
        waypoints, v_max, a_max, j_max, snap_max, v0=v0, a0=a0, j0=j0, dt=dt,
    )

    timespot_spl = t_out
    spline_data = pos_out.T          # (3, N) -> (N, 3)
    spline_yaw = _heading_yaw(vel_out)

    return timespot_spl, spline_data, spline_yaw


def save_for_matlab(path, timespot_spl, spline_data, spline_yaw, waypoints=None):
    """MATLAB에서 load()로 바로 workspace에 올릴 수 있는 .mat으로 저장.

    waypoints를 같이 저장하면 quadcopter_waypoints_to_path_vis()로
    Ground/Trajectory 시각화 블록(Spline/Waypoints)의 DataPoints도 채울 수 있다.
    """
    data = {
        "timespot_spl": timespot_spl.reshape(-1, 1),
        "spline_data": spline_data,
        "spline_yaw": spline_yaw.reshape(-1, 1),
    }
    if waypoints is not None:
        data["waypoints"] = np.asarray(waypoints, float)
    savemat(path, data)
    print(f"[save_for_matlab] 저장 완료: {path}")


if __name__ == "__main__":
    # 예시: waypoint를 하나씩 잡아 순서대로 통과하는 경로
    waypoints = np.array([
        [-2.0, -2.0, 0.15],
        [-2.0, -2.0, 6.0],
        [0.0,   0.0, 6.0],
        [2.0,   2.0, 6.0],
        [5.0,   0.0, 0.15],
    ])

    V_MAX, A_MAX, J_MAX, SNAP_MAX = 1.0, 0.8, 2.0, 10.0

    timespot_spl, spline_data, spline_yaw = waypoints_to_maneuver_input(
        waypoints, V_MAX, A_MAX, J_MAX, SNAP_MAX, dt=0.01,
    )

    save_for_matlab("trajectory.mat", timespot_spl, spline_data, spline_yaw, waypoints)
