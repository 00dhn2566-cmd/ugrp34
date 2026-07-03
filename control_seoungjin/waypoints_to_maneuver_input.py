"""
Waypoint 목록 -> Quadcopter-Drone-Model-Simscape (Maneuver Controller)용 입력 생성.

path_time.ipynb 의 plan_waypoints() (waypoint 하나씩 잡아 7차 다항식으로
잇는 최소시간 궤적 계획)를 그대로 재사용해서, quadcopter_package_delivery.slx
안의 4개 1-D Lookup Table 블록이 요구하는 형식으로 저장한다.

    timespot_spl : (N,)   시간 breakpoint
    spline_data  : (N, 3) x, y, z 위치
    spline_yaw   : (N,)   yaw(rad), 진행 방향(heading) 기준

MATLAB에서 사용:
    >> load('trajectory.mat')   % timespot_spl, spline_data, spline_yaw 로드
    >> sim('quadcopter_package_delivery')
"""

import numpy as np
from scipy.io import savemat


# ---------------------------------------------------------------------------
# plan_waypoints (path_time.ipynb 그대로)
# ---------------------------------------------------------------------------

def _to3(val):
    v = np.asarray(val, float).ravel()
    return np.full(3, v[0]) if v.size == 1 else v[:3].copy()


def _deriv_coeffs(c, order=1):
    """다항식 계수 (오름차순) -> order차 미분 계수."""
    c = np.asarray(c, float)
    for _ in range(order):
        if len(c) <= 1:
            return np.array([0.0])
        c = np.arange(1, len(c)) * c[1:]
    return c


def _eval_poly(c, t):
    """p(t) = c[0] + c[1]*t + c[2]*t^2 + ..."""
    c = np.asarray(c, float)
    t = np.asarray(t, float)
    return sum(ck * t**k for k, ck in enumerate(c))


def _poly7_coeffs(p0, v0, a0, j0, pf, T):
    c0, c1, c2, c3 = float(p0), float(v0), float(a0) / 2.0, float(j0) / 6.0
    T2, T3, T4, T5, T6, T7 = T**2, T**3, T**4, T**5, T**6, T**7
    A = np.array([
        [T4,    T5,     T6,     T7],
        [4*T3,  5*T4,   6*T5,   7*T6],
        [12*T2, 20*T3,  30*T4,  42*T5],
        [24*T,  60*T2, 120*T3, 210*T4],
    ])
    b = np.array([
        pf - p0 - v0*T - 0.5*a0*T2 - (j0/6.0)*T3,
        -v0 - a0*T - 0.5*j0*T2,
        -a0 - j0*T,
        -j0,
    ])
    c4, c5, c6, c7 = np.linalg.solve(A, b)
    return np.array([c0, c1, c2, c3, c4, c5, c6, c7])


def _seg_feasible(coeffs_3ax, T, v_max, a_max, j_max, snap_max, n=400):
    t = np.linspace(0, T, n)
    for i, c in enumerate(coeffs_3ax):
        if np.max(np.abs(_eval_poly(_deriv_coeffs(c, 1), t))) > v_max[i] + 1e-6:
            return False
        if np.max(np.abs(_eval_poly(_deriv_coeffs(c, 2), t))) > a_max[i] + 1e-6:
            return False
        if np.max(np.abs(_eval_poly(_deriv_coeffs(c, 3), t))) > j_max[i] + 1e-6:
            return False
        if np.max(np.abs(_eval_poly(_deriv_coeffs(c, 4), t))) > snap_max[i] + 1e-6:
            return False
    return True


def _find_min_time(p0, pf, v0, a0, j0, v_max, a_max, j_max, snap_max,
                    tol=1e-3, max_iter=60):
    def make_coeffs(T):
        return [_poly7_coeffs(p0[i], v0[i], a0[i], j0[i], pf[i], T) for i in range(3)]

    d = np.maximum(np.abs(pf - p0), 1e-9)
    T_lo = float(np.max([
        np.max(d / v_max),
        np.max(np.sqrt(2.0 * d / a_max)),
        np.max((6.0 * d / j_max) ** (1.0 / 3.0)),
        np.max((24.0 * d / snap_max) ** (1.0 / 4.0)),
        1e-4,
    ]))

    T_hi = T_lo
    for _ in range(40):
        if _seg_feasible(make_coeffs(T_hi), T_hi, v_max, a_max, j_max, snap_max):
            break
        T_hi *= 2.0

    for _ in range(max_iter):
        if T_hi - T_lo < tol:
            break
        T_mid = 0.5 * (T_lo + T_hi)
        if _seg_feasible(make_coeffs(T_mid), T_mid, v_max, a_max, j_max, snap_max):
            T_hi = T_mid
        else:
            T_lo = T_mid

    return T_hi, make_coeffs(T_hi)


def plan_waypoints(waypoints, v_max, a_max, j_max, snap_max,
                    v0=None, a0=None, j0=None, dt=0.01):
    """N개 경로점을 순서대로 최소시간 7차 다항식으로 잇는 궤적 계획."""
    waypoints = np.asarray(waypoints, float)
    if waypoints.ndim == 1:
        waypoints = waypoints.reshape(1, 3)

    v_max = _to3(v_max)
    a_max = _to3(a_max)
    j_max = _to3(j_max)
    snap_max = _to3(snap_max)

    n_wp = len(waypoints)
    p_cur = waypoints[0].copy()
    v_cur = np.zeros(3) if v0 is None else _to3(v0)
    a_cur = np.zeros(3) if a0 is None else _to3(a0)
    j_cur = np.zeros(3) if j0 is None else _to3(j0)

    t_segs, pos_segs, vel_segs = [], [], []
    t_offset = 0.0

    for k in range(1, n_wp):
        p_next = waypoints[k]
        if np.linalg.norm(p_next - p_cur) < 1e-9:
            continue

        T_opt, coeffs = _find_min_time(
            p_cur, p_next, v_cur, a_cur, j_cur, v_max, a_max, j_max, snap_max,
        )

        n_pts = max(2, int(np.round(T_opt / dt)) + 1)
        t_seg = np.linspace(0.0, T_opt, n_pts)

        p_arr = np.stack([_eval_poly(c, t_seg) for c in coeffs])
        v_arr = np.stack([_eval_poly(_deriv_coeffs(c, 1), t_seg) for c in coeffs])

        sl = slice(None) if k == 1 else slice(1, None)
        t_segs.append(t_seg[sl] + t_offset)
        pos_segs.append(p_arr[:, sl])
        vel_segs.append(v_arr[:, sl])

        t_offset += T_opt
        p_cur = p_next.copy()
        v_cur = np.zeros(3)
        a_cur = np.zeros(3)
        j_cur = np.zeros(3)

    t_out = np.concatenate(t_segs)
    pos_out = np.concatenate(pos_segs, axis=1)
    vel_out = np.concatenate(vel_segs, axis=1)
    T_total = t_offset

    print(f"[plan_waypoints] {n_wp - 1}개 세그먼트  총 소요시간: {T_total:.3f}s")
    return t_out, pos_out, vel_out, T_total


# ---------------------------------------------------------------------------
# yaw 계산 + Simulink 입력 포맷 변환
# ---------------------------------------------------------------------------

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
    t_out, pos_out, vel_out, T_total = plan_waypoints(
        waypoints, v_max, a_max, j_max, snap_max, v0=v0, a0=a0, j0=j0, dt=dt,
    )

    timespot_spl = t_out
    spline_data = pos_out.T          # (3, N) -> (N, 3)
    spline_yaw = _heading_yaw(vel_out)

    return timespot_spl, spline_data, spline_yaw


def save_for_matlab(path, timespot_spl, spline_data, spline_yaw):
    """MATLAB에서 load()로 바로 workspace에 올릴 수 있는 .mat으로 저장."""
    savemat(path, {
        "timespot_spl": timespot_spl.reshape(-1, 1),
        "spline_data": spline_data,
        "spline_yaw": spline_yaw.reshape(-1, 1),
    })
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

    save_for_matlab("trajectory.mat", timespot_spl, spline_data, spline_yaw)
