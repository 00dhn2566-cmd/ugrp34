"""
path_time.ipynb 핵심 로직만 남긴 간략 버전 (검증/플롯 셀 제외).

경로(x, y, z)에 시간을 부여해서 PID 컨트롤러에 넣을 feed를 만드는 두 가지 방식:

1. arc-length 기반: reparameterize_by_arc_length -> compute_curvature_and_kN
   -> generate_velocity_profile -> generate_pid_reference
   (연속 경로에 v_max/a_max/j_max + 곡률 제약을 적용해 시간 부여)

2. waypoint 기반: plan_waypoints
   (waypoint를 하나씩 순서대로 최소시간 7차 다항식으로 이어서 시간 부여)
"""

import numpy as np
from scipy.interpolate import CubicSpline


# ---------------------------------------------------------------------------
# 1. arc-length 재매개변수화 + 곡률
# ---------------------------------------------------------------------------

def reparameterize_by_arc_length(x, y, z, n_points=None, ds=None):
    """3D 경로를 등간격 호 길이(arc-length)로 재매개변수화."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    z = np.asarray(z, dtype=float)

    segment_lengths = np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2)
    s = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    total_length = s[-1]

    # 길이 0인 구간(중복 점) 제거 -> CubicSpline duplicate 에러 방지
    unique_mask = np.concatenate([[True], segment_lengths > 0])
    s = s[unique_mask]
    x, y, z = x[unique_mask], y[unique_mask], z[unique_mask]

    if ds is not None:
        n_points = int(np.floor(total_length / ds)) + 1
    elif n_points is None:
        n_points = len(x)

    s_uniform = np.linspace(0.0, total_length, n_points)

    cs_x = CubicSpline(s, x)
    cs_y = CubicSpline(s, y)
    cs_z = CubicSpline(s, z)

    return cs_x(s_uniform), cs_y(s_uniform), cs_z(s_uniform), s_uniform


def compute_curvature_and_kN(x, y, z, s):
    """arc-length 재매개변수화된 경로에서 TNB 틀의 곡률 kappa와 kappa*N 계산."""
    cs_x = CubicSpline(s, x)
    cs_y = CubicSpline(s, y)
    cs_z = CubicSpline(s, z)

    kN_x = cs_x(s, 2)
    kN_y = cs_y(s, 2)
    kN_z = cs_z(s, 2)

    kappa = np.sqrt(kN_x**2 + kN_y**2 + kN_z**2)

    return kappa, kN_x, kN_y, kN_z


# ---------------------------------------------------------------------------
# 2. 속도 프로파일 + PID feed (arc-length 기반)
# ---------------------------------------------------------------------------

def generate_velocity_profile(s, kappa, v_max, a_max, j_max):
    """Forward-backward scan으로 최소시간 속도 프로파일 생성."""
    n = len(s)
    ds = np.diff(s)
    eps = 1e-9

    v_curve = np.where(kappa > eps, np.sqrt(a_max / (kappa + eps)), v_max)
    v_lim = np.minimum(v_max, v_curve)

    v_fwd = np.zeros(n)
    a_fwd = np.zeros(n)
    for i in range(n - 1):
        dsi = ds[i]
        vi, ai = v_fwd[i], a_fwd[i]
        dt_est = dsi / max(vi, eps)
        a_avail = min(a_max, ai + j_max * dt_est)
        a_avail = max(a_avail, 0.0)
        v_next = np.sqrt(max(vi**2 + 2.0 * a_avail * dsi, 0.0))
        v_next = min(v_next, v_lim[i + 1])
        v_fwd[i + 1] = v_next
        a_fwd[i + 1] = (v_next**2 - vi**2) / (2.0 * dsi + eps)

    v_bwd = np.zeros(n)
    a_bwd = np.zeros(n)
    for i in range(n - 1, 0, -1):
        dsi = ds[i - 1]
        vi, ai = v_bwd[i], a_bwd[i]
        dt_est = dsi / max(vi, eps)
        a_avail = min(a_max, ai + j_max * dt_est)
        a_avail = max(a_avail, 0.0)
        v_prev = np.sqrt(max(vi**2 + 2.0 * a_avail * dsi, 0.0))
        v_prev = min(v_prev, v_lim[i - 1])
        v_bwd[i - 1] = v_prev
        a_bwd[i - 1] = (v_prev**2 - vi**2) / (2.0 * dsi + eps)

    v = np.minimum(v_fwd, v_bwd)
    return np.clip(v, eps, None)   # 시간 적분 시 0 나눔 방지


def _velocity_to_time(s, v):
    """arc-length 속도 프로파일 -> 누적 시간 배열."""
    ds = np.diff(s)
    v_mid = 0.5 * (v[:-1] + v[1:])
    dt = ds / np.maximum(v_mid, 1e-9)
    return np.concatenate([[0.0], np.cumsum(dt)])


def generate_pid_reference(x_r, y_r, z_r, s, kappa, total_time,
                            v_max, a_max, j_max, dt=0.01):
    """
    PID 제어기용 reference trajectory 생성.

    Returns
    -------
    t_out    : (N,)   균일 시간 배열
    pos      : (3, N) 위치
    vel      : (3, N) 속도 (velocity feedforward)
    acc      : (3, N) 가속도 (acceleration feedforward)
    t_actual : 실제 사용된 총 시간 (요청값 또는 자동 연장값)
    """
    v_s = generate_velocity_profile(s, kappa, v_max, a_max, j_max)

    t_s = _velocity_to_time(s, v_s)
    T_min = t_s[-1]

    if total_time < T_min:
        print(f"[경고] 요청 시간 {total_time:.3f}s < 최소 가능 시간 {T_min:.3f}s"
              f" -> {T_min:.3f}s 로 자동 연장")
        total_time = T_min

    scale = T_min / total_time
    v_s_scaled = v_s * scale
    t_s_scaled = _velocity_to_time(s, v_s_scaled)

    t_out = np.linspace(0.0, t_s_scaled[-1], int(np.round(total_time / dt)) + 1)

    cs_x = CubicSpline(t_s_scaled, x_r)
    cs_y = CubicSpline(t_s_scaled, y_r)
    cs_z = CubicSpline(t_s_scaled, z_r)

    pos = np.stack([cs_x(t_out),    cs_y(t_out),    cs_z(t_out)])
    vel = np.stack([cs_x(t_out, 1), cs_y(t_out, 1), cs_z(t_out, 1)])
    acc = np.stack([cs_x(t_out, 2), cs_y(t_out, 2), cs_z(t_out, 2)])

    return t_out, pos, vel, acc, total_time


# ---------------------------------------------------------------------------
# 3. waypoint 기반 7차 다항식 궤적 계획
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
    """[0,T] 구간에서 3축 다항식이 v/a/j/snap 제약을 모두 만족하면 True."""
    t = np.linspace(0, T, n)
    for i, c in enumerate(coeffs_3ax):
        if np.max(np.abs(_eval_poly(_deriv_coeffs(c, 1), t))) > v_max[i]    + 1e-6: return False
        if np.max(np.abs(_eval_poly(_deriv_coeffs(c, 2), t))) > a_max[i]    + 1e-6: return False
        if np.max(np.abs(_eval_poly(_deriv_coeffs(c, 3), t))) > j_max[i]    + 1e-6: return False
        if np.max(np.abs(_eval_poly(_deriv_coeffs(c, 4), t))) > snap_max[i] + 1e-6: return False
    return True


def _find_min_time(p0, pf, v0, a0, j0,
                    v_max, a_max, j_max, snap_max,
                    tol=1e-3, max_iter=60):
    """제약을 만족하는 최소 세그먼트 시간 T를 이진탐색으로 반환."""

    def make_coeffs(T):
        return [_poly7_coeffs(p0[i], v0[i], a0[i], j0[i], pf[i], T) for i in range(3)]

    d = np.maximum(np.abs(pf - p0), 1e-9)
    T_lo = float(np.max([
        np.max(d / v_max),
        np.max(np.sqrt(2.0 * d / a_max)),
        np.max((6.0 * d / j_max)    ** (1.0/3.0)),
        np.max((24.0 * d / snap_max) ** (1.0/4.0)),
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


def plan_waypoints(waypoints,
                    v_max, a_max, j_max, snap_max,
                    v0=None, a0=None, j0=None,
                    dt=0.01):
    """
    N개 경로점에 대한 순차적 최소시간 궤적 계획.

    waypoints : (N, 3)  경로점. waypoints[0] = 출발점.
    v_max/a_max/j_max/snap_max : 스칼라 또는 [x,y,z] 상한
    v0, a0, j0 : 초기 속도/가속도/저크 (기본 0)

    Returns
    -------
    t_out, pos, vel, acc, jerk, T_total
    """
    waypoints = np.asarray(waypoints, float)
    if waypoints.ndim == 1:
        waypoints = waypoints.reshape(1, 3)

    v_max    = _to3(v_max)
    a_max    = _to3(a_max)
    j_max    = _to3(j_max)
    snap_max = _to3(snap_max)

    n_wp  = len(waypoints)
    p_cur = waypoints[0].copy()
    v_cur = np.zeros(3) if v0 is None else _to3(v0)
    a_cur = np.zeros(3) if a0 is None else _to3(a0)
    j_cur = np.zeros(3) if j0 is None else _to3(j0)

    t_segs, pos_segs, vel_segs, acc_segs, jerk_segs = [], [], [], [], []
    t_offset = 0.0

    for k in range(1, n_wp):
        p_next = waypoints[k]
        if np.linalg.norm(p_next - p_cur) < 1e-9:
            continue

        T_opt, coeffs = _find_min_time(
            p_cur, p_next, v_cur, a_cur, j_cur,
            v_max, a_max, j_max, snap_max,
        )

        n_pts = max(2, int(np.round(T_opt / dt)) + 1)
        t_seg = np.linspace(0.0, T_opt, n_pts)

        p_arr = np.stack([_eval_poly(c,                   t_seg) for c in coeffs])
        v_arr = np.stack([_eval_poly(_deriv_coeffs(c, 1), t_seg) for c in coeffs])
        a_arr = np.stack([_eval_poly(_deriv_coeffs(c, 2), t_seg) for c in coeffs])
        j_arr = np.stack([_eval_poly(_deriv_coeffs(c, 3), t_seg) for c in coeffs])

        sl = slice(None) if k == 1 else slice(1, None)
        t_segs.append(t_seg[sl] + t_offset)
        pos_segs.append(p_arr[:, sl])
        vel_segs.append(v_arr[:, sl])
        acc_segs.append(a_arr[:, sl])
        jerk_segs.append(j_arr[:, sl])

        t_offset += T_opt
        p_cur = p_next.copy()
        v_cur = np.zeros(3)
        a_cur = np.zeros(3)
        j_cur = np.zeros(3)

    t_out    = np.concatenate(t_segs)
    pos_out  = np.concatenate(pos_segs,  axis=1)
    vel_out  = np.concatenate(vel_segs,  axis=1)
    acc_out  = np.concatenate(acc_segs,  axis=1)
    jerk_out = np.concatenate(jerk_segs, axis=1)
    T_total  = t_offset

    print(f"[plan_waypoints] {n_wp-1}개 세그먼트  총 소요시간: {T_total:.3f}s")
    return t_out, pos_out, vel_out, acc_out, jerk_out, T_total
