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
    """종점 정지(vf=af=jf=0) 특수형 — _poly7_coeffs_bc의 축약."""
    return _poly7_coeffs_bc(p0, v0, a0, j0, pf, 0.0, 0.0, 0.0, T)


def _poly7_coeffs_bc(p0, v0, a0, j0, pf, vf, af, jf, T):
    """일반 경계조건 7차 다항식: 시작·종점의 p/v/a/j 8개 조건 전부 지정.

    종점 속도 vf≠0이면 무정지 통과(fly-through) 세그먼트가 된다 —
    중간 waypoint에서 정지하지 않아 가용 성능을 잃지 않는 핵심 부품.
    """
    c0, c1, c2, c3 = float(p0), float(v0), float(a0) / 2.0, float(j0) / 6.0
    T2, T3, T4, T5, T6, T7 = T**2, T**3, T**4, T**5, T**6, T**7
    A = np.array([
        [T4,    T5,     T6,     T7],
        [4*T3,  5*T4,   6*T5,   7*T6],
        [12*T2, 20*T3,  30*T4,  42*T5],
        [24*T,  60*T2, 120*T3, 210*T4],
    ])
    b = np.array([
        pf - c0 - c1*T - c2*T2 - c3*T3,
        vf - c1 - 2*c2*T - 3*c3*T2,
        af - 2*c2 - 6*c3*T,
        jf - 6*c3,
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


# ---------------------------------------------------------------------------
# 4. 무정지 통과(fly-through) waypoint 계획 — 통과 속도 경계조건
# ---------------------------------------------------------------------------

def _find_min_time_bc(p0, pf, v0, a0, j0, vf, af, jf,
                      v_max, a_max, j_max, snap_max,
                      tol=1e-3, max_iter=60):
    """일반 경계조건(종점 v/a/j 지정) 최소 세그먼트 시간 탐색.

    주의: 양끝 속도가 0이 아니면 "T가 길수록 가능"이라는 단조성이 깨진다
    (v 고정 + 긴 T = 더 먼 왕복 요구). 그래서 T_hi 배증에 상한을 두고,
    가능 창을 못 찾으면 None 반환 — 호출측이 노드 상태를 낮춰 재시도.
    """

    def make_coeffs(T):
        return [_poly7_coeffs_bc(p0[i], v0[i], a0[i], j0[i],
                                 pf[i], vf[i], af[i], jf[i], T)
                for i in range(3)]

    d = np.maximum(np.abs(pf - p0), 1e-9)
    T_lo = float(np.max([
        np.max(d / v_max),
        np.max(np.sqrt(2.0 * d / a_max)),
        np.max((6.0 * d / j_max) ** (1.0 / 3.0)),
        np.max((24.0 * d / snap_max) ** (1.0 / 4.0)),
        1e-4,
    ]))

    T_hi = None
    T_probe = T_lo
    for _ in range(20):                      # 상한: T_lo × 2^20 (폭주 차단)
        if _seg_feasible(make_coeffs(T_probe), T_probe,
                         v_max, a_max, j_max, snap_max):
            T_hi = T_probe
            break
        T_probe *= 2.0
    if T_hi is None:
        return None, None                    # 이 경계조건으로는 불능

    for _ in range(max_iter):
        if T_hi - T_lo < tol:
            break
        T_mid = 0.5 * (T_lo + T_hi)
        if _seg_feasible(make_coeffs(T_mid), T_mid, v_max, a_max, j_max, snap_max):
            T_hi = T_mid
        else:
            T_lo = T_mid

    return T_hi, make_coeffs(T_hi)


def plan_waypoints_flythrough(waypoints,
                              v_max, a_max, j_max, snap_max,
                              corner_exp=2.0, dt=0.01):
    """중간 waypoint를 **정지 없이 통과**하는 최소시간 7차 다항식 계획.

    가용 성능 문제의 근본 해법: plan_waypoints는 점마다 v=0 정지라 촘촘한
    입력에서 성능을 버림. 여기서는 중간점마다 통과 속도 벡터를 부여 —
      방향 = 앞뒤 세그먼트 단위벡터의 이등분선
      속력 = min(v_max축) × cos(꺾임각/2)^corner_exp
             (직선=풀스피드, 급코너=자동 감속, 반전(180°)=자연 정지)
    각 세그먼트는 8개 경계조건(p/v/a/j 시작·끝)을 만족하는 7차 다항식의
    최소시간을 이진탐색. v/a/j/snap 제약은 다항식 차원에서 보장되므로
    중간점 **정확 통과** + 게이트 4종 통과가 함께 성립한다.

    Returns: plan_waypoints와 동일 (t, pos, vel, acc, jerk, T_total).
    """
    waypoints = np.asarray(waypoints, float)
    v_max = _to3(v_max)
    a_max = _to3(a_max)
    j_max = _to3(j_max)
    snap_max = _to3(snap_max)
    n_wp = len(waypoints)
    if n_wp < 2:
        raise ValueError("plan_waypoints_flythrough: waypoint 2개 이상 필요")

    # 세그먼트 단위벡터
    segs = np.diff(waypoints, axis=0)
    lens = np.linalg.norm(segs, axis=1)
    if np.any(lens < 1e-9):
        raise ValueError("plan_waypoints_flythrough: 중복 waypoint - "
                         "normalize_waypoints(merge)를 먼저 적용할 것")
    dirs = segs / lens[:, None]

    # 중간점 통과 상태 기록 (사용자 설계: 현재 v/a를 노드에 기록해 승계 —
    # a=0 강제는 코너에서 부자연·보수적. 구심 가속을 경계조건으로 준다)
    v_pass_mag = float(np.min(v_max))
    a_avail = float(np.min(a_max))
    v_nodes = [np.zeros(3)]
    a_nodes = [np.zeros(3)]
    for i in range(1, n_wp - 1):
        bis = dirs[i - 1] + dirs[i]
        nb = np.linalg.norm(bis)
        cos_half = 0.5 * nb                 # 단위벡터 합: |d1+d2| = 2cos(θ/2)
        if nb < 1e-9 or cos_half <= 0.0:
            v_nodes.append(np.zeros(3))     # 반전 코너 -> 정지
            a_nodes.append(np.zeros(3))
            continue
        # Menger 곡률 (3점 외접원): 코너를 "원호로 지나는" 물리 해석
        a_, b_, c_ = waypoints[i - 1], waypoints[i], waypoints[i + 1]
        cross = np.linalg.norm(np.cross(b_ - a_, c_ - a_))
        chord = np.linalg.norm(c_ - a_)
        kappa = (2.0 * cross / (lens[i - 1] * lens[i] * chord)
                 if chord > 1e-9 else 0.0)
        speed = v_pass_mag * cos_half ** corner_exp
        if kappa > 1e-9:
            # 원호 법칙 — 계수 0.5: 구심 가속은 노드 기록분 외에 세그먼트
            # 내부(접선 가감속 + 저크/스냅 천이)에도 여유가 필요
            speed = min(speed, np.sqrt(0.5 * a_avail / kappa))
        L_adj = min(lens[i - 1], lens[i])
        speed = min(speed, np.sqrt(2.0 * a_avail * L_adj))
        v_nodes.append((bis / nb) * speed)
        # 구심 가속 기록: a = v²·κ · (중심 방향 = 접선 변화 방향)
        n_vec = dirs[i] - dirs[i - 1]
        nn = np.linalg.norm(n_vec)
        if kappa > 1e-9 and nn > 1e-9:
            a_nodes.append((n_vec / nn) * (speed ** 2 * kappa))
        else:
            a_nodes.append(np.zeros(3))
    v_nodes.append(np.zeros(3))
    a_nodes.append(np.zeros(3))

    # 사전 가능성 패스: 불능 세그먼트의 양끝 노드 상태를 0.6배씩 낮춰 수렴
    # (최후엔 0 = 그 점만 정지 통과 — 전체가 죽는 것보다 국소 감속이 낫다)
    zero = np.zeros(3)
    for _pass in range(8):
        all_ok = True
        for k in range(n_wp - 1):
            T_try, _ = _find_min_time_bc(
                waypoints[k], waypoints[k + 1],
                v_nodes[k], a_nodes[k], zero,
                v_nodes[k + 1], a_nodes[k + 1], zero,
                v_max, a_max, j_max, snap_max, max_iter=0)
            if T_try is None:
                all_ok = False
                for idx in (k, k + 1):
                    v_nodes[idx] = v_nodes[idx] * 0.6
                    a_nodes[idx] = a_nodes[idx] * 0.6
                    if np.linalg.norm(v_nodes[idx]) < 0.05:
                        v_nodes[idx] = np.zeros(3)
                        a_nodes[idx] = np.zeros(3)
        if all_ok:
            break
    else:
        raise ValueError("plan_waypoints_flythrough: 노드 상태 완화 후에도 "
                         "불능 세그먼트 존재 - waypoint 간격/한계 확인")

    t_segs, pos_segs, vel_segs, acc_segs, jerk_segs = [], [], [], [], []
    t_offset = 0.0
    for k in range(n_wp - 1):
        T_opt, coeffs = _find_min_time_bc(
            waypoints[k], waypoints[k + 1],
            v_nodes[k], a_nodes[k], zero, v_nodes[k + 1], a_nodes[k + 1], zero,
            v_max, a_max, j_max, snap_max)
        n_pts = max(2, int(np.round(T_opt / dt)) + 1)
        t_seg = np.linspace(0.0, T_opt, n_pts)
        p_arr = np.stack([_eval_poly(c, t_seg) for c in coeffs])
        v_arr = np.stack([_eval_poly(_deriv_coeffs(c, 1), t_seg) for c in coeffs])
        a_arr = np.stack([_eval_poly(_deriv_coeffs(c, 2), t_seg) for c in coeffs])
        j_arr = np.stack([_eval_poly(_deriv_coeffs(c, 3), t_seg) for c in coeffs])
        sl = slice(None) if k == 0 else slice(1, None)
        t_segs.append(t_seg[sl] + t_offset)
        pos_segs.append(p_arr[:, sl])
        vel_segs.append(v_arr[:, sl])
        acc_segs.append(a_arr[:, sl])
        jerk_segs.append(j_arr[:, sl])
        t_offset += T_opt

    t_out = np.concatenate(t_segs)
    pos_out = np.concatenate(pos_segs, axis=1)
    vel_out = np.concatenate(vel_segs, axis=1)
    acc_out = np.concatenate(acc_segs, axis=1)
    jerk_out = np.concatenate(jerk_segs, axis=1)
    print(f"[plan_waypoints_flythrough] {n_wp-1}개 세그먼트 (무정지)  "
          f"총 소요시간: {t_offset:.3f}s")
    return t_out, pos_out, vel_out, acc_out, jerk_out, t_offset
