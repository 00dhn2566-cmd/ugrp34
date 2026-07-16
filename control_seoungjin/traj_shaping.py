"""
궤적 성형 체인의 Python 포팅 (MATLAB 정답지: controller/.../Scripts_Data/traj_*.m).

파이프라인 위치 (순서 고정 — HANDOFF_PATHTIME_PIPELINE.md):
    path_time -> traj_smoother(물리 한계) -> traj_zv(1.8Hz 모드 상쇄)
              -> traj_gate(최종 검증) -> 컨트롤러

구현 원칙 4개 (TUNING_STATUS §V/§W 실측 함정 — 위반 금지):
 1. 성형기 상태(v, a)는 반드시 "출력의 후방차분"으로 정의. 저크 적분 병렬
    전파는 한계 사이클(0.37m 개입) 유발. 드론 측정값 사용 금지(피드백 성형).
 2. 한계는 envelope 실측(v/a≈2.5)보다 깎은 값(기본 2.0/2.0/j10) 사용.
 3. 각 축 독립 성형하되, xy 동시 기동 경로는 xy 한계 ×0.7 축배분
    (대각 노름 √2 초과 방지 — smooth_with_axis_sharing 참고).
 4. 정지거리는 sqrt 근사 말고 정확 2단(저크 스윙 + 정속 제동) 공식.
"""

import numpy as np

__all__ = [
    "traj_smoother",
    "traj_zv",
    "traj_gate",
    "smooth_with_axis_sharing",
    "counter_swing_offset",
]


# ---------------------------------------------------------------------------
# traj_smoother — min/max 도달가능성 포락선 명령 성형기 (traj_smoother.m 포팅)
# ---------------------------------------------------------------------------

def _stop_dist(v, a, ab, jmax):
    """전진(v>0) 정확 2단 정지거리: 최대저크로 a를 -ab까지 스윙 후 정속 -ab 제동.

    스윙 중 v가 먼저 0이 되면 1단 도중 정지점까지만 적분.
    sqrt 근사는 저크 천이 시간을 예측 못해 45cm 오버슈트 실측 (12차 실험).
    """
    if v <= 0.0:
        return 0.0
    t1 = max((a + ab) / jmax, 0.0)
    v1 = v + a * t1 - jmax * t1**2 / 2.0
    if v1 <= 0.0:
        ts = (a + np.sqrt(a**2 + 2.0 * jmax * v)) / jmax
        return max(v * ts + a * ts**2 / 2.0 - jmax * ts**3 / 6.0, 0.0)
    d1 = v * t1 + a * t1**2 / 2.0 - jmax * t1**3 / 6.0
    return d1 + v1**2 / (2.0 * ab)


def traj_smoother(t, pos, vmax, amax, jmax):
    """min/max 도달가능성 포락선 성형기.

    성형된 기준의 스텝 변위 d를 매 샘플 아래 구간에 클램프:
        상한: min( v·dt + a·dt² + jmax·dt³,  v·dt + amax·dt²,  +vmax·dt )
        하한: max( v·dt + a·dt² - jmax·dt³,  v·dt - amax·dt²,  -vmax·dt )
    + 정지거리 트리거: 현 상태의 정확 2단 정지거리가 "미래 기준의 전방
    극값"(running max/min)까지 남은 거리를 넘으면 물리 최대 제동 모드.

    무개입 보장: 입력의 후방차분 v/a/j가 전 구간 한계 이내이고 감속이
    0.8·amax 이내면 출력 == 입력 (정상 궤적 개입 < 2mm).

    Parameters
    ----------
    t    : (N,) 시간 [s]
    pos  : (N,) 또는 (N, C) 위치 [m] — 각 열 독립 성형
    vmax, amax, jmax : 물리 한계 (권장 2.0 / 2.0 / 10)

    Returns
    -------
    pos_s : pos와 같은 shape의 성형된 기준
    info  : dict {"vPk", "aPk", "jPk", "maxDev"} 열별 (C,) 배열
    """
    t = np.asarray(t, float).ravel()
    pos = np.asarray(pos, float)
    single_col = pos.ndim == 1
    if single_col:
        pos = pos.reshape(-1, 1)
    N = len(t)
    if pos.shape[0] != N:
        raise ValueError("traj_smoother: t와 pos 길이 불일치")
    C = pos.shape[1]

    pos_s = pos.copy()
    ab = 0.8 * amax      # 제동 정속 가속 (저크 천이 마진 20%)
    EPS_G = 0.002        # 제동 트리거 데드밴드 [m] — 종점 수렴부 채터 방지

    info = {k: np.zeros(C) for k in ("vPk", "aPk", "jPk", "maxDev")}

    for ax in range(C):
        p = pos[:, ax]

        # 전방 극값 (뒤에서부터 running max/min) — 정지거리 트리거의 목표점.
        # 순간 기준으로 잡으면 정상 입력에도 동적 랙 발생.
        fwd_max = np.maximum.accumulate(p[::-1])[::-1]
        fwd_min = np.minimum.accumulate(p[::-1])[::-1]

        r, v, a = p[0], 0.0, 0.0
        out = p.copy()
        mode = 0    # 0 자유추종 / +1 전진제동 / -1 후진제동
        for k in range(1, N):
            dt = t[k] - t[k - 1]
            # 속도 여유 테이퍼 (Python 포팅에서 추가 — MATLAB 원본의 잠재 구멍):
            # vmax 접근 시 a를 미리 -jmax 램프로 감가속해야 순항 진입 순간
            # a가 한 샘플 만에 꺾이는 저크 스파이크(-70 실측)가 없다.
            # 연속식 a<=sqrt(2·jmax·h)의 이산-정확판: 후방차분 동역학에서
            # a를 -jmax로 램프다운할 때 v 추가 증가분이 a²/2j + 1.5·a·dt라
            # a_cap = jmax·(sqrt(2.25·dt² + 2h/jmax) - 1.5·dt).
            h_up = max(vmax - v, 0.0)
            h_dn = max(vmax + v, 0.0)
            a_cap_up = max(jmax * (np.sqrt(2.25 * dt**2 + 2.0 * h_up / jmax)
                                   - 1.5 * dt), 0.0)
            a_cap_dn = max(jmax * (np.sqrt(2.25 * dt**2 + 2.0 * h_dn / jmax)
                                   - 1.5 * dt), 0.0)
            up3 = min(v * dt + a * dt**2 + jmax * dt**3,
                      v * dt + min(amax, a_cap_up) * dt**2,
                      vmax * dt)
            lo3 = max(v * dt + a * dt**2 - jmax * dt**3,
                      v * dt - min(amax, a_cap_dn) * dt**2,
                      -vmax * dt)
            if up3 < lo3:      # 테이퍼가 저크 하한과 충돌하면 저크 한계 우선
                up3 = lo3
            g_up = max(fwd_max[k] - r, 0.0)
            g_dn = max(r - fwd_min[k], 0.0)
            ds_f = _stop_dist(v, a, ab, jmax)
            ds_b = _stop_dist(-v, -a, ab, jmax)
            if mode == 1 and (v <= 0.0 or ds_f <= 0.85 * g_up):
                mode = 0
            if mode == -1 and (v >= 0.0 or ds_b <= 0.85 * g_dn):
                mode = 0
            if mode == 0 and ds_f > g_up + EPS_G:
                mode = 1
            if mode == 0 and ds_b > g_dn + EPS_G:
                mode = -1
            if mode == 1:
                d = lo3                                   # 물리 최대 전진제동
            elif mode == -1:
                d = up3                                   # 물리 최대 후진제동
            else:
                d = min(max(p[k] - r, lo3), up3)          # 자유추종
            # 상태 = 출력의 후방차분 (원칙 1)
            r += d
            v_new = d / dt
            a = (v_new - v) / dt
            v = v_new
            out[k] = r
        pos_s[:, ax] = out

        dv = np.diff(out) / np.diff(t)
        da = np.diff(dv) / np.diff(t[:-1])
        dj = np.diff(da) / np.diff(t[:-2])
        info["vPk"][ax] = np.max(np.abs(dv))
        info["aPk"][ax] = np.max(np.abs(da)) if len(da) else 0.0
        info["jPk"][ax] = np.max(np.abs(dj)) if len(dj) else 0.0
        info["maxDev"][ax] = np.max(np.abs(out - p))

    if single_col:
        pos_s = pos_s.ravel()
    return pos_s, info


def smooth_with_axis_sharing(t, pos, vmax, amax, jmax, xy_share=0.7):
    """xyz (N,3) 궤적을 성형 — xy 동시 기동이면 xy 한계에 ×xy_share 축배분.

    원칙 3 (박스 투어 실증 §W): 축별 2.0씩 동시 감가속 → 노름 2.83으로
    게이트 재차단. xy 동시 기동 경로는 xy 한계 ×0.7, z는 전한계.
    동시 기동 여부는 입력의 후방차분 속도로 판정(같은 샘플에서 |vx|,|vy|
    둘 다 유의미하면 동시 기동). 측정값이 아니라 입력 기준의 판정이라
    피드백 성형 아님.

    jmax도 ×0.7 배분한다 — 게이트가 저크를 xy 노름으로 검사(15차 추가)하므로
    저크만 전한계로 두면 대각 동시 기동에서 노름 √2·jmax로 게이트 재차단됨.
    """
    pos = np.asarray(pos, float)
    if pos.ndim != 2 or pos.shape[1] != 3:
        raise ValueError("smooth_with_axis_sharing: pos는 (N,3)이어야 함")
    t = np.asarray(t, float).ravel()

    dt = np.diff(t)
    vx = np.abs(np.diff(pos[:, 0]) / dt)
    vy = np.abs(np.diff(pos[:, 1]) / dt)
    v_eps = 0.05 * vmax
    simultaneous = bool(np.any((vx > v_eps) & (vy > v_eps)))

    share = xy_share if simultaneous else 1.0
    xy_s, info_xy = traj_smoother(t, pos[:, :2],
                                  vmax * share, amax * share, jmax * share)
    z_s, info_z = traj_smoother(t, pos[:, 2], vmax, amax, jmax)

    pos_s = np.column_stack([xy_s, z_s])
    info = {k: np.concatenate([info_xy[k], info_z[k]]) for k in info_xy}
    info["xy_share_applied"] = share
    return pos_s, info


# ---------------------------------------------------------------------------
# traj_zv — 잔류진동 소거 input shaper (traj_zv.m 포팅, §W 실증 -65%)
# ---------------------------------------------------------------------------

def traj_zv(t, pos, f_mode, mode="zv"):
    """기준 궤적을 임펄스열과 컨볼루션해 f_mode 진동 모드 가진을 자기 상쇄.

        ZV  : [1/2, 1/2] @ 반주기      — 지연 T/2. 주파수 정확할 때 최대 소거
        ZVD : [1/4, 1/2, 1/4] @ 반주기 — 지연 T. 주파수 오차에 강건 (권장 후보)

    스무더 뒤에 두는 이유: ZV는 볼록 결합(가중평균)이라 v/a/j 한계를 보존.
    순서를 바꾸면 스무더가 임펄스 간격을 뭉개 상쇄 조건이 깨진다.
    감쇠비 0 가정 (실측 감쇠비 ~1.0이라 정당). 시작 구간은 첫 샘플 값 패딩.

    Parameters
    ----------
    t      : (N,) [s] 균일 샘플
    pos    : (N,) 또는 (N, C)
    f_mode : 진동 모드 주파수 [Hz] (현재 1.80)
    mode   : 'zv'(기본) | 'zvd'
    """
    t = np.asarray(t, float).ravel()
    pos = np.asarray(pos, float)
    single_col = pos.ndim == 1
    if single_col:
        pos = pos.reshape(-1, 1)
    N = len(t)
    if pos.shape[0] != N:
        raise ValueError("traj_zv: t와 pos 길이 불일치")
    dt = t[1] - t[0]
    if np.max(np.abs(np.diff(t) - dt)) > 1e-9:
        raise ValueError("traj_zv: 균일 샘플 필요")
    d_half = int(round(1.0 / (2.0 * f_mode) / dt))
    if d_half < 1:
        raise ValueError(
            f"traj_zv: 샘플링이 모드 반주기보다 성김 (dt={dt:g}, f={f_mode:g})")

    def delayed(P, d):
        return np.vstack([np.tile(P[0], (d, 1)), P[:-d]])

    mode = mode.lower()
    if mode == "zv":
        pos_s = 0.5 * pos + 0.5 * delayed(pos, d_half)
    elif mode == "zvd":
        pos_s = (0.25 * pos + 0.5 * delayed(pos, d_half)
                 + 0.25 * delayed(pos, 2 * d_half))
    else:
        raise ValueError(f"traj_zv: mode는 zv 또는 zvd (받은 값: {mode})")

    if single_col:
        pos_s = pos_s.ravel()
    return pos_s


# ---------------------------------------------------------------------------
# counter_swing_offset — 역위상 카운터 가속 오프셋 (지터 소거 2호기, 사용자 설계)
# ---------------------------------------------------------------------------

def counter_swing_offset(t, amp_pos_m, phase_rad, t_ref_s, f_mode,
                         jerk_budget, ramp_cycles=2.0):
    """잔류 지터를 역위상 사인 위치 오프셋으로 소거하는 델타 레이어 생성.

    attitude_feedback의 tail:{amp_deg, phase_rad, t_ref_s} 실측을 근거로,
    측정 진동과 역위상(측정 위상 + π)인 f_mode 사인파를 t_ref_s부터 얹는다.
    amp_pos_m(위치 진폭)은 교정 상수(자세° ↔ 카운터 가속 m/s² 이득,
    diagnose_swing_calib.m)로 상위에서 환산해 넘길 것.

    저크 예산 클램프: 사인 오프셋의 저크 진폭은 (2πf)³·A라서 저크가 지배
    제약 — A를 jerk_budget/(2πf)³로 자동 클램프한다 (f=1.8Hz, 예산 2.0이면
    A ≤ 1.4mm ↔ 카운터 가속 ~0.18 m/s²). 시작·끝은 ramp_cycles 주기 코사인
    램프로 부드럽게 (스위치-온 저크 킥 방지).

    Returns
    -------
    offset : (N,) 위치 오프셋 [m] — 원하는 축(피치→x, 롤→y; yaw 회전 반영은
             호출측)에 더할 것. amp가 0으로 클램프되면 전부 0.
    a_clamped : 클램프 후 실제 사용된 진폭 [m]
    """
    t = np.asarray(t, float).ravel()
    w = 2.0 * np.pi * f_mode
    a_max_by_jerk = jerk_budget / w**3
    a_used = min(abs(amp_pos_m), a_max_by_jerk)
    if a_used <= 0.0:
        return np.zeros_like(t), 0.0

    # 역위상 = 측정 위상 + π
    offset = a_used * np.sin(w * (t - t_ref_s) + phase_rad + np.pi)

    # 활성 창: t >= t_ref_s, 진입/이탈 코사인 램프
    ramp_T = ramp_cycles / f_mode
    env = np.zeros_like(t)
    active = t >= t_ref_s
    tt = t[active] - t_ref_s
    env_in = np.where(tt < ramp_T, 0.5 * (1 - np.cos(np.pi * tt / ramp_T)), 1.0)
    t_end = t[-1]
    tt_out = t_end - t[active]
    env_out = np.where(tt_out < ramp_T,
                       0.5 * (1 - np.cos(np.pi * tt_out / ramp_T)), 1.0)
    env[active] = env_in * env_out
    return offset * env, a_used


# ---------------------------------------------------------------------------
# traj_gate — 궤적 물리 한계 검증 게이트 (traj_gate.m 포팅, 컨트롤러 입구 백스톱)
# ---------------------------------------------------------------------------

def traj_gate(t, pos, vmax, amax, do_error=True, jmax=10.0):
    """전체 시계열을 수치미분해 v/a/j 피크 검사, 초과 시 시끄럽게 raise.

    x/y는 벡터 노름(기울기 물리는 축별이 아니라 수평합), z는 별도 채널.
    저크 검사 필수(15차): v/a만 보면 온건해 보이는 저크-불가능 입력이
    스무더 급제동 뱅뱅으로 기체를 가진함 (10cm/0.67s 펄스 = 저크 20 사건).

    Parameters
    ----------
    t        : (N,) [s]
    pos      : (N, 3) [m]
    vmax, amax : 한계 (envelope 여유율 적용치 권장)
    do_error : False면 raise 대신 ok=False 반환 (리포트 모드)
    jmax     : 스무더와 동일 값 사용 (기본 10)

    Returns
    -------
    ok  : bool
    rep : dict {"vxyPk","axyPk","jxyPk","vzPk","azPk","jzPk","tol"}
    """
    t = np.asarray(t, float).ravel()
    pos = np.asarray(pos, float)
    if len(t) < 4:
        raise ValueError("traj_gate: 샘플 4개 미만 - 궤적 아님 (저크 검사 불가)")
    if pos.ndim != 2 or pos.shape[1] != 3 or pos.shape[0] != len(t):
        raise ValueError("traj_gate: pos는 (N,3)이어야 하고 t와 길이 일치")
    dt1 = np.diff(t)
    if np.any(dt1 <= 0):
        raise ValueError("traj_gate: 시간축이 단조증가 아님")

    vv = np.diff(pos, axis=0) / dt1[:, None]            # (N-1, 3)
    aa = np.diff(vv, axis=0) / dt1[:-1, None]           # (N-2, 3)
    jj = np.diff(aa, axis=0) / dt1[:-2, None]           # (N-3, 3)

    rep = {
        "vxyPk": float(np.max(np.hypot(vv[:, 0], vv[:, 1]))),
        "axyPk": float(np.max(np.hypot(aa[:, 0], aa[:, 1]))),
        "jxyPk": float(np.max(np.hypot(jj[:, 0], jj[:, 1]))),
        "vzPk": float(np.max(np.abs(vv[:, 2]))),
        "azPk": float(np.max(np.abs(aa[:, 2]))),
        "jzPk": float(np.max(np.abs(jj[:, 2]))),
        "tol": 1.001,                                   # 수치미분 노이즈 허용 0.1%
    }

    tol = rep["tol"]
    ok = (rep["vxyPk"] <= vmax * tol and rep["axyPk"] <= amax * tol
          and rep["vzPk"] <= vmax * tol and rep["azPk"] <= amax * tol
          and rep["jxyPk"] <= jmax * tol and rep["jzPk"] <= jmax * tol)

    if not ok and do_error:
        raise ValueError(
            "traj_gate: 궤적이 물리 한계 초과 - 컨트롤러 투입 거부.\n"
            f"  |v_xy| {rep['vxyPk']:.2f} / 한계 {vmax:.2f} m/s\n"
            f"  |a_xy| {rep['axyPk']:.2f} / 한계 {amax:.2f} m/s2\n"
            f"  |j_xy| {rep['jxyPk']:.1f} / 한계 {jmax:.1f} m/s3\n"
            f"  |v_z|  {rep['vzPk']:.2f} / 한계 {vmax:.2f} m/s\n"
            f"  |a_z|  {rep['azPk']:.2f} / 한계 {amax:.2f} m/s2\n"
            f"  |j_z|  {rep['jzPk']:.1f} / 한계 {jmax:.1f} m/s3\n"
            "  -> path_time 재-시간매개화 또는 traj_smoother 적용 후 재시도")
    return ok, rep
