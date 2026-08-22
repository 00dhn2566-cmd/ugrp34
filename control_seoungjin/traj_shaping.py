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
    "KeepOutViolation",
    "keep_out_clearance",
    "keep_out_check",
    "keep_out_avoid_waypoints",
]


# ---------------------------------------------------------------------------
# A-2 금지 구역 (INTERFACE_SPEC §9 — 비상 세션 추가)
#   적용 범위: 위치 제어가 살아 있는 모든 모드 (계획/스플라이스/비상 재계획
#   전부 게이트에서 전 샘플 교차 검사). B 회생만 면제 (자세 상실 중엔
#   위치 제어 부재로 준수 불가능).
# ---------------------------------------------------------------------------

class KeepOutViolation(ValueError):
    """금지 구역 침범 — reject_code 안정 계약 (추가만, 의미 변경/삭제 금지)."""
    reject_code = "KEEP_OUT_VIOLATION"


def keep_out_clearance(pos, zones, inflate_m=0.5):
    """전 샘플 x 전 구역 최소 이격 [m] (음수 = inflate 포함 구역 내부).

    zone 스키마 (§9): {"shape":"box","min":[x,y,z],"max":[x,y,z]} 또는
    {"shape":"sphere","center":[x,y,z],"radius_m":r}.
    inflate_m: 드론 반경 + 현수 짐 반경 + 정적 편각 최대 처짐 몫 (§9 —
    짐은 드론 위치보다 로프 길이만큼 밑에서 흔들린다).

    Returns
    -------
    (min_clearance_m, sample_idx, zone_idx) — 구역이 없으면 (inf, -1, -1)
    """
    pos = np.atleast_2d(np.asarray(pos, float))
    best = (np.inf, -1, -1)
    for zi, z in enumerate(zones):
        shape = z.get("shape")
        if shape == "box":
            lo = np.asarray(z["min"], float)
            hi = np.asarray(z["max"], float)
            if np.any(hi < lo):
                raise ValueError(f"keep_out box: max < min ({z})")
            d_out = np.maximum(np.maximum(lo - pos, pos - hi), 0.0)
            outside = np.linalg.norm(d_out, axis=1)
            depth = np.min(np.minimum(pos - lo, hi - pos), axis=1)
            dist = np.where(outside > 0.0, outside, -depth)
        elif shape == "sphere":
            c = np.asarray(z["center"], float)
            r = float(z["radius_m"])
            dist = np.linalg.norm(pos - c, axis=1) - r
        else:
            raise ValueError(f"keep_out zone shape 미지원: {shape!r} "
                             "(box | sphere)")
        clearance = dist - float(inflate_m)
        k = int(np.argmin(clearance))
        if clearance[k] < best[0]:
            best = (float(clearance[k]), k, zi)
    return best


def keep_out_check(pos, keep_out, do_error=True):
    """§9 게이트 검사: 침범 시 KeepOutViolation 즉사 (조용히 통과 금지).

    keep_out: {"zones": [...], "inflate_m": 0.5} (§9 keep_out_update 스키마).
    do_error=False면 raise 대신 리포트만 (비상 정지 불가피 보고 경로용).

    Returns
    -------
    rep : {"min_clearance_m", "sample_idx", "zone_idx", "violated"}
    """
    zones = (keep_out or {}).get("zones", [])
    if not zones:
        return {"min_clearance_m": None, "sample_idx": None,
                "zone_idx": None, "violated": False}
    inflate = float(keep_out.get("inflate_m", 0.5))
    c, k, zi = keep_out_clearance(pos, zones, inflate)
    rep = {"min_clearance_m": c, "sample_idx": k, "zone_idx": zi,
           "violated": bool(c < 0.0)}
    if rep["violated"] and do_error:
        raise KeepOutViolation(
            f"금지 구역 침범: 최소 이격 {c:.3f}m (구역 #{zi}, 샘플 #{k}, "
            f"inflate {inflate}m) - 회피 재계획 필요")
    return rep


def _push_out_dir(p, zone):
    """구역 중심에서 바깥으로 미는 단위 방향 (이격 증가 방향)."""
    if zone["shape"] == "sphere":
        d = p - np.asarray(zone["center"], float)
        n = np.linalg.norm(d)
        return d / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
    lo = np.asarray(zone["min"], float)
    hi = np.asarray(zone["max"], float)
    inside = np.all((p >= lo) & (p <= hi))
    if inside:
        # 내부: 박스 중심에서 방사 방향 (sphere와 동일 규약).
        # 최근접 면 축 방식은 경로가 그 축과 평행할 때 점이 경로를 따라
        # 미끄러지기만 하는 퇴화(미수렴 실측 60회)가 있어 기각.
        d = p - 0.5 * (lo + hi)
        n = np.linalg.norm(d)
        return d / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
    d = np.maximum(np.maximum(lo - p, p - hi), 0.0)
    sign = np.where(p > hi, 1.0, np.where(p < lo, -1.0, 0.0))
    d = d * sign
    n = np.linalg.norm(d)
    return d / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])


def _densify_polyline(wp, step):
    """세그먼트별 등간격 재샘플 (양끝 포함) — 현(chord) 침범 검사의 전제."""
    acc = [np.asarray(wp[0], float)]
    for a, b in zip(wp[:-1], wp[1:]):
        n = max(int(np.ceil(np.linalg.norm(b - a) / step)), 1)
        for k in range(1, n + 1):
            acc.append(a + (b - a) * k / n)
    return np.asarray(acc)


def _rdp_polyline(pts, eps):
    """RDP 폴리라인 단순화 (traj_pipeline._rdp와 동일 알고리즘 — 순환
    임포트 회피용 사본. 원본 수정 시 함께 갱신)."""
    pts = np.asarray(pts, float)
    n = len(pts)
    keep = np.zeros(n, bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        L2 = float(np.dot(ab, ab))
        seg = pts[i + 1:j]
        if L2 < 1e-24:
            d = np.linalg.norm(seg - a, axis=1)
        else:
            t = np.clip((seg - a) @ ab / L2, 0.0, 1.0)
            d = np.linalg.norm(seg - a - t[:, None] * ab, axis=1)
        k = int(np.argmax(d))
        if d[k] > eps:
            m = i + 1 + k
            keep[m] = True
            stack.append((i, m))
            stack.append((m, j))
    return pts[keep]


def keep_out_avoid_waypoints(waypoints, keep_out, extra_margin_m=0.1,
                             sample_step_m=0.05, max_iter=60):
    """§9 A-2 회피 재계획: 구역을 지나는 waypoint 폴리라인을 우회로로 수정.

    방법 (v1 — 재조밀화 push-out 반복):
      매 반복: ① 폴리라인을 sample_step_m 간격 재샘플 (밀린 점 사이의 현이
      구역을 관통하는지 새 중간점으로 검사 — 구역 중심 정관통 경로에서 점만
      좌우로 갈라지고 현이 남는 퇴화 대응) ② 이격 < extra_margin_m 샘플을
      경계 바깥 방향으로 밀기 ③ 전 샘플 이격 확보 시 종료.
      종료 후 RDP(2cm)로 간소화, 간소화가 침범을 재유발하면 조밀 경로 유지.
    첫/끝 waypoint가 이미 구역 안이면 회피 불가능 — KeepOutViolation
    (unavoidable=True) 즉사. §9: 이 경우 A-1 정지 강등 + 보고가 규정.

    Returns: (new_waypoints (M,3), moved: bool)
    """
    zones = (keep_out or {}).get("zones", [])
    wp = np.asarray(waypoints, float)
    if not zones:
        return wp, False
    inflate = float(keep_out.get("inflate_m", 0.5))
    margin = float(extra_margin_m)

    for end_pt in (wp[0], wp[-1]):
        c, _, zi = keep_out_clearance(end_pt.reshape(1, 3), zones, inflate)
        if c < 0.0:
            e = KeepOutViolation(
                f"회피 불가능: 시작/종점이 이미 구역 #{zi} 안 (이격 {c:.3f}m)"
                " - A-1 정지 강등 대상 (§9)")
            e.unavoidable = True
            raise e

    pts = wp.copy()
    moved = False
    for _ in range(max_iter):
        pts = _densify_polyline(pts, sample_step_m)
        # 근접 중복 병합 (반복 재샘플로 인한 점 증식 방지)
        merged = [pts[0]]
        for p in pts[1:]:
            if np.linalg.norm(p - merged[-1]) > 0.4 * sample_step_m:
                merged.append(p)
        if np.linalg.norm(merged[-1] - pts[-1]) > 1e-12:
            merged.append(pts[-1])               # 종점 보존
        pts = np.asarray(merged)

        dirty = False
        for z in zones:
            c_all = np.array([keep_out_clearance(p.reshape(1, 3), [z],
                                                 inflate)[0] for p in pts])
            bad = np.where(c_all < margin)[0]
            bad = bad[(bad != 0) & (bad != len(pts) - 1)]   # 양끝 고정
            if len(bad) == 0:
                continue
            dirty = moved = True
            for i in bad:
                d = _push_out_dir(pts[i], z)
                # 이격을 c -> margin까지 올리는 부족분 + 2cm 여유
                pts[i] = pts[i] + d * (margin - c_all[i] + 0.02)
        if not dirty:
            break
    else:
        raise KeepOutViolation(
            f"회피 재계획 미수렴 ({max_iter}회) - 구역 배치가 경로를 막음")

    if not moved:
        return wp, False

    # 간소화 (RDP 2cm) + 가드: 간소화 후 현이 다시 침범하면 조밀 경로 유지
    out = _rdp_polyline(pts, 0.02)
    c_chk, _, _ = keep_out_clearance(
        _densify_polyline(out, sample_step_m), zones, inflate)
    if c_chk < 0.0:
        out = pts
    return out, True


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


def _v_from_dist(g, ab, jmax, smax=None):
    """남은 거리 g 안에서 저크·가속 한계로 '정확히' 0까지 감속 가능한 최대 속도.

    `_stop_dist` 의 해석적 역함수 (a=0 기준). smax 를 주면 스냅 제한까지 반영. 저크 제한 감속의 속도곡선은
    중점 대칭이라 평균속도가 정확히 v/2 이므로 d = v·T/2:
        상수감속 구간 없음 (v <= ab²/j):  T = 2·sqrt(v/j)      -> d = v^1.5 / sqrt(j)
        상수감속 구간 있음              :  T = v/ab + ab/j     -> d = v²/(2ab) + v·ab/(2j)
    경계는 v = ab²/j  <=>  d = ab³/j².

    이 값을 속도 상한으로 쓰면 (a) 정지가 포락선 안에서 매끄럽게 일어나고
    (b) 남은 거리가 짧으면 애초에 vmax 까지 안 올라간다 (사다리꼴 -> 삼각형 자동 절단).
    """
    if g <= 0.0:
        return 0.0
    if smax is None:
        # 저크만 제한: T = v/ab + ab/j
        d_b = ab**3 / jmax**2
        if g <= d_b:
            return (g * np.sqrt(jmax)) ** (2.0 / 3.0)
        c = ab / jmax
    else:
        # 스냅까지 제한: 미분 차수마다 천이 시간이 한 항씩 붙는다
        #   T = v/ab + ab/j + j/s   (표준 S-커브 계열 결과)
        # 전 구간 이 식을 쓰면 소속도 구간에서 T 를 과대평가 -> v 과소평가 = 보수적.
        c = ab / jmax + jmax / smax
    # v²/(2ab) + v·c/2 − g = 0   ->   v² + v·ab·c − 2·ab·g = 0
    b = ab * c
    return 0.5 * (-b + np.sqrt(b * b + 8.0 * ab * g))


def traj_smoother(t, pos, vmax, amax, jmax, smooth_stop=False, smax=None,
                  profile='precision'):
    """min/max 도달가능성 포락선 성형기.

    성형된 기준의 스텝 변위 d를 매 샘플 아래 구간에 클램프:
        상한: min( v·dt + a·dt² + jmax·dt³,  v·dt + amax·dt²,  +vmax·dt )
        하한: max( v·dt + a·dt² - jmax·dt³,  v·dt - amax·dt²,  -vmax·dt )
    + 거리 연동 속도 상한 (`smooth_stop=True`, 2026-08-22 신설): 속도 상한을
    `min(vmax, _v_from_dist(전방 극값까지 남은 거리))` 로 둔다. 정지가 **포락선
    안에서** 일어나므로 매끄럽고, 남은 거리가 짧으면 애초에 vmax 까지 안 올라간다
    (가속 구간과 감속 구간이 겹치면 사다리꼴 -> 삼각형 자동 절단).
    + 정지거리 트리거(백스톱): 그래도 정확 2단 정지거리가 남은 거리를 넘으면
    물리 최대 제동 모드. `smooth_stop=True` 에서는 거의 발동하지 않는다.

    무개입 보장: 입력의 후방차분 v/a/j가 전 구간 한계 이내이고 감속이
    0.8·amax 이내면 출력 == 입력 (정상 궤적 개입 < 2mm).

    Parameters
    ----------
    t    : (N,) 시간 [s]
    pos  : (N,) 또는 (N, C) 위치 [m] — 각 열 독립 성형
    vmax, amax, jmax : 물리 한계 (권장 2.0 / 2.0 / 10)
    smooth_stop : 거리 연동 속도 상한(매끄러운 정지). **기본 False = 기존 동작.**
           True 로 켜면 계단 입력의 오버슈트가 0.74 -> 0.00 cm, 정착 12.0 -> 3.8 s 로
           좋아지고 짧은 이동은 vmax 까지 안 올라간다(사다리꼴->삼각형 자동 절단).
           **[미완] 다만 목표 근처에서 저크 밴드를 깨는 경우가 남아 있다**
           (killer_step 에서 j 39 실측, 기존 회귀 5건 실패). 원인은 상한이 0 으로
           죄이는 구간과 뱅뱅 백스톱의 상호작용. 프로덕션 투입 전 재작업 필요.
    profile : 'precision'(기본) | 'agile' — **선행 감쇄(미리 깎기) 여유의 차이**.
              스냅이 유한하면 저크·가속을 즉시 못 꺾으므로 정지를 '미리' 시작해야 한다.
              그 선행량을 얼마로 잡느냐가 두 프로파일을 가른다:
                precision : 제동권한 0.60·amax, 선행 천이시간 100% -> 도달 오차 최소
                agile     : 제동권한 0.90·amax, 선행 천이시간  50% -> 빠르지만 오버슈트 큼
    smax : 스냅 한계 [m/s⁴]. **None(기본) 이면 스냅 밴드 미적용.**
           값을 주면 스냅까지 지키지만, 이 탐욕적 위치 추종 구조에서는 4차 지연이
           링잉을 만들어 목표 도달 오차가 0.8 cm -> 7~30 cm 로 악화된다 (2026-08-22 실측).
           스냅은 `traj_gate(..., smax=)` 의 사후 검사로 두는 편이 낫다.

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
    # smax=None 이면 스냅 밴드 미적용 (기본). 켜면 스냅은 지켜지지만 이 '탐욕적
    # 위치 추종' 구조에서는 4차 지연이 링잉을 만들어 목표 도달 오차가 커진다 — 실측 §아래.
    use_snap = smax is not None
    if not use_snap:
        smax = float("inf")
    # precision = 기존 채택값(0.8·amax) 그대로 — 기본 경로에서 동작 불변.
    # agile 은 제동 여유를 줄여 더 늦게·세게 선다.
    _PROFILES = {'precision': (0.80, 0.0), 'agile': (0.90, 0.0)}
    if isinstance(profile, tuple):          # 스윕용 직접 지정 (brake_share, look_share)
        brake_share, look_share = profile
    elif profile in _PROFILES:
        brake_share, look_share = _PROFILES[profile]
    else:
        raise ValueError(f"traj_smoother: 알 수 없는 profile '{profile}'")
    ab = brake_share * amax      # 제동 정속 가속
    # 선행 천이시간 — 저크를 0->ab 로, (스냅 켜면) 스냅을 0->jmax 로 세우는 데 걸리는 시간.
    # 이 시간 동안 기체는 계속 전진하므로, 목표를 그만큼 '앞당겨' 잡아야 늦게 깎이지 않는다.
    t_trans = look_share * (ab / jmax + (jmax / smax if use_snap else 0.0))
    EPS_G = 0.002        # 제동 트리거 데드밴드 [m] — 종점 수렴부 채터 방지

    info = {k: np.zeros(C) for k in ("vPk", "aPk", "jPk", "sPk", "maxDev")}

    for ax in range(C):
        p = pos[:, ax]

        # 전방 극값 (뒤에서부터 running max/min) — 정지거리 트리거의 목표점.
        # 순간 기준으로 잡으면 정상 입력에도 동적 랙 발생.
        fwd_max = np.maximum.accumulate(p[::-1])[::-1]
        fwd_min = np.minimum.accumulate(p[::-1])[::-1]

        r, v, a, jj = p[0], 0.0, 0.0, 0.0     # jj = 현재 저크 (스냅 밴드용 상태)
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
            # 거리 연동 속도 상한 — 정지를 포락선에 흡수 (신판)
            if smooth_stop:
                g_up0 = max(fwd_max[k] - r, 0.0)
                g_dn0 = max(r - fwd_min[k], 0.0)
                sm_env = smax if use_snap else None
                # 목표를 이미 지난 뒤(g<=0)에는 상한을 걸지 않는다 — 0 으로 죄면
                # 전진이 한 샘플에 끊겨 저크가 튄다(j 27 실측). 그 구간은 기존
                # 뱅뱅 백스톱이 담당한다.
                v_up = (min(vmax, _v_from_dist(max(g_up0 - max(v, 0.0) * t_trans, 0.0),
                                               ab, jmax, sm_env))
                        if g_up0 > 1e-9 else vmax)
                v_dn = (min(vmax, _v_from_dist(max(g_dn0 - max(-v, 0.0) * t_trans, 0.0),
                                               ab, jmax, sm_env))
                        if g_dn0 > 1e-9 else vmax)
            else:
                v_up = vmax
                v_dn = vmax
            h_up = max(v_up - v, 0.0)
            h_dn = max(v_dn + v, 0.0)
            a_cap_up = max(jmax * (np.sqrt(2.25 * dt**2 + 2.0 * h_up / jmax)
                                   - 1.5 * dt), 0.0)
            a_cap_dn = max(jmax * (np.sqrt(2.25 * dt**2 + 2.0 * h_dn / jmax)
                                   - 1.5 * dt), 0.0)
            # 저크 밴드를 먼저 확정하고, 속도/가속 상한은 그 안으로만 조인다.
            # (거리 연동 상한은 목표를 지나면 v_up=0 이 되는데, 이를 경성 클램프로
            #  쓰면 전진이 한 샘플에 끊겨 저크가 튄다 — j 42 실측. 밴드 클립으로 봉인)
            j_up = v * dt + a * dt**2 + jmax * dt**3
            j_lo = v * dt + a * dt**2 - jmax * dt**3
            # 속도 상한 항은 저크 한 스텝이 낼 수 있는 범위 안으로 먼저 클립한다.
            # (안 하면 목표 근처에서 v_up/v_dn 이 0 으로 죄면서 진행을 한 샘플에
            #  끊어 저크가 튄다 — j 39 실측. 가속 항은 건드리지 않으므로
            #  '감속 ≤ amax' 보장은 그대로 유지된다.)
            vc_up = min(max(v_up * dt, j_lo), j_up)
            vc_dn = min(max(-v_dn * dt, j_lo), j_up)
            up_raw = min(j_up, v * dt + min(amax, a_cap_up) * dt**2, vc_up)
            lo_raw = max(j_lo, v * dt - min(amax, a_cap_dn) * dt**2, vc_dn)
            # 스냅 밴드 = 가장 안쪽 실현가능 밴드. 한 스텝에 저크가 ±smax·dt 만큼만
            # 변할 수 있으므로, 다음 저크는 [jj−smax·dt, jj+smax·dt] ∩ [−jmax, +jmax].
            # 저크 여유 테이퍼 — a_cap 과 같은 구조를 한 미분 위로 올린 것.
            # 스냅이 유한하면 저크를 즉시 못 꺾으므로, a 가 amax 를 넘지 않으려면
            # 저크를 미리 조여야 한다 (안 하면 스냅 밴드가 가속 한계를 삼킨다).
            if not use_snap:
                j_hi_s = jmax
                j_lo_s = -jmax
            else:
                h_a_up = max(amax - a, 0.0)
                h_a_dn = max(amax + a, 0.0)
                j_cap_up = max(smax * (np.sqrt(2.25 * dt**2 + 2.0 * h_a_up / smax)
                                       - 1.5 * dt), 0.0)
                j_cap_dn = max(smax * (np.sqrt(2.25 * dt**2 + 2.0 * h_a_dn / smax)
                                       - 1.5 * dt), 0.0)
                j_hi_s = min(jmax, jj + smax * dt, j_cap_up)
                j_lo_s = max(-jmax, jj - smax * dt, -j_cap_dn)
            s_hi = v * dt + a * dt**2 + j_hi_s * dt**3
            s_lo = v * dt + a * dt**2 + j_lo_s * dt**3
            if s_hi < s_lo:
                s_hi = s_lo
            # 나머지 한계(저크/가속/속도)는 전부 스냅 밴드 안으로 클립
            # ⚠ lo3 를 위에서 자르면 안 된다 — 자르면 '감속 ≤ amax' 보장이 깨진다
            #   (초판이 s_hi 로 클립해서 a 2.20 실측, 게이트 거부 7건).
            #   하한은 실현가능 바닥(s_lo)만 보장하고, 상한은 하한 아래로 못 내려간다.
            a_lo = v * dt - amax * dt**2
            up3 = min(up_raw, s_hi)
            lo3 = max(lo_raw, a_lo, s_lo)
            if up3 < lo3:      # 실현가능성(저크/스냅) 우선 — 원판과 같은 우선순위
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
            a_new = (v_new - v) / dt
            jj = (a_new - a) / dt
            a = a_new
            v = v_new
            out[k] = r
        pos_s[:, ax] = out

        dv = np.diff(out) / np.diff(t)
        da = np.diff(dv) / np.diff(t[:-1])
        dj = np.diff(da) / np.diff(t[:-2])
        info["vPk"][ax] = np.max(np.abs(dv))
        info["aPk"][ax] = np.max(np.abs(da)) if len(da) else 0.0
        info["jPk"][ax] = np.max(np.abs(dj)) if len(dj) else 0.0
        ds4 = np.diff(dj) / np.diff(t[:-3]) if len(dj) > 1 else np.array([0.0])
        info["sPk"][ax] = np.max(np.abs(ds4)) if len(ds4) else 0.0
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

def counter_params_from_calib(calib, tail, band_hz=(1.0, 3.0)):
    """swing_calib.json(schema 0.2) + 피드백 tail 실측 → counter_swing_offset 인자.

    변환 사슬 (교정 v2 공진 체류 근거):
      진폭: 측정 스윙 amp_deg / S[도/(m/s²)] = 카운터 가속 → /ω² = 위치 진폭
      위상: counter_swing_offset은 "오프셋 위상 = 측정 위상 그대로" 규약(내부
            에서 +π)이므로, 가속→스윙 전달 지연(phase_lag_rad)만큼 미리 빼서
            전달 — 유발 스윙이 측정 스윙과 정확히 역위상이 되게.

    가드: S<=0/결측이면 즉사(쓰레기 교정 소비 방지), f0는 짐 모드 대역 밖이면
    즉사 (대역 가드 원칙 재사용 — 대역 밖 = 비궤적성 진동).
    Returns dict(amp_pos_m, phase_rad, f_mode).
    """
    S = float(calib.get("S_deg_per_ms2") or 0.0)
    if S <= 0.0:
        raise ValueError("swing_calib: S_deg_per_ms2 결측/비양수 - 교정 재실행 필요")
    f0 = calib.get("f0_hz")
    if f0 is None or not np.isfinite(f0):
        f0 = calib.get("drive_freq_hz")
    f0 = float(f0)
    if not (band_hz[0] <= f0 <= band_hz[1]):
        raise ValueError(
            f"swing_calib: f0={f0}Hz가 짐 모드 대역 {band_hz} 밖 - 소비 거부")
    w = 2.0 * np.pi * f0
    a_counter = float(tail["amp_deg"]) / S              # [m/s^2]
    return {
        "amp_pos_m": a_counter / w**2,
        "phase_rad": float(tail["phase_rad"])
                     - float(calib.get("phase_lag_rad", 0.0)),
        "f_mode": f0,
    }


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

def traj_gate(t, pos, vmax, amax, do_error=True, jmax=10.0, smax=None):
    """전체 시계열을 수치미분해 v/a/j(+snap) 피크 검사, 초과 시 시끄럽게 raise.

    x/y는 벡터 노름(기울기 물리는 축별이 아니라 수평합), z는 별도 채널.
    저크 검사 필수(15차): v/a만 보면 온건해 보이는 저크-불가능 입력이
    스무더 급제동 뱅뱅으로 기체를 가진함 (10cm/0.67s 펄스 = 저크 20 사건).
    snap 검사(사용자 요구, 선택): smax를 주면 4계 미분까지 검사. 계획층
    (plan_waypoints)이 snap_max를 다항식으로 강제하고 ZV는 볼록결합이라
    보존하므로, 정품 경로에선 통과가 정상 (실측: 계획 10 대비 피크 6.3).

    Parameters
    ----------
    t        : (N,) [s]
    pos      : (N, 3) [m]
    vmax, amax : 한계 (envelope 여유율 적용치 권장)
    do_error : False면 raise 대신 ok=False 반환 (리포트 모드)
    jmax     : 스무더와 동일 값 사용 (기본 10)
    smax     : snap 한계 [m/s4] (None이면 snap 검사 생략 — 하위 호환)

    Returns
    -------
    ok  : bool
    rep : dict {"vxyPk","axyPk","jxyPk","vzPk","azPk","jzPk","tol"
                (+ smax 지정 시 "sxyPk","szPk")}
    """
    t = np.asarray(t, float).ravel()
    pos = np.asarray(pos, float)
    n_min = 5 if smax is not None else 4
    if len(t) < n_min:
        raise ValueError(f"traj_gate: 샘플 {n_min}개 미만 - 궤적 아님")
    measure_snap = len(t) >= 5          # 측정은 가능하면 항상, 강제는 smax 시만
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

    snap_line = ""
    if measure_snap:
        ss = np.diff(jj, axis=0) / dt1[:-3, None]       # (N-4, 3) snap
        rep["sxyPk"] = float(np.max(np.hypot(ss[:, 0], ss[:, 1])))
        rep["szPk"] = float(np.max(np.abs(ss[:, 2])))
    if smax is not None:
        ok = ok and rep["sxyPk"] <= smax * tol and rep["szPk"] <= smax * tol
        snap_line = (f"  |s_xy| {rep['sxyPk']:.1f} / 한계 {smax:.1f} m/s4\n"
                     f"  |s_z|  {rep['szPk']:.1f} / 한계 {smax:.1f} m/s4\n")

    if not ok and do_error:
        raise ValueError(
            "traj_gate: 궤적이 물리 한계 초과 - 컨트롤러 투입 거부.\n"
            f"  |v_xy| {rep['vxyPk']:.2f} / 한계 {vmax:.2f} m/s\n"
            f"  |a_xy| {rep['axyPk']:.2f} / 한계 {amax:.2f} m/s2\n"
            f"  |j_xy| {rep['jxyPk']:.1f} / 한계 {jmax:.1f} m/s3\n"
            f"  |v_z|  {rep['vzPk']:.2f} / 한계 {vmax:.2f} m/s\n"
            f"  |a_z|  {rep['azPk']:.2f} / 한계 {amax:.2f} m/s2\n"
            f"  |j_z|  {rep['jzPk']:.1f} / 한계 {jmax:.1f} m/s3\n"
            + snap_line +
            "  -> path_time 재-시간매개화 또는 traj_smoother 적용 후 재시도")
    return ok, rep
