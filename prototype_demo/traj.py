"""웨이포인트 → 시간 매개화된 참조 궤적.

성진 제어기의 **입력 계약**이 "스무더+게이트 통과 궤적" 이다. 날것 폴리라인을 그대로
주면 원본 Simulink 도 릴레이 한계사이클로 왕복한다 (그의 README, 구조 한계).
그래서 여기서 두 가지를 한다.

  1. **시간 매개화** — 호 길이를 따라 smoothstep 으로 s(t) 를 만든다. 시작/끝에서
     속도가 0 이라 급출발·급정지가 없다. v_max 는 planner_limits.yaml 값을 쓴다.
  2. **코너 라운딩** — 폴리라인 꼭짓점은 C0 라 가속도가 튄다. 이동평균으로 둥글린다.
     창을 넓게 잡으면 부드럽지만 웨이포인트에서 멀어지므로 개구부 통과가 깨진다.

속도/가속도도 같이 돌려주지만 **제어기에는 안 들어간다** — 그의 ``QcInput`` 에는
속도 필드가 없다 (pose 만 받는 계약). 로그·평가용이다.
"""
from __future__ import annotations

from typing import Tuple

import numpy as np


def _resample(points: np.ndarray, n: int) -> Tuple[np.ndarray, np.ndarray]:
    """폴리라인을 호 길이 균일 간격 n 점으로. 반환 (점(n,3), 총길이)."""
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    L = float(s[-1])
    if L < 1e-9:
        return np.repeat(points[:1], n, axis=0), 0.0
    q = np.linspace(0.0, L, n)
    out = np.stack([np.interp(q, s, points[:, i]) for i in range(3)], axis=1)
    return out, L


def _smooth(points: np.ndarray, win: int) -> np.ndarray:
    """가장자리 복제 패딩 이동평균.

    끝점을 원래 값으로 **되돌리면 안 된다**. 평활된 이웃과 사이에 한 스텝짜리
    점프가 생겨서 그 지점 속도가 폭발한다 (실측: 최대속도 0.96 -> 13.2 m/s).
    가장자리 복제 패딩이면 끝점은 저절로 거의 제자리에 남는다.
    """
    if win < 3:
        return points
    k = np.ones(win) / win
    pad = win // 2
    ext = np.vstack([np.repeat(points[:1], pad, axis=0), points,
                     np.repeat(points[-1:], pad, axis=0)])
    sm = np.stack([np.convolve(ext[:, i], k, mode="valid") for i in range(3)], axis=1)
    return sm[:len(points)]


def build(waypoints, dt: float = 0.001, v_max: float = 1.6,
          v_frac: float = 0.6, smooth_m: float = 0.25,
          hold_s: float = 1.0, resample_n: int = 40000,
          smooth_t: float = 0.35):
    """웨이포인트 → (t, pos(n,3), vel(n,3), 총길이, 총시간).

    v_frac   순항속도 = v_max × v_frac (smoothstep 이라 최대속도는 이보다 높다)
    smooth_m 경로 상 코너 라운딩 길이 [m]
    smooth_t **시간축** 평활 창 [s] — 가속도를 실제로 묶는 건 이쪽이다
    hold_s   마지막에 그 자리 유지 (정지 확인용)

    시간축 평활이 왜 따로 필요한가: 호 길이 경로만 둥글리면 시간축에서는 여전히
    계단이 남는다. 재샘플 간격보다 스텝당 이동거리가 작으면 ``np.interp`` 가
    구간별 선형이라 속도가 계단이 되고, 그 미분인 가속도가 임펄스로 튄다.
    실측: 시간축 평활 없이 263 m/s^2 (27 g) — 어떤 제어기도 못 따라간다.
    """
    P = np.asarray(waypoints, float)
    dense, L = _resample(P, resample_n)
    # 평활 창은 **경로 길이에 비례해 제한**한다. smooth_m 을 그대로 쓰면 짧은 구간에서
    # 창이 경로보다 커져 양 끝이 먹힌다 — 실측: 0.5 m 이동에 창 0.20 m 면 win 이
    # 전체의 40% 라 시작 0 -> 0.025, 끝 0.5 -> 0.475 로 잘렸고, 그 어긋난 기준을
    # 쫓다가 제어기가 발산했다. 10 m 경로에서는 2% 라 티가 안 나서 오래 못 찾았다.
    sm_m = min(smooth_m, 0.10 * L) if L > 1e-9 else 0.0
    if L > 1e-9 and sm_m > 0:
        win = max(3, int(resample_n * sm_m / L) | 1)
        dense = _smooth(dense, win)

    v_cruise = max(1e-6, v_max * v_frac)
    # smoothstep 의 평균속도는 최대속도의 1/1.5 라 총시간을 그에 맞춘다
    T = L / v_cruise * 1.5 if L > 1e-9 else 0.0
    n_move = max(2, int(T / dt))
    n_hold = int(hold_s / dt)
    t = np.arange(n_move + n_hold) * dt

    u = np.clip(np.arange(n_move) * dt / max(T, 1e-9), 0, 1)
    s = L * (3 * u ** 2 - 2 * u ** 3)                 # smoothstep 호길이
    sd = np.linspace(0, L, len(dense))
    pos = np.stack([np.interp(s, sd, dense[:, i]) for i in range(3)], axis=1)
    pos = np.vstack([pos, np.repeat(pos[-1:], n_hold, axis=0)])
    # 시간축 평활 — 가속도를 묶는 건 여기다. 정지 유지 구간까지 포함해서 걸어야
    # 도착 직전 감속도 부드러워진다.
    # 시간축도 같은 이유로 총 시간에 비례해 제한한다.
    sm_t = min(smooth_t, 0.10 * (len(pos) * dt))
    if sm_t > 0:
        pos = _smooth(pos, max(3, int(sm_t / dt) | 1))

    vel = np.zeros_like(pos)
    vel[1:] = np.diff(pos, axis=0) / dt
    return t, pos, vel, L, T


def peak_speed(vel: np.ndarray) -> float:
    return float(np.linalg.norm(vel, axis=1).max())


# --------------------------------------------------------------------------- #
# 성진 궤적 생성기 어댑터
# --------------------------------------------------------------------------- #
def build_seoungjin(waypoints, cfg: dict, dt: float = 0.001,
                    hold_s: float = 1.0, flythrough: bool = True,
                    merge_m: float = 0.12, v0=None, a0=None):
    """성진 ``path_time.plan_waypoints_flythrough`` 로 궤적을 만든다.

    위 ``build`` 보다 모든 면에서 낫다 — 세그먼트마다 7차 다항식 최소시간을
    이진탐색으로 풀어서 v/a/j/snap **축별** 제약을 다항식 차원에서 보장하고,
    중간 웨이포인트를 **정지 없이 정확히** 통과한다 (실측 8/8 오차 0.0 mm).
    창문 통과처럼 "게이트를 정확히 지나되 서지는 말아야" 하는 임무에 맞는 형태다.

    ⚠ 배열 규약 주의: 그의 함수는 **(3, N)** 을 돌려준다 (traj_pipeline 의
    ``pos_3xN`` 규약). 전치를 빼먹으면 속도가 100 m/s 로 읽힌다 — 실제로 밟았다.

    flythrough=False 면 ``plan_waypoints`` (점마다 정지) 를 쓴다.
    """
    import path_time as pt          # control_seoungjin 이 sys.path 에 있어야 한다
    L = cfg["limits"]
    # 그의 flythrough 는 중복 waypoint 를 거부한다 (normalize_waypoints 선행 요구).
    # 그리고 점이 촘촘하면 세그먼트마다 최소시간을 풀어서 0.5 m 원호가 63 s 가 된다.
    # 여기서 병합해 둘 다 막는다.
    W = np.asarray(waypoints, float)
    keep = [W[0]]
    for q in W[1:]:
        if np.linalg.norm(q - keep[-1]) > merge_m:
            keep.append(q)
    if len(keep) < 2:
        keep.append(keep[0] + np.array([1e-3, 0.0, 0.0]))
    waypoints = np.array(keep)
    # v0/a0 = 구간 **시작 시점의 실제 속도·가속도**. 안 넘기면 그의 함수가 정지
    # 상태(v=0)를 가정하고 궤적을 만드는데, 스캔 직후처럼 기체가 움직이는 중에
    # 새 구간을 시작하면 첫 샘플부터 기준과 어긋난다. 실측: 그 어긋남을 쫓다가
    # APPROACH 진입에서 추락했다 (z 1.0 -> 0.04). 그의 API 가 원래 지원하는 인자다.
    fn = pt.plan_waypoints_flythrough if flythrough else pt.plan_waypoints
    kw = {}
    if not flythrough:
        if v0 is not None: kw["v0"] = np.asarray(v0, float)
        if a0 is not None: kw["a0"] = np.asarray(a0, float)
    t, pos, vel, acc, jerk, T = fn(np.asarray(waypoints, float),
                                   L["v_max"], L["a_max"], L["j_max"],
                                   L["snap_max"], dt=dt, **kw)
    P, V = np.asarray(pos), np.asarray(vel)
    if P.shape[0] == 3:             # (3,N) -> (N,3)
        P, V = P.T, V.T
    if hold_s > 0:
        n_hold = int(hold_s / dt)
        P = np.vstack([P, np.repeat(P[-1:], n_hold, axis=0)])
        V = np.vstack([V, np.zeros((n_hold, 3))])
        t = np.arange(len(P)) * dt
    L_path = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
    return t, P, V, L_path, float(T)


# --------------------------------------------------------------------------- #
# 우리 궤적 생성기 — 곡률 기반 속도 제한 (flythrough 대체)
# --------------------------------------------------------------------------- #
def _curvature(P: np.ndarray, ds: float) -> np.ndarray:
    """호 길이 균일 샘플 경로의 곡률 kappa = |dT/ds| [1/m]."""
    T = np.gradient(P, ds, axis=0)
    n = np.linalg.norm(T, axis=1, keepdims=True)
    T = T / np.maximum(n, 1e-12)
    dT = np.gradient(T, ds, axis=0)
    return np.linalg.norm(dT, axis=1)


def build_capped(waypoints, cfg: dict, dt: float = 0.001,
                 hold_s: float = 1.0, resample_n: int = 40000,
                 v_frac: float = 1.0, a_frac: float = 1.0,
                 lat_frac: float = 0.6, smooth_frac: float = 0.06):
    """웨이포인트 → (t, pos, vel, 길이, 시간). **곡률로 속도를 제한**한다.

    왜 새로 만들었나 — 앞의 둘이 각각 이렇게 깨졌다.

    ``build`` (내 smoothstep)
        총 시간을 v_max 로만 정하고 a_max 를 안 본다. 0.5 m 를 0.94 s 에 가면서
        가속 2.73 m/s^2 (a_max 1.6 의 1.7배) 를 냈고 자세 루프가 발산했다.

    ``build_seoungjin`` (그의 flythrough)
        통과속도 = v_max x cos(꺾임각/2)^2 인데, **원호를 점으로 쪼갤수록 점당
        꺾임각이 작아져서 감속을 안 한다.** 곡률은 그대로인데 점만 늘어난 것을
        "직선에 가깝다" 고 읽는다. 실측: 같은 원호를 4점이면 살고 6점이면 전복,
        24점이면 63 s 짜리 궤적이 나왔다. 점 개수가 물리를 바꿔서는 안 된다.

    여기서는 표준적인 방법을 쓴다.
      1. 호 길이 균일 재샘플 -> 곡률 kappa(s) 계산
      2. 속도 상한 v_lim = min(v_max, sqrt(a_lat / kappa))   <- 원심가속 제한
      3. 전진/후진 스윕으로 접선가속 a_tan 제약 + 양 끝 v=0 강제
      4. 적분해서 t(s) 를 얻고 dt 격자로 재샘플

    점을 아무리 촘촘히 넣어도 kappa 가 안 변하므로 결과가 같다. 그리고 양 끝
    v=0 이 보장돼서 구간을 이어 붙일 때 시작 속도 불일치가 생기지 않는다.
    """
    L_lim = cfg["limits"]
    v_max = float(L_lim["v_max"]) * v_frac
    a_tan = float(L_lim["a_max"]) * a_frac
    a_lat = float(L_lim["a_max"]) * lat_frac

    P, L = _resample(np.asarray(waypoints, float), resample_n)
    if L < 1e-9:
        n_hold = max(2, int(hold_s / dt))
        pos = np.repeat(P[:1], n_hold, axis=0)
        return np.arange(n_hold) * dt, pos, np.zeros_like(pos), 0.0, 0.0
    if smooth_frac > 0:
        P = _smooth(P, max(3, int(resample_n * smooth_frac) | 1))
    ds = L / (resample_n - 1)

    kap = _curvature(P, ds)
    v_lim = np.minimum(v_max, np.sqrt(a_lat / np.maximum(kap, 1e-6)))
    v_lim = np.minimum(v_lim, v_max)

    v = v_lim.copy()
    v[0] = v[-1] = 0.0
    for i in range(1, len(v)):                       # 전진 (가속 제한)
        v[i] = min(v[i], np.sqrt(v[i-1] ** 2 + 2 * a_tan * ds))
    for i in range(len(v) - 2, -1, -1):              # 후진 (감속 제한)
        v[i] = min(v[i], np.sqrt(v[i+1] ** 2 + 2 * a_tan * ds))

    vm = np.maximum((v[:-1] + v[1:]) / 2.0, 1e-4)    # 구간 평균속도
    t_nodes = np.concatenate([[0.0], np.cumsum(ds / vm)])
    T = float(t_nodes[-1])

    n_move = max(2, int(T / dt))
    tq = np.linspace(0.0, T, n_move)
    s_nodes = np.linspace(0.0, L, len(P))
    s_of_t = np.interp(tq, t_nodes, s_nodes)
    pos = np.stack([np.interp(s_of_t, s_nodes, P[:, i]) for i in range(3)], axis=1)

    n_hold = int(hold_s / dt)
    if n_hold > 0:
        pos = np.vstack([pos, np.repeat(pos[-1:], n_hold, axis=0)])
    # 시간축 평활. s(t) 보간이 구간별 선형이라 그대로 두면 속도가 계단이 되고
    # 그 미분인 가속도가 임펄스로 튄다 (실측 10.6 m/s^2 — 계획상 한계는 지켰는데
    # 샘플링 아티팩트로 깨진다). 창은 총 시간에 비례해서 잡는다 — 고정값을 쓰면
    # 짧은 구간에서 창이 경로보다 커져 양 끝이 먹힌다.
    w_t = max(3, int(0.02 * len(pos)) | 1)
    pos = _smooth(pos, w_t)
    t = np.arange(len(pos)) * dt
    vel = np.zeros_like(pos)
    vel[1:] = np.diff(pos, axis=0) / dt
    return t, pos, vel, L, T
