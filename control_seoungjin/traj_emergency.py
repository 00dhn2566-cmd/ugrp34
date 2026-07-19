"""A-1 비상 정지 궤적 생성기 (INTERFACE_SPEC §9 유형 A-1).

실측 상태(current_state §5 — 비상은 기준 아닌 측정 사용)에서 최단 정지 후
그 자리 래치 호버. 비상 레짐 규칙(§9):
    - ZVD 생략 (군지연 1/f0 = 0.56s는 비상에 사치 — 짐 흔들림 감수)
    - 지터 마진 반납: 물리 한계 풀사용 (PHYS 2.0/2.0/j10)
    - snap은 측정만 (게이트 강제 없음 — 뱅뱅 저크 구조상 보장 불가)

수학은 traj_shaping._stop_dist(2단 정지 정확식)와 동일 계열의 시계열판 —
sqrt 근사 금지 (45cm 오버슈트 실측이 정확식의 존재 이유). traj_smoother를
동결 기준으로 돌려쓰지 않는 이유: 스무더는 기준 추종기라 정지 후 동결점으로
"복귀"한다 — §1 ④ 스냅백 금지 위반. 여기는 정지점이 곧 래치점.

이산 동역학은 스무더/게이트와 같은 후방차분 (v=dp/dt, a=dv/dt, j=da/dt) —
게이트가 보는 미분과 생성기가 지킨 한계가 같은 정의라 경계 초과가 없다.
저크 램프는 0.9x 소프트 한계 사용 (이산 경계 정확히 걸치는 것 방지).
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_pipeline import (                     # noqa: E402
    F_MODE_DEFAULT,
    PHYS_AMAX,
    PHYS_JMAX,
    PHYS_VMAX,
    _traj_hash,
)
from traj_shaping import _stop_dist, traj_gate  # noqa: E402

# 제동 정속 = 0.8·amax (저크 천이 마진 20% — traj_smoother의 ab와 동일 규약)
BRAKE_SHARE = 0.8
# xy 동시 기동 축배분 (smooth_with_axis_sharing과 동일 규약)
XY_SHARE = 0.7
XY_MOVE_EPS = 0.05          # 이 이상 |v|면 그 축은 "기동 중" [m/s]
# 이산 저크 소프트 한계: 게이트 경계(jmax)에 정확히 걸치지 않도록 여유
J_SOFT = 0.9
HOLD_S_DEFAULT = 2.0        # 정지 후 래치 관측 hold [s] (§1 ② 종점 클램프)
MAX_STOP_S = 60.0           # 수렴 안전벽 — 초과 시 error 즉사 (저장소 규칙)


def _axis_stop(v0, a0, ab, jmax, dt):
    """1축 저크 제한 최단 정지: (v0, a0) -> (0, 0), 위치 증분 시계열 반환.

    후방차분 이산 동역학 (스무더와 동일 정의):
        a_k 선택 (|a_k - a_{k-1}| <= 0.9·jmax·dt, |a_k| <= ab)
        v_k = v_{k-1} + a_k·dt,  p_k = p_{k-1} + v_k·dt
    국면: |v| 방향 제동 정속 -ab 접근 -> release 곡선 |v| <= a²/(2·jmax)
    진입 시 a 램프아웃 (v와 a가 함께 0 도달 — 2단 정확식의 시계열판).
    부호 대칭 (후진/하강 동일 처리). 반환 배열에 초기점(0.0)은 포함 안 함.
    """
    jd = J_SOFT * jmax * dt
    v, a = float(v0), float(a0)
    out = []
    n_max = int(MAX_STOP_S / dt)
    for _ in range(n_max):
        # 종료 박스: 잔여 v/a를 한 번에 끊어도 이산 저크가 소프트 한계 이내
        if abs(v) < 0.4 * jmax * dt * dt and abs(a) < 0.4 * jd:
            return np.asarray(out, float)
        s = 1.0 if v > 0.0 else (-1.0 if v < 0.0 else
                                 (1.0 if a > 0.0 else -1.0))
        w, b = s * v, s * a                     # w >= 0 프레임으로 통일
        if b < 0.0 and w <= b * b / (2.0 * jmax):
            b_new = min(b + jd, 0.0)            # release: a 램프아웃
        else:
            # 제동: -ab로 접근하되 v 부호 반전은 정확 착지(-w/dt)로 방지
            b_new = max(b - jd, -ab, -w / dt) if b >= -ab else min(b + jd, -ab)
        a = s * b_new
        v = v + a * dt
        out.append(v * dt)
    raise ValueError(
        f"_axis_stop 미수렴: v0={v0} a0={a0} (한계 {MAX_STOP_S}s)")


def build_emergency_stop(state, dt=0.01, hold_s=HOLD_S_DEFAULT):
    """실측 상태 -> 정지 궤적 res dict (save_outputs 계약 호환).

    state: current_state.json 파싱 결과 (§5). pos/vel 필수 (누락 시 즉사),
    acc는 0 폴백 (구 스키마 호환), yaw는 att.yaw_rad 동결.
    """
    for key in ("pos", "vel"):
        if key not in state:
            raise KeyError(f"current_state에 필수 키 '{key}' 없음 (비상 정지)")
    p0 = np.asarray(state["pos"], float)
    v0 = np.asarray(state["vel"], float)
    a0 = np.asarray(state.get("acc", [0.0, 0.0, 0.0]), float)
    yaw0 = float(state.get("att", {}).get("yaw_rad", 0.0))

    # xy 동시 기동이면 xy 한계 x0.7 축배분 (게이트가 xy 노름 검사 — 축별
    # 풀한계 동시 제동은 노름 2.26 > 2.0으로 재차단됨)
    xy_moving = (abs(v0[0]) > XY_MOVE_EPS) and (abs(v0[1]) > XY_MOVE_EPS)
    share = XY_SHARE if xy_moving else 1.0
    ab_xy = BRAKE_SHARE * PHYS_AMAX * share
    j_xy = PHYS_JMAX * share
    ab_z = BRAKE_SHARE * PHYS_AMAX
    j_z = PHYS_JMAX

    profiles = [
        _axis_stop(v0[0], a0[0], ab_xy, j_xy, dt),
        _axis_stop(v0[1], a0[1], ab_xy, j_xy, dt),
        _axis_stop(v0[2], a0[2], ab_z, j_z, dt),
    ]
    n_stop = max(len(pr) for pr in profiles)
    n_hold = int(np.ceil(hold_s / dt))
    n = n_stop + n_hold + 1

    pos = np.empty((n, 3))
    for ax, pr in enumerate(profiles):
        cum = p0[ax] + np.concatenate([[0.0], np.cumsum(pr)])
        pos[:len(cum), ax] = cum
        pos[len(cum):, ax] = cum[-1]            # 정지점 유지 = 래치 호버 기준
    t = np.arange(n) * dt

    # 게이트 (비상 레짐: 풀 물리 한계, snap 측정만 — smax=None)
    ok, gate_rep = traj_gate(t, pos, PHYS_VMAX, PHYS_AMAX,
                             do_error=True, jmax=PHYS_JMAX, smax=None)

    dv = np.diff(pos, axis=0) / dt
    da = np.diff(dv, axis=0) / dt
    dj = np.diff(da, axis=0) / dt
    info = {"vPk": np.max(np.abs(dv), axis=0),
            "aPk": np.max(np.abs(da), axis=0) if len(da) else np.zeros(3),
            "jPk": np.max(np.abs(dj), axis=0) if len(dj) else np.zeros(3),
            "maxDev": np.zeros(3),              # 성형 없음 — 생성 = 최종
            "xy_share_applied": share}

    speed = float(np.linalg.norm(v0))
    ds_ref = _stop_dist(speed, float(np.dot(a0, v0) / speed) if speed > 0
                        else 0.0, BRAKE_SHARE * PHYS_AMAX, PHYS_JMAX)
    return {
        "t": t, "base": pos, "smoothed": pos, "shaped": pos,
        "delta": np.zeros_like(pos),            # ZVD 생략 (비상 레짐)
        "yaw": np.full(n, yaw0), "dt": dt,
        "f_mode": F_MODE_DEFAULT, "shaper_mode": "none",
        "smoother_info": info, "gate_report": gate_rep, "gate_ok": ok,
        "trajectory_hash": _traj_hash(t, pos),
        "limits_effective": {"v_max": PHYS_VMAX, "a_max": PHYS_AMAX,
                             "j_max": PHYS_JMAX},
        "retimed": None, "limits_clamped": None,
        "yaw_meta": {"mode": "hold", "angle_rad": yaw0,
                     "note": "emergency freeze"},
        "emergency": {
            "type": "stop",
            "stop_point": pos[-1].tolist(),
            "stop_dist_m": float(np.linalg.norm(pos[-1] - p0)),
            "stop_dist_ref_m": float(ds_ref),   # 1D 정확식 참조값 (진단용)
            "stop_T_s": float(n_stop * dt),
            "hold_s": float(hold_s),
            "v0": v0.tolist(), "a0": a0.tolist(),
        },
    }
