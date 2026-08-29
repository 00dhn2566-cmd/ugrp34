"""실시간 능력 표 (`capability.json`) — 상위 경로 생성기가 읽는 **기계용** 스펙.

2026-08-22 신설. 사용자 요구: "상위(경로 만드는 프로그램)가 참조할 실시간 스펙표".
사람이 읽는 정적 표는 `PERFORMANCE.md §8b` (능력 카드). 이 모듈은 그 표를 기저로 삼아
**지금 이 순간 줘도 되는 값**으로 깎아서 내보낸다.

깎는 입력 세 가지
  1. 짐 질량      pkg_kg     — 게인 스케줄이 질량에 걸려 있다 (0/1/2 kg 앵커 보간)
  2. 외란 크기    rho        — yaw/자세 채널 권한 점유율 [0,1]. 제어기 출력에서 관측
  3. 시간 지연    latency_s  — 지연 x 속도 = 위치 오차 예산 잠식.
                             **예상량(연산 부하 모델)과 실측값 둘 다**를 근거로 만든다
                             (`compute_load.LoadGovernor.update(예상, 실측)` -> applied_s).
                             부하가 줄면 확인 후 천천히 되돌린다 (비대칭 복귀).

한계 감쇄는 **가상 시계 스케일 s 하나**로 표현한다 (SPEED_GOVERNOR.md §2 와 같은 대수):
    v ∝ s,  a ∝ s²,  j ∝ s³,  snap ∝ s⁴
경로 기하를 바꾸지 않고 시간축만 늘리는 것과 동치라, 상위가 "s 배로 느리게"만 알면 된다.

사용:
    from capability import build_capability, write_capability
    cap = build_capability(pkg_kg=1.0, rho=0.31, latency_s=0.04, profile="precision")
    write_capability(cap)          # UGRP_RT_DIR/capability.json 로 원자적 쓰기
"""
from __future__ import annotations

import json
import math
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

# 0.1 -> 0.2 (2026-08-23): 필드 **추가만** (§5b 스키마 진화 규칙상 마이너).
#   observed.scale_disturbance / scale_latency / scale_latency_source
#   degraded.scale_sources / mission_allowed / latency_extrapolated   (spec_governor 가 채움)
#   observed.latency_att_s / latency_pos_applied_s                    (동)
#   reasons 에 latency_severe / att_latency_* / pos_latency_unflyable 추가
# 의미가 바뀐 것 하나: 지연이 이제 v 만 자르지 않고 **배율 전체**에 걸린다
#   (같은 지연에서 a/j/snap 이 전보다 작게 나온다 — 소비자는 값이 더 보수적이 될 뿐이라
#    깨지지 않지만, 회귀 비교표를 갖고 있다면 갱신할 것).
SCHEMA_VERSION = "0.2"

# ── 정적 기저: PERFORMANCE.md §8b 능력 카드 (질량 앵커) ───────────────────────
# limits  : 줘도 되는 궤적 한계 (게이트 상한 = 물리 x0.8)
# budget  : 상위가 여유로 잡아야 하는 값 [m] / [s]
# yaw     : yaw 계획 규칙
_ANCHORS = {
    0.0: dict(
        limits=dict(v=1.2, a=1.0, j=8.0, snap=64.0),
        budget=dict(track=0.05, overshoot=0.13, settle=2.4, keepout_inflate=0.15),
        yaw=dict(scan_rate=0.6, align_deg=15.0, step90_s=2.4),
        disturb=dict(recovery="bounded_only", pulse_dev_deg=20.0),
    ),
    1.0: dict(
        limits=dict(v=1.6, a=1.6, j=8.0, snap=64.0),
        budget=dict(track=0.04, overshoot=0.05, settle=2.2, keepout_inflate=0.10),
        yaw=dict(scan_rate=1.0, align_deg=15.0, step90_s=2.1),
        disturb=dict(recovery="full", pulse_dev_deg=2.3),
    ),
    # ⚠ 2 kg 앵커는 아직 1 kg 복사본이다 (PERFORMANCE §8b 도 "1 kg과 동일"로 적혀 있음).
    #    2 kg 전용 실측이 들어오면 교체할 것 — 그전까지 상위는 2 kg 에서 1 kg 성능을
    #    믿으면 안 된다. `basis.anchor_provisional` 로 그 사실을 같이 내보낸다.
    2.0: dict(
        limits=dict(v=1.6, a=1.6, j=8.0, snap=64.0),
        budget=dict(track=0.04, overshoot=0.05, settle=2.2, keepout_inflate=0.10),
        yaw=dict(scan_rate=1.0, align_deg=15.0, step90_s=2.1),
        disturb=dict(recovery="full", pulse_dev_deg=2.3),
    ),
}

# 프로파일: 계획기에 넘길 한계 배율 + 쉐이퍼 제동 여유.
#   스냅까지 지키려면 참조를 '처음부터' 한계 안에서 만들어야 한다 (2026-08-22 실측:
#   한계를 넘겨 놓고 쉐이퍼로 깎으면 스냅 제한이 회복을 막아 오버슈트 10~25 cm).
#   그 여유를 얼마나 두느냐가 두 프로파일의 정체다.
_PROFILE = {
    "precision": dict(limit_scale=0.75, brake_share=0.80),   # traj_shaping 채택값과 일치
    "agile":     dict(limit_scale=1.00, brake_share=0.90),
}

RHO_STOP = 0.90        # 이 이상이면 권한 소진 — s 를 s_min 으로
S_MIN = 0.10           # 최저 시계 배율 (0 이면 정지)

# ── 지연 -> 허용 스펙 (2026-08-23 MATLAB 실측) ────────────────────────────────
#
# 두 경로를 **다르게** 다뤄야 한다. 이게 이 절의 요점이다.
#
# 자세(IMU->제어기) 경로: 감쇄가 아니라 **게이트**다.
#   실측(diagnose/sweep_delay_margin.m, 1 kg, 제자리 호버 + 0.3 N·m 펄스):
#       지연[ms]   0      8      12     16     20     24
#       호버RMS[°] 0.021  0.004  0.004  0.211  2.437  4.614
#   20 ms 부터는 **기동을 전혀 안 해도** 자세가 2.4° 로 떨린다. 궤적을 느리게 만드는
#   것으로는 못 고친다 — 정지해 있는데도 불안정하니까. 그래서 스펙을 깎는 대신
#   임무를 거부한다. 16 ms 는 통과지만 12 ms 대비 50배 열화라 여유 구간으로 본다.
LAT_ATT_CLEAN_S = 0.012    # 이 아래는 무보정
LAT_ATT_MAX_S = 0.016      # 이 위는 운용 불가 (호버 자체가 불안정)
LAT_ATT_MARGIN_SCALE = 0.60  # 청정~한계 사이 구간에서 적용할 배율 (보수적 고정값)

# 위치(VIO->제어기) 경로: 여기는 **속도에 비례해** 오차가 커지므로 감쇄가 먹힌다.
# 실측(diagnose/sweep_delay_spec.m, 3 m 이동, 자세 5 ms 고정, 1 kg).
#
# ★ 표가 **두 벌**인 이유 (사용자 정정 2026-08-23):
#     "디폴트는 외란 없다. 만약 시간 지연에서 외란 생기면 그때 깎는 거."
#   상위에 늘 내보내는 기본 스펙은 **지연만** 반영해야 한다. 외란까지 섞어 재면
#   평시에도 돌풍을 가정한 값이 나가 임무가 필요 이상으로 느려진다.
#
#   기본 표  : 외란 없음. 판정 = 종단오차 <= 5 cm AND 추종 이탈 <= 10 cm
#   돌풍 표  : 0.3 N*m x 0.3 s 펄스를 이동 중간에 맞음 (yaw 권한의 94.6% = 최악값).
#              판정에 **외란 복귀 <= 3 s** 가 추가된다. 외란이 실제로 감지될 때만 쓴다.
#
#   왜 두 축을 곱하지 않고 표를 따로 재나 — 지연과 외란은 **상호작용**한다. 지연이
#   위상 여유를 먹은 상태에서 돌풍이 들어오면 각각의 영향을 더한 것보다 나빠진다.
#   독립 배율 두 개를 min 하는 것보다 조합을 직접 잰 표가 정확하다.
#
#   0.00 = 그 지연에서는 **어떤 배율로도** 통과 못함 (운용 불가, 임무 거부).
#   두 표 모두 `sync_delay_anchors.py` 가 MATLAB 결과에서 생성한다 — 손으로 옮기지 말 것.
#   1 kg 실측 완료 2026-08-23 (`sweep_delay_spec_progress_1kg_nominal.txt`).
#   0 kg 실측 완료 2026-08-28 (`sweep_delay_spec_progress_0kg_nominal.txt`).
#
#   ── 질량 축 ──────────────────────────────────────────────────────────────
#   두 질량은 딴판이다. 120/160 ms 에서 1 kg 은 **운용 불가**인데 0 kg 은 0.55/0.40
#   으로 산다. 이유는 질량 자체가 아니라 **그 질량의 튜닝 강도**다 — 0 kg 구성
#   (sA 0.35, kp_pos 5)은 이미 물러서 위상 여유가 남고, 1 kg 구성(kp_pos 8)은
#   뻣뻣해서 지연에 쓸 여유가 없다 (보드 08-23 / 08-28).
#
#   사이 질량은 `_lat_table_for_pkg` 가 선형 보간한다. MATLAB 게인 스케줄
#   (`qc_mass_lerp_apply`) 이 두 앵커 사이를 1차식으로 잇고 있으므로, 그 결과인
#   지연 내성도 1차로 잇는 것이 같은 가정 위에 서는 것이다.
#
#   ⚠ 0.00(운용 불가)은 보간하지 않는다. 한쪽 앵커가 0.00 이면 결과도 0.00 이다.
#     0.00 은 "작은 배율이면 된다" 가 아니라 "**어떤 배율로도 안 된다**" 는 뜻이고,
#     실제로 1 kg 120 ms 는 배율을 더 깎을수록 나빠졌다 (0.55 에서 6.3 cm 였다가
#     0.40 에서 25 m 발산). 0.55 와 0.00 을 이어 0.275 를 내주면 상위는 그 배율이
#     통과한다고 읽는데, 그건 아무도 재지 않은 값이다.
_LAT_POS_ANCHORS_0KG = {
    0.000: 1.00,
    0.020: 1.00,
    0.040: 1.00,
    0.060: 0.83,
    0.080: 0.75,
    0.120: 0.55,
    0.160: 0.40,
}

_LAT_POS_ANCHORS = {
    0.000: 1.00,
    0.020: 1.00,
    0.040: 0.88,
    0.060: 0.75,
    0.080: 0.37,
    0.120: 0.00,
    0.160: 0.00,
}

# 질량 축은 위 두 표를 **그대로 참조**한다 (사본이 아니다). 사본을 두면
# `sync_delay_anchors.py --write` 가 `_LAT_POS_ANCHORS` 만 갱신하므로, 1 kg 표를
# 다시 재도 실제 동작에는 반영되지 않는 채로 조용히 갈라진다.
_LAT_POS_ANCHORS_BY_PKG = {
    0.0: _LAT_POS_ANCHORS_0KG,
    1.0: _LAT_POS_ANCHORS,
}

_LAT_POS_ANCHORS_GUST = {
    0.000: 1.00,
    0.020: 1.00,
    0.030: 1.00,
    0.040: 0.55,
    0.060: 0.28,
    0.080: 0.00,
}

# 돌풍 표를 잰 외란 크기에 대응하는 rho. 0.3 N*m 는 yaw 권한(tau_max 0.317 N*m,
# 08-22 산출)의 94.6% 라 사실상 포화점이다. 관측 rho 를 이 값으로 정규화해
# 두 표 사이를 보간한다 — rho 0 이면 기본표, 이 값 이상이면 돌풍표.
GUST_RHO_REF = 0.90


def _lerp(a, b, w):
    return a + (b - a) * w


def _lat_table_for_pkg(pkg_kg: float) -> dict:
    """질량별 위치-지연 -> 허용 배율 표. 실측 앵커(0/1 kg) 사이는 선형, 밖은 클램프.

    ⚠ 0.00(운용 불가)은 흡수한다 — 한쪽 앵커가 0.00 이면 결과도 0.00. 이유는
    `_LAT_POS_ANCHORS_BY_PKG` 위 주석 참조 (0.00 은 "작은 배율이면 된다" 가 아니다).

    ⚠ 한쪽 질량에만 있는 지연 점은 내지 않는다. 표 하나에만 있는 값을 그대로 쓰면
    "그 질량에서도 쟀다" 로 읽힌다.

    돌풍 표에는 이 함수를 쓰지 않는다 — 0 kg 돌풍 표는 **다른 복귀 게이트**로 잰
    것이라(그 질량의 tau=0 복귀의 2배 = 약 18 s vs 1 kg 3 s) 두 표를 이으면 서로
    다른 기준을 섞는다. 게이트 표기 방식이 정해지기 전까지 돌풍은 1 kg 표만 쓴다.
    """
    tables = _LAT_POS_ANCHORS_BY_PKG
    keys = sorted(tables)
    m = min(max(float(pkg_kg), keys[0]), keys[-1])
    lo = max(k for k in keys if k <= m)
    hi = min(k for k in keys if k >= m)
    if lo == hi:
        return dict(tables[lo])
    w = (m - lo) / (hi - lo)
    out = {}
    for tau in sorted(set(tables[lo]) & set(tables[hi])):
        a, b = tables[lo][tau], tables[hi][tau]
        out[tau] = 0.0 if (a == 0.0 or b == 0.0) else _lerp(a, b, w)
    return out


def _interp_anchor(pkg_kg: float) -> dict:
    """질량 앵커 선형 보간 (0~1 kg, 1~2 kg). 범위 밖은 클램프."""
    keys = sorted(_ANCHORS)
    m = min(max(float(pkg_kg), keys[0]), keys[-1])
    lo = max(k for k in keys if k <= m)
    hi = min(k for k in keys if k >= m)
    w = 0.0 if hi == lo else (m - lo) / (hi - lo)
    out = {}
    for grp in ("limits", "budget", "yaw"):
        out[grp] = {k: _lerp(_ANCHORS[lo][grp][k], _ANCHORS[hi][grp][k], w)
                    for k in _ANCHORS[lo][grp]}
    # 문자열 항목은 보간 불가 — 보수적으로 나쁜 쪽(낮은 질량 앵커) 채택.
    # 예: 0.5 kg 은 0 kg 의 `bounded_only` 를 물려받는다 (1 kg 의 `full` 이 아님).
    out["disturb"] = dict(_ANCHORS[lo]["disturb"])
    out["disturb"]["interp_rule"] = "lower_anchor_conservative"
    return out


def combine_scales(*scales) -> float:
    """여러 감쇄 배율을 하나로 — **깎인 양을 더한다** (min 이 아니라).

    사용자 지적 (2026-08-23): "min 으로 깎는 게 아니라 + 로 해서 깎아야 하는 거 아님?"
    맞다. 배율 s 가 아니라 **깎인 양 d = 1 - s** 가 소모량이다. 외란이 여유의 30%,
    지연이 20% 를 먹으면 합쳐서 50% 가 나간다. `min` 은 "30% 만 깎으면 된다" 는 뜻이라
    **두 원인이 같은 제약을 다르게 표현한 경우에만** 맞고, 서로 다른 소비자일 때는
    낙관적이다 (그리고 지연과 외란은 실제로 상호작용해서 각각의 합보다 나쁘다).

    ※ 조합을 **직접 잰** 경우에는 이걸 쓰면 안 된다 — 이중 계산이 된다.
      지연 x 외란(위치축)은 실측표 두 벌 사이를 보간한다 (`spec_governor`).
    """
    d = 0.0
    for x in scales:
        d += max(0.0, 1.0 - min(max(float(x), 0.0), 1.0))
    return max(0.0, 1.0 - d)


def scale_from_rho(rho: float) -> float:
    """외란 권한 점유율 -> 시계 배율 s. 벗어난 양에 선형 비례, 소진하면 s_min."""
    r = min(max(float(rho), 0.0), 1.0)
    if r >= RHO_STOP:
        return S_MIN
    return max(S_MIN, 1.0 - r / RHO_STOP)


def v_cap_from_latency(latency_s: float, track_budget_m: float) -> float:
    """지연이 있으면 속도가 곧 위치 오차다: v·τ ≤ 추종 예산 의 절반.

    절반만 쓰는 이유 — 나머지 절반은 지연과 무관한 추종 오차(게인·외란) 몫.
    τ=0 이면 제한 없음(inf).

    ⚠ 이것은 **추종 오차** 예산에서 나온 상한이지, 외란 강건성 기준이 아니다.
    강건성 쪽 상한은 MATLAB 실측표(`_LAT_POS_ANCHORS`)에서 나오고, 둘 중 더
    빡빡한 쪽을 쓴다 (`spec_governor.SpecGovernor.tick`).
    """
    tau = max(float(latency_s), 0.0)
    if tau <= 0.0:
        return math.inf
    return 0.5 * float(track_budget_m) / tau


def latency_track_scale(latency_s: float, track_budget_m: float,
                        v_base: float) -> float:
    """추종 예산 상한을 **시계 배율**로 환산.

    속도만 따로 자르면 안 된다 — v 만 낮추고 a/j/snap 을 그대로 두면 상위가
    "느린데 급격한" 궤적을 만들 수 있고, 그건 경로 기하가 바뀐다는 뜻이라
    금지구역 판정과 다리(traj_bridge) 의 전제가 함께 깨진다. 감쇄는 언제나
    배율 하나로 (v∝s, a∝s², j∝s³, snap∝s⁴).
    """
    if v_base <= 0:
        return 1.0
    vc = v_cap_from_latency(latency_s, track_budget_m)
    if not math.isfinite(vc):
        return 1.0
    return min(1.0, vc / v_base)


def build_capability(pkg_kg: float, rho: float = 0.0, latency_s: float = 0.0,
                     profile: str = "precision", yaw_err_rad: float = 0.0,
                     load: dict | None = None, now=None,
                     latency_scale: float | None = None) -> dict:
    """지금 줘도 되는 스펙 한 장. 반환 dict 를 그대로 JSON 으로 쓴다.

    latency_scale : 지연 배율을 **밖에서** 지정 (spec_governor 가 실측표로 계산해 넘긴다).
        None 이면 해석적 추종 예산 규칙(`latency_track_scale`)으로 대체한다.

        왜 밖에서 받나 — 해석 규칙은 실제보다 훨씬 보수적이다. 2026-08-23 MATLAB
        실측에서 위치 지연 60 ms, v=1.6 m/s 로 3 m 이동했을 때 종단 오차가
        1.03 cm 였는데, 해석 규칙은 같은 조건에서 v 를 0.33 m/s 로 깎으라고 한다
        (약 5배 과잉 감쇄). 기준 궤적 피드포워드가 있어 측정 지연이 곧바로 추종
        오차가 되지 않기 때문이다. 그래서 **잰 구간에서는 실측이 이기고**, 표 밖
        에서만 해석 규칙을 보수적 대체값으로 쓴다.
    """
    if profile not in _PROFILE:
        raise ValueError(f"capability: 알 수 없는 profile '{profile}'")
    prof = _PROFILE[profile]
    base = _interp_anchor(pkg_kg)

    # ① 외란 -> 시계 배율.  yaw 오차도 '예약된 권한'으로 환산해 같이 본다
    #    (돌풍이 끝나도 회복 전까지는 스펙을 되돌리지 않기 위해 — SPEED_GOVERNOR §5.2)
    rho_eff = max(float(rho), abs(float(yaw_err_rad)) / math.radians(45.0))
    s_dist = scale_from_rho(rho_eff)

    # ② 지연 -> 시계 배율 (추종 예산 몫). 속도만 따로 자르지 않는 이유는
    #    `latency_track_scale` 주석 참조. 강건성 몫은 spec_governor 가 얹는다.
    if latency_scale is None:
        s_lat = latency_track_scale(latency_s, base["budget"]["track"],
                                    base["limits"]["v"] * prof["limit_scale"])
        lat_src = "analytic_track_budget"
    else:
        s_lat = float(min(max(latency_scale, 0.0), 1.0))
        lat_src = "measured_table"

    # ③ 한계 = 기저 x 프로파일 x 시계 배율 (v∝s, a∝s², j∝s³, snap∝s⁴).
    #
    #    두 배율은 **가산**으로 합친다 (사용자 지적 2026-08-23: "min 으로 깎는 게
    #    아니라 + 로 해서 깎아야 하는 거 아님?"). 배율 s 가 아니라 **깎인 양 1-s**
    #    가 소모량이라, 서로 다른 원인이면 더해야 맞다. min 은 두 원인이 같은 제약을
    #    다르게 표현한 경우에만 옳고, 그렇지 않을 때는 낙관적이다.
    #
    #    ⚠ 지연 배율에는 S_MIN 바닥을 적용하지 않는다. S_MIN 은 "움직이는 드론을
    #    얼려버리지 말라"는 외란 쪽 규약이다. 지연 쪽에서 그 아래를 요구한다면
    #    그건 운용점이 범위 밖이라는 신호라, 바닥으로 올려 **더 빠른 스펙을
    #    보고하는 것**이 곧 위험이 된다. 그대로 내보내고 사유로 표시한다.
    s = combine_scales(max(s_dist, S_MIN), s_lat)
    lat_binding = s_lat < s_dist
    ls = prof["limit_scale"]
    lim = base["limits"]
    limits = dict(
        v=lim["v"] * ls * s,
        a=lim["a"] * ls * s ** 2,
        j=lim["j"] * ls * s ** 3,
        snap=lim["snap"] * ls * s ** 4,
    )

    reasons = []
    if rho_eff > 0.05:
        reasons.append("disturbance")
    if lat_binding or s_lat < 1.0:
        reasons.append("latency")
    if s < S_MIN:
        # 최저 배율보다도 깎였다 = 사실상 정지에 가깝다. 상위는 임무 재검토할 것.
        reasons.append("latency_severe")
    if load and load.get("saturated"):
        reasons.append("load_saturated")

    return {
        "schema_version": SCHEMA_VERSION,
        "timestamp": (now or datetime.now(timezone.utc)).strftime("%Y-%m-%dT%H-%M-%S.%f")[:-3],
        "basis": {
            "pkg_kg": float(pkg_kg),
            "profile": profile,
            "mass_source": "scheduled",   # 온라인 추정기 미배선 (estimate_params.py 는 오프라인)
            # 2 kg 쪽 앵커는 1 kg 복사본(잠정). 이 플래그가 true 면 상위는 보수적으로 볼 것.
            "anchor_provisional": float(pkg_kg) > 1.0,
        },
        # 단위는 필드명으로 알 수 없으므로 한 번 못박아 둔다 (소비자 오해 방지)
        "units": {
            "limits": {"v": "m/s", "a": "m/s^2", "j": "m/s^3", "snap": "m/s^4"},
            "budget": {"track": "m", "overshoot": "m", "settle": "s",
                       "keepout_inflate": "m"},
            "yaw": {"scan_rate": "rad/s", "align_deg": "deg", "step90_s": "s"},
        },
        # ── 상위가 궤적 생성에 그대로 넣을 값 ──────────────────────────────
        "limits": {k: round(v, 6) for k, v in limits.items()},
        # ── 상위가 여유로 잡아야 할 값 [m] / [s] ───────────────────────────
        "budget": {k: round(v, 4) for k, v in base["budget"].items()},
        "yaw": {k: round(v, 4) for k, v in base["yaw"].items()},
        # ── 관측치 (상위가 판단에 참고) ────────────────────────────────────
        "observed": {
            "rho": round(float(rho), 4),
            "yaw_err_deg": round(math.degrees(float(yaw_err_rad)), 3),
            "rho_eff": round(rho_eff, 4),
            "latency_s": round(float(latency_s), 5),
            "scale_disturbance": round(s_dist, 4),
            "scale_latency": round(s_lat, 4),
            "scale_latency_source": lat_src,
            **({"load": load} if load else {}),
        },
        "degraded": {
            "active": s < 1.0,
            "time_scale": round(s, 4),
            "reasons": reasons,
            "hold_until_recovered": True,
        },
        "guarantees": base["disturb"],
        "shaper": {"brake_share": prof["brake_share"]},
        "valid_for_s": 1.0,
    }


def rt_dir() -> Path:
    """current_state.json 과 같은 실시간 경로 규칙 (INTERFACE_SPEC §0)."""
    env = os.environ.get("UGRP_RT_DIR")
    if env:
        return Path(env)
    local = os.environ.get("LOCALAPPDATA")
    if local:
        return Path(local) / "ugrp_drone"
    return Path("output")


def write_capability(cap: dict, path=None) -> Path:
    """원자적 쓰기 (tmp -> rename). 30 Hz 상태와 달리 ~5 Hz / 변화 시로 충분."""
    p = Path(path) if path else rt_dir() / "capability.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(p.parent), suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(cap, f, ensure_ascii=False, indent=2)
        os.replace(tmp, p)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return p


if __name__ == "__main__":
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    for kw in (dict(pkg_kg=1.0),
               dict(pkg_kg=1.0, rho=0.31),
               dict(pkg_kg=1.0, rho=0.31, latency_s=0.04),
               dict(pkg_kg=0.0, rho=0.0),
               dict(pkg_kg=0.0, rho=0.7, yaw_err_rad=math.radians(30)),
               dict(pkg_kg=1.0, profile="agile")):
        c = build_capability(**kw)
        print(f"{str(kw):<58} -> v {c['limits']['v']:.3f} a {c['limits']['a']:.3f} "
              f"j {c['limits']['j']:.3f} snap {c['limits']['snap']:.2f} "
              f"| s {c['degraded']['time_scale']:.3f} {c['degraded']['reasons']}")
