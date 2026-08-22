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

SCHEMA_VERSION = "0.1"

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


def _lerp(a, b, w):
    return a + (b - a) * w


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
    """
    tau = max(float(latency_s), 0.0)
    if tau <= 0.0:
        return math.inf
    return 0.5 * float(track_budget_m) / tau


def build_capability(pkg_kg: float, rho: float = 0.0, latency_s: float = 0.0,
                     profile: str = "precision", yaw_err_rad: float = 0.0,
                     load: dict | None = None, now=None) -> dict:
    """지금 줘도 되는 스펙 한 장. 반환 dict 를 그대로 JSON 으로 쓴다."""
    if profile not in _PROFILE:
        raise ValueError(f"capability: 알 수 없는 profile '{profile}'")
    prof = _PROFILE[profile]
    base = _interp_anchor(pkg_kg)

    # ① 외란 -> 시계 배율.  yaw 오차도 '예약된 권한'으로 환산해 같이 본다
    #    (돌풍이 끝나도 회복 전까지는 스펙을 되돌리지 않기 위해 — SPEED_GOVERNOR §5.2)
    rho_eff = max(float(rho), abs(float(yaw_err_rad)) / math.radians(45.0))
    s = scale_from_rho(rho_eff)

    # ② 한계 = 기저 x 프로파일 x 시계 배율 (v∝s, a∝s², j∝s³, snap∝s⁴)
    ls = prof["limit_scale"]
    lim = base["limits"]
    limits = dict(
        v=lim["v"] * ls * s,
        a=lim["a"] * ls * s ** 2,
        j=lim["j"] * ls * s ** 3,
        snap=lim["snap"] * ls * s ** 4,
    )

    # ③ 지연 -> 속도 상한 추가 (a/j/snap 은 v 를 낮추면 자연히 여유가 생긴다)
    v_lat = v_cap_from_latency(latency_s, base["budget"]["track"])
    lat_binding = v_lat < limits["v"]
    if lat_binding:
        limits["v"] = v_lat

    reasons = []
    if rho_eff > 0.05:
        reasons.append("disturbance")
    if lat_binding:
        reasons.append("latency")
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
            **({"load": load} if load else {}),
        },
        "degraded": {
            "active": s < 1.0 or lat_binding,
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
