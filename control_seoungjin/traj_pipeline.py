"""
경로 JSON → 컨트롤러 궤적 파이프라인 (HANDOFF_PATHTIME_PIPELINE.md 본체).

체인 (순서 고정 — 역전 금지):
    input/<mission>.json
      → plan_waypoints   : 7차 다항식 최소시간 (v/a/j/snap 제약)
      → 균일 그리드 재샘플 (traj_zv가 균일 샘플 요구)
      → traj_smoother    : 물리 한계 포락선 (xy 동시기동 ×0.7 축배분)
      → traj_zv          : 지터(1.8Hz 짐 모드) 상쇄 오프셋 레이어
      → traj_gate        : 최종 검증 — 통과분만 컨트롤러로
      → output/trajectory.mat + trajectory.json + pipeline_meta.json

한계 예산 구조 (사용자 설계):
    물리 한계(PHYS_*: 성형·게이트 공용, envelope 2.5 실측에서 깎은 2.0/2.0/j10)
      = 계획 한계(입력 JSON limits, 시간 부여용 스펙)
      + 지터 상쇄 오프셋 예산 (JITTER_MARGIN 몫 — 상쇄 수정이 얹혀도 총합이
        물리 한계 안에 남도록 시간 부여 단계에서 미리 떼어둠)
    입력 limits가 (1-JITTER_MARGIN)·물리 한계를 넘으면 시끄럽게 error.

지터 상쇄 레이어: 최종 궤적 = 스무딩 궤적 + delta (현재는 ZV/ZVD가 1호기).
delta는 trajectory.mat에 jitter_delta로 별도 저장 — attitude_feedback 학습
루프가 이 레이어만 갱신하는 구조.

attitude_feedback.json 핸드셰이크: used:false만 소비 → mode_freq_hz로 셰이퍼
f0 갱신 → 궤적 생성 성공 후 used:true 재기록 (이중 보정 방지).

사용 (작업 API, INTERFACE_SPEC §8 — 동사 생략 = plan, 기존 호출 하위 호환):
    python traj_pipeline.py plan --input input/example_mission.json
    python traj_pipeline.py splice --input input/new_mission.json [--state <current_state.json>]
    python traj_pipeline.py check --input input/mission.json     # 부작용 없음
    python traj_pipeline.py feedback --log <sim_result_baked.mat>
    python traj_pipeline.py estimate --log <sim_result_baked.mat>
    python traj_pipeline.py status
종료 코드: 0 성공(조정 포함) / 2 거부 / 1 내부 오류.
stdout 마지막 줄 = 기계용 JSON 한 줄 ({"verdict", "report_path", "trajectory_hash"}).
"""

import argparse
import hashlib
import json
import os
import sys
from datetime import datetime

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.io import savemat

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from path_time import (                         # noqa: E402
    plan_waypoints,
    plan_waypoints_flythrough,
)
from traj_shaping import (                      # noqa: E402
    keep_out_check,
    smooth_with_axis_sharing,
    traj_gate,
    traj_smoother,
    traj_zv,
)

HERE = os.path.dirname(os.path.abspath(__file__))
INPUT_DIR = os.path.join(HERE, "input")
OUTPUT_DIR = os.path.join(HERE, "output")

# 물리 한계 (성형·게이트 공용) — envelope 실측 v/a≈2.5에서 깎은 확정 상수
PHYS_VMAX, PHYS_AMAX, PHYS_JMAX = 2.0, 2.0, 10.0
# snap 한계 (사용자 요구 — 4계 미분까지 고려). 잠정치: 실측 근거 없음
# (모터 대역폭 실측 후 조정). 기존 미션(10~60) 수용 + 유한 상한 확보 목적.
PHYS_SNAP = 80.0
# 지터 상쇄 오프셋 예산: 계획(시간 부여) 한계는 물리 한계의 (1-MARGIN)까지만
JITTER_MARGIN = 0.2
# 동적 배분 (사용자 확정: 지터 실측 기반): 승인 기준(RL 계약)은 JITTER_MARGIN
# 고정으로 정상성 유지, 실제 계획 유효 한계만 최근 잔류 지터에 따라 조임.
# 수렴(<0.5°) -> 0.2(요청 그대로) / 상승(>2°) -> 0.3 / 심각(>4°) -> 0.35.
MARGIN_DYNAMIC_STEPS = [(4.0, 0.35), (2.0, 0.30)]   # (tail RMS 초과값, 마진)
MARGIN_FRESH_S = 24 * 3600                          # 이보다 낡은 실측은 무시
LEDGER_RECENT_N = 3                                 # 최근 N건 중앙값으로 판정
# 짐 모드 기본값 (§W 실증 1.80Hz; attitude_feedback로 갱신됨)
F_MODE_DEFAULT = 1.80
# f0 갱신 수용 대역: 이 밖의 실측 주파수는 궤적으로 가진되는 짐 모드가 아니라
# 제어루프 진동일 가능성 — 쫓아가면 오히려 악화 (A/B/B' 실증: 4.39Hz 추종
# tail 12.25° vs 1.8Hz 고수 9.93°). 대역 밖이면 갱신 거부 + 경고.
F_MODE_BAND_HZ = (1.0, 3.0)
SHAPER_DEFAULT = "zvd"          # 주파수 오차 강건 (핸드오프 권장 후보)
# 컨트롤러 게인 프로파일 (튜닝 세션 계약 v1, 2026-07-17): 값의 진실은
# parameters.m ctrl_profile switch / C++ qc_apply_profile — 여기선 검증·동봉만.
CONTROLLER_PROFILES = ("precision", "balanced", "agile")
# yaw 명령 (INTERFACE_SPEC §1 yaw 절, 2026-07-19): 상위는 "어디 볼지"만,
# 회전 시간표는 여기서. yaw는 드래그 토크 차동이라 4축 중 권한 최약 + 모터가
# 호버에서 이미 토크 클램프 평형(HANDOFF_EMERGENCY §8 실측) — 잠정 한계 보수.
YAW_MODES = ("heading", "hold", "look_at", "scan")
YAW_RATE_MAX = 1.0      # [rad/s] 잠정 (토크 포화 근거 — 실측 후 조정)
YAW_ACC_MAX = 2.0       # [rad/s^2] 잠정
YAW_JERK_MAX = 10.0     # [rad/s^3] 성형용 (위치 j 한계와 동급 관용치)
LOOKAT_FREEZE_R = 0.3   # [m] look_at 특이점 동결 반경 (창문 통과 순간)
SCAN_PRIORITIES = ("move", "coupled", "scan")
SCAN_SLOW_RATIO = 0.3   # priority='scan' 1상(스캔 중)의 이동 진행률 (잠정 —
                        # 저속 스캔 = 패럴랙스/블러 감소, 비전 품질 몫)
# 원시 궤적 완화 정책 (계약 v0.2): 성형 편차가 TOL을 넘으면 거부 대신
# "경로 보존 재시간화" — 공간 경로만 추출(RDP ε)해 시간을 새로 배분.
# 경로 이탈 ~ε로 의도 보존, 소요시간 팽창률(dilation)을 벌점 신호로 회신.
RETIME_DEV_TOL = 0.30           # 이 이상 성형 편차 -> 재시간화 발동 [m]
RETIME_PATH_EPS = 0.05          # 경로 추출 RDP ε [m]

FEEDBACK_PATH = os.path.join(OUTPUT_DIR, "attitude_feedback.json")
LEDGER_PATH = os.path.join(OUTPUT_DIR, "feedback_ledger.jsonl")
FEEDBACK_STALE_S = 24 * 3600        # 신선도 경고 임계 (INTERFACE_SPEC §3)
STATE_MAX_AGE_S = 0.5               # 재계획 이어붙이기 신선도 임계 (§5)


def _rt_dir():
    """실시간 파일(current_state.json) 저장 경로 (INTERFACE_SPEC §5).

    30Hz 덮어쓰기 파일은 OneDrive 동기화 폴더(이 저장소 위치) 밖이어야
    한다 — sync 잠금으로 원자적 rename이 실패할 수 있음.
    우선순위: env UGRP_RT_DIR → %LOCALAPPDATA%/ugrp_drone → (폴백) output/.
    """
    d = os.environ.get("UGRP_RT_DIR")
    if d:
        return d
    local = os.environ.get("LOCALAPPDATA")
    if local:
        return os.path.join(local, "ugrp_drone")
    return OUTPUT_DIR


CURRENT_STATE_PATH = os.path.join(_rt_dir(), "current_state.json")

TS_FMT = "%Y-%m-%dT%H-%M-%S"


def _parse_ts(s):
    """ISO 유사(콜론→하이픈) 타임스탬프 파싱. 소수초 지원 (신선도 검사용)."""
    s = str(s)
    for fmt in (TS_FMT + ".%f", TS_FMT):
        try:
            return datetime.strptime(s, fmt)
        except ValueError:
            pass
    raise ValueError(f"타임스탬프 형식 오류: {s} (기대: {TS_FMT}[.밀리초])")


# ---------------------------------------------------------------------------
# 입출력 유틸
# ---------------------------------------------------------------------------

def _atomic_write_json(path, obj):
    """임시파일→rename 원자적 쓰기 (반쯤 써진 JSON 읽기 사고 방지)."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


# --- 코어/옵션 분리 (RL→control seam 형식 정합, 2026-08-01) -----------------
# 코어 파일은 윤호 reinforcement_yunho/interface/waypoints_config.schema.json과
# **바이트 호환**이어야 한다. 그 스키마는 additionalProperties:false라 확장 키를
# 한 개라도 섞으면 RL 측 validate()에서 거부된다 → 성진 확장은 전부 사이드카
# 파일 <mission>.options.json으로 분리한다.
MISSION_CORE_KEYS = ("waypoints", "limits", "dt")
MISSION_OPTION_KEYS = ("trajectory", "waypoint_mode", "waypoint_prep", "shaper",
                       "controller_profile", "yaw", "strict", "_comment")


def mission_options_path(path):
    """<mission>.json → <mission>.options.json (성진 확장 사이드카 경로)."""
    stem = path[:-5] if path.endswith(".json") else path
    return stem + ".options.json"


def load_mission_options(path):
    """확장 사이드카 로드. 파일 없으면 {} (옵션은 전부 선택 항목).

    코어 키(waypoints/limits/dt)가 사이드카에 있으면 즉사 — 계획 스펙이 두
    파일에 흩어지면 어느 쪽이 진실인지 알 수 없다 (저장소 규칙: 조용한 병합 금지).
    """
    if not os.path.isfile(path):
        return {}
    with open(path, encoding="utf-8") as f:
        opts = json.load(f)
    if not isinstance(opts, dict):
        raise ValueError(f"옵션 JSON은 object여야 함: {path}")
    stray = sorted(k for k in opts if k in MISSION_CORE_KEYS)
    if stray:
        raise KeyError(
            f"코어 키 {stray}는 옵션 파일에 두면 안 됨: {path} "
            "(waypoints/limits/dt는 코어 미션 JSON에만)")
    return opts


def load_mission(path):
    """input/ 경로 JSON 로드 + 스키마 검증 (누락 시 즉사 — 저장소 규칙).

    **코어** (`<mission>.json`, sample/INPUT_FORMAT.md == 윤호 waypoints_config
    스키마) — 두 입구 중 하나 필수:
        waypoints  : [[x,y,z], ...]  (N>=2) — plan_waypoints가 최소시간 부여
        limits     : {v_max, a_max, j_max, snap_max}  (필수, 숫자 또는 [x,y,z])
        dt         : 샘플 간격 [s] (선택, 기본 0.01)

    **옵션** (`<mission>.options.json`, 성진 확장 v0.2 — 전부 선택):
        waypoint_mode      : 'stop'(기본) | 'fly_through'
        waypoint_prep      : {merge_dist, collinear_tol, max_seg_len}
        shaper             : {mode: 'zv'|'zvd'|'none', f_mode_hz}
        controller_profile : 'precision'(기본) | 'balanced' | 'agile'
        yaw                : {mode, ...} (INTERFACE_SPEC §1 yaw 절)
        strict             : true면 클램프 대신 거부
        trajectory         : {"t": [...], "pos": [[x,y,z], ...]} — 이미 시간
                             붙은 원시 궤적 입구 (waypoints 대신). RL seam이
                             아니므로 코어 스키마 적용 대상 외 — 이 입구를 쓰는
                             미션은 코어 파일에 그대로 둬도 된다.

    하위 호환: 확장 키가 코어 파일에 인라인으로 있어도 그대로 동작한다
    (`_legacy_inline_options`로 표시 + 통지). 새 미션은 분리해서 쓸 것.
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(f"경로 JSON 없음: {path}")
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)

    opt_path = mission_options_path(path)
    opts = load_mission_options(opt_path)
    if opts:
        dup = sorted(set(opts) & set(cfg))
        if dup:
            raise KeyError(
                f"코어/옵션 양쪽에 중복 정의된 키 {dup}: {path} vs {opt_path} "
                "(조용한 병합 금지 — 한쪽에서 지울 것)")
        cfg.update(opts)
        cfg["_options_src"] = opt_path

    inline = sorted(k for k in cfg
                    if k in MISSION_OPTION_KEYS and k not in opts
                    and k != "trajectory")
    if inline:
        cfg["_legacy_inline_options"] = inline
        print(f"[호환] 코어 미션에 확장 키 인라인: {inline} — "
              f"{os.path.basename(opt_path)}로 분리하면 RL(윤호) 스키마 통과")

    if "limits" not in cfg:
        raise KeyError(f"경로 JSON에 필수 키 'limits' 없음: {path}")
    if ("waypoints" in cfg) == ("trajectory" in cfg):
        raise KeyError(
            f"waypoints 또는 trajectory 중 정확히 하나 필요: {path}")

    wp = None
    if "waypoints" in cfg:
        wp = np.asarray(cfg["waypoints"], float)
        if wp.ndim != 2 or wp.shape[1] != 3 or len(wp) < 2:
            raise ValueError(f"waypoints는 (N>=2, 3)이어야 함 - 현재 {wp.shape}")
    else:
        tr = cfg["trajectory"]
        for key in ("t", "pos"):
            if key not in tr:
                raise KeyError(f"trajectory에 필수 키 '{key}' 없음: {path}")
        t_in = np.asarray(tr["t"], float)
        p_in = np.asarray(tr["pos"], float)
        if p_in.ndim != 2 or p_in.shape[1] != 3 or len(t_in) != len(p_in):
            raise ValueError("trajectory.pos는 (N,3), t와 길이 일치 필요")
        if np.any(np.diff(t_in) <= 0):
            raise ValueError("trajectory.t는 단조증가여야 함")

    lim = cfg["limits"]
    for key in ("v_max", "a_max", "j_max", "snap_max"):
        if key not in lim:
            raise KeyError(f"limits에 필수 키 '{key}' 없음: {path}")

    # 컨트롤러 게인 프로파일 (튜닝 세션 계약, 2026-07-17): 미션 단위로
    # parameters.m ctrl_profile / C++ qc_apply_profile을 전환. 미지정=precision.
    # 파이프라인은 검증·동봉만 하고 값 자체는 컨트롤러 측이 소비한다.
    profile = cfg.get("controller_profile", "precision")
    if profile not in CONTROLLER_PROFILES:
        raise ValueError(
            f"controller_profile='{profile}' 미지원 - "
            f"{sorted(CONTROLLER_PROFILES)} 중 하나여야 함: {path}")
    cfg["controller_profile"] = profile

    cfg["yaw"] = normalize_yaw_cfg(cfg.get("yaw"), src=path)

    # 한계 예산: 계획 한계 <= (1-JITTER_MARGIN)·물리 한계 (snap 포함).
    # 완화 정책 (사용자 확정, 계약 v0.2): 초과분은 거부 대신 상한으로 클램프
    # 하고 통지 — 공간 의도는 살리고 시간만 양보. "strict": true면 기존 거부.
    budget = {
        "v_max": (1.0 - JITTER_MARGIN) * PHYS_VMAX,
        "a_max": (1.0 - JITTER_MARGIN) * PHYS_AMAX,
        "j_max": (1.0 - JITTER_MARGIN) * PHYS_JMAX,
        "snap_max": (1.0 - JITTER_MARGIN) * PHYS_SNAP,
    }
    clamped = {}
    for key, cap in budget.items():
        if np.max(np.asarray(lim[key], float)) > cap + 1e-9:
            if cfg.get("strict"):
                raise ValueError(
                    f"limits.{key}={lim[key]}가 지터 오프셋 예산 반영 상한 "
                    f"{cap:.2f}(물리×(1-{JITTER_MARGIN}))을 초과 (strict 모드)")
            clamped[key] = {"requested": lim[key], "applied": cap}
            lim[key] = (np.minimum(np.asarray(lim[key], float), cap).tolist()
                        if np.ndim(lim[key]) else cap)
    if clamped:
        cfg["_limits_clamped"] = clamped
        print("[완화] limits 예산 초과분 클램프: "
              + ", ".join(f"{k} {v['requested']}->{v['applied']:.2f}"
                          for k, v in clamped.items()))

    return cfg, wp


def normalize_yaw_cfg(ycfg, src=""):
    """yaw 블록 검증·정규화 (INTERFACE_SPEC §1 yaw 절). 미지정 = heading.

    scan.rate_rad_s는 필수 (기본값 없음 — 스캔 속도의 정답은 비전만 안다,
    사용자 확정 2026-07-19). 물리 상한 초과만 클램프 + _rate_clamped 기록.
    """
    if ycfg is None:
        return {"mode": "heading"}
    if not isinstance(ycfg, dict):
        raise ValueError(f"yaw 블록은 객체여야 함: {src}")
    mode = ycfg.get("mode", "heading")
    if mode not in YAW_MODES:
        raise ValueError(
            f"yaw.mode='{mode}' 미지원 - {sorted(YAW_MODES)} 중 하나: {src}")
    out = {"mode": mode}
    if mode == "hold":
        if "angle_rad" not in ycfg:
            raise KeyError(f"yaw.mode=hold에는 angle_rad 필수: {src}")
        out["angle_rad"] = float(ycfg["angle_rad"])
    elif mode == "look_at":
        tgt = np.asarray(ycfg.get("target", []), float)
        if tgt.shape != (3,):
            raise ValueError(f"yaw.mode=look_at에는 target [x,y,z] 필수: {src}")
        out["target"] = tgt.tolist()
    elif mode == "scan":
        sc = ycfg.get("scan")
        if not isinstance(sc, dict):
            raise KeyError(f"yaw.mode=scan에는 scan 블록 필수: {src}")
        for key in ("from_rad", "to_rad", "rate_rad_s"):
            if key not in sc:
                raise KeyError(
                    f"scan에 필수 키 '{key}' 없음 (rate_rad_s의 정답은 "
                    f"비전 - 기본값 없음): {src}")
        rate_req = float(sc["rate_rad_s"])
        if rate_req <= 0:
            raise ValueError(f"scan.rate_rad_s는 양수여야 함: {src}")
        sweep = sc.get("sweep", "once")
        if sweep not in ("once", "back_and_forth"):
            raise ValueError(f"scan.sweep='{sweep}' 미지원: {src}")
        prio = sc.get("priority", "coupled")
        if prio not in SCAN_PRIORITIES:
            raise ValueError(
                f"scan.priority='{prio}' 미지원 - {sorted(SCAN_PRIORITIES)}: {src}")
        rate = min(rate_req, YAW_RATE_MAX)
        out["scan"] = {"from_rad": float(sc["from_rad"]),
                       "to_rad": float(sc["to_rad"]),
                       "sweep": sweep, "rate_rad_s": rate, "priority": prio}
        if rate < rate_req:
            out["scan"]["_rate_clamped"] = {"requested": rate_req,
                                            "applied": rate}
            print(f"[완화] scan rate {rate_req} > 물리 상한 {YAW_RATE_MAX}"
                  f" -> 클램프 (adjustments 통지)")
    rmax = ycfg.get("rate_max")
    if rmax is not None:
        out["rate_max"] = min(float(rmax), YAW_RATE_MAX)
    return out


def _scan_duration_s(sc):
    """스캔 소요시간 = 구간각/rate (back_and_forth는 왕복 1회 = 2배)."""
    sweep_angle = abs(sc["to_rad"] - sc["from_rad"])
    mult = 2.0 if sc["sweep"] == "back_and_forth" else 1.0
    return mult * sweep_angle / sc["rate_rad_s"]


def current_jitter_margin():
    """최근 잔류 지터 실측 기반 동적 마진 (사용자 확정: 동적 배분).

    원장(feedback_ledger.jsonl)의 최근 LEDGER_RECENT_N건 tail 잔류의 중앙값을
    MARGIN_DYNAMIC_STEPS에 대조. 신선도(consumed_at < MARGIN_FRESH_S) 밖이거나
    데이터 없으면 기본 JITTER_MARGIN. 반환: (마진, 근거 dict).
    """
    basis = {"source": "default", "residuals": []}
    if not os.path.isfile(LEDGER_PATH):
        return JITTER_MARGIN, basis
    entries = []
    with open(LEDGER_PATH, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                entries.append(json.loads(line))
    now = datetime.now()
    fresh = []
    for e in reversed(entries):
        try:
            age = (now - _parse_ts(e["consumed_at"])).total_seconds()
        except (KeyError, ValueError):
            continue
        if age > MARGIN_FRESH_S:
            break
        r = e.get("residual", {}).get("tail_pitch_rms_deg")
        if r is not None:
            fresh.append(float(r))
        if len(fresh) >= LEDGER_RECENT_N:
            break
    if not fresh:
        return JITTER_MARGIN, basis
    med = float(np.median(fresh))
    margin = JITTER_MARGIN
    for thresh, m in MARGIN_DYNAMIC_STEPS:
        if med > thresh:
            margin = m
            break
    basis = {"source": "ledger", "residuals": fresh,
             "median_tail_deg": med}
    return margin, basis


def _ledger_flight_ids():
    """원장에서 이미 처리한 flight_id 집합 (used 태그 유실 대비 안전망)."""
    if not os.path.isfile(LEDGER_PATH):
        return set()
    ids = set()
    with open(LEDGER_PATH, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                ids.add(json.loads(line).get("flight_id"))
    return ids


def consume_attitude_feedback(f_mode):
    """used:false인 attitude_feedback.json 소비 → 갱신된 f_mode 반환.

    처리 여부 판정 2중 장치 (INTERFACE_SPEC §3/§4):
      ① used 태그 — 최신 1건의 소비 상태
      ② feedback_ledger.jsonl — flight_id 전체 이력 (태그 유실 안전망)
    신선도: written_at 나이 > FEEDBACK_STALE_S면 경고 (적용은 하되 시끄럽게).

    1차 추론 중 ② (mode_freq로 셰이퍼 f0 갱신 — 잔여 1.5°의 주범이 주파수
    오차)만 구현. ①(tail RMS → Tm 연장)은 구간 매핑 확정 후.
    used:true 재기록·원장 append는 궤적 생성 성공 후 mark_feedback_used()로.
    """
    if not os.path.isfile(FEEDBACK_PATH):
        return f_mode, None
    with open(FEEDBACK_PATH, encoding="utf-8") as f:
        fb = json.load(f)
    if fb.get("used", True):
        return f_mode, None
    if fb.get("flight_id") in _ledger_flight_ids():
        print(f"[feedback] flight_id={fb.get('flight_id')}는 원장에 이미 처리"
              " 기록 있음 - 건너뜀 (used 태그 유실 의심, 태그만 복구)")
        fb["used"] = True
        _atomic_write_json(FEEDBACK_PATH, fb)
        return f_mode, None

    age_s = None
    if "written_at" in fb:
        age_s = (datetime.now() - _parse_ts(fb["written_at"])).total_seconds()
        if age_s > FEEDBACK_STALE_S:
            print(f"[경고] 피드백이 {age_s/3600:.1f}시간 전 실측 -> 모델/게인"
                  " 변경 이후의 낡은 데이터일 수 있음. 적용은 진행.")

    new_f = float(fb.get("mode_freq_hz", f_mode))
    if not (0.2 <= new_f <= 10.0):
        raise ValueError(f"attitude_feedback mode_freq_hz={new_f} 비정상 범위")
    lo, hi = F_MODE_BAND_HZ
    if not (lo <= new_f <= hi):
        print(f"[경고] 실측 {new_f:.2f}Hz는 짐 모드 대역({lo}~{hi}Hz) 밖 ->"
              " 제어루프 진동 의심, f0 갱신 거부 (게인 영역 이슈로 보고 권장)")
        fb["_consume"] = {"age_s": age_s, "f_mode_old": f_mode,
                          "f_mode_new": f_mode, "rejected_out_of_band": new_f}
        return f_mode, fb
    fb["_consume"] = {"age_s": age_s, "f_mode_old": f_mode, "f_mode_new": new_f}
    print(f"[feedback] used:false 감지 -> 셰이퍼 f0 {f_mode:.2f} -> {new_f:.2f}Hz"
          f" (flight_id={fb.get('flight_id')}, 나이 "
          f"{'%.0fs' % age_s if age_s is not None else '미기재'})")
    return new_f, fb


def mark_feedback_used(fb):
    """소비 완료 처리: used:true 재기록 + 원장 append (INTERFACE_SPEC §4)."""
    if fb is None:
        return
    consume = fb.pop("_consume", {})
    fb["used"] = True
    _atomic_write_json(FEEDBACK_PATH, fb)
    entry = {
        "consumed_at": datetime.now().strftime(TS_FMT),
        "flight_id": fb.get("flight_id"),
        "trajectory_hash": fb.get("trajectory_hash"),
        "feedback_age_s": consume.get("age_s"),
        "action": {"f_mode_hz": [consume.get("f_mode_old"),
                                 consume.get("f_mode_new")],
                   **({"rejected_out_of_band_hz":
                       consume["rejected_out_of_band"]}
                      if "rejected_out_of_band" in consume else {})},
        "residual": {"tail_pitch_rms_deg":
                     fb.get("tail", {}).get("pitch_rms_deg")},
    }
    os.makedirs(os.path.dirname(LEDGER_PATH), exist_ok=True)
    with open(LEDGER_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    print("[feedback] used:true 재기록 + 원장 append 완료")


def load_current_state(path=None, max_age_s=STATE_MAX_AGE_S, now=None):
    """current_state.json 로드 + 신선도 검사 (INTERFACE_SPEC §5).

    timestamp 나이 > max_age_s면 error 즉사 — 낡은 상태 이어붙이기 = 새 궤적
    첫 샘플 점프 = 미분킥 자초 (§W ①).
    """
    path = path or CURRENT_STATE_PATH
    if not os.path.isfile(path):
        raise FileNotFoundError(f"current_state.json 없음: {path}")
    with open(path, encoding="utf-8") as f:
        st = json.load(f)
    for key in ("timestamp", "ref_state"):
        if key not in st:
            raise KeyError(f"current_state에 필수 키 '{key}' 없음")
    age = ((now or datetime.now()) - _parse_ts(st["timestamp"])).total_seconds()
    if age > max_age_s:
        raise ValueError(
            f"current_state가 낡음 (나이 {age:.2f}s > 임계 {max_age_s}s) -> "
            "낡은 상태 이어붙이기 거부 (점프 = 미분킥). 컨트롤러 갱신 확인")
    return st


def splice_waypoints_from_state(state, remaining_waypoints, emergency=False):
    """재계획 이어붙이기: 초기조건 + 시작점을 현재 상태에서 취한다.

    평시(emergency=False)는 ref_state(성형 기준 상태)에서 — 측정 상태로
    이어붙이면 궤적 생성에 측정 피드백이 섞여 성형기 원칙 1 위반 (§V 함정).
    비상(emergency=True)만 측정 pos/vel 사용 + 온건 스플라이스 필요(경고).

    Returns: (waypoints, v0, a0) — plan_waypoints/build_trajectory 입력.
    """
    if emergency:
        print("[경고] 비상 재계획: 측정 상태에서 이어붙임 -> 스플라이스 구간"
              " 온건(Tm>=0.9s) 유지 필요 (한계 낮춘 limits 권장)")
        base = {"pos": state["pos"], "vel": state["vel"],
                "acc": state.get("acc", [0, 0, 0])}
    else:
        base = state["ref_state"]
    wp = np.vstack([np.asarray(base["pos"], float),
                    np.asarray(remaining_waypoints, float)])
    # jerk: v0.2 스키마 (7차 경계조건이 p/v/a/j — j까지 승계해야 C³ 연속.
    # 구 스키마/비상 측정 상태에는 없으므로 0 폴백)
    j0 = np.asarray(base.get("jerk", [0.0, 0.0, 0.0]), float)
    return (wp, np.asarray(base["vel"], float),
            np.asarray(base["acc"], float), j0)


def _rdp(points, eps):
    """Ramer-Douglas-Peucker 폴리라인 단순화 (반복 스택 — 촘촘 입력 대응).

    구간 [i, j]에서 현 i-j까지 수직 거리가 최대인 점이 eps를 넘으면 그 점을
    보존하고 양쪽을 재귀 처리, 아니면 중간점 전부 병합.
    """
    pts = np.asarray(points, float)
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


def normalize_waypoints(waypoints, merge_dist=0.01, collinear_tol=None,
                        max_seg_len=None, divide_mode="linear"):
    """상위 waypoint 집합 전처리: merge(병합) / divide(분할).

    성능 목적 (사용자 확정: reference 추종 성능 좋게 시간 부여 + 촘촘한
    입력에서 상위 의도(곡률) 존중):
      - merge_dist: 직전 점과 이내 거리인 점 제거 (RL 노이즈/중복 방어).
      - collinear_tol: **RDP(Ramer-Douglas-Peucker) 단순화의 ε [m]** —
        "원래 점열에서 ε 이상 벗어나지 않는 최소 점집합"만 남긴다.
        직선 구간은 전부 병합(순항 가능), 곡선·코너는 편차가 ε에 닿는
        만큼 점이 자동 보존 = 곡률(상위 의도) 존중이 내장. (None이면 안 함)
      - max_seg_len: 초과 구간에 등간격 중간점 삽입 (None이면 안 함;
        fly_through는 build_trajectory가 기본 1.0m 자동 적용 — 긴 직선의
        스플라인 휨 방지).
    """
    wp = np.asarray(waypoints, float)
    keep = [wp[0]]
    for p in wp[1:]:
        if np.linalg.norm(p - keep[-1]) > merge_dist:
            keep.append(p)
    wp = np.array(keep)

    if collinear_tol is not None and len(wp) >= 3:
        wp = _rdp(wp, float(collinear_tol))

    if max_seg_len is not None and len(wp) >= 2:
        # divide_mode: "linear"=폴리라인 의도 그대로 / "spline"=원점들을 지나는
        # 곡선(호길이 매개 CubicSpline) 위에 삽입 — 성긴 곡선 입력에서 인위적
        # 꺾임 방지 (사용자 제안: 곡률 돌려주기). 촘촘 입력은 linear로 충분.
        use_spline = (divide_mode == "spline" and len(wp) >= 3)
        if use_spline:
            s = np.concatenate([[0.0], np.cumsum(
                np.linalg.norm(np.diff(wp, axis=0), axis=1))])
            cs = [CubicSpline(s, wp[:, i]) for i in range(3)]
        out = [wp[0]]
        for k, (a, b) in enumerate(zip(wp[:-1], wp[1:])):
            d = np.linalg.norm(b - a)
            n_div = int(np.ceil(d / max_seg_len))
            for i in range(1, n_div + 1):
                if use_spline and i < n_div:
                    si = s[k] + (s[k + 1] - s[k]) * i / n_div
                    out.append(np.array([c(si) for c in cs]))
                else:
                    out.append(a + (b - a) * i / n_div)
        wp = np.array(out)
    if len(wp) < 2:
        raise ValueError("normalize_waypoints: 병합 후 waypoint가 2개 미만 - "
                         "집합이 사실상 한 점")
    return wp


def replan_splice(res1, tau_s, new_waypoints, cfg):
    """비행 중 새 waypoint 집합 도착: τ 시점 상태에서 무정지로 꺾어 계속.

    1차 궤적(res1)의 τ 시점 기준 상태(p/v/a/j — base의 수치미분 = 성형 기준,
    측정값 아님: 원칙 1 준수)를 초기조건으로 새 집합을 재계획하고,
    base1[0:τ] + base2를 하나의 타임라인으로 결합한 뒤 스무더→ZV→게이트를
    **전체에 한 번에** 적용한다 (세그먼트별 ZV는 스플라이스 패딩 킥 유발).

    반환: build_trajectory와 동일 구조의 res dict (결합 궤적).
    """
    lim = cfg["limits"]
    t1, base1 = res1["t"], res1["base"]
    dt1 = float(t1[1] - t1[0])
    k = int(np.clip(np.round(tau_s / dt1), 1, len(t1) - 2))

    vel1 = np.gradient(base1, t1, axis=0)
    acc1 = np.gradient(vel1, t1, axis=0)
    jrk1 = np.gradient(acc1, t1, axis=0)
    p0, v0, a0, j0 = base1[k], vel1[k], acc1[k], jrk1[k]

    wp2 = np.vstack([p0, normalize_waypoints(
        np.vstack([p0, np.asarray(new_waypoints, float)]))[1:]])
    t2_raw, pos2_raw, *_ = plan_waypoints(
        wp2, lim["v_max"], lim["a_max"], lim["j_max"], lim["snap_max"],
        v0=v0, a0=a0, j0=j0, dt=dt1)

    # 2차 구간을 정확히 dt1 격자(tau 이후 연속)로 재샘플 — ZV 균일성 요건
    m = max(int(np.round(t2_raw[-1] / dt1)), 2)
    t2_grid = dt1 * np.arange(1, m + 1)
    ev = np.minimum(t2_grid, t2_raw[-1])    # 격자 끝이 살짝 넘치면 종점(정지) 고정
    base2 = np.column_stack([
        CubicSpline(t2_raw, pos2_raw[i],
                    bc_type=((1, float(v0[i])), (1, 0.0)))(ev)
        for i in range(3)])

    t_c = np.concatenate([t1[:k + 1], t1[k] + t2_grid])
    base_c = np.vstack([base1[:k + 1], base2])

    smoothed, info_sm = smooth_with_axis_sharing(
        t_c, base_c, PHYS_VMAX, PHYS_AMAX, PHYS_JMAX)
    shaper_mode = cfg.get("shaper", {}).get("mode", SHAPER_DEFAULT)
    f_mode = res1["f_mode"]
    if shaper_mode == "none":
        shaped = smoothed.copy()
    else:
        shaped = traj_zv(t_c, smoothed, f_mode, shaper_mode)
    ok, gate_rep = traj_gate(t_c, shaped, PHYS_VMAX, PHYS_AMAX,
                             do_error=True, jmax=PHYS_JMAX, smax=PHYS_SNAP)
    # A-2 금지 구역 (§9): 스플라이스 결합 타임라인도 전 샘플 검사 (비상 세션)
    ko_rep = keep_out_check(shaped, cfg.get("keep_out"), do_error=True)
    return {
        "t": t_c, "base": base_c, "smoothed": smoothed, "shaped": shaped,
        "delta": shaped - smoothed, "yaw": _heading_yaw(t_c, shaped),
        "dt": dt1, "f_mode": f_mode, "shaper_mode": shaper_mode,
        "smoother_info": info_sm, "gate_report": gate_rep, "gate_ok": ok,
        "trajectory_hash": _traj_hash(t_c, shaped),
        "splice_at_s": float(t1[k]),
        "keep_out_report": ko_rep,
    }


# ---------------------------------------------------------------------------
# 궤적 생성 체인
# ---------------------------------------------------------------------------

def _resample_uniform(t, pos_3xN, dt, v0=None):
    """세그먼트별 linspace라 미세 불균일한 시간축 → 균일 그리드 재샘플.

    traj_zv가 균일 샘플(오차 1e-9)을 요구하므로 필수 단계.
    끝점 보존을 위해 linspace 사용 — 실제 간격은 dt에 가장 가까운 등분값.
    v0: 시작 속도 (재계획 이어붙임). 스플라인 경계조건으로 물려야 경계
    흔들림(snap 스파이크 700 실측)이 없다. 종점은 정지(도함수 0) 고정.
    """
    n = max(int(np.round(t[-1] / dt)), 3)
    t_u = np.linspace(0.0, t[-1], n + 1)
    if v0 is None:
        v0 = np.zeros(3)
    pos_u = np.column_stack([
        CubicSpline(t, pos_3xN[i],
                    bc_type=((1, float(v0[i])), (1, 0.0)))(t_u)
        for i in range(3)])
    return t_u, pos_u


def _heading_yaw(t, pos, speed_eps=1e-4):
    """진행 방향(atan2(vy,vx)) yaw. 정지 구간은 직전 값 유지."""
    vel = np.gradient(pos, t, axis=0)
    speed_xy = np.hypot(vel[:, 0], vel[:, 1])
    yaw = np.arctan2(vel[:, 1], vel[:, 0])
    for i in range(1, len(yaw)):
        if speed_xy[i] < speed_eps:
            yaw[i] = yaw[i - 1]
    return np.unwrap(yaw)


def _apply_scan_time_coupling(t, base, dt, ycfg):
    """스캔↔이동 시간 배분 (INTERFACE_SPEC §1, scan.priority 3정책).

    미션 시간 = max(이동, 스캔)이 기본 골격. 반환 (t, base, scan_meta).
      move    : 이동 시간표 불변(최속 도착) + 종점 hold로 스캔 마저 —
                도착 시점 완료율 coverage_at_arrival 보고
      coupled : 이동 균일 팽창(한 동작) — TIME_DILATED 계열 통지
      scan    : 3상 — 스캔 중 이동 진행률 SCAN_SLOW_RATIO(저속 = 패럴랙스·블러
                감소), 스캔 완료 후 잔여 구간 원래 속도
    """
    if ycfg.get("mode") != "scan":
        return t, base, None
    sc = ycfg["scan"]
    T_scan = _scan_duration_s(sc)
    T_move = float(t[-1])
    meta = {"policy": sc["priority"], "T_scan_s": round(T_scan, 3),
            "T_move_s": round(T_move, 3), "coverage_at_arrival": 1.0,
            "time_dilation": None}
    if sc.get("_rate_clamped"):
        meta["rate_clamped"] = sc["_rate_clamped"]
    if T_scan <= T_move:                     # 이동 우세: 정책 무관 결합 불필요
        return t, base, meta

    if sc["priority"] == "move":
        # 최속 도착 유지, 스캔 잔여분은 도착 후 hold에서 완료
        # (패딩 간격은 실제 그리드 간격 - dt 인자와 미세 불일치 시 ZV 거부)
        dtg = float(t[1] - t[0])
        n_pad = int(np.ceil((T_scan - T_move) / dtg))
        t2 = np.concatenate([t, t[-1] + dtg * np.arange(1, n_pad + 1)])
        base2 = np.vstack([base, np.tile(base[-1], (n_pad, 1))])
        meta["coverage_at_arrival"] = round(T_move / T_scan, 4)
        return t2, base2, meta

    # 시간 왜곡(재보간)을 거친 궤적은 "다항식 snap 보장" 범주 이탈 —
    # §7 snap 정책대로 백스톱과 같은 측정-only로 강등 (v/a/j는 그대로 강제).
    meta["snap_guaranteed"] = False
    warp = CubicSpline(t, base, axis=0)      # C2 보간 (선형은 저크 노이즈)

    if sc["priority"] == "scan":
        # 3상: [0,T_scan] 동안 원 타임라인을 SCAN_SLOW_RATIO 속도로 소비,
        # 스캔 완료 후 잔여를 원속으로. 이음새 킥은 후단 스무더가 처리.
        tau1 = SCAN_SLOW_RATIO * T_scan
        if tau1 < T_move:
            total = T_scan + (T_move - tau1)
            n = int(np.round(total / dt))
            t2 = dt * np.arange(n + 1)
            tau = np.where(t2 <= T_scan, t2 * (tau1 / T_scan),
                           tau1 + (t2 - T_scan))
            base2 = warp(np.clip(tau, 0.0, T_move))
            meta["time_dilation"] = round(total / T_move, 4)
            return t2, base2, meta
        # 이동이 너무 짧아 1상 안에 다 소화 -> coupled와 동일 처리로 강등

    # coupled (기본): 균일 팽창 - 이동 중 스캔 완료, 한 동작
    k = T_scan / T_move
    n = int(np.round(T_scan / dt))
    t2 = dt * np.arange(n + 1)
    base2 = warp(np.clip(t2 / k, 0, T_move))
    meta["time_dilation"] = round(k, 4)
    meta["policy_effective"] = "coupled"
    return t2, base2, meta


def _make_yaw(ycfg, t, pos):
    """yaw 기준 생성 (4모드) + yaw 전용 성형(rate/acc/jerk 한계) [rad].

    반환 (yaw, yaw_meta). ZVD는 yaw 비적용 — 요잉은 추력을 기울이지 않아
    짐 스윙과 사실상 비결합 (§1).
    """
    mode = ycfg.get("mode", "heading")
    if mode == "heading":
        raw = _heading_yaw(t, pos)
    elif mode == "hold":
        raw = np.full(len(t), ycfg["angle_rad"])
    elif mode == "look_at":
        tgt = np.asarray(ycfg["target"], float)
        dx = tgt[0] - pos[:, 0]
        dy = tgt[1] - pos[:, 1]
        raw = np.arctan2(dy, dx)
        frozen = np.hypot(dx, dy) < LOOKAT_FREEZE_R
        raw = np.unwrap(raw)
        # 특이점 동결: 목표 수평 근접 구간은 마지막 유효 각 유지 (§1 —
        # 창문 통과 순간). 시작부터 동결이면 첫 유효 각으로 backfill.
        if frozen.any():
            idx = np.where(~frozen, np.arange(len(t)), -1)
            idx = np.maximum.accumulate(idx)
            first_ok = int(np.argmax(~frozen)) if (~frozen).any() else 0
            idx[idx < 0] = first_ok
            raw = raw[idx]
    elif mode == "scan":
        sc = ycfg["scan"]
        a0, a1, rate = sc["from_rad"], sc["to_rad"], sc["rate_rad_s"]
        sweep = abs(a1 - a0)
        sgn = np.sign(a1 - a0) if sweep > 0 else 1.0
        prog = rate * t
        if sc["sweep"] == "once":
            raw = a0 + sgn * np.minimum(prog, sweep)
        else:                                # back_and_forth: 왕복 1회 후 유지
            prog2 = np.minimum(prog, 2 * sweep)
            raw = a0 + sgn * np.where(prog2 <= sweep, prog2,
                                      2 * sweep - prog2)
    else:                                    # 방어 (normalize가 걸렀어야 함)
        raise ValueError(f"yaw.mode='{mode}' 미지원")

    # scan은 성형 상한 = 요청 rate (불가침 — 비전의 제약. 물리 상한 1.0으로
    # 두면 따라잡기 과도에서 요청 rate 초과 = 블러/겹침 계약 위반)
    if mode == "scan":
        rate_lim = ycfg["scan"]["rate_rad_s"]
    else:
        rate_lim = ycfg.get("rate_max", YAW_RATE_MAX)
    shaped, info = traj_smoother(t, raw.reshape(-1, 1),
                                 rate_lim, YAW_ACC_MAX, YAW_JERK_MAX)
    yaw = shaped[:, 0]
    meta = {"mode": mode,
            "rate_peak": round(float(info["vPk"][0]), 4),
            "acc_peak": round(float(info["aPk"][0]), 4),
            "shaping_dev_rad": round(float(info["maxDev"][0]), 4)}
    # yaw 게이트: 성형 후에도 한계 초과면 버그 (성형기가 보장해야 정상)
    if meta["rate_peak"] > rate_lim * 1.02 or meta["acc_peak"] > YAW_ACC_MAX * 1.02:
        raise ValueError(
            f"yaw 게이트 위반: rate {meta['rate_peak']} / acc {meta['acc_peak']}"
            f" (한계 {rate_lim}/{YAW_ACC_MAX}) - 성형기 버그 의심")
    return yaw, meta


def _traj_hash(t, pos):
    """성형 궤적 식별자 (attitude_feedback trajectory_hash 대조용)."""
    h = hashlib.sha256()
    h.update(np.round(t, 6).tobytes())
    h.update(np.round(pos, 6).tobytes())
    return h.hexdigest()[:16]


def build_trajectory(cfg, waypoints, f_mode, v0=None, a0=None, gate_error=True):
    """계획 → 재샘플 → 스무딩 → ZV → 게이트. 반환: dict (산출 일체).

    v0/a0: 재계획 이어붙이기용 초기조건 (splice_waypoints_from_state 산출).
    gate_error=False면 게이트 초과 시 raise 대신 res["gate_ok"]=False 반환
    (traj_report.py 판정 리포트용 — 운용 경로는 True 유지).
    """
    # 동적 배분: 지터 상승 시 유효 계획 한계를 자동 온건화 (승인 기준은 불변
    # — RL 계약 정상성). 수렴하면 요청 스펙 복원.
    margin_dyn, margin_basis = current_jitter_margin()
    lim = dict(cfg["limits"])
    if margin_dyn > JITTER_MARGIN:
        clamped = []
        for key, phys in (("v_max", PHYS_VMAX), ("a_max", PHYS_AMAX),
                          ("j_max", PHYS_JMAX)):
            cap = (1.0 - margin_dyn) * phys
            if np.max(np.asarray(lim[key], float)) > cap:
                lim[key] = (np.minimum(np.asarray(lim[key], float), cap).tolist()
                            if np.ndim(lim[key]) else min(float(lim[key]), cap))
                clamped.append(f"{key}<={cap:.2f}")
        if clamped:
            print(f"[동적 배분] 최근 잔류 지터 {margin_basis.get('median_tail_deg')}"
                  f"도 -> 마진 {margin_dyn:.2f}, 유효 한계 온건화: "
                  + ", ".join(clamped))
    dt = float(cfg.get("dt", 0.01))
    shaper_cfg = cfg.get("shaper", {})
    shaper_mode = shaper_cfg.get("mode", SHAPER_DEFAULT)

    wp_mode = cfg.get("waypoint_mode", "stop")
    if waypoints is not None:
        prep = cfg.get("waypoint_prep", {})
        # divide 기본 없음: fly_through는 다항식 통과 속도가 곡률을 만들어
        # 분할이 불필요(오히려 통과점만 늘림), 정지형은 분할 = 정지 = 성능 손실
        max_seg = prep.get("max_seg_len")
        if max_seg is not None and wp_mode == "stop":
            print("[경고] 정지형 + divide: 분할점마다 정지 -> 가용 성능 미달"
                  " (실측 +40% 소요시간). fly_through 모드 검토 권장")
        n_before = len(waypoints)
        waypoints = normalize_waypoints(
            waypoints,
            merge_dist=float(prep.get("merge_dist", 0.01)),
            collinear_tol=prep.get("collinear_tol"),
            max_seg_len=max_seg,
            divide_mode=prep.get("divide_mode", "linear"))
        if len(waypoints) != n_before:
            print(f"[전처리] waypoint {n_before} -> {len(waypoints)}개 "
                  "(merge/divide)")
    if waypoints is not None and wp_mode == "fly_through":
        # 1a') 무정지 통과 (다항식판): 중간점마다 통과 속도 경계조건 —
        #      정확 통과 + v/a/j/snap 다항식 보장 + 가용 성능 유지
        #      (구 arc-length판은 코너 성형 개입 82cm/snap 비보장으로 대체)
        if v0 is not None or a0 is not None:
            raise ValueError("fly_through는 아직 재계획 초기조건(v0/a0) 미지원"
                             " - waypoint_mode='stop' 사용")
        t_raw, pos_raw, *_ = plan_waypoints_flythrough(
            waypoints, lim["v_max"], lim["a_max"], lim["j_max"],
            lim["snap_max"], dt=dt)
        t, base = _resample_uniform(t_raw, pos_raw, dt)
    elif waypoints is not None:
        # 1a) 정지형: waypoint마다 정지 후 재출발 (snap까지 제약 7차 최소시간)
        t_raw, pos_raw, *_ = plan_waypoints(
            waypoints, lim["v_max"], lim["a_max"], lim["j_max"], lim["snap_max"],
            v0=v0, a0=a0, dt=dt)
        # 2a) 균일 그리드 재샘플 (스플라인 — 정품 궤적은 매끈해서 안전)
        t, base = _resample_uniform(t_raw, pos_raw, dt, v0=v0)
    else:
        # 1b) 원시 궤적 입구: 스텝/거친 프로파일 허용 — 선형 재샘플만 하고
        #     (스플라인은 불연속에서 링잉) 성형은 전부 스무더에 맡긴다.
        #     "unit step이 들어오면 시간을 부여해 ramp로" = 이 경로.
        tr = cfg["trajectory"]
        t_in = np.asarray(tr["t"], float)
        p_in = np.asarray(tr["pos"], float)
        n = max(int(np.round((t_in[-1] - t_in[0]) / dt)), 3)
        t = np.linspace(0.0, t_in[-1] - t_in[0], n + 1)
        base = np.column_stack(
            [np.interp(t + t_in[0], t_in, p_in[:, i]) for i in range(3)])

    # 2b) 스캔↔이동 시간 배분 (scan.priority 3정책 — §1). yaw 블록은
    #     load_mission이 정규화하지만 직접 호출(테스트)도 허용.
    ycfg = cfg.get("yaw")
    if ycfg is None or "mode" not in (ycfg or {}):
        ycfg = normalize_yaw_cfg(ycfg)
    t, base, scan_meta = _apply_scan_time_coupling(t, base, dt, ycfg)
    if scan_meta and scan_meta.get("time_dilation"):
        print(f"[스캔 결합] {scan_meta['policy']}: 이동 {scan_meta['T_move_s']}s"
              f" / 스캔 {scan_meta['T_scan_s']}s -> 팽창"
              f" x{scan_meta['time_dilation']}")

    # 2c) 셰이퍼 지연 hold 패딩: ZV/ZVD는 궤적을 반주기~1주기 지연시키므로
    #     계획 끝에 그만큼 hold를 붙여야 성형 궤적이 종점에 완전 수렴
    #     (미패딩 시 고속 미션 종점 1.1cm 미달 실측)
    def _pad_hold(t_in_, base_in_):
        if shaper_mode == "none":
            return t_in_, base_in_
        delay = (1.0 / f_mode) if shaper_mode == "zvd" else 0.5 / f_mode
        n_pad = int(np.ceil(delay / dt)) + 1
        dt_g = t_in_[1] - t_in_[0]
        t_out_ = np.concatenate(
            [t_in_, t_in_[-1] + dt_g * np.arange(1, n_pad + 1)])
        base_out_ = np.vstack([base_in_, np.tile(base_in_[-1], (n_pad, 1))])
        return t_out_, base_out_

    t, base = _pad_hold(t, base)

    # 3) 물리 한계 포락선 (정품 궤적은 무개입이 정상 — maxDev 로그로 확인)
    smoothed, info_sm = smooth_with_axis_sharing(
        t, base, PHYS_VMAX, PHYS_AMAX, PHYS_JMAX)
    max_dev = float(np.max(info_sm["maxDev"]))

    # 3b) 완화: 원시 궤적의 성형 편차가 크면 경로 보존 재시간화 (거부 대신
    #     시간 재배분 — 공간 의도는 RDP ε 안에서 그대로, 시간만 양보)
    retimed = None
    if waypoints is None and max_dev > RETIME_DEV_TOL:
        tr = cfg["trajectory"]
        t_in = np.asarray(tr["t"], float)
        p_in = np.asarray(tr["pos"], float)
        path_pts = normalize_waypoints(p_in, merge_dist=0.01,
                                       collinear_tol=RETIME_PATH_EPS)
        t_raw2, pos_raw2, *_ = plan_waypoints(
            path_pts, lim["v_max"], lim["a_max"], lim["j_max"],
            lim["snap_max"], dt=dt)
        t, base = _resample_uniform(t_raw2, pos_raw2, dt)
        t, base = _pad_hold(t, base)
        smoothed, info_sm = smooth_with_axis_sharing(
            t, base, PHYS_VMAX, PHYS_AMAX, PHYS_JMAX)
        max_dev = float(np.max(info_sm["maxDev"]))
        req_T = float(t_in[-1] - t_in[0])
        retimed = {"dilation": (float(t[-1]) / req_T if req_T > 0 else None),
                   "path_eps_m": RETIME_PATH_EPS,
                   "n_path_pts": int(len(path_pts)),
                   "requested_T_s": req_T, "retimed_T_s": float(t[-1])}
        print(f"[완화] 성형 편차 초과 -> 경로 보존 재시간화: 경로점 "
              f"{len(path_pts)}개, T {req_T:.2f}s -> {t[-1]:.2f}s "
              f"(팽창 x{retimed['dilation']:.2f})")
    if waypoints is None:
        print(f"[성형] 원시 궤적 재성형량 {max_dev*100:.1f}cm"
              " (스텝 -> S-커브 시간 부여, 의도된 동작)")
    elif max_dev > 0.01:
        print(f"[경고] 스무더 개입 {max_dev*100:.1f}cm -> 계획 한계가 물리"
              " 한계에 너무 근접했거나 입력 궤적 이상. 산출물은 유효(게이트"
              " 통과 시)하나 원인 확인 권장.")

    # 4) 지터 상쇄 오프셋 레이어 (ZV/ZVD — 볼록결합이라 v/a/j·snap 보존)
    #    'none' = 셰이퍼 끔 (지터 유발 A/B 검증용 — 운용 시엔 쓰지 말 것)
    if shaper_mode == "none":
        print("[경고] shaper.mode='none' -> 지터 상쇄 없이 출력 (A/B 검증용)")
        shaped = smoothed.copy()
    else:
        shaped = traj_zv(t, smoothed, f_mode, shaper_mode)
    delta = shaped - smoothed

    # 5) 최종 게이트 (실패 시 raise — 통과분만 컨트롤러로).
    #    snap 검사는 계획층 경로(waypoint)만 강제 — 다항식이 snap_max를 보장하는
    #    경로라 위반=버그. 원시 궤적 백스톱 경로는 스무더가 v/a/j까지만
    #    보장(뱅뱅 저크 = snap 임펄스가 정상 동작)이라 snap은 리포트 마진으로만.
    #    fly_through도 다항식판이라 snap 보장 → 강제 대상.
    #    v0/a0≠0(비상 단독 재계획)만 측정 전용 — 스무더가 정지 초기상태 가정이라
    #    개입 스파이크가 정상 동작 (비행 중 이어붙임 정식 경로는 replan_splice:
    #    결합 타임라인이라 snap까지 강제됨).
    ic_rest = ((v0 is None or not np.any(v0))
               and (a0 is None or not np.any(a0)))
    snap_ok = scan_meta is None or scan_meta.get("snap_guaranteed", True)
    smax = PHYS_SNAP if (waypoints is not None and ic_rest and snap_ok) else None
    ok, gate_rep = traj_gate(t, shaped, PHYS_VMAX, PHYS_AMAX,
                             do_error=gate_error, jmax=PHYS_JMAX, smax=smax)

    # 5b) A-2 금지 구역 (§9, 비상 세션): 최종 성형 궤적 전 샘플 교차 검사.
    #     위반 = KeepOutViolation 즉사 (gate_error=False면 리포트만).
    ko_rep = keep_out_check(shaped, cfg.get("keep_out"), do_error=gate_error)
    if ko_rep["violated"]:
        ok = False

    yaw, yaw_meta = _make_yaw(ycfg, t, shaped)
    if scan_meta:
        yaw_meta["scan"] = scan_meta
    return {
        "t": t, "base": base, "smoothed": smoothed, "shaped": shaped,
        "delta": delta, "yaw": yaw, "dt": dt,
        "f_mode": f_mode, "shaper_mode": shaper_mode,
        "smoother_info": info_sm, "gate_report": gate_rep, "gate_ok": ok,
        "trajectory_hash": _traj_hash(t, shaped),
        "jitter_margin": margin_dyn, "margin_basis": margin_basis,
        "limits_effective": lim, "retimed": retimed,
        "limits_clamped": cfg.get("_limits_clamped"),
        "yaw_meta": yaw_meta,
        "keep_out_report": ko_rep,
    }


def save_outputs(res, waypoints, out_dir=OUTPUT_DIR):
    """컨트롤러 계약 형식으로 저장.

    trajectory.mat : timespot_spl (N,1) / spline_data (N,3) / spline_yaw (N,1)
                     / waypoints (M,3) / jitter_delta (N,3)
                     (run_traj_baked.m·모델워크스페이스 주입 계약 —
                      waypoints는 MATLAB에서 3×M 전치 필요)
    trajectory.json: 동일 내용 JSON (Isaac Sim 등 비MATLAB 소비자용)
    pipeline_meta.json: 예산·성형 개입·게이트 리포트·hash (검증 추적용)
    """
    os.makedirs(out_dir, exist_ok=True)

    mat_path = os.path.join(out_dir, "trajectory.mat")
    profile = res.get("controller_profile", "precision")
    savemat(mat_path, {
        "timespot_spl": res["t"].reshape(-1, 1),
        "spline_data": res["shaped"],
        "spline_yaw": res["yaw"].reshape(-1, 1),
        "waypoints": np.asarray(waypoints, float),
        "jitter_delta": res["delta"],
        "controller_profile": profile,
    })

    _atomic_write_json(os.path.join(out_dir, "trajectory.json"), {
        "dt": res["dt"],
        "trajectory_hash": res["trajectory_hash"],
        "controller_profile": profile,
        "t": res["t"].tolist(),
        "pos": res["shaped"].tolist(),
        "yaw_rad": res["yaw"].tolist(),
    })

    info_sm = res["smoother_info"]
    _atomic_write_json(os.path.join(out_dir, "pipeline_meta.json"), {
        "trajectory_hash": res["trajectory_hash"],
        "phys_limits": {"v_max": PHYS_VMAX, "a_max": PHYS_AMAX,
                        "j_max": PHYS_JMAX},
        "jitter_margin": JITTER_MARGIN,
        "controller_profile": profile,
        "yaw": res.get("yaw_meta"),
        "shaper": {"mode": res["shaper_mode"], "f_mode_hz": res["f_mode"]},
        "smoother": {
            "max_dev_m": float(np.max(info_sm["maxDev"])),
            "xy_share_applied": float(info_sm["xy_share_applied"]),
            "v_peak": info_sm["vPk"].tolist(),
            "a_peak": info_sm["aPk"].tolist(),
            "j_peak": info_sm["jPk"].tolist(),
        },
        "jitter_delta_max_m": float(np.max(np.abs(res["delta"]))),
        "gate_report": res["gate_report"],
        "duration_s": float(res["t"][-1]),
        "n_samples": int(len(res["t"])),
    })
    print(f"[save] {mat_path}")
    print(f"[save] trajectory.json / pipeline_meta.json "
          f"(hash={res['trajectory_hash']}, {res['t'][-1]:.2f}s, "
          f"게이트 통과)")


def run(input_path, out_dir=OUTPUT_DIR):
    cfg, waypoints = load_mission(input_path)
    f_mode = float(cfg.get("shaper", {}).get("f_mode_hz", F_MODE_DEFAULT))
    f_mode, fb = consume_attitude_feedback(f_mode)
    res = build_trajectory(cfg, waypoints, f_mode)
    res["controller_profile"] = cfg["controller_profile"]
    if waypoints is None:
        # 원시 궤적 입구: 시각화용 경유점 = 경로의 RDP(5cm) 대표점.
        # 일직선 경로는 양끝 2점(공선 3점 이상을 주면 Ground/Trajectory/
        # Spline 블록이 거부 - 스텝 미션 2회 실측), 곡선은 코너점 보존.
        p = res["shaped"]
        s = np.concatenate([[0.0], np.cumsum(
            np.linalg.norm(np.diff(p, axis=0), axis=1))])
        if s[-1] < 1e-9:
            raise ValueError("궤적 총 이동거리 0 - 경유점 시각화 불가")
        waypoints = _rdp(p[::10], 0.05)
    save_outputs(res, waypoints, out_dir)
    mark_feedback_used(fb)      # 성공 후에만 used:true (실패 시 다음 기회 소비)
    return res


# ---------------------------------------------------------------------------
# 작업 API — 동사 진입점 (INTERFACE_SPEC §8)
#   종료 코드: 0 성공(조정 포함) / 2 거부(reject) / 1 내부 오류
#   stdout 마지막 줄 = 기계용 JSON 한 줄 — 상위 파서는 마지막 줄만 읽는다
# ---------------------------------------------------------------------------

VERBS = ("plan", "splice", "check", "feedback", "estimate", "status",
         "emergency")
EXIT_OK, EXIT_INTERNAL, EXIT_REJECTED = 0, 1, 2


def _emit(obj):
    """기계용 JSON 한 줄 (§8 계약: stdout 마지막 줄). 사람용 로그는 이 위에."""
    print(json.dumps(obj, ensure_ascii=False))


def _verdict_of(res):
    return ("adjusted" if (res.get("limits_clamped") or res.get("retimed"))
            else "accepted")


def cli_plan(args):
    res = run(args.input, args.out_dir)
    _emit({"verdict": _verdict_of(res),
           "report_path": os.path.join(args.out_dir, "pipeline_meta.json"),
           "trajectory_hash": res["trajectory_hash"]})
    return EXIT_OK


def cli_splice(args):
    """비행 중 새 명령 (새 명령 승리 policy) — current_state 기준 무정지 전환."""
    cfg, wp_new = load_mission(args.input)
    if wp_new is None:
        raise ValueError("splice는 waypoints 입구만 지원 (trajectory 입구 불가)")
    try:
        st = load_current_state(args.state)
    except (ValueError, FileNotFoundError, KeyError) as e:
        # 신선도/부재/스키마 — 낡은 상태 이어붙이기는 미분킥 자초라 전부 거부
        _emit({"verdict": "rejected",
               "reject_codes": [{"code": "STATE_STALE", "detail": str(e)}],
               "report_path": None, "trajectory_hash": None})
        return EXIT_REJECTED
    wp, v0, a0, _j0 = splice_waypoints_from_state(
        st, wp_new.tolist(), emergency=args.emergency)
    f_mode = float(cfg.get("shaper", {}).get("f_mode_hz", F_MODE_DEFAULT))
    res = build_trajectory(cfg, wp, f_mode, v0=v0, a0=a0)
    res["controller_profile"] = cfg["controller_profile"]
    save_outputs(res, wp, args.out_dir)
    _emit({"verdict": _verdict_of(res),
           "report_path": os.path.join(args.out_dir, "pipeline_meta.json"),
           "trajectory_hash": res["trajectory_hash"]})
    return EXIT_OK


def cli_emergency(args):
    """A-1 비상 정지 (§9, 비상 세션): 실측 상태 최단 정지 -> 래치 호버.

    비상 레짐: ZVD 생략 + 마진 반납(물리 한계 풀사용) + snap 측정만.
    상태는 기준(ref_state) 아닌 **실측**(pos/vel/acc) 사용 — 기존 규칙.
    신선도 위반은 splice와 동일하게 STATE_STALE 거부 (낡은 실측에서 만든
    정지 궤적 = 엉뚱한 곳으로 제동).
    """
    from traj_emergency import build_emergency_stop   # 지연 임포트
    try:
        st = load_current_state(args.state)
    except (ValueError, FileNotFoundError, KeyError) as e:
        _emit({"verdict": "rejected",
               "reject_codes": [{"code": "STATE_STALE", "detail": str(e)}],
               "report_path": None, "trajectory_hash": None})
        return EXIT_REJECTED
    res = build_emergency_stop(st, hold_s=args.hold_s)
    # A-2 연동 (§9 적용 범위): 정지 궤적도 구역 검사. 단 정지는 거부하지
    # 않는다 — 물리적으로 불가피한 침범은 KEEP_OUT_UNAVOIDABLE로 보고만
    # (정지가 관통 회피보다 우선. 측방 회피 제동은 후속 확장).
    ko = _load_keep_out(args.keep_out)
    ko_rep = keep_out_check(res["shaped"], ko, do_error=False)
    if ko_rep["violated"]:
        print(f"[emergency] 경고: 제동 경로가 금지 구역 침범 "
              f"(이격 {ko_rep['min_clearance_m']:.2f}m) - "
              "KEEP_OUT_UNAVOIDABLE 보고")
        _append_emergency_ledger("keep_out_unavoidable", {
            "min_clearance_m": ko_rep["min_clearance_m"],
            "zone_idx": ko_rep["zone_idx"],
            "traj_hash": res["trajectory_hash"]})
    res["keep_out_report"] = ko_rep
    p0 = np.asarray(st["pos"], float)
    stop_pt = np.asarray(res["emergency"]["stop_point"], float)
    save_outputs(res, np.vstack([p0, stop_pt]), args.out_dir)
    em = res["emergency"]
    print(f"[emergency] stop: 거리 {em['stop_dist_m']*100:.1f}cm, "
          f"제동 {em['stop_T_s']:.2f}s, hold {em['hold_s']:.1f}s")
    _emit({"verdict": "accepted",
           "report_path": os.path.join(args.out_dir, "pipeline_meta.json"),
           "trajectory_hash": res["trajectory_hash"],
           "emergency": em,
           "keep_out": (dict(ko_rep, code="KEEP_OUT_UNAVOIDABLE")
                        if ko_rep["violated"] else ko_rep)})
    return EXIT_OK


KEEP_OUT_PATH = os.path.join(OUTPUT_DIR, "keep_out.json")   # 감독자가 영속화


def _load_keep_out(path):
    """감독자가 영속화한 keep_out 상태 로드 (없으면 None — 구역 미설정)."""
    p = path or KEEP_OUT_PATH
    if not os.path.isfile(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def _append_emergency_ledger(event, detail):
    """비상 이벤트 원장 기록 (§9 — append-only, 기존 스키마와 별개 형태)."""
    os.makedirs(os.path.dirname(LEDGER_PATH), exist_ok=True)
    with open(LEDGER_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps({
            "event": event,
            "at": datetime.now().strftime(TS_FMT + ".%f")[:-3],
            "detail": detail}, ensure_ascii=False) + "\n")


def cli_check(args):
    """실행 없이 검정만 — 부작용 0 (output/ 미기록, 피드백/원장 비소비).
    RL이 후보 궤도를 대량 사전 질의해도 상태 오염 없음."""
    import traj_report                      # 지연 임포트 (순환 방지)
    report, _res = traj_report.static_report(args.input)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    _emit({"verdict": report["verdict"], "report_path": None,
           "trajectory_hash": (report.get("trajectory") or {}).get("hash")})
    return EXIT_OK if report["verdict"] != "rejected" else EXIT_REJECTED


def cli_feedback(args):
    from analyze_flight_log import analyze, write_feedback
    rep = analyze(args.log)
    write_feedback(rep)
    _emit({"verdict": "accepted", "report_path": FEEDBACK_PATH,
           "trajectory_hash": rep.get("trajectory_hash")})
    return EXIT_OK


def cli_estimate(args):
    from estimate_params import ESTIMATE_PATH, estimate, write_estimate
    est = estimate(args.log)
    write_estimate(est)
    _emit({"verdict": "accepted", "report_path": ESTIMATE_PATH,
           "trajectory_hash": None})
    return EXIT_OK


def cli_status(args):
    """현황 요약: 실시간 상태 + 원장 최근 N건 + 최신 산출물 meta."""
    out = {"current_state": None, "ledger_recent": [], "latest_meta": None}
    try:
        out["current_state"] = load_current_state()
    except (ValueError, FileNotFoundError, KeyError) as e:
        out["current_state_error"] = str(e)
    if os.path.isfile(LEDGER_PATH):
        with open(LEDGER_PATH, encoding="utf-8") as f:
            lines = [json.loads(ln) for ln in f if ln.strip()]
        out["ledger_recent"] = lines[-LEDGER_RECENT_N:]
    meta_p = os.path.join(OUTPUT_DIR, "pipeline_meta.json")
    if os.path.isfile(meta_p):
        with open(meta_p, encoding="utf-8") as f:
            out["latest_meta"] = json.load(f)
    print(json.dumps(out, indent=2, ensure_ascii=False, default=str))
    _emit({"verdict": "accepted", "report_path": None,
           "trajectory_hash": (out["latest_meta"] or {}).get("trajectory_hash")})
    return EXIT_OK


_CLI = {"plan": cli_plan, "splice": cli_splice, "check": cli_check,
        "feedback": cli_feedback, "estimate": cli_estimate,
        "status": cli_status, "emergency": cli_emergency}


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    verb = "plan"                            # 하위 호환: 동사 생략 = plan
    if argv and argv[0] in VERBS:
        verb = argv.pop(0)

    ap = argparse.ArgumentParser(
        description=f"경로 JSON -> 컨트롤러 궤적 체인 (작업 API §8, 동사: {verb})")
    if verb in ("plan", "splice", "check"):
        ap.add_argument("--input",
                        default=os.path.join(INPUT_DIR, "example_mission.json"),
                        help="경로 JSON (기본: input/example_mission.json)")
    if verb in ("plan", "splice", "emergency"):
        ap.add_argument("--out-dir", default=OUTPUT_DIR)
    if verb in ("splice", "emergency"):
        ap.add_argument("--state", default=None,
                        help="current_state.json (기본: RT 경로 - §0)")
    if verb == "splice":
        ap.add_argument("--emergency", action="store_true",
                        help="비상 재계획 - 기준 아닌 측정 상태에서 이어붙임")
    if verb == "emergency":
        ap.add_argument("--hold-s", type=float, default=2.0,
                        help="정지 후 래치 관측 hold [s] (기본 2.0)")
        ap.add_argument("--keep-out", default=None,
                        help="keep_out JSON (기본: output/keep_out.json - "
                             "감독자 영속화 파일)")
    if verb in ("feedback", "estimate"):
        ap.add_argument("--log", required=True,
                        help="비행 로그 .mat (sim_result_baked.mat)")
    args = ap.parse_args(argv)

    try:
        rc = _CLI[verb](args)
    except (KeyError, ValueError, FileNotFoundError) as e:
        # 검증 계열 실패 = 거부 (스키마/예산/게이트/시간역행 - 즉사 원칙의 CLI 번역)
        # 예외가 reject_code를 가지면 그 코드로 (예: KEEP_OUT_VIOLATION §9)
        _emit({"verdict": "rejected",
               "reject_codes": [{"code": getattr(e, "reject_code",
                                                 "SCHEMA_ERROR"),
                                 "detail": str(e)}],
               "report_path": None, "trajectory_hash": None})
        rc = EXIT_REJECTED
    except Exception as e:                   # noqa: BLE001 - 최후 방벽
        _emit({"verdict": "error", "detail": f"{type(e).__name__}: {e}",
               "report_path": None, "trajectory_hash": None})
        rc = EXIT_INTERNAL
    sys.exit(rc)


if __name__ == "__main__":
    main()
