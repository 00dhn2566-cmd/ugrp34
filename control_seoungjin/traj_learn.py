# -*- coding: utf-8 -*-
"""traj_learn.py — 사후 비행 로그로 궤적을 살짝 보정하는 반복 학습(ILC) 층 (2026-08-18, 성능 지표 세션).

아이디어(사용자): "비행 중 로그를 남기고, 사후에 그 로그를 보고 다음 비행의 경로를 살짝씩 조정" — 컨트롤러 게인은
그대로 두고(회귀 위험 없음), 같은 미션을 다시 날 때 **기준 궤적에 보정 오프셋 c(t)를 더해** 추종 오차를 반복적으로 줄인다.

  e_k(t)   = target(t) − act_k(t)            target = 상위가 준 원 궤적(셰이퍼 통과 후, 보정 전)
  c_{k+1}  = clip( c_k + clip(L · LPF(e_k), ±UPDATE_MAX) , ±C_MAX ) · taper(t)   (영위상 저역 LPF, 양끝 테이퍼로 시작·종점 불변)
  한계(사용자 요구): |c| ≤ C_MAX 5 cm (추종 예산의 절반) — 기체는 항상 "원 명령 ±5 cm" 안에서만 기준을 바꾼다; 회당 갱신 ≤ 3 cm;
  plan 적용 시 파일의 c 를 다시 검사(상한·시작/종점 0)해 위반이면 거부.
  다음 비행 기준 = shaped(t) + c_{k+1}(t)  → 게이트(v/a/j)·keep_out 재검사 (위반 시 L 절반으로 재시도)

파일: output/traj_correction.json
  { "schema_version": 0.1, "base_trajectory_hash": <보정 전 shaped 해시>, "iter": k, "gain": L, "lpf_hz": f,
    "t": [...], "c": [[dx,dy,dz], ...] (m), "history": [{"iter", "log", "rms_before_cm"(축별/3D), "c_max_cm", ...}] }
안전 규약: base_trajectory_hash 가 plan 시점의 보정 전 궤적과 다르면 즉사(다른 미션·셰이퍼 주파수·limits 변경 = 보정 무효).
상위 계층 계약(INTERFACE_SPEC §1/§7)은 불변 — 회신에 correction_iter 만 추가된다.

사용:
  python traj_pipeline.py plan  --input input/step_mission.json                                  # k=0 비행 (기준)
  python traj_pipeline.py learn --input input/step_mission.json --log <sim_result_baked.mat>       # 로그 → 보정 k=1
  python traj_pipeline.py plan  --input input/step_mission.json --correction output/traj_correction.json   # k=1 비행
  (반복)  learn --correction output/traj_correction.json --log ...   →   plan --correction ...
"""
import json
import os

import numpy as np

CORRECTION_SCHEMA = 0.1
DEFAULT_GAIN = 0.6         # ILC 학습률 L (0<L<1: 단조 수렴 안전권; 플랜트 위상 지연 ≤ LPF 대역에서)
DEFAULT_LPF_HZ = 1.0       # 영위상 저역 차단 [Hz] — 짐 모드(1.8 Hz)·셰이퍼 대역 가드(1~3 Hz) 아래
C_MAX_M = 0.05             # 보정 오프셋 절대 상한 [m] — 사용자 요구(08-18) "기준 변경에 한계": 추종 예산 10 cm 의 절반. 어떤 시점에도 기준은 원 명령에서 이만큼 이상 못 벗어남
UPDATE_MAX_M = 0.03        # 회당 갱신 상한 [m] — 한 번의 학습으로 기준이 튀지 않게
TAPER_S = 1.5              # 시작 테이퍼 [s] — 이륙 과도(새그, 추력 상승 지연)는 학습 대상 아님 + 이륙점 불변
TAPER_END_S = 1.0          # 끝 테이퍼 [s] — 연장 hold 끝에서 0 (종점 불변)
ATT_GAIN = 0.6             # 자세 항 학습률 (꼬리 잔류 스윙 역위상 오프셋 누적 비율)
ATT_JERK_BUDGET = 2.0      # counter_swing_offset 저크 예산 [m/s³] (PIPELINE_STATUS 2호기 설계값 → 1.9 Hz 진폭 ≤1.2 mm)
ATT_MIN_AMP_DEG = 0.05     # 이 미만 잔류는 학습 안 함 (측정 잡음)
HOLD_EXT_S = 1.5           # 보정 적용 시 궤적 뒤에 붙이는 hold 연장 [s] — 도착 후 정착(지연·오버슈트) 구간까지 보정하려면 기준이 그만큼 있어야 함


def _lpf_zero_phase(x, dt, fc):
    """영위상 2차 버터워스 (scipy 있으면 filtfilt, 없으면 전후 이동평균 2회)."""
    try:
        from scipy.signal import butter, filtfilt
        b, a = butter(4, fc / (0.5 / dt))          # 4차: 저크/스냅(ω³·ω⁴) 성분 억제 → 게이트 통과
        return filtfilt(b, a, x, axis=0)
    except Exception:  # noqa: BLE001 - scipy 부재 시 대체
        n = max(1, int(round(1.0 / (2.0 * np.pi * fc) / dt)))
        k = np.ones(n) / n
        y = np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 0, x)
        return np.apply_along_axis(lambda v: np.convolve(v[::-1], k, mode="same")[::-1], 0, y)


def _taper(t, taper_s=TAPER_S, taper_end_s=TAPER_END_S):
    T = float(t[-1] - t[0])
    if T <= taper_s + taper_end_s:
        return np.ones_like(t)
    a = (t - t[0]) / taper_s
    b = (t[-1] - t) / taper_end_s
    w = np.clip(np.minimum(1.0, np.minimum(a, b)), 0, 1)
    return w ** 3 * (10 - 15 * w + 6 * w ** 2)      # 5차 smoothstep (C² 연속 → 게이트 저크/스냅 스파이크 없음)


def extend_hold(t, pos, ext_s=HOLD_EXT_S):
    """궤적 뒤에 마지막 샘플 hold 를 ext_s 만큼 붙인다 (보정 정의역 = 정착 구간 포함)."""
    dt = float(t[1] - t[0])
    n = int(round(ext_s / dt))
    if n <= 0:
        return t, pos
    t2 = np.concatenate([t, t[-1] + dt * np.arange(1, n + 1)])
    p2 = np.vstack([pos, np.tile(pos[-1], (n, 1))])
    return t2, p2


def load_flight_tracks(mat_path):
    """sim_result_*.mat (run_traj_baked 저장 형식) → dict(t_ref, ref(N,3), act_on_ref(N,3))."""
    from scipy.io import loadmat
    m = loadmat(mat_path, squeeze_me=True, struct_as_record=False)
    if "trk" not in m or "timespot_spl" not in m or "spline_data" not in m:
        raise KeyError("로그에 trk/timespot_spl/spline_data 없음 - run_traj_baked.m 저장 형식 아님")
    t_ref = np.ravel(np.asarray(m["timespot_spl"], float))
    ref = np.asarray(m["spline_data"], float).reshape(len(t_ref), 3)
    trk = m["trk"]
    act = np.zeros_like(ref)
    tracks = {}
    for i, ax in enumerate("xyz"):
        s = getattr(trk, ax)
        ta = np.ravel(np.asarray(s.t, float))
        va = np.ravel(np.asarray(s.act, float))
        tracks[ax] = (ta, va)                       # 원시 실측 (hold 구간 포함 — 연장 격자 보간용)
        act[:, i] = np.interp(t_ref, ta, va)
    return {"t": t_ref, "ref": ref, "act": act, "tracks": tracks}


def act_on_grid(log, t):
    """원시 실측 트랙을 임의 시간축 t 위로 보간 (N,3)."""
    return np.column_stack([np.interp(t, *log["tracks"][ax]) for ax in "xyz"])


def load_correction(path):
    with open(path, encoding="utf-8") as f:
        c = json.load(f)
    if c.get("schema_version") != CORRECTION_SCHEMA:
        raise ValueError(f"traj_correction schema_version {c.get('schema_version')} != {CORRECTION_SCHEMA}")
    c["t"] = np.asarray(c["t"], float)
    c["c"] = np.asarray(c["c"], float).reshape(len(c["t"]), 3)
    return c


def correction_on_grid(corr, t):
    """보정 c(t)를 계획 시간축 t 위로 보간 (범위 밖 0)."""
    out = np.zeros((len(t), 3))
    for i in range(3):
        out[:, i] = np.interp(t, corr["t"], corr["c"][:, i], left=0.0, right=0.0)
    return out


def apply_correction(t, shaped, corr, base_hash):
    """plan 단계 훅: base 해시 검증 → 보정 한계 재검증(|c| ≤ C_MAX_M, 시작·종점 0) → hold 연장 → shaped_ext + c.
    반환 (t_ext, shaped_base_ext, corrected, meta). 한계 위반 = 즉사 (파일 손상/임의 수정 방어 — 기체는 항상 명령 ±C_MAX 안)."""
    if corr["base_trajectory_hash"] != base_hash:
        raise ValueError(
            f"traj_correction base_trajectory_hash 불일치 ({corr['base_trajectory_hash']} != {base_hash}) — "
            "미션/limits/셰이퍼 주파수가 바뀌었으면 보정 무효: learn 부터 다시")
    cmax_file = float(corr.get("c_max_m", C_MAX_M))
    if cmax_file > C_MAX_M + 1e-12:
        raise ValueError(f"traj_correction c_max_m={cmax_file} > 규약 상한 {C_MAX_M} m — 거부")
    if np.abs(corr["c"]).max() > C_MAX_M + 1e-9:
        raise ValueError(f"traj_correction |c| 최대 {np.abs(corr['c']).max()*100:.1f} cm > 상한 {C_MAX_M*100:.0f} cm — 거부")
    if np.abs(corr["c"][0]).max() > 1e-6 or np.abs(corr["c"][-1]).max() > 1e-6:
        raise ValueError("traj_correction 시작/종점 오프셋이 0 이 아님 — 이륙점·종점 불변 규약 위반, 거부")
    t2, sh2 = extend_hold(t, shaped, float(corr.get("hold_ext_s", HOLD_EXT_S)))
    c = correction_on_grid(corr, t2)
    return t2, sh2, sh2 + c, {"correction_iter": int(corr["iter"]), "correction_max_cm": float(np.abs(c).max() * 100),
                              "c_max_m": C_MAX_M, "update_max_m": UPDATE_MAX_M,
                              "gain": corr["gain"], "lpf_hz": corr["lpf_hz"], "hold_ext_s": float(t2[-1] - t[-1])}


def learn(t_ref, target, act, prev=None, gain=DEFAULT_GAIN, lpf_hz=DEFAULT_LPF_HZ, c_max=C_MAX_M, upd_max=UPDATE_MAX_M):
    """한 번의 ILC 갱신. target = 보정 전 원 궤적(shaped), act = 실측(같은 t_ref 격자).
    prev = 이전 correction dict(없으면 c_0=0). 반환 (c_new (N,3), stats)."""
    dt = float(np.median(np.diff(t_ref)))
    e = target - act
    c_prev = correction_on_grid(prev, t_ref) if prev is not None else np.zeros_like(target)
    c_max = min(float(c_max), C_MAX_M)                          # 호출자가 더 크게 못 늘림 (상한은 규약)
    upd = np.clip(gain * _lpf_zero_phase(e, dt, lpf_hz), -upd_max, upd_max)
    c_new = np.clip(c_prev + upd, -c_max, c_max) * _taper(t_ref)[:, None]
    rms = np.sqrt(np.mean(e ** 2, axis=0)) * 100
    stats = {"rms_before_cm": {"x": float(rms[0]), "y": float(rms[1]), "z": float(rms[2]),
                               "3d": float(np.sqrt(np.sum(rms ** 2)))},
             "max_err_cm": float(np.abs(e).max() * 100),
             "c_max_cm": float(np.abs(c_new).max() * 100),
             "update_max_cm": float(np.abs(upd).max() * 100),
             "bounds_cm": {"c_max": c_max * 100, "update_max": upd_max * 100},
             "saturated": bool(np.any(np.abs(c_prev + upd) > c_max - 1e-9))}   # 상한에 걸림 = 더 못 배움 (경로/게인 쪽 문제 신호)
    return c_new, stats


def write_correction(path, t_ref, c, base_hash, it, gain, lpf_hz, history):
    obj = {"schema_version": CORRECTION_SCHEMA, "base_trajectory_hash": base_hash, "iter": int(it),
           "gain": float(gain), "lpf_hz": float(lpf_hz), "c_max_m": C_MAX_M, "update_max_m": UPDATE_MAX_M, "taper_s": TAPER_S,
           "taper_end_s": TAPER_END_S, "hold_ext_s": HOLD_EXT_S,
           "t": [float(v) for v in t_ref], "c": [[float(v) for v in row] for row in c],   # 반올림 금지: 1e-5 반올림이 snap 1000 급 킹크를 만듦 (실측)
           "history": history}
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False)
    os.replace(tmp, path)
    return obj


def attitude_counter_term(log_path, t_grid, calib_path, gain=ATT_GAIN, jerk_budget=ATT_JERK_BUDGET, meta_path=None):
    """자세 항 (2호기 counter_swing 배선, 사용자 요청 08-18 "자세가 떨고 있는 걸 사후 경로 조정으로"):
    로그 꼬리(궤적 종료+셰이퍼 지연 이후)의 pitch/roll 잔류를 f_mode 사인으로 피팅(analyze_flight_log.analyze) →
    swing_calib.json(S °/(m/s²), 위상지연)로 위치 진폭·위상 환산(counter_params_from_calib) → 역위상 오프셋
    (counter_swing_offset, 저크 예산 클램프) × gain 을 지배축(pitch→x, roll→y)에 얹는다. 반환 (offset (N,3), info) —
    잔류가 작거나 교정/대역 조건이 안 맞으면 (0, info{"skipped":사유}) (즉사 아님: 위치 ILC 는 계속).
    """
    import json as _json
    from analyze_flight_log import analyze
    from traj_shaping import counter_swing_offset, counter_params_from_calib
    off = np.zeros((len(t_grid), 3))
    try:
        rep = analyze(log_path) if meta_path is None else analyze(log_path)
    except (KeyError, ValueError) as e:
        return off, {"skipped": f"tail 분석 불가: {e}"}
    tail = rep["tail"]
    if rep["mode_freq_hz"] is None or tail["amp_deg"] < ATT_MIN_AMP_DEG:
        return off, {"skipped": f"잔류 스윙 없음/작음 (amp {tail['amp_deg']}°, f {rep['mode_freq_hz']})",
                     "tail_amp_deg": tail["amp_deg"], "tail_pitch_rms_deg": tail["pitch_rms_deg"], "tail_roll_rms_deg": tail["roll_rms_deg"]}
    if not os.path.isfile(calib_path):
        return off, {"skipped": f"swing_calib 없음: {calib_path}", "tail_amp_deg": tail["amp_deg"]}
    with open(calib_path, encoding="utf-8") as f:
        calib = _json.load(f)
    try:
        prm = counter_params_from_calib(calib, tail)
    except ValueError as e:
        return off, {"skipped": f"교정 소비 거부: {e}", "tail_amp_deg": tail["amp_deg"]}
    f_use = float(rep["mode_freq_hz"])          # 실측 잔류 주파수 (교정 f0 와 근접해야 정상)
    if abs(f_use - prm["f_mode"]) > 0.4:
        return off, {"skipped": f"잔류 f {f_use:.2f} Hz 가 교정 f0 {prm['f_mode']:.2f} 와 0.4 Hz 초과 차이", "tail_amp_deg": tail["amp_deg"]}
    ax = 0 if tail["pitch_rms_deg"] >= tail["roll_rms_deg"] else 1     # 지배축: pitch→x, roll→y (yaw≈0 가정)
    o, a_used = counter_swing_offset(t_grid, gain * prm["amp_pos_m"], prm["phase_rad"], float(tail["t_ref_s"]), f_use, jerk_budget)
    off[:, ax] = o
    return off, {"axis": "x" if ax == 0 else "y", "tail_amp_deg": tail["amp_deg"], "tail_phase_rad": tail["phase_rad"],
                 "tail_pitch_rms_deg": tail["pitch_rms_deg"], "tail_roll_rms_deg": tail["roll_rms_deg"],
                 "f_mode_hz": f_use, "t_ref_s": float(tail["t_ref_s"]), "amp_pos_mm": float(a_used * 1000),
                 "amp_pos_req_mm": float(gain * prm["amp_pos_m"] * 1000), "gain": gain}
