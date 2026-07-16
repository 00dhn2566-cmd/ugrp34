"""
비행(시뮬) 로그 → 잔류 지터 검출 → attitude_feedback.json 생성 (쓰기 측).

미션: 하위(컨트롤러)가 로그를 남기면, 자세제어만으로 못 없애는 지터(도착 후
잔류 진동 = 짐 모드)를 검출해 attitude_feedback.json으로 보고 → traj_pipeline이
소비해 다음 궤적을 보정 (INTERFACE_SPEC §3).

분석 로직은 diagnose_smoother_tail.m의 창 분할 RMS / 영교차 주파수 방식을 따름:
  - tail 구간(궤적 종료 T 이후 hold): pitch/roll RMS, 영교차 주파수,
    최소자승 사인 피팅으로 amp/phase (counter_swing_offset 2호기의 입력)
  - moving 구간(0~T): 자세 피크, 추종 RMS (act/des 로그 있으면)

사용:
    python analyze_flight_log.py                      # 기본 경로에서 읽고 씀
    python analyze_flight_log.py --mat <sim_result_baked.mat> --dry-run
"""

import argparse
import json
import os
import sys
from datetime import datetime

import numpy as np
from scipy.io import loadmat

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_pipeline import OUTPUT_DIR, TS_FMT, _atomic_write_json  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MAT = os.path.join(
    HERE, "controller", "Quadcopter-Drone-Model-Simscape", "sim_result_baked.mat")
META_PATH = os.path.join(OUTPUT_DIR, "pipeline_meta.json")
FEEDBACK_PATH = os.path.join(OUTPUT_DIR, "attitude_feedback.json")


def _ts_struct(m, name):
    """StructureWithTime(To Workspace) 필드 → (t, v). 없으면 즉사."""
    if name not in m:
        raise KeyError(f"로그 변수 '{name}' 없음 — run_traj_baked.m 태핑 확인")
    s = m[name]
    return np.ravel(s.time), np.ravel(s.signals.values)


def _zero_crossing_freq(t, x):
    """영교차 주파수 [Hz] (diagnose_smoother_tail.m 방식). 교차 2회 미만이면 None."""
    x = x - np.mean(x)
    sign_change = np.where(np.diff(np.sign(x)) != 0)[0]
    if len(sign_change) < 2:
        return None
    # 인접 영교차 간격 = 반주기
    half_periods = np.diff(t[sign_change])
    return float(1.0 / (2.0 * np.mean(half_periods)))


def _fit_sine(t, x, f_hz, t_ref):
    """고정 주파수 최소자승 사인 피팅 → (amp, phase).

    x(t) ≈ A·sin(w(t - t_ref) + φ). counter_swing_offset이 같은 규약
    (w(t - t_ref) + phase)을 쓰므로 위상 기준이 일치한다.
    """
    w = 2.0 * np.pi * f_hz
    arg = w * (t - t_ref)
    M = np.column_stack([np.sin(arg), np.cos(arg)])
    (a, b), *_ = np.linalg.lstsq(M, x - np.mean(x), rcond=None)
    amp = float(np.hypot(a, b))
    phase = float(np.arctan2(b, a))
    return amp, phase


def analyze(mat_path, t_traj_end=None):
    """sim_result_baked.mat → 지터 리포트 dict."""
    m = loadmat(mat_path, squeeze_me=True, struct_as_record=False)
    t_r, roll = _ts_struct(m, "real_roll")
    t_p, pitch = _ts_struct(m, "real_pitch")
    roll_deg = np.degrees(roll)
    pitch_deg = np.degrees(pitch)

    if t_traj_end is None:
        if "timespot_spl" not in m:
            raise KeyError("timespot_spl 없음 — 궤적 종료 시각 판정 불가")
        t_traj_end = float(np.ravel(m["timespot_spl"])[-1])
    t_end = float(t_p[-1])
    if t_end <= t_traj_end + 1.0:
        raise ValueError(
            f"tail 구간 없음 (로그 끝 {t_end:.1f}s <= 궤적 끝 {t_traj_end:.1f}s"
            " + 1s) — run_traj_baked.m의 T_hold 마진 확인")

    # --- tail: 도착 후 잔류 진동 = 자세제어가 못 없애는 지터 본체 ---
    tail_p = (t_p >= t_traj_end)
    tail_r = (t_r >= t_traj_end)
    tp_, pp_ = t_p[tail_p], pitch_deg[tail_p]
    tr_, rr_ = t_r[tail_r], roll_deg[tail_r]
    pitch_rms = float(np.sqrt(np.mean((pp_ - np.mean(pp_)) ** 2)))
    roll_rms = float(np.sqrt(np.mean((rr_ - np.mean(rr_)) ** 2)))

    # 지배축(피치/롤 중 RMS 큰 쪽)으로 주파수·위상 추정
    dom_t, dom_x = (tp_, pp_) if pitch_rms >= roll_rms else (tr_, rr_)
    f_mode = _zero_crossing_freq(dom_t, dom_x)
    amp, phase = (0.0, 0.0)
    if f_mode is not None:
        amp, phase = _fit_sine(dom_t, dom_x, f_mode, t_traj_end)

    # --- moving: 이동 중 자세 피크 + 추종 RMS (act/des 있으면) ---
    mov_p = pitch_deg[t_p < t_traj_end]
    mov_r = roll_deg[t_r < t_traj_end]
    att_peak = float(max(np.max(np.abs(mov_p)), np.max(np.abs(mov_r))))
    track_rms_cm = None
    if all(k in m for k in ("sim_time", "act_x1", "des_x1",
                            "act_y1", "des_y1", "act_z1", "des_z1")):
        st = np.ravel(m["sim_time"])
        n = min(len(st), *(len(np.ravel(m[k])) for k in
                           ("act_x1", "des_x1", "act_y1", "des_y1",
                            "act_z1", "des_z1")))
        mask = st[:n] < t_traj_end
        err2 = np.zeros(int(np.sum(mask)))
        for ax in ("x", "y", "z"):
            act = np.ravel(m[f"act_{ax}1"])[:n][mask]
            des = np.ravel(m[f"des_{ax}1"])[:n][mask]
            err2 = err2 + (act - des) ** 2
        track_rms_cm = float(np.sqrt(np.mean(err2)) * 100.0)

    return {
        "t_traj_end": t_traj_end,
        "mode_freq_hz": f_mode,
        "tail": {
            "pitch_rms_deg": round(pitch_rms, 3),
            "roll_rms_deg": round(roll_rms, 3),
            "amp_deg": round(amp, 3),
            "phase_rad": round(phase, 4),
            "t_ref_s": t_traj_end,
        },
        "moving": {
            "att_peak_deg": round(att_peak, 2),
            "track_rms_cm": (round(track_rms_cm, 2)
                             if track_rms_cm is not None else None),
        },
    }


def write_feedback(report, feedback_path=FEEDBACK_PATH, meta_path=META_PATH):
    """리포트 → attitude_feedback.json (used:false, INTERFACE_SPEC §3)."""
    traj_hash = None
    if os.path.isfile(meta_path):
        with open(meta_path, encoding="utf-8") as f:
            traj_hash = json.load(f).get("trajectory_hash")
    now = datetime.now().strftime(TS_FMT)
    fb = {
        "flight_id": now,
        "written_at": now,
        "used": False,
        "trajectory_hash": traj_hash,
        "mode_freq_hz": report["mode_freq_hz"],
        "tail": report["tail"],
        "moving": report["moving"],
    }
    os.makedirs(os.path.dirname(feedback_path), exist_ok=True)
    _atomic_write_json(feedback_path, fb)
    print(f"[write] {feedback_path} (used:false, hash={traj_hash})")
    return fb


# 비행 유효성 임계: 추종 RMS가 이 값을 넘으면 비행 실패 = 측정치 무효
# (발산 비행의 tail 신호는 짐 모드가 아니라 발산 과도 — 피드백 오염 방지)
TRACK_RMS_VALID_CM = 30.0


def main():
    ap = argparse.ArgumentParser(description="시뮬 로그 → 지터 검출 → 피드백 JSON")
    ap.add_argument("--mat", default=DEFAULT_MAT)
    ap.add_argument("--dry-run", action="store_true",
                    help="리포트만 출력, attitude_feedback.json 안 씀")
    args = ap.parse_args()

    report = analyze(args.mat)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    track = report["moving"]["track_rms_cm"]
    if track is not None and track > TRACK_RMS_VALID_CM:
        raise ValueError(
            f"비행 실패 (추종 RMS {track:.0f}cm > {TRACK_RMS_VALID_CM}cm) -> "
            "측정치 무효, 피드백 쓰지 않음. 발산 원인부터 해결할 것")
    if report["mode_freq_hz"] is None:
        print("[정보] tail에서 진동 미검출 (영교차 부족) -> 수렴 상태로 판단, "
              "피드백을 쓰지 않음 (추론 ③ 수렴 시 무수정)")
        return
    if not args.dry_run:
        write_feedback(report)


if __name__ == "__main__":
    main()
