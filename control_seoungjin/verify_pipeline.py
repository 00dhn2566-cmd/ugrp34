"""
전체 파이프라인 MATLAB 검증 매트릭스 — 게인 확정 후 한 줄 재실행용.

미션 5종(정지형/fly_through/스텝 백스톱/지터 A/지터 B')을 순차로:
  생성(traj_pipeline) → 구운 모델 비행(run_traj_baked) → 지터 분석
하고 output/verification_matrix.json + 콘솔 표로 요약한다.

안전장치: 시작 전 다른 MATLAB 프로세스가 돌고 있으면 즉사 (16GB 머신,
동시 2개 금지 — 시스템 다운 전력. 튜닝 세션과의 충돌 방지).

사용:
    python verify_pipeline.py              # 전체 (MATLAB 5회, ~30분)
    python verify_pipeline.py --static     # 생성/게이트만 (MATLAB 없이)
    python verify_pipeline.py --only step,jitter_a
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_pipeline import OUTPUT_DIR, _atomic_write_json  # noqa: E402
import traj_pipeline as tp                                 # noqa: E402
from analyze_flight_log import analyze                     # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SUB = os.path.join(HERE, "controller", "Quadcopter-Drone-Model-Simscape")
MATLAB = os.environ.get(
    "MATLAB_EXE", r"C:\Program Files\MATLAB\R2026a\bin\matlab.exe")
MATRIX_PATH = os.path.join(OUTPUT_DIR, "verification_matrix.json")

# (이름, 미션 파일, 비고)
MISSIONS = [
    ("stop_batch", "input/example_mission.json", "정지형 배치 (운용 기본)"),
    ("fly_through", "input/flythrough_mission.json", "무정지 통과"),
    ("step", "input/step_mission.json", "원시 스텝 백스톱"),
    ("jitter_a", "input/aggressive_mission.json", "셰이퍼 off (지터 기준선)"),
    ("jitter_b", "input/aggressive_mission_b.json", "ZVD@1.8 (소거 검증)"),
]


def _other_matlab_running():
    """다른 MATLAB 프로세스 감지 (동시 실행 금지 규칙)."""
    try:
        out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq MATLAB.exe"],
                             capture_output=True, text=True).stdout
        return "MATLAB.exe" in out
    except Exception:
        return False


def _parse_tracking(log_path):
    """run_traj_baked 로그(cp949)에서 자세/추종 요약 파싱."""
    try:
        txt = open(log_path, encoding="cp949", errors="replace").read()
    except OSError:
        return {}
    out = {}
    m = re.search(r"RMS ([\d.]+)\S* / \S+\|roll\| ([\d.]+)\S* / \S+\|pitch\| ([\d.]+)", txt)
    if m:
        out["att_rms_deg"] = float(m.group(1))
        out["roll_peak_deg"] = float(m.group(2))
        out["pitch_peak_deg"] = float(m.group(3))
    for ax in "xyz":
        m = re.search(rf"{ax}\S*: RMS ([\d.]+)m / \S+ ([\d.]+)m / \S+ ([\d.]+)m", txt)
        if m:
            out[f"track_{ax}"] = {"rms_m": float(m.group(1)),
                                  "max_m": float(m.group(2)),
                                  "end_m": float(m.group(3))}
    return out


def run_matrix(only=None, static=False):
    if not static and _other_matlab_running():
        raise RuntimeError(
            "다른 MATLAB 프로세스 실행 중 - 동시 시뮬 금지 (16GB, 다운 전력). "
            "튜닝/다른 세션 종료 후 재시도하거나 --static으로 생성만 검증")

    rows = []
    for name, mission, note in MISSIONS:
        if only and name not in only:
            continue
        row = {"name": name, "mission": mission, "note": note}
        print(f"\n===== {name}: {note} =====")
        try:
            res = tp.run(os.path.join(HERE, mission))
            row["generated"] = True
            row["gate_ok"] = bool(res["gate_ok"])
            row["duration_s"] = round(float(res["t"][-1]), 2)
            row["trajectory_hash"] = res["trajectory_hash"]
        except Exception as e:
            row["generated"] = False
            row["error"] = str(e)[:200]
            rows.append(row)
            print(f"[생성 실패] {e}")
            continue

        if not static:
            shutil.copy(os.path.join(OUTPUT_DIR, "trajectory.mat"),
                        os.path.join(SUB, "trajectory.mat"))
            log = os.path.join(SUB, f"run_verify_{name}.txt")
            with open(log, "w") as f:
                rc = subprocess.run([MATLAB, "-batch", "run_traj_baked"],
                                    cwd=SUB, stdout=f, stderr=f).returncode
            row["matlab_exit"] = rc
            row.update(_parse_tracking(log))
            try:
                fl = analyze(os.path.join(SUB, "sim_result_baked.mat"))
                row["tail_pitch_rms_deg"] = fl["tail"]["pitch_rms_deg"]
                row["tail_roll_rms_deg"] = fl["tail"]["roll_rms_deg"]
                row["residual_freq_hz"] = fl["mode_freq_hz"]
                row["moving_track_rms_cm"] = fl["moving"]["track_rms_cm"]
            except Exception as e:
                row["analyze_error"] = str(e)[:200]
        rows.append(row)

    _atomic_write_json(MATRIX_PATH, {"rows": rows})
    print(f"\n[write] {MATRIX_PATH}")
    # 요약 표
    print(f"\n{'미션':<12} {'생성':<4} {'게이트':<6} {'추종cm':<8} {'tail p/r':<12}")
    for r in rows:
        track = r.get("moving_track_rms_cm")
        tail = (f"{r.get('tail_pitch_rms_deg','-')}/"
                f"{r.get('tail_roll_rms_deg','-')}")
        print(f"{r['name']:<12} {str(r.get('generated')):<4} "
              f"{str(r.get('gate_ok','-')):<6} {str(track):<8} {tail:<12}")
    return rows


def main():
    ap = argparse.ArgumentParser(description="파이프라인 검증 매트릭스")
    ap.add_argument("--static", action="store_true", help="생성/게이트만")
    ap.add_argument("--only", default=None, help="쉼표 구분 미션 이름")
    args = ap.parse_args()
    run_matrix(only=set(args.only.split(",")) if args.only else None,
               static=args.static)


if __name__ == "__main__":
    main()
