"""비상 검증 오케스트레이터 (§9 검증 의무 — 비상 세션).

에피소드 (MATLAB 정답 플랜트, run_traj_baked.m 재사용):
    ① A-1 정지: 고속 순항 비행 -> τ 시점 실측 상태 채취 -> emergency 동사로
       정지 궤적 -> 순항[0..τ]+정지 합성 기준 재비행 -> 오버슈트/래치 드리프트.
       합격선(HANDOFF_EMERGENCY §7): v>=1.0 정지 오버슈트 < 10cm,
       래치 후 드리프트 < 5cm/8s.
    ② A-2 회피: 구역 관통 미션 -> keep_out_avoid_waypoints 재계획 -> 실비행
       실측 경로의 구역 최소 이격 >= 0 (inflate 포함).

주의:
    - MATLAB 1대 규칙: 다른 MATLAB 프로세스 감지 시 즉사 (SESSIONS_BOARD
      점유 확인 후 실행할 것. RAM 16GB — 동시 시뮬 = 실제 다운 전력).
    - 이 스크립트는 작성 후 미실행 (튜닝 세션 MATLAB 점유 중) — 첫 실행 시
      로그 전체 확인 (run_traj_baked.m 유의사항과 동일).

사용:
    python verify_emergency.py            # ① + ② (MATLAB 3회, ~20분)
    python verify_emergency.py --only a1  # ①만
    python verify_emergency.py --only a2  # ②만
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime, timedelta

import numpy as np
from scipy.io import loadmat, savemat

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_shaping import (                      # noqa: E402
    keep_out_avoid_waypoints,
    keep_out_clearance,
)

HERE = os.path.dirname(os.path.abspath(__file__))
SUB = os.path.join(HERE, "controller", "Quadcopter-Drone-Model-Simscape")
OUT = os.path.join(HERE, "output", "verify_emergency")
MATLAB = os.environ.get(
    "MATLAB_EXE", r"C:\Program Files\MATLAB\R2026a\bin\matlab.exe")
PY = sys.executable

# ① 합격선 (HANDOFF_EMERGENCY §7 — 첫 실측 후 보드에 조정 근거와 함께 갱신)
A1_OVERSHOOT_MAX_M = 0.10
A1_DRIFT_MAX_M = 0.05
A1_HOLD_S = 8.0

# ② 구역 (검증용 시나리오): 직선 경로 정중앙 관통 -> 회피 필수
A2_ZONE = {"shape": "sphere", "center": [4.0, 0.0, 2.0], "radius_m": 1.0}
A2_INFLATE = 0.5


def _die(msg):
    raise SystemExit(f"[verify_emergency] 실패: {msg}")


def _other_matlab_running():
    try:
        out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq MATLAB.exe"],
                             capture_output=True, text=True).stdout
        return "MATLAB.exe" in out
    except OSError:
        return False


def _run_matlab_flight(tag):
    """<SUB>/trajectory.mat -> run_traj_baked -> sim_result_<tag>.mat 경로 반환."""
    if _other_matlab_running():
        _die("다른 MATLAB 실행 중 - 동시 시뮬 금지 (보드 점유 확인)")
    log = os.path.join(OUT, f"matlab_{tag}.log")
    with open(log, "w", encoding="utf-8") as f:
        rc = subprocess.run([MATLAB, "-batch", "run_traj_baked"],
                            cwd=SUB, stdout=f, stderr=subprocess.STDOUT,
                            timeout=1800).returncode
    if rc != 0:
        _die(f"MATLAB 종료 코드 {rc} - 로그 확인: {log}")
    src = os.path.join(SUB, "sim_result_baked.mat")
    if not os.path.isfile(src):
        _die(f"sim_result_baked.mat 미생성 - 로그 확인: {log}")
    dst = os.path.join(OUT, f"sim_result_{tag}.mat")
    shutil.copy(src, dst)
    return dst


def _series(md, name, sim_time):
    """sim_result의 로그 변수 -> (t, v). Array/StructureWithTime 2형식."""
    x = md.get(name)
    if x is None:
        _die(f"로그 변수 없음: {name}")
    if isinstance(x, np.ndarray) and x.dtype.names and \
            "time" in x.dtype.names:                    # StructureWithTime
        t = np.ravel(x["time"][0, 0])
        v = np.ravel(x["signals"][0, 0]["values"][0, 0])
        return t, v
    v = np.ravel(np.asarray(x, float))                  # Array + sim_time 동승
    t = np.ravel(np.asarray(sim_time, float))
    n = min(len(t), len(v))
    return t[:n], v[:n]


def _act_xyz(mat_path):
    md = loadmat(mat_path, squeeze_me=False)
    sim_time = md.get("sim_time")
    if sim_time is None:
        _die(f"sim_time 없음: {mat_path} (SaveFormat Array + Clock 동승 전제)")
    t, x = _series(md, "act_x1", sim_time)
    _, y = _series(md, "act_y1", sim_time)
    _, z = _series(md, "act_z1", sim_time)
    n = min(len(t), len(x), len(y), len(z))
    return t[:n], np.column_stack([x[:n], y[:n], z[:n]])


def _pipeline(verb_args):
    """traj_pipeline CLI 호출 -> (rc, 마지막 줄 JSON)."""
    proc = subprocess.run([PY, os.path.join(HERE, "traj_pipeline.py")]
                          + verb_args, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", timeout=300)
    lines = proc.stdout.strip().splitlines()
    msg = {}
    if lines:
        try:
            msg = json.loads(lines[-1])
        except json.JSONDecodeError:
            pass
    if proc.returncode != 0:
        _die(f"pipeline {verb_args[0]} rc={proc.returncode}: "
             f"{msg or proc.stdout[-500:]}")
    return msg


def _plan_and_fly(mission, tag):
    """미션 dict -> plan -> trajectory.mat 배치 -> MATLAB 비행."""
    mdir = os.path.join(OUT, tag)
    os.makedirs(mdir, exist_ok=True)
    mp = os.path.join(mdir, "mission.json")
    with open(mp, "w", encoding="utf-8") as f:
        json.dump(mission, f, ensure_ascii=False, indent=2)
    msg = _pipeline(["plan", "--input", mp, "--out-dir", mdir])
    shutil.copy(os.path.join(mdir, "trajectory.mat"),
                os.path.join(SUB, "trajectory.mat"))
    sim = _run_matlab_flight(tag)
    return mdir, msg, sim


# ---------------------------------------------------------------------------
# ① A-1 정지
# ---------------------------------------------------------------------------

def episode_a1():
    print("=== ① A-1 비상 정지 ===")
    mission = {"waypoints": [[0.0, 0.0, 2.0], [8.0, 0.0, 2.0]],
               "limits": {"v_max": 1.5, "a_max": 1.2, "j_max": 6.0,
                          "snap_max": 60.0}}
    mdir, _, sim_cruise = _plan_and_fly(mission, "a1_cruise")

    with open(os.path.join(mdir, "trajectory.json"), encoding="utf-8") as f:
        tj = json.load(f)
    t_ref = np.asarray(tj["t"], float)
    p_ref = np.asarray(tj["pos"], float)
    v_ref = np.gradient(p_ref, t_ref, axis=0)
    speed = np.linalg.norm(v_ref, axis=1)
    if speed.max() < 1.0:
        _die(f"순항 최고속 {speed.max():.2f} < 1.0 - 합격선 전제 미충족")
    tau = float(t_ref[int(np.argmax(speed))])   # 최고속 시점에 정지 명령
    print(f"τ = {tau:.2f}s (계획 속도 {speed.max():.2f}m/s)")

    # 실측 상태 채취 (§9: 비상은 기준 아닌 실측) — 순항 비행 로그에서
    t_a, pos_a = _act_xyz(sim_cruise)
    vel_a = np.gradient(pos_a, t_a, axis=0)
    acc_a = np.gradient(vel_a, t_a, axis=0)
    k = int(np.argmin(np.abs(t_a - tau)))
    # 미분 노이즈 완화: τ 주변 5샘플 평균
    s = slice(max(k - 2, 0), k + 3)
    st = {"timestamp": (datetime.now() + timedelta(seconds=60))
          .strftime("%Y-%m-%dT%H-%M-%S.%f")[:-3],   # 오케스트레이터 지연 버퍼
          "t_sim_s": float(t_a[k]),
          "pos": pos_a[k].tolist(),
          "vel": vel_a[s].mean(axis=0).tolist(),
          "acc": acc_a[s].mean(axis=0).tolist(),
          "att": {"roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 0.0},
          "ref_state": {"pos": p_ref[np.argmin(np.abs(t_ref - tau))].tolist(),
                        "vel": v_ref[np.argmin(np.abs(t_ref - tau))].tolist(),
                        "acc": [0, 0, 0], "traj_hash": tj["trajectory_hash"],
                        "t_on_traj_s": tau}}
    sdir = os.path.join(OUT, "a1_stop")
    os.makedirs(sdir, exist_ok=True)
    sp = os.path.join(sdir, "current_state.json")
    with open(sp, "w", encoding="utf-8") as f:
        json.dump(st, f)

    msg = _pipeline(["emergency", "--state", sp, "--out-dir", sdir,
                     "--hold-s", str(A1_HOLD_S)])
    em = msg["emergency"]
    p_stop = np.asarray(em["stop_point"], float)
    print(f"정지 궤적: 거리 {em['stop_dist_m']*100:.1f}cm, "
          f"제동 {em['stop_T_s']:.2f}s")

    # 합성 기준: 순항[0..τ) + 정지 궤적(τ..) — 컨트롤러는 통짜 궤적으로 비행
    with open(os.path.join(sdir, "trajectory.json"), encoding="utf-8") as f:
        sj = json.load(f)
    t_s = np.asarray(sj["t"], float) + tau
    p_s = np.asarray(sj["pos"], float)
    m_pre = t_ref < tau
    t_comp = np.concatenate([t_ref[m_pre], t_s])
    p_comp = np.vstack([p_ref[m_pre], p_s])
    yaw_comp = np.zeros(len(t_comp))
    savemat(os.path.join(SUB, "trajectory.mat"), {
        "timespot_spl": t_comp.reshape(-1, 1),
        "spline_data": p_comp,
        "spline_yaw": yaw_comp.reshape(-1, 1),
        "waypoints": np.vstack([p_ref[0], st["pos"], p_stop]),
        "jitter_delta": np.zeros_like(p_comp),
        "controller_profile": "precision"})
    sim_stop = _run_matlab_flight("a1_stop")

    # 판정: 오버슈트(정지점 지나 진행 방향 초과) + 래치 드리프트
    t_f, pos_f = _act_xyz(sim_stop)
    v0 = np.asarray(st["vel"], float)
    dire = v0 / np.linalg.norm(v0)
    after = t_f >= tau
    over = float(np.max((pos_f[after] - p_stop) @ dire))
    t_latch = tau + em["stop_T_s"] + 1.0        # 정지 후 1s 정착 마진
    hold = t_f >= t_latch
    drift = float(np.max(np.linalg.norm(pos_f[hold] - pos_f[hold][0], axis=1))
                  ) if hold.any() else float("nan")
    ok = over < A1_OVERSHOOT_MAX_M and drift < A1_DRIFT_MAX_M
    rep = {"episode": "A1_stop", "pass": bool(ok), "tau_s": tau,
           "v_at_stop_mps": float(np.linalg.norm(v0)),
           "overshoot_m": over, "overshoot_limit_m": A1_OVERSHOOT_MAX_M,
           "latch_drift_m": drift, "drift_limit_m": A1_DRIFT_MAX_M,
           "stop_dist_m": em["stop_dist_m"], "stop_T_s": em["stop_T_s"]}
    print(json.dumps(rep, ensure_ascii=False, indent=2))
    return rep


# ---------------------------------------------------------------------------
# ② A-2 회피
# ---------------------------------------------------------------------------

def episode_a2():
    print("=== ② A-2 금지 구역 회피 ===")
    ko = {"zones": [A2_ZONE], "inflate_m": A2_INFLATE}
    wp = np.array([[0.0, 0.0, 2.0], [8.0, 0.0, 2.0]])
    new_wp, moved = keep_out_avoid_waypoints(wp, ko)
    if not moved:
        _die("회피 재계획 미발동 - 시나리오 구역 설정 확인")
    print(f"회피 재계획: waypoint {len(wp)} -> {len(new_wp)}개")

    mission = {"waypoints": new_wp.tolist(),
               "limits": {"v_max": 1.2, "a_max": 1.0, "j_max": 6.0,
                          "snap_max": 60.0},
               "keep_out": ko}                   # 게이트가 전 샘플 재검사
    _, _, sim = _plan_and_fly(mission, "a2_avoid")

    t_f, pos_f = _act_xyz(sim)
    c, ki, _ = keep_out_clearance(pos_f, ko["zones"], ko["inflate_m"])
    ok = c >= 0.0
    rep = {"episode": "A2_avoid", "pass": bool(ok),
           "actual_min_clearance_m": float(c),
           "worst_t_s": float(t_f[ki]),
           "inflate_m": A2_INFLATE, "n_waypoints_avoided": int(len(new_wp))}
    print(json.dumps(rep, ensure_ascii=False, indent=2))
    return rep


def main():
    ap = argparse.ArgumentParser(description="비상 검증 (§9 의무 1·2편)")
    ap.add_argument("--only", choices=["a1", "a2"], default=None)
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    reps = []
    if args.only in (None, "a1"):
        reps.append(episode_a1())
    if args.only in (None, "a2"):
        reps.append(episode_a2())
    out = os.path.join(OUT, "verify_emergency_report.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"at": datetime.now().isoformat(), "episodes": reps},
                  f, ensure_ascii=False, indent=2)
    print(f"[save] {out}")
    if not all(r["pass"] for r in reps):
        sys.exit(2)


if __name__ == "__main__":
    main()
