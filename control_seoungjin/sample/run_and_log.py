"""
정해진 입력 형식(waypoints)을 받아서 Simulink(quadcopter_package_delivery)를
실행하고, 먹인 입력(feed)과 시뮬레이션 결과를 각각 CSV로 남긴다.

입력 형식
---------
waypoints : (N, 3) array-like  — 순서대로 지나갈 x, y, z 좌표
v_max, a_max, j_max, snap_max : 스칼라 또는 [x, y, z] — plan_waypoints 제약조건

실행 흐름
---------
1. path_time.plan_waypoints로 waypoints -> (time, pos, yaw) 궤적 생성
2. 입력 피드를 trajectory_feed.csv로 저장 (각 시간에 어떤 pos/yaw를 먹였는지 스냅샷)
3. trajectory.mat으로 MATLAB에 전달 (Maneuver Controller Lookup Table 입력 형식)
4. MATLAB 배치 모드로 controller/.../run_sample_sim.m 실행
   (run_sample_sim.m이 sim() 도중 새로 생긴 워크스페이스 변수를 sim_result.mat에 저장.
   여기엔 우리 파트의 최종 출력인 모터별 명령 motor_cmd_w1~w4(각속도 setpoint, rad/s)와
   그 시간축 sim_time도 포함됨 — 그 이후 모터/프로펠러 물리 응답은 Isaac Sim 쪽 역할)
5. sim_result.mat의 각 로그 변수를 sim_result_<변수명>.csv로 저장
6. motor_cmd_w1~w4 + sim_time을 Isaac Sim이 바로 읽을 프레임별 JSON(isaacsim_motor_commands.json)으로 저장

입력 설정 파일 (JSON 또는 YAML, config.yaml 참고)
---------------------------------------------
waypoints: [[x, y, z], ...]
limits: {v_max, a_max, j_max, snap_max}
dt: 출력 시간 간격 [s] (기본 0.01)

사용 예:
    python run_and_log.py                     # sample/config.yaml 사용
    python run_and_log.py --config my.json    # 다른 설정 파일 지정
"""

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys

import numpy as np
from scipy.io import loadmat

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from waypoints_to_maneuver_input import waypoints_to_maneuver_input, save_for_matlab  # noqa: E402

SAMPLE_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_DIR = os.path.dirname(SAMPLE_DIR)
MODEL_DIR = os.path.join(CONTROL_DIR, "controller", "Quadcopter-Drone-Model-Simscape")
OUTPUT_DIR = os.path.join(SAMPLE_DIR, "output")


def load_input_config(path):
    """정해진 입력 형식(JSON/YAML)을 읽어서 (waypoints, v_max, a_max, j_max, snap_max, dt)를 반환.

    형식:
        waypoints: [[x, y, z], ...]
        limits: {v_max, a_max, j_max, snap_max}
        dt: 0.01   (선택, 기본 0.01)
    """
    ext = os.path.splitext(path)[1].lower()
    with open(path, encoding="utf-8") as f:
        if ext == ".json":
            cfg = json.load(f)
        elif ext in (".yaml", ".yml"):
            import yaml
            cfg = yaml.safe_load(f)
        else:
            raise ValueError(f"지원하지 않는 설정 파일 형식: {ext} (.json/.yaml/.yml만 지원)")

    waypoints = np.asarray(cfg["waypoints"], dtype=float)
    limits = cfg["limits"]
    dt = cfg.get("dt", 0.01)
    return waypoints, limits["v_max"], limits["a_max"], limits["j_max"], limits["snap_max"], dt


def save_feed_csv(path, timespot_spl, spline_data, spline_yaw):
    """이 시간(time_s)에 이 위치/yaw(feed)를 PID 컨트롤러에 먹였다 -> 한 줄씩 기록."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "x_m", "y_m", "z_m", "yaw_rad"])
        for t, (x, y, z), yaw in zip(timespot_spl, spline_data, spline_yaw):
            w.writerow([f"{t:.4f}", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}", f"{yaw:.6f}"])
    print(f"[save_feed_csv] 저장 완료: {path} ({len(timespot_spl)}행)")


def save_feed_isaacsim_json(path, timespot_spl, spline_data, spline_yaw):
    """Isaac Sim에서 바로 읽어 쓸 수 있는 프레임별 pose 목록(JSON).

    형식 (아직 Isaac Sim 쪽 스키마가 정해지지 않아 가장 쉬운 형태로 결정):
        {
          "fps": ...,
          "frames": [
            {"time": t, "position": [x, y, z], "yaw_rad": yaw,
             "orientation_quat_wxyz": [w, x, y, z]},
            ...
          ]
        }

    orientation_quat_wxyz는 yaw만 반영한 쿼터니언(roll=pitch=0)이라
    Isaac Sim의 set_world_pose(position, orientation) 등에 바로 넣을 수 있다.
    """
    dt = float(timespot_spl[1] - timespot_spl[0]) if len(timespot_spl) > 1 else 0.0
    fps = 1.0 / dt if dt > 0 else 0.0

    frames = []
    for t, (x, y, z), yaw in zip(timespot_spl, spline_data, spline_yaw):
        half = yaw / 2.0
        quat_wxyz = [float(np.cos(half)), 0.0, 0.0, float(np.sin(half))]
        frames.append({
            "time": float(t),
            "position": [float(x), float(y), float(z)],
            "yaw_rad": float(yaw),
            "orientation_quat_wxyz": quat_wxyz,
        })

    with open(path, "w", encoding="utf-8") as f:
        json.dump({"fps": fps, "frames": frames}, f, indent=2)
    print(f"[save_feed_isaacsim_json] 저장 완료: {path} ({len(frames)}프레임)")


def find_matlab():
    """run_sample_sim.sh와 동일한 우선순위: MATLAB_EXE 환경변수 -> PATH -> 최신 설치버전."""
    env = os.environ.get("MATLAB_EXE")
    if env and os.path.isfile(env):
        return env

    which = shutil.which("matlab")
    if which:
        return which

    base = r"C:\Program Files\MATLAB"
    if os.path.isdir(base):
        versions = sorted(d for d in os.listdir(base) if d.startswith("R"))
        for v in reversed(versions):
            candidate = os.path.join(base, v, "bin", "matlab.exe")
            if os.path.isfile(candidate):
                return candidate

    raise FileNotFoundError(
        "MATLAB 실행파일을 찾을 수 없습니다. MATLAB_EXE 환경변수로 경로를 지정하세요.\n"
        r'예: set MATLAB_EXE=C:\Program Files\MATLAB\R2025b\bin\matlab.exe'
    )


def run_matlab_sim():
    matlab_exe = find_matlab()
    print(f"[run_matlab_sim] MATLAB: {matlab_exe}")
    result = subprocess.run(
        [matlab_exe, "-batch", "run_sample_sim"],
        cwd=MODEL_DIR,
        capture_output=True,
        text=True,
    )
    print(result.stdout)
    if result.returncode != 0:
        print(result.stderr)
        raise RuntimeError(f"MATLAB 실행 실패 (exit code {result.returncode})")


def save_motor_cmd_isaacsim_json(mat_path, path):
    """sim_result.mat의 모터별 명령(motor_cmd_w1~w4)을 Isaac Sim에서 바로 읽을 프레임별 JSON으로 저장.

    우리 파트의 출력 경계는 "각 모터에 들어가는 명령"(각속도 setpoint, rad/s)까지이고,
    그 이후(모터 전기/역학, 프로펠러 공력 등 실제 물리 응답)는 Isaac Sim 쪽에서 담당한다.

    motor_cmd_w1~w4는 To Workspace(Array 포맷)라 시간이 따로 없어서, run_sample_sim.m이
    같은 솔버 스텝마다 Clock으로 같이 로깅해둔 sim_time과 행 순서로 맞춰 짝짓는다.

    형식 (아직 Isaac Sim 쪽 스키마가 정해지지 않아 가장 쉬운 형태로 결정):
        {
          "fps": ...,
          "frames": [
            {"time": t, "motor_cmd_w": [w1, w2, w3, w4]},
            ...
          ]
        }
    """
    data = loadmat(mat_path)
    sim_time = np.asarray(data["sim_time"]).ravel()
    motor_keys = ["motor_cmd_w1", "motor_cmd_w2", "motor_cmd_w3", "motor_cmd_w4"]
    motor_cmds = np.column_stack([np.asarray(data[k]).ravel() for k in motor_keys])

    dt = np.median(np.diff(sim_time)) if len(sim_time) > 1 else 0.0
    fps = 1.0 / dt if dt > 0 else 0.0

    frames = []
    for t, w in zip(sim_time, motor_cmds):
        frames.append({"time": float(t), "motor_cmd_w": [float(x) for x in w]})

    with open(path, "w", encoding="utf-8") as f:
        json.dump({"fps": fps, "frames": frames}, f, indent=2)
    print(f"[save_motor_cmd_isaacsim_json] 저장 완료: {path} ({len(frames)}프레임)")


def save_result_csvs(mat_path, out_dir):
    """sim_result.mat 안의 각 로그 변수를 sim_result_<변수명>.csv로 저장."""
    os.makedirs(out_dir, exist_ok=True)
    data = loadmat(mat_path)
    saved = []
    for name, value in data.items():
        if name.startswith("__"):
            continue
        arr = np.asarray(value)
        if arr.ndim == 1:
            arr = arr.reshape(-1, 1)
        csv_path = os.path.join(out_dir, f"sim_result_{name}.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([f"col_{i}" for i in range(arr.shape[1])])
            w.writerows(arr.tolist())
        saved.append(csv_path)
    print(f"[save_result_csvs] 저장 완료: {len(saved)}개 파일 -> {out_dir}")
    return saved


def run_and_log(waypoints, v_max, a_max, j_max, snap_max, dt=0.01):
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    timespot_spl, spline_data, spline_yaw = waypoints_to_maneuver_input(
        waypoints, v_max, a_max, j_max, snap_max, dt=dt,
    )

    save_feed_csv(
        os.path.join(OUTPUT_DIR, "trajectory_feed.csv"),
        timespot_spl, spline_data, spline_yaw,
    )
    save_feed_isaacsim_json(
        os.path.join(OUTPUT_DIR, "isaacsim_trajectory.json"),
        timespot_spl, spline_data, spline_yaw,
    )

    trajectory_mat = os.path.join(MODEL_DIR, "trajectory.mat")
    save_for_matlab(trajectory_mat, timespot_spl, spline_data, spline_yaw, waypoints)

    run_matlab_sim()

    sim_result_mat = os.path.join(MODEL_DIR, "sim_result.mat")
    save_result_csvs(sim_result_mat, OUTPUT_DIR)
    save_motor_cmd_isaacsim_json(
        sim_result_mat,
        os.path.join(OUTPUT_DIR, "isaacsim_motor_commands.json"),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=os.path.join(SAMPLE_DIR, "config.yaml"),
        help="입력 설정 파일 경로 (.json/.yaml/.yml, 기본: sample/config.yaml)",
    )
    args = parser.parse_args()

    waypoints, v_max, a_max, j_max, snap_max, dt = load_input_config(args.config)
    print(f"[config] {args.config} 로드: waypoints {waypoints.shape[0]}개, "
          f"v_max={v_max}, a_max={a_max}, j_max={j_max}, snap_max={snap_max}, dt={dt}")

    run_and_log(waypoints, v_max, a_max, j_max, snap_max, dt=dt)
