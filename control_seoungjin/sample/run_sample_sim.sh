#!/usr/bin/env bash
# 샘플 waypoint -> trajectory.mat 생성 -> Simscape 모델(quadcopter_package_delivery) 시뮬레이션 실행
# 실행: ./run_sample_sim.sh  (Git Bash)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_DIR="$(dirname "$SCRIPT_DIR")"
MODEL_DIR="$CONTROL_DIR/controller/Quadcopter-Drone-Model-Simscape"
MATLAB_EXE="/c/Program Files/MATLAB/R2026a/bin/matlab.exe"

echo "[1/2] 샘플 trajectory.mat 생성 중..."
cd "$SCRIPT_DIR"
python3 waypoints_to_maneuver_input.py
cp trajectory.mat "$MODEL_DIR/trajectory.mat"
rm -f trajectory.mat

echo "[2/2] MATLAB에서 quadcopter_package_delivery 시뮬레이션 실행 중 (첫 실행은 Simscape 컴파일 때문에 오래 걸릴 수 있음)..."
"$MATLAB_EXE" -batch "run_sample_sim" -sd "$(cygpath -w "$MODEL_DIR")"

echo "완료. 결과: $MODEL_DIR/run_sample_sim_result.png"
