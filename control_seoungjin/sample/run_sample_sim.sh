#!/usr/bin/env bash
# 샘플 waypoint -> trajectory.mat 생성 -> Simscape 모델(quadcopter_package_delivery) 시뮬레이션 실행
# 실행: ./run_sample_sim.sh  (Git Bash)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_DIR="$(dirname "$SCRIPT_DIR")"
MODEL_DIR="$CONTROL_DIR/controller/Quadcopter-Drone-Model-Simscape"

# MATLAB 실행파일 찾기: 사람마다 설치 버전/위치가 다를 수 있어서 하드코딩하지 않는다.
# 1) 환경변수 MATLAB_EXE로 직접 지정 가능 (예: 여러 버전이 깔려 있을 때)
# 2) PATH에 matlab이 잡혀 있으면 그걸 사용
# 3) Windows 기본 설치 경로(C:\Program Files\MATLAB\R20XXx)에서 가장 최신 버전 자동 탐색
if [ -n "$MATLAB_EXE" ] && [ -x "$MATLAB_EXE" ]; then
    :
elif command -v matlab >/dev/null 2>&1; then
    MATLAB_EXE="$(command -v matlab)"
else
    MATLAB_EXE="$(ls -d "/c/Program Files/MATLAB/"R*/bin/matlab.exe 2>/dev/null | sort -V | tail -n 1)"
fi

if [ -z "$MATLAB_EXE" ] || [ ! -x "$MATLAB_EXE" ]; then
    echo "[오류] MATLAB 실행파일을 찾을 수 없습니다."
    echo "       MATLAB_EXE 환경변수로 경로를 직접 지정해주세요. 예:"
    echo "       MATLAB_EXE=\"/c/Program Files/MATLAB/R2025b/bin/matlab.exe\" ./run_sample_sim.sh"
    exit 1
fi
echo "[MATLAB] $MATLAB_EXE"

echo "[1/2] 샘플 trajectory.mat 생성 중..."
cd "$SCRIPT_DIR"
python3 waypoints_to_maneuver_input.py
cp trajectory.mat "$MODEL_DIR/trajectory.mat"
rm -f trajectory.mat

echo "[2/2] MATLAB에서 quadcopter_package_delivery 시뮬레이션 실행 중 (첫 실행은 Simscape 컴파일 때문에 오래 걸릴 수 있음)..."
"$MATLAB_EXE" -batch "run_sample_sim" -sd "$(cygpath -w "$MODEL_DIR")"

echo "완료. 결과: $MODEL_DIR/run_sample_sim_result.png"
