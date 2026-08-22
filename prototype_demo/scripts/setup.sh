#!/usr/bin/env bash
# 실행 환경 구성. 한 번만 돌리면 됩니다.
#
#   bash prototype_demo/scripts/setup.sh              # GPU 자동 감지
#   bash prototype_demo/scripts/setup.sh --cpu        # CPU 강제 (GPU 없는 노트북)
#   bash prototype_demo/scripts/setup.sh --gpu        # GPU 강제
#
# 왜 conda 가 아니라 venv 인가
# ---------------------------
# ROS2 의 rclpy 는 **시스템 python 에 컴파일된 C 확장**입니다. conda python 은 버전이
# 같아도 다른 바이너리라 import 하면 심볼이 안 맞습니다. 그래서 시스템 python 기반의
# `venv --system-site-packages` 를 씁니다 — ROS2 가 보이면서 우리 패키지도 깔립니다.
# ROS2 를 안 깔았어도 이 환경 하나로 오프라인 데모까지 다 돌아갑니다.
#
# torch 는 **맨 마지막에 버전 고정**으로 깝니다. 다른 패키지가 torch 를 덮어쓰면
# torchvision 과 짝이 깨져 "operator torchvision::nms does not exist" 로 죽습니다.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${VENV:-$HOME/.venvs/ugrp}"

MODE="auto"
case "${1:-}" in
  --cpu) MODE="cpu" ;;
  --gpu) MODE="gpu" ;;
  "")    MODE="auto" ;;
  *)     echo "사용법: setup.sh [--cpu|--gpu]"; exit 1 ;;
esac

if [[ "$MODE" == "auto" ]]; then
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
    MODE="gpu"; echo "==> GPU 감지: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"
  else
    MODE="cpu"; echo "==> GPU 없음 -> CPU 모드"
  fi
fi

# ---- ROS2 유무 (없어도 진행. 실시간 데모만 못 씀) --------------------------------
ROS_SETUP=""
for d in /opt/ros/*/setup.bash; do [[ -f "$d" ]] && ROS_SETUP="$d"; done
if [[ -n "$ROS_SETUP" ]]; then
  echo "==> ROS2 발견: $ROS_SETUP  (실시간 데모 사용 가능)"
else
  echo "==> ROS2 없음  (오프라인 데모만 사용 가능 — 실시간이 필요하면"
  echo "    먼저 bash prototype_demo/scripts/install_ros2.sh)"
fi

# ---- venv -----------------------------------------------------------------
echo "==> venv 생성: $VENV  (--system-site-packages)"
python3 -m venv --system-site-packages "$VENV"
PIP="$VENV/bin/pip"; PY="$VENV/bin/python"
"$PIP" install -q -U pip "setuptools>=83" wheel

echo "==> 1/2 패키지"
"$PIP" install -q numpy scipy pyyaml matplotlib tqdm pytest opencv-python-headless tensorboard
"$PIP" install -q pybullet gymnasium stable-baselines3 "ultralytics==8.4.87"
"$PIP" install -q "git+https://github.com/utiasDSL/gym-pybullet-drones.git"

echo "==> 2/2 torch ($MODE) — 마지막에 고정"
if [[ "$MODE" == "gpu" ]]; then
  "$PIP" install -q --force-reinstall \
    torch==2.13.0+cu126 torchvision==0.28.0+cu126 \
    --index-url https://download.pytorch.org/whl/cu126 \
    --extra-index-url https://pypi.org/simple
else
  "$PIP" install -q --force-reinstall torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu
fi

echo "==> 검증"
"$PIP" check || true
"$PY" - <<'PY'
import importlib, torch
for m in ("numpy","cv2","torch","torchvision","pybullet","gymnasium",
          "stable_baselines3","gym_pybullet_drones","ultralytics"):
    mod = importlib.import_module(m)
    print(f"  OK  {m:22s} {getattr(mod,'__version__','?')}")
import torchvision.ops          # 여기서 죽으면 torch/torchvision 짝이 깨진 것
print("  torchvision nms OK")
print("  cuda:", torch.cuda.is_available(),
      torch.cuda.get_device_name(0) if torch.cuda.is_available() else "(CPU 모드)")
PY

if [[ -n "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  ( source "$ROS_SETUP"; "$PY" -c "import rclpy; print('  rclpy OK (실시간 데모 가능)')" ) \
    || echo "  [주의] rclpy import 실패 — 실시간 데모는 못 씁니다 (오프라인은 정상)"
fi

cat <<EOF

완료.

  bash $HERE/scripts/run_pipeline.sh     # 이미지 -> 검출 -> 삼각측량 -> 웨이포인트
  bash $HERE/scripts/run_rl.sh           # 학습된 PPO 정책 비행
  bash $HERE/scripts/run_taemin.sh       # 태민 복원 노드 연동 (수정 없이)
EOF
