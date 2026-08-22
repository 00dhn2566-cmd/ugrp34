#!/usr/bin/env bash
# 공통 환경 로더. 실행 스크립트들이 source 합니다. 직접 실행하지 마세요.
#
# 순서가 중요합니다: ROS2 setup.bash 를 **먼저** source 해서 PYTHONPATH 를 잡고,
# 그 다음 venv 를 활성화합니다. venv 가 --system-site-packages 라 ROS2 의
# site-packages 가 그대로 보입니다. 반대로 하면 venv 가 경로를 가려 rclpy 를 놓칩니다.
#
# venv 가 없으면 conda 환경으로 넘어갑니다. rclpy(=ROS2)는 시스템 python 에
# 컴파일된 C 확장이라 conda 에서는 안 잡히므로, conda 로 떨어지면 HAVE_ROS=0 입니다.
# ROS 없이 도는 경로(offline 데모·GIF·비교)는 conda 로도 전부 돕니다.

VENV="${VENV:-$HOME/.venvs/ugrp}"
CONDA_ENV="${CONDA_ENV:-ugrp}"
PROTO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEAM="$(dirname "$PROTO")"

# ROS2 (있으면)
ROS_SETUP=""
for d in /opt/ros/*/setup.bash; do [[ -f "$d" ]] && ROS_SETUP="$d"; done
if [[ -n "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  HAVE_ROS=1
else
  HAVE_ROS=0
fi

if [[ -x "$VENV/bin/python" ]]; then
  # shellcheck disable=SC1091
  source "$VENV/bin/activate"
  PY="$VENV/bin/python"
  ENV_KIND="venv:$VENV"
else
  CONDA_PY=""
  for base in "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" "/opt/conda"; do
    [[ -x "$base/envs/$CONDA_ENV/bin/python" ]] && CONDA_PY="$base/envs/$CONDA_ENV/bin/python" && break
  done
  if [[ -z "$CONDA_PY" ]]; then
    echo "환경이 없습니다." >&2
    echo "  venv:  $VENV" >&2
    echo "  conda: $CONDA_ENV" >&2
    echo "  먼저:  bash $PROTO/scripts/setup.sh" >&2
    exit 1
  fi
  PY="$CONDA_PY"
  ENV_KIND="conda:$CONDA_ENV"
  if [[ "$HAVE_ROS" == "1" ]]; then
    # conda python 은 시스템 rclpy 를 import 할 수 없다 — ABI 가 다르다.
    HAVE_ROS=0
  fi
fi

export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"   # 렌더/추론 스레드 과다경합 방지
export PYTHONPATH="$TEAM/reinforcement_yunho:${PYTHONPATH:-}"
export PROTO TEAM PY HAVE_ROS ENV_KIND
