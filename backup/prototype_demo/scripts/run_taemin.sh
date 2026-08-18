#!/usr/bin/env bash
# PyBullet 데모 -> 태민 window_recon_node.py (수정 0) -> 창문 3D
#
#   bash prototype_demo/scripts/run_taemin.sh          # offline (ROS2 불필요)
#   bash prototype_demo/scripts/run_taemin.sh --ros    # ROS2 발행 (노드는 별도 터미널)
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"
if [[ " $* " == *" --ros "* && "$HAVE_ROS" != "1" ]]; then
  echo "ROS2 가 없습니다. 먼저: bash $PROTO/scripts/install_ros2.sh" >&2; exit 1
fi
exec "$PY" "$PROTO/taemin_demo.py" "$@"
