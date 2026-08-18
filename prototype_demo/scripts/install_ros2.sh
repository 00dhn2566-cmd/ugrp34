#!/usr/bin/env bash
# ROS2 Jazzy 설치 (Ubuntu 24.04 noble). **sudo 필요 — 이 스크립트만 sudo 를 씁니다.**
#
#   bash prototype_demo/scripts/install_ros2.sh
#
# 실시간 데모(태민 노드 연동)에만 필요합니다. 오프라인 데모만 쓸 거면 건너뛰어도 됩니다
# (module/contract.py 가 ROS 스텁으로 태민 노드를 그대로 구동합니다).
#
# ros-jazzy-desktop(2~3GB) 이 아니라 ros-base(~500MB) 를 깝니다 — 우리가 쓰는 건
# rclpy / std_msgs / geometry_msgs 뿐이고 RViz 같은 GUI 는 필요 없습니다.
set -euo pipefail

. /etc/os-release
if [[ "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "이 스크립트는 Ubuntu 24.04(noble) 기준입니다. 현재: ${PRETTY_NAME:-unknown}"
  echo "다른 배포판이면 https://docs.ros.org 에서 해당 ROS2 배포판을 설치하세요."
  exit 1
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  echo "이미 설치돼 있습니다: /opt/ros/jazzy"
  exit 0
fi

echo "==> 저장소 등록"
sudo apt update
sudo apt install -y software-properties-common curl gnupg
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "==> ros-jazzy-ros-base 설치"
sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-rclpy

echo "==> 확인"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
python3 -c "import rclpy, std_msgs.msg, geometry_msgs.msg; print('  rclpy OK')"

cat <<'EOF'

완료. 다음:
  bash prototype_demo/scripts/setup.sh
EOF
