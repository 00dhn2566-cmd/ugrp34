#!/bin/bash
# README.md 1~5단계: WSL2/ROS2 설치 ~ OpenVINS 빌드 ~ bag 변환
set -e

# 1. WSL2 + Ubuntu 24.04 설치
#    이 명령은 Windows PowerShell에서 실행해야 합니다 (Ubuntu 내부 스크립트에서는 실행 불가):
#    wsl --install -d Ubuntu-24.04

# 2. ROS2 Jazzy 설치
sudo apt install ros-jazzy-desktop

# 3. 필수 패키지
sudo apt install -y git cmake python3-pip python3-colcon-common-extensions
sudo apt install -y libceres-dev
sudo apt install -y ros-jazzy-image-transport ros-jazzy-cv-bridge
sudo apt install -y ros-jazzy-message-filters ros-jazzy-tf2-ros

# 4. OpenVINS 클론 및 빌드
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/rpng/open_vins.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. ROS1 bag -> ROS2 변환
pip install rosbags
rosbags-convert MH_01_easy.bag --dst ~/datasets/euroc/MH_01_easy_ros2
