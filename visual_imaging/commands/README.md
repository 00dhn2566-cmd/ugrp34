# Ubuntu 핵심 명령어 정리 (WSL2 + ROS2 Jazzy + OpenVINS)

## 1. WSL2 + Ubuntu 24.04 설치 (PowerShell)
```powershell
wsl --install -d Ubuntu-24.04
```

## 2. ROS2 Jazzy 설치
```bash
sudo apt install ros-jazzy-desktop
```

## 3. 필수 패키지 설치
```bash
sudo apt install -y git cmake python3-pip python3-colcon-common-extensions
sudo apt install -y libceres-dev
sudo apt install -y ros-jazzy-image-transport ros-jazzy-cv-bridge
sudo apt install -y ros-jazzy-message-filters ros-jazzy-tf2-ros
```

## 4. OpenVINS 클론 및 빌드
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/rpng/open_vins.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 5. ROS1 bag → ROS2 변환
```bash
pip install rosbags
rosbags-convert MH_01_easy.bag --dst ~/datasets/euroc/MH_01_easy_ros2
```

## 6. OpenVINS 실행

**터미널 1 - ROS2 환경 로드 및 OpenVINS 실행**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```

**터미널 2 - 데이터셋 재생**
```bash
ros2 bag play ~/datasets/euroc/MH_01_easy_ros2 --rate 0.5
```

**터미널 3 - 결과 확인**
```bash
ls ~/results/
```

## 7. 결과 파일 TUM 형식 변환
```bash
awk 'NR>1 {print $1, $6, $7, $8, $3, $4, $5, $2}' ~/results/state_estimate.txt \
  > ~/results/state_estimate_tum.txt
```

## 8. Ground Truth 변환
```bash
awk -F',' 'NR>1 {print $1/1e9, $2, $3, $4, $5, $6, $7, $8}' \
  ~/datasets/euroc/MH_01_easy_asl/mav0/state_groundtruth_estimate0/data.csv \
  > ~/results/groundtruth_tum.txt
```

## 9. ATE 평가 (evo)
```bash
pip install evo
evo_ape tum ~/results/groundtruth_tum.txt ~/results/state_estimate_tum.txt \
  -a --plot --save_results ~/results/ate_results.zip

evo_traj tum ~/results/state_estimate_tum.txt --save_plot ~/results/traj
```

## 10. 결과 파일을 Windows 쪽으로 복사
```bash
cp ~/results/pose_for_pid.txt /mnt/c/Users/parkb/OneDrive/Desktop/ugrp123/
```
