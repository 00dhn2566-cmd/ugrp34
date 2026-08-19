# Gazebo 셋업 작업 로그

> 목적: Gazebo를 실제로 띄우기 전까지 작성하는 코드/설정 파일을 시간순으로 기록. Gazebo가 정상 구동되면(첫 예제 월드 실행 확인) 이 로그는 마감하고, 이후 진행 상황은 정식 문서(README/COMMANDS류)로 옮긴다.
> 실행 머신: RTX 5060 (i9-14900HX, 16GB RAM) — 이 노트북(Iris Xe + MX450)에서는 실행 안 함, 근거: [cloud_gpu_ssh_setup.md](cloud_gpu_ssh_setup.md) 참고 및 팀 결정(2026-07-14).

---

## 2026-07-14

- 이 컴퓨터에서 ROS2/Gazebo 설치 여부만 확인함 (WSL `Ubuntu-24.04`에 ROS 미설치 확인).
- 결정: Gazebo는 RTX 5060 머신에서 진행.
- **RTX 5060 머신용 설치 명령어 미리 작성 (아직 미실행 — 그 머신에서 그대로 붙여넣기)**.
  가정: 이 저장소의 다른 ROS2 작업(`visual_imaging_taemin`)과 동일하게 **Windows + WSL2(Ubuntu-24.04) + ROS2 Jazzy** 구성. Jazzy 짝 버전인 **Gazebo Harmonic**을 `ros_gz` 브리지 패키지로 같이 설치한다. (5060 머신이 실제로 네이티브 Linux면 WSL 관련 줄만 빼면 나머지는 동일)

  ```bash
  # 0) WSL2 + Ubuntu 24.04 (5060 머신에 WSL 자체가 없을 때만, PowerShell에서)
  wsl --install -d Ubuntu-24.04

  # --- 아래부터는 WSL Ubuntu-24.04 안에서 실행 ---

  # 1) locale
  sudo apt update && sudo apt install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  # 2) universe repo + 필수 툴
  sudo apt install -y software-properties-common curl
  sudo add-apt-repository universe

  # 3) ROS2 apt 저장소 등록
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  # 4) ROS2 Jazzy 설치 (desktop = rviz2, demo 등 포함)
  sudo apt update && sudo apt upgrade -y
  sudo apt install -y ros-jazzy-desktop

  # 5) Gazebo Harmonic + ROS2↔Gazebo 브리지
  sudo apt install -y ros-jazzy-ros-gz

  # 6) 빌드 툴 (OpenVINS 쪽 setup_1to5.sh와 동일 패턴)
  sudo apt install -y python3-colcon-common-extensions python3-rosdep
  sudo rosdep init 2>/dev/null; rosdep update

  # 7) 매 셸마다 자동 source
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  source /opt/ros/jazzy/setup.bash

  # 8) 설치 확인 (기본 예제 월드 실행)
  gz sim shapes.sdf
  ```

  8번(`gz sim shapes.sdf`)이 GUI로 뜨면 설치 완료로 간주하고 이 로그를 마감한다.
- 다음 할 일: 위 명령어를 RTX 5060 머신에서 실제로 실행 → 8번 확인 → 결과를 이 로그에 추가.

<!-- 이후 항목은 아래 형식으로 추가:

## YYYY-MM-DD

- 작성/수정한 파일: `경로/파일명` — 무엇을 했는지 한 줄
- 확인한 것: (실행 결과, 에러, 해결 방법)
- 다음 할 일:

-->
