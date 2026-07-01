#!/bin/bash
# README.md 6~10단계: OpenVINS 실행 ~ TUM 변환 ~ evo 평가 ~ 결과 복사
#
# 6단계는 터미널 3개에서 각각 따로 실행해야 하므로 이 스크립트로 자동화하지 않았습니다.
# 아래 순서대로 먼저 진행한 뒤, 이 스크립트(7~10단계)를 실행하세요.
#
#   터미널 1: source ~/ros2_ws/install/setup.bash && ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
#   터미널 2: ros2 bag play ~/datasets/euroc/MH_01_easy_ros2 --rate 0.5
#   터미널 3: ls ~/results/   (결과 파일 생성 확인용)
set -e

# 7. 결과 파일 TUM 형식 변환
awk 'NR>1 {print $1, $6, $7, $8, $3, $4, $5, $2}' ~/results/state_estimate.txt \
  > ~/results/state_estimate_tum.txt

# 8. Ground Truth 변환
awk -F',' 'NR>1 {print $1/1e9, $2, $3, $4, $5, $6, $7, $8}' \
  ~/datasets/euroc/MH_01_easy_asl/mav0/state_groundtruth_estimate0/data.csv \
  > ~/results/groundtruth_tum.txt

# 9. ATE 평가 (evo)
pip install evo
evo_ape tum ~/results/groundtruth_tum.txt ~/results/state_estimate_tum.txt \
  -a --plot --save_results ~/results/ate_results.zip

evo_traj tum ~/results/state_estimate_tum.txt --save_plot ~/results/traj

# 10. 결과 파일을 Windows 쪽으로 복사
cp ~/results/pose_for_pid.txt /mnt/c/Users/parkb/OneDrive/Desktop/ugrp123/
