# Dataset

## EuRoC MAV Dataset

용량 문제로 데이터셋 파일을 직접 저장소에 올리지 않고, 다운로드 링크만 공유합니다.

- 공식 페이지: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
- 발행 기관: ETH Zurich, Autonomous Systems Lab (ASL)
- 내용: MAV(Micro Aerial Vehicle)에 탑재된 스테레오 카메라 + IMU로 수집한 visual-inertial 데이터셋 (Machine Hall, Vicon Room 시퀀스 등)

### 사용 방법
1. 위 링크에서 필요한 시퀀스(zip 또는 ASL dataset format)를 다운로드
2. 압축 해제 후 이 폴더(`dataset/`) 하위에 배치 (예: `dataset/MH_01_easy/`)
3. 실제 데이터 파일은 `.gitignore`에 등록하여 git에는 커밋하지 않는 것을 권장
