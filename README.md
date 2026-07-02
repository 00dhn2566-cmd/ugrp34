# 현수 하중 드론의 강건 통합 비행 제어 시스템 연구

> 2026학년도 UGRP 연구과제 · DGIST · 장영실 코스
> 지도교수: 이성민 (로봇 및 기계공학부 조교수)

**국문 과제명**: 결함 허용 및 장애물 회피 기능을 갖춘 현수하물 탑재 드론의 강건 통합 비행 제어 시스템 연구
**영문 과제명**: A Study on Robust Integrated Flight Control System for Slung-load Drones with Fault Tolerance and Obstacle Avoidance Capabilities

---

## 1. 주제 (Topic)

비전 센서 기반 환경 인식과 드론 동역학을 결합해, 임의 위치에 놓인 창문(개구부)을 자율적으로 통과하는 드론 비행 제어 시스템을 연구한다. 창문들은 공간에 임의로 배치되며, 통과 순서는 색깔로 라벨링되어 미리 주어진다. IMU와 카메라 이미지를 융합해 드론이 제한된 공간을 안정적으로 통과하도록 하는 것이 핵심 문제다.

## 2. 목표 (Goals)

- **최종 목표**: 비전 기반 환경 인식 → 상태 추정 → 동역학 제약을 고려한 궤적 생성 → 저수준 제어로 이어지는 통합 파이프라인을 구성하고, 정해진 순서대로 창문을 안정적으로 통과하는 자율 비행 알고리즘을 개발한다.
- **이번 프로젝트의 실질 목표**: **Isaac Sim 시뮬레이션 환경**에서 위 파이프라인을 구축·검증하고, 강화학습 모델 훈련을 완료한다.
- **확장 목표 (여유 시)**: 개발된 알고리즘을 실제 하드웨어에 구현·검증한다. (2학기 이후)

## 3. 시스템 구조 (Pipeline)

전체 시스템은 아래 흐름으로 구성된다.

```
카메라/IMU → [비전: 창문 탐지] → [VIO: 상태추정·3D 복원] → [경로계획: ALM] → [저수준 제어: PID] → 드론
```

- **비전(이미지 처리기)**: 프레임에서 다음 통과 창문을 탐지하고, 색으로 순서를 식별하며, 네 모서리 픽셀 좌표를 산출.
- **VIO(상태추정)**: IMU + 카메라 융합으로 드론 pose를 추정하고, 비전 결과와 결합해 창문의 3D 위치를 복원.
- **경로계획**: Augmented Lagrangian Method(ALM)로 창문 통과·충돌 회피 제약을 만족하는 궤적 생성. *(초기 시도 후 엎음 → 재설계 예정)*
- **저수준 제어**: 관성 모멘트·추력·토크 모델링 기반 PID 제어기로 자세·위치 안정화. *(초기 시도 후 엎음 → 재설계 예정)*

## 4. 해야 할 일 (Tasks)

### 공통 / 인프라
- [ ] Isaac Sim 시뮬레이션 환경 구축 (창문 배치·드론·물리)
- [ ] 시뮬레이션 내 합성 데이터셋 자동 생성 (Replicator)
- [ ] 데이터셋·인터페이스 규격 문서 확정 (진행 중, v0.1 작성됨)
- [ ] 전체 파이프라인 통합 검증 (초기엔 ground-truth 값으로 흐름 확인)

### 비전 / 이미지 처리기
- [ ] 창문 검출 모델 방식 확정 → **4-corner keypoint 검출** (YOLO-pose 기반)
- [ ] 색 판정 (HSV 규칙 기반 후처리) 구현
- [ ] 데이터셋으로 검출 모델 학습 → corner 정밀화
- [ ] VIO 전달 규격에 맞춘 출력 인터페이스 구현

### 상태추정 (VIO)
1. 시물레이션에서 사용하는 카메라와 imu에 대한 아래의 정보 수령(from 윤호)
[OpenVINS 설정 YAML 입력값 정리]

① 카메라 스펙 (→ kalibr_imucam_chain.yaml)

- intrinsics (fx, fy): 초점거리 — 픽셀 이동과 실제 각도 간 환산 비율 (예: 400.0, 400.0)
- intrinsics (cx, cy): 이미지 중심점 — 렌즈가 똑바로 보는 픽셀 위치 (예: 320.0, 240.0)
- resolution: 이미지 크기 (예: 640 x 480)
- T_imu_cam: 카메라가 IMU에서 어느 위치에, 어느 방향을 보고 붙어 있는지 (4x4 변환행렬)


② IMU 스펙 (→ kalibr_imu_chain.yaml)

- gyroscope_noise_density: 자이로가 순간순간 얼마나 떨리는지
- gyroscope_random_walk: 자이로 영점이 시간이 지나며 얼마나 흘러가는지
- accelerometer_noise_density: 가속도계의 순간 떨림
- accelerometer_random_walk: 가속도계의 영점 흐름
- update_rate: IMU 주파수 (예: 200)

2. 테스트용으로 사용할 임의의 시물레이션 비행 데이터 수령(from 수령)
해당 비행에 대한 카메라 이미지 데이터 + imu 측정값 및 groundtruth(측정값이 아닌 실제 드론의 위치와 가속도에 대한 정답지)

3. 위의 내용들을 바탕으로 OpenVINS의 파라미터값들 조정 및 최적화

4. OpenVINS를 실시간 구독으로 바꾸어 제어기 파트에 신뢰도 높은 위치 및 가속도 값 제공 목표


### 경로계획 · 제어
- [ ] ALM 기반 궤적 생성 모듈 (초기 시도 후 엎음 → 재설계 예정)
- [ ] PID 저수준 제어기 (초기 시도 후 엎음 → 재설계 예정)

### 학습
- [ ] 강화학습 환경 조성 및 모델 훈련 (방학 중 checkpoint)

## 5. 역할 분담 (Roles)

| 이름 | 학번 | 역할 |
|---|---|---|
| 류길남 | 202111056 | 파이프라인 전반 설계·감독 + 이미지 처리 상용모델 조사·파인튜닝 |
| 박성진 | 202111068 | PID 제어기 설계 |
| 박태민 | 202211085 | VIO 관련 논문 다수 공부 |
| 조윤호 | 202211191 | 시뮬레이션 환경 조성 및 강화학습 환경 조성 (데이터셋 생성 포함) |

## 6. 마일스톤 (Milestones)

- **단기 (방학 checkpoint)**: 시뮬레이션 환경 구축 + 강화학습 모델 훈련 완료
- **장기 (2학기)**: 개발 알고리즘의 실제 하드웨어 구현·검증

## 7. 참고문헌 (References)

- Kaufmann, E., Bauersfeld, L., Loquercio, A., Müller, M., Koltun, V., & Scaramuzza, D. (2023). *Champion-level drone racing using deep reinforcement learning*. Nature, 620, 982–987.
- Ahmed, M. F., Zafar, M. N., & Mohanta, J. C. (2020). *Modeling and analysis of quadcopter F450 frame*. In 2020 International Conference on Contemporary Computing and Applications (IC3A) (pp. 196–201). IEEE.
- Bouabdallah, S. (2007). *Design and control of quadrotors with application to autonomous flying* (Thesis No. 3727) [Doctoral dissertation, École Polytechnique Fédérale de Lausanne].
