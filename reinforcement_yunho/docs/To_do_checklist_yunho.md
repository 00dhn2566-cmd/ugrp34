# 조윤호 To-Do 체크리스트 (구체화)

> 2026학년도 UGRP · 현수 하중 드론 강건 통합 비행 제어 시스템 연구
> 역할: 시뮬레이션 환경 조성 + 강화학습 환경 조성 (데이터셋 생성 포함)
> 기준 문서: README.md, window_detection_spec_v0.2.md, state_window_interface_spec_v0_1.md
> ※ GPU 클러스터(40GB×20)에서 돌릴 대량 작업은 별도 문서 `gpu_jobs_yunho.md` 참조

---

## 0. 최우선 — 다른 팀원이 지금 대기 중인 소형 산출물

씬/카메라만 확정되면 각각 반나절 안에 끝나는 항목들. 아래 1~4번 대형 작업보다 먼저 처리.

- [ ] **§6 intrinsics 기입** → 받는 사람: 길남 · 태민
  - 시뮬 카메라 확정 후 fx, fy, cx, cy, 왜곡계수(통상 무왜곡)를 spec §6 표에 기입
  - 길남은 `vision/synth_intrinsics.yaml`의 숫자만 교체하면 됨, 태민은 VIO 3D 복원의 기준값으로 사용
- [ ] **시뮬 렌더 색 샘플 전달** → 받는 사람: 길남
  - 3색(red/green/blue) 창문이 §4.1 조명 랜덤화(밝기·방향) 하에서 렌더된 이미지 수십 장
  - 목적: §3.1 HSV 판정 구간의 실렌더 검증·미세조정 (`color_order.yaml` 갱신, spec §7 잔여 항목)
- [ ] **카메라·IMU 스펙 문서 전달** → 받는 사람: 태민 (OpenVINS 설정 YAML 입력값)
  - kalibr_imucam_chain.yaml용: intrinsics(fx,fy,cx,cy), resolution, T_imu_cam(4×4 — 카메라가 IMU 기준 어디에 어느 방향으로 붙었는지)
  - kalibr_imu_chain.yaml용: gyroscope noise_density / random_walk, accelerometer noise_density / random_walk, update_rate(예: 200Hz)
  - 선행 확인: Isaac Sim IMU 센서의 노이즈 파라미터를 어디서 설정/조회하는지 (노이즈 주입 설정 자체가 이 항목의 일부)
- [ ] **테스트용 시뮬 비행 데이터 1세트** → 받는 사람: 태민
  - 임의 비행 1회분: 카메라 이미지 시퀀스 + IMU 측정값 + groundtruth(실제 pose·가속도 정답지)
  - 타임스탬프는 §5와 동일하게 ns 단위, 카메라–IMU 동기 기준 명시

## 1. Isaac Sim 시뮬레이션 환경 구축

- [ ] **씬 구성: 창문(사각 개구부) 배치기**
  - §4.1 도메인 랜덤화 규격 준수: 씬당 1~5개 / 위치 랜덤 / 거리 근·중·원 균등 / 각도 ±60° / 조명 밝기·방향 랜덤 / 3색 균등
  - 창문 재질: §3.1 테이블 기준 채도 높은 원색 (시뮬 색과 판정 코드의 단일 기준)
- [ ] **드론 + 센서 세팅**
  - 카메라: 1280×720 RGB (§2 — 모든 좌표 기준 해상도)
  - IMU: update_rate 확정(예: 200Hz) + 노이즈 파라미터
- [ ] **성진(제어) 파트와의 물리 접점 확정**
  - 성진 쪽 출력 경계 = 모터별 각속도 setpoint(rad/s). 그 이후 모터/프로펠러 물리 응답은 Isaac Sim 담당
  - [ ] `isaacsim_motor_commands.json`({fps, frames:[{time, motor_cmd_w:[w1..w4]}]}) 수신·재생 확인
  - [ ] `isaacsim_trajectory.json`의 Isaac Sim 측 정식 입력 스키마 확정 → 성진에게 통보 (현재 성진이 임시 스키마 `{fps, frames:[{time, position, yaw_rad, orientation_quat_wxyz}]}`로 출력 중 — 스키마 확정되면 성진 쪽 변환 로직만 수정)

## 2. 데이터셋 자동 생성 (Replicator, spec §4)

- [ ] **Replicator 4-corner keypoint 추출 방법 확인** (spec §7, 데이터셋 착수의 선행 조건)
  - 기본 annotator 출력이 아닐 수 있음 → 창문 프레임의 3D 모서리 4점을 카메라로 투영하는 커스텀 writer 필요 가능성. 공식 문서 확인
- [ ] **1차 데이터셋 (정책 A, ~3,000장)**
  - 라벨 대상: 네 corner가 모두 화면 안에 완전히 보이는 창문만
  - 라벨 포맷(§4.3): YOLO-pose txt `<class> <cx> <cy> <w> <h> <u1> <v1> <vis1> ... <u4> <v4> <vis4>`
    - class = order_index (red=0, green=1, blue=2) / 좌표는 0~1 정규화
    - corner 순서 고정: 좌상→우상→우하→좌하 (창문 정면 기준 시계방향)
    - visibility 필드는 1차부터 포함(전부 1로 채움) — 2차 전환 시 포맷 변경 없음
  - 저장: `images/{train,val,test}` + `labels/{train,val,test}`, 분할 80/10/10
  - 검수: 길남의 토이셋(`make_toy_dataset.py` 산출물)과 포맷 대조 + 몇 장 시각화로 corner 순서·정규화 확인
  - 전달: 길남에게 (수령 즉시 `window_pose.yaml` path 기입 → 본 학습)
- [ ] **2차 데이터셋 (정책 C, visibility 도입)** — 1차 모델 동작 확인 후
  - 화면 밖/가림 corner를 추정 좌표 + visibility=0으로 라벨 (통과 직전 corner가 화면 밖으로 나가는 정상 상황 대응)

## 3. RL 훈련 환경 조성 (방학 checkpoint 핵심)

- [ ] **훈련 파이프라인 먼저 안정화** (README §7.5)
  - 환경 병렬화 · 로깅(학습 곡선) · 체크포인트 저장/재개 — 보상 고도화 전에 이것부터
  - 최소 기능 보상으로 end-to-end 학습이 도는 것 확인 → 이후 고도화
- [ ] **관측/행동 경계 구현** (README §7.3)
  - 정책 출력 = 웨이포인트/참조 궤적 (저수준은 PID 추종, end-to-end 방식은 이번 학기 범위 제외)
  - 관측 입력 정의는 길남의 `state_window_interface_spec_v0_1`(태민 출력 규격 후보안)과 연동 — 회의에서 확정 필요
  - 시뮬 치트(GT depth, GT pose 직접 관측) 사용 금지 (§7.1 — spec의 depth 배제 원칙과 동일)
- [ ] **보상 함수 초안 + 가중치 config 분리** (§7.2)
  - 항목: 창문 통과 보상 / 충돌 페널티 / progress 보상 / 자세·에너지 페널티
  - 가중치는 config 파일로 분리, 튜닝 이력 기록 (reward hacking 감시: 창문 근처 맴돌기, 통과 직전 회피)
  - ※ 보상 설계 담당은 미지정 상태(README 후속 조치) — 환경 쪽 구현은 윤호, 설계 확정은 회의 안건
- [ ] **도메인 랜덤화 목록화** (§7.1) — 동역학 파라미터·센서 노이즈·조명. 데이터셋 랜덤화 규격(§4.1)과 정합 유지
- [ ] **관측 노이즈 주입 학습 지원** (§7.6) — GT 관측으로 시작하되, 추정치 노이즈 주입 옵션을 환경에 마련
- [ ] **평가 프로토콜 구현** (§7.4)
  - 랜덤 씬 N개(창문 개수·배치·색 순서 랜덤) 자동 생성 규칙
  - 지표: 통과 성공률 · 충돌률 · 평균 통과 시간 / baseline(단순 웨이포인트 추종)과 동일 씬 세트 비교

## 4. 미결 · 회의 안건

- [ ] **궤적 생성(RL 경로계획) 실무 담당 확정** — 7/3 회의 기준 미정, "윤호 역할 안건"과 연동 (환경 조성은 윤호 확정, 정책 학습 실무는 별도 확정 필요)
- [x] **본 학습용 CUDA GPU 확보** — 윤호 GPU 환경(40GB×20)으로 해소. 작업 배분은 `gpu_jobs_yunho.md`

---

## 진행 순서 참고

**즉시 (대기자 있음)**: 0번 전체 — intrinsics·렌더 색 샘플(길남), 카메라/IMU 스펙·비행 데이터(태민)
**병렬 진행**: 1번(씬·센서) → 2번(Replicator keypoint 확인 → 1차 데이터셋) / 3번 파이프라인 안정화
**하류 의존**: 길남 본 학습 ← 2번 1차 데이터셋 / 태민 OpenVINS 튜닝 ← 0번 스펙+비행 데이터 / RL 스윕 ← 3번 파이프라인 검증

*작성일: 2026-07-04*
