# 류길남 To-Do 체크리스트

> 2026학년도 UGRP · 현수 하중 드론 강건 통합 비행 제어 시스템 연구
> 역할: 파이프라인 전반 설계·감독 + 이미지 처리(창문 탐지) 실무
> 기준 문서: README.md, window_detection_spec_v0.2.md

---

## 0. 최우선 — 다른 파트가 대기 중 (2026-08-01 풀 반영)

- [ ] **`scan.rate_rad_s` 산정 → 성진 회신** — yaw `scan` 모드의 회전 속도는 비전이 산정하는 필수 입력 (누락 시 미션 거부). 카메라 FOV·탐지 주기·모션 블러 기준으로 최대 스캔 속도 계산. 참고치: FOV 90°·탐지 10Hz면 1.0 rad/s에서 프레임당 ~6°. 제어는 물리 상한(잠정 1.0 rad/s)만 집행 (`control_seoungjin/EXTERNAL_INTERFACE.md` 3·6절)
- [ ] **중복 색 창문 평가 범위 답변 → 윤호** — 한 프레임에 같은 색 창문이 여러 개인 경우가 eval 범위인지 확정 (`meta.jsonl` 거리 기록 방식에 영향 — `reinforcement_yunho/sim/export_dataset.py` 주석의 질문)
- [ ] **폴백 데이터셋 선행 여부 결정 → 윤호 협의** — Isaac 렌더러가 클러스터 드라이버 버그로 차단(관리자 조치 대기, `sim/ISAAC_CLUSTER_NOTES.md`). CPU 절차적 렌더러로 1차 데이터셋 선행 생성 가능 — 방학 checkpoint 고려 시 폴백 선행 후 Isaac 데이터로 재학습이 유리

## 1. 검출 모델 개발 (핵심 실무)

- [ ] **YOLO-pose 기반 4-corner keypoint 검출 모델 구축**
  - 프레임 내 창문(사각 개구부) 검출 + 네 모서리 keypoint 추출
  - corner 순서 고정: 좌상 → 우상 → 우하 → 좌하 (창문 정면 기준, 시계방향)
- [x] **모델 입력 해상도 확정** → 640 확정 (vision/model_decisions.md #6)
  - 내부 리사이즈 해상도(예: 640) 결정 — 학습 시 확정 (규격 §7 잔여 사항)
  - 외부로 내보내는 좌표는 항상 원본 720p(1280x720) 기준 유지
- [ ] **1차 데이터셋으로 학습 (정책 A, 약 3,000장)**
  - 조윤호의 Isaac Sim 자동 생성 데이터셋 수령 후 진행
  - 분할 기준: train 80% / val 10% / test 10%
  - 라벨 포맷: YOLO-pose txt (`<class> <cx> <cy> <w> <h> <u1> <v1> <vis1> ...`)
  - **본 학습은 윤호 GPU 클러스터에 위탁** (`gpu_jobs_yunho.md` Job 1, 최우선 순위 규칙 있음) — 길남은 `window_pose.yaml`의 `path:` 기입 + `vision/` 폴더(window_pose.yaml, eval_corners.py, requirements.txt) 전달만. 반환물: best.pt + results.csv + eval_corners 표
- [ ] **corner 정밀화**
  - 학습 결과 확인 후 keypoint 좌표 정확도 개선
- [ ] **2차 데이터셋 대응 (정책 C, visibility 도입)**
  - 화면 밖/가림 corner를 추정 좌표 + visibility=0으로 처리하는 학습 확장
  - 목적: 창문 통과 직전 corner가 화면 밖으로 나가는 정상 상황 대응
  - 라벨 포맷은 1차부터 visibility 필드 포함이므로 포맷 변경 없음
- [ ] **추론 래퍼: 검출 결과 → color_judge → vision_msg 연결** — 학습 후
  - det_conf = 박스 conf, unknown 색 창문 처리 정책 확정 포함 — 드롭 권장
- [x] **ultralytics 버전 핀 고정** — ultralytics==8.4.87 (requirements.txt, 2026-07-04 리허설에서 확정)
- [x] **평가 스크립트: corner 픽셀 오차(720p) 거리 구간별 측정** — `vision/eval_corners.py` (테스트 포함, model_decisions #7 목표치 검증)

## 2. 색 판정 (통과 순서 식별)

- [x] **HSV 규칙 기반 색 판정 후처리 구현** — `vision/color_judge.py` (2026-07-03, 합성 이미지 테스트 통과)
  - red(order 0): H ∈ [0,10] ∪ [170,179], S ≥ 100, V ≥ 80
  - green(order 1): H ∈ [50,70], S ≥ 100, V ≥ 80
  - blue(order 2): H ∈ [110,130], S ≥ 100, V ≥ 80
- [x] **색↔순서 매핑을 config 파일로 분리** — `vision/color_order.yaml` (코드는 이 파일만 읽음, 하드코딩 없음)
- [ ] **HSV 판정 구간을 실제 시뮬 렌더 색으로 검증·미세조정** (규격 §7 잔여 사항)
  - 조윤호의 시뮬 환경 완성 후 진행
  - 조명 밝기·방향 랜덤화 하에서도 오판정이 없는지 확인

## 3. VIO 전달 인터페이스

- [x] **§5 메시지 규격에 맞춘 출력 구현** — `vision/vision_msg.py` (2026-07-03)
  - 포함 필드: timestamp(int, ns) / frame_id / windows 리스트
  - 창문별: order_index(필수) / color(디버깅용) / corners(720p 픽셀) / corner_vis / center / det_conf / color_conf
  - depth 미포함 — 3D 복원은 VIO/융합단(박태민) 몫
- [x] **det_conf와 color_conf 분리 산출** — color_conf는 `color_judge.py`가 산출(완료), det_conf는 모델 학습 후 박스 conf 연결
  - 탐지 신뢰도와 색 판정 신뢰도를 각각 제공 → 하류 필터링 가능하게
- [x] **GT 라벨(§4.3) → §5 어댑터 구현** (`vision/gt_stream.py`, 테스트 포함)
  - 모델 학습 완료 전 파이프라인 검증에는 시뮬 GT corner를 동일 규격으로 사용
  - 모델 교체 시 하류 수정이 없도록 인터페이스 일치 확인
- [x] **태민과 실스트림으로 인터페이스 검증** — 태민이 길남 합성 샘플 스트림(302프레임)으로 창문 3D 복원(삼각측량) 검증 성공 (07/04, corner 오차 0.01~0.07mm, 창문 크기 GT 일치 — `visual_imaging_taemin/README.md`)
- [ ] **실모델 출력으로 재검증** — 본 학습 후, GT 스트림 대신 실제 추론 출력(노이즈 포함)으로 태민 복원 정확도 재확인

## 4. 파이프라인 총괄 (설계·감독)

- [ ] **전체 파이프라인 설계·감독**
  - 카메라/IMU → 비전 → VIO → 경로계획(강화학습(RL), 담당 미정 — 추후 결정) → PID(재설계 예정) → 드론
  - 각 모듈 간 정합성 관리
- [ ] **규격 문서 관리**
  - window_detection_spec 유지·갱신 (현재 v0.2 확정본)
  - 변경 시 팀 전체 공유
- [ ] **전체 파이프라인 통합 검증 주도**
  - 초기: ground-truth 값으로 흐름 확인 → 단계적으로 실제 모듈 교체
- [ ] **팀원 산출물 접점 관리**
  - 조윤호: 데이터셋 생성(§2~4 준수), intrinsics 기입(§6), Replicator 4-corner 추출 방법 확인(§7)
  - 박태민: §5 규격으로 corner + pose 융합, 창문 3D 위치 복원
  - 박성진: PID 저수준 제어 (하류 연결) — 팀 공개 인터페이스는 `control_seoungjin/EXTERNAL_INTERFACE.md` 하나로 정리됨 (미션 JSON / yaw 4모드 / 비상 명령)
  - 궤적 생성(RL 경로계획): **담당 미정 — 추후 결정** (비전+VIO 산출물을 받아 궤적을 생성하는 역할, 7/3 회의 기준)
- [ ] **인터페이스 동결 관리 (2026-08-01 추가)** — 윤호 코드(`sim/export_stream.py` 등)가 `overall_gilnam/vision/`의 `gt_stream.py`·`vision_msg.py`를 **직접 import**하고, 17-토큰 라벨 포맷·intrinsics 스키마·투영 공식을 정확히 따름. 비전 코드 시그니처/포맷 변경 = 팀 계약 변경 → 사전 통보 필수
- [ ] **다음 회의 안건 (2026-08-01 추가)**
  - `state_window_interface_spec_v0_1` 확정 (윤호 `rl/state_window_adapter.py` 관측 정의와 연동 — 축 조합 바뀌면 어댑터 재검토)
  - 보상 함수 설계 담당 지정 (`rl/configs/reward_default.yaml` 스텁 상태)
  - RL 경로계획(궤적 생성) 실무 담당 지정
  - 클러스터 드라이버 다운그레이드(595→580) 진행 상황 팔로업 — 전체 일정 병목 (윤호가 관리자 요청, 총괄 확인)

## 5. 일정 연동 (마일스톤)

- [ ] **방학 checkpoint 대응**
  - 시뮬 환경 구축 + 강화학습 모델 훈련 완료 시점까지 비전 파트가 병목이 되지 않도록 1~3번 항목 완료
- [ ] **2학기 하드웨어 확장 대비**
  - 시뮬 전용 치트(depth 등)에 의존하지 않는 구조 유지 확인

---

## 진행 순서 참고

**선행 의존성이 있는 항목**
- 1번 학습 → 조윤호의 데이터셋 완성 이후
- 2번 HSV 검증 → 조윤호의 시뮬 렌더 색 확인 이후

**데이터 없이 즉시 진행 가능한 항목** → 전부 완료 (2026-07-03)
- 모델 구조 확정, config 설계, VIO 인터페이스 구현, GT 스트림 파이프라인 준비

**다음 즉시 진행 가능**: 합성 씬 생성기 (가상 창문 + 카메라 궤적 → §4.3 라벨 + GT pose → gt_stream으로 §5 스트림 생성) — 윤호 합류 전 태민 융합 착수용 → 완료 (2026-07-04)

*작성일: 2026-07-02*
*2026-07-03 갱신: 비전 코드 골격(색 판정·§5 빌더·GT 어댑터) 완료 반영.*
*2026-07-04 갱신: 합성 씬 생성기 + 태민용 샘플 스트림(sample_stream/) 커밋.*
*2026-07-04 갱신: 학습 리허설(토이 120장, yolo11n-pose 5에폭) 완료 — 학습 루프 검증. 평가 스크립트 커밋.*
*2026-08-01 갱신: 7월 팀 진행분 풀 반영 — 0번 신설(성진 scan 속도·윤호 질문·폴백 결정), 태민 삼각측량 검증 완료 처리, Job 1 위탁 체계·인터페이스 동결·회의 안건 추가.*
