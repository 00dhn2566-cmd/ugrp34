# 처음 받는 사람용 안내 (GETTING_STARTED)

이 저장소를 처음 clone/pull 받았을 때 읽는 문서. 프로젝트 자체(주제·목표·마일스톤)는
[README.md](README.md)에 있고, 여기는 **"받아서 뭘 어떻게 하면 되는지"**만 담는다.

---

## 0. 받자마자 주의할 것 3가지

1. **서브모듈을 통째로 init 하지 말 것.** 이 저장소엔 서브모듈이 3개 있는데
   (`simul/` Isaac Sim, `visual_imaging_taemin/openvins_source/`,
   `control_seoungjin/controller/Quadcopter-Drone-Model-Simscape/`), 이 중
   **Simscape 드론 모델은 절대 `git submodule update --init` 금지** — 우리 기체(FX450)
   CAD·튜닝이 그 폴더 위에 덮여 있어서 init 하면 MathWorks 원본으로 되돌아가 망가진다.
   그 폴더는 zip으로 따로 배포받는다 (성진에게 요청). Isaac Sim/OpenVINS 서브모듈은
   그 파트를 실제로 돌릴 사람만 개별 경로 지정으로 init 하면 된다.
2. **없는 파일이 있어도 에러가 아니다.** 데이터셋(EuRoC, YOLO 학습셋), 시뮬 결과물
   (`output/`, `.mat`/CSV), 학습 가중치는 전부 의도적으로 git 밖이다 (`.gitignore`).
   필요한 데이터는 각 파트 README의 안내대로 따로 받는다.
3. **문서·주석은 한국어가 기본.** 기존 문서를 고칠 때도 한국어로 맞춘다.

## 1. 폴더 지도 — 누가 어디서 뭘 하나

```
카메라/IMU → [비전: 창문 탐지] → [VIO: 상태추정·3D 복원] → [경로계획: RL] → [저수준 제어: PID] → 드론
```

| 폴더 | 담당 | 역할 | 지금 상태 (한 줄) |
|---|---|---|---|
| `overall_gilnam/` | 류길남 | 파이프라인 총괄 + 비전 (창문 4-corner 검출, 색 순서) | 모델 구조 확정·인터페이스 구현 완료, 실데이터셋 학습 대기 |
| `visual_imaging_taemin/` | 박태민 | VIO (OpenVINS) + 창문 3D 재구성 | WSL2 환경·재구성 노드 2종 완성, 카메라 파라미터 확정 대기 |
| `reinforcement_yunho/` | 조윤호 | Isaac Sim 환경·데이터셋 생성·RL 경로계획 | 씬/데이터셋/RL 스캐폴드 병합됨 (07-19), 환경 구축 진행 |
| `control_seoungjin/` | 박성진 | 궤적 생성 + PID 저수준 제어 + 비상 체계 | 파이프라인·튜닝·비상까지 동작, MATLAB 시뮬로 검증 진행 중 |
| `simul/` | (공용) | Isaac Sim 엔진 원본 (vendored) | 우리 코드 아님 — 손대지 않음 |

**전 파트 공통 계약 문서 2개** (충돌하면 이쪽이 이김):
- [window_detection_spec_v0.2.md](window_detection_spec_v0.2.md) — 비전↔VIO↔데이터셋 규격
  (좌표는 1280x720 픽셀 기준, 비전→VIO JSON 메시지 §5 등)
- [control_seoungjin/EXTERNAL_INTERFACE.md](control_seoungjin/EXTERNAL_INTERFACE.md) —
  제어 파트와 통신하는 법 (미션 JSON 주고 성적표 받기, 비상 명령, yaw 명령).
  제어 내부는 블랙박스로 취급하면 되고 이 문서만 알면 됨.

## 2. 파트별 진행 현황 (2026-07-19 기준)

### 비전 (길남)
- 창문 검출 = **YOLO-pose 4-corner keypoint** 방식으로 구조 확정 (7건 결정 기록:
  [model_decisions.md](overall_gilnam/vision/model_decisions.md)).
- 색 판정은 모델이 아니라 HSV 후처리 (`color_judge.py`, 기준값은 `color_order.yaml`).
- VIO로 보내는 메시지(§5)와, 모델 학습 전에 파이프라인을 검증할 **GT 라벨 스트림**
  (`gt_stream.py`)까지 구현 — 학습 루프는 합성 토이 데이터로 리허설 완료.
- 남은 것: Isaac Sim 실데이터셋 도착 → 본 학습.

### VIO (태민)
- OpenVINS(WSL2 + ROS2 Jazzy)에서 EuRoC 재생·평가 파이프라인 동작
  (명령 순서: [commands/](visual_imaging_taemin/commands/)).
- 창문 3D 재구성 노드 2종: 검출 스탠드인(`window_sim_node.py`) + 실제 재구성
  (`window_recon_node.py` — 시선 누적 최소자승 삼각측량).
- 남은 것: 카메라 내부/외부 파라미터(§6) 확정되면 하드코딩 플레이스홀더 교체.

### RL / Isaac Sim (윤호)
- `reinforcement_yunho/` 스캐폴드가 main에 병합됨 (씬·캘리브레이션·RL 환경·스크립트
  골격 — 상세는 [reinforcement_yunho/README.md](reinforcement_yunho/README.md)).
- 렌더링(데이터셋 생성)은 RTX 로컬/클라우드, 대규모 학습은 GPU 클러스터로 분리하는
  운영안 확정 ([gpu_jobs_yunho.md](reinforcement_yunho/docs/gpu_jobs_yunho.md)).
  Isaac Sim은 팀 노트북에서 안 돌아가므로 클라우드(Paperspace/RunPod) 사용.

### 제어 (성진)
- **궤적 파이프라인**: waypoint 집합 → 시간 부여(7차 다항식) → 물리 한계 성형 →
  짐 스윙 상쇄(ZVD) → 검증 게이트 → 컨트롤러 궤적. CLI 한 줄로 호출
  (`python traj_pipeline.py plan --input <mission.json>`), RL 학습 신호용 성적표
  회신 포함. yaw 4모드(진행방향/고정/주시/스캔)까지 구현.
- **PID 튜닝**: MATLAB 정답 플랜트(Simscape)에서 짐 질량 0~2kg 대응 게인 법칙 채택,
  대표 성적 추종 RMS ~1.3cm / 도착 후 잔류 진동 ~0°.
- **비상 체계**: 비행 감독자(모드 관리·명령 게이트) + 비상 정지 + 금지 구역
  회피까지 구현 (자세 상실 회생·추력 부족 강하는 진행 중).
- **C++ 이식**: 제어기 C++판이 Simulink와 골든 트레이스 대조 진행 중 (위치 체인 합격).
- 파이썬 테스트 169개 통과 상태. 제어와 엮이는 팀원은
  [EXTERNAL_INTERFACE.md](control_seoungjin/EXTERNAL_INTERFACE.md)만 읽으면 된다.

## 3. 내 파트별 "받고 나서 읽을 순서"

| 나는… | 읽을 순서 |
|---|---|
| 처음 온 사람 (파트 무관) | [README.md](README.md) → 이 문서 → [window_detection_spec_v0.2.md](window_detection_spec_v0.2.md) |
| 비전 쪽과 일함 | [overall_gilnam/README.md](overall_gilnam/README.md) → [vision/model_decisions.md](overall_gilnam/vision/model_decisions.md) |
| VIO 쪽과 일함 | [visual_imaging_taemin/README.md](visual_imaging_taemin/README.md) → [commands/README.md](visual_imaging_taemin/commands/README.md) |
| RL/Isaac Sim 쪽과 일함 | [reinforcement_yunho/README.md](reinforcement_yunho/README.md) → [docs/gpu_jobs_yunho.md](reinforcement_yunho/docs/gpu_jobs_yunho.md) |
| 제어에 미션/명령을 보냄 (RL·통합) | [control_seoungjin/EXTERNAL_INTERFACE.md](control_seoungjin/EXTERNAL_INTERFACE.md) 하나면 충분 |
| 제어 내부를 같이 개발함 | [control_seoungjin/README.md](control_seoungjin/README.md) → [INTERFACE_SPEC.md](control_seoungjin/INTERFACE_SPEC.md) → 각 STATUS 문서 |

## 4. 받은 김에 바로 돌려볼 수 있는 것

특별한 장비 없이 (MATLAB/GPU/WSL 불필요) 파이썬만으로:

```bash
# 비전: 단위 테스트 (색 판정·메시지 규격·GT 스트림)
cd overall_gilnam/vision
python -m pip install -r requirements.txt
python -m pytest tests/ -q

# 제어: 단위 테스트 169개 (numpy/scipy/matplotlib 필요)
cd control_seoungjin
python -m pytest tests/ -q

# 제어: 예시 미션으로 궤적 생성 (output/에 결과 생성)
python traj_pipeline.py plan --input input/example_mission.json
```

장비가 필요한 것: MATLAB R2026a(+Simscape 계열)는 **제어 시뮬 전용** — 다른 파트는
설치할 필요 없음. Isaac Sim은 RTX 머신/클라우드 전용. OpenVINS는 WSL2 + ROS2 Jazzy.

## 5. 같이 일하는 방식

- **작업은 자기 폴더 안에서.** 다른 파트 폴더나 공통 계약 문서(위 2개 + 각 SPEC)를
  바꿔야 하면 먼저 담당자와 협의 — 계약 문서의 코드/필드는 "추가는 자유, 의미 변경·
  삭제는 금지"가 원칙이다.
- 파트 간 주고받는 데이터가 생기면 **스키마부터 문서에 박고 코드는 그 다음** —
  지금까지 전 파트가 이 순서로 왔다.
- 궁금한 건 각 폴더 README → 그래도 안 풀리면 담당자에게. 제어 폴더 안의
  `SESSIONS_BOARD.md`/`*_STATUS.md`류는 제어 내부 작업 기록이니 몰라도 된다.
