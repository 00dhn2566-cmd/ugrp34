# prototype — 노트북에서 도는 파이프라인 프로토타입

Isaac Sim 없이 **PyBullet** 위에서 파이프라인을 끝에서 끝까지 돌려 보는 작업 공간입니다.
팀 파일(`overall_gilnam/`, `control_seoungjin/`, `visual_imaging_taemin/`)은 **읽기만** 하고
수정하지 않습니다.

## 왜 이게 있나

지금까지 리포의 3D 복원·margin 수치는 전부 길남의 **합성 GT 코너 스트림 + 통계로 재현한
가짜 노이즈** 위에서 나왔습니다. 실제 검출 모델의 출력이 그 경로를 탄 적이 없었습니다
(`e2e_rehearsal.py` 의 입력이 이미 코너가 들어있는 `sample_stream.jsonl` 이라서).

여기서는 **진짜 이미지에서 시작합니다**: PyBullet 씬을 렌더 → 학습된 YOLO-pose 가중치로
코너 검출 → 그 결과를 팀의 `reconstruct_windows` → `assemble_window_map` →
`plan_waypoints` 에 그대로 흘려보냅니다. 하류 코드는 한 줄도 안 고쳤습니다.

## 빠른 시작

```bash
bash prototype_demo/scripts/setup.sh     # conda 환경 'ugrp' (GPU 없으면 --cpu)
conda activate ugrp
bash prototype_demo/control/build.sh     # 성진 제어기 ctypes 브리지 빌드
# prototype_demo/model/ 에 가중치 넣기 (model/README.md 참고)

cd prototype_demo
python window_flight.py                          # GT 계획으로 창문 통과 (제어 검증)
python window_flight.py --seq                    # 비전만으로 순차 통과 (GT 미사용)
python window_flight.py --n-windows 10 --merge-m 0.60   # 10창문
```

### window_flight.py 가 전부입니다

계획 → 궤적 → 성진 제어기 → PyBullet 물리 → (선택) 관측·복원·재계획까지 한 파일에서
돕니다. 주요 스위치:

| 스위치 | 하는 일 |
|---|---|
| `--seq` | 목표 색만 마스킹 + 같은 색 중 최대 박스 1개로 창문을 하나씩 비전으로 찾아 통과 |
| `--observe` | GT 계획으로 날면서 관측·복원 (복원 정확도 측정용) |
| `--replan` | 조금 날아 관측 → 비전으로 재계획 → 나머지 통과 |
| `--gif PATH` | 3인칭(`_3p`)·1인칭(`_fpv`) GIF 두 개 |
| `--export-euroc DIR` | EuRoC MAV(MH) 포맷 덤프. IMU 200 Hz + 카메라 20 Hz + GT |
| `--merge-m` | 궤적 웨이포인트 중복제거 반경. **창문이 많으면 0.6 필요** (아래) |

**`--merge-m` 주의.** 플래너 게이트 간격이 `2.5, 0.25, 2.5, 0.30 …` 으로 극단적으로
불균등한데, 성진의 7차 최소시간 다항식이 그 짧은 구간마다 오버슛했다가 되돌아온다.
창문 3개는 후퇴폭 1.12 m 로 버티지만 10개는 2.59 m 라 t=18 s 에 전복한다. 중복제거
반경을 0.60 m 로 올리면 그 짧은 구간이 사라져 후퇴폭이 0.70 m 로 내려가고 완주한다.

## 구조

```
prototype_demo/
├── config/camera.yaml     카메라 규격 (HFOV 80°, fx=fy=763) — 근거·대안·소비처 명시
├── model/                 가중치 (git 제외, model/README.md 참고)
├── utils/                 ★ 공통 유틸 (데모 스크립트에 복붙돼 있던 것들)
│   ├── paths.py           경로 상수 + sys.path 부트스트랩 + 가중치 해결
│   ├── device.py          GPU/CPU 선택, device 를 물고 있는 검출기 래퍼
│   ├── scene.py           env 생성(step=0.3 고정), 관측 경로 3종, GT 코너
│   ├── metrics.py         center/size 오차, 시드 집계, 색 혼동행렬
│   └── viz.py             팔레트 + 그림 헬퍼 (라벨 전부 영문 — claude.md 규칙 3)
├── overrides/             ★ 팀 코드를 우리 입맛대로 고친 버전 (원본은 불변)
│   ├── detections.py      §5 검출 스트림 후처리 (중복 표 제거, conf/크기 필터)
│   ├── recon_rays.py      태민 수치 경로 + conf 가중 + Huber IRLS
│   ├── frames.py          T_imu_cam 을 표준 전방 카메라 값으로 (OpenVINS 규약)
│   └── README.md          뭘 왜 바꿨는지 + 측정 결과
├── module/                태민 코드 연결 (그의 파일 무수정)
│   ├── contract.py        그의 상수를 AST 로 읽음 + ROS 스텁
│   └── taemin_bridge.py   observe / run_offline / run_ros_publisher
├── scripts/
│   ├── setup.sh           venv 환경 구성 (ROS2 rclpy 때문에 conda 아님)
│   ├── compare_weights.py 가중치 세대 비교 (v1 vs v2)
│   ├── compare_recon.py   복원 방식 비교 (태민 원본 vs overrides)
│   ├── sweep_layout.py    모터 배정 전수조사 (96조합 -> [3,2,1,0])
│   └── tune_gains.py      제어 게인 스윕
├── window_flight.py       **본체** — 계획→궤적→제어→물리→관측→재계획
├── planner.py             통과 후 거동까지 정하는 플래너 (길남 원본이 안 하는 부분)
├── traj.py                궤적 생성기 3종 (성진 flythrough 어댑터 / 자체 2종)
├── export_euroc.py        EuRoC MAV(MH) 포맷 덤프 (IMU·카메라·GT, 노이즈 포함)
└── out/                   산출물 (프레임·그림)
```

`utils/` 를 쓰는 쪽은 부트스트랩을 먼저 부릅니다:

```python
from utils import paths
paths.bootstrap()                       # 팀 폴더들을 sys.path 에
from utils import device, scene, metrics
```

실제 로직은 `reinforcement_yunho/` 에 있습니다:

| 모듈 | 역할 |
|---|---|
| `rl/domain.py` | PyBullet 씬을 학습 도메인에 정합. **학습 렌더러의 배경 생성기·색 테이블을 그대로 import** 해서 도메인이 어긋날 수 없게 함 |
| `sim/pybullet_stream.py` | 렌더 → YOLO → §5+pose 스트림. `noisy_stream.load_records` 가 먹는 형식과 동일 |
| `sim/pybullet_dataset.py` | 파인튜닝용 데이터셋 (GT 코너 투영 = 어노테이션 불필요, 3클래스) |
| `sim/finetune_pybullet.py` | `freeze=11` 파인튜닝 |
| `rl/pybullet_window_env.py` | 창문 통과 RL 환경 (gym-pybullet-drones, CF2X) |

## 알아둘 것 (실측으로 확인된 것들)

- **창문을 꽉 채우면 벽이 됩니다.** 원본 검출기가 "꽉 찬 색판"으로 학습돼서 PyBullet 창문도
  채웠더니, 앞 창문이 뒤 창문을 가려 `color_judge` 가 세 창문 전부 red 로 판정했고
  삼각측량이 서로 다른 창문을 하나로 융합했습니다 (복원 오차 207 mm). 그래서 창문을
  **뚫린 테두리로 되돌리고 모델 쪽을 파인튜닝**하는 방향으로 갔습니다.
- **RL 정책의 `step` 은 0.3 입니다.** `train_pybullet.py` 기본값은 0.6 이고, 기본값으로
  평가하면 성공률이 95% → 5% 로 보입니다. 학습에 쓴 값이 리포에 기록돼 있지 않았습니다.
- **PyBullet env 의 CF2X 는 팀 기체(FX450 2.27 kg)가 아닙니다.** 27 g 크레이지플라이라
  창문/기체 비율이 실제 임무보다 약 2배 헐렁하고, 현수 짐도 없습니다. 계획 계층 학습의
  대리 실험으로는 유효하지만 임무 자체의 대역은 아닙니다.
- **카메라 intrinsics 는 아직 잠정입니다** (`config/camera.yaml`, `status: provisional`).
  팀 전체가 쓰는 fx=600 은 길남 파일의 placeholder 이고 근거가 없습니다. 확정되면
  삼각측량 오차·margin·scan rate 를 전부 재계산해야 합니다 (학습 가중치는 영향 없음).
- **복원단에는 안전장치가 없습니다.** 태민 노드는 관측을 전부 동등 1표로 누적하고
  (`A += I − ddᵀ`, 계수 없음), conf 는 0.7 문턱 통과/탈락에만 쓰고 버립니다.
  아웃라이어 제거도 없어서 색이 한 번 잘못 붙으면 그 창문의 광선 4개가 통째로 다른
  바구니에 들어가 복원을 끌고 갑니다. `overrides/` 가 이걸 겨냥합니다 — 같은 관측에서
  center 오차 중앙값 268 → 63 mm. 자세한 건 [overrides/README.md](overrides/README.md).
- **학습 중에는 추론을 같이 돌리지 마세요. VRAM 이 아니라 시스템 RAM 이 터집니다.**
  WSL 에 할당된 건 7.8 GB 인데 (Windows 16 GB 의 절반), `--workers 2` 파인튜닝이
  부모 2.4 GB + 워커 1.2 GB × 6 개를 씁니다. 여기에 PyBullet+YOLO 프로세스(1.6 GB)를
  얹으면 free 가 100 MB 대로 떨어지고 **WSL 세션이 통째로 끊깁니다.** MX450 2 GB 는
  둘을 같이 물고도 멀쩡했습니다 (학습 1.0 G + 추론 0.11 G) — 병목은 RAM 쪽입니다.
  죽었으면 `finetune_pybullet.py --resume` 으로 `last.pt` 에서 이어서 돌리면 됩니다.
