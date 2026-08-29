# Isaac Sim 전환 체크리스트 — 무엇이 있고 무엇을 새로 만들어야 하는가

> 작성: 류길남 2026-08-29 (회의 안건 2-1 보조 자료). 대상: 윤호(시뮬) + 성진(제어 결합) + 태민(센서 입력).
> 기준: `reinforcement_yunho/` 스텁·스키마·계약(`CONVENTIONS.md`, `rl/window_env.py::IsaacSimBackend`, `calib/`, `interface/`), `sim/ISAAC_CLUSTER_NOTES.md`, `sim/ISAAC_SETUP.md`, `prototype_demo/`(PyBullet 참조 구현), `control_seoungjin/docs/DYNAMICS.md`.
> 상태 표기: ✅ 있음·검증됨 / ⚠ 있으나 미검증·부분 / ⛔ 없음·미결

## 0. 전제

- Isaac은 두 용도: **① 렌더(Replicator)** — 데이터셋·VIO 비행 데이터, **② 물리(PhysX)** — RL 롤아웃·제어 폐루프. 지금 막힌 건 ①의 RTX 렌더러(호스트 드라이버 595.x, NVIDIA 확인 버그)이고 ②는 렌더 없이도 가능하나, 카메라 없는 RL은 팀 관측 정의와 안 맞아 결국 둘 다 필요.
- 버전은 **Isaac Sim 5.1**(Blackwell 검증). `sim/isaac_replicator.py`는 4.5 기준이라 `omni.isaac.*`→`isaacsim.*` 모듈명 변경을 첫 실행 때 점검.
- **PyBullet 프로토타입(`prototype_demo/window_flight.py`)이 백엔드의 참조 구현이다** — 계획→궤적→성진 제어기(ctypes)→물리→관측→재계획 흐름이 이미 한 파일에 있으므로, Isaac 백엔드는 이 흐름에서 "물리·렌더" 부분만 갈아끼우는 식으로 설계한다.

## A. 인프라

| 요소 | 상태 | 비고 |
|---|---|---|
| RTX 계열 GPU + 호환 드라이버(580.65.06) | ⛔ | 클러스터 다운그레이드 대기 or 외부 GPU. **없으면 B~H 전부 불가** |
| 컨테이너 실행 경로(apptainer rootfs·EULA env·캐시 바인드) | ✅ | `ISAAC_CLUSTER_NOTES.md` turnkey 레시피 |
| headless 렌더 설정(EGL, renderer 문자열) | ⚠ | `"RaytracedLighting"` 철자 확인(`ISAAC_SETUP.md` "첫 실행 시 고칠 것" 1번) |
| 재현성(seed·config·commit hash 기록) | ✅ | RTX 렌더는 seed 고정해도 픽셀 단위 재현 불가 — 메타데이터 JSON이 진실원 |

## B. 씬·에셋 (USD)

| 요소 | 상태 | 비고 |
|---|---|---|
| 창문 배치 샘플러(1~5개·근/중/원·±60°·3색 균등) | ✅ | `sim/scene_gen.py` 순수 로직 |
| 창문 프림: **색 테두리 프레임 + 뚫린 개구부**(루트 spec §2.5 잠정 확정) — 렌더용 색 + 충돌 지오메트리 겸용 | ⛔ | 기존 설계는 "채운 색판"(thin box) — 채움은 색 판정·삼각측량을 무너뜨림(윤호 PyBullet 실측). 프레임 4변이 각각 collider여야 RL 충돌 판정 가능 |
| 창문 재질: `color_order.yaml` HSV 대역 안 채도 원색, 조명 랜덤화에 강건(emissive 혼합) | ⚠ | 실렌더 검증은 CPU 폴백(94.8%)에서만 |
| 텍스처 배경(VIO 특징점용, S<100 탈채도) | ⚠ | 절차 생성 `bg_noise.png` → 실제 USD 에셋 교체 예정 |
| 조명 랜덤화(밝기·방향) | ⚠ | 샘플러 있음. CPU 렌더러 키 불일치 버그(`brightness/direction`→`intensity/azimuth_rad`, 08-11 통보) 수정 여부 확인 |
| 창문-창문 가림 판정 → vis=0 라벨 | ⛔ | 잠정 확정 ①. 세그멘테이션 annotator 또는 occlusion query 필요 |
| 클러터 소품 | ⚠ | `--clutter` 있으나 "가려졌는데 라벨됨" 미해결 — 위 가림 판정과 함께 |

## C. 카메라

| 요소 | 상태 | 비고 |
|---|---|---|
| 1280×720, intrinsics 확정값 | ⛔ | 안건 2-4: 후보 fx=fy=763(HFOV 80°). 확정 즉시 `scripts/gen_intrinsics.py` → `synth_intrinsics.yaml` |
| USD focalLength/aperture ↔ fx 변환 검증 | ⚠ | 스크립트가 `fx_eff` 출력. `Camera.get_intrinsics_matrix()`로 교차검증 |
| USD↔CV 좌표 뒤집기(`diag(1,-1,-1)`) | ✅ | `common/geometry.py` + smoke test |
| 카메라 프레임 레이트(VIO ~20Hz / 탐지 2Hz as-built) | ⚠ | E의 스텝 스케줄에서 확정 |
| 왜곡 | — | 무왜곡(`distortion: []`), Isaac 기본 핀홀과 일치 |
| 모션 블러 | — | Isaac 기본 없음 → scan rate 시뮬 1.0 rad/s 근거. 실기 대비 시 옵션 |

## D. IMU

| 요소 | 상태 | 비고 |
|---|---|---|
| IMU 프림 부착 + 200Hz | ⛔ | |
| 노이즈 4개(가속/자이로 × density/random walk) | ⛔ | 숫자 미결 + **Isaac IMU 센서 내장 노이즈 유무 불확실** — 없다고 가정하고 `export_vio.py` 단계 후처리 주입, **같은 숫자를 `calib/kalibr_imu_chain.yaml`에 기입**(calib 주석 원칙). PyBullet `export_euroc.py`가 이미 노이즈 주입을 하므로 그 모델을 재사용 |
| 카메라–IMU extrinsics `T_cam_imu` | ⚠ | 루트 spec §6.1 잠정 확정: R_IC=[[0,0,1],[−1,0,0],[0,−1,0]], 병진 = 장착 오프셋(시뮬 기본 0), 태민 노드 상수가 단일 진실. 드론 USD의 카메라 프림을 이 값과 일치하게 배치(반대로 USD에서 읽어 채우지 말 것). Kalibr `T_cam_imu` vs 태민 역행렬 방향 주의 |
| 단일 클럭·int-ns 타임스탬프 | ✅ | 시뮬 시간→ns 변환은 한 곳에서만 |

## E. 드론 동역학·액추에이션 (가장 큰 신규 작업)

| 요소 | 상태 | 비고 |
|---|---|---|
| 기체 USD(FX450): 질량·관성·로터 4개 위치 | ⛔ | 파라미터 원천 = 성진 `DYNAMICS.md`(질량 2.27kg @1kg 적재, 유효 모멘트 암 0.0930m, CAD 암 배치 "+"형 −11.7° 회전, 개별 암 미식별). **PyBullet CF2X(27g)는 팀 기체가 아님** — Isaac 전환의 의미 절반이 여기 |
| 모터 모델: ω → 추력 kT·ω², 반토크 kQ·ω² 힘/토크 인가 | ⛔ | Isaac에 프로펠러 공력 내장 없음 → 매 물리 스텝 `apply_forces` 류로 직접. **단위 함정**: Simscape 로그는 **rpm**(호버 634rpm, kT=9.79), C++는 **rad/s**(Ct=0.1072 @634rad/s) — 추력은 같지만 회전속도가 다름(성진 08-18 실측). 어느 쪽 상수를 쓰는지 명시 |
| 믹서 부호표·로터 인덱스/CW·CCW 매핑 | ⛔ | 실측 확정 표: roll `[-1,+1,-1,+1]` / pitch `[+1,+1,-1,-1]` / yaw `[-1,+1,+1,-1]`, 회전방향 `[+,-,-,+]`(성진 08-18 저녁 정정본). C++ 표와 인덱스 치환만큼 어긋남 → **로터 인덱스→기하 매핑(윤호 결정)** 확정과 함께 대조 |
| 물리 스텝 스케줄: 제어 1kHz ↔ PhysX dt ↔ IMU 200Hz ↔ 렌더 20Hz | ⛔ | 물리 1kHz(또는 500Hz), IMU 5스텝마다, 카메라 50스텝마다 |
| 항력·외란(바람·펄스) + 도메인 랜덤화 훅 | ⚠ | `rl/domain_randomization.py` 샘플러 있음, Isaac 적용 코드 없음. 성진 외란 배터리(0.3N·m 펄스, 바람 5m/s)를 재현할 수 있어야 제어 검증 비교 가능 |
| 충돌 판정(프레임·벽·바닥) | ⛔ | contact report / contact sensor |
| 측정 지연 모델(`measAgeS`) | ⛔ | 성진 지연 강건화(08-23)가 요구 — 생산자 미정(안건 2-8). Isaac 백엔드가 채우기 자연스러움 |
| 현수하물 | 범위 밖 | 2학기 확장. joint+rigid body로 가능하나 지금 넣지 말 것 |

## F. 제어 결합 (Isaac ↔ 성진)

| 요소 | 상태 | 비고 |
|---|---|---|
| RL→제어 `waypoints_config` | ✅ | 코어/옵션 분리로 성진 확장 키와 호환(`HANDOFF_RL_SEAM.md`) |
| 제어→Isaac `isaacsim_motor_commands.json`(float 초, 4×rad/s) | ⚠ | 윤호 서명 대기 |
| 운반: 파일 재생 vs 실시간 | ⛔ | 1단계 파일 재생(오픈루프) → 폐루프는 PyBullet 프로토타입처럼 성진 C++ 제어기를 ctypes로 물리 루프 안에서 호출(검증된 방식) 또는 ROS2 |
| 드론 상태 피드백(`current_state.json` 30Hz) | ⚠ | 성진 생산자 있음, Isaac 채우는 쪽 없음 |
| `capability.json`(시계 배율) 소비 | ⛔ | 계획기(길남) 미연동 — Isaac과 무관하게 필요 |
| 감독자 하트비트·비상 명령 | ⚠ | 성진 규약 §9 존재, 연결 미착수 |

## G. RL 백엔드

| 요소 | 상태 | 비고 |
|---|---|---|
| `IsaacSimBackend(PhysicsBackend)`: `reset(start_pos, params)` / `step(waypoint_world, dt)` → `StepPhysics` | ⛔ | 인터페이스 고정 — 환경 코드 무수정 |
| 벡터화(num_envs) | ⛔ | `window_env.py` 단일 인스턴스 가정. Isaac 병렬은 사실상 **Isaac Lab** 방식(한 스테이지에 N 복제) — 직접 구현 부담 큼. PyBullet은 SubprocVecEnv로 해결 중 |
| 관측 17차원·GT 치트 금지·노이즈 주입 | ✅ | `state_window_adapter.py` — 단 corner→normal **폴백** 부호가 아직 `cross(c1−c0, c3−c0)`(확정 부호 반대, 08-11 요청). 명시 `normal` 필드가 있으면 무해 |
| 정책 `step` 파라미터 기록 | ⚠ | 학습 0.3 vs 기본 0.6 불일치로 성공률 95%→5% 오독 사례 — config에 박을 것 |
| 보상 가중치 | ⛔ | 스텁(안건 2-7) |

## H. 데이터 내보내기 (팀원 산출물)

| 요소 | 상태 | 비고 |
|---|---|---|
| 프레임 메타데이터 → `export_dataset.py` → YOLO-pose 17토큰 + `meta.jsonl` | ✅ | 오프라인 경로 검증됨 |
| Replicator 4-corner 커스텀 writer | ⚠ | 순수 로직 있음, Isaac 실행 미검증(spec §7) |
| 렌더↔라벨 정합(`visualize_labels.py`) | ✅ | |
| EuRoC-ASL 비행 bag(`export_vio.py`) | ⚠ | 작성기 있음, Isaac 비행·IMU 값 넣는 쪽 없음. PyBullet `export_euroc.py`가 선례 |
| §5 GT 스트림(`export_stream.py`, 길남 `gt_stream` 경유) | ✅ | |
| ROS2 브릿지(`/cam0/image_raw`, `/imu0`, odom) | ⛔ | 태민 실시간 구독 단계. 컨테이너 안 ROS2 Jazzy 정합 확인 |

## I. 검증 순서 (권장)

1. 렌더 5장 스모크 → `visualize_labels` 정합 → `color_judge` 통과율 (테두리 창문 기준)
2. 카메라 intrinsics 왕복(`fx_eff`, pose round-trip) ≈ 0
3. 호버: 모터 모델 + 성진 파일 재생(오픈루프) → Simscape 골든 트레이스와 대조(성진이 C++ 대조에 쓰는 방식, `compare_golden.py`)
4. IMU 정지 통계(Allan 편차)가 기입한 노이즈 값과 일치
5. 비행 1회분 EuRoC bag → 태민 OpenVINS ATE
6. `IsaacSimBackend` reset/step → 같은 씬에서 PyBullet 프로토타입 성공률과 비교 → 그 다음 학습

## J. 선행 결정 (안건 번호는 `overall_gilnam/docs/meeting_agenda_2026-08-29.md`)

- 2-1 시뮬레이터 역할 분담 — **PyBullet을 RL/계획 대리 환경으로 두고 Isaac을 렌더 전용으로 하면 E·F·G가 통째로 빠진다**(작업량 차이가 가장 큰 결정). 클러스터 GPU는 RTX PRO 6000(Blackwell) — 학습은 지금도 가능, RTX 렌더만 드라이버 차단
- 2-4 카메라 intrinsics·extrinsics·IMU 노이즈
- 2-8 로터 인덱스/CW·CCW 매핑, 모터 JSON 스키마 서명, `measAgeS` 생산자
- 병렬 RL을 Isaac Lab으로 갈지 단일 환경으로 갈지

## 참고 (확실도 표시)

- Isaac Sim 5.1 요구사항·지원 GPU: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html
- Replicator SDG·annotator·커스텀 writer: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html
- Isaac Sim ROS 2 튜토리얼(카메라·IMU·odom 퍼블리시): 문서 사이트 "ROS 2 Tutorials" 절 — 경로가 버전마다 달라 사이트에서 확인
- 쿼드로터 모터 모델·IMU 노이즈·ROS2를 Isaac 위에 구현한 참고: **Pegasus Simulator** — Jacinto, Pinto, Oliveira, Cunha, "Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation," ICUAS 2023, https://pegasussimulator.github.io (D·E 항목 시간 절약)
- 병렬 RL: Isaac Lab https://isaac-sim.github.io/IsaacLab (쿼드로터 예제 포함)
- **불확실**: Isaac 내장 IMU 센서의 노이즈 파라미터 유무는 버전별로 다르다고 기억함 — 확인 전엔 "없다"로 가정
