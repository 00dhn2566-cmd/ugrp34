# rl_planner — 상태머신 임무 계획기를 대체하는 강화학습 계획기

> 작성: 2026-10-27 (윤호 요청, Claude 작업). 브랜치 `yunho/rl-planner` (main 미병합).
> 결과·학습 기록: [RESULTS.md](RESULTS.md) · 작업 로그(전체 IO 기록): `/data/worklog/ACTION_LOG.md`

## 1. 무엇을 바꿨나

| | 기존 | 이번 |
|---|---|---|
| 임무 계획 | `backup/prototype_demo/traj_manager.py` 상태머신 (SCAN → RECON → APPROACH → TRAVERSE → RESCAN) + `window_flight.py --seq` 프로브 규칙 | **학습된 정책** (PPO, rsl_rl) 이 10 Hz 로 다음 웨이포인트를 냄 |
| 정책 출력 | — | 현재 위치 + heading 프레임 변위 = 다음 웨이포인트 (README §7.3 "정책은 경로계획 계층만 대체") → 저수준 제어가 추종 |
| 정책 입력 | 스캔 앵커에 **GT 창문 중심** 사용(traj_manager) | VIO 상태 + IMU + **검출기 출력과 그 삼각측량 추정치만** (README §7.1 치트 금지) |
| 시뮬레이터 | PyBullet | **Isaac Sim 6.2 + Isaac Lab 3.0 (PhysX)**, 같은 MDP 의 순수 torch 백엔드 병행 |
| 기준선 | — | 상태머신을 **같은 행동공간·같은 인지 추정치**로 이식(`planners/fsm.py`) 해 같은 장면에서 비교 (README §7.4) |

상태머신이 하던 일(시차가 쌓일 때까지 비켜 보기 → 추정이 수렴하면 정렬 → 접근 → 통과 → 다음 창문)을
정책이 보상으로부터 스스로 배운다. 핵심 관찰: **접근 비행 자체가 시차를 준다** (traj_manager 주석: 통과 경로가
시차각 49~50° 를 줌) — 정책은 따로 멈춰서 스캔하지 않고 "보면서 다가가는" 경로를 학습할 수 있다.

## 2. 구조

```
rl_planner/
  configs/default.yaml      ← 모든 숫자의 단일 진실원 (기체·제어·씬·인지·보상·커리큘럼·DR·학습)
  core/                     ← 백엔드 무관 MDP (torch, Isaac 없이 테스트 가능)
    scene.py                  창문 배치 샘플러 (커리큘럼 레벨별 범위)
    perception.py             인지 대리 모델: 4-corner 검출기 + conf 가중 광선 삼각측량 (recon_rays 와 동일 수식)
    controller.py             Lee SE(3) 위치 제어기 + 모터 지연 (교체 가능: CONTROLLERS 레지스트리)
    dynamics.py               torch 6-DOF 강체 (torch 백엔드 물리)
    mission.py                심판: 통과/벽 충돌/순서 위반/역주행/이탈/전복 (GT, 관측 아님)
    observations.py           관측 항목 레지스트리 (정책용 / 비평가 특권용)
    rewards.py                보상 항목 레지스트리 (가중치는 yaml)
    env_core.py               위 모듈을 묶은 WindowMissionCore — 두 백엔드가 공유
  envs/
    isaaclab_env.py           Isaac Lab DirectRLEnv (PhysX 강체 + wrench composer). gym id: UGRP-WindowMission-Direct-v0
    torch_env.py              순수 torch rsl_rl VecEnv (Isaac 불필요)
  planners/
    fsm.py                    상태머신 기준선 (traj_manager 이식, 벡터화)
    policy.py                 학습된 정책 로더 (TorchScript + policy_meta.json)
    onboard.py                실제 파이프라인 연동 어댑터: §5 검출 + §6.1 VIO → 웨이포인트 + 성진 waypoints_config
    safety.py                 실행 시 안전 필터: 인지 상실 시 제자리 정지 비행 + 경계(가상 울타리) — 온보드 기본 적용
  scripts/train.py          rsl_rl PPO 학습 (비대칭 actor-critic, 커리큘럼, DR)
  scripts/evaluate.py       고정 시드 장면에서 RL vs 상태기계 (성공률·충돌률·시간·여유), 계획기 이름 뒤 +safe = 안전 필터
  eval_harness.py           평가 공통 루틴
  tests/                    단위 테스트 (팀 공용 기하 규약 일치 포함)
```

두 백엔드는 `WindowMissionCore` 를 공유하므로 **물리 적분만 다르고 MDP 는 같다**. 같은 시드·레벨·장면 수면 두 백엔드가
**같은 장면**을 받는다(장면 지문 일치). 검증: L3 64 장면에서 RL 완료시간 torch 8.280 s / Isaac 8.280 s,
상태기계 12.075 s / 12.070 s, 성공률 모두 100% (512 장면 교차 비교는 [RESULTS.md](RESULTS.md)).

## 3. 환경 (이 PC 기준, 2026-10-27 구축)

| 항목 | 값 |
|---|---|
| OS | Ubuntu 26.04.1 (커널 7.0) — Isaac Sim 공식 지원은 22.04/24.04, **실측으로 동작 확인** |
| GPU | RTX 4060 Ti 8 GB, 드라이버 595.99 (apt hold 로 고정) |
| Python 환경 | `/data/envs/isaac` (uv venv, Python 3.12.15) |
| Isaac Sim | 6.2.0.0 (pip, EULA 동의) — headless 기동 / PhysX / RTX 렌더 검증 (`/data/isaac/checks/`) |
| Isaac Lab | 3.0.0 (NVIDIA 공식 wheel, `--no-deps`) — 공식 Cartpole-Direct 256 env 동작 확인 |
| RL | rsl-rl-lib 5.5.1 (Isaac Lab 기본), torch 2.11.0+cu130 |

**Ubuntu 26.04 주의 1건**: Isaac 확장 하나가 `libxml2.so.2` 를 찾는데 26.04 에는 `.so.16` 만 있다. Ubuntu 24.04 공식
아카이브의 `libxml2`/`libicu74` 를 서명 검증 후 `/data/isaac/compat_libs` 에 **압축만 풀어** 두었다 (시스템 설치 아님).
Isaac 을 돌릴 때만 아래 환경변수를 준다.

```bash
export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y
export LD_LIBRARY_PATH=/data/isaac/compat_libs/root/usr/lib/x86_64-linux-gnu
PY=/data/envs/isaac/bin/python
cd reinforcement_yunho
```

다른 PC 에서 재현: `uv venv --python 3.12 env && pip install "isaacsim[all,extscache]==6.2.0.0" --extra-index-url https://pypi.nvidia.com`
→ `pip install --no-deps isaaclab==3.0.0 --extra-index-url https://pypi.nvidia.com` → `pip install rsl-rl-lib==5.5.1 gymnasium prettytable hydra-core h5py rich tqdm lazy_loader einops numba hidapi toml tensorboard`.
(isaaclab wheel 이 `warp-lang==1.17.0` 을 고정하지만 Isaac Sim 6.2 는 1.18.0 을 설치한다 — 1.18.0 그대로 동작 확인.)

## 4. 실행

```bash
$PY -m pytest rl_planner/tests -q                       # 단위 테스트 (CPU, 수 초)

# 학습 — Isaac (PhysX) / torch (같은 MDP, ~5배 빠름)
$PY rl_planner/scripts/train.py --backend isaac --num-envs 4096 --max-iterations 3000
$PY rl_planner/scripts/train.py --backend torch --num-envs 4096 --max-iterations 3000
#   실험 변형: --config my_exp.yaml (부분 yaml) / --set reward.crash=-30 (점 표기 덮어쓰기)
#   run 폴더: /data/runs/rl_planner/window_mission/<시각>_<백엔드>_<설정해시>_<git>/
#     config.yaml(병합 설정) · model_*.pt · tensorboard 로그 · export/{policy.pt, policy.onnx, policy_meta.json}

# 평가 — 같은 시드 장면에서 RL vs 상태기계 (레벨 3 = 창문 3개, 실측 수준 잡음)
$PY rl_planner/scripts/evaluate.py --backend isaac --planners fsm,rl --policy <run>/export --level 3
#   --planners fsm,rl,rl+safe   (+safe = 안전 필터 뒤에서 실행)   --config rl_planner/configs/stress/<조건>.yaml
tensorboard --logdir /data/runs/rl_planner
```

평가는 계획기마다 같은 시드로 난수를 다시 맞추고(`core.reseed`) 전체 리셋한다. 장면·도메인 랜덤화는 에피소드용 난수열,
카메라 잡음은 매 결정 고정 크기로 뽑는 별도 난수열이라, 각 환경의 첫 에피소드는 **모든 계획기에게 같은 실험**이다
(장면 지문을 `summary.json` 에 기록, 어긋나면 평가 중단). 그래서 결과 표에 같은 장면 짝 비교(McNemar)를 함께 낸다.

배포(실제 파이프라인): `planners/onboard.py` 의 `OnboardPlanner` 가 traj_manager 자리를 대신한다.
§5 검출 메시지와 VIO 자세(§6.1, XYZW)를 넣으면 학습 때와 **같은 코드**로 삼각측량해 관측을 만들고,
다음 웨이포인트와 성진 스키마 `waypoints_config` (`{waypoints:[현재, 다음], limits, dt}`) 를 돌려준다.
정책 출력은 **안전 필터**(`planners/safety.py`)를 거친다: 창문이 하나도 검출되지 않고 목표 창문을 모르는(추정이 없거나
σ > 0.3 m) 상태가 시작 후 0.5 s / 첫 검출 이후 3 s 를 넘으면 그 자리에 정지 비행하고(`hold: true`), 다시 보이면 정책에
돌려준다. 웨이포인트는 경기장 상자 안으로 제한한다. 배포 후보는 카메라 끊김 DR 로 미세조정한 정책 + 이 필터 (RESULTS.md §5).
실제 방 크기는 `OnboardPlanner(..., safety={"bounds": {"x_min": .., "y_abs": .., "z_min": .., "z_max": ..}})` (VIO 좌표).

## 5. MDP 정의 (설정: `configs/default.yaml`)

**결정 주기** 10 Hz (물리·제어 100 Hz × decimation 10). **에피소드** 최대 45 s.

**행동** (3): heading 프레임(yaw 만 반영, 중력 정렬) 변위 `[-1,1]³ × step_max=[0.8, 0.6, 0.4] m` → 웨이포인트 =
현재 위치 + 변위 → Lee 위치 제어기 추종. yaw 는 기본 고정(traj_manager `YAW_FIXED` 와 같은 결정). `action.yaw: true` 로 확장 가능.

**정책 관측** (41, 실기에서 얻을 수 있는 것만)

| 항목 | 차원 | 내용 |
|---|---|---|
| lin_vel_heading | 3 | VIO 속도 (heading 프레임) + 잡음 |
| gravity_body | 3 | IMU 중력 방향 |
| ang_vel_body | 3 | 자이로 |
| altitude | 1 | 고도 |
| last_action | 3 | 직전 행동 |
| target_estimate | 12 | 목표 창문 **삼각측량 추정**: 유효, 상대 중심(3), 법선(3), 크기(2), log σ, 시차각, 관측수 |
| target_detection | 6 | 이번 프레임 검출 여부, 검출 경과, 시선 방향(3), 겉보기 크기 |
| next_estimate | 7 | 다음 창문 추정: 유효, 상대 중심(3), 법선(3) |
| progress | 3 | 통과 비율, 남은 창문, 경과 시간 |

**비평가 추가 관측** (16, 학습 때만 — 비대칭 actor-critic): 목표·다음 창문 GT 상대 위치/법선/크기, 추정 오차, 시작점 대비 위치.

**보상** (가중치는 yaml; 구성은 드론 레이싱 RL 표준 — Song et al. 2021, Kaufmann et al. 2023 Swift)

| 항목 | 가중치 | 의미 |
|---|---|---|
| progress | 1.0 | 목표 창문 중심까지 거리 감소 [m] |
| window_pass | 10 | 목표 창문 통과 |
| pass_clearance | 5 | 통과 시 여유 / 최대 여유 (가운데 통과 = 1) |
| mission_success | 20 | 전 창문 통과 |
| crash | −20 | 벽·프레임 접촉, 지면, 이탈, 전복, 순서 위반, 역주행 (종료) |
| perception | 0.01 | 광축이 목표를 향함 exp(−δ⁴) |
| estimate_improve | 2.0 | 추정 오차 감소 (**potential-based** — 보상 해킹 불가) |
| time / action_rate / ang_vel | −0.02 / −0.02 / −0.005 | 시간, 행동 변화, 각속도 |

**커리큘럼** (환경별 레벨, 성공 +1 / 충돌 −1): L0 창문 1개·저잡음 → L3 창문 3개·실측 잡음(σ 2~9 px, 누락 10%) →
L4 창문 3~5개 + 창문 yaw ±20° → L5 스트레스(σ 4~12 px, 누락 5~20%, yaw ±30°, 간격 2.6~3.2 m).
`curriculum.fixed_level: <n>` 이면 커리큘럼 없이 한 레벨로만 학습.

**도메인 랜덤화**: 질량 ×[0.56,1] (짐 0~1 kg), 관성 ×[0.55,1] (제어기는 공칭 관성만 앎), 추력 ×[0.9,1.1], 항력,
모터 시정수 20~50 ms, 수평 바람 0~0.3 m/s², 관측 잡음, 검출 잡음·누락. 선택: 카메라 끊김
(`domain_randomization.blackout_prob`, 에피소드마다 0.5~4 s 구간 프레임 없음 — 기본 끔, 평가에서는 항상 끔).

**인지 대리 모델**: 코너 4개가 모두 화면 안 + 정면 방향(입사각 < 70°) + 다른 벽에 가리지 않음 + 짧은 변 ≥ 12 px 일 때
검출, conf 는 겉보기 크기·입사각의 함수. 코너 픽셀 잡음(실측 검출기 8.87 px 근거) → 시선 → `A += w(I−ddᵀ), b += w(I−ddᵀ)c`
최소자승 (태민 노드 / recon_rays 와 같은 수식), 시차 2° 게이트. σ 는 잔차 RMS 와 정보행렬 최소 고유값으로 계산 (GT 미사용).

## 6. 확장 (scalability) — 어디를 고치면 되나

| 하고 싶은 것 | 고칠 곳 |
|---|---|
| 보상 항목 추가 | `core/rewards.py` 에 `@register("name")` 함수 + yaml `reward.name: 가중치` |
| 관측 항목 추가/순서 변경 | `core/observations.py` `@register` + yaml `observation.policy` 목록 (export 메타에 레이아웃 자동 기록) |
| 난이도 축 추가 | yaml `curriculum.levels` 각 레벨에 키 추가 (`scene.LevelTable.KEYS` 에 등록) |
| 성진 PID(C++)로 저수준 교체 | `core/controller.py` 에 `Controller` 하위 클래스 + `CONTROLLERS` 등록 + `controller.type` |
| 실제 기체 USD (FX450) 사용 | `envs/isaaclab_env.py` 의 `WindowMissionSceneCfg.drone` spawn 교체 (힘은 wrench composer 로 그대로) |
| 대리 인지 → 실제 렌더 + YOLO | `PerceptionSurrogate.add_detections()` 가 실제 검출을 받는 입구. Isaac 카메라 + 길남 검출기 출력을 여기에 넣으면 됨 |
| yaw 제어 허용 | `action.yaw: true` (4차원 행동) |
| 다른 RL 라이브러리 | Isaac 백엔드는 표준 DirectRLEnv 라 Isaac Lab 의 skrl/rl_games/sb3 래퍼를 그대로 쓸 수 있음 |
| 클러스터(40GB×20)로 이전 | `train.py` 는 인자만 받음. `--num-envs` 를 키우고 `/data/envs/isaac` 와 같은 버전으로 설치 |
| 안전 규칙 추가/변경 | `planners/safety.py` `SafetyFilter.filter()` + yaml `safety` (학습·MDP 와 분리, 시뮬 평가 `+safe` 로 효과 측정) |
| 새 스트레스 조건 | `configs/stress/<이름>.yaml` (부분 설정) — 평가 매트릭스가 자동으로 포함 |

## 7. 가정과 한계 (정직하게)

- **인지는 대리 모델**이다. 검출 확률·잡음 모델은 실측 숫자(8.87 px 등)에 맞췄지만 실제 YOLO 의 실패 양상(배경 오검출,
  창문 혼동)은 없다. 창문 식별(색+순서)은 맞게 연결된다고 가정. → 다음 단계: Isaac RTX 카메라 + 실제 검출기로 평가.
- 창문은 **무한 벽 속 개구부**로 판정한다 (돌아가기 불가). 기체는 반경 0.30 m 구. 프레임 두께·부분 가림 없음.
- 저수준 제어는 Lee 기하 제어기다 (Isaac Lab 드론 예제와 같은 계열). 성진 PID 와 응답이 다르므로 실기 전환 전
  `controller.type` 교체 후 재학습/재평가 필요.
- 통과 카운트는 관측에서 "자기가 통과한 개수"로 쓴다 (실기에서는 추정 창문 평면 통과로 판단 — `onboard.py` 가 그렇게 함).
- 정책은 웨이포인트를 매 0.1 s 로 낸다. 성진 `waypoints_config` 에는 `[현재, 다음]` 2점으로 넘긴다 (리시딩 호라이즌).
- **카메라가 아무것도 못 보는 상황은 학습 분포 밖**이다. 이때 정책은 뒤로 물러나 경계를 벗어나고, 상태기계도 SEARCH 가
  계속 전진해 벽에 부딪힌다. 실행 시 안전 필터로 막는다 (RESULTS.md 안전 절). 근본 대책은 학습에 인지 상실 구간을 넣는 것.

## 8. 팀 결정이 필요한 것

1. 저수준 제어기: 학습에 성진 C++ PID 를 넣을지 (지금은 Lee). 넣으면 `controller.type` 만 바꿔 재학습.
2. 카메라 intrinsics 확정 (지금 fx=763, HFOV 80° 잠정) — 바뀌면 `perception.camera` 수정 후 재학습.
3. 창문 개수 상한(지금 5)·배치 분포(간격 2.6~3.6 m, yaw ±20°)가 실제 대회/실험 조건과 맞는지.
4. 안전 여유: 충돌 반경 0.30 m, 안전 여유 0.35 m (planner_limits) — 기체 실측으로 확정 필요.
5. 안전 필터: 정지 비행 전환 시간(지금 1 s), 실험실 경계 상자, 정지 비행 대신 착륙할지.
