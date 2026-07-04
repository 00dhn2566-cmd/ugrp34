# PID 튜닝 / 모터 명령 파이프라인 진행 상황 (2026-07-03 기준)

## 완료된 것 — 입력 → 모터 명령 파이프라인 (동작함)

목표: waypoint 입력을 받아서 궤적 생성 → Simulink 시뮬레이션 → **각 모터에 들어가는 명령**(w1~w4, 각속도 setpoint)까지 뽑아서 Isaac Sim이 읽을 형식으로 출력. (그 이후 모터/프로펠러 물리 응답은 Isaac Sim 쪽 역할로 분리)

- `run_sample_sim.m`에 `Motor Mixer`(`Maneuver Controller/Motor Mixer`)의 `Add4/Add5/Add7/Add6` 출력(= w1/w2/w3/w4)을 `To Workspace`로 로깅하는 코드 추가. 배선은 안 건드리고 분기만 추가.
- 같은 솔버 스텝마다 시간을 찍는 `Sim Time Clock` + `To Workspace sim_time`도 추가 (Array 포맷 To Workspace는 시간이 안 붙어서 별도로 필요).
- `run_and_log.py`에 `save_motor_cmd_isaacsim_json()` 추가: `sim_time` + `motor_cmd_w1~w4`를 `{"fps":..., "frames":[{"time":t,"motor_cmd_w":[w1,w2,w3,w4]}, ...]}` 형식으로 `sample/output/isaacsim_motor_commands.json`에 저장.
- 전체 파이프라인(`python control_seoungjin/sample/run_and_log.py --config control_seoungjin/sample/config.yaml`) 엔드투엔드 테스트 완료, 정상적으로 6716프레임 JSON 생성 확인.

**주의**: 파이프라인 배선 자체는 정상 동작하지만, 지금 나오는 모터 명령 **값 자체는 신뢰할 수 없음** (아래 참고).

## 미해결 — PID 재튜닝 (`tune_pid.m`)

FX450 CAD로 교체하면서 질량/관성모멘트가 바뀌었는데 PID 게인은 원래 MathWorks 예제 값 그대로라서, 지금 나오는 `motor_cmd_w1~w4` 값이 비정상적으로 큼 (평균 3만~9만대 rad/s, 정상 범위는 수백 rad/s 수준이어야 함, 음수 역회전 값도 나옴). **PID 재튜닝을 마쳐야 모터 명령 값을 신뢰할 수 있음.**

### 오늘 순서대로 잡은 에러들 (전부 실제로 고쳐짐)

1. **"has one or more input signal ports with no explicit line connections"**
   `Control Pitch` 안의 `PID Compensator Formula`는 실제 PID 블록이 아니라 **비활성 상태인 내부 구현 variant**(포트 0개)였음. → 마스크 블록 자체(`Control Pitch` 등)를 타겟으로 바꿔서 해결.
2. **이산(z=0) 적분기 zoh 변환 에러**
   `ST.Options.RateConversionOptions.Method = 'tustin';`로 해결.
3. **Tracking 목표 "9 vs 35" 크기 불일치**
   `From`(전체 9채널 레퍼런스 버스) vs `Quadcopter`(전체 35채널 상태 버스)를 통째로 io 포인트로 잡아서 발생. → `Scope/Demux`의 outport 1/2/3(des_x/y/z) vs `Scope/In Bus Element,1,2`(act_x/y/z, `Chassis.px/py/pz`) 스칼라 3쌍으로 교체해서 해결.
4. **"조정 가능한 파라미터가 없음"**
   `Control Pitch`의 P/I/D 필드(`attitude_kp` 등)는 사실 **부모 마스크** `Altitude and  YPR Control`이 노출하는 마스크 파라미터였음 (`attitude_kp = kp_attitude`, `yaw_kp = kp_yaw`, `altitude_kp = kp_altitude` 식으로, pitch/roll이 `attitude_*` 게인 공유). → 개별 `Control Pitch/Roll/Thrust/Yaw` 대신 부모 `Altitude and  YPR Control` 자체를 타겟으로 바꿔서 해결.
5. **"waypoints를 찾을 수 없음" 계열 연쇄 에러**
   `sim()`은 스크립트의 base workspace를 그대로 쓰지만, `slTuner`의 내부 배치 선형화(`compileForLinearization`)는 별도 컨텍스트라 base workspace의 `waypoints`/`wayp_path_vis`를 못 찾음. → 모델의 **Model Workspace**에 `mws.assignin(...)`으로 직접 넣어서 해결.

### 아직 안 풀린 것 — "InputName은 채널마다 하나의 이름을 지정해야 함"

- `Altitude and  YPR Control` 블록을 **단독으로** 튜닝 대상으로 넣어도 동일 에러 발생 → 이 블록 자체에 내재된 문제 (다른 블록과 합쳐서 생기는 이름 충돌이 아님).
- `Altitude and  YPR Control`의 입력 5개/출력 4개 포트는 전부 signal name이 빈 문자열("")로 확인됨.
- `ST.Options.UseFullBlockNameLabels = 'on'`으로도 해결 안 됨.
- **포트 이름을 실제로 유일하게 바꿔봐도(`YPR_` 접두어) 똑같은 에러** → 포트 이름 충돌 가설은 반증됨.
- `Position Control/PID Controller`를 단독으로 넣으면 "조정 가능한 파라미터 없음" 에러. 실제로 확인해보니 이것도 `Control Pitch`처럼 부모 마스크(`Position Control`, `kp_xy2ypr` 등)의 파라미터를 참조하는 구조였음 — 부모로 타겟 변경 시 이번엔 **"InputName" 에러가 아니라 다시 "조정 가능한 파라미터 없음"**이 남음 (원인 불명, `Position Control`과 `Altitude and YPR Control`의 `MaskTunableValues`는 둘 다 전부 `on`으로 확인되어 이 차이는 아님).
- `Simulink.Parameter`로 게인 변수를 감싸서 `slTuner(mdl, mdl, io)`(모델 전체 자동탐색)도 시도 → "튜닝할 블록을 지정해야 한다"고 거부됨. 이 방식 자체가 안 먹힘.

### 2차 세션: `systune`/`slTuner`를 포기하고 `linearize()` + `pidtune()` 직접 시도

`slTuner`의 `TunedBlocks` 메커니즘이 이 모델의 버스 기반 마스크 서브시스템 구조와 근본적으로 안 맞는 것으로 보여, 더 단순한 API로 우회 시도:

- `linearize(mdl, io, snapshotTime)`으로 `Position Control/PID Controller`의 출력→실제 위치(`act_x`) 구간을 직접 선형화 → **에러 없이 성공**하지만 결과가 `1×3` 크기의 **완전히 0인 시스템**(`A=[]`, `D=[0 0 0]`, `nstates=0`)으로 나옴. `pidtune`이 "SISO 플랜트가 아니다"로 거부.
- 스냅샷 시간을 5/10/20/30초로 바꿔봐도 **전부 동일하게 0** → 특정 시점 문제 아님.
- 더 단순한 내부 경로(`Position Control`의 `Traj` 입력 → `Pitch Cmd` 출력, 한 서브시스템 안)로 좁혀도 **여전히 정확히 0**.
- **포트 번호가 잘못됐나 확인** — `Position Control` 외부 Inport 1=`m`(Chassis 상태 버스), Inport 2=`Traj`(pos/vel/yaw 레퍼런스 버스), Outport 1=`Pitch Cmd`(내부적으로 `Pitch Limit` 블록에서 나옴) 맞게 짚은 것으로 확인됨 → 포트 번호 문제 아님.
- **Block Reduction 최적화가 io 지점을 최적화로 없애버리나** 의심 → `set_param(mdl,'BlockReduction','off')` 해도 **동일하게 0**.
- **`Pitch Limit`(Saturate, `[-pi/3, pi/3]`)가 포화돼서 그런가** 의심 → 실제 로깅해보니 `pitch_cmd`는 한계값에 안 붙고 `0.857`(범위 안)로 **안정적으로 수렴**해있음 → 포화 가설도 반증됨.
- 위치+자세 게인을 둘 다 훨씬 낮춰(`kp_position=0.5`, `kp_attitude=5`) 재시도해도 **여전히 0** (t=5/15/25 전부).
- **현재 남은 가설**: `PID Controller` 내부의 `Reset Signal`(외부 리셋) 로직이 시뮬레이션 내내 켜져 있어서 출력이 고정(freeze)돼 있고, 그래서 입력을 흔들어도(perturbation) 국소 미분값이 0으로 나오는 것 아닌가 — **확인 안 됨, 다음에 이어서 볼 것**.

### 다음에 시도해볼 것

1. **최우선**: `PID Controller`(및 `Position Control`/`Altitude and YPR Control`) 내부의 `Reset Signal` 서브시스템이 실제로 리셋 상태(활성)인지 직접 로깅해서 확인. `ExternalReset` 마스크 파라미터가 `none`이 아니라면 이게 원인일 가능성.
2. 그래도 안 풀리면: 계속 blind 배치 스크립트로 파는 것보다 **MATLAB/Simulink GUI를 직접 열어서** 포트/신호를 시각적으로 확인하는 게 더 빠를 수 있음 (오늘 PID Tuner GUI 자체는 열려서 돌아갔지만, "폐루프 다시 선형화"도 결국 같은 이유로 막혔을 가능성 있음 — 위 원인이 GUI 쪽에도 똑같이 적용될 것으로 보임).
3. 위 문제가 풀려서 새 게인이 나오면: `quadcopter_package_parameters.m`에 반영하고, `run_and_log.py` 파이프라인을 다시 돌려서 `motor_cmd_w1~w4` 값이 정상 범위(수백 rad/s대)로 나오는지 확인.
4. (참고) F450/FX450 관련 인터넷 공개 PID 값은 찾아봤으나, 전부 다른 제어 구조/단위 체계(정규화된 RC 게인 등)라 이 모델에 직접 못 씀 — 참고용 스케일 감으로만 사용.

## 관련 파일

- `tune_pid.m` — 현재 WIP 상태, 위 5개 에러(1차 세션)는 이미 반영된 최신 코드. "InputName" 에러(6번째)에서 막힌 채로 남아있음 — 2차 세션은 `tune_pid.m` 자체는 안 건드리고 `diagnose/`의 별도 스크립트로만 진행함.
- `diagnose/` — 디버깅에 쓴 1회성 진단 스크립트 모음 (원인 규명에 실제로 도움 됨, `COMMANDS.md`에 1차 세션 스크립트 설명 있음. 2차 세션에서 추가된 스크립트: `diagnose_poscontrol_mask.m`, `diagnose_poscontrol_alone.m`, `diagnose_posgains.m`, `diagnose_mask_tunable_flag.m`, `diagnose_mass_check.m`, `diagnose_inertia_check.m`/`2.m`, `diagnose_arm_radius.m`/`2.m`, `diagnose_conservative_gains.m`, `diagnose_rename_ports_test.m`, `diagnose_simparam_tuning.m`, `diagnose_pidtune_position.m`, `diagnose_port_numbers.m`, `diagnose_port_elements.m`, `diagnose_pitch_limit_check.m`).
- `run_sample_sim.m` — `motor_cmd_w1~w4` + `sim_time` 로깅 추가된 최신 버전.
- `../../sample/run_and_log.py` — `isaacsim_motor_commands.json` 출력 추가된 최신 버전.
