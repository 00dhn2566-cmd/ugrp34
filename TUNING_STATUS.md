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
- `Position Control/PID Controller`를 **단독으로** 튜닝 대상으로 넣으면 다른 에러("조정 가능한 파라미터 없음")가 남 → 이 블록도 `Control Pitch`처럼 부모 마스크(`Position Control`)의 파라미터를 참조하는 구조일 가능성이 높음 (아직 확인 안 함).

### 내일 시도해볼 것

1. `Position Control/PID Controller`도 `Control Pitch`와 같은 패턴인지 확인 — 부모 `Maneuver Controller/Position Control` 서브시스템 자체를 타겟으로 바꿔서 "조정 가능한 파라미터 없음" 에러가 없어지는지 테스트.
2. `Altitude and  YPR Control`의 "InputName" 충돌은 blind 배치 스크립트보다 **MATLAB/Simulink GUI를 직접 열어서** 포트/신호 이름을 시각적으로 확인하는 게 더 빠를 수 있음. 또는 MATLAB PID Tuner 앱(GUI)으로 개별 루프씩 접근하는 방법도 고려.
3. 대안: `systune` 대신 개별 루프별 `pidtune` (사용자가 처음에 명시적으로 제외했던 방식)로 우회하는 것도 옵션으로 남겨둠.
4. 위 문제가 풀려서 새 게인이 나오면: `tuned_pid.mat`의 값을 실제 `quadcopter_package_parameters.m`(또는 모델 파라미터)에 반영하고, `run_and_log.py` 파이프라인을 다시 돌려서 `motor_cmd_w1~w4` 값이 정상 범위(수백 rad/s대)로 나오는지 확인.

## 관련 파일

- `tune_pid.m` — 현재 WIP 상태, 위 5개 에러는 이미 반영된 최신 코드 (6번째 에러에서 막힘).
- `diagnose/` — 오늘 디버깅에 쓴 1회성 진단 스크립트 모음 (원인 규명에 실제로 도움 됨, `COMMANDS.md`에 각 스크립트 설명 있음).
- `run_sample_sim.m` — `motor_cmd_w1~w4` + `sim_time` 로깅 추가된 최신 버전.
- `../../sample/run_and_log.py` — `isaacsim_motor_commands.json` 출력 추가된 최신 버전.
