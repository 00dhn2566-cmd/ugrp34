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

### 3차 세션: GUI PID Tuner + 실제 호버 테스트로 진짜 근본 원인 확정

GUI PID Tuner(`Control Pitch`)로 게인을 조절해봤을 때 스텝 응답 자체는 그럴듯해 보였지만, 나온 파라미터가 `P=-5.68e10, I=-5.97e11, D=2.19e9` 같은 천문학적 값이었음 → 실제로 물리 시뮬레이션에 적용하면 바로 발산할 값들. 이걸 계기로 궤적 없이 **"제자리에서 1m 높이로 5초간 가만히 떠있기"**라는 아주 단순한 호버 테스트를 만들어서 원래(수정 안 한) 게인으로 돌려봄:

- **z(고도)**: 1m → -0.014m (추락), **y**: 0 → 2.77m (드리프트)
- **roll**: 478.96°, **pitch**: 478.96°, **yaw**: **425,135°** (5초 만에 1180바퀴 이상 회전!)

즉 원래 게인은 **궤적 추종은커녕 제자리 호버링조차 못 함** — 명백한 발산. 특히 yaw가 압도적으로 심각해서 원인을 추적:

1. **가설 1: 프로펠러 회전방향(CW/CCW) 오류.** 4개 프로펠러 `Aerodynamic Propeller.direction`이 전부 `Positive`로 확인됨(정상 쿼드콥터는 대각선 2쌍이 반대 방향이어야 반작용 토크가 상쇄됨). Motor Mixer의 yaw 부호 그룹(모터1&4 vs 2&3, `Out Bus Element` 매핑 기준)에 맞춰 프로펠러 2,3을 `Negative`로 바꿔서 재검증:
   - **X/Y 드리프트는 거의 완전히 해결됨** (y: 2.77m → 0.0016m) — 실질적 개선.
   - 하지만 **yaw 값은 수정 전후로 소수점까지 완전히 동일**(425135 vs 425135) → `direction` 파라미터가 애초에 yaw 반작용 토크 계산에 반영이 안 되는 구조로 보임 (추력 부호에만 영향). 이 가설은 yaw에 대해서는 반증됨.

2. **가설 2: 모터 전기 속도 서보(`Control1~4`, `kp_motor=0.00375`)가 너무 약해서 차등 속도 제어가 안 걸림.** 모터 1~4의 `ref`(명령)/`meas`(실측 속도)를 직접 로깅 → **4개 모터 전부 정확히 같은 속도(7420.01 rad/s)로 수렴**, 각자 다른 명령(10212~10807 rad/s)을 받았는데도 실측은 완전히 동일. `kp_motor`를 200배 키워도 **결과가 전혀 안 변함**(여전히 7420.01) → 게인 문제가 아님.

3. **진짜 원인 확정: 모터의 물리적 토크-속도(파워 리밋) 곡선.** `Motor 1~4`의 `w_t = [0, 3750, 7500, 8000]` rad/s (`torque_speed_param = torque_power`, `power_max=160W`, `torque_max=0.8N·m` 기준 파워리밋 토크-속도 곡선의 속도 구간). 즉 이 모터는 **물리적으로 최대 8000 rad/s까지만 낼 수 있는 스펙**인데, 상위 루프가 **10,000~11,000 rad/s**를 명령하고 있어서 4개 모터 전부 "낼 수 있는 한계"에 박혀 똑같은 평형 속도로 수렴 → 차등(요) 제어 권한이 물리적으로 사라짐. 사용자가 "상한이 7450쯤으로 정의된 거 아니냐"고 짚은 게 정확히 이 현상(리터럴 Saturation 블록은 아니지만, 파워리밋 토크-속도 곡선이 사실상 같은 역할).

   **결론: 오늘 전체의 근본 원인은 결국 원래 목표였던 "바깥쪽(자세/위치) PID가 FX450 동역학에 안 맞아서 말도 안 되는 속도를 명령한다"로 귀결됨.** 프로펠러 방향/모터 서보는 부수적인 문제였고(X/Y 개선에는 실질적으로 기여), yaw 폭주의 핵심은 모터가 애초에 낼 수 없는 속도를 계속 요구받고 있었다는 것.

4. **부가 발견: 모터 스펙(`w_t` 최대 8000 rad/s)도 FX450 실물 기준으로 보면 이미 비현실적으로 높음.** FX450은 보통 **A2212 1000KV 모터 + 3S(11.1V) 배터리** 조합(무부하 최대 rpm = KV×V = 1000×11.1 ≈ 11,100rpm ≈ **1,162 rad/s**)을 씀 — 출처: [Robocraze F450 kit](https://robocraze.com/products/f450-quadcopter-frame-kit-with-a2212-kv1000-brushless-motor-and-4-30a-esc-and-2-pair-1045-propeller), [Amazon QWinOut F450 kit (A2212 1000KV)](https://www.amazon.com/QWinOut-Airframe-Quadcopter-Brushless-Propellers/dp/B08GX5Z4SN).

5. **`w_t`/`T_t`/`w_eff_vec`는 사실 죽은 파라미터였음(중요, 시간 많이 씀).** `Motor 1~4`의 `w_t`를 8000→1162.4 rad/s(FX450 실제 스펙)로 스케일링해서 재테스트했는데 **결과가 수정 전과 소수점까지 완전히 동일**(포화 속도 여전히 7420.01). `set_param` 자체는 성공(즉시 다시 읽으면 새 값 확인됨)했는데도 시뮬레이션 결과가 전혀 안 바뀜 → `torque_speed_param = ee.enum.electromech.envelope.torque_power` 모드에서는 **`w_t`/`T_t` 배열이 아예 안 쓰이고**, 대신 `qc_motor.max_power`(`power_max`)/`qc_motor.max_torque`(`trq_max`) 스칼라로 토크-속도 한계가 계산됨. `qc_motor.max_power`를 절반(160→80W)으로 줄여서 검증 → **평형 속도가 실제로 7420.01 → 5869.63으로 바뀜** (진짜 활성 파라미터 확인됨).

   **최종 결론**: 모터 속도 한계를 실제로 좌우하는 건 `qc_motor.max_power`/`max_torque`(`Electrical` 마스크의 `power_max`/`trq_max`)이고, `w_t`/`T_t`/`w_eff_vec`는 현재 설정에서 무시됨. 다만 `power_max`를 낮춰도(80W) 4개 모터가 **여전히 전부 같은 값**으로 수렴함 — 상위 PID가 여전히 1만+ rad/s를 명령하는 한, 모터 스펙만 realistic하게 낮춰봤자 "다 같이 새 한계에 박히는" 현상 자체(=차등/요 제어 상실)는 해결 안 됨. **모터 스펙 교정 + 상위 PID 재튜닝을 함께 해야 함.**

### 4차 세션: "게인을 아무리 바꿔도 yaw가 안 바뀐다" → 필터 대역폭/물리적 원인으로 재추적

`kp_attitude=5`(원래의 1/25 수준)까지 낮춰도 yaw는 여전히 369,440°(거의 그대로) — **yaw 폭주가 게인 크기와 완전히 무관하다는 게 재확인됨** (반면 roll/pitch는 초반에 한 번 크게 튄 뒤 안정화됨, 즉 실제로 게인에 반응함 — yaw만 다름).

- 프로펠러 2,3 방향 수정 + 낮은 게인 조합으로 재검증: **X/Y는 다시 거의 완벽**(거의 0 유지), **모터1&4 vs 2&3이 이번엔 실제로 살짝 다른 값**(6398.37/6379.31 vs 6398.54/6379.47 — 전처럼 4개가 완전 동일하지 않음, 차등 제어가 조금은 걸림). 하지만 **yaw는 여전히 365,507°로 초기 상태와 거의 동일**.
- **`Control Yaw`의 입력(오차 e)을 직접 로깅 → 사실상 항상 0** (~1e-8 수준), 그런데 실제 yaw는 365,507°까지 감. 게인×0=0이니 게인을 뭘 넣어도 안 바뀌는 이유가 설명됨.
- 오차 계산 체인 추적: `Add3 = Yaw Cmd(+) - Filter Yaw(-)`. `Yaw Cmd`는 `Position Control`의 **Traj(레퍼런스) 버스 Port 2**에서 온 값(우리가 고정한 `spline_yaw=0`) — 즉 목표값 자체는 정상. 문제는 `Filter Yaw`(m.Chassis.yaw 필터링) 쪽.
- **`act_yaw`(Scope 경로, 실제로는 `nYaw`=unwrap 누적값 추정) vs `filter_yaw_out`(Control Yaw가 실제 보는 값) 동시 로깅** → `act_yaw`는 369,439°까지 누적, `filter_yaw_out`은 시종일관 거의 0(-0.011°) — **두 신호가 완전히 다르게 거동**.
- **`m` 버스 자체의 배선은 정상**: `Maneuver Controller`의 Inport 1(`m`)은 `Quadcopter`(실제 물리 모델)에 직결되어 있음 — "선이 끊긴" 게 아님. `6 DOF` 블록에는 `yaw`(wrap 추정)/`yawWr`(wrap)/`nYaw`(unwrap 누적) 세 가지 버전이 존재.
- **가설(검증 필요)**: unwrap은 wrap 신호의 인위적 -180/180 점프만 제거하고 진짜 누적 회전은 보존하므로, `nYaw`가 크다는 건 **기체가 실제로 그만큼 누적 회전했다는 뜻**(허위 정보 아님). 반면 `Filter Yaw`가 받는 wrap된 `m.Chassis.yaw`는, 만약 기체가 필터 대역폭(`filtM_yaw=0.01s` 기준 ~100 rad/s)보다 훨씬 빠르게(추정 ~1484 rad/s) 실제로 회전하고 있다면, 빠르게 -π~π를 반복 순환하는 신호를 저역통과 필터가 평균 내버려 **거의 0으로 뭉개져 보이는 것**일 수 있음 — 이러면 "피드백이 끊긴 버그"가 아니라 "컨트롤러가 원천적으로 감지 불가능한 속도로 실제 물리적 회전이 일어나고 있다"는 뜻이 됨. **아직 순간 각속도를 직접 측정해서 확정하지는 못함** (6DOF 출력 버스에 각속도/omega 항목이 안 보여서 확인 방법 추가 조사 필요).
- 이 가설이 맞다면, 남은 진짜 질문은 여전히 **"왜 기체가 처음부터 그렇게 빨리 도는가"**(반작용 토크 불균형 등 물리적 원인) — 프로펠러 방향 수정으로 425k→365~380k대로 소폭 개선된 것과는 앞뒤가 맞지만, 완전한 원인 규명은 아직 못함.

### 5차 세션(오늘 마지막 확인): "롤/피치도 같은 문제 있나?" → yaw만 특이함, 빼기(오차) 로직 자체는 정상

사용자 질문: yaw만 그런 거면 이전에 짠 "오차 = 목표 - 필터출력" 계산 방식 자체가 롤/피치에도 똑같이 적용됐을 텐데, 거기서도 오차가 항상 0으로 나온다면 애초에 그 빼기 로직/탭 방식이 잘못됐다고 봐야 함. 그래서 `diagnose_yaw_reconcile.m`을 `Control Pitch`/`Control Roll`/`Control Yaw` 세 축 모두의 입력(오차)과 `Filter Pitch/Roll/Yaw` 출력을 동시에 로깅하도록 확장해서(입력 소스는 이름 추측 없이 `Line`/`SrcPortHandle` 추적으로 확정) 재검증(게인: `kp_attitude=5,ki=0,kd=2; kp_yaw=3,ki=0,kd=1; kp_altitude=0.05,ki=0,kd=0.05; kp_position=1,ki=0,kd=0.5`).

결과 (5초 호버 테스트):
- `act_yaw` (실제 물리, unwrap 누적): 마지막 6447.93 rad ≈ **369,439°** (여전히 폭주)
- `roll_error`: min=-0.086, max=**1.76098 rad(~100.9°)**, 마지막 0.460687 rad(~26.4°) — **실제로 크게 움직이고 값이 계속 변함**
- `filter_roll_out`: min=**-1.40252 rad(~-80.4°)**, 마지막 0.0598555 rad(~3.43°) — 큰 진폭에서 시작해 정착, 실제 물리 반영됨
- `pitch_error`: min=-0.00589, max=0.0134243, 마지막 0.00680731 — 절댓값은 작지만(원래 피치 게인이 낮아 응답이 작음) **0에 딱 붙어있지 않고 정상적으로 움직임**
- `filter_pitch_out`: min=-0.00965, max=0.000607, 마지막 1.64e-5 rad(~0.0009°) — 마찬가지로 작은 진폭이지만 실제 변화 반영
- `yaw_error`: min=-0.0193735, max=0.000197, 마지막 0.000197 — **시종일관 거의 0** (최대 절댓값도 ~1.1° 수준)
- `filter_yaw_out`: min=-0.000197, max=0.0193735, 마지막 -0.000197 rad(-0.011°) — **역시 거의 0**

**결론**: 롤/피치는 "오차 = 목표 - 필터출력" 탭 방식이 실제 물리 변화를 제대로 반영하고 있음(롤은 최대 100° 가까이 실제로 움직이는 게 오차에 그대로 나타남). **오직 yaw만 실제로는 369,439°까지 발산했는데 오차/필터출력 모두 최대 1° 수준에서 멈춰있음.** 즉:
- 이전에 우려했던 "빼기(오차 계산) 로직 자체가 잘못된 가정"이라는 가설은 **기각** — 탭 방식 자체는 정상.
- 문제는 **yaw 축에만 국한**된 현상이며, 4차 세션에서 세운 가설(yaw가 필터 대역폭(`filtM_yaw=0.01s`≈100 rad/s)보다 훨씬 빠르게 wrap 신호(-π~π)를 반복 순환하면, 저역통과 필터가 그 신호를 평균 내서 거의 0으로 뭉개버린다 — 롤/피치는 이런 식으로 빠르게 wrap을 반복할 만큼 빠르게 돌지 않았기 때문에 정상적으로 오차가 보임)와 정합적.
- 남은 미해결 질문(다음 세션 우선순위): (1) yaw의 순간 각속도를 직접 측정해서 이 가설을 확정, (2) yaw가 애초에 왜 그렇게 빨리 도는지의 물리적 근본 원인(반작용 토크 불균형 등, 프로펠러 `direction` 파라미터 자체는 이미 무관함이 확인됨).

**오늘 세션은 여기서 종료.**

### 다음에 시도해볼 것

1. **최우선**: 순간 각속도(yaw rate)를 직접 측정할 방법 찾기 (6DOF 출력에 없으면 `nYaw`를 수치 미분하거나, Simscape 센서 블록에 각속도 감지를 켜서 확인) — "필터 대역폭 문제" 가설을 확정하기 위함.
2. 반작용 토크 불균형이 진짜 원인인지 확정: 프로펠러 CW/CCW 방향, Motor Mixer의 yaw 배분식이 실제 CAD 배치와 맞는지 GUI로 직접 시각 확인.
3. `qc_motor.max_power`/`max_torque`를 FX450 A2212 1000KV 실제 스펙(대략 150~250W급)에 맞게 설정 + 상위(`kp_attitude`, `kp_position` 등) PID를 동시에 낮춰서 모터 명령이 물리적으로 달성 가능한 범위(수백~1000대 rad/s) 안에 들어오도록 재튜닝 — 다만 모터/PID만으로는 yaw 자체의 근본 원인(위 1,2번)이 안 풀리면 여전히 발산할 가능성 있음.
4. 재튜닝 후: 호버 테스트(`diagnose_hover_only_test.m`/`diagnose_hover_attitude.m` 패턴)로 roll/pitch/yaw가 발산 안 하고 0 근처에서 유지되는지 확인 → 통과하면 `run_and_log.py` 전체 파이프라인으로 실제 궤적 테스트.
5. `Altitude and YPR Control`의 "InputName" `systune` 에러는 여전히 미해결이지만, 이제 `linearize()`+`pidtune()` 조합보다 **호버 테스트 기반 수동/반자동 튜닝**이 훨씬 실용적인 경로로 보임.
6. (참고) F450/FX450 관련 인터넷 공개 PID 값(Betaflight/ArduPilot 등)은 찾아봤으나, 전부 다른 제어 구조/단위 체계라 이 모델에 직접 못 씀 — 참고용 스케일 감으로만 사용.

## 관련 파일

- `tune_pid.m` — 현재 WIP 상태, 위 5개 에러(1차 세션)는 이미 반영된 최신 코드. "InputName" 에러(6번째)에서 막힌 채로 남아있음 — 2차 세션은 `tune_pid.m` 자체는 안 건드리고 `diagnose/`의 별도 스크립트로만 진행함.
- `diagnose/` — 디버깅에 쓴 1회성 진단 스크립트 모음 (원인 규명에 실제로 도움 됨, `COMMANDS.md`에 1차 세션 스크립트 설명 있음. 2차 세션에서 추가된 스크립트: `diagnose_poscontrol_mask.m`, `diagnose_poscontrol_alone.m`, `diagnose_posgains.m`, `diagnose_mask_tunable_flag.m`, `diagnose_mass_check.m`, `diagnose_inertia_check.m`/`2.m`, `diagnose_arm_radius.m`/`2.m`, `diagnose_conservative_gains.m`, `diagnose_rename_ports_test.m`, `diagnose_simparam_tuning.m`, `diagnose_pidtune_position.m`, `diagnose_port_numbers.m`, `diagnose_port_elements.m`, `diagnose_pitch_limit_check.m`. 3~4차 세션 추가분: `diagnose_yaw_sign.m`, `diagnose_prop_layout.m`, `diagnose_fix_prop_direction.m`, `diagnose_motor_rotation_source.m`, `diagnose_motor_speed_loop.m`, `diagnose_all_motors_speed.m`, `diagnose_motor_pid_field.m`, `diagnose_speed_saturation.m`, `diagnose_wt_source.m`, `diagnose_motor_rescale_test.m`, `diagnose_wt_set_verify.m`, `diagnose_power_max_test.m`, `diagnose_combined_fix.m`, `diagnose_bisect_gains.m`, `diagnose_yaw_error_trace.m`, `diagnose_yawcmd_source.m`, `diagnose_yawcmd_port.m`, `diagnose_yaw_reconcile.m`, `diagnose_m_bus_trace.m`, `diagnose_yaw_rate_check.m`, `diagnose_r_to_xyz.m`).
- `run_sample_sim.m` — `motor_cmd_w1~w4` + `sim_time` 로깅 추가된 최신 버전.
- `../../sample/run_and_log.py` — `isaacsim_motor_commands.json` 출력 추가된 최신 버전.
