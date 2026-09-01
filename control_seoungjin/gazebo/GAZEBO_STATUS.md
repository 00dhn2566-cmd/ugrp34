# GAZEBO_STATUS

Gazebo 검증 진행 기록. 사용법·설계 근거는 [README.md](README.md).
`HANDOFF_CPP_GAZEBO.md` 가 "진행 기록은 별도 상태 md 에" 라고 했던 그 파일이다.

기록 규칙: 최신이 위. 한 줄 = 사건 1건. `날짜 — 내용`.
실행 결과는 `compare_plants.py` 출력을 그대로 붙이고, **어긋난 항목마다 어느 쪽을
믿을 것인지** 한 줄로 남길 것.

---

## 2026-08-26 — 하네스 작성 (실행 전)

Gazebo 없는 노트북에서 **코드만** 작성. 그 머신에 가서 `bash preflight.sh` 한 줄로
빌드~호버 스모크까지 가도록 구성.

작성물:
- `worlds/gen_worlds.py` + 생성 SDF 2종 (1 kg / 0 kg). 관성·질량은 `qc_phys()` 에서
  뽑고, **로터 좌표는 믹서 부호표에서 유도**한다 (손으로 45° X 를 가정하지 않음).
- `plugin/QcGzController.{hh,cc}` — gz-sim8 시스템 플러그인. 제어기를 물리와 같은
  프로세스·같은 1 kHz 고정 스텝으로 돌린다. 지연 주입(링버퍼) / 토크 펄스 / 정상풍 /
  Ct·Cq 진실 주입 / CSV 로깅 / 개루프 프로브 모드.
- `scripts/run_case.sh`, `scripts/run_matrix.sh` — 헤드리스 실행 (`gz sim -s -r
  --iterations`). GUI/GPU 불필요.
- `analyze/{gz_metrics,compare_plants,selftest}.py` + `simulink_ref.json`.

이 노트북에서 실제로 검증한 것:
- 믹서 표 자기검증 통과 (직교성 4종 + `mixYaw = ±mixDir` + 대각 회전방향)
- SDF 3종 XML 파싱 통과
- `analyze/selftest.py` 11개 검사 통과 (b=α/u 되뽑기, yaw 권한 0 보고,
  호버 RMS, 추종 RMS, 복귀 시간, 복귀 실패를 0 아닌 None 으로)
- `compare_plants.py` 합성 지표로 전 구간 출력 확인

**한 번도 검증 못 한 것: 플러그인 컴파일.** gz-sim8 헤더가 이 머신에 없다.
API 시그니처(`AddWorldWrench` / `EnableVelocityChecks` / `worldPose` / `GZ_ADD_PLUGIN`)
는 Harmonic 기준으로 썼지만 첫 빌드에서 오류를 각오할 것.

### ★ 작성 중 발견 — `qc_controller.hpp` 의 `mixYaw` 는 yaw 토크를 못 만든다

월드의 로터 사분면을 믹서 표에서 유도하는 과정에서 나왔다.

```
헤더   mixYaw = {-1,+1,-1,+1},  mixDir = {+1,-1,-1,+1}
       sum(mixDir_i * mixYaw_i) = 0        -> yaw 권한 0
       유도되는 배치에서 대각 로터의 회전방향이 서로 반대  -> X 쿼드 규약 위반
08-18  mixPitch={+1,+1,-1,-1} mixRoll={-1,+1,-1,+1} mixYaw={-1,+1,+1,-1}
       mixYaw == -mixDir (완전 정렬), 네 행 상호 직교
```

yaw 토크는 로터 반토크의 차동으로만 생기므로 위 내적이 0 이면 원리적으로 0 이다.
SESSIONS_BOARD 08-18 이 "C++ 표와 실측표가 모터 인덱스 치환만큼 어긋나 보인다"고
남긴 건이 **판정 가능해졌다** — 헤더 표는 인덱스 치환으로도 실측표가 될 수 없다
(치환은 `mixYaw·mixDir` 내적을 바꾸지 못한다).

조치: 하네스 기본값을 실측표로 두고, 헤더는 **건드리지 않았다** (골든 트레이스
불변). `run_matrix.sh probe` 가 `probe_yaw` 와 `probe_yaw_headertable` 을 나란히
돌려 이 예측을 실측으로 확인한다. 확인되면 그때 헤더를 고치고 골든 재대조.

### ★ 작성 중 발견 2 — 고도 클램프 배선 오류 (고침, `qc_controller.cpp`)

`base = clamp(uA + bias, ±30)` 이라 1 kg 에서 base 가 항상 30 rev/s 로 잘렸다
-> 추력 1.97 N vs 무게 22.29 N. **제어기가 뜰 수 없는 상태였다.**
Simulink 는 `cmd -> Alt Cmd Sat -> Bias Chassis` (bake_tuned_model.m (3)) 이므로
포화는 바이어스 **전**이다. `base = clamp(uA, ±30) + bias` 로 정정.

검증 (이 노트북, msys64 g++ 로 재빌드해서 실측):
- 호버 `motorRef = 634.0 rad/s` (기록값 634 와 일치), 추력 22.26 vs 무게 22.29 N
- 추력비 0.895 (controller_cpp README 기록 0.907)
- 0 kg 역산 base 75.6 rev/s == `qc_mass_lerp` 0 kg 앵커 75.5 (독립 유도, 0.1% 일치)

★ 튜닝/C++ 세션: **motorRef/motorCmd 골든 트레이스 재대조 필요.** 위치 체인
(cmd_pitch/cmd_roll)은 불변이라 07-18 합격은 유지된다.

부수 효과: 0 kg 은 `massLerpOn` 없이는 여전히 못 뜬다 (bias 56.5 -> 6.98 N <
12.48 N). 0 kg 월드는 `<massLerpOn>true</massLerpOn>` 를 기본으로 켰고, 플러그인은
"고도 권한을 다 써도 무게 미달"이면 시작을 거절한다.

### 추가 — 외란 작용점 + 사용 전력량 (같은 날 늦게)

사용자 요구로 둘을 더 넣었다.

- **외란에 작용점이 생겼다.** 무게중심에 순수 힘을 걸면 평행이동만 하는데, 실제
  돌풍은 기체의 어느 지점에 걸려 힘과 모멘트를 같이 만든다. `QC_DISTPOINT*` 로
  작용점을 주면 `tau = r x F` 를 자동으로 더한다. 토크·힘 각각 축 지정형과 3축
  벡터형을 다 받는다. Gazebo 자체 API 경로도 열었다 — 월드에 `ApplyLinkWrench` 를
  붙여 두고 `scripts/poke.sh` 로 돌고 있는 시뮬을 손으로 찌를 수 있다 (밖에서 넣은
  렌치는 플러그인 것과 합산된다). 속도 의존 바람은 `gen_worlds.py --wind-effects`.
- **사용 전력량을 로그에 남긴다** (`P_est_W`, `E_est_Wh`). `control_seoungjin/energy.py`
  와 **같은 식**이라 Gazebo / MATLAB / 계획기 셋이 같은 수를 낸다. Gazebo 는 총 추력을
  정확히 알므로 여기서는 사실상 모델 내 실측이다 (효율 상수만 여전히 미측정).

### 미해결 — 모멘트 암 두 값

기하 0.1125 m (FX450 45° X) vs 골든 트레이스 유효값 0.0930 m. CAD 암이 −11.7°
돌아간 "+"형이라 45° X 가 아니라는 08-18 기록도 있다. 월드는 일단 기하값을 쓰고,
`probe_pitch`/`probe_roll` 실측으로 `QC_ARMXY` 를 맞춘다.

관련: Simulink `b = −0.0296` (u→pitch 각가속도) 는 단위 규약이 불명확하다. 같은
정의로 기하 계산하면 ≈2.9 가 나와 약 98배 차이인데, 08-18 이 지적한 rpm↔rad/s
혼선 `(30/π)² = 91.2` 와 크기가 비슷하다. `simulink_ref.json` 에서 `comparable=false`
로 두고 **부호만** 비교한다.

---

## 다음에 할 것 (Gazebo 머신에서)

1. `python3 analyze/selftest.py` — 지표 코드부터 (Gazebo 없이 됨)
2. `bash preflight.sh` — 빌드 오류가 나면 여기 기록하고 고칠 것
3. `bash scripts/run_matrix.sh probe` — **폐루프 전에.** 세 축 부호/이득 + yaw 권한
4. `bash scripts/run_matrix.sh` — 전체
5. `python3 analyze/gz_metrics.py out/*.csv --json out/metrics.json && python3 analyze/compare_plants.py`
6. 결과를 이 파일에 붙이고 SESSIONS_BOARD 에 헤드라인 한 줄

미배선 (필요해지면):
- `mode=traj` 에 실제 `output/trajectory.json` 물리기 (`QC_TRAJ`)
- 회복 감시(`QC_RECON=1`) 폐루프 검증 — 표가 **틀린** 조건(0.5 kg / 100 ms)에서
- 0 kg 전용 지연 표 (구조 미구현, 08-23 이월)
- `MulticopterMotorModel` 경로(`fx450_test.sdf`)로 모터 모델까지 독립시키는 별개 실험
