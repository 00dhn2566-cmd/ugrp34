# Gazebo 검증 하네스

`controller_cpp/` 의 C++ 제어기를 **Gazebo 플랜트에 폐루프로** 붙여 돌리는 도구
한 벌. 이 노트북(MX450)에는 Gazebo 가 없으므로 여기서는 **작성만** 했고, 실행은
Gazebo 가 깔린 머신에서 한다. 그쪽에서 `bash preflight.sh` 한 줄이면 빌드부터
호버 스모크까지 간다. 설치 스크립트는 없다 — 이미 깔려 있는 머신을 전제한다.

> 상태: **아직 한 번도 Gazebo 에서 실행되지 않음.** 아래 "검증된 것 / 안 된 것" 참조.

---

## 왜 Gazebo 를 또 돌리나

MATLAB 구운 모델(Simscape)은 계속 **정답 플랜트**다. Gazebo 는 그것을 대체하는 게
아니라 **독립 물리로 교차 검증**하는 역할이다. 지금 시점에서 이게 필요한 이유는
구체적으로 셋이다.

1. **08-23 지연→스펙 표가 Simscape 과적합인지 모른다.** `capability.py` 의
   `_LAT_POS_ANCHORS` (40 ms → 0.88, 60 ms → 0.75, 80 ms → 0.37, 120 ms → 0) 는
   전부 한 플랜트에서 잰 값이다. 다른 적분기·다른 접촉 모델에서 같은 절벽이
   보이면 그건 "적합"이 아니라 "스펙"이 된다.
2. **자세 지연 관문 16 ms 도 마찬가지다.** 관문은 임무를 거절하는 판정이라
   근거가 한 플랜트뿐이면 곤란하다.
3. **0 kg 질량 법칙은 혼돈 구간에서 튜닝됐다** (게인 상대 3e-7 섭동에 지표 34%
   변동). 다른 플랜트에서 무너지면 그건 법칙이 아니라 그 시뮬의 우연이었다는 뜻이다.

## 무엇을 증명하고, 무엇은 증명하지 못하나

증명한다:
- 제어 사슬(위치→자세→믹서→모터)의 **부호와 구조**가 플랜트 독립적으로 옳다
- **강체 동역학**(관성 합성, 좌표변환, 적분)이 Simscape 와 같은 답을 낸다
- 지연/외란에 대한 **정성적 거동**이 두 플랜트에서 같다

증명하지 못한다 (정직하게):
- **모터 모델은 공유한다.** `qc_motor.hpp` 를 그대로 쓰므로 모터 1차 지연·추력
  계수는 교차검증 대상이 아니다. 이건 의도적 선택이다 — Gazebo 의
  `MulticopterMotorModel` 을 쓰면 모터도 독립이 되지만, 그 플러그인의 계수 규약
  (`motorConstant` / `momentConstant`)을 Simscape 의 `Aerodynamic Propeller` 규약
  으로 환산하는 것 자체가 미검증이라 오차원이 하나 더 늘어난다.
  (`fx450_test.sdf` 가 그 경로로 만들어 둔 옛 월드다. 별개 실험으로 남겨 둠.)
- **공력 항력이 없다.** 두 플랜트 모두 기체 항력을 모델링하지 않는다.
- 절대 성능 수치가 실기와 같다는 것 — 그건 시뮬 둘의 문제가 아니다.

---

## 파일

```
gazebo/
  preflight.sh              도착해서 첫 줄. 환경 확인 -> 빌드 -> SDF 파싱 -> 호버 스모크
  worlds/
    gen_worlds.py           qc_phys() 에서 SDF 를 생성 + 믹서 표 자기검증
    fx450_qc_1kg.sdf        생성물 (1 kg 용접 짐)   <- 기본
    fx450_qc_0kg.sdf        생성물 (생 드론)
  plugin/
    QcGzController.hh/.cc   gz-sim8 시스템 플러그인 (제어기 + 지연 주입 + 외란 + 로깅)
    CMakeLists.txt          controller_cpp 를 서브디렉토리로 끌어와 링크
  scripts/
    run_case.sh             케이스 1개 헤드리스 실행 -> out/<이름>.csv
    run_matrix.sh           검증 행렬 (probe/base/dist/delay/mass)
  analyze/
    gz_metrics.py           CSV -> 지표 (표준 라이브러리만)
    compare_plants.py       Simulink 기록값과 나란히
    simulink_ref.json       그 기록값 + '조건이 같은가' 표시
    selftest.py             분석 사슬 자체 시험 (Gazebo 없이 돈다)
  fx450_test.sdf            옛 월드 (MulticopterMotorModel 경로) — 참고용
  GAZEBO_STATUS.md          진행 기록. 실행하면 여기에 쓴다
```

`out/` 는 gitignore 다.

---

## 실행 순서

순서가 곧 논리다. 앞이 깨지면 뒤 숫자는 의미가 없다.

```bash
cd control_seoungjin/gazebo

python3 analyze/selftest.py        # 0. 지표 코드가 정상인지 (Gazebo 없이 됨)
bash preflight.sh                  # 1. 빌드 + SDF 파싱 + 호버 3초
bash scripts/run_matrix.sh probe   # 2. 개루프 프로브  <- 폐루프보다 먼저!
bash scripts/run_matrix.sh         # 3. 전체 행렬
python3 analyze/gz_metrics.py out/*.csv --json out/metrics.json
python3 analyze/compare_plants.py
```

`preflight.sh` 가 개발 헤더 없다고 멈추면 그 한 줄만 깔면 된다
(`libgz-sim8-dev libgz-plugin2-dev libsdformat14-dev`). Gazebo 본체는 이미 있는
머신을 전제하므로 재설치는 하지 않는다.

### 외란을 "어디에" 거는가

토크만 걸면 기체는 제자리에서 돌고, 무게중심에 힘만 걸면 평행이동만 한다. 실제
돌풍은 **기체의 어느 지점에** 걸려서 힘과 모멘트를 동시에 만든다. 그래서 세 가지
경로를 다 열어 뒀다.

| 무엇 | 어떻게 | 작용점 |
|---|---|---|
| 순수 토크 | `QC_PULSETORQUE` + `QC_PULSEAXIS`, 또는 `QC_PULSETORQUEX/Y/Z` | 무관 |
| 힘 (펄스) | `QC_PULSEFORCE` + `QC_PULSEFORCEAXIS`, 또는 `QC_PULSEFORCEX/Y/Z` | `QC_DISTPOINT*` |
| 힘 (상시풍) | `QC_WINDX/Y/Z` (+ `QC_WINDFRAME`) | `QC_DISTPOINT*` |

작용점을 주면 플러그인이 `tau = r × F` (`r = distPoint − (0,0,comZ)`)를 자동으로
더한다. 예를 들어 짐(z = −0.082 m)에 옆바람이 걸리는 경우:

```bash
bash scripts/run_case.sh gust_on_pkg QC_TEND=16      QC_WINDX=2.0 QC_DISTPOINTZ=-0.082
```

돌고 있는 시뮬에 **손으로** 찔러 보고 싶으면 Gazebo 의 `ApplyLinkWrench` API 를 쓴다
(월드에 이미 붙여 뒀다):

```bash
bash scripts/poke.sh fy=3 tz=0.2 --persist   # 힘·토크 동시, 지속
bash scripts/poke.sh --clear                 # 해제
```

밖에서 넣은 렌치는 플러그인이 거는 외란과 **합산**된다 (`AddWorldWrench` 는 누적).
단 `ApplyLinkWrench` 의 힘은 무게중심에 걸리므로 작용점 개념이 없다 — 재현이
필요한 실험은 `QC_DISTPOINT*` 쪽으로 박아 두는 게 맞다.

속도에 따라 힘이 변하는 Gazebo 기본 바람 모델을 쓰고 싶으면 월드를 다시 뽑는다
(기본은 꺼져 있다 — 물리가 달라지는데 이 노트북에서 검증할 수 없어서):

```bash
python3 worlds/gen_worlds.py --wind-effects "3 0 0"
```

### 궤적을 물릴 때

날것 스텝을 주면 안 된다 (`HANDOFF_CPP_GAZEBO.md` 절대 규칙 3 — 이 제어기는
스무더+게이트를 통과한 궤적을 전제한다). `traj_pipeline.py` 산출물을 준다:

```bash
QC_TRAJ=/abs/path/output/trajectory.json bash scripts/run_matrix.sh base
# 또는 케이스 하나만
bash scripts/run_case.sh line QC_MODE=traj QC_TEND=40 \
     QC_TRAJECTORY=/abs/path/output/trajectory.json
```

이륙은 플러그인이 알아서 한다 — 초기 위치에서 궤적 첫 점까지 7차 스무스스텝으로
`takeoffS` 초 동안 올린다 (기본 3 s). 궤적 시각은 그 뒤부터 0 으로 센다.

---

## 설정 (QC_* 환경변수)

SDF 태그명을 대문자로 한 환경변수가 월드 파일을 **덮어쓴다**. 그래서 케이스마다
월드를 다시 쓸 필요가 없다.

| 변수 | 기본 | 뜻 |
|---|---|---|
| `QC_MODE` | `hover` | `hover` / `traj` / `probe` |
| `QC_WORLD` | `worlds/fx450_qc_1kg.sdf` | 월드 파일 (run_case.sh 전용) |
| `QC_TEND` | `10` | 시뮬 길이 [s] → `--iterations` 로 환산 (run_case.sh 전용) |
| `QC_PKGMASS` | `1.0` | 짐 질량 [kg] — 월드도 같이 바꿀 것 |
| `QC_PROFILE` | `precision` | `precision` / `balanced` / `agile` |
| `QC_HOVERZ` | `1.0` | 호버 고도 [m] |
| `QC_TAKEOFFS` | `3.0` | 이륙 램프 [s] |
| `QC_TRAJECTORY` | — | `output/trajectory.json` 절대경로 |
| `QC_POSDELAYS` | `0` | 위치(VIO) 경로 지연 [s] — `measAgeS` 로도 들어간다 |
| `QC_ATTDELAYS` | `0` | 자세 경로 지연 [s] |
| `QC_PULSETORQUE` | `0` | 토크 펄스 [N·m] |
| `QC_PULSEAXIS` | `y` | `x`(roll) / `y`(pitch) / `z`(yaw), 기체 좌표 |
| `QC_PULSESTARTS` / `QC_PULSEDURS` | `0` / `0.3` | 펄스 시각/길이 [s] |
| `QC_PULSETORQUEX/Y/Z` | `0` | 토크 펄스 3축 벡터 [N·m] (약식과 더해짐) |
| `QC_PULSEFORCE` / `QC_PULSEFORCEAXIS` | `0` / `x` | 힘 펄스 [N] — 병진을 직접 민다 |
| `QC_PULSEFORCEX/Y/Z` | `0` | 힘 펄스 3축 벡터 [N] |
| `QC_WINDX/Y/Z` | `0` | 정상풍 [N] (창 없이 상시) |
| `QC_WINDFRAME` | `world` | `world`(기울어도 방향 고정) / `body` |
| `QC_DISTPOINTX/Y/Z` | `0` | **외란 힘의 작용점** (기체 좌표, 링크 원점 기준) — `r×F` 만큼 모멘트가 같이 생긴다 |
| `QC_CTSCALE` / `QC_CQSCALE` | `1` | 플랜트 진실 주입 — 제어기는 공칭 유지 |
| `QC_MIXERTABLE` | `measured` | `measured`(08-18 실측) / `header`(C++ 기본, 대조군) |
| `QC_MASSLERPON` | `0` (0 kg 월드는 `1`) | 질량 1차식 적용 — 저질량은 이게 없으면 못 뜬다 |
| `QC_BIASCHASSIS` | `56.5` | 추력 바이어스 [rev/s]. 1차식보다 우선 |
| `QC_ALTCMDSAT` | `30` | 고도 PID 출력 클램프 [rev/s] |
| `QC_ARMXY` | `0.1125` | 로터 x=y 성분 [m] |
| `QC_SPECON` / `QC_RECON` / `QC_GOVON` | `1` / `0` / `0` | 스펙 보고 / 회복 감시 / 조속기 |
| `QC_CONTROLRATEHZ` | `1000` | 제어 주기 |
| `QC_LOGRATEHZ` | `200` | 로그 주기 (프로브는 1000 권장) |
| `QC_PROBECHANNEL` / `QC_PROBEU` | `pitch` / `1.0` | 프로브 축 / 차동 크기 [rev/s] |

---

## 작성 중에 나온 것 — 반드시 먼저 읽을 것

### 1. `qc_controller.hpp` 의 `mixYaw` 는 yaw 토크를 못 만든다

월드를 만들려면 믹서 부호표에서 로터 사분면을 유도해야 하는데, 그 과정에서 헤더의
기본 표가 자기모순이라는 게 드러났다.

```
헤더   mixYaw = {-1, +1, -1, +1},  mixDir = {+1, -1, -1, +1}
       sum(mixDir_i * mixYaw_i) = 0
```

yaw 토크는 로터 **반토크의 차동**으로만 생긴다. 즉 속도를 올리는 로터들이 모두
같은 방향으로 돌아야 순 토크가 남는다. 위 내적이 0 이면 정확히 상쇄돼서 **yaw
권한이 원리적으로 0** 이다. 게다가 그 표에서 유도한 로터 배치는 대각 로터의
회전 방향이 서로 반대가 되어 X 쿼드 규약에도 어긋난다.

08-18 골든 트레이스 실측표는 깨끗하다:

```
실측   mixPitch = {+1,+1,-1,-1}, mixRoll = {-1,+1,-1,+1}, mixYaw = {-1,+1,+1,-1}
       mixYaw == -mixDir          (완전 정렬)
       세 행 + 추력 행이 서로 모두 직교
```

그래서 이 하네스는 **기본값을 실측표로 쓴다**. 헤더는 골든 트레이스 불변을 위해
건드리지 않았다 — 바꾸려면 골든 재대조가 먼저다 (SESSIONS_BOARD 08-18 항목이
"모터 인덱스 치환만큼 어긋나 보인다"고 남긴 그 건이 여기서 판정 가능해진 것).

`run_matrix.sh probe` 는 `probe_yaw` 와 `probe_yaw_headertable` 을 나란히 돌려
**이 예측(헤더 표의 yaw 각가속도 ≈ 0)을 실측으로 확인한다.** 이게 이 하네스가
첫날 낼 수 있는 가장 값싼 성과다.

### 2. 모멘트 암이 두 값으로 갈려 있다 — 프로브가 판정한다

- 기하 (FX450 대각 450 mm, 45° X): 로터 반경 0.1591 m → x=y = **0.1125 m**
- 골든 트레이스 유효 모멘트 암: **0.0930 m** (roll 0.0932 / pitch 0.0929)
- 게다가 CAD 암 배치는 축에서 −11.7° 돌아간 "+"형이라 45° X 가 아니다

월드는 일단 기하값 0.1125 를 쓴다. `probe_pitch` / `probe_roll` 이 각가속도를
실측하면 어느 쪽이 맞는지 `QC_ARMXY` 한 개로 맞출 수 있다.

관련해서 **Simulink 의 `b = −0.0296` (u→pitch 각가속도) 는 단위 규약이 불명확하다.**
기하로 계산하면 같은 정의에서 ≈ 2.9 rad/s²/(rev/s) 가 나와 약 98배 차이인데,
SESSIONS_BOARD 08-18 이 지적한 rpm↔rad/s 혼선 `(30/π)² = 91.2` 와 비슷한 크기다.
확정되지 않았으므로 `simulink_ref.json` 에서 이 항목은 `comparable=false` 이고
**부호만** 비교 대상으로 쓴다.

### 3. `qc_controller.cpp` 의 고도 클램프 배선 오류 — 고쳤다 (2026-08-26)

하네스를 쓰려고 호버 평형을 따라가다 나왔다. C++ 는 이렇게 돼 있었다:

```cpp
base = clamp(uA + biasChassis + biasLoadGain*pkgMass, -altCmdSat, +altCmdSat);  // altCmdSat = 30
```

1 kg 에서 `56.5 + 44.4 = 100.9` 가 항상 30 으로 잘린다. 그러면 모터 기준속도가
2π·30 = 188 rad/s 이고 추력이 **1.97 N** 인데 기체 무게는 **22.29 N** 이다.
즉 이 제어기는 **뜰 수가 없는 상태였다.**

Simulink 배선은 다르다. `diagnose/bake_tuned_model.m` 의 "(3) 고도 클램프" 가
`cmd` → **Alt Cmd Sat** → **Bias Chassis** 순으로 잇는다 — 포화는 **바이어스를
더하기 전** 고도 PID 출력에만 걸린다. 고친 형태:

```cpp
base = clamp(uA, -altCmdSat, +altCmdSat) + biasChassis + biasLoadGain*pkgMass;
```

두 가지로 교차 확인했다:
- 고친 뒤 호버 `motorRef = 634.0 rad/s` — SESSIONS_BOARD 가 기록한 호버 평형
  634 rad/s 와 일치. 추력 22.26 N vs 무게 22.29 N (0.1%). 추력비 0.895 는
  `controller_cpp/README.md` 의 "호버 평형 재현(추력비 0.907)" 과 같은 값.
- 0 kg 에서 필요한 base 를 역산하면 75.6 rev/s 인데, `qc_mass_lerp` 의 0 kg 앵커
  `biasChassis = 75.5` 와 0.1% 로 맞는다 (완전히 독립적으로 잡힌 값인데 일치).

⚠ **이 정정은 motorRef/motorCmd 채널의 골든 트레이스를 바꾼다.** `cmd_pitch`/
`cmd_roll` 위치 체인은 불변이므로 07-18 의 골든 1차 합격은 유지되지만, 모터 채널
대조는 다시 떠야 한다.

플러그인은 이 종류의 사고를 다시 겪지 않도록 **호버 가능성 방어**를 넣었다:
고도 PID 권한을 다 써도 무게를 못 이기면 시작 전에 거절하고, 0 kg 이면
`QC_MASSLERPON=1` 을 쓰라고 알려준다.

### 4. 절대 규칙 (튜닝 세션 피의 교훈 — 여기서도 유효)

- **자세 게인 음수는 의도**다. Gazebo 에서 부호가 반대로 나오면 게인이 아니라
  **월드의 로터 사분면**을 고친다.
- **앵커 불변**: `QcConfig` 의 `*Ref` 값은 튜닝 당시 값이다. 갱신 금지.
- **입력 계약**: 스무더+게이트 통과 궤적 전제. 날것 스텝으로 "불안정하다" 판정 금지.
- **anti-windup 없음 유지** (yaw 제외) — 원본과 같아야 대조가 성립한다.
- `qc_phys` 는 `parameters.m` ↔ `qc_controller.hpp` ↔ `gen_worlds.py` 세 곳의
  1:1 사본이다. 한쪽을 바꾸면 셋 다 바꿔야 한다.

---

## 검증된 것 / 안 된 것 (작성 시점)

이 노트북에서 실제로 돌려 본 것:
- `worlds/gen_worlds.py` — 믹서 표 자기검증 통과, SDF 2종 생성
- 생성된 SDF 3종 XML 파싱 (`fx450_test.sdf` 포함)
- `analyze/selftest.py` — 지표 11개 검사 전부 통과
- `analyze/compare_plants.py` — 합성 지표로 전 구간 출력 확인

**한 번도 안 돌려 본 것 (Gazebo 가 없으므로):**
- 플러그인 컴파일. gz-sim8 헤더가 이 머신에 없어 문법 확인조차 못 했다.
  `Link::AddWorldWrench` / `Link::EnableVelocityChecks` / `gz::sim::worldPose`
  / `GZ_ADD_PLUGIN` 시그니처는 Harmonic 기준으로 썼지만 **컴파일 오류를 각오할 것.**
  깨지면 대개 include 경로나 함수 시그니처 한두 줄이다.
- 실제 비행. 첫 실행에서 호버가 안 되면 `run_matrix.sh probe` 부터 볼 것 —
  부호 문제인지 이득 문제인지가 거기서 갈린다.
