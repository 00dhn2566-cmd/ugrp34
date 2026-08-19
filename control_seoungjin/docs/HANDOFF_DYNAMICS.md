# HANDOFF: 동역학 방정식 문서화 세션

작성: 2026-08-18, 저사양 노트북 세션. **대상: 성능 좋은 머신에서 이어받는 클로드 세션.**
계기: 지도교수 요구 — "이 시뮬레이터의 dynamics를 내놓아라".
전제 문서: [TUNING_STATUS.md](../controller/Quadcopter-Drone-Model-Simscape/TUNING_STATUS.md),
[PERFORMANCE_SPEC.md](../controller/Quadcopter-Drone-Model-Simscape/PERFORMANCE_SPEC.md),
[quadcopter_package_parameters.m](../controller/Quadcopter-Drone-Model-Simscape/Scripts_Data/quadcopter_package_parameters.m)

---

## 임무

Simscape 모델이 **실제로 푸는 동역학 방정식**을 문서로 복원해 제출 가능한 형태로 만든다.
산출물 목표: `docs/DYNAMICS.md` (또는 보고서 섹션) — 좌표계·EOM·힘요소·상수표·검증.

**이 문서를 왜 넘기나**: 재료 추출은 이 세션에서 끝냈다(아래 전량 수록). 남은 건
① 서술 작성 ② 골든 트레이스 검증(MATLAB 반복 배치 — 이 노트북 16GB로는 느리고 위험).
②가 무거워서 넘긴다. ①만 필요하면 이 문서만으로 바로 쓸 수 있다.

---

## 0. 가장 중요한 사실 (이거 하나면 절반 끝)

**이 모델엔 손으로 쓴 EOM이 없고, 있을 필요도 없다.**

Simscape Multibody는 심볼릭 EOM을 export하지 않는다(1-D 도메인과 달리 `.ssc` 소스가 없다).
그런데 **이 모델의 위상에서는 생성되는 방정식이 뉴턴-오일러와 항등이다.**

```
World Frame → Cartesian Joint (병진 3) → Spherical Joint (회전 3) → Body
```

- 자유물체 **하나**, 6자유도, **닫힌 루프 없음, 구속조건 없음**
- 짐 = Weld(자유도 0). 지면접촉·항력·추력은 전부 **구속이 아니라 외력 요소**

다물체 일반 기계장치(재귀 알고리즘·라그랑주 승수·구속 안정화)는 이 위상에서 전부 퇴화한다.
남는 것:

```
m·v̇_W = R(η)·[0,0,ΣTᵢ]ᵀ + [0,0,−m·g]ᵀ + F_drag,W
I·ω̇  + ω×(I·ω) = τ_prop + τ_drag
```

→ **뉴턴-오일러를 쓰는 건 근사가 아니라 정답이다.** 보고서에서 이 논증을 먼저 세울 것.

오일러각은 사후 추출이다: `R to X-Y-Z Extrinsic`(`R(3,1)`→asin으로 pitch, `R(3,2:3)`·`R([2 1],1)`→
atan2로 roll/yaw, yaw는 Unwrap). 별도 `Roll Pitch` 서브시스템은 추력축 단위벡터로 계산하고
각속도는 워시아웃 `s/(0.01s+1)`로 미분한다.

---

## 1. 지금 상태 (믿어도 되는 것 / 아직 아닌 것)

### 믿어도 됨 (이 세션에서 `.slx` 직접 추출 + 검산)

- 위상 구조(§0), 질량·관성 전량(§3), 프로펠러/모터/항력/접촉 파라미터 전량(§4~7)
- 호버 평형 검산: `T=5.5716 N/모터 → n=100.96 rev/s → ω=634.4 rad/s`, `Q=0.200 N·m`
  — TUNING_STATUS의 실측 평형(634 rad/s, 토크 클램프 0.2)과 일치
- 실측 선형 플랜트: `ÿ = b·u + c·ẏ`, **b = −0.0296** (자세 게인 음수의 물리적 근거)
- `qc_phys()` 물성 합성이 실측 관성을 0.3% 내 재현 (기존 검증)

### 아직 아님 (보고서에 넣지 말 것)

- **`I/√m` 자세 게인 법칙** — 이 세션의 **가설**. 물리 예측 0.730 vs 실측 앵커 0.75로 3% 일치하나,
  0kg 검증이 혼돈 구간이라 판별 실패(§10). 검증 전 인용 금지.
- **고도 채널 스케일 미해명** — 물리 예측 `√m`(0.748) vs 실측 앵커 0.56. `0.748² = 0.560`이라
  제곱 관계가 의심되나 원인 불명. parameters.m 주석의 "고도는 물리, 자세는 관성비보다 덜"은
  **거꾸로일 가능성**이 있다.
- 믹서 차동 부호표(`τ_roll`/`τ_pitch`의 ± 배열) — HANDOFF_CPP_GAZEBO의 [TODO-verify] 그대로 미확정.

---

## 2. 재료 추출 방법 (재현용)

`.slx`는 zip이다. **MATLAB 없이** 전부 열린다. 파일을 복사해 `unzip`한 뒤
`simulink/systems/system_*.xml`을 파싱하면 되고, 블록 파라미터는 `<Block>` 아래
`<InstanceData>` 안의 `<P Name="...">`에 들어 있다 (라이브러리 기본값과 다른 것만 저장됨).
서브시스템 트리는 `<System Ref="system_XXXX"/>`로 연결된다.

주요 매핑:

| 경로 | 파일 |
|---|---|
| `Quadcopter` | `system_4845` |
| `Quadcopter/6 DOF/Joints` | `system_6989` |
| `Quadcopter/Body/Body` | `system_6976` |
| `Quadcopter/Wind and Drag/Aerodynamic Drag` | `system_7697` |
| `Quadcopter/Electrical` | `system_6159` |
| `Quadcopter/Load` | `system_6956` |
| 라이브러리 `Propeller/Thrust and Drag` | `quadcopter_library.slx` → `system_181_261` |

---

## 3. 질량·관성 (CAD 실측, `InertiaType=Custom`으로 하드코딩)

| 요소 | 질량 | Ixx, Iyy, Izz [kg·m²] | 배치 |
|---|---|---|---|
| `plate_top` | 0.0317292 kg | 2.77785e-5, 2.75764e-5, 5.53432e-5 | z = −26 mm |
| `plate_bottom` | 0.0317292 kg | 2.77785e-5, 2.75764e-5, 5.53432e-5 | z = −38 mm |
| `Arm1~4` (`quadcopter_drone_arm.stp`) | 0.0589152 kg 각 | 2.63222e-4, 2.60855e-4, 1.23093e-5 | (12,58,−38), (58,−12,−38), (−58,12,−38), (−12,−58,−38) mm |
| `Flight Computer` (Brick) | **638 g** (`BasedOnType=Mass`) | 기하에서 계산 (60×60×15 mm) | 중앙 |
| 모터 `Housing` ×4 | 51 g 각 | 7.0438e-6, 7.04379e-6, 6.06908e-6 | 암 끝 |
| 모터 `Cap` ×4 | 0.000930841 kg | 1.32154e-8, 2.22535e-8, 1.32154e-8 | 〃 |
| 모터 `Base` ×4 | 실린더 r=14 mm, h=5 mm, ρ=`rho_pla`=1.25 | 기하 계산 | 〃 |
| `Package` (Brick) | `pkgSize³ × pkgDensity` = **정확히 1 kg** | 기하 계산 | 부착면 z = −12 mm |
| `Legs` | 압출 솔리드 2조 + 접점 4 | — | 하부 |

**`Plate Anchor Comp`**: Rigid Transform `[−30.7741, 30.1152, 0.78248] mm` — 튜닝 세션이 넣은
앵커 보정. 물리 부품이 아니라 좌표 보정이므로 보고서에선 각주 처리.

### 합성 물성 (`qc_phys()` 재현값)

| 짐 | m_tot [kg] | z_cg [m] | Ixx | Iyy | Izz | I_att |
|---|---|---|---|---|---|---|
| 1 kg | 2.2726 | −0.03175 | 1.711e-2 | 1.716e-2 | 2.124e-2 | 1.7135e-2 |
| 0 kg | 1.2726 | +0.00773 | 9.334e-3 | 9.384e-3 | 1.797e-2 | 9.3593e-3 |

X-쿼드 모터 반경 `l = 0.225/√2 = 0.15910 m` (FX450 휠베이스 450 mm).

---

## 4. 프로펠러 — ⚠ 단위 함정 있음

블록: **Simscape Driveline `Aerodynamic Propeller`** (`sdl_lib/Engines & Motors`),
`parameterization = Constant`, `n_blades = 2`, `direction`은 대각 쌍 반전.

```
D        = propeller.diameter  = 0.254 m
kt_const = propeller.Kthrust   = 9.79
kp_const = propeller.Kdrag·2π  = 0.597 × 2π ≈ 3.751
```

### ⚠ 보고서에는 이 값을 쓰지 마라

`.slx`의 `Kthrust=9.79`는 블록 내부 정규화 때문에 **표준 계수와 91.3배 차이**가 난다
(parameters.m L32 주석: "표준계수 0.1072 × 91.3"). 손으로 식을 쓸 땐 **반드시** 아래 조합:

```
T = Ct·ρ·n²·D⁴      Ct = 0.1072,  n = ω/2π [rev/s]
Q = Cq·ρ·n²·D⁵      Cq = 0.01517
```

근거·검산: [qc_motor.hpp](../controller_cpp/include/qc_motor.hpp) L6~14.
**검산**: `n=100.96 → T=5.5716 N/모터 ×4 = 22.29 N ≈ m_tot·g = 2.2726×9.80665 = 22.29 N` ✓

### 유입속도 반영 (논문 인용 시 주의)

`Sensing Va` 서브시스템이 `Transform Sensor`(NonRotatingFollower, `SenseZDot=on`)로 축방향
유입속도를 재서 블록에 넣는다 → **전진비 J가 반영된다.** 단순 `T=k·ω²` 논문만 인용하면
"하강 시 추력 변화는?"에서 막힌다. Bangura & Mahony (ACRA 2012)를 같이 걸 것.

인가 방식: `External Force and Torque`(Y축 힘 + Y축 토크, AttachedFrame). `Transform Propeller`가
−X 90° 회전으로 프롭축 정렬. `Thrust Direction` PS Gain ±1 = CW/CCW.

라이브러리에 상수계수 수식판(`Thrust and Drag Calc`)도 있지만 **모델엔 배선되어 있지 않다.**

---

## 5. 공력 항력 — 유일하게 `.slx`에 식이 그대로 보이는 부분

6채널 전부 동일 형태 (Simulink 기본 블록으로 조립):

```
f = −sign(v) · (ρ_air/2) · A · Cd · v²
```

| 채널 | Cd | A [m²] |
|---|---|---|
| X / Y | 0.35 / 0.35 | 0.0875 (YZ) / 0.0900 (XZ) |
| Z | 0.6 | 0.2560 (XY) |
| Roll / Pitch | 0.2 | 0.512 (XY×2) |
| Yaw | 0.2 | 0.256 (XY) |

입력은 **상대속도**(기체 − 바람). `Environment`가 고도별 돌풍 프로파일 생성(2 m 이하 0 →
6 m 이상 최대). `Compare To Zero`로 짐 유무를 판정해 압력중심 프레임 2개 중 하나에 인가한다.

---

## 6. 모터 전기동역학

`ee_lib/Electromechanical/Motor & Drive (System Level)` ×4,
`torque_speed_param = torque_power` 모드:

```
torque_max = qc_motor.max_torque = 0.8 N·m
power_max  = qc_motor.max_power  = 160 W
time_const = 0.02 s,  rotor_damping = 1e-7 N·m/(rad/s)
```

⚠ **`T_t`/`w_t` 배열은 죽은 파라미터다** — `torque_power` 모드에서는 안 쓰이고 위 스칼라만
활성이다 (TUNING_STATUS에서 실측 확인, 여기서 시간 많이 씀). 보고서에 배열 넣지 말 것.

C++ 근사 등가 모델(Gazebo/독립 실행용):

```
J·ẇ = τ − Q − b·w      J = 1.26e-5 kg·m² (시정수 0.02s 역산), b = 1e-7
V = duty·V_batt,  duty = |cmd|/limit_motor
```

---

## 7. 중력 / 짐 / 지면접촉

- **중력**: `Mechanism Configuration`, uniform `[0, 0, −9.80665] m/s²`
- **짐**: Brick `pkgSize=[0.14]³ m`, `pkgDensity = 1/부피` → 질량 정확히 1 kg.
  `Weld Joint`(강체) + `Disengage Logic`(투하)
- **지면접촉**: `Spatial Contact Force`, `SmoothSpringDamper`
  강성 `pkgGrndStiff=1000 N/m`, 감쇠 `pkgGrndDamp=300 N·s/m`, 전이폭 `1e-3 m`,
  마찰 `SmoothStickSlip` μs=0.5 / μd=0.3, 임계속도 1e-2 m/s.
  짐 8꼭짓점 `Point Cloud` vs `Infinite Plane`. 다리 4접점도 동일 방식.

---

## 8. ⚠ 페이로드 — 문헌 인용 시 여기서 틀린다

**이 모델의 짐은 Weld(강체)다. 케이블 현수가 아니다.**
관측되는 1.8Hz 모드는 케이블 진자가 아니라 **짐 때문에 CG가 추력면 아래 8.1 cm로 내려간
강체 전체의 저중심 모드**다 (TUNING_STATUS §567 조인트 전수조사로 확정).

증거 3종 (전부 실측):

1. **질량 지문** — 1→2 kg에도 1.75 Hz 소수점까지 불변 (`ω² = g·mL/mL² = g/L`, 질량 약분)
2. **크기 지문** — 0.10/0.14/0.20 m 큐브에도 1.80 Hz 불변, 진폭만 비례 (중심점 용접)
3. **결합 지문** — 조인트 전수조사 결과 Weld

→ **현수하중(Sreenath, Palunko 등) 논문을 인용하면 안 된다.** 모델에 없는 2자유도 구면진자를
주장하게 되고, "그 케이블 상태변수 어디 있냐"에서 막힌다. 인용은 §11의 강체 부착 계열로.

**부록거리**: 예전 플레이트 방향 버그로 CG가 추력면 *위*로 갔을 때의 발산(TUNING_STATUS §K/§317)은
**같은 중력 모멘트의 부호 반전**이다. `Δz>0`(아래) → 안정 진동, `Δz<0`(위) → 도립 발산.
한 식으로 두 사건을 설명할 수 있어 보고서 재료로 좋다.

**주의**: 나중에 Weld를 케이블 조인트로 바꾸면 모드 주파수가 완전히 달라진다
(L=0.5 m면 0.71 Hz). 1.8 Hz용 ZV 셰이퍼도 그때 다시 잡아야 한다.

---

## 9. 검증 절차 — 골든 트레이스 (이 세션이 못 한 부분, ★핵심)

"내가 쓴 식이 곧 이 시뮬레이터가 푸는 식"임을 **주장이 아니라 실증**으로 만든다.

1. §0 EOM + §4~7 힘요소를 **독립 수치적분기로 구현** (Python ~40줄. `qc_motor.hpp`가
   액추에이터 사슬 참조 구현)
2. 동일 초기조건 + 동일 모터 입력을 Simscape에 주고 궤적 로깅
3. 두 궤적 중첩 비교

**기존 인프라 재사용**: `controller_cpp/compare_trace.py` + `diagnose/diagnose_golden_trace.m`가
제어기에 대해 정확히 이 대조를 하려고 만들어져 있다. **플랜트로 확장하는 것뿐**이다.
합격선 제안: 10초 자유낙하/호버/스텝 3편에서 위치 오차 cm급, 자세 오차 1° 이내.

일치하면 보고서 §검증에 그림 한 장으로 끝난다. 유도만 적은 보고서보다 훨씬 세다.

---

## 10. 이 세션이 돌린 실험 (참고 — 재실행 불필요)

산출물: `diagnose/probe_0kg_precision.m` / `.csv`, `diagnose/probe_airframe_mass.m` / `.csv`

### ① 0kg precision 3구성 — 저질량에서 질량 보정 게인의 필요성 확증

| 구성 | sA/sZ | 추종 cm | 오버 cm | z피크 cm | 자세피크 ° |
|---|---|---|---|---|---|
| 1차식 | 0.750/0.560 | 21.35 | 37.1 | 11.0 | 22.2 |
| 물리 I/√m | 0.730/0.560 | 25.36 | 29.3 | 5.4 | 26.5 |
| **고정게인** | 1.000/1.000 | 31.41 | **4293.5** | **85.1** | **44.5** |

- **결론**: 보정 없이 0kg 비행 시 x로 42.9 m 이탈 = 완전 붕괴. 보정하면 생존.
- **스펙 채점(0kg, 1차식)**: 오버슈트 37.1 cm (스펙 ≤10) ❌ / 추종 21.4 cm (≤10) ❌ /
  z 이탈 11.0 cm (≤10) ❌ / 호버 드리프트 2.0 cm (≤5) ✅ → **4항 중 3항 탈락.**
  같은 시나리오 1 kg에서는 전항 통과(7.73 / 4.08 / 0.22 / 0.08).
  → **"저질량에서 precision 권장"은 "스펙 만족"이 아니라 "그나마 안 죽는다"는 뜻이다.**
- **혼돈 구간 정량 증거**: 게인을 상대 3×10⁻⁷만 바꿔도 추종이 21.35 → 28.61 cm (34% 변동).
  시뮬 자체는 결정론적(비트 단위 재현 확인). 0kg에서 게인 미세 비교는 **원리적으로 무의미**.

### ② 기체 증량 @0kg (실패한 실험) — 배운 것만 기록

- Flight Computer 638 g → 888/1138 g. **B/C 모두 z피크 85.2 cm = 지면에 주저앉음**(못 뜸).
- 원인: 고도 트림(`Bias Chassis`)과 게인(`sZ`)이 **`m_pkg`만 보고 기체 질량은 모른다.**
- 트림 +22.2 rev/s 보정(D)도 실패(82.5 cm) + 횡방향 21.7 m 발산.
- **미해결**: "관성을 키우면 자세가 나아지나"는 고도 붕괴에 가려 답 못 함.
  제대로 하려면 `qc_phys(drone_mass+dm, 0, pkgSize)`로 **게인·트림 전부 재계산** 필요.
- **부수 발견**: `drone_mass` 변수는 `.slx`가 참조조차 안 한다 — `qc_phys()` 게인 계산 전용.
  실제 기체 질량 knob은 `Quadcopter/Body/Body/Flight Computer`의 `Mass`다.

---

## 11. 인용 문헌 (섹션별)

**강체 EOM 형태**

- Beard, *Quadrotor Dynamics and Control Rev 0.1*, Brigham Young University, 2008 — 무료 테크리포트.
  표기법을 통째로 따라가기 가장 편하고, Z-Y-X 규약이 모델의 `R to X-Y-Z Extrinsic`과 일치.
- Bouabdallah, PhD thesis, EPFL, 2007 — **공력 항력 항을 명시적으로 포함**해 유도. §5 대응.
- Mahony, Kumar, Corke, IEEE Robotics & Automation Magazine 19(3), 2012 — 표준 인용 레퍼런스.

**프로펠러**

- Bangura & Mahony, ACRA 2012 — 로터 유입(inflow) 반영 근거 (§4의 J 의존성).
- Pounds, Mahony, Corke, *Control Engineering Practice* 18(7), 2010 — 플래핑까지. "무시했다" 명시용.
- Brandt & Selig, AIAA Aerospace Sciences Meeting, 2011 — `Ct`/`Cq` 실측값 출처 (UIUC DB, APC 포함).

**모터**

- Quan Quan, *Introduction to Multicopter Design and Control*, Springer, 2017 — 5~6장.

**페이로드 (강체 부착 — §8 참조, 현수 논문 쓰지 말 것)**

- Mellinger, Lindsey, Shomin, Kumar, IROS 2011 — 짐 → m, I, CG 변화와 보상. `qc_phys()`와 동일 문제의식.
- Pounds, Bersak, Dollar, ICRA 2011 — **CG 오프셋이 안정성을 바꾼다**. §8 부록거리의 직접 근거.
- Ruggiero, Lippiello, Ollero, IEEE RA-L 3(3), 2018 — 서베이. 서론 위치 짓기용.

**진동 억제 (`traj_zv.m` 근거)**

- Singer & Seering, ASME J. Dynamic Systems, Measurement, and Control, 1990 — ZV 원전.
- Singhose, *Int. J. Precision Eng. and Manufacturing*, 2009 — ZV→ZVD→EI, 주파수 오차 민감도.

**서술 전략**: 유도는 표준을 인용하고 **상수와 검증만 우리 것**으로 내세운다.

> "동역학 모델은 [Beard 2008], [Bouabdallah 2007]의 표준 정식화를 따르되,
> 물성 상수는 CAD 실측에서 산출하고 호버 평형으로 검산하였다."

그러면 질문이 "어디서 베꼈냐"가 아니라 "이 숫자 어디서 나왔냐"로 옮겨가고, 그건 전부 답할 수 있다.

---

## 12. 절대 규칙

1. **`save_system` 금지** — 모델은 구운 상태다. in-memory 편집 후 저장 없이 닫을 것.
2. **자세 게인 음수는 의도** (`b=−0.0296`). "부호 수정"하면 즉시 발산.
3. **앵커 불변** — `Kthrust_ref=9.79`, `Kdrag_ref=0.597` 등은 "튜닝했던 그 날의 값". 갱신 금지.
4. **`git submodule update --init` 금지**(컨트롤러 경로) — FX450 CAD가 날아간다.
5. **라이브러리 링크 해제는 부모부터** — 대상 블록 자신부터 끊으면 Simscape 리프 블록(`Brick Solid`)
   링크까지 해제되어 컴파일이 거부된다. 이 세션에서 실제로 4구성을 전멸시킨 버그.
   `p = get_param(blk,'Parent')`부터 시작할 것 (`diagnose_pid_ident.m` 패턴).
6. **§1의 "아직 아님" 3건을 보고서에 넣지 말 것.**

---

## 13. 머신 요구사항

- **§11까지 서술만** → MATLAB 불필요. 이 문서만으로 어느 머신에서든 작성 가능.
- **§9 골든 트레이스** → MATLAB R2026a + Simulink / Simscape / Multibody / Electrical / **Driveline**.
  배치 1회 ≈ 2~4 GB, 반복 대조라 **RAM 16 GB 이상 권장**(현 노트북이 딱 16 GB라 위험).
  MATLAB 슬롯은 단일 점유 규칙 — SESSIONS_BOARD 확인 후 착수.

---

## 14. 권장 착수 순서

1. §0 위상 논증 + §3~7 상수표로 `docs/DYNAMICS.md` 초안 (MATLAB 불필요, 반나절)
2. §11 문헌으로 인용 채우기
3. §9 골든 트레이스 구현 → 검증 그림 1장 추가
4. (여력 있으면) §10 ②의 미해결 — 기체 질량 축을 물성 정규화에 되살리기
