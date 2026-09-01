# 쿼드로터 동역학 방정식 — Simscape 모델 복원

**대상 모델**: `controller/Quadcopter-Drone-Model-Simscape/Models/quadcopter_package_delivery.slx`
**작성**: 2026-08-18 · **근거**: `.slx` 블록 파라미터 직접 추출 + `프레젠테이션1.pptx` 블록 다이어그램 대조
**관련 문서**: [HANDOFF_DYNAMICS.md](HANDOFF_DYNAMICS.md) (추출 방법·재료 지도·미확정 항목)

> **핵심 주장**: 이 모델의 위상(자유물체 1개 · 6자유도 · 구속 없음)에서 Simscape Multibody가
> 생성하는 방정식은 **뉴턴-오일러 방정식과 항등**이다. 아래 식은 근사가 아니라 복원이다.

---

## 목차

| § | 내용 | PPT 슬라이드 |
|---|---|---|
| [1](#1-좌표계와-규약) | 좌표계와 규약 | — |
| [2](#2-플랜트-전체-구조) | 플랜트 전체 구조 | 1 |
| [3](#3-강체-6-dof-운동방정식) | 강체 6-DOF 운동방정식 | 1, 4, 5 |
| [4](#4-자세-표현과-추출) | 자세 표현과 추출 | 3, 4, 5 |
| [5](#5-프로펠러-추력과-반토크) | 프로펠러 추력·반토크 | 1, 6 |
| [6](#6-공력-항력) | 공력 항력 | 3, 4, 5 |
| [7](#7-모터-전기동역학) | 모터 전기동역학 | 2 |
| [8](#8-제어기-전달함수) | 제어기 전달함수 | 2 |
| [9](#9-질량관성-상수) | 질량·관성 상수 | — |
| [10](#10-조립된-최종-eom) | 조립된 최종 EOM | — |
| [11](#11-미확정-항목) | 미확정 항목 | — |

### 슬라이드 → 절 (캡션용 역매핑)

| 슬라이드 | 화면 내용 | 대응 절 |
|---|---|---|
| **1** | `Quadcopter` 최상위 — Propeller 1~4 / Body / Wind and Drag / Load / 6 DOF / Gravity / World Frame / Electrical | [§2](#2-플랜트-전체-구조) 위상, [§3](#3-강체-6-dof-운동방정식) EOM, [§5](#5-프로펠러-추력과-반토크) 프로펠러 |
| **2** | `Electrical` (배터리 + 모터 4) · 모터 속도 `Control` · PID 블록 대화상자 | [§7](#7-모터-전기동역학) 모터, [§8](#8-제어기-전달함수) 제어기 |
| **3** | `Wind and Drag` · `Roll Pitch` · `Aerodynamic Drag` | [§6](#6-공력-항력) 항력, [§4.3](#43-경로--roll-pitch-body-frame-계열-특이점-우회) 자세 추출 경로 ② |
| **4** | `6 DOF` · `drag force X` · `drag force Z` | [§4](#4-자세-표현과-추출) 자세, [§6.1](#61-채널-공통-식) 항력 공통식 |
| **5** | `6 DOF` · drag 내부 2종 | [§4](#4-자세-표현과-추출) 자세, [§6](#6-공력-항력) 항력 |
| **6** | 측정층 — Position and Orientation / RPM Measure / Thrust Measure / Current Measure / Battery | [§5.3](#53--계수-값--두-쌍을-섞으면-91배-틀린다) 단위 규약 규명 |

> 슬라이드 4·5는 `6 DOF` 개요 화면을 공유하며, 항력 내부 블록만 다르다.

---

## 1. 좌표계와 규약

### 1.1 월드 프레임 — ⚠ z축이 위다

`Mechanism Configuration` 블록의 중력 벡터:

$$\mathbf g_W = [0,\text{ }0,\text{ }-9.80665]^{\top}\text{ }\mathrm{m/s^2}$$

중력이 $-z$이므로 **$+z$가 위**인 ENU 계열 규약이다.

> **주의**: Beard(2008), Bouabdallah(2007) 등 항공 문헌 다수는 **NED**(z 아래)를 쓴다.
> 식을 그대로 옮기면 중력항·추력항 부호가 뒤집힌다.
> 본 문서는 **모델과 일치하도록 z-up을 유지**하며, 인용 시 이 차이를 명시할 것.

### 1.2 오일러각 순서

$$R(\eta) = R_z(\psi) R_y(\theta) R_x(\phi)$$

$$R = \begin{bmatrix} c\psi c\theta & c\psi s\theta s\phi - s\psi c\phi & c\psi s\theta c\phi + s\psi s\phi \cr s\psi c\theta & s\psi s\theta s\phi + c\psi c\phi & s\psi s\theta c\phi - c\psi s\phi \cr -s\theta      & c\theta s\phi                    & c\theta c\phi \end{bmatrix}$$

($c\cdot = \cos$, $s\cdot = \sin$, $\phi$ = roll, $\theta$ = pitch, $\psi$ = yaw)

### 1.3 기호

| 기호 | 의미 | 단위 |
|---|---|---|
| $m$ | 총질량 (기체 + 짐) | kg |
| $I$ | CoM 기준 관성 텐서 | kg·m² |
| $\mathbf v_W,\text{ }\boldsymbol\omega$ | 월드 속도, 바디 각속도 | m/s, rad/s |
| $n_i$ | $i$번 프로펠러 회전속도 | **rev/s** |
| $T_i,\text{ }Q_i$ | $i$번 추력, 반토크 | N, N·m |
| $\ell$ | 모터 모멘트 암 | m |
| $\rho$ | 공기 밀도 | kg/m³ |
| $D$ | 프로펠러 직경 | m |

---

## 2. 플랜트 전체 구조

**PPT 슬라이드 1** — `Quadcopter` 서브시스템.

```
World Frame ──► 6 DOF ──► Body
                            ▲
        ┌───────────────────┼───────────────────┐
   Propeller 1~4      Wind and Drag          Load
   (추력·반토크)        (공력 항력)        (짐 + 지면접촉)
                            │
                        Gravity
                            │
                       Electrical  (배터리 + 모터 4)
```

### 2.1 위상이 왜 중요한가

| 다물체 동역학의 복잡성 원인 | 이 모델 |
|---|---|
| 여러 물체 간 재귀적 관성 전파 | 물체 **하나** (짐은 Weld = 자유도 0) |
| 구속조건 → 라그랑주 승수 | 구속 **없음** |
| 닫힌 루프 → 구속 안정화 | 루프 **없음** |

접촉·항력·추력은 전부 **구속이 아니라 외력 요소**(`External Force and Torque`,
`Spatial Contact Force`)다. 따라서 일반 다물체 기계장치가 전부 퇴화하고,
자유물체 하나의 운동방정식만 남는다.

---

## 3. 강체 6-DOF 운동방정식

### 3.1 병진

$$m \dot{\mathbf v}_W  =  R(\eta)\begin{bmatrix}0\cr 0\cr  \sum_{i=1}^{4} T_i\end{bmatrix} +  \begin{bmatrix}0\cr 0\cr -mg\end{bmatrix} +  \mathbf F_{\text{drag},W} +  \mathbf F_{\text{contact},W}$$

### 3.2 회전 (바디 프레임)

$$I \dot{\boldsymbol\omega}  +  \boldsymbol\omega\times(I\boldsymbol\omega) =  \boldsymbol\tau_{\text{prop}}  +  \boldsymbol\tau_{\text{drag}}$$

자이로 항 $\boldsymbol\omega\times(I\boldsymbol\omega)$가 붙는 이유 — 각운동량 $\mathbf L = I\boldsymbol\omega$는
바디 프레임에서 $I$가 상수라 바디로 쓰는데, 바디 프레임 자체가 회전하므로:

$$\left(\frac{d\mathbf L}{dt}\right)_W = \left(\frac{d\mathbf L}{dt}\right)_B + \boldsymbol\omega\times\mathbf L$$

> $I_{xx}\approx I_{yy}$이고 yaw rate가 작아(반토크 권한 최약) 실제 기여는 작다.
> 다만 식에는 유지한다.

### 3.3 관성 텐서 대각화 근거

추출된 관성곱(`ProductsOfInertia`):

```
plate_top  :  [-9.2e-13, -9.7e-13,  1.0e-10]
Arm 1~4    :  [ 0,        0,        0      ]
```

관성곱이 실질적으로 0 → $I = \mathrm{diag}(I_{xx}, I_{yy}, I_{zz})$로 단순화 가능.
**추정이 아니라 모델 파라미터에 근거한 단순화**다.

---

## 4. 자세 표현과 추출

**PPT 슬라이드 4·5** — `6 DOF` 서브시스템.

### 4.1 자유도 정의

| 블록 | 자유도 | 출력 |
|---|---|---|
| `Cartesian Joint` | 병진 3 | `px, vx, py, vy, pz, vz` |
| `Spherical Joint` | 회전 3 | `nRoll, nPitch, nYaw` |

`Spherical Joint`는 **내부적으로 쿼터니언을 적분**한다. 따라서 플랜트 자체에는
짐벌락이 없고, 오일러각은 **관측·제어 입력용 사후 변환**일 뿐이다.

### 4.2 경로 ① — `R to X-Y-Z Extrinsic` (World Frame 계열)

§1.2의 $R$을 역산한다:

$$\theta = \arcsin(-R_{31}), \qquad \phi = \mathrm{atan2}(R_{32}, R_{33}), \qquad \psi = \mathrm{atan2}(R_{21}, R_{11})$$

블록 배선과 정확히 일치한다:

| 블록 | 역할 |
|---|---|
| `Selector R(3,1)` → `Gain(-1)` → `Asin` | $\theta$ — 게인 $-1$이 $-s\theta$를 뒤집음 |
| `Selector R(3, 2:3)` → `Atan2` | $\phi$ |
| `Selector R([2 1], 1)` → `Atan2` → `Unwrap` | $\psi$ (연속화) |

### 4.3 경로 ② — `Roll Pitch` (Body Frame 계열, 특이점 우회)

추력축 단위벡터로 직접 계산한다:

$$\phi_a = \mathrm{atan2}\left(u_3,\text{ }\sqrt{u_1^2+u_2^2}\right) \qquad (\mathbf u = R\text{의 y축})$$

$$\theta_a = -\arcsin\left(\frac{u_3 / |\mathbf u|}{\cos\phi_a}\right) \qquad (\mathbf u = R\text{의 x축})$$

$-1$ 게인의 블록 이름이 그대로 `Flip sign for -x axis`다.

### 4.4 각속도 — 워시아웃 미분

$$n_{\text{Roll}}(s) = \frac{s}{0.01 s+1} \phi_a(s), \qquad n_{\text{Pitch}}(s) = \frac{s}{0.01 s+1} \theta_a(s)$$

순수 미분이 아니라 **1차 필터가 걸린 미분**(시정수 0.01 s)이다.
고주파 노이즈 증폭을 막는 표준 처리.

---

## 5. 프로펠러 추력과 반토크

**블록**: Simscape Driveline `Aerodynamic Propeller`, `parameterization = Constant`, 2엽.

### 5.1 블록 공식 (MathWorks 공식 문서)

$$T = k_T \rho D^4 \varepsilon n\sqrt{n^2 + n_{\text{thr}}^2} \quad\longrightarrow\quad k_T \rho D^4 n^2 \quad (n \gg n_{\text{thr}})$$

$$Q = \frac{k_P}{2\pi} \rho D^5 n\sqrt{n^2 + n_{\text{thr}}^2} \quad\longrightarrow\quad \frac{k_P}{2\pi} \rho D^5 n^2$$

- $n$ = **rev/s** (블록이 $\omega = 2\pi n$으로 정의)
- $\varepsilon = \pm 1$ — 프로펠러 회전 방향. 대각 쌍이 반대
- $n_{\text{thr}}$ — 0 근처 특이점 방지 평활항

### 5.2 유입속도 반영

`Sensing Va` 서브시스템이 `Transform Sensor`(NonRotatingFollower, `SenseZDot=on`)로
축방향 유입속도를 측정해 블록에 공급한다 → **상승·하강 시 추력 변화가 반영된다.**

> 단순 $T = k\omega^2$ 모델을 인용하면 이 부분을 설명할 수 없다.

### 5.3 ⚠ 계수 값 — 두 쌍을 섞으면 91배 틀린다

$$\left(\frac{30}{\pi}\right)^2 = 91.19  \approx  91.3\text{ }\text{(실측 보정비)}$$

$30/\pi = 9.5493$은 **rad/s ↔ rpm** 변환 상수다. 프로펠러 라이브러리 내부에
`Gain1 = 30/pi`가 있고 그 출력 포트 이름이 `rpm`이며, PPT 슬라이드 6의
`RPM Measure` 스코프가 `m.Prop1~4.w`를 **`rpm 1~4`** 라벨로 받는다.

즉 `m.Prop.w` 채널은 rad/s가 아니라 **rpm**이고, 그래서 두 쌍이 공존한다:

| | $k_T$ / $C_t$ | 호버 회전속도 | 추력/모터 |
|---|---|---|---|
| **모델 내부 쌍** | $k_T = 9.79$ | 634 rpm | 5.58 N |
| **물리 현실 쌍** | $C_t = 0.1072$ | 6057 rpm (= 634 rad/s) | 5.57 N |

**두 쌍 모두 올바른 추력을 준다. 절대 섞어 쓰지 말 것.**

- 보고서·논문에는 **물리 쌍**을 쓴다 — APC 10×4.5MR 실측 역산이 $C_t = 0.1068\text{–}0.1098$로
  1% 내 일치하기 때문
- 모델의 $k_T = 9.79$는 **속도 단위 불일치의 제곱을 흡수한 보정값**이라고 각주로 밝힌다
- 모델은 **건드리지 않는다** — 두 오차가 정확히 상쇄되어 출력은 옳다 (앵커 불변 규칙)

### 5.4 호버 검산

$$T_{\text{hover}} = \frac{m g}{4} = \frac{2.2726 \times 9.80665}{4} = 5.5716\text{ }\mathrm N$$

$$n = \sqrt{\frac{T}{C_t \rho D^4}} = \sqrt{\frac{5.5716}{0.1072 \times 1.225 \times 0.254^4}} = 100.96\text{ }\mathrm{rev/s} = 634.4\text{ }\mathrm{rad/s}$$

$$Q = C_q \rho D^5 n^2 = 0.01517 \times 1.225 \times 0.254^5 \times 100.96^2 = 0.200\text{ }\mathrm{N\cdot m}$$

→ 튜닝 세션 실측 평형(634 rad/s, 토크 클램프 0.2 N·m)과 **일치**. ✅

---

## 6. 공력 항력

**PPT 슬라이드 3·4·5** — `Wind and Drag / Aerodynamic Drag`.
이 모델에서 **유일하게 `.slx`에 식이 그대로 보이는 부분**이다 (Simulink 기본 블록 조립).

### 6.1 채널 공통 식

$$f = - \frac{\rho_{\text{air}}}{2}\cdot A\cdot C_d\cdot v^2 \cdot \mathrm{sign}(v)$$

블록 다이어그램에서 5입력 `Product`에 $\rho/2$ · `area` · `Cd` · `u²` · `sign`이
나란히 들어가고 마지막에 `Gain(-1)`이 걸린다.

### 6.2 계수·면적

| 채널 | $C_d$ | $A$ [m²] | 파라미터 |
|---|---|---|---|
| X | 0.35 | 0.0875 | `qd_area.YZ` |
| Y | 0.35 | 0.0900 | `qd_area.XZ` |
| Z | 0.6 | 0.2560 | `qd_area.XY` |
| Roll | 0.2 | 0.512 | `XY × 2` |
| Pitch | 0.2 | 0.512 | `XY × 2` |
| Yaw | 0.2 | 0.256 | `XY` |

### 6.3 상대속도와 yaw 회전

$$\mathbf v_{\text{rel}} = \mathbf v_{\text{chassis}} - \mathbf v_{\text{env}}$$

`Body to World` 블록이 **yaw로 면적·계수를 회전**시킨다:

$$\psi  \longmapsto  (A_x,\text{ }C_{d,x},\text{ }A_y,\text{ }C_{d,y})$$

즉 유효 전면적이 기수 방위에 따라 x/y 축으로 섞인다.

### 6.4 인가 지점 — 짐 유무로 전환

| 조건 | 압력중심 프레임 |
|---|---|
| `Load.status == 0` | `Transform Center of Pressure With Load` |
| `Load.status < 0` | `Transform Center of Pressure No Load` |

### 6.5 환경 — 고도별 돌풍

`Environment` 블록: 고도 2 m 이하에서 0, 6 m 이상에서 최대 풍속으로 선형 증가.

---

## 7. 모터 전기동역학

**PPT 슬라이드 2** — `Electrical` 서브시스템.

### 7.1 구성

배터리(7.6 V × 3) → `Motor & Drive (System Level)` × 4 → 회전 다물체 인터페이스 → 프로펠러.
`Convective Heat Transfer`로 발열, `Sensor M1~4`로 전류 계측.

### 7.2 토크–파워 엔벨로프

`torque_speed_param = torque_power` 모드:

$$\tau_{\max} = 0.8\text{ }\mathrm{N\cdot m}, \qquad P_{\max} = 160\text{ }\mathrm W, \qquad T_c = 0.02\text{ }\mathrm s, \qquad b = 10^{-7}\text{ }\mathrm{N\cdot m/(rad/s)}$$

> ⚠ 블록의 `T_t` / `w_t` 배열은 **이 모드에서 사용되지 않는 죽은 파라미터**다
> (튜닝 세션 실측 확인). 보고서에 포함하지 말 것.

### 7.3 근사 등가 모델 (Gazebo / C++ 독립 실행용)

$$J \dot\omega = \tau - Q - b \omega, \qquad J = 1.26\times10^{-5}\text{ }\mathrm{kg\cdot m^2}\text{ }(T_c = 0.02 \mathrm s\text{ }\text{역산})$$

$$V = \text{duty}\times V_{\text{batt}}, \qquad \text{duty} = \frac{|u|}{u_{\max}}$$

---

## 8. 제어기 전달함수

**PPT 슬라이드 2** — PID 블록 대화상자 캡처.

### 8.1 공통 형식 — 병렬형 + 필터드 미분

$$C(s) = k_p + k_i \frac{1}{s} + k_d \frac{N}{1 + N\frac{1}{s}} =  k_p + \frac{k_i}{s} + \frac{k_d N s}{s + N}$$

**모델의 모든 PID가 이 형식을 공유한다** — 모터·자세·yaw·고도·위치 전부.

### 8.2 측정 필터

$$H_{\text{meas}}(s) = \frac{1}{f_{\text{meas}} s + 1}, \qquad e = r - H_{\text{meas}}(s) y$$

모터 루프의 경우 블록 다이어그램에 $\frac{1}{f_{spd} s+1}$로 그대로 보인다.

### 8.3 채널별 게인

| 채널 | $k_p$ | $k_i$ | $k_d$ | $N$ | $f_{\text{meas}}$ | 출력 한계 |
|---|---|---|---|---|---|---|
| 모터 | 0.00375 | 4.5e-4 | 0 | 10000 | 0.001 | 0.25 |
| 자세 | $-85 s_T s_A$ | $-10 s_T s_A$ | $-127.5 s_T s_A$ | 2500 | 0.01 | 800 |
| yaw | $15 s_Q$ | $1.5 s_Q$ | $4 s_Q$ | 100 | 0.01 | 20 |
| 고도 | $0.5 s_T s_Z$ | $0.1 s_T s_Z$ | $0.15 s_T s_Z$ | 1000 | 0.05 | 10 |
| 위치 | 8 | 0.04 | 3.2 | 100 | 0.005 | — |

모터는 $k_d = 0$이므로 실질 **PI 제어**다.

### 8.4 자세 게인이 음수인 이유

실측 플랜트 식별(±5° 사각파 가진 후 회귀):

$$\ddot y = b u + c \dot y, \qquad b = -0.0296$$

제어 출력 → 각가속 경로의 이득이 **음수**다. 따라서 음수 게인이 물리적으로 옳다.
부호를 "고치면" 즉시 발산한다.

### 8.5 정규화 스케일

$$s_T = \frac{K_{\text{thrust,ref}}}{K_{\text{thrust}}}, \qquad s_Q = \frac{K_{\text{drag,ref}}}{K_{\text{drag}}}$$

$$s_A = 0.75 + 0.25 \min(m_{\text{pkg}}, 2), \qquad s_Z = 0.56 + 0.44 \min(m_{\text{pkg}}, 2)$$

$m_{\text{pkg}} = 1$ kg에서 모든 스케일이 정확히 1이 되도록 앵커링돼 있다.

---

## 9. 질량·관성 상수

### 9.1 구성 요소 (CAD 실측, `InertiaType = Custom`)

| 요소 | 질량 | $I_{xx}, I_{yy}, I_{zz}$ [kg·m²] | 배치 |
|---|---|---|---|
| `plate_top` | 0.0317292 kg | 2.778e-5, 2.758e-5, 5.534e-5 | z = −26 mm |
| `plate_bottom` | 0.0317292 kg | 2.778e-5, 2.758e-5, 5.534e-5 | z = −38 mm |
| `Arm` × 4 | 0.0589152 kg | 2.632e-4, 2.609e-4, 1.231e-5 | (±12, ±58, −38) mm 등 |
| `Flight Computer` | **638 g** | 기하 계산 (60×60×15 mm) | 중앙 |
| 모터 `Housing` × 4 | 51 g | 7.044e-6, 7.044e-6, 6.069e-6 | 암 끝 |
| 모터 `Cap` × 4 | 0.930841 g | 1.322e-8, 2.225e-8, 1.322e-8 | 암 끝 |
| `Package` | **1.0 kg** (정확히) | 기하 계산 | 부착면 z = −12 mm |

`Package`는 $\rho_{\text{pkg}} = 1/V_{\text{pkg}}$로 정의되어 크기와 무관하게 질량이 1 kg으로 고정된다.

### 9.2 합성 물성

$$\ell = \frac{0.225}{\sqrt2} = 0.15910\text{ }\mathrm m \quad \text{(FX450 휠베이스 450 mm, X-쿼드)}$$

| 짐 | $m$ [kg] | $z_{cg}$ [m] | $I_{xx}$ | $I_{yy}$ | $I_{zz}$ |
|---|---|---|---|---|---|
| **1 kg** | 2.2726 | −0.03175 | 1.711e-2 | 1.716e-2 | 2.124e-2 |
| 0 kg | 1.2726 | +0.00773 | 9.334e-3 | 9.384e-3 | 1.797e-2 |

합성식은 평행축 정리다 (`qc_phys()` 함수). 로터 항을 제외하고 계산하면
실측 비행구성 관성을 **0.3% 이내로 재현**한다. ✅

### 9.3 페이로드 결합 — ⚠ 강체다

**짐은 `Weld Joint`로 강체 부착되어 있다. 케이블 현수가 아니다.**

관측되는 1.8 Hz 모드는 케이블 진자가 아니라, 짐 때문에 CG가 추력면 아래
**8.1 cm**로 내려간 **강체 전체의 저중심 모드**다:

$$\omega^2 = \frac{g m L}{m L^2} = \frac{g}{L} \quad\Rightarrow\quad L = \frac{g}{\omega^2} = 8.1\text{ }\mathrm{cm}$$

**증거 3종** (전부 실측):

| 지문 | 관측 | 함의 |
|---|---|---|
| 질량 | 1 → 2 kg에도 1.75 Hz 불변 | 질량이 약분됨 = 중력 진자 |
| 크기 | 0.10/0.14/0.20 m 큐브에도 1.80 Hz 불변 | 중심점 용접 → CG 오프셋 불변 |
| 결합 | 조인트 전수조사 결과 Weld | 분리 스윙 아님 |

> **인용 주의**: 현수하중(cable-suspended) 논문을 인용하면 모델에 없는
> 2자유도 구면진자를 주장하게 된다. 강체 부착 계열을 인용할 것.

**부록거리**: CG가 추력면 *위*로 갔을 때의 과거 발산 사건은 **같은 중력 모멘트의 부호 반전**이다.
$\Delta z > 0$(아래) → 안정 진동, $\Delta z < 0$(위) → 도립 발산. 한 식으로 두 사건이 설명된다.

---

## 10. 조립된 최종 EOM

### 10.1 병진

$$m \dot{\mathbf v}_W = R(\eta)\begin{bmatrix}0\cr 0\cr  C_t \rho D^4 \sum_{i=1}^4 n_i^2\end{bmatrix} -\begin{bmatrix}0\cr 0\cr mg\end{bmatrix} -\frac{\rho}{2} \begin{bmatrix} C_{d,x}A_x v_x|v_x| \cr C_{d,y}A_y v_y|v_y| \cr C_{d,z}A_z v_z|v_z| \end{bmatrix}$$

### 10.2 회전

$$I \dot{\boldsymbol\omega} + \boldsymbol\omega\times(I\boldsymbol\omega) = \begin{bmatrix} \ell C_t\rho D^4 (-n_1^2 + n_2^2 - n_3^2 + n_4^2) \cr \ell C_t\rho D^4 (n_1^2 + n_2^2 - n_3^2 - n_4^2) \cr C_q\rho D^5 (-n_1^2 + n_2^2 + n_3^2 - n_4^2) \end{bmatrix} -\frac{\rho}{2} \begin{bmatrix} C_{d,\phi}A_\phi \omega_x|\omega_x| \cr C_{d,\theta}A_\theta \omega_y|\omega_y| \cr C_{d,\psi}A_\psi \omega_z|\omega_z| \end{bmatrix}$$

> **믹서 부호와 유효 모멘트 암은 골든 트레이스로 확정됐다** (2026-08-18).
>
> 각 축을 강하게 여기하는 전용 런에서 프로펠러 차동량과 **각가속도**를 최소제곱 대조했다.
>
> | 여기 런 | 축 | 측정 최대 | 예측 최대 | 상관 | R² | 유효계수 |
> |---|---|---|---|---|---|---|
> | +x 이동 | pitch | 20.09 | 19.03 | **+0.916** | 0.839 | 0.0929 m |
> | +y 이동 | roll | 20.40 | 19.52 | **+0.918** | 0.844 | 0.0932 m |
> | yaw 스텝 | yaw | 0.97 | 0.51 | **+0.722** | 0.521 | 1.09 (Q 배수) |
>
> roll과 pitch의 유효계수가 **0.4% 이내로 일치**한다 — 플랜트 대칭 자기검증.
>
> ⚠ **이전 판(정착 각도로 유추한 부호)은 틀렸다.** 일정 각도를 유지하는 구간은
> 순토크가 0에 가까워, 각도와 차동량의 관계로는 토크 부호를 알 수 없다.
> 각가속도와의 직접 상관이 옳은 검정이다 (뉴턴 법칙, 순간값).
> 부호를 반전하자 피치 상관이 −0.916에서 **+0.975**로 뒤집혔다.
>
> **유효 모멘트 암은 0.0930 m**로, 가정했던 X-쿼드 균등값 0.159 m가 아니다.

### 10.3 액추에이터

$$J \dot n_i = \tau_i - C_q \rho D^5 n_i^2 - b n_i, \qquad i = 1,\dots,4$$

### 10.4 상수 요약

$$\begin{aligned} m &= 2.2726\text{ }\mathrm{kg} & I &= \mathrm{diag}(1.711,\text{ }1.716,\text{ }2.124)\times10^{-2}\text{ }\mathrm{kg\cdot m^2} \cr \ell &= 0.0930\text{ }\mathrm m & \rho &= 1.225\text{ }\mathrm{kg/m^3} \cr D &= 0.254\text{ }\mathrm m & g &= 9.80665\text{ }\mathrm{m/s^2} \cr C_t &= 0.1072 & C_q &= 0.01517 \cr J &= 1.26\times10^{-5}\text{ }\mathrm{kg\cdot m^2} & b &= 10^{-7}\text{ }\mathrm{N\cdot m/(rad/s)} \end{aligned}$$

---

## 11. 미확정 항목

보고서에 **단정적으로 쓰면 안 되는** 항목들이다.
1·2번은 `diagnose/probe_prop_and_mixer.m` 실행(2026-08-18)으로 닫혔다.

| # | 항목 | 상태 | 확정 방법 |
|---|---|---|---|
| ~~1~~ | ~~믹서 차동 부호표~~ | ✅ **확정 (08-18)** | 골든 트레이스 각가속도 상관 +0.92 (3축 전용 런) → §10.2 반영 |
| ~~2~~ | ~~프로펠러 축 실제 회전속도~~ | ✅ **확정 (08-18)** | 호버 실측 **633.7 rpm**, 추력 5.55 N/모터 → §5.3 반영 |
| 3 | 자세 게인 스케일 법칙 | **판별 불가 (08-18)** | 2 kg 실험 완료 — 아래 §11.2 참조 |
| 4 | 고도 채널 스케일 근거 (예측 0.748 vs 실측 0.56) | 미해명 | — |
| ~~5~~ | ~~로터 모멘트 암 가중치~~ | ✅ **부분 확정 (08-18)** | 유효계수 0.0930 m 확정. 개별 모터 암은 **식별 불가** (§11.3) |

### 11.0 신규 발견 — 로터 배치가 X-쿼드가 아니다

CAD의 암 장착 Transform을 각도로 환산하면:

| | 암 베이스 [mm] | 각도 | roll 성분 $\sin	heta$ | pitch 성분 $\cos	heta$ |
|---|---|---|---|---|
| M1 | (12, 58) | 78.3° | +0.979 | +0.203 |
| M2 | (58, −12) | −11.7° | −0.203 | +0.979 |
| M3 | (−58, 12) | 168.3° | +0.203 | −0.979 |
| M4 | (−12, −58) | −101.7° | −0.979 | −0.203 |

인접 간격은 정확히 90°지만, **축에서 −11.7° 회전한 "+" 배치**다. 45° X-쿼드가 아니다.

**관성에는 영향 없다.** 반지름 $R$인 로터 4개가 90°씩 떨어져 있으면 회전각과 무관하게
$\sum y_i^2 = 2R^2$ 이므로, `qc_phys()`의 `m_rot * r_arm^2` ($r_{arm} = R/\sqrt2$) 은 그대로 옳다.

**모멘트 암에는 영향 있다.** §10.2는 네 로터가 균등하게 기여한다고 가정하지만,
위 기하로는 roll이 주로 M1·M4, pitch가 주로 M2·M3에서 나온다.
실측 차동분도 이 비대칭과 정합한다(주도 쌍 3.7 : 보조 쌍 2.5).

> 부호 배열은 이 기하와 **일치하므로 §10.2의 부호는 유효하다.**
> 미확정인 것은 **가중치**뿐이다. 골든 트레이스 정량 대조에서 드러날 것이다.

**교차결합으로 본 반증**: 위 기하 가중치를 그대로 쓰면 roll 명령이 pitch를
66% 크기로 끌고 온다(적분기로 계산). 그런데 실측 교차결합은 훨씬 작다 —
+y 이동에서 roll −0.25° 대비 pitch +0.04°로 **16%** 수준이다.

즉 **실제 로터 위치는 암 베이스 방향보다 X 배치에 더 가깝다.** 암이 바깥으로
벌어지면서 각도가 45°쪽으로 이동하는 것으로 보인다. 따라서 §10.2의 균등 가중치가
현재로선 더 나은 근사이며, `plant_sim.py`도 균등을 기본값으로 둔다
(`use_geometric_lever=False`). 정밀 확정은 모터 장착점 좌표 실측이 필요하다.

### 11.2 자세 게인 스케일 법칙 — 판별 불가로 결론

세 후보를 2 kg에서 대조했다 (`diagnose/probe_2kg_law.m`, 고도 배율 1.44 고정).

| 후보 | $s_A$ | 추종 cm | 오버 cm | 꼬리 ° | z피크 cm | 자세피크 ° |
|---|---|---|---|---|---|---|
| A 현행 1차식 | 1.250 | 3.96 | 7.6 | 4.77 | 1.9 | 13.5 |
| B 물리 $I/\sqrt{m}$ | 1.077 | 4.04 | 7.8 | 4.83 | 1.9 | 13.6 |
| C 관성비 $I/I_{ref}$ | 1.293 | 4.01 | 7.7 | 4.73 | 1.8 | 13.5 |

**게인이 20% 달라져도 성능은 2% 안에서 움직인다.** 세 후보가 구별되지 않는다.
(A행은 18차 기록 3.96 / 7.64 / 1.86 을 소수점까지 재현 — 하네스 회귀 무결.)

#### 그래서 무엇을 알게 되었나

판별에 실패한 이유가 저질량 때와 **정반대**다.

| 구간 | 실패 원인 | 성질 |
|---|---|---|
| 0 kg | 게인 $3	imes10^{-7}$ 섭동에 추종 34% 변동 | **과민** (혼돈) |
| 2 kg | 게인 20% 차이에 성능 2% 변동 | **둔감** (평탄) |

즉 **스케일 법칙의 형태가 결과를 좌우하는 구간은 저질량뿐이고, 그 구간은
측정으로 구별할 수 없다.** 반대로 0.5~2 kg에서는 어떤 법칙을 쓰든 무방하다 —
중요한 것은 **질량 보정이 존재한다는 사실 자체**이지 그 함수 형태가 아니다.

#### 실무 결론

- 현행 1차식을 **유지**한다. 대체할 근거가 없다.
- 이 항목은 "미확정"이 아니라 **"판별 불가, 사유 규명됨"** 으로 종결한다.
- 보고서에는 스케일 법칙을 **경험적 보정**으로 기술하고,
  물리적 유도(관성비/$I/\sqrt{m}$)를 정답처럼 주장하지 않는다.

### 11.1 골든 트레이스 — 실행 결과 (2026-08-18)

`diagnose/golden_plant_export*.m` (Simscape 내보내기) + `compare_golden.py` (대조).

#### 설계 — 왜 궤적 재생이 아니라 잔차 대조인가

처음에는 Simscape가 기록한 $n(t)$를 독립 적분기에 먹여 궤적을 길게 재생하려 했으나
**실패했다**. 쿼드로터 자세는 감쇠가 없는 이중적분기라, 호버 로그의 미세한 추력
불균형(0.08 N)만으로도 개루프 재생이 수 초 만에 눕는다(위치 오차 16 m).
폐루프에서는 제어기가 그걸 계속 잡아주지만 재생에는 제어기가 없다.

그래서 **매 시점 가속도를 직접 대조**하는 방식으로 바꿨다. 불안정성을 적분하지
않으므로 식 자체만 시험한다.

$$a_{	ext{측정}} = rac{d}{dt} v_{	ext{Simscape}}, \qquad a_{	ext{예측}} = rac{R [0,0,\sum T] + [0,0,-mg] + F_{	ext{drag}}}{m}$$

#### 병진 (§10.1) — 검증됨

| 축 | 측정 최대 | 예측 최대 | 잔차 RMS | 잔차 최대 |
|---|---|---|---|---|
| x | 2.2899 | 2.3001 | 0.1505 | 0.3881 |
| y | 0.0738 | 0.0713 | 0.0272 | 0.0857 |
| z | 0.0581 | 0.1081 | 0.0207 | 0.0725 |

단위 m/s². 주 운동축인 x에서 **진폭이 0.4% 이내로 일치**한다.
호버 구간 z 잔차 평균은 $+0.00136$ m/s²로 사실상 0 — 추력·질량·중력이 정합한다.

#### 회전 (§10.2) — 검증됨 (부호 정정 후)

§10.2의 표 참조. 세 축 모두 전용 여기 런에서 양의 상관($+0.72 \sim +0.92$),
roll/pitch 유효계수가 0.4% 이내 일치.

#### ⚠ 계수 함정을 실제로 밟았다

첫 재생에서 위치 오차가 35 m 나왔다. 원인은 Simscape의 회전속도(633.7 rpm)를
**물리 현실 쌍**($C_t = 0.1072$)에 먹인 것 — §5.3이 경고한 바로 그 실수다.
모델 계수($k_T = 9.79$)로 바꾸자 호버 추력이 필요량과 0.007% 일치했다.
**이 문서의 §5.3 경고는 실제로 사람을 잡는다.**

### 11.3 개별 모터 모멘트 암 — 식별 불가

$Ilpha_{	ext{pitch}} = \sum_i x_i T_i$ 로 4개 암을 동시 추정해 보았다.

| | M1 | M2 | M3 | M4 |
|---|---|---|---|---|
| 식별값 [m] | 0.0155 | 0.1531 | −0.0537 | −0.1391 |
| CAD 기하 | 0.0456 | 0.2203 | −0.2203 | −0.0456 |

**신뢰할 수 없다.** 조건수가 $1.1	imes10^4$ 이고, 4파라미터 적합의 $R^2$(0.8404)가
1파라미터 적합(0.839)과 **차이가 없다**. 호버에서 네 추력이 거의 같아 개별 분리가
안 되기 때문이다.

결정되는 것은 **유효계수 하나**뿐이다. 개별 암을 보려면 모터를 독립적으로
가진해야 하며, 이는 폐루프 비행이 아니라 개루프 모터 명령이 필요하다.

## 참고 문헌

### 강체 EOM

- **Luukkonen, T.** (2011). *Modelling and control of quadcopter*. Aalto University.
  [PDF](https://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf) — 뉴턴-오일러와
  오일러-라그랑주로 동일 결과를 두 번 유도. 식이 빠짐없이 있음.
- **Bouabdallah, S.** (2007). *Design and control of quadrotors with application to autonomous flying*.
  PhD thesis, EPFL (THÈSE N° 3727). [PDF](https://infoscience.epfl.ch/record/95939/files/EPFL_TH3727.pdf)
  — 공력 항력 항을 명시적으로 포함한 유도. §6 대응.
- **Beard, R. W.** (2008). *Quadrotor Dynamics and Control Rev 0.1*. Brigham Young University.
  [BYU ScholarsArchive](https://scholarsarchive.byu.edu/facpub/1325) — 좌표계 정의가 가장 상세. §1 대응.
- **Mahony, R., Kumar, V., & Corke, P.** (2012). Multirotor aerial vehicles: Modeling, estimation,
  and control of quadrotor. *IEEE Robotics & Automation Magazine*, 19(3), 20–32.

### 프로펠러·액추에이터

- **MathWorks**. *Aerodynamic Propeller* block reference (Simscape Driveline).
  [문서](https://www.mathworks.com/help/sdl/ref/aerodynamicpropeller.html) — §5.1 식의 1차 출처.
- **Bangura, M., & Mahony, R.** (2012). Nonlinear dynamic modeling for high performance control
  of a quadrotor. *ACRA 2012* — 로터 유입 반영 근거. §5.2 대응.
- **Brandt, J. B., & Selig, M. S.** (2011). Propeller performance data at low Reynolds numbers.
  *AIAA Aerospace Sciences Meeting* — UIUC 프로펠러 DB. $C_t$/$C_q$ 실측 출처.
- **Quan, Q.** (2017). *Introduction to Multicopter Design and Control*. Springer — 6장.
  액추에이터까지 식으로 완비된 교재.

### 페이로드 (강체 부착)

- **Pounds, P. E. I., Bersak, D. R., & Dollar, A. M.** (2012). Stability of small-scale UAV
  helicopters and quadrotors with added payload mass under PID control. *Autonomous Robots*,
  33(1–2), 129–142. [Springer](https://link.springer.com/article/10.1007/s10514-012-9280-5)
  — §9.3 대응. PID 제어 하 페이로드 질량 변화의 안정성 한계.
- **Mellinger, D., Lindsey, Q., Shomin, M., & Kumar, V.** (2011). Design, modeling, estimation
  and control for aerial grasping and manipulation. *IROS 2011*, 2668–2673 — 짐 → $m, I, CG$ 변화.
- **Villa, D. K. D., Brandão, A. S., & Sarcinelli-Filho, M.** (2020). A survey on load transportation
  using multirotor UAVs. *J. Intelligent & Robotic Systems*, 98, 267–296.
  [Springer](https://link.springer.com/article/10.1007/s10846-019-01088-w) — 강체 부착 vs 케이블 현수 비교.

---

## 서술 전략 (보고서 작성 시)

> 유도는 표준을 인용하고, **상수와 검증만 자체 기여**로 내세운다.

예시 문장:

> 본 연구의 동역학 모델은 [Luukkonen 2011], [Bouabdallah 2007]의 표준 뉴턴-오일러 정식화를
> 따르되, 물성 상수는 CAD 실측에서 산출하고 호버 평형(§5.4) 및 관성 재현도 0.3%(§9.2)로
> 검산하였다. 좌표계는 Simscape 모델과 일치하도록 z-up 규약을 사용하며, 이는 [Beard 2008]의
> NED 표기와 z축 부호가 반대이다.

이렇게 쓰면 질문이 "어디서 베꼈나"가 아니라 "이 숫자가 어디서 나왔나"로 옮겨가고,
그건 전부 답할 수 있다.
