# control_seoungjin

## controller/
[Quadcopter-Drone-Model-Simscape](https://github.com/mathworks/Quadcopter-Drone-Model-Simscape.git) (MathWorks 공식)를 서브모듈로 가져와서 파라미터(질량, 관성, PID 게인 등) 튜닝을 진행하는 폴더입니다. 실제 컨트롤러에 적용하기 전에 여기서 시뮬레이션으로 값을 검증합니다.

- **파라미터**: `Scripts_Data/quadcopter_package_parameters.m`에 평문 스크립트로 정의되어 있습니다 (`drone_mass`, `propeller.*`, `qc_motor.*`, PID 게인 등). 실제 기체(FX450)에 맞는 값으로 교체 예정입니다.
- **회전관성**: 숫자로 직접 입력하는 값이 아니라 `CAD/Geometry/`의 CAD 형상(질량 분포)에서 자동 계산됩니다.
- **CAD 교체 (FX450 프레임 반영 완료)**: `CAD/Geometry/`의 제네릭 형상을 FX450 실측 프레임 CAD(SolidWorks 원본 → Fusion 360 경유 STEP 변환)로 교체했습니다.
  | FX450 부품 | 교체된 파일 |
  |---|---|
  | `arm` | `quadcopter_drone_arm.stp` |
  | `Base Top` | `quadcopter_drone_plate_top.stp` |
  | `Base bottom` | `quadcopter_drone_plate_bottom.stp` |

  모터/모터캡/프로펠러 형상은 당분간 MathWorks 제네릭 CAD를 그대로 사용합니다.

- **PID 튜닝 위치**: `Models/quadcopter_package_delivery.slx`의 `Maneuver Controller` 서브시스템. 게인 값 자체는 `Scripts_Data/quadcopter_package_parameters.m`의 `kp_position/kp_attitude/kp_yaw/kp_altitude/kp_motor` 등 변수. 스크립트에서 값을 바꾸거나, Simulink에서 서브시스템 열어 PID 블록의 "Tune" 기능으로 조정 가능.

- **모델 입력 형식**: `Maneuver Controller` 안 1-D Lookup Table 블록 4개(x/y/z/yaw)가 시뮬레이션 시간을 입력으로 받아 아래 워크스페이스 변수를 보간합니다.
  | 변수 | 형식 | 의미 |
  |---|---|---|
  | `timespot_spl` | (N,) | 시간 breakpoint [s] |
  | `spline_data` | (N, 3) | x, y, z 위치 [m] |
  | `spline_yaw` | (N,) | yaw [rad] |

## waypoints_to_maneuver_input.py
`path_time.ipynb`의 `plan_waypoints`(waypoint를 하나씩 순서대로 최소시간 7차 다항식으로 잇는 궤적 계획)를 재사용해서, 위 `Maneuver Controller` 입력 형식(`timespot_spl`, `spline_data`, `spline_yaw`)에 맞는 `.mat` 파일을 생성하는 스크립트입니다. yaw는 진행 방향(속도 벡터의 heading)으로 자동 계산합니다.

```
python waypoints_to_maneuver_input.py   # trajectory.mat 생성
```
```matlab
load('trajectory.mat')                  % timespot_spl, spline_data, spline_yaw 로드
sim('quadcopter_package_delivery')
```

## path_time.ipynb
경로(x, y, z)에 시간을 부여해서 PID 컨트롤러에 넣을 feed(position/velocity/acceleration setpoint)를 생성하는 노트북입니다. 파이프라인은 다음과 같습니다.

```
path(x, y, z) → arc-length 재매개변수화 → 곡률 κ 계산 → velocity profile v(s) → t(s) 변환 → PID feed
```

1. **arc-length 재매개변수화** (`reparameterize_by_arc_length`) — 원본 경로 점들을 3차 스플라인(CubicSpline)으로 보간해서, 파라미터 간격이 아니라 호 길이(거리) 기준으로 등간격이 되도록 다시 샘플링합니다.
2. **곡률 계산** (`compute_curvature_and_kN`) — 재매개변수화된 경로에서 TNB(Frenet-Serret) 프레임을 이용해 각 지점의 곡률 κ와 κ·N(원심가속도 방향 벡터)을 구합니다.
3. **속도 프로파일 생성** (`generate_velocity_profile`) — `v_max`(최고속도), `a_max`(최고가속도), `j_max`(최고저크) 제약과, 곡률 기반 코너링 속도 제한(`v ≤ √(a_max / κ)`)을 forward-backward scan으로 반영해 각 지점의 속도를 계산합니다.
4. **시간 변환 + 자동 연장** (`generate_pid_reference`) — 속도 프로파일을 적분해 각 지점에 실제 시간 `t(s)`를 부여합니다. 목표 `total_time`이 물리적 제약상 불가능하면 경고를 띄우고 시간을 자동으로 늘립니다.
5. **PID feed 출력** — 최종적으로 `pos`, `vel`, `acc` (3×N 배열)를 시간에 맞춰 만들어 PID 컨트롤러의 setpoint/feedforward 입력으로 사용합니다.

추가로 `plan_waypoints`/`plan_trajectory`는 waypoint 사이 구간을 7차 다항식으로 이어서, 시작/종료 속도·가속도·저크(jerk)까지 경계조건으로 맞추고 `v_max`/`a_max`/`j_max`/`snap_max` 제약을 만족하는 궤적을 생성합니다.
