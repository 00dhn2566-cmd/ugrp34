# control_seoungjin

## 요구사항 (Requirements)

**MATLAB / Simulink** (`sample/run_sample_sim.sh` 및 `controller/` 시뮬레이션 실행용)
- MATLAB R2025b 이상
- Simulink
- Simscape, Simscape Multibody, Simscape Electrical
- **Simscape Driveline** — `Aerodynamic Propeller` 블록(`sdl_lib`)에 필요. 없으면 `Quadcopter/Propeller 1~4` 서브시스템에서 라이브러리 로드 오류가 남. Add-On 관리자에서 설치.
- (선택) Simulink Control Design — PID 블록의 "Tune" 기능 사용 시

**Python** (`path_time.py`, `sample/*.py` 실행용)
- numpy
- scipy (`scipy.interpolate`, `scipy.io`)
- matplotlib (`sample/verify_sample_trajectory.py`만 해당)

**CAD 툴** (FX450 CAD 교체/재작업 시)
- SolidWorks (원본 `.SLDPRT`/`.SLDASM`) 또는 Fusion 360 (STEP 내보내기)

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

- **⚠️ 서브모듈 받는 법 (zip 배포)**: `controller/Quadcopter-Drone-Model-Simscape`는 아직 MathWorks 공식 저장소 주소를 그대로 가리키고 있어서, FX450 CAD로 교체한 커밋은 그 원격에 없습니다. 그래서 `git submodule update --init`으로는 안 받아지고, zip으로 따로 배포합니다.
  1. 배포된 zip 파일을 받는다.
  2. 저장소 루트 기준 **`control_seoungjin/controller/Quadcopter-Drone-Model-Simscape/`** 폴더 안에 압축을 풀어 덮어쓴다. (기존 폴더 내용 위에 그대로 압축 해제)
  3. `git submodule update --init`은 실행하지 않는다 (실행하면 zip으로 넣은 내용이 지워지고 MathWorks 원본으로 되돌아갈 수 있음).

- **PID 튜닝 위치**: `Models/quadcopter_package_delivery.slx`의 `Maneuver Controller` 서브시스템. 게인 값 자체는 `Scripts_Data/quadcopter_package_parameters.m`의 `kp_position/kp_attitude/kp_yaw/kp_altitude/kp_motor` 등 변수. 스크립트에서 값을 바꾸거나, Simulink에서 서브시스템 열어 PID 블록의 "Tune" 기능으로 조정 가능.

- **모델 입력 형식**: `Maneuver Controller` 안 1-D Lookup Table 블록 4개(x/y/z/yaw)가 시뮬레이션 시간을 입력으로 받아 아래 워크스페이스 변수를 보간합니다.
  | 변수 | 형식 | 의미 |
  |---|---|---|
  | `timespot_spl` | (N,) | 시간 breakpoint [s] |
  | `spline_data` | (N, 3) | x, y, z 위치 [m] |
  | `spline_yaw` | (N,) | yaw [rad] |

## path_time.py
`path_time.ipynb`의 핵심 함수(arc-length 재매개변수화, 곡률, 속도 프로파일, `plan_waypoints` 등)만 검증/플롯 코드 없이 정리한 재사용 가능한 모듈입니다. `sample/`의 스크립트들이 이 모듈을 import해서 씁니다.

## sample/
FX450 CAD/모델 검증용 샘플 궤적 생성 및 Simscape 시뮬레이션 실행 폴더입니다.

- **`waypoints_to_maneuver_input.py`**: `path_time.py`의 `plan_waypoints`(waypoint를 하나씩 순서대로 최소시간 7차 다항식으로 잇는 궤적 계획)를 재사용해서, `Maneuver Controller` 입력 형식(`timespot_spl`, `spline_data`, `spline_yaw`, 시각화용 `waypoints`)에 맞는 `trajectory.mat`을 생성합니다. yaw는 진행 방향(속도 벡터의 heading)으로 자동 계산합니다.
- **`verify_sample_trajectory.py`**: 위 궤적이 `v_max`/`a_max`/`j_max` 제약을 실제로 만족하는지 파이썬에서 검증하고 3D 경로 + 속도/가속도/저크 그래프(`sample_trajectory.png`)를 저장합니다. (참고: 이 제약은 x/y/z 축별로 적용되어서, 대각선 이동 시 벡터 크기 기준 속도/가속도가 축별 한계보다 커질 수 있습니다.)
- **`run_sample_sim.sh`**: 위 궤적 생성 → `controller/Quadcopter-Drone-Model-Simscape/`로 복사 → MATLAB 배치 모드로 `run_sample_sim.m` 실행까지 한 번에 처리합니다.
  ```bash
  ./control_seoungjin/sample/run_sample_sim.sh
  ```
  MATLAB 실행에는 **Simscape Driveline**이 필요합니다 (`Aerodynamic Propeller` 블록이 `sdl_lib`를 참조). 없으면 Add-On 관리자에서 설치해야 합니다.
  MATLAB 실행파일은 자동으로 찾습니다 (PATH → `C:\Program Files\MATLAB\`의 최신 버전 순). 여러 버전이 깔려 있거나 다른 경로에 있으면 `MATLAB_EXE` 환경변수로 직접 지정하세요: `MATLAB_EXE="/c/Program Files/MATLAB/R2025b/bin/matlab.exe" ./run_sample_sim.sh`
- **`run_sample_sim.m`** (`controller/Quadcopter-Drone-Model-Simscape/` 안에 위치): `trajectory.mat` 로드 → 파라미터/라이브러리 경로 설정 → `sim('quadcopter_package_delivery')` 실행. 실행 도중 새로 생긴 워크스페이스 변수(`act_x1/y1/z1`, `des_x1/y1/z1` 등 로그 신호)를 `sim_result.mat`으로 저장합니다.
- **`run_and_log.py`**: 정해진 입력 형식(`config.yaml`/`config.json`, `waypoints` + `limits` + `dt`)을 읽어서 궤적 생성 → `trajectory.mat` 생성/복사 → MATLAB 배치 실행 → 결과를 CSV로 저장하는 전체 파이프라인을 한 번에 실행합니다.
  ```bash
  python control_seoungjin/sample/run_and_log.py --config control_seoungjin/sample/config.yaml
  ```
  출력은 `sample/output/`에 남습니다: `trajectory_feed.csv`(이 시간에 이 pos/yaw를 먹였다는 입력 스냅샷), `sim_result_*.csv`(로그된 각 신호별 결과, 예: `sim_result_act_x1.csv` vs `sim_result_des_x1.csv`로 실제/목표 위치 비교).

### "택배 배송 드론" 예제 관련 (Package / Disengage Logic)

`quadcopter_package_delivery`는 원래 MathWorks의 "짐을 배송하는 드론" 예제라서, 우리가 안 쓰는 짐(Package)/투하(Disengage) 로직이 딸려 있습니다. `run_sample_sim.m` 상단에 이걸 켜고 끄는 플래그 두 개가 있습니다 (배선은 그대로 두고 파라미터 값만 덮어써서 켜고 끕니다).

| 플래그 | 기본값 | 설명 |
|---|---|---|
| `use_default_package_branding` | `false` | `true`면 MathWorks 기본 로고(Logo 1/2, `Basket_Logo.STL`)가 짐 위에 표시됨. `false`면 로고 없이 빈 박스만 표시(짐 본체 자체는 CAD 파일이 아니라 `BrickSolid` 단순 박스라 항상 그대로 있음). |
| `enable_package_drop` | `true` | `false`면 `Distance to drop waypoint`의 거리 임계값(`dist_release`)을 `-1`로 덮어써서 투하 조건이 절대 만족되지 않게 함 (투하 로직 자체는 안 건드리고 조건만 비활성화). |

### 경로 관련 주의사항 (MATLAB)

- **CAD 파일 경로**: `File Solid` 블록(Arm/Plate/Propeller 등)들은 `ExtGeomFileName`에 `quadcopter_drone_arm.stp`처럼 **파일명만** 저장하고 있어서, MATLAB path에서 파일을 찾습니다. `addpath('Scripts_Data')`, `addpath('Models')`, `addpath('Libraries')`뿐 아니라 **`addpath(genpath('CAD'))`도 반드시 필요**합니다 (안 하면 CAD를 안 건드린 프로펠러/로고 파일까지 전부 "파일을 찾을 수 없음" 오류가 남).
- **`waypoints` 변수 방향**: `Ground/Trajectory/Waypoints` 블록은 `unique(waypoints','rows')`로 워크스페이스 변수를 사용하므로, `waypoints`는 **3×N** (x/y/z가 행, 각 열이 한 점) 이어야 합니다. `waypoints_to_maneuver_input.py`가 저장하는 `waypoints`는 N×3이라 MATLAB에서 꼭 전치(`S.waypoints'`)해서 써야 합니다. (반면 `spline_data`는 Lookup Table이 `spline_data(:,1)`처럼 열로 읽으므로 N×3 그대로 사용.)
- **MATLAB 실행파일**: 하드코딩하지 않고 `run_sample_sim.sh`가 자동 탐지합니다 (위 참고).

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
