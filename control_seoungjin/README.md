# control_seoungjin

## controller/
[Quadcopter-Drone-Model-Simscape](https://github.com/mathworks/Quadcopter-Drone-Model-Simscape.git) (MathWorks 공식)를 서브모듈로 가져와서 파라미터(질량, 관성, PID 게인 등) 튜닝을 진행하는 폴더입니다. 실제 컨트롤러에 적용하기 전에 여기서 시뮬레이션으로 값을 검증합니다.

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
