# 인수인계: JSON path → 궤적 → 컨트롤러 명령 파이프라인 (다음 Claude 세션용)

작성: 2026-07-13, 이전 세션(CoM 수정 + PID 재튜닝 + 모델 굽기 완료) 직후.

## 목표 (사용자 지시)

path가 **JSON 형태로 주어지면** → `path_time.py`로 시간 파라미터화 → 구운 Simscape 모델로 시뮬 → **컨트롤러 명령 + 추종 결과**까지 나오는 파이프라인. **controller 부분까지만** (비전/VIO/Isaac Sim 연동은 범위 밖). 기존 `sample/run_and_log.py` 흐름과 별개로 **새로 하나 더** 만드는 것.

## 전제: 현재 검증된 상태 — 이걸 깨면 안 됨

1. **`controller/Quadcopter-Drone-Model-Simscape/Models/quadcopter_package_delivery.slx`는 "구운" 자립 모델이다.**
   앵커 보정(Plate Anchor Comp), 추력 Bias 재스케일, 고도 클램프, x/y 위치오차 saturation이 전부 파일 안에 저장돼 있다. 아무 스크립트 수정 없이 로드→호버만 시켜도 안정 비행한다 (10초, 자세 RMS 0.56°). 검증 스크립트: `diagnose/verify_hover.m`.
   → **절대 `save_system` 하지 말 것.** 실험용 수정은 메모리에서만 하고 저장 없이 닫는다.
2. **자세 PID 게인이 음수인 것은 버그가 아니라 정답이다.**
   실측 플랜트 이득이 음수(u→pitch 가속 b=−0.0296, 스텝 응답 회귀로 측정)라서 `kp_attitude=-100, kd_attitude=-150`이 맞다. "부호가 이상하다"고 고치면 즉시 발산한다. 게인 위치: `Scripts_Data/quadcopter_package_parameters.m`.
3. **CAD 파일 3개 중 arm은 절대 회전 금지.**
   플레이트 2개(`plate_top`/`plate_bottom` STEP)는 눕혀서 재저장된 상태(원래 세워져 있던 게 roll 플립의 근본 원인이었음). arm은 Transform의 ZXZ 회전 + Custom 관성이 현재 방향 기준으로 정합돼 있어 건드리면 깨진다. 전체 사건 기록: `controller/Quadcopter-Drone-Model-Simscape/TUNING_STATUS.md`.
4. **git**: 서브모듈 브랜치 `fix/plate-orientation-cg`, 부모 브랜치 `fix/plate-orientation-cg` (원격에는 `fix/plate-orientation-cg-workload`로 푸시됨, 같은 공개 repo를 공유해서 이름 충돌 회피). **`git submodule update --init` 금지** (FX450 CAD가 MathWorks 원본으로 덮임). Claude의 push는 권한상 차단되므로 푸시는 사용자에게 명령어를 준다.

## 새로 만들어 둔 글루 코드 (미실행 — 첫 작업으로 검증할 것)

**`controller/Quadcopter-Drone-Model-Simscape/run_traj_baked.m`** — `trajectory.mat`을 구운 모델에 주입해 실행하고 위치/자세 추종 성능을 요약·저장한다. 헤더에 사용법 있음. **아직 한 번도 실행 안 됨** — 특히 `act_x1`/`des_x1` 내장 로그 변수의 존재와 형식(timeseries vs struct)은 추정이므로 첫 실행 로그를 보고 맞출 것 (스크립트가 Scope 버스 신호 매핑을 출력해주니 그걸 근거로).

남는 작업은 앞단: **JSON path → `trajectory.mat`**. 재료가 다 있다:
- `path_time.py`의 `plan_waypoints`(waypoint 리스트 → 7차 다항식 최소시간 궤적)
- `sample/waypoints_to_maneuver_input.py`에 waypoint → `trajectory.mat` 변환 로직이 있음 (참고용)
- `sample/INPUT_FORMAT.md`에 기존 config.json 스펙 있음 (waypoints + limits + dt)

**⚠ 파일 위치 규칙 (사용자 지정, 2026-07-14)**: `sample/`은 사용자가 직접 작성한 개인 테스트 케이스 폴더다 — **새 파이프라인 코드를 sample/ 안에 만들지 말 것.** 신규 산출물은 `control_seoungjin/` 바로 밑에 둔다 (path_time.py, yaw_spin.py와 같은 층위). sample/ 파일은 읽고 참고만 하고, 수정·확장은 사용자 확인 후에.

**폴더 인터페이스 규약 (사용자 지정, 2026-07-14)**:
- `control_seoungjin/input/` — 상위 단계(경로계획 등)에서 오는 입력 수신 (JSON path는 여기서 읽는다)
- `control_seoungjin/output/` — 다음 단계(Isaac Sim 등)로 보낼 결과 저장 (모터 명령, 추종 로그 등은 여기에 쓴다)
- output/은 생성물이므로 .gitignore 처리 검토 후 커밋에서 제외

## 모델 구동 레시피 (검증된 패턴 — verify_hover.m 그대로 따라할 것)

```matlab
% 1) 경로 (4개 전부 필요. CAD는 genpath 필수 - File Solid가 파일명만 저장함)
addpath('Scripts_Data'); addpath('Models'); addpath('Libraries'); addpath(genpath('CAD'));
load_system('quadcopter_library');
quadcopter_package_parameters;          % 게인/프로펠러 계수
mdl = 'quadcopter_package_delivery'; load_system(mdl);

% 2) 궤적 5변수는 "모델 워크스페이스"에 assignin (base 아님!)
mws = get_param(mdl, 'ModelWorkspace');
mws.assignin('timespot_spl', t_col);        % (N,1) [s]
mws.assignin('spline_data', xyz);           % (N,3) [m]  <- N×3 그대로
mws.assignin('spline_yaw', yaw_col);        % (N,1) [rad]
mws.assignin('waypoints', wp3xM);           % 3×M    <- 파이썬 N×3에서 전치 필요!
mws.assignin('wayp_path_vis', quadcopter_waypoints_to_path_vis(wp3xM));
set_param(mdl, 'StopTime', num2str(t_col(end)));
sim(mdl);
```

- 실행: `"/c/Program Files/MATLAB/R2026a/bin/matlab.exe" -batch "스크립트명"` — **R2026a가 실사용 버전** (R2025b 폴더는 있지만 실행파일 없음).
- 상태 신호 읽기: 최상위 `Scope` 안 `In Bus Element` 블록에 To Workspace를 붙인다. 검증된 매핑: **Element2=z, Element3=pitch, Element4=roll**. Element21/22는 Z-mix 이후 모터측 cmd라 자세 명령으로 오독하지 말 것.

## 함정 목록 (전부 실제로 당한 것)

| 함정 | 대책 |
|---|---|
| 블록 이름에 개행 (`"Altitude and⏎YPR Control"`, `"Transform⏎Arm1"`) | 이름 비교 전 `regexprep(name,'\s+',' ')` + `strtrim` |
| `waypoints`는 3×M, `spline_data`는 N×3 (반대 방향) | 위 레시피 참고. 틀리면 조용히 이상 비행함 |
| find_system이 자기 자신도 반환 (같은 이름 서브시스템) | 결과에서 `~strcmp(결과, 검색루트)` 필터 |
| 라인 핸들 −1, 분기 라인의 LineChildren/LineParent 순환 | 방문 목록 기반 반복 탐색 (diagnose_robust_torque.m의 `collect_line_ends` 참고) |
| External Force and Torque 포트: **RConn1=프레임, LConn=PS입력** | 거꾸로 연결하면 컴파일에서 "연결 규칙 위반" |
| 모델 수정 스크립트가 대상 못 찾고 조용히 통과 | **대상 개수 출력 + 미발견 시 error() 즉사**가 이 저장소의 규칙 |
| 컴파일/사전검사 전에 궤적 변수 미주입 | waypoints 없다는 오류 폭탄 — 워크스페이스 주입을 먼저 |
| -batch 로그의 한글 깨짐(mojibake) | 숫자/구조는 읽을 수 있음. 신경 쓰지 말 것 |
| Git Bash 경로 → MATLAB | 네이티브 Windows MATLAB에 POSIX 경로 주지 말 것 (`cygpath -w`) |

## ⚠ RAM (이 컴퓨터 16GB — 실제로 시스템 다운 경험 있음)

- 배치 시뮬 1회 = 2~4GB. **동시에 2개 이상 돌리지 말 것.**
- 모델 굽기(bake, 컴파일 2회)는 6~8GB+ — 이 머신에서 실제로 죽었음. 다시 구울 일이 생기면 다른 앱 다 닫고 하거나 사용자와 상의.

## 명령 스무더 설계 스펙 (사용자 확정, 2026-07-14 — **같은 날 구현·검증 완료**)

> **구현됨**: `controller/.../Scripts_Data/traj_gate.m` + `traj_smoother.m` + `traj_zv.m`(잔류진동 소거 셰이퍼, ZV/ZVD), 검증 `diagnose/diagnose_smoother.m`·`diagnose_zv_shaper.m`.
> **파이프라인 순서(고정)**: path_time → traj_smoother(물리 한계) → **traj_zv(1.8Hz 모드 상쇄)** → 컨트롤러. ZV는 볼록 결합이라 스무더의 v/a/j 한계를 보존 — 순서 역전 금지 (스무더가 임펄스 간격을 뭉개면 상쇄 조건 깨짐). 실증: 도착 후 pitch RMS 4.26→1.51°.
> 발산 확정 궤적(1m/0.67s)이 성형 후 안정 비행 (모터 81%, 자세 13.2°, RMS 2.8cm — 원본은 포화·발산).
> 구현 교훈 2개(후방차분 상태 원칙, 정확 정지거리 트리거)는 TUNING_STATUS §V 참고.
> 남은 일: `run_traj_baked.m` 입구에 traj_gate 호출 연결 (run_traj_baked 자체가 미실행이라 그 검증과 함께).

배경: 발산 배터리(TUNING_STATUS §T~U)의 결론 — 모터 포화의 원인은 게인이 아니라 **기준 궤적의 요구량이 물리 한계(실측 envelope: v≈2.5 m/s, a≈2.5 m/s²)를 초과**하는 것. 오차 성형(클램프/램프)은 피드백 경로라 부작용(무감쇠 활공)이 있고 옆문(§U 둘째 경로)을 못 막는다. **기준 궤적 자체를 성형하는 open-loop 필터**가 정답.

**2층 방어:**
1. **검증 게이트 (오프라인, 즉시 구현)** — `run_traj_baked.m` 입구에서 `trajectory.mat` 전체를 수치미분 → v/a 피크가 envelope 초과 시 시끄럽게 `error()`. 현재 파이프라인(사전계산 룩업테이블)은 이것만으로 발산 시나리오 전부 차단.
2. **스트리밍 성형기 (C코드 단계 부품)** — 상위층이 실시간 명령을 던질 때(quick모드, RL 직결) 대비. 아래 식.

**성형기 식 (사용자 설계)**: 시점 t 앞으로의 성형된 기준 변위 Δr(t)를 다음 구간에 클램프한다.

```
상한: min( +j_max/6·t³ + a₀/2·t² + v₀·t,   +a_max/2·t² + v₀·t,   +v_max·t )
하한: max( −j_max/6·t³ + a₀/2·t² + v₀·t,   −a_max/2·t² + v₀·t,   −v_max·t )
```

세 항 = 저크 지배(짧은 t) / 가속 지배(중간 t) / 속도 지배(긴 t) 구간의 도달가능성 상한이며, min/max가 구간별 구속 제약을 자동 선택한다. **하한이 핵심 반쪽**: 이동 중 기준이 순간정지를 요구해도 하한이 v₀t − a_max/2t² > 0이라 성형 기준은 a_max 물리 제동으로만 감속 — 오차 클램프의 무감쇠 활공 문제를 원천 차단.

**구현 원칙 4개 (위반 시 오늘 실험의 함정 재현):**
1. **v₀, a₀는 성형기 자신의 내부 상태** (지금까지 통과시킨 성형 기준의 미분). 드론 측정값을 넣으면 피드백 성형으로 변질 → 활공 함정 부활.
2. min/max는 약간 낙관적 상한(구간 전환부) — envelope 실측치보다 깎은 값 사용 (예: a_max=2.0).
3. 구현 분담(사용자 확정): **게이트는 xy 노름 검사**(엄격, `traj_gate.m`) / **성형기는 각 축 독립 적용**(`traj_smoother.m`). **⚠ xy 동시 기동 경로에서는 성형기 호출 시 xy 한계에 ×0.7 축배분 필수** — 박스 투어 실증(§W): 축별 2.0씩 동시 감가속 → 노름 2.83으로 게이트 재차단됨. 사용 예: `traj_smoother(t, pos(:,1:2), VMAX*0.7, AMAX*0.7, JMAX)` + z는 전한계.
4. 전방 전용 필터라 목표 앞 정지 예측은 없음 — 비정상 입력 시 발산 대신 오버슈트로 열화(수용 가능). C코드 완성판에서는 정지거리 v²/2a 항을 넷째 인자로 추가.

path_time.py는 snap까지 고려해 시간을 늘이는 최정밀 층이므로 그대로 상류에 유지 — 이 스펙은 path_time을 거치지 않는 입구(직접 작성 궤적, yaw_spin 병합, RL 직결, config 실수)에 대한 백스톱이다.

## 스윙(백래시) 대응 방향 (사용자 확정, 2026-07-15 15차 마감)

잔류 스윙(1.8Hz 저중심 모드) 대응은 온라인 능동 소거가 아니라 **경로 시간 분배기(path_time) 레벨의 피드포워드 성형**으로 간다:
1. **백래시(스윙 응답) 값을 측정** — 이동 프로파일별 유발 스윙 진폭/위상 (교정 인프라: `diagnose_swing_calib.m`, 단 펄스는 저크-가능 설계 필수)
2. **path_time에서 경로를 살짝 꼬아 반동 상쇄** — 측정된 응답을 근거로 궤적 워핑. `traj_zv.m`(ZV/ZVD)가 이 계열의 1호 구현(−65% 실증)이며, 이를 path_time 레벨로 승격·정밀화하는 형태.
3. 잔여분은 나중에 **별도 PID 제어**로 (사용자 결정 — 자세채널 직접 주입 아이디어는 그때 후보로 보관, §W 참고).

근거 데이터(§W): 기준 채널은 예방(온건 이동 20배 저감, ZV −65%)에 강하고 온라인 소거(7mm 펌핑 널)에 약함 — 사전 성형 쪽이 물리적으로 유리한 채널.

## 참고 문서

- `controller/Quadcopter-Drone-Model-Simscape/TUNING_STATUS.md` — CoM 사고 원인 추적 + 튜닝 전 과정 (가장 상세)
- `control_seoungjin/README.md` — controller 섹션에 요약본
- `sample/INPUT_FORMAT.md` — 기존 config.yaml/json 입력 스펙
- `COMMANDS.md` — MATLAB 배치 실행 커맨드 모음

## 강건성/튜닝 현황 (2026-07-14 종료 시점 — 상세는 TUNING_STATUS.md §Q~U)

- **① 토크 펄스 — 합격** (최대 3.52°, 0.7s 복귀) / **② CG ±10.5mm 4방향 — 합격** / **병진 외란 6종 — 합격**
- **③ 패키지 드롭, ④ 바람 — 미실시.**
- **yaw — 해결됨(§R~S)**: `In Bus Element5`=Chassis.yaw 확정, kp/ki/kd_yaw=15/1.5/4 (지속 외란 오프셋 I로 소거). 로깅 미확정이라던 이전 기술은 폐기.
- **roll/pitch 지터 — 해결됨(§T)**: 진범은 고도 D경로(filtD=10000 링잉) → kd_altitude=0.15/filtD=1000, filtD_attitude=2000. 자세 RMS 0.534→0.098°.
- **이동비행 6종 중 5합격(§T)**, envelope 실측 v≈2.5/a≈2.5. 케이스4(z 하강) 모델 에러 미해결 — getReport 전문 캡처부터.
- **둘째 경로(옆문) — 해결됨(§W, 14차)**: 외부 피드포워드 배선이 아니라 PC 내부 구조 2개였음 — ① z 오차 무클램프 + RBI 회전 혼입(PIDz가 클램프 x의 30배로 body-x 누설, Sat Z 삽입으로 봉인 실증) ② 클램프 진입 미분 킥(50° 순간 명령). 추가 발견: 자세 측정 필터 지연 7°, 활공 정상오차 5~7°(자세 I 부재 — ki_att −30에서 활공거리 −29% 실증). **발산 5대 메커니즘 전부 실측·처방 매핑 완료 — §W 표 참고.** 남은 결정: Sat Z·ki_attitude 정식 채택 여부(회귀 배터리 통과 조건).
- **신규 발견(§V, 13차)**: 이동 후 자세 왕복(pitch RMS ~4°, 1.75Hz)은 게인 문제가 아니라 **매달린 짐의 무감쇠 진자(L≈8cm)** — 질량 2배에도 주파수 불변으로 확정. 게인/자세I/위치D 전부 무효(스윕 완료, §V). 처방은 스윙 억제(input shaping/스윙 댐핑) = 과제 본론. 짐 결합 구조 조사 미완(`diagnose_all_joints.m` 작성만 됨).
- **CAD 정합 확인 (사용자 지시, 13차 말)**: 짐 결합 구조 조사와 함께 CAD 파일이 제대로 들어가 있는지 검증할 것 — ① File Solid 블록들의 참조 파일명이 실제 FX450 CAD를 무는지 (`addpath(genpath('CAD'))` 후 미해결 참조 없는지) ② 플레이트 2개는 눕혀 재저장된 버전인지 ③ `quadcopter_drone_arm.stp`는 회전 안 된 원본인지 (arm 회전 금지 — Transform ZXZ + Custom 관성이 현재 방향 기준). 짐 진자 길이(8.1cm 예측)와 앵커/플레이트 지오메트리 대조도 이때 함께.
