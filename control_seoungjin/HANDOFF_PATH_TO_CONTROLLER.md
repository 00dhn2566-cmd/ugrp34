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

## 참고 문서

- `controller/Quadcopter-Drone-Model-Simscape/TUNING_STATUS.md` — CoM 사고 원인 추적 + 튜닝 전 과정 (가장 상세)
- `control_seoungjin/README.md` — controller 섹션에 요약본
- `sample/INPUT_FORMAT.md` — 기존 config.yaml/json 입력 스펙
- `COMMANDS.md` — MATLAB 배치 실행 커맨드 모음

## 강건성 테스트 현황 (2026-07-13 종료 시점)

- **① 토크 펄스 0.3 N·m×0.3s — 합격** (최대 3.52°, 0.7s 복귀, z 유지)
- **② CG 오프셋 ±5mm — 4방향 전부 합격** (RMS ≤1.01°, 정상상태 잔류 ~0°)
- **③ 패키지 드롭, ④ 바람(z>2m) — 미실시.** 상세와 배선 교훈은 TUNING_STATUS.md 섹션 (Q) 참고.
- **yaw 미로깅**: Scope의 yaw Element 번호가 미확정이라 강건성 스크립트가 roll/pitch/z만 로깅함. `run_traj_baked.m`이 출력하는 버스 매핑으로 번호 확정 후 yaw 로깅을 추가하는 것이 다음 세션 첫 작업.
- **yaw 제어 이슈(미확인)**: 사용자가 "yaw 안 잡는 것 같다"고 관찰. 현재 yaw는 각도 PD(ki=0) — 구조 자체는 정상이나 상수 반토크 외란이 정상상태 오차로 남을 수 있음. yaw 로깅 → 드리프트 실측 → 실측되면 ki_yaw 0.1~0.3 추가 검토 순서로. 상세는 TUNING_STATUS.md (Q) 참고.
