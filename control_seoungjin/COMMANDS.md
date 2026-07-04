# control_seoungjin 작업용 명령어 모음

이 세션에서 실제로 쓴, 재현 가능성이 높은 명령어들을 정리한다. (Git Bash 기준)

## 전체 파이프라인 한 번에 (추천)

```bash
python control_seoungjin/sample/run_and_log.py --config control_seoungjin/sample/config.yaml
```
`config.yaml`의 `waypoints`/`limits`/`dt`를 읽어서: 궤적 생성 → `trajectory.mat` 생성/복사 → MATLAB 배치로 `run_sample_sim.m` 실행 → 입력/출력 CSV(`sample/output/`) 저장까지 전부 처리한다.

MATLAB 실행파일은 `MATLAB_EXE` 환경변수 → PATH → `C:\Program Files\MATLAB\`의 최신 버전 순으로 자동 탐지한다 (하드코딩 안 함). 여러 버전이 깔려 있으면:
```bash
MATLAB_EXE="/c/Program Files/MATLAB/R2025b/bin/matlab.exe" python control_seoungjin/sample/run_and_log.py
```

## 개별 단계 (디버깅용)

```bash
# 1) 샘플 궤적만 생성
cd control_seoungjin/sample
python waypoints_to_maneuver_input.py        # trajectory.mat 생성

# 2) 궤적이 v_max/a_max/j_max 지키는지 검증 + 플롯
python verify_sample_trajectory.py           # sample_trajectory.png 생성

# 3) 전체 셸 스크립트 (궤적 생성 + MATLAB 실행)
./run_sample_sim.sh
```

## MATLAB 배치 실행 (Git Bash에서 Windows MATLAB 호출)

Git Bash의 POSIX 경로(`/c/Users/...`)는 네이티브 Windows 실행파일이 못 읽는 경우가 있어 `cygpath -w`로 변환해서 넘긴다.

```bash
# 버전/라이선스된 툴박스 확인
"/c/Program Files/MATLAB/R2026a/bin/matlab.exe" -batch "ver"

# 특정 폴더를 시작 폴더로 잡고 스크립트 실행 (-sd)
MODEL_DIR=$(cygpath -w "$(pwd)/control_seoungjin/controller/Quadcopter-Drone-Model-Simscape")
"/c/Program Files/MATLAB/R2026a/bin/matlab.exe" -batch "run_sample_sim" -sd "$MODEL_DIR"

# 오래 걸리는 작업은 백그라운드 + 로그 파일로
"/c/Program Files/MATLAB/R2026a/bin/matlab.exe" -batch "tune_pid" -sd "$MODEL_DIR" > /tmp/tune_pid.log 2>&1 &
```

`control_seoungjin/controller/Quadcopter-Drone-Model-Simscape/` 안의 MATLAB 스크립트:
- **`run_sample_sim.m`**: `trajectory.mat` 로드 → 경로/라이브러리 설정 → `sim('quadcopter_package_delivery')` → 로그 변수(`act_x1` 등)를 `sim_result.mat`으로 저장. 상단 `use_default_package_branding`/`enable_package_drop` 플래그로 짐 로고/투하 로직 on-off 가능.
- **`tune_pid.m`**: Position/Pitch/Roll/Thrust/Yaw PID 5개 블록을 `systune`으로 재튜닝.

### 진단용 스크립트 (`diagnose/`)

모델 내부 블록/포트/신호 연결 상태를 조사할 때 쓰는 1회성 디버깅 스크립트 모음. 파이프라인 실행에는 필요 없음. `Scripts_Data`/`Models`/`Libraries`/`CAD` 경로를 `mfilename`으로 자동 계산하므로 실행 폴더(`-sd`)는 어디든 상관없이, 스크립트 폴더만 path에 추가하면 된다:

```bash
DIAG_DIR=$(cygpath -w "$(pwd)/control_seoungjin/controller/Quadcopter-Drone-Model-Simscape/diagnose")
"/c/Program Files/MATLAB/R2026a/bin/matlab.exe" -batch "addpath('$DIAG_DIR'); diagnose_goto" -sd "$DIAG_DIR"
```

- **`diagnose_pid_ports.m`**: `Control Pitch` 내부 `PID Compensator Formula`가 실제로는 비활성 variant(포트 0개)임을 확인.
- **`diagnose_sltuner_props.m`**: `slTuner` 객체의 `Options`/`RateConversionOptions` 등 속성 목록 조회.
- **`diagnose_motor_signals.m`**: 모델에서 Motor/Propeller/Mixer 관련 블록 이름 검색.
- **`diagnose_motor_mixer.m`**: `Motor Mixer` 서브시스템의 입출력 포트와 `Maneuver Controller` ↔ `Quadcopter` 간 연결 확인.
- **`diagnose_goto.m`**: Goto/From 태그로 연결된 신호(`ref`, `Roll`/`Pitch`/`Yaw`/`Thrust` 등) 추적, `Motor Mixer` 내부 블록 목록.

이후 PID 재튜닝 디버깅 과정에서 추가된 스크립트(포트 번호/Element, 마스크 tunable 플래그, CAD 질량/관성, `linearize()`+`pidtune()` 시도 등)는 개수가 많아 여기 일일이 나열하지 않고 [`TUNING_STATUS.md`](controller/Quadcopter-Drone-Model-Simscape/TUNING_STATUS.md)에 각각 무엇을 확인했는지 정리해뒀다.

## 알려진 이슈

- **Simscape Driveline 필수**: `Aerodynamic Propeller` 블록이 `sdl_lib`(Simscape Driveline)를 참조. 없으면 `Propeller 1~4` 서브시스템에서 라이브러리 로드 오류.
- **CAD 경로**: `File Solid` 블록은 파일명만 저장(`quadcopter_drone_arm.stp`)하므로 `addpath(genpath('CAD'))` 필수.
- **`waypoints` 방향**: `Ground/Trajectory/Waypoints` 블록은 3×N을 기대 (`unique(waypoints','rows')`). `spline_data`는 N×3 그대로 사용.
- 한글 `print`문은 Git Bash 콘솔에서 인코딩이 깨져 보일 수 있음(cp949) — 실제 동작엔 문제없음.
