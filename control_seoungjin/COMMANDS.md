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

## 알려진 이슈

- **Simscape Driveline 필수**: `Aerodynamic Propeller` 블록이 `sdl_lib`(Simscape Driveline)를 참조. 없으면 `Propeller 1~4` 서브시스템에서 라이브러리 로드 오류.
- **CAD 경로**: `File Solid` 블록은 파일명만 저장(`quadcopter_drone_arm.stp`)하므로 `addpath(genpath('CAD'))` 필수.
- **`waypoints` 방향**: `Ground/Trajectory/Waypoints` 블록은 3×N을 기대 (`unique(waypoints','rows')`). `spline_data`는 N×3 그대로 사용.
- 한글 `print`문은 Git Bash 콘솔에서 인코딩이 깨져 보일 수 있음(cp949) — 실제 동작엔 문제없음.
