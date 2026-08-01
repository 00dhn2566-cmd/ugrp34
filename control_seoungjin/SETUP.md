# 제어 파트(성진) 받아서 돌리기 — 준비물 체크리스트

`control_seoungjin/`을 pull 받아 쓰려는 사람용. 뭘 하려는지에 따라 필요한 게 다르다.

## A. 궤적/미션 인터페이스만 쓸 사람 (RL·통합 — 대부분 여기)

준비물: **Python 3.10+ 와 pip 뿐.** MATLAB 불필요.

```bash
pip install numpy scipy matplotlib
cd control_seoungjin
python -m pytest tests/ -q                                  # 동작 확인 (169개)
python traj_pipeline.py plan --input input/example_mission.json   # 예시 미션 -> output/
```

- 통신 규약(미션 JSON 보내는 법, 회신 읽는 법, 비상 명령)은
  [EXTERNAL_INTERFACE.md](EXTERNAL_INTERFACE.md) 하나만 읽으면 된다.
- 내부 코드·문서(`INTERFACE_SPEC.md`, `*_STATUS.md`, `SESSIONS_BOARD.md`)는 몰라도 됨.

## B. MATLAB 시뮬(정답 플랜트)까지 돌릴 사람

위 A에 더해:

1. **드론 모델 폴더는 zip으로 따로 받는다** —
   `controller/Quadcopter-Drone-Model-Simscape/`는 서브모듈이지만
   **`git submodule update --init` 절대 금지** (우리 기체 FX450 CAD·튜닝이 원본 위에
   덮여 있어서 init 하면 MathWorks 원본으로 되돌아가 망가짐). 성진에게 zip 받아서
   그 경로에 풀 것.
2. **MATLAB R2025b 이상** (개발기는 R2026a) + 툴박스 5개:
   Simulink / Simscape / Simscape **Multibody** / Simscape **Electrical** /
   Simscape **Driveline** (Driveline 빠지면 프로펠러 블록 로드 에러).
3. MATLAB이 여러 버전이면 환경변수로 지정:
   `MATLAB_EXE="C:\Program Files\MATLAB\R2026a\bin\matlab.exe"`
4. 동작 확인: 모델 폴더에서 `matlab -batch "run_traj_baked"` (output/의 trajectory.mat
   을 비행) — 또는 `diagnose/verify_hover.m` (bare 호버 10s, 자세 RMS ~0.5도면 정상).

주의: 모델 파일(`Models/quadcopter_package_delivery.slx`)은 튜닝이 구워져 있으니
**절대 저장하지 말 것** (실험은 메모리 수정만). RAM 16GB 미만이면 시뮬 동시 2개 금지.

## C. C++ 제어기까지 빌드할 사람

위 A에 더해: cmake + g++ (Windows는 msys64, `mingw64\bin`을 PATH에).

```bash
cd control_seoungjin/controller_cpp
cmake -B build && cmake --build build
./build/qc_trace --smoke        # 산수 정상 확인
```

## 공통 참고

- 산출물(`output/`, `.mat`, CSV)은 git에 없음 — 없는 게 정상, 돌리면 생긴다.
- 실시간 파일(current_state.json 등)은 OneDrive 밖에 써야 함 — 기본값이 알아서
  `%LOCALAPPDATA%\ugrp_drone\`로 가므로 보통은 신경 쓸 것 없음.
