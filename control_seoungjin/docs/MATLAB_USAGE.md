# MATLAB 배치 시뮬 사용서 (전 세션 공용)

작성: 튜닝/C++ 세션 (배치 시뮬 40+회 실전), 2026-07-19. 대상: 비상 세션 포함
MATLAB으로 구운 모델을 돌려야 하는 모든 클로드 세션. 함정 상세는
[HANDOFF_EMERGENCY.md](HANDOFF_EMERGENCY.md) §8, 명령 모음은 [../COMMANDS.md](../COMMANDS.md).

## 0. 시작 전 3계명

1. **SESSIONS_BOARD.md의 "MATLAB 점유"를 확인·선언** — 1대 규칙 (RAM 16GB, 동시 시뮬 = 실제 다운 전력).
2. **구운 모델에 `save_system` 절대 금지** — 수정은 메모리에서만, 저장 없이 닫기.
3. **스크립트 1개 = MATLAB 프로세스 1개** — 한 세션에 체이닝하면 탭 포트 충돌.

## 1. 실행 방법

실행기는 **R2026a만** 진짜다 (`C:\Program Files\MATLAB\R2025b`는 빈 껍데기).

```bash
# Git Bash (권장 — 출력은 반드시 파일로: 콘솔 cp949라 한글 깨짐)
cd control_seoungjin/controller/Quadcopter-Drone-Model-Simscape
"/c/Program Files/MATLAB/R2026a/bin/matlab.exe" -batch \
  "cd(fullfile(pwd,'diagnose')); 스크립트명" > /tmp/out.txt 2>&1
```
```powershell
# PowerShell
& "C:\Program Files\MATLAB\R2026a\bin\matlab.exe" -batch "cd diagnose; 스크립트명" *> out.txt
```

- 소요시간 감: 콜드 스타트 ~40s + T=14s 비행 1건 ~90s. 격자 9점 ≈ 14분.
- 백그라운드로 돌리고 완료 후 파일을 읽어라. 폴링 금지.

## 2. 새 시뮬 스크립트 = 템플릿 복사

**절대 백지에서 쓰지 말 것.** `diagnose/refine_*.m`이 전부 검증된 같은 골격이다:

```
경로 추가(addpath Scripts_Data/Models/Libraries + genpath(CAD))   ← CAD 빠지면 File Solid 실패
→ load_system('quadcopter_library') → quadcopter_package_parameters
→ load_system(mdl)
→ 투하(dropBlocks) 비활성화                                        ← 안 끄면 공중 종점에서 가짜 불안정
→ 궤적 주입: 반드시 모델 워크스페이스(mws.assignin)                  ← base에 넣으면 Lookup이 못 봄
→ 신호 탭 (아래 §3)
→ sim(mdl) 루프 (스크립트 레벨 — 함수 안에서 sim() 금지)
→ 지표 계산 → CSV를 diagnose/results/에 저장
```

추천 출발점: **`diagnose/refine_pos_r1.m`** (격자 스윕의 정본). 격자·지표만 바꿔라.
궤적 변수 4종: `timespot_spl (N,1)` / `spline_data (N,3)` / `spline_yaw (N,1)` /
`waypoints (3×M — 전치 주의!)` + `wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints)`.
짧은 경로면 waypoints는 가짜 `[0 0 1; 1 0 1]'` 사용 (시각화 전용이라 무해).

## 3. 신호 태핑 (측정값 꺼내기)

Scope 버스에서 분기 — `refine_pos_r1.m`의 sigMap 블록을 그대로 복사:

| Scope 요소 | 신호 |
|---|---|
| `In Bus Element` | px (x 위치) |
| `In Bus Element1` | py |
| `In Bus Element2` | pz |
| `In Bus Element3` | real_pitch [rad] |
| `In Bus Element4` | real_roll [rad] |
| `In Bus Element5` | real_yaw [rad] |

제어기 내부 신호(cmd 등)는 `diagnose/diagnose_golden_trace.m`의 탭 패턴 참고
(Pitch Limit / Roll Limit 블록 출력 분기). 블록 찾기는 반드시:
```matlab
find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on')   % 옵션 없으면 조용히 빈 결과!
% 이름 매칭은 정규화로 (블록명에 개행 있음):
nm = regexprep(get_param(blk,'Name'), '\s+', ' ');
```
읽기는 To Workspace(StructureWithTime) → `interp1(s.time, s.signals.values, tu)`.

## 4. 비상 세션 특화 인프라 (검증 4편에 바로 쓸 것)

| 필요 | 있는 것 | 위치 |
|---|---|---|
| 외란 토크 주입 (B 검증 ③) | External Force and Torque 배선 + 펄스 패턴 | `diagnose/diagnose_robust_torque.m` (0.3N·m×0.3s 실전 검증본) |
| 초기 기울기 진입 (B 회생 트리거) | Spherical Joint `PositionTargetRotationSequenceAngles` IC 설정 | `diagnose/diagnose_pitch_ic_test.m` (IC 10° 실증) |
| 표준 펄스 교란 + 저크-가능 크기 산정 | 펄스 설계 패턴 | `diagnose/diagnose_swing_calib.m` |
| 호버 안정 관문 | 채택 관문 정본 | `diagnose/verify_hover.m` |
| trajectory.mat 실비행 + act/des 로그 | 접합 정본 | `run_traj_baked.m` (투하 off 기본 내장) |
| 궤적 게이트/정지거리 | stop_dist 정확식 | `Scripts_Data/traj_smoother.m` (§8 계보) |

## 5. 게인/파라미터 만지기

- 원본은 `Scripts_Data/quadcopter_package_parameters.m` — 스크립트에서 임시로 바꿀 땐
  parameters 호출 **후** base 워크스페이스 변수로 덮어쓰기 (refine_* 패턴).
  parameters.m 파일 수정은 튜닝 세션과 협의 (보드 ★).
- `ctrl_profile` 변수: `'precision'`(기본)/`'balanced'`/`'agile'` — sim 전에 base에 지정.
- 비상 실험에서 발산이 나면: 그 로그의 tail은 학습/판정에 쓰지 말 것 (발산 과도 오염 —
  analyze_flight_log의 30cm 게이트와 같은 원칙).

## 6. 결과 판독 습관

- 지표는 CSV로 `diagnose/results/`에 저장 (원자료 보존 — 재판정 대비).
- "합격" 출력을 믿지 말고 수치로 판정 — 스크립트 문구는 기준이 느슨할 수 있다
  (실사례: 호버 관문 "합격" 출력이 있었지만 지터 130배 퇴행으로 반려).
- 발산 시그니처 빠른 식별: 오버슈트 수 m = 위치/자세 발산, z피크 수십 cm = 고도
  과출력, 왕복 지속 = 릴레이 한계사이클 (게인 문제 아님 — 입력 계약 위반).
