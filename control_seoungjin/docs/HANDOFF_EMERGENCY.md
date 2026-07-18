# HANDOFF: 비상(emergency) 전담 세션 착수 문서

작성: path_time 세션, 2026-07-18. **규약 원본은 [../INTERFACE_SPEC.md](../INTERFACE_SPEC.md) §9** —
이 문서는 요약 + 맥락이며, 충돌 시 §9가 이긴다.

## 0. 임무 한 줄

두 종류의 비상을 구현하고 MATLAB 정답 플랜트에서 검증한다:
- **A. 상위 선언형** — A-1 비상 정지("지금 멈춰"), A-2 금지 구역("거기 가면 큰일")
- **B. 자체 회생형** — 자세제어 완전 상실 시 회생 비행 (RECOVER 상태기계)

우선순위 B > A-1 > A-2 > 일반 명령. 상세·메시지 스키마·검증 의무 3편은 §9.

## 1. 시작 전 필독 (순서대로)

1. `control_seoungjin/SESSIONS_BOARD.md` — 세션 프로토콜. **자기 섹션만 갱신**,
   MATLAB 사용 전 점유 선언 (1대 규칙, RAM 16GB — 동시 시뮬 절대 금지, 실제 다운 전력).
2. `control_seoungjin/INTERFACE_SPEC.md` — 전체 계약. 특히:
   - §0 저장 경로 (`UGRP_IO_ROOT`, 30Hz 파일은 `UGRP_RT_DIR` → OneDrive 밖)
   - §1 **무명령 기본 정책** — "현재 자리 1회 래치 호버". 비상의 종착 상태가 이것.
     ④ "대이탈 시 setpoint 스냅백 금지"는 사용자가 직접 교정한 안전 규칙.
   - §5 current_state v0.2 (비상은 v0.3으로 `mode` 필드 확장 — §9)
   - §8 작업 API 동사 카탈로그 (`emergency` 동사가 여기 편입됨. §8 자체도 미구현 —
     splice CLI부터. 비상 세션이 emergency 동사만 먼저 구현해도 무방하나 종료 코드
     0/1/2 + stdout 마지막 줄 JSON 규약은 지킬 것)
   - §9 **비상 규약 (너의 헌법)**
3. `control_seoungjin/PIPELINE_STATUS.md` — 궤적 층 현황 + 남은 일 (emergency 예약 항목 있음)
4. `controller/Quadcopter-Drone-Model-Simscape/TUNING_STATUS.md` — 플랜트/게인 전 역사
5. `docs/HANDOFF_CPP_GAZEBO.md` — C++ 쪽 구조 (B 상태기계가 C++에도 실려야 함)

## 2. 이미 만들어져 있는 재료 (재사용하라, 재발명 금지)

| 재료 | 위치 | 비상에서의 용도 |
|---|---|---|
| `stop_dist()` 2단 정지거리 정확식 | `Scripts_Data/traj_smoother.m` (Python판 `traj_shaping.py`) | A-1 정지 궤적의 수학. sqrt 근사 쓰지 말 것 — 45cm 오버슈트 실측 사건이 정확식의 존재 이유 |
| `replan_splice()` + 비상 분기 | `traj_pipeline.py` | 상태 승계 재계획. **비상만 실측 상태 사용** (평시 스플라이스는 기준 상태 — 테스트 `test_emergency_splice_uses_measured`가 이 구분을 지킴) |
| current_state v0.2 (30Hz) | C++ qc_io 생산자 구현 완료, 파이썬 소비자 `load_current_state()` | 트리거 감지 입력 + 정지/회생 초기 상태. 신선도 0.5s 검사 내장 |
| 무명령 래치 호버 | Simulink Lookup 클램프 (실증: tail 8s 잔류 0.000°) | 모든 비상의 종착 상태. C++ 이식 시 클램프 외삽 재현 필수 (§1 경고) |
| 게이트 4종 | `traj_shaping.py::traj_gate` | A-2 keep_out 검사를 여기에 추가 (전 샘플 교차) |
| 원장/보고 체계 | `feedback_ledger.jsonl`, `traj_report.py` | hash 무효 선언·비상 이벤트 기록. reject_code 추가 규칙: 의미 변경/삭제 금지, 추가만 |
| 표준 펄스 교란 인프라 | `diagnose/diagnose_swing_calib.m` | B 검증 ③(인위 교란 주입)의 출발점 — 펄스 설계(저크-가능 Tm 산정) 패턴 복사 |

## 3. 알아야 할 물리/튜닝 사실 (전부 실측)

- **자세 게인은 음수가 정상** (kp=-85, kd=-127.5): 플랜트 이득이 음수. "고치면" 즉시 발산.
- **0kg 레짐 붕괴** (튜닝 세션 07-17): 1kg 튜닝 시스템이 0.5~0kg 사이에서 준발산.
  회생 게인 설계 시 질량 유효값 요동(짐 크게 흔들릴 때)이 최악 케이스라는 근거.
  임무에 투하는 없음(사용자 확정) — 0kg은 운영 구간이 아니라 과적합 경계.
- **짐 모드 1.8Hz** (현수 1kg): 비상 정지 시 ZVD 생략이므로 짐이 흔들리며 도착함을
  전제하라. 래치 호버 후 잔류 스윙은 2호기(counter_swing) 소관 — 네 소관 아님.
- **위치 게인 프로파일 3종** precision/balanced/agile (§1 controller_profile) — 회생
  모드는 프로파일과 무관하게 자세 우선 (위치 루프 차단이 회생의 핵심).
- 물리 한계 실측 근거: v/a ≈ 2.5 envelope에서 깎은 확정 상수 2.0/2.0/j10. 비상은
  이 풀 한계 사용 (지터 마진 20% 반납 — §9 비상 레짐).

## 4. MATLAB 함정 목록 (하나라도 어기면 몇 시간 날림)

- **구운 모델 `Models/quadcopter_package_delivery.slx`에 절대 `save_system` 금지** —
  메모리 수정만, 닫을 때 저장 안 함. 앵커 보정·클램프가 파일에 구워져 있음.
- 궤적 변수는 **모델 워크스페이스**에 assignin (`get_param(mdl,'ModelWorkspace')`) —
  base workspace에 넣으면 Lookup이 못 봄.
- `waypoints`는 3×M 전치 필요 (Waypoints 블록), `spline_data`는 N×3 그대로.
- `addpath(genpath('CAD'))` 필수 — File Solid가 파일명만 저장.
- 시각화 Spline: 1m 미만 세그먼트 버그는 패치됨(`quadcopter_waypoints_to_path_vis.m`
  최소 2점 가드). 그래도 궤적이 아주 짧으면 가짜 1m waypoints를 시각화용으로 넣는
  패턴 사용 (diagnose_swing_calib.m 참고).
- 투하(Disengage) 로직은 **끄고 시작** — `enable_package_drop=false` 패턴
  (run_traj_baked.m / diagnose_swing_calib.m의 dropBlocks 참고). 안 끄면 공중 종점
  미션에서 1kg 분리 → 가짜 불안정 (실제로 하루 재판정 사건 있었음).
- 콘솔 인코딩 cp949 — print/오류 문자열에 em-dash(—)·화살표(→) 금지 (Python도 동일).
- 로그는 SaveFormat Array + Clock→sim_time 동승 또는 StructureWithTime
  (diagnose_swing_calib.m 패턴).

## 5. 다른 세션과의 경계 (침범 금지선)

| 영역 | 주인 | 너와의 접점 |
|---|---|---|
| 궤적 파이프라인 본체 (plan/splice/shaper) | path_time 세션 | emergency 동사·keep_out 게이트는 네가 추가하되 기존 74개 테스트 깨지 말 것 (`python -m pytest tests/ -q`, control_seoungjin/에서) |
| 게인/플랜트/C++ 제어 체인 | 튜닝·C++ 세션 | B 상태기계의 C++ 측 구현은 **협의 후** (보드 ★로) — 골든 트레이스 체계 훼손 금지 |
| 2호기 counter_swing | path_time 세션 | 비상 후 잔류 스윙 처리는 넘겨라 (swing_calib.json 소비 쪽) |
| 파라미터 가드레일 3종 | 전 세션 공통 | 앵커 불변 / 시뮬 질량 진실은 CAD / 비율만 |

## 6. 착수 순서 제안

1. §9 정독 → 트리거 임계값 후보 정리 (45°/0.3s는 잠정 — 실측으로 확정할 것)
2. A-1부터 (제일 쉽고 재료 완비): emergency 동사 → 정지 궤적 → MATLAB 검증 ①
3. A-2 keep_out: 스키마 → 게이트 검사 → 회피 재계획 → 검증 ②
4. B 회생: Simulink에서 상태기계 프로토타입 → 트리거 실측 → 검증 ③ → C++ 협의
5. 각 단계 완료 시 SESSIONS_BOARD "비상 세션" 섹션 신설 후 한 줄 보고 (수치 포함)

## 7. 검증 합격선 (§9 검증 의무의 수치화 제안)

- A-1: 고속 이동(v≥1.0) 중 정지 명령 → 오버슈트 < 10cm, 래치 후 드리프트 < 5cm/8s
- A-2: 구역 침범 예정 궤적 → 회피 재계획이 게이트 통과 + 구역 최소 이격 ≥ inflate_m
- B: 인위 30° 교란 → RECOVER 진입 → 수평 복구 → 래치 호버까지 완주, 고도 손실 < 1m
  (합격선 자체도 잠정 — 첫 실측 후 보드에 조정 근거와 함께 갱신하라)
