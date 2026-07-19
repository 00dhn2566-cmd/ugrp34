# 비상(emergency) 세션 상태 (2026-07-19 착수)

임무: [docs/HANDOFF_EMERGENCY.md](docs/HANDOFF_EMERGENCY.md) — 비행 감독자 +
비상 3종(A/B/C) 구현·MATLAB 검증. 규약 원본은 [INTERFACE_SPEC.md](INTERFACE_SPEC.md) §9
(충돌 시 §9가 이김). 여기는 이 세션의 산출물·결정·실측 기록.

## 만든 것

| 파일 | 역할 |
|---|---|
| `flight_supervisor.py` | §9 감독자 골격 v0.1: `flight_state.json` 단일 소유(원자적 쓰기=하트비트), 미션 게이트(`REJECTED_RECOVERING`), 우선순위 중재(B>C>A-1>A-2), A-1/A-2 명령 소비, B/C 통보 처리, `heartbeat_stale()`(철칙 3 컨트롤러 측 기준 구현) |
| `tests/test_flight_supervisor.py` | 단위테스트 24개 (게이트 5모드/우선순위 4조합/A-1 중복 방지/B hash 무효/깨진 입력 견고성) — 전체 스위트 106개 통과 확인 |
| `traj_emergency.py` | A-1 정지 궤적 생성기: 실측 상태에서 저크 제한 최단 정지(후방차분 이산 동역학 — 게이트와 미분 정의 동일) + 래치 hold. 비상 레짐 준수(ZVD 생략/마진 반납/snap 측정만). 스무더 미사용 이유: 동결 기준 추종기는 정지 후 동결점 복귀 = §1 ④ 스냅백 위반 |
| `traj_pipeline.py` `emergency` 동사 | §8 CLI 편입 (`emergency [--state] [--out-dir] [--hold-s]`): STATE_STALE 거부(신선도 0.5s), 산출물 3종 + stdout JSON에 `emergency{stop_point, stop_dist_m, stop_T_s}` 동봉. 감독자 `plan_emergency_stop` action의 실행 대응물 |
| `tests/test_traj_emergency.py` | 19개: 게이트 풀한계 통과/정확식 대비 정지 거리(0.819 vs 0.821m)/xy 0.7 축배분/초기 가속·하강·정지 상태/스냅백 금지(누적 후퇴 <5mm)/CLI 왕복·STALE 거부. 전체 스위트 145개 통과 (path_time yaw 구현과 병행, 회귀 0) |
| `traj_shaping.py` A-2 절 | `keep_out_clearance/check`(box·sphere 이격, 전 샘플 교차, `KeepOutViolation` reject_code=KEEP_OUT_VIOLATION) + `keep_out_avoid_waypoints`(재조밀화 push-out 회피 재계획, 시작/종점 구역 내 = unavoidable 즉사) |
| `traj_pipeline.py` A-2 연동 | build_trajectory·replan_splice 게이트에 keep_out 전 샘플 검사(`keep_out_report` 동봉), 미션 JSON 최상위 `keep_out` 필드, CLI 거부 코드 매핑(예외 reject_code 속성), emergency 동사 `--keep-out`(제동 경로 침범 시 KEEP_OUT_UNAVOIDABLE 보고+원장, 정지는 비거부) |
| `flight_supervisor.py` v0.2 | keep_out 영속화(`output/keep_out.json` — emergency 동사 기본 소비), 러너 `execute_action()`(plan_emergency_stop을 §8 CLI subprocess로 실행, stdout JSON 파싱, hash 승계·원장), C-모드 트리거 감시 `_PowerDegradedMonitor`(포화율>90% 1s AND 고도 오차 증가, w_sat 실측 확정 전 옵트인) |
| `tests/test_keep_out.py` | 14개: 이격 기하(구/박스/모서리)/게이트 연동/CLI KEEP_OUT_VIOLATION/회피 재계획(구·박스 관통 경로, 파이프라인 전 체인 통과)/unavoidable/emergency 불가피 보고+원장 |
| `tests/test_flight_supervisor.py` 확장 | +7개 (총 31): keep_out 영속화, C 트리거 3케이스(발동/고도 유지 시 비발동/비포화 비발동)+기본 비활성, 러너 실전 왕복(subprocess, 산출물·hash·원장 검증). **전체 스위트 169개 통과** |
| `verify_emergency.py` | MATLAB 검증 ①(A-1: 순항 실비행 → τ 실측 상태 채취 → 정지 궤적 합성 재비행 → 오버슈트<10cm/드리프트<5cm/8s 판정)·②(A-2: 회피 경로 실비행 → 실측 최소 이격≥0) 오케스트레이터. **작성만, 미실행** — MATLAB 튜닝 세션 점유. 타 MATLAB 감지 가드 내장 |

## 구현 결정 (골격 v0.1 — §9가 구현 세션에 위임한 것들)

- **컨트롤러 통보 채널 = `controller_events.jsonl`** (RT 경로, append-only,
  감독자는 오프셋 이후 새 줄만 소비). current_state 부가 신호안 대신 파일
  분리 — 물리 보고(§5)와 이벤트(판단 입력)의 소유권 분리 유지. 판단의
  원본은 항상 flight_state.json (§9 상태 보고 절).
- **감독자는 판단만, 실행은 action 반환**: tick()이 `plan_emergency_stop` /
  `plan_controlled_descent` / `check_keep_out_replan` action dict를 돌려주고,
  궤적 생성은 러너가 §8 CLI(path_time 세션 07-19 구현 완료)로 호출 — 철칙 1
  (결정 경로에만)의 코드화.
- **A-2는 모드가 아니라 제약 갱신**: keep_out은 감독자가 보관, 모든 비상
  action에 동봉 (A-1 정지·C 강하 궤적도 구역 회피 대상 — §9 적용 범위).
- **emergency_cmd.json 중복 방지**: written_at 변경 시에만 1회 처리.
- **낮은 우선순위는 활성 비상을 선점 불가**: C 진행 중 A-1은 거부가 아니라
  **유예**(원장 `stop_deferred` 기록) — C 해제 후 상위가 재발행. B 진행 중
  C 통보 무시(B가 이미 최상위).
- 원장 기록은 기존 스키마와 다른 `{event, at, mode, traj_hash, detail}` 형태
  append (추가만 — 기존 소비자 무영향).

## B/C 트리거 임계값 후보 (착수 순서 1번 — 실측 확정 대기)

§8 실측(HANDOFF_EMERGENCY)을 반영한 후보. **모두 잠정 — MATLAB 검증 ③④에서
확정 후 여기 갱신.**

**B (자세 상실, 잠정 45°/0.3s):**
- 함정 1 — 측정 필터 0.05s: 트리거가 보는 측정 자세는 실제보다 수십 ms +
  급기동 시 수 도(7° 실측) 지연. 각도 단독 조건은 발화가 그만큼 늦음.
- 함정 2 — 명령 한계 ±60° > 45°: 절대 자세 조건은 (이론상) 공격 기동에서
  오발화 가능. 현행 궤적은 온건해 실위험 낮으나 조건 설계에 반영.
- **후보 A (기본)**: |roll| 또는 |pitch| > 45° 지속 0.3s (원안 유지).
- **후보 B (지연 보상 병행, §8 권장)**: 후보 A **또는** [|자세 오차(명령
  대비)| > 30° **그리고** |각속도| > ω_c 지속 0.1s] — ω_c는 검증 ③ 교란
  주입에서 정상 회복 시 rate 분포 실측 후 결정 (일상 외란 기준점: 0.3N·m
  ×0.3s 펄스 → 피크 2.28°, 45°는 그 20배라 각도 쪽 오발화 여유는 충분).
- **RECOVER 진입 반사에 적분기 리셋/동결 포함** (§8 권장: anti-windup 부재
  → 포화 장시간 후 해제 역스윙). 평시 경로 불변(골든 유지), 반사 상태만.

**C (추력 부족, 잠정 포화율>90% 1s + 고도 오차 증가):**
- 함정 — 호버가 이미 토크 클램프 평형(§8): "모터 PI 클램프 도달"은 평시에도
  참이라 포화율 신호로 부적합. 신호 후보: ① 믹서 요구(집단+차동 합)가 모터
  한계를 초과하는 샘플 비율 ② 고도 PID 출력 상한(±10) 지속 점유율 + 고도
  오차 증가 추세(원안의 AND 조건 유지).
- 검증 도구 확정: `qc_trace --mission <traj> <out> 0.8 1.0` (Ct 20% 열화,
  제어기 몰래) — C-반사·C-모드 트리거 실측의 출발점 (검증 ④).

**A-1 (재료 완비, 설계 확정치):** stop_dist 2단 정지 정확식(sqrt 근사 금지
— 45cm 오버슈트 사건), ZVD 생략, 마진 반납(풀 한계 v2.0/a2.0/j10), snap
측정만, 실측 상태 사용(`splice --emergency` 경로 이미 존재).

## A-1 구현 노트 (2026-07-19)

- 정지 수학: `_axis_stop()` — 축별 (v0,a0)→(0,0), 제동 정속 0.8·amax(스무더
  ab 규약), release 곡선 |v|≤a²/2j에서 램프아웃. 저크는 0.9x 소프트 한계
  (이산 경계 정확히 걸치는 것 방지 — 실측 jPk 9.0/10). xy 동시 기동 시
  0.7 축배분(게이트 노름 검사 정합).
- 실측 (v0=1.5m/s): 정지 거리 0.8189m (2단 정확식 0.8214m 대비 -0.3%),
  게이트 풀한계 통과. 종단 이산 착지 잔차로 mm급 미세 왕복 존재(누적 후퇴
  ~1mm) — 합격선 10cm 대비 100배 여유라 수용, 테스트는 5mm 상한으로 고정.
- 첫 샘플이 실측 pos에서 시작하고 첫 차분 속도 = v0 — 컨트롤러 승계 연속.
- 시각화 waypoints = [p0, 정지점] 2행. 정지 거리가 1m 미만일 수 있음 —
  path_vis 최소 2점 가드(07-17 패치)가 전제. MATLAB 검증 ① 때 재확인.

## A-2 구현 노트 (2026-07-19)

- 회피 재계획은 **반복마다 재조밀화** 필수 — 구역 중심 정관통 경로에서
  push 방향이 경로 축과 평행하면 점만 좌우로 갈라지고 현(chord)이 구역에
  남는 퇴화 실측 (v1 첫 구현 실패 원인). box 내부 push도 최근접 면 축이
  아니라 **중심 방사** (면 축 방식은 같은 퇴화로 60회 미수렴 실측).
- RDP(2cm) 간소화 후 현이 재침범하면 조밀 경로 유지 (가드).
- emergency 동사에서 keep_out 침범은 **거부가 아니라 보고** — 정지가 관통
  회피보다 우선 (§9). 측방 회피 제동(불가피 판정 전 회피 시도)은 미구현
  후속 (§9 규정은 "측방 회피 제동 우선" — 현재는 직진 제동 + 불가피 보고).

## 다음 일 (착수 순서 기준)

- [ ] 1단계 잔여: 컨트롤러 측 하트비트 감시 + 래치 강하 — C++/Simulink 측
      구현은 튜닝/C++ 세션과 협의(보드 ★ 기존 항목). 파이썬 기준 구현
      `heartbeat_stale()`은 완료 (판정 계약: 부재/깨짐/나이>1.0s = stale).
- [ ] MATLAB 검증 ①·② — `python verify_emergency.py` (작성 완료, 슬롯 대기.
      첫 실행이므로 로그 전체 확인 필수). 합격선: ① 오버슈트<10cm·드리프트
      <5cm/8s, ② 실측 이격 ≥ 0.
- [ ] 5단계: B — Simulink RECOVER 상태기계(반사) + 트리거 실측 → 검증 ③ →
      C++ 동기화 협의(보드 ★). 트리거 후보는 위 절 참조.
- [ ] C 잔여: C-반사(믹서 자세 우선 배분)는 컨트롤러 내장 — Simulink/C++
      수정이라 협의 대상. C-모드 감독자 측(트리거 감시+게이트)은 구현 완료,
      w_sat 임계는 검증 ④(qc_trace Ct 0.8 열화)에서 실측 후 기본값 승격.
      통제 강하 궤적 생성(plan_controlled_descent 러너)도 그때 함께.
- [ ] 검증 ④: C 강하 (Ct 열화 주입, 메모리 수정만).
- [ ] A-1 측방 회피 제동 (§9 "회피 제동 우선" 완전 구현 — 현재는 불가피
      보고만).
