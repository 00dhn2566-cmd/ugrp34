# path_time 파이프라인 구축 상태 (2026-07-16 세션)

HANDOFF_PATHTIME_PIPELINE.md의 착수 순서를 따라 구축. 이 문서는 산출물 목록 + 세션 중 발견사항 기록.

## 만든 것

| 파일 | 역할 |
|---|---|
| `traj_shaping.py` | MATLAB 성형기 3종 포팅: `traj_smoother`(포락선) / `traj_zv`(ZV·ZVD) / `traj_gate`(v/a/j 3종 검증) + `smooth_with_axis_sharing`(xy 동시기동 ×0.7 축배분) + `counter_swing_offset`(지터 소거 2호기 훅) |
| `traj_pipeline.py` | 체인 본체: `input/*.json` → plan_waypoints → 균일 재샘플 → 스무더 → ZV/ZVD → 게이트 → `output/trajectory.mat`·`trajectory.json`·`pipeline_meta.json`. attitude_feedback used 핸드셰이크 포함 |
| `input/example_mission.json` | 경로 JSON 스키마 예시 (INPUT_FORMAT.md 확장: `shaper` 블록 추가) |
| `analyze_flight_log.py` | 지터 검출기(쓰기 측): sim_result_baked.mat → tail RMS·영교차 주파수·사인 피팅(amp/phase) → attitude_feedback.json (used:false) |
| `INTERFACE_SPEC.md` | 통신 규격 v0.1 — 5개 파일 인터페이스(경로 JSON/궤적/피드백/원장/실시간 상태) |
| `estimate_params.py` | 플랜트 상수 추정기: 모터 입력↔센서 출력 회귀 → K̂_thrust/K̂_drag/질량/Îxx·Îyy·Îzz + R² 신뢰도 → `output/param_estimate.json` (INTERFACE_SPEC §6) |
| `tests/` | 단위·통합 테스트 45개 (`python -m pytest tests/ -q`, control_seoungjin/에서) |

## 설계 결정 (사용자 확정 반영)

- **예산 우선순위 (사용자 확정 2026-07-16)**: 지터 소거가 물리 한계 예산의 1순위
  (자세제어와 직결), 시간 부여(속도)는 **남는 자산**으로. 현행 JITTER_MARGIN=0.2가
  이 원칙의 정적 구현 — 수렴 시 마진을 줄여 속도 회복하는 적응형은 추후.

- **한계 예산 구조**: 물리 한계(2.0/2.0/j10) = 계획 한계(입력 JSON limits) + **지터 상쇄 오프셋 예산**(`JITTER_MARGIN=0.2`). 시간 부여 스펙은 상쇄 여유를 빼고 작성 — 입력 limits가 0.8×물리 한계를 넘으면 즉사.
- **지터 상쇄 레이어 분리 보관**: 최종 궤적 = 스무딩 궤적 + delta. delta는 trajectory.mat의 `jitter_delta`로 저장 — attitude_feedback 학습 루프가 이 레이어만 갱신.
- **지터 소거 2호기 = 역위상 카운터 가속** (사용자 구상): `counter_swing_offset()` — tail:{amp,phase,t_ref} 실측 기반 역위상 사인 오프셋. 1.8Hz에선 저크가 지배 제약이라 진폭 자동 클램프(예산 j=2.0 → A≤1.4mm ↔ 카운터 가속 ~0.18 m/s²). **자세°↔가속 교정 상수 확보 전까지 파이프라인 미연결** (교정 인프라: `diagnose_swing_calib.m`).
- **attitude_feedback 1차 추론**: ②(mode_freq→셰이퍼 f0 갱신)만 구현. ①(tail RMS→Tm 연장)은 구간 매핑 확정 후.
- 셰이퍼 기본 ZVD (주파수 오차 강건, 핸드오프 권장 후보).

## 발견사항 (MATLAB 원본 traj_smoother.m 공통 — 백포팅 후보 2건)

1. **vmax 순항 진입 저크 스파이크**: 원본 3항 클램프에는 속도 상한 접근 테이퍼가 없어, vmax 도달 순간 a가 한두 샘플 만에 꺾이며 저크 −70 m/s³ (한계 7 설정 시) 실측. 기존 MATLAB 검증(1m 스텝)은 vmax 미도달이라 안 걸렸음. Python판은 이산-정확 속도 여유 테이퍼(a_cap = jmax·(√(2.25dt²+2h/jmax)−1.5dt))로 해결 — v/a/j가 정확히 한계에 안착함을 테스트로 확인.
2. **종점 헌팅 리밋사이클**: 제동 트리거 뱅뱅(진입 EPS_G=2mm / 이탈 0.85 히스테리시스) 특성으로 살인 궤적의 종점 수렴부에서 ±1~5cm, 0.02Hz 준정적 배회가 감쇠하지 않고 지속 (§W의 "추종 RMS 2.8cm"와 정합). path_time 정품은 무개입이라 실해 없음(백스톱 경로에서만 발생) — Python판도 원본 거동 유지, 수정하려면 MATLAB과 함께.

백포팅은 MATLAB 쪽 diagnose_smoother.m 재검증과 함께 할 것 (구운 모델 불변).

## run_traj_baked.m 접합 검증 결과 (이 세션에서 최초 실행)

파이프라인 산출 trajectory.mat(예시 미션 5경유점, 34.3s)을 구운 모델에 주입:

- **추종: x/y RMS 1.4cm, z RMS 0.5cm, 종점오차 1~2mm** / 자세 RMS 2.54°, 최대 pitch 6.8°
- act/des 내장 로그 형식 확정: **SaveFormat Array(double)** — run_sample_sim.m 패턴대로
  Clock→`sim_time` 동승 로깅으로 해결 (run_traj_baked.m 패치 완료)
- Scope 버스 매핑 전수 확보 (로그에 출력): Element15=pos, 21=roll(cmd측), 22=pitch(cmd측),
  3=Chassis.pitch, 4=Chassis.roll, 26~28=Load.px/py/pz 등
- tail(도착 후) 관측 마진 `T_hold=8s` 추가 — 잔류 지터 분석 구간 확보

## 셰이퍼 A/B 실증 (지터 유발 실험)

- **셰이퍼 off + 공격적 다중 경유(왕복 대각, v1.1/a1.1/j4.9) → 발산** (x 종점오차
  97m, roll 피크 50.7° = §W 클램프 미분킥 시그니처). 기준 궤적이 게이트를 통과해도
  (물리 가능) 짐 스윙 무상쇄면 발산 가능 — **ZV/ZVD는 장식이 아니라 안정성 요건**.
- 교훈 → `analyze_flight_log.py`에 유효성 게이트 추가: 추종 RMS > 30cm(비행 실패)면
  피드백 거부 (발산 비행의 tail 신호는 짐 모드가 아니라 발산 과도 — 학습 오염 방지).
- **단일 고속 이동(2m, v1.1/a1.1/j4.9) A/B**: A(셰이퍼 off) tail pitch/roll RMS
  7.9/10.1° @ 4.39Hz → 피드백 루프 완주(소비·f0 갱신·원장) → B(ZVD@4.39Hz) tail
  9.5/7.7° @ 4.22Hz — **개선 없음**. 판정: 이 4.2~4.4Hz 링잉은 짐 모드(1.8Hz)가
  아니라 **게인 영역 진동**(사용자 진단: 짐-가변 게인 미조정) — 궤적 층 ZV로는
  원리적으로 소거 불가. z축도 정속 고도 미션에서 1.86m 딥(추력 커플링) — 같은 원인.
  **결론: 이 레짐은 게인(소숫점 깎기/짐-가변) 선행 → 그 후 궤적 층이 잔여(진짜
  짐 모드) 담당.** 온건 레짐에선 ZVD가 tail 0.001°로 이미 완전 소거 확인.
- 루프 기계 장치(측정→피드백→핸드셰이크→원장→f0 갱신→재비행)는 실전 왕복 검증 완료.

## 최종 목표 오케스트레이션 (2026-07-16 후반, 16~17차 게인 확정 후)

Phase 1 — 정식 미션 (`example_mission.json` → 체인 → 새 게인 비행) 완주:
- 추종 x/y RMS 1.3cm / z 0.5cm / 종점 1mm. 도착 후 잔류 지터 **0.001°** →
  분석기 "수렴, 무수정" 판단 (추론 ③ 경로 실동작).
- **파라미터 추정기 실데이터 검증**: 질량 **2.2712kg** (17차 실측 2.2726 대비
  오차 0.06%, R²=0.98) + K̂_thrust 1.39e-5 (R²=1.0) — 둘 다 confident.
  관성/K_drag는 온건 비행에 차동 가진 부재 + 짐 스윙 토크 미모델로 게이트가
  거부 (정상 동작 — 참조는 17차 실측 §Y).
- **공중 창 교훈**: 저고도(z<1m)에선 현수 짐이 지면에 앉아 추력이 드론만
  듦(1.21kg 오측) → 질량 추정은 z>1m 공중 구간 필수 (Z_AIRBORNE_M).

## Phase 2 지터 A/B/B′ 최종 판정 (2m 단일 고속 이동, v1.1/a1.1/j4.9)

| 변형 | tail pitch/roll RMS | 합성 | 판정 |
|---|---|---|---|
| A: 셰이퍼 off | 7.87 / 10.10° @4.39Hz | 12.80° | 지터 유발 기준선 |
| B: ZVD@4.39 (실측 추종) | 9.50 / 7.73° @4.22Hz | 12.25° | **개선 없음 — 잘못된 표적** |
| B′: ZVD@1.8 (짐 모드 고수) | 6.43 / 7.56° @3.94Hz | **9.93° (−22%)** | 추종·자세 전 지표 최고 |

- 결론 ①: ZVD는 **짐 모드(1.8Hz)를 정조준**해야 함 — 실측 주파수가 대역 밖이면
  제어루프 진동(게인 영역)이라 추종하면 악화 → 파이프라인에 **f0 대역 가드
  (1~3Hz)** 추가, 대역 밖 거부는 원장에 `rejected_out_of_band_hz` 기록.
- 결론 ②: 잔여 ~4Hz 링잉 + z 딥(1.86m)은 궤적 층 소관 아님(전 변형 공통) —
  짐-가변 게인(사용자 작업 영역) 이슈로 이관.

## 상위 배치 프로토콜 구현 (사용자 확정 구조)

- `normalize_waypoints()`: 집합 전처리 merge(1cm)/divide(옵션)
- `replan_splice()`: **새 명령 승리 policy** — 비행 중 새 집합 call 시 τ 시점
  성형 기준 상태(p/v/a/j 연속)에서 무정지로 꺾음. 결합 타임라인 일괄 성형이라
  게이트 4종(snap 포함) 전부 통과. `waypoint_mode: fly_through`(무정지 통과,
  곡률 감속)도 추가.
- snap 4종째 게이트 (회로 부담 근거, PHYS_SNAP=80 잠정): 정지형 계획 경로 강제,
  백스톱/fly_through/비상 경로는 측정·마진 보고만 (구조상 보장 불가).

## 남은 일

- [ ] tail hold 실행 로그로 지터 루프 1바퀴 실증 (analyze → feedback → pipeline 소비 → f0 갱신)
- [ ] counter_swing_offset 파이프라인 연결 (스윙 교정 상수 확보 후 — `diagnose_swing_calib.m`)
- [ ] attitude_feedback ① 추론(tail RMS→해당 구간 Tm 연장) — 구간 매핑 설계 후
- [ ] current_state.json 신선도 검사 + ref_state 이어붙이기 재계획
- [ ] MATLAB 백포팅 2건 (위 발견사항) — diagnose_smoother.m 재검증과 함께
