# 웨이포인트 계획기 v2 — 실전 요구사항 흡수 (후진·정렬·정지·z안전·재계획·구조화 경고) — 설계

> 2026-08-18 · 담당: 류길남 · v1 설계: `2026-08-08-window-waypoint-planner-design.md`
> 계기: 윤호 PyBullet 프로토타입(`feat/pybullet-prototype-pipeline`, `prototype_demo/planner.py`)이 우리 `plan_waypoints`를 실검출 파이프라인에서 소비하면서 "v1이 안 하는 부분" 4가지 + 실사고 기반 안전장치 2가지를 래퍼로 구현. 계획기 소유자로서 이를 본체에 흡수해 계획기가 둘이 되는 상황을 막는다.

## 흡수 대상 (전부 윤호 래퍼 실측·설계에서 유래 — 출처 표기)

| # | 동작 | 근거 |
|---|---|---|
| A | **법선 수평 강제**: n ← (nx, ny, 0) 정규화. 수평 성분이 거의 0이면(법선 ≈ 수직) 원본 유지 | 복원 corner 하나만 틀어져도 법선이 35° 기울어 1.5m 접근점이 z=−0.02(지면 아래) → 드론 추락 실측. 창문 수직은 씬 사전지식(synth_scene pitch 0, spec §4.1) |
| B | **게이트 z 클램프** [gate_z_min, gate_z_max] | A 후에도 남는 중심 z 오차의 안전망 |
| C | **후진 검출**: 진행방향(−n)으로 (이탈ₖ − 접근ₖ₊₁)·(−n) > 0 이면 후진량. **완화**: standoff 축소 계수 열(1.0→0.75→0.55→0.4)로 재계획 | 창문 간격 <2.5m(d_app+d_exit)이면 이탈점이 다음 접근점을 지나침 — v1 감사의 "d_app 순환 근거" 지적의 실전 형태 |
| D | **정렬점 삽입**: 이탈ₖ→접근ₖ₊₁ 직선이 창문 k+1 평면을 개구부 밖에서 뚫으면 접근점 뒤 법선상(align_back)에 점 추가 | v1 crossing_warnings가 경고만 하던 사례의 해결 |
| E | **정지점**: 마지막 이탈점에서 법선 반대(진행) 방향으로 stop_ahead 더 나간 점을 종점으로 | 임무 종료 상태 정의 |
| F | **재계획 루프**: 경고 또는 후진이 있으면 완화 단계 상향, max_passes 안에 못 없애면 최선(경고 수, 후진량 사전식 최소)을 남은 경고와 함께 반환 — 조용히 성공한 척하지 않음 | |
| G | **구조화 경고**: `crossing_warnings`가 dict 리스트(`order_index, color, seg_index, a, b, u, v, half_w, half_h`) 반환, 문자열은 `format_warning(w)`로 분리. 기존 문자열 소비자는 `[format_warning(x) for x in ...]` | 재계획 트리거·리포트가 파싱 없이 쓰게 |
| H | **`assemble_window_map` 승격**: `integration/e2e_rehearsal.py` → `planning/window_waypoint_planner.py`로 이동, integration은 re-export | 실전 소비처(윤호 pipeline_demo)가 생김 — 계약 고정 |

## 하지 않는 것
- 최적화/RL 기반 계획 (윤호도 "임시 플래너"라 명시 — v2 범위 밖)
- 창문 간 장애물 회피 (벽 범위 스펙 없음, v1 동일)
- limits/dt 값 변경 (성진 협의 사안)

## 계약

### 유지 (하위 호환)
- `plan_waypoints(drone_state, window_map, cfg, warn=print) -> WaypointsConfig` 시그니처·반환 불변. **v2 동작은 cfg에 키가 있을 때만 활성** — 기존 yaml에 키 추가로 켠다. 키 부재 시 v1과 동일 웨이포인트(회귀 테스트로 보장). `warn`은 이제 `format_warning(dict)` 문자열을 받는다(콜백 시그니처 불변).
- `gate_points(window, d_app, d_exit, clearance_margin)` 유지 + 선택 인자 `force_horizontal=False, gate_z=None`.
- 소비자 무수정: 윤호 `pipeline_demo.py`, 우리 `integration/e2e_rehearsal.py`(assemble import 경로만 re-export로 유지), 테스트 15+3.

### 신규
- `plan_waypoints_v2(drone_state, window_map, cfg) -> Plan` — `Plan(waypoints, labels, warnings: list[dict], passes, shrink, backtrack_m, ok)`. `plan_waypoints`는 내부에서 이걸 호출해 `WaypointsConfig`로 포장.
- `crossing_warnings(...) -> list[dict]`, `format_warning(dict) -> str`.
- cfg 신규 키(전부 선택, `planner_limits.yaml`에 기본값 기입): `force_horizontal_normal: true`, `gate_z: [0.5, 1.9]`, `stop_ahead: 0.6`, `align_back: 0.45`, `max_passes: 4`, `shrink: [1.0, 0.75, 0.55, 0.4]`.
- 라벨: `start`, `align{k}`, `approach{k}`, `exit{k}`, `stop`.

## 검증 게이트
1. 기존 테스트 전부 green (planning 15 · integration 3) — v1 경로 무회귀.
2. **윤호 래퍼 동치**: 동일 입력(창문 3개, cfg 동일)에서 v2 웨이포인트가 `prototype_demo/planner.py._build`와 일치(고정값 회귀 테스트 — 그의 로직을 독립 재구현하므로 값 일치가 흡수의 증명).
3. 단위: 후진 검출·완화 / 정렬점 삽입 조건 / 정지점 / 법선 수평화·수직 법선 예외 / z 클램프 / 재계획 최선 선택·정직 보고 / 구조화 경고 필드 / assemble 승격 후 E2E 게이트 불변.

## 성공 기준
1. 게이트 1~3 통과.
2. `planner_limits.yaml`에 v2 키 기본값 + 주석(출처: 윤호 실측), 계획기 docstring·README 갱신.
3. 윤호 `prototype_demo/planner.py`가 `plan_waypoints_v2`로 대체 가능함을 문서에 명시(대체 자체는 윤호 몫 — 병합 후 통보).

## 한계 (문서 기재)
- 법선 수평 강제는 "창문 수직" 씬 가정 — 기울어진 창문(pitch≠0) 씬에서는 끄고 써야 함(cfg).
- 후진 완화는 standoff 축소뿐 — 그래도 못 없애면 보고만 (구조 변경은 최적화 계획기 몫).
- 정렬점은 1개 삽입 휴리스틱 — 창문 배치가 급격히 꺾이면 부족할 수 있음.
