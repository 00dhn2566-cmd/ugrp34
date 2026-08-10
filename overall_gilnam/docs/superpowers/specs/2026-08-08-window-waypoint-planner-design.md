# 창문 통과 웨이포인트 계획기 (고전, 비학습) — 설계

> 2026-08-08 · 담당: 류길남 · 경로계획(궤적 생성) 빈칸 착수 — RL 담당 확정과 무관하게 유효한 소비측 참조 구현
> 근거 문서: `overall_gilnam/docs/state_window_interface_spec_v0_1.md`(입력, 미확정 후보안), `reinforcement_yunho/interface/waypoints_config.schema.json`(출력), `reinforcement_yunho/rl/README.md`(현황)

## 배경·목적

파이프라인의 경로계획 자리(비전→VIO→**경로계획**→PID)는 담당 미정이고, 윤호 RL 스캐폴드의 baseline은 "창문 중심 직진"뿐이다. 한편 RL→제어 이음새는 이미 확정돼 있다: 성진 컨트롤러 입력은 `waypoints_config`(순서 있는 [x,y,z] + limits)이며 **최소시간 궤적 스무딩은 성진 `plan_waypoints`가 수행**한다. 따라서 경로계획의 실제 빈칸은 궤적 생성이 아니라 **웨이포인트 선정**이다.

본 모듈은 (드론 상태, 창문 3D 맵) → 창문 법선 정렬 접근·이탈점 열 → `waypoints_config`를 만드는 고전 계획기다. 역할: ① 통합 경로의 첫 실동작 경로계획, ② RL 대비 비교 기준(향후), ③ `state_window_interface_spec` 소비측 참조 구현 (spec v1.0 확정 논의의 실물 근거).

## 범위

- 신설 `overall_gilnam/planning/` — 다른 팀원 코드 수정 없음. 윤호 `interface.schemas`는 **import만** (출력 조립·검증 경유, §5 때 vision_msg 경유 원칙과 동일).
- 입력 스키마는 spec v0.1 **후보안(§6.1/§6.2) 기준** — 미확정임을 docstring에 명시, 확정 시 필드명 조정.
- 장애물/벽 맵 없음 (v1): 순차 전진 배치(합성 씬 규격 §4.1) 가정. RL env 연동(관측공간 정책)은 범위 밖.

## 구성요소

| 파일 | 역할 |
|---|---|
| `planning/window_waypoint_planner.py` | 순수 계획 로직 + CLI |
| `planning/planner_limits.yaml` | limits 기본값 (v_max/a_max/j_max/snap_max, dt) |
| `planning/demo_from_scene_gt.py` | scene_gt.json → §6.2 창문 맵 변환 → 계획 실행 데모 |
| `planning/tests/test_planner.py` | 단위 테스트 (conftest로 경로 주입, vision/tests 패턴) |

## 알고리즘 (v1)

1. 창문 맵에서 `passed == false` 창문만 취해 `order_index` 오름차순 정렬. (passed 필드 부재 시 false 취급 — 소유권 미결 스펙 §7 대응)
2. 창문마다:
   - **접근점** = center + d_app·n̂, **이탈점** = center − d_exit·n̂ (n̂ = 접근측을 향하는 단위 법선, spec §3.1 관례. `normal` 필드 부재 시 에러 — corner 유도 법선은 ± 방향 관례가 미확정이라 접근측을 판정할 수 없음, `rl/README.md` 동일 지적 참조)
   - 접근·이탈 직선이 기하적으로 center를 통과하므로 center 웨이포인트는 넣지 않는다 (웨이포인트 최소화 → 성진 최소시간 계획이 더 부드러움)
3. **여유 검사**: min(w, h)/2 − clearance_margin < 0 이면 해당 창문 통과 불가 → 전체 계획 거부(ValueError, 창문 명시). margin 개념은 `eval_target_derivation.md`의 통과 여유와 동일 축.
4. 웨이포인트 열 = [드론 현재 위치] + [접근ᵢ, 이탈ᵢ] 순차 연결 → `interface.schemas.WaypointsConfig` 조립 → `validate()` 통과 확인 후 반환/저장.
5. **경고 수준 검사** (v1 한계 보완): 이탈ᵢ → 접근ᵢ₊₁ 구간이 창문 i+1의 벽 평면을 개구부 밖에서 교차하면 경고 로그 (거부 아님 — 벽 범위 정보가 스펙에 없으므로 판단 불가, 한계로 문서화).

### 파라미터 (planner_limits.yaml + CLI 인자)

- `d_app` / `d_exit`: 접근·이탈 거리 (기본 1.5m / 1.0m — 합성 씬 전방 간격 4~6m 대비 여유. 엔지니어링 기본값, 문서 명시)
- `clearance_margin`: 기체 여유 (기본 0.35m ≈ 휠베이스 450mm 반 + 프로펠러 여유 — margin 회의 확정 시 갱신)
- `limits`: 성진 스키마 그대로 (기본값은 보수적으로: v_max 2.0, a_max 2.0, j_max 10, snap_max 50, dt 0.01 — 성진 협의 전 임시)

## 입출력 계약

- **입력**: spec v0.1 §6.1 드론 상태(`position` 사용, 나머지 무시) + §6.2 창문 맵(`windows[].order_index/center/normal/size_wh/passed` 사용). JSON 파일 2개 (CLI) 또는 dict (라이브러리).
- **출력**: `waypoints_config` dict/JSON — `interface/waypoints_config.schema.json` 준수, `interface.schemas.validate()`로 확인.
- **의존성**: numpy + pyyaml (리포 관례. `interface.schemas`는 리포 내 모듈).

## 테스트 (성공 기준)

1. 접근점이 접근측: dot(접근 − center, n̂) > 0, 이탈점은 반대측
2. order_index 순서 보존 + passed=true 제외
3. 좁은 창문(min(w,h)/2 ≤ margin) → ValueError에 창문 식별 정보 포함
4. 출력이 `validate()` 통과 (waypoints ≥ 2, limits 필수 키)
5. 데모: scene_gt 창문 3개 → 웨이포인트 7개(시작 + 3×2), 각 창문 평면 교차점이 개구부 내부
6. 결정성: 동일 입력 → 동일 출력 (난수 없음)

## 한계·후속

- 벽·장애물 회피 없음 (순차 corridor 가정) — 벽 범위가 스펙에 들어오면 v2에서 경로 검사 승격.
- limits 기본값은 성진 미협의 임시값 — 협의 후 갱신 (yaml만 수정).
- 재계획(창문 맵 갱신 시 호출 주기·hysteresis)은 통합 단계 결정 사항 — v1은 단발 호출 함수로 설계해 어느 쪽으로든 감쌀 수 있게.
- RL env 관측공간 어댑터(baseline 대체) 는 RL 담당 확정 후.
