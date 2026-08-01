# 제어 파트(성진) 외부 인터페이스 — 팀원 공개용

다른 파트가 제어와 통신할 때 필요한 것 전부. 상세 원본은
[INTERFACE_SPEC.md](INTERFACE_SPEC.md) (조항 번호 병기) — 이 문서와 충돌 시 원본이 이김.

```
[윤호 RL/경로계획] ──미션 JSON──────────────▶ ┌──────────────┐
                  ◀─trajectory_report 회신── │  제어 파트     │──드론 비행──▶
[상위 통합]       ──emergency_cmd──────────▶ │  (블랙박스)    │
                  ◀─flight_state (모드)───── └──────────────┘
[길남 비전]       ←조율: 스캔 속도 산정→   [태민 VIO] ←조율: 실기 상태 출처→
```

제어 내부(궤적 생성·성형·게인·시뮬)는 블랙박스로 취급하면 됨 — **주고받는 파일 5종**만 알면 된다.

---

## 1. 미션 주기 — `mission.json` + `mission.options.json` (§1)

**파일 2개로 분리돼 있다** (형식 정합 2026-08-01). 코어는 윤호
`reinforcement_yunho/interface/waypoints_config.schema.json`과 **바이트 호환**이라
RL 쪽에서 `validate(cfg, kind="waypoints")`로 그대로 검사할 수 있다. 그 스키마가
`additionalProperties: false`라 확장 키를 섞으면 거부되므로, 제어 확장은 사이드카로 뺐다.

```json
// mission.json — 코어 (이것만 보내도 비행 가능)
{
  "waypoints": [[x,y,z], ...],            // 필수. 월드좌표 [m], 첫 점 = 출발점
  "limits": {                              // 필수. 이 미션의 속도 스펙
    "v_max": 1.0, "a_max": 0.8, "j_max": 2.0, "snap_max": 10.0
  },
  "dt": 0.01                               // 선택 (기본 0.01)
}
```

```json
// mission.options.json — 확장 (파일째 생략 가능. 전 키 선택)
{
  "waypoint_mode": "stop" | "fly_through", // 선택. fly_through = 중간점 무정지 통과
  "controller_profile": "precision" | "balanced" | "agile",   // 선택. 게인 프로파일
  "yaw": { ... },                          // 선택. 아래 3절
  "strict": false                          // 선택. true = 완화 대신 거부 원할 때
}
```

- 이름 규칙: 코어가 `foo.json`이면 옵션은 `foo.options.json`.
- 같은 키를 양쪽에 쓰면 **즉시 거부** (조용한 병합 없음).
- 예전처럼 한 파일에 다 넣어도 동작은 하지만, 그 파일은 RL 측 검증을 통과하지 못한다.

**알아둘 정책 4가지:**
- **waypoint는 촘촘히 줘도 된다** — 병합·시간 부여는 제어가 알아서. 시간은 절대 붙이지 말 것.
- **비행 중 새 미션을 보내면 새 미션이 이긴다** — 정지 없이 부드럽게 꺾어서 따라감.
  이전 미션 잔여 구간은 폐기.
- **웬만하면 거부하지 않는다** — 한계 초과는 클램프하고, 물리적으로 급한 궤적은
  시간을 늘려 소화한 뒤 "얼마나 조정했는지"를 회신으로 알려줌. 진짜 거부는
  스키마 오류/시간 역행 같은 치명적 경우뿐 (2절 코드표).
- 명령을 안 보내면 드론은 **그 자리에서 호버** (기본 안전 상태).

## 2. 회신 받기 — `trajectory_report.json` (§7)

미션을 보낼 때마다 돌아오는 성적표. RL 학습 신호로 쓰라고 설계됨.

| 필드 | 의미 | RL에서의 용도 |
|---|---|---|
| `verdict` | accepted / adjusted / rejected | rejected = 액션 무효 (하드 제약) |
| `reject_codes` | SCHEMA_ERROR / TIME_NOT_MONOTONIC / RESHAPED_BEYOND_TOL / GATE_EXCEEDED / KEEP_OUT_VIOLATION | 거부 사유 (안정 계약 — 코드 추가만 되고 의미 변경 없음) |
| `adjustments` | LIMITS_CLAMPED / TIME_DILATED (+팽창 배율) | 연속 벌점 — "요청보다 얼마나 양보됐나" |
| `margins` | 물리 한계 대비 피크 비율 (1.0 = 한계) | 1.0에 가까울수록 여유 없는 궤도 |
| `flight.*` | (비행 후) 추종 RMS, 잔류 지터 등 | 성능 보상 (내부 지표) |
| `command_fidelity.*` | (비행 후) **명령 수행도 — 요청 기준 실측**: waypoint별 통과 오차, 실측 주시 오차, 실측 스캔 완료율, 구역 이격, 갭 분해(plan/track) | **보상은 이걸 우선 사용** — 추종 RMS는 "고쳐진 궤적 대비"라 명령을 많이 고치면 왜곡됨 |
| `command_fidelity.abort_reason` | `superseded` = 새 명령 승리로 대체됨 | **실패 아님 — 보상 계산에서 제외** (안 그러면 재계획 기피 학습) |

## 3. yaw 명령 (§1 yaw 절) — 카메라 방향이 필요할 때

```json
"yaw": {
  "mode": "heading",     // 기본: 진행 방향 (신경 안 쓰면 생략)
  "mode": "look_at",  "target": [x,y,z],     // 이 점을 계속 바라보며 비행 (창문 접근용 권장)
  "mode": "hold",     "angle_rad": 1.57,     // 고정 방위 (이산 정책이면 이산값을 보내면 됨)
  "mode": "scan",     "scan": {              // 주변 훑기
      "from_rad": -1.57, "to_rad": 1.57,
      "sweep": "once" | "back_and_forth",
      "rate_rad_s": 0.5,                     // ★필수 — 비전이 산정 (아래 조율점)
      "priority": "move" | "coupled" | "scan"  // 이동↔스캔 시간 우선순위
  }
}
```

- **look_at 권장**: 창문 좌표만 주면 됨 — 방위각 계산을 RL이 할 필요 없음.
- **scan 속도는 보내는 쪽(비전 요구사항) 책임** — 탐지 주기·블러·FOV로 산정.
  제어는 물리 상한(잠정 1.0 rad/s)만 집행. 누락 시 미션 거부됨.
- scan priority: `move`(도착 우선, 스캔은 남는 시간) / `coupled`(스캔 끝나게 이동을
  늘림, 기본) / `scan`(탐색 국면 — 스캔 동안 저속, 끝나면 풀스피드).

## 4. 비상 (§9)

- **비상 정지**: `emergency_cmd.json`에 `{"type": "stop"}` — 최단 정지 후 그 자리 호버.
- **금지 구역**: `{"type": "keep_out_update", "zones": [{"shape":"box"|"sphere", ...}], "inflate_m": 0.5}`
  — 이후 모든 궤적이 구역을 회피. 현행 비행이 걸리면 즉시 회피 재계획.
- **모드 확인**: `flight_state.json`의 `mode` 필드 —
  `normal / recovering / hover_latched / emergency_stopping / power_degraded`.
  `recovering` 등일 때 보낸 미션은 `REJECTED_RECOVERING`으로 거부되니 mode 확인 후 재전송.
- 드론이 스스로 하는 것 (명령 불필요): 자세 상실 시 자동 회생(수평 복구→호버),
  추력 부족 시 통제 강하. 결과는 flight_state로 통지됨.

## 5. 파일 위치·공통 규칙 (§0)

- 교환 디렉터리: `UGRP_IO_ROOT` (기본 `control_seoungjin/output/`; 런타임은
  `active/` + `runs/<flight_id>/`). 실시간 파일(30Hz)은 `UGRP_RT_DIR`.
- 단위: SI (m, rad, s). 각도 보고만 deg. 좌표는 월드 프레임.
- 모든 JSON에 `written_at` 타임스탬프. 파일 교체는 원자적(임시 파일 → rename).
- 스키마 필수 키 누락 = 즉시 거부 (조용한 무시 없음).

## 6. 열려 있는 조율점 (파트별 to-do)

| 상대 | 조율 내용 | 상태 |
|---|---|---|
| 길남 (비전) | `scan.rate_rad_s` 산정 기준 (카메라 FOV·탐지 주기 → 최대 스캔 속도) | 대기. 참고: FOV 90°·10Hz면 1.0 rad/s에서 프레임당 ~6° |
| 윤호 (RL) | Isaac Sim/ROS2 전환 시 파일→토픽 매핑 (스키마는 그대로, 운반만 변경) | 윤호 환경 구축 후 |
| 태민 (VIO) | 실기에서 드론 상태(위치·자세) 출처를 VIO 추정치로 교체하는 규약 | 실기 단계 |
| 윤호 (RL) | `look_at.target`에 넣을 창문 좌표의 출처 (VIO 재구성 `/window_positions` 연동) | 파이프라인 통합 시 |
