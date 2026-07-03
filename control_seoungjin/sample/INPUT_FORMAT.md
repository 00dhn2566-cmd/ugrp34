# 입력 파일 형식 (`input.yaml` / `input.json`)

다른 사람이 waypoint를 만들어서 이 파이프라인(`run_and_log.py`)에 넣을 때 쓰는 입력 파일 스펙이다.
파일 이름은 자유(`input.yaml`, `input.json`, `config.yaml` 등 무엇이든 상관없음) — 확장자(`.yaml`/`.yml`/`.json`)로 포맷만 자동 판별한다.

## 스키마

```yaml
waypoints:            # (필수) 순서대로 지나갈 점, 2개 이상
  - [x, y, z]          # 단위: m, world frame, z는 고도(위로 갈수록 +)
  - [x, y, z]
  ...

limits:                # (필수) plan_waypoints에 넘어가는 궤적 제약조건
  v_max: <number>       # 최대 속도 [m/s]
  a_max: <number>       # 최대 가속도 [m/s^2]
  j_max: <number>       # 최대 저크(jerk) [m/s^3]
  snap_max: <number>    # 최대 snap [m/s^4]

dt: <number>            # (선택, 기본 0.01) 출력 시간 간격 [s]
```

같은 구조를 JSON으로 쓰면 `config.example.json` 참고.

### 필드 설명

| 필드 | 필수 | 타입 | 설명 |
|---|---|---|---|
| `waypoints` | O | `[[x,y,z], ...]` (N≥2) | 지나갈 순서대로 나열. 첫 점이 출발점. |
| `limits.v_max/a_max/j_max/snap_max` | O | 숫자 **또는** `[x, y, z]` | 숫자 하나면 x/y/z축에 동일 적용, `[x,y,z]`로 축별로 다르게도 지정 가능. |
| `dt` | X (기본 0.01) | 숫자 [s] | 궤적/피드를 몇 초 간격으로 샘플링할지. |

- `yaw`(기수 방향)는 입력으로 받지 않는다 — 진행 방향(속도 벡터의 heading)으로 자동 계산됨.
- `limits`는 축별(x/y/z)로 독립 적용되는 제약이라, 대각선 이동 시 벡터 크기 기준 속도/가속도가 `v_max`/`a_max`보다 커질 수 있음 (README "경로 관련 주의사항" 참고).
- 필수 필드가 빠지면 `run_and_log.py`가 `KeyError`로 즉시 실패한다 (별도 검증/기본값 처리 없음).

## 예시

```yaml
waypoints:
  - [-2.0, -2.0, 0.15]
  - [-2.0, -2.0, 6.0]
  - [0.0, 0.0, 6.0]
  - [2.0, 2.0, 6.0]
  - [5.0, 0.0, 0.15]

limits:
  v_max: 1.0
  a_max: 0.8
  j_max: 2.0
  snap_max: 10.0

dt: 0.01
```

## 이 입력으로 파이프라인 돌리기

```bash
python control_seoungjin/sample/run_and_log.py --config <입력파일 경로>
# 예:
python control_seoungjin/sample/run_and_log.py --config control_seoungjin/sample/input.yaml
```

`--config`를 생략하면 `control_seoungjin/sample/config.yaml`(샘플 입력)을 사용한다.

출력(`control_seoungjin/sample/output/`, git에는 안 올라감):
- `trajectory_feed.csv` — 이 시간에 이 pos/yaw를 PID에 먹였다는 입력 스냅샷
- `sim_result_*.csv` — Simulink 시뮬레이션 결과 신호별 로그 (`sim_result_act_x1.csv` vs `sim_result_des_x1.csv` 등)
- `isaacsim_trajectory.json` — Isaac Sim용 프레임별 pose 목록 (스키마는 잠정적, README 참고)
