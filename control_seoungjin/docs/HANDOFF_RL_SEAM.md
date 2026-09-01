# RL(윤호) ↔ 제어(성진) seam 정합 인수인계 — 2026-08-01

**읽는 사람**: 윤호(RL/Isaac Sim), 그리고 이 스레드를 이어받는 세션.
**한 줄**: 윤호 `interface/` 스키마와 성진 `INTERFACE_SPEC v0.2` 사이의 형식 충돌을
**성진 쪽을 바꿔서** 해소했고(코어/옵션 분리 + 변환기 신설), 윤호 씬 → 성진 궤적 →
MATLAB 실비행 → Isaac Sim 입력 파일까지 **전 구간을 2개 씬으로 왕복 검증**했다.

---

## 1. 무엇이 충돌했나

윤호 `reinforcement_yunho/interface/waypoints_config.schema.json`은
`"additionalProperties": false`다. 그래서 성진 §1 미션 JSON의 확장 키
(`waypoint_mode`, `waypoint_prep`, `shaper`, `controller_profile`, `yaw`, `strict`,
`_comment`)를 **한 개라도 섞으면 RL 측 `validate()`가 거부**했다.

```
구형 플랫 (확장 인라인)  -> REJECT: Additional properties are not allowed
                                   ('waypoint_mode', 'yaw' were unexpected)
```

즉 스키마가 안전망이 아니라 **기능 차단기**로 작동했다 — `fly_through`로 날거나
`look_at`으로 창문을 주시하는 순간 RL 쪽 자체 검증에서 막혔다.

## 2. 해결 — 미션 JSON 코어/옵션 분리

**코어** `input/<mission>.json` — `waypoints`/`limits`/`dt`만. 윤호 스키마와 **바이트 호환**.
**옵션** `input/<mission>.options.json` — 성진 확장 전부. 파일째 생략 가능.

```jsonc
// input/lookat_mission.json  (코어)
{ "waypoints": [[0,0,1], [2,2,1.5], [4,0,1]],
  "limits": {"v_max":1.0, "a_max":0.8, "j_max":2.0, "snap_max":10.0},
  "dt": 0.01 }

// input/lookat_mission.options.json  (확장)
{ "_comment": "...",
  "waypoint_mode": "fly_through",
  "yaw": {"mode": "look_at", "target": [2.0, 3.0, 1.5]} }
```

**병합 규칙** (`traj_pipeline.load_mission`)

| 상황 | 동작 |
|---|---|
| 옵션 파일 없음 | 전부 기본값 |
| 코어 키를 옵션 파일에 둠 | **즉사** (계획 스펙이 두 파일로 흩어지는 것 차단) |
| 같은 키가 양쪽에 | **즉사** (조용한 병합 금지, §공통규칙) |
| 확장 키가 코어에 인라인 (구형) | **동작함** + `_legacy_inline_options` 표시 + `[호환]` 통지 |

`input/` 예시 6종을 분리했다. `step_mission.json`은 `trajectory` 입구(RL seam 아님)라
코어 스키마 대상 외로 두었다.

### 윤호 쪽에서 검증하는 법

```bash
cd reinforcement_yunho
python3 interface/schemas.py validate ../control_seoungjin/input/lookat_mission.json --kind waypoints
```

`input/` 코어 6종 전부 `VALID (waypoints) [jsonschema]`.

## 3. 신설 — 성진 산출물 → Isaac Sim 입력 변환기

성진 내부 계약과 윤호 스키마는 모양이 다르다. 경계를 넘기는 모듈을 새로 만들었다:
**[`control_seoungjin/isaacsim_export.py`](../isaacsim_export.py)** (여기 말고 다른 곳에서
`isaacsim_*` 파일을 만들지 말 것).

| 입력 (성진 내부) | 출력 (윤호 기대) |
|---|---|
| §2 `trajectory.json` `{dt, trajectory_hash, controller_profile, t[], pos[][3], yaw_rad[]}` | `isaacsim_trajectory.json` `{fps, frames[{time, position, yaw_rad, orientation_quat_wxyz}]}` |
| 비행 로그 `sim_result_baked.mat` `{sim_time, prop1_w..prop4_w}` | `isaacsim_motor_commands.json` `{fps, frames[{time, motor_cmd_w[4]}]}` |

```bash
python isaacsim_export.py \
    --trajectory output/trajectory.json \
    --flight-mat controller/Quadcopter-Drone-Model-Simscape/sim_result_baked.mat \
    --out-dir output/
```

**설계 결정 3가지**

1. **`trajectory_hash`를 못 넣는다.** 윤호 스키마가 `additionalProperties:false`라
   대조 열쇠를 동봉할 수 없다 → `isaacsim_trajectory.meta.json`으로 분리.
   미션 코어/옵션 분리와 같은 패턴.
2. **쿼터니언은 WXYZ** (`interface/` 규약): `[cos(y/2), 0, 0, sin(y/2)]`.
   윤호 `rl/` 쪽 `drone_state`는 XYZW라 서로 다르다 — 이 파일은 `interface/`
   소비자용이므로 WXYZ가 맞다.
3. **모터 부호를 손대지 않는다.** 출력에 음수가 섞이는데(`[634.8, -634.1, -637.2,
   636.7]`) 모터 2·3이 내장 역회전이라 실측 w가 음수인 게 정상이다
   (TUNING_STATUS 9차). 절댓값을 씌우면 요 토크 부호 정보가 사라진다.

## 4. 검증 결과 — 전 구간 왕복 2회

```
윤호 rl/window_env.sample_scene (창문 좌표)
  → 코어+옵션 미션 JSON        [윤호 WaypointsConfig.validate() 통과]
  → traj_pipeline plan          [accepted, 게이트 통과]
  → MATLAB run_traj_baked       [실물리 추종 + prop*_w 로그 생산]
  → isaacsim_export.py          [윤호 스키마 VALID ×2]
```

| | seed 0 | seed 1 |
|---|---|---|
| 씬 | 창문 3개 | 창문 2개 |
| plan | accepted, 24.63s, hash `1470a8ec76802fce` | accepted, 21.66s, hash `ec911128fa1685ff` |
| 마진 (최대) | vxy 0.52 | — |
| MATLAB 자세 | RMS 0.78° / 최대 pitch 2.4° | RMS 0.72° / 최대 pitch 2.1° |
| MATLAB 추종 RMS x/y/z | 5 / 2 / 10 mm | 5 / 2 / 10 mm |
| 종점오차 3축 | 0.000 m | 0.000 m |
| 실비행 창문 통과 오차 | 0.5 / 0.2 / 0.1 cm | — |
| tail 잔류(도착 후 6s) | pitch·roll RMS 0.000° | — |
| `isaacsim_trajectory.json` | 2464 프레임 / fps 100 ✅ | 2224 프레임 / fps 100 ✅ |
| `isaacsim_motor_commands.json` | 3741 프레임 / fps 114.6 ✅ | 3465 프레임 / fps 114.6 ✅ |
| `motor_cmd_w` 범위 | −825.9 ~ 824.6 rad/s | −825.7 ~ 824.8 rad/s |

윤호 쪽 소비 코드로도 확인: `infer_kind` 자동 판별, `TrajectoryFile`/
`MotorCommandsFile` dataclass 왕복, `trajectory_frame_to_T()` 정상 4×4 반환.

거부 경로도 확인:

| 입력 | 윤호 검증기 |
|---|---|
| 분리형 코어 | VALID |
| 구형 플랫 (확장 인라인) | REJECT — `Additional properties are not allowed` |
| `limits.snap_max` 누락 | REJECT — `'snap_max' is a required property` |
| waypoint 1개 | REJECT — `too short` |

기존 테스트 **169개 전부 통과** (회귀 0).

> **주의**: `motor_cmd_w`는 계산값이 아니라 Simscape 시뮬의 `prop1_w~prop4_w`
> **실측 로그**다. MATLAB을 돌리지 않으면 이 파일은 생성되지 않는다.

---

## 5. ★ 윤호 확인 요청 (성진 코드는 어느 답이든 불변)

### 5.1 로터 인덱스 → 기하 + 회전방향 **미확정**

`motor_cmd_w = [w1,w2,w3,w4]`를 현재 **Simulink Prop1~4 번호 순서 그대로** 내보낸다.
Isaac Sim이 어느 암에 어느 값을 물릴지는 윤호 결정 사항이다
(`interface/README.md`에도 같은 취지로 적혀 있음). **순서가 틀리면 파일은 VALID인데
드론이 뒤집힌다.** 인덱스 `0..3` 각각에 대해 (a) 어느 암/위치, (b) CW/CCW를
공표해 달라. 확정되면 `--rotor-order`로 재배열한다.

### 5.2 회신 경로(`trajectory_report.json`) 스키마가 윤호 쪽에 없음

성진 §7 `trajectory_report.json`은 *"RL 학습 신호로 쓰라고 설계"*된 것인데
(`verdict`/`reject_codes`/`adjustments`/`margins`/`command_fidelity`), 윤호
`interface/`에 대응 스키마·dataclass가 없다. **RL 보상이 붙을 자리가 비어 있다.**
`reject_codes`는 성진이 "안정 계약(추가만, 의미 변경 없음)"으로 못박았으니 enum으로
박아도 안전하다.

### 5.3 추론 어댑터 17차원 vs 배포 체크포인트 60차원

| | 관측 차원 |
|---|---|
| `rl/state_window_adapter.py` (윤호가 만든 **추론/통합** 경로) | 17 |
| `rl/window_env.py` `WindowTraversalEnv` | 17 |
| **배포된 `rl/models/ppo_window_3win.zip`** | **60** |
| `rl/pybullet_window_env.py` | 60 |

배포된 체크포인트가 **윤호 자신의 추론 어댑터에 꽂히지 않는다.** 어느 쪽이 본선인지
확인 필요. (성진 파트에는 영향 없음 — 성진은 웨이포인트만 받으면 된다.)

### 5.4 두 isaacsim 스키마는 아직 **PROVISIONAL**

윤호가 Isaac Sim 쪽 형식을 확정하면 `isaacsim_export.py`의 변환 로직만 고치면 된다.

---

## 6. 아직 안 된 것 (성진 쪽 잔여)

1. **Isaac Sim이 실제로 읽어서 도는지 미확인.** 스키마 검증만 통과한 상태다
   (개발 노트북은 Isaac Sim 최소 사양 미달 — 윤호 환경/클라우드 GPU 몫).
2. **C++ 경로(`controller_cpp/qc_io`)에 export가 없다.** 현재 변환기는 파이썬이라
   MATLAB 경로 산출물을 처리한다. `qc_io`는 §2 소비 / §3·§5 생산만 하고 isaacsim
   출력 담당이 없다 → **Gazebo/C++ 단계로 넘어가면 Isaac Sim으로 가는 seam이 끊긴다.**
   C++가 이미 `pos`/`yaw`/`w_cmd`를 들고 있으므로 키 매핑 + yaw→WXYZ + 프레임
   누적만 하면 된다.
3. **정책 실물 롤아웃 미실행.** 검증에 쓴 웨이포인트는 씬의 창문 중심을 그대로 쓴
   대역이고 정책 출력이 아니다. 실제 정책을 태우려면 `pybullet` +
   `gym-pybullet-drones`가 필요한데, Windows에 MSVC가 없어(로컬 툴체인 MinGW) 빌드
   실패 → 윤호 환경에서 할 일. **성진 파트에는 불필요** (입력 형식만 같으면 동일).
4. **문서 오타**: `INTERFACE_SPEC.md §7` 제목은 `contract_version 0.1`인데 실제
   `traj_report.py` 출력은 `"0.2"`다.

---

## 7. 바뀐 파일 지도

| 파일 | 변경 |
|---|---|
| `traj_pipeline.py` | `MISSION_CORE_KEYS`/`MISSION_OPTION_KEYS`, `mission_options_path()`, `load_mission_options()`, `load_mission` 병합·하위호환 |
| `isaacsim_export.py` | **신설** — 성진 → 윤호 형식 변환기 |
| `input/*.json` (6) | 확장 키 제거 → 코어만 |
| `input/*.options.json` (6) | **신설** — 확장 사이드카 |
| `INTERFACE_SPEC.md` | §1 코어/옵션 분리 + 병합 규칙, 파일 목록에 1b |
| `EXTERNAL_INTERFACE.md` | §1 팀 공개용 갱신 |
| `sample/INPUT_FORMAT.md` | RL seam 코어 스키마 주석 + 검증 명령 |
| `SESSIONS_BOARD.md` | 08-01 기록 3건 |
