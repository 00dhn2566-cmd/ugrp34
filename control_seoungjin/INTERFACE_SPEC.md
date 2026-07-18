# control_seoungjin 통신 규격 v0.2 (2026-07-16~17)

path_time 파이프라인 ↔ 상위(경로계획) ↔ 하위(컨트롤러) 간 파일 인터페이스 규격.
모든 JSON은 UTF-8, **원자적 쓰기 필수**(임시파일→`os.replace`/rename — 반쯤 써진 JSON 읽기 방지).
좌표는 world frame [m], z는 고도(+위), yaw는 [rad], 시각은 ISO 8601 로컬.

## 0. 저장 경로 체계 (런타임 배치 — Python/C++/MATLAB/Gazebo 공통)

**원칙: 파일 이름·스키마는 불변, 루트만 env로 이동. 루트는 "프로세스 그룹이
돌아가는 쪽" 파일시스템에** (30Hz 파일을 OneDrive나 /mnt/c 너머로 쓰지 말 것 —
sync/9p 잠금으로 원자적 rename 실패·지연).

```
$UGRP_IO_ROOT/                # 미설정 시: repo output/ (개발 기본, 현행 그대로)
  active/                     #   런타임 배치 시 권장 기본:
    trajectory.mat/.json      #   Windows = %LOCALAPPDATA%\ugrp_drone
    trajectory_report.json    #   Linux/WSL(Gazebo 단계) = ~/.ugrp_drone
    current_state.json        # 30Hz — 반드시 로컬 fs (env UGRP_RT_DIR로 단독 이동 가능)
    attitude_feedback.json
    feedback_ledger.jsonl
    param_estimate.json
  runs/<flight_id>/           # 비행별 스냅샷 (sim_result·리포트·로그) — 골든 트레이스/사후 분석
```

- 개발(현재, 3세션 병행): env 미설정 → 전부 repo `control_seoungjin/output/` 플랫
  (임무·비행 단위 저율이라 OneDrive 무해). **current_state만 예외** — 기본이
  `%LOCALAPPDATA%\ugrp_drone\` (§5).
- Gazebo 단계: 컨트롤러(C++)와 시뮬레이터가 같은 리눅스에 있으므로
  `UGRP_IO_ROOT=~/.ugrp_drone` — Windows와의 교환은 runs/ 스냅샷 복사로만.
- 동시 다중 시뮬 인스턴스: 인스턴스마다 별도 `UGRP_IO_ROOT` (active/는 단일 임무 전제).
- `input/`(미션 JSON 예시)은 저장소에 유지 (재현·git 가치).

## 파일 목록

| # | 파일 | 방향 | 갱신 주기 | 용도 |
|---|---|---|---|---|
| 1 | `input/<mission>.json` | 상위 → path_time | 임무 단위 | 경로점 + 계획 스펙 |
| 2 | `output/trajectory.mat` / `trajectory.json` | path_time → 컨트롤러 | 임무 단위 | 성형 완료 궤적 (게이트 통과분만) |
| 3 | `output/attitude_feedback.json` | 컨트롤러 → path_time | 비행 후 1회 | 잔류 지터 실측 → 경로 보정 학습 |
| 4 | `output/feedback_ledger.jsonl` | path_time 전용 (append) | 소비 시마다 | 보정 이력 원장 (처리 여부·경과 시간 조회) |
| 5 | `output/current_state.json` | 컨트롤러 → 모두 | 상시 20~50Hz | 실시간 상태 (재계획 이어붙이기) |

## 1. 경로 JSON (`input/<mission>.json`)

`sample/INPUT_FORMAT.md`의 확장. 필수 키 누락 시 파이프라인이 error로 즉사.

```json
{
  "waypoints": [[x, y, z], ...],          // 필수, N>=2, 첫 점 = 출발점
  "limits": {                              // 필수 — "계획 스펙"
    "v_max": 1.0, "a_max": 0.8,            //   숫자 또는 [x,y,z]
    "j_max": 2.0, "snap_max": 10.0
  },
  "dt": 0.01,                              // 선택 (기본 0.01) [s]
  "waypoint_mode": "stop",                 // 선택: "stop"(기본, 점마다 정지) |
                                           //   "fly_through"(무정지 통과 — 스플라인
                                           //   연속 경로 + 곡률 감속. 급코너는 자동 감속)
  "waypoint_prep": {                       // 선택: 집합 전처리 (merge/divide)
    "merge_dist": 0.01,                    //   근접점 병합 [m] (기본 1cm)
    "collinear_tol": 0.05,                 //   일직선 중간점 병합 [m] — 정지 없이
                                           //   순항 (일직선 5점 12.3→8.8s 실증)
    "max_seg_len": null                    //   긴 구간 분할 [m]
  },
  "shaper": {                              // 선택
    "mode": "zvd",                         //   "zv" | "zvd" (기본) | "none"(A/B 검증용 — 운용 금지)
    "f_mode_hz": 1.8                       //   짐 모드 주파수 (피드백으로 갱신됨)
  },
  "controller_profile": "precision",       // 선택: "precision"(기본) | "balanced" | "agile"
                                           //   — 컨트롤러 위치 게인 프로파일 (임무 단위 전환,
                                           //   튜닝 세션 계약 v1). 값의 진실은 parameters.m
                                           //   ctrl_profile / C++ qc_apply_profile. 파이프라인은
                                           //   검증 후 산출물 3종(.mat/.json/meta)에 동봉만 한다.
  "yaw": {                                 // 선택 (설계 확정 2026-07-19, 구현 대기)
    "mode": "heading",                     //   "heading"(기본, 진행 방향) | "hold"(고정 방위,
                                           //   게걸음) | "look_at"(목표점 주시 — 창문 접근용)
                                           //   | "scan"(구역 스윕 — 주변 물체 탐색용)
    "angle_rad": 0.0,                      //   hold일 때만
    "target": [0.0, 0.0, 1.5],             //   look_at일 때만 (월드 좌표)
    "scan": {"from_rad": -1.57,            //   scan일 때만: 스윕 구간 (한 바퀴는
             "to_rad": 1.57,               //   from 0, to 6.283) — 왕복/일회는 상위
             "sweep": "once",              //   "once" | "back_and_forth"
             "rate_rad_s": 0.5},           //   **필수** — 스캔 속도의 정답은 비전
                                           //   (탐지 주기·블러·FOV)만 안다. 제어는
                                           //   모름 → 기본값 없음, 누락 시 즉사.
                                           //   물리 상한(1.0 잠정) 초과분만 클램프+통지
    "rate_max": 1.0                        //   [rad/s] 선택 — 아래 yaw 물리 잠정치로 클램프
  }
}
```

### yaw 명령 인터페이스 원칙 (설계 확정 2026-07-19, 구현 대기)

- **상위는 "어디를 볼지"만** (mode/target), 회전 시간표는 파이프라인이 산정 —
  waypoint 시간부여와 동일 철학. yaw는 (x,y,z)와 독립 평탄 출력이라 이동 궤적과
  분리 스케줄 (스플라인·성형·시간축은 공유).
- **yaw 물리 한계 (잠정, 실측 대기)**: rate 1.0 rad/s / acc 2.0 rad/s². yaw는
  드래그 토크 차동이라 4축 중 권한 최약 + 모터가 호버에서 이미 토크 클램프
  평형(HANDOFF_EMERGENCY §8 실측) — 과속 요잉 = 포화 = C 비상 인접. 보수 기본값
  필수, 게이트에 yaw rate/acc 검사 추가.
- **완화 정책 일관**: look_at이 요구하는 회전 속도가 한계를 넘으면 (근접 고속
  통과 시 각속도 폭증) 거부하지 않고 rate 클램프 → 주시 오차(pointing error)를
  §7 margins에 연속값으로 보고. 상위가 벌점으로 학습.
- **look_at 특이점**: 목표점 수평 거리 < r_freeze(기본 0.3m)에서는 방위각이
  발산하므로 마지막 yaw 동결 — 창문 통과 순간이 정확히 이 경우 (통과 직전
  yaw 고정, 통과 후 다음 목표로 전환).
- 셰이퍼(ZVD)는 yaw에 비적용 — 요잉은 추력을 기울이지 않아 짐 스윙과 사실상
  비결합 (스윙에 가장 무해한 DOF).
- **단계형 명령 (1~N단계 yaw) 관련 결정 (사용자 질의 2026-07-19)**: 계약은
  연속(float `angle_rad`)으로 열어둔다 — **양자화는 상위(RL) 행동공간 설계
  재량**이며 인터페이스 관심사가 아님. 이산 정책이면 hold에 45° 배수 등 이산값만
  보내면 자연 수용되고, 연속 정책으로 업그레이드해도 계약 불변. (계약은 넓게,
  사용은 좁게.) 운용 기본은 look_at — 창문 좌표만 주면 yaw가 행동공간에서 제거됨.
- **스캐닝은 각도 명령이 아니라 `scan` 프리미티브** (사용자 용도 확인 2026-07-19:
  주변 물체 스캐닝): 스윕 동작은 파이프라인이 등속 생성 — 상위가 각도 시퀀스를
  지휘하지 않는다 (waypoint에 시간을 안 붙이는 것과 동일 철학). 이동 중 스캔
  허용 (yaw 독립 DOF).
- **스캔 속도의 소유권 (사용자 지적 2026-07-19: "얼마나 빨리 스캔할지 제어는
  모른다")**: `rate_rad_s`는 **필수 입력** — 정답은 비전(탐지 주기·블러·FOV)에
  있고 제어 층은 알 수 없으므로 기본값을 두지 않는다 (누락 시 즉사). 제어의
  소관은 물리 상한(잠정 1.0 rad/s, 토크 포화 근거)뿐 — 초과 요청은 클램프 +
  §7 adjustments 통지. 참고 계산(길남 파트용): FOV 90°·탐지 10Hz면 1.0 rad/s에서
  프레임당 ~6° 회전 — 겹침 충분. 요청값 산정은 비전 쪽 몫.
- 배치 프로토콜과의 결합: yaw 블록은 미션(집합) 단위, 새 명령 승리 스플라이스에
  yaw 채널도 동일 연속 조건(각도·각속도)으로 승계.

### waypoint 배치 프로토콜 (상위 call 구조, 사용자 확정 2026-07-16)

- 집합 단위로 도착. 전처리 `normalize_waypoints()`: 근접점 **merge**(기본 1cm) /
  긴 구간 **divide**(옵션). 집합 종점에서는 **기본 정지**.
- **새 명령 승리 policy**: 비행 중 새 집합이 call되면 이전 집합의 잔여 구간은
  버리고 새 집합을 따른다 — `replan_splice(res1, τ, new_set, cfg)`가 τ 시점
  성형 기준 상태(p/v/a/j 연속)에서 무정지로 꺾는 결합 궤적 생성. 스무더~ZV~
  게이트는 결합 타임라인 전체에 일괄 적용(성형기 원칙 1: 상태 연속 보장).
- 비상(기준 대이탈) 단독 재계획만 `build_trajectory(v0,a0)` 경로 — snap은
  측정만(스무더 정지 초기상태 가정 특성), v/a/j는 강제.
- **무명령 기본 정책 (사용자 확정 2026-07-17)**: 명령이 없으면 **현재 자리
  유지 호버** — 명령 부재를 감지한 순간의 실측 위치를 **1회 래치**해서 그 점을
  기준으로 잡는다 (계속 실측 추종 금지 — 바람에 밀리는 대로 기준이 따라가는
  표류자가 됨). 경우별:
  ① 부팅 직후 미션 없음 → 현재 실측 위치 래치 호버 (setpoint가 없는 유일한
  경우라 이 규칙만이 기준을 정의).
  ② 궤적 정상 종료 후 새 명령 없음 → 종점 클램프 유지(Simulink Lookup 외삽;
  tail hold 8s 잔류 0.000° 실측). 종점오차 0mm라 "현재 자리"와 일치 — 동일 정책의
  특수해.
  ③ 비행 중 통신 두절 → 궤적이 전체 선적재라 현행 시간표를 끝까지 비행 후 ②로.
  ④ 기준 대이탈 상태에서 명령 부재 → **마지막 setpoint로 스냅백 금지**, 현재
  자리 래치 (setpoint 복귀는 공격 기동 — 명령 없이는 하지 않는다).
  ⑤ 새 미션 거부 → 기존 궤적 유효 유지.
  **C++ 이식 시 ②의 클램프 외삽 + ①·④의 실측 래치를 모두 구현할 것** — 종점
  이후 기준 미정의는 발산 위험.

**한계 예산 규칙**: `limits`는 지터 상쇄 오프셋 예산을 **빼고** 작성한다.
`limits ≤ (1 − JITTER_MARGIN=0.2) × 물리 한계(v2.0 / a2.0 / j10)` — 초과 시 거부(error).
상쇄 수정이 얹혀도 최종 궤적이 물리 한계 안에 남게 하기 위한 몫이다.

**동적 배분 (지터 실측 연동)**: 승인 기준은 위 0.2로 고정(RL 계약 정상성)하되,
**실제 계획에 쓰는 유효 한계**는 원장의 최근 잔류 지터(신선도 24h, 최근 3건
중앙값)에 따라 자동 조정 — 수렴(≤2°)이면 요청 그대로, 상승(>2°)이면 마진 0.30
(유효 v/a ≤1.4, j ≤7), 심각(>4°)이면 0.35로 온건화. "지터 우선, 속도는 남는
자산" 원칙의 동적판. 적용 내역은 `pipeline_meta.json`에 기록.

**xy 동시 기동 주의**: 대각 이동(어느 샘플에서든 x·y가 동시에 유의미하게 움직임)이
있는 미션은 스무더가 xy 한계에 ×0.7 축배분을 적용하므로, `limits`의 v/a/j도
×0.7을 추가 반영해 작성할 것 (예: v ≤ 1.12). 안 그러면 스무더가 개입해 계획
시각과 실제 궤적이 어긋난다 (개입량은 `pipeline_meta.json`의 `max_dev_m` 경고로 확인).

## 2. 궤적 (`output/trajectory.mat` + `trajectory.json`)

`.mat`(컨트롤러/Simulink 계약)과 `.json`(비MATLAB 소비자용)은 동일 내용. 게이트(v/a/j 3종) 통과분만 기록된다.

| .mat 변수 | shape | 의미 |
|---|---|---|
| `timespot_spl` | (N,1) | 시간 [s], 균일 간격 |
| `spline_data` | (N,3) | 최종(성형+상쇄) 목표 위치 — MATLAB에서 N×3 그대로 사용 |
| `spline_yaw` | (N,1) | yaw [rad], 진행방향 기준 |
| `waypoints` | (M,3) | 경유점 — **Waypoints 블록은 3×M이라 MATLAB에서 전치** |
| `jitter_delta` | (N,3) | 지터 상쇄 레이어 (최종 = 스무딩 + delta). 학습 루프가 이 레이어만 갱신 |
| `controller_profile` | char | 게인 프로파일 (`precision`/`balanced`/`agile`) — 컨트롤러가 로드 시 `ctrl_profile`로 적용 |

`trajectory.json`: `{dt, trajectory_hash, controller_profile, t[], pos[][3], yaw_rad[]}`.
`pipeline_meta.json`(부속): 예산·스무더 개입량·게이트 리포트·`trajectory_hash`.
**`trajectory_hash`** = sha256(t, pos) 앞 16자리 — 피드백이 어느 궤적의 실측인지 대조하는 열쇠.

## 3. 잔류 지터 보고 (`output/attitude_feedback.json`)

컨트롤러(시뮬 후처리)가 쓰고 path_time이 소비하는 **최신 1건** 파일 (덮어쓰기).

```json
{
  "flight_id": "2026-07-16T14-30-00",     // 필수 — 비행(시뮬) 식별자
  "written_at": "2026-07-16T14-35-12",    // 필수 — 기록 시각 (경과 시간 판정 근거)
  "used": false,                           // 필수 — 소비 핸드셰이크 태그
  "trajectory_hash": "32940f664e2e6dc4",  // 필수 — 어느 궤적의 실측인지
  "mode_freq_hz": 1.83,                    // tail 구간 실측 진동 주파수
  "tail": {                                // 도착 후 잔류 진동 (지터 본체)
    "pitch_rms_deg": 1.51, "roll_rms_deg": 0.4,
    "amp_deg": 2.1, "phase_rad": 2.76, "t_ref_s": 30.0
  },
  "moving": { "att_peak_deg": 6.8, "track_rms_cm": 2.8 },
  "k_est": { "kthrust": null, "kdrag": null, "confidence": 0.0 }   // 선택 (K 추정기)
}
```

**소비 프로토콜 (이중 보정 방지 핸드셰이크)**:
1. path_time은 `used:false`인 파일만 소비. `used:true`면 건너뜀.
2. 보정 반영(현재: `mode_freq_hz`→셰이퍼 f0 갱신. 추후: tail RMS→Tm 연장, 카운터스윙 진폭)
3. 궤적 생성이 **성공한 뒤에만** `used:true`로 재기록 (실패 시 태그 유지 → 다음 기회 소비).
4. 소비 내역을 원장(§4)에 append.

**신선도**: 소비 시 나이 = now − `written_at`을 원장에 기록하고 리포트. 나이가 임계
(기본 24h) 초과면 경고 로그 (모델/게인 변경 이후의 낡은 실측일 수 있음 — 적용은 하되 시끄럽게).

## 4. 보정 이력 원장 (`output/feedback_ledger.jsonl`)

**"이미 처리했나 / 언제 이후 얼마나 지났나"를 답하는 단일 창구.** path_time만 쓴다
(append-only, 한 줄 = 소비 1건). used 태그가 "최신 1건의 상태"라면 원장은 "전체 이력".

```json
{"consumed_at": "2026-07-16T15-02-00", "flight_id": "2026-07-16T14-30-00",
 "trajectory_hash": "32940f664e2e6dc4", "feedback_age_s": 1608.0,
 "action": {"f_mode_hz": [1.80, 1.83]},
 "residual": {"tail_pitch_rms_deg": 1.51}}
```

- **처리 여부 판정**: 같은 `flight_id`가 원장에 있으면 이미 처리된 것 (used 태그가 유실돼도 안전망).
- **경과 시간 판정**: 마지막 줄의 `consumed_at`(또는 특정 hash의 마지막 줄)과 now의 차.
- **수렴 판정**(추론 ③ "수렴 시 무수정"): 같은 궤적 hash의 최근 N건 `residual` 추세가 평탄하면 보정 중단.

## 5. 실시간 상태 (`current_state.json`) — schema_version 0.2

**⚠ 저장 경로 (v0.2 확정)**: 이 파일만 repo `output/`이 아니라 **실시간 전용
로컬 경로**에 쓴다 — 이 저장소는 OneDrive 안이라 30Hz 덮어쓰기가 동기화
잠금과 충돌(원자적 rename 실패 위험). 경로 규칙 (쓰기·읽기 공통):
`env UGRP_RT_DIR` → 없으면 `%LOCALAPPDATA%\ugrp_drone\` → (호환 폴백) `output/`.
그 외 통신 파일(궤적/리포트/피드백/원장 — 임무·비행 단위 저율)은 `output/` 유지.

컨트롤러가 비행 내내 **상시 덮어쓰기** (권장 30Hz; C++ 1kHz 루프에서 30Hz 데시메이션,
시뮬 배치에선 To Workspace 후처리로 흉내). **원자적 쓰기 필수** (tmp 파일 → rename).
**구현 대상: C++(controller_cpp) / MATLAB(run_traj_baked 후처리) — 이 스키마 그대로.**

```json
{
  "schema_version": "0.2",
  "timestamp": "2026-07-17T00-15-32.123",   // 쓰기 시각 (신선도 판정 기준, ms 포함)
  "t_sim_s": 12.345,                         // 비행/시뮬 내부 시각 [s]
  "pos": [x, y, z],                          // 측정 위치 [m, world]
  "vel": [vx, vy, vz],                       // 측정 속도 [m/s]
  "acc": [ax, ay, az],                       // 측정 가속 [m/s²]
  "att": { "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 0.0 },
  "ref_state": {                             // ★ 현재 성형 기준의 상태 (재계획용)
    "pos": [...], "vel": [...], "acc": [...],
    "jerk": [...],                           // v0.2 추가 — 아래 근거
    "traj_hash": "652abbbd463d5dac",         // 어느 궤적을 따르는 중인지
    "t_on_traj_s": 12.3                      // 그 궤적의 어느 시점인지
  },
  "motors": { "w_cmd": [w1, w2, w3, w4] }    // 선택 (디버깅/추정기용)
}
```

**설계 근거 (구현자 참고)**:
- **jerk까지 기록하는 이유**: 이어붙이기 다항식(7차)의 경계조건이 양끝 p/v/a/j
  4개씩 — j까지 있어야 스플라이스가 C³ 연속 (snap은 경계조건에 안 들어가고
  게이트가 최종 검증하므로 기록 불필요). ref_state의 jerk는 궤적 데이터의
  해석값이라 노이즈 없음 (측정 미분 아님).
- **traj_hash + t_on_traj_s**: 재계획기가 "지금 상태"를 자기가 만든 궤적 위의
  정확한 점과 대조 가능 — 파일 타이밍 오차에 강건.
- **재계획 이어붙이기**: 평시엔 **`ref_state`에서** (측정 상태 사용 = 피드백 성형
  함정, 성형기 원칙 1 위반). 측정 상태(`pos`/`vel`)는 비상 이탈 재계획에만 +
  스플라이스 온건(Tm≥0.9s).
- **신선도 검사**: 소비자는 timestamp 나이 > 0.5s면 error() 즉사
  (낡은 상태 이어붙이기 = 점프 = 미분킥).
- MATLAB 배치판: 시뮬 종료 후 로그를 30Hz 격자로 후처리해 "마지막 상태" 1건을
  쓰는 것으로 충분 (실시간 흉내는 실기/Gazebo 단계 몫).

## 6. 플랜트 상수 추정 (`output/param_estimate.json`)

`estimate_params.py`가 시뮬 로그(모터 입력 w/T ↔ 센서 출력 자세/vz) 회귀로 생성.
용도: 짐 탑재·프롭 교체·배터리 새그 시 재튜닝 없이 계수 재추정 → parameters.m의
`sT`/`sQ` 정규화 스케일로 게인 자동 보상 (강건 제어의 적응 요소).

```json
{
  "estimated_at": "...", "trajectory_hash": "...",
  "r2_confident_threshold": 0.9,
  "estimates": {
    "k_thrust_lumped": {"value": ..., "unit": "N/(rad/s)^2", "r2": ..., "confident": true},
    "k_drag_lumped":  {"value": ..., "assumes": {"Izz_nominal": ...}, ...},
    "mass_kg":        {"value": ..., "note": "기체+짐 총질량 (추력 로그 직접 회귀)"},
    "inertia": {"Ixx": {..., "assumes": {"arm_length_m": ...}}, "Iyy": {...}, "Izz": {...}}
  }
}
```

- **추정 원리**: 질량 = z 평형(m·(z̈+g)=ΣT·cosφcosθ, K 무관) / K̂_thrust = T↔w² 회귀 /
  K̂_drag·Îzz = yaw 각가속↔차동 w² 회귀 (상보 — 한쪽 공칭 전제) / Îxx·Îyy = roll·pitch
  각가속↔차동 추력 회귀 (팔길이 전제). 프로펠러 배치 부호는 후보 조합 최고 R² 자동 선택.
- **소비 규칙**: `confident:true`(R²≥0.9) 항목만 반영. parameters.m 반영은 급변 방지
  램프/저역 필터 권장 (핸드오프 K-추정기 스펙).

**⚠ 소비 가드레일 3종 (튜닝 세션 확정, 2026-07-16 — 위반 시 보상 체계 붕괴)**:
1. **앵커 절대 불변**: `Kthrust_ref=9.79` / `Kdrag_ref=0.597` / qc_phys ref 호출값
   (1.2726/1.0/0.14)은 "튜닝했던 그 날의 값" — 새 실측이 나와도 갱신 금지.
   앵커를 갈면 정규화 보상(sT/sQ/sIa/sIz/sM)이 통째로 무효.
2. **시뮬 플랜트 일관성 (가장 위험한 함정)**: 시뮬 안에서 질량·관성의 진실은
   parameters.m이 아니라 **CAD 솔리드**다. `propeller.Kthrust`는 블록이 직접 쓰므로
   바꾸면 플랜트도 같이 바뀌어 정합이지만, `drone_mass`/관성값을 실기체 실측으로
   바꾸면 **게인만 스케일되고 시뮬 플랜트는 그대로** → 미스매치로 시뮬 성능이
   오히려 악화. 실기체 실측 질량/관성 반영은 CAD 질량과 함께 바꾸거나,
   **실기체용 파라미터 세트를 시뮬용과 분리**할 것. 즉 `param_estimate.json`의
   질량/관성 추정치는 시뮬 세팅에 자동 반영 금지 — 실기 이전 단계 전용.
3. **단위는 비율로만**: `k_thrust_lumped`(T/w² 집중계수)는 블록 계수와 단위가
   달라 절대값 대입 금지 — sT = 기준치/새치 **비율**로만 사용.

## 7. 상위(경로계획 RL) 궤도 계약 — "이런 궤도는 넘기지 마"의 정형화

임의 궤도를 §1 형식(waypoints 또는 trajectory)으로 던지면, 이 층이 성형·검증하고
**`output/trajectory_report.json`** 으로 기계 판독 가능한 판정을 회신한다
(`python traj_report.py --input <json> [--flight-mat <mat>]`).

### 완화 정책 (계약 v0.2 — "시간 배분으로 살릴 수 있으면 거부하지 않는다")

위치들이 도달 가능한 한, 요청의 **공간 의도는 살리고 시간만 재배분**해서 수용한다.
조정 내역은 `adjustments[]`로 통지 (거부 아님 — RL 벌점용 연속 신호):

| 상황 | v0.1 (구) | v0.2 (현행) |
|---|---|---|
| `limits`가 예산(1.6/1.6/8/64) 초과 | 거부 | **상한 클램프** 후 비행 → `LIMITS_CLAMPED` |
| 원시 궤적 성형 편차 > 0.3m (스텝 등) | 거부 | **경로 보존 재시간화**(RDP ε=5cm로 경로 추출 → path_time이 시간 재배분) → `TIME_DILATED {dilation}` |
| 위 완화로도 불능 | 거부 | 거부 (아래 표) |

미션에 `"strict": true`를 주면 v0.1처럼 클램프 없이 즉시 거부 (검증용).

### 진짜 거부 (완화 불가능한 것만)

| # | 규칙 | 코드 |
|---|---|---|
| 1 | 스키마 준수, `trajectory.t` 단조증가 | `SCHEMA_ERROR` / `TIME_NOT_MONOTONIC` |
| 2 | 재시간화 후에도 성형 편차 > 0.3m | `RESHAPED_BEYOND_TOL` |
| 3 | 성형 후에도 게이트 초과 | `GATE_EXCEEDED` |
| 4 | 재계획 이어붙임은 `current_state.json`의 **ref_state** 기준 (신선도 0.5s) | 파이프라인 error |

여전히 권장 (조정 없이 요청 그대로 날게 하려면): limits ≤ 예산, xy 대각 ×0.7,
저크-가능 Tm ≥ (60·A/(0.8·j_max))^⅓, 스텝 대신 시간-현실적 궤적.

### 회신 스키마 (`trajectory_report.json`, contract_version 0.1)

```json
{
  "verdict": "accepted" | "rejected",
  "reject_codes": [{"code": "...", "detail": "...", "value": 0.42, "limit": 0.3}],
  "margins": {"vxy": 0.69, "axy": 0.53, "jxy": 0.17, "vz": 0.5, "az": 0.13, "jz": 0.01},
  "shaping": {"deviation_max_m": 0.0, "xy_share_applied": 0.7, "jitter_delta_max_m": 0.28},
  "trajectory": {"hash": "...", "duration_s": 34.3, "n_samples": 3435, "shaper": {...}},
  "flight": null | {"track_rms_cm": 2.0, "att_peak_deg": 6.8,
                    "tail_pitch_rms_deg": 0.001, "tail_roll_rms_deg": 0.0,
                    "residual_mode_freq_hz": null},
  "contract_version": "0.1"
}
```

### RL 학습 신호로 쓰는 법 (권장)

- **snap 정책** (사용자 요구 — 회로 내부 부담 근거, 실측 전 잠정 물리 상한 80):
  정지형 waypoint 경로는 계획층(7차 다항식)이 snap_max를 보장하고 게이트가
  4종째로 **강제** 검사. fly_through/원시 궤적 백스톱/비상 재계획 경로는
  구조상(스플라인 C², 뱅뱅 저크) 보장 불가 → **측정·마진 보고만** (`margins.sxy/sz`
  — 정보용, 1.0 초과가 곧 거부는 아님). 회로 실측 나오면 PHYS_SNAP 조정.
- **f0 학습 대역 가드**: attitude_feedback의 실측 주파수는 짐 모드 대역
  (1.0~3.0Hz) 안일 때만 셰이퍼 f0에 반영 — 대역 밖(예: 4.4Hz)은 제어루프
  진동이라 쫓아가면 악화 (A/B/B′ 실증: 4.39Hz 추종 tail 12.25° vs 1.8Hz 고수
  9.93°). 거부 이력은 원장에 `rejected_out_of_band_hz`로 남음.
- **하드 제약**: `verdict=rejected` → 해당 액션 무효 (게이트가 어차피 차단).
- **연속 벌점**: `margins.*`는 물리 한계 대비 피크 비율(1.0=한계) — 1.0에 붙을수록
  여유 없는 궤도. `shaping.deviation_max_m`은 "요청과 실비행의 괴리" — 클수록
  RL이 의도한 경로가 아님.
- **성능 보상**: `flight.track_rms_cm`(추종 정밀도), `flight.tail_*`(도착 후 잔류
  지터 — 짐 흔들림), `trajectory.duration_s`(속도) 트레이드오프.
- reject_codes의 `code` 값은 안정 계약: 추가는 있어도 의미 변경/삭제는 없음.

## 8. 작업 API — 동사 카탈로그 (설계 확정 2026-07-17, **구현 전**)

상위/타 세션이 "이거 하고 싶으면 이거 실행"으로 쓰는 명령 계약. 파일 스키마(§1~§7)가
명사라면 이 절이 동사다. 단일 진입점 `python traj_pipeline.py <verb> ...`로 구현 예정
(하위 호환: verb 생략 시 `plan`으로 동작 — 기존 `--input` 호출 그대로 유효).

| 동사 | 하고 싶은 것 | 명령 | 입력 계약 | 출력 계약 |
|---|---|---|---|---|
| `plan` | 새 미션 → 궤적 생성 | `plan --input <mission.json> [--out-dir]` | §1 | §2 산출물 3종 + §7 report |
| `splice` | **비행 중 새 명령 (새 명령 승리)** | `splice --input <new_mission.json> [--state <current_state.json>]` | §1 + §5 | §2 (결합 궤적, 새 hash) + §7 report |
| `check` | 실행 없이 검정만 (RL 사전 질의) | `check --input <mission.json>` | §1 | §7 report만 (**부작용 없음** — output/ 미기록) |
| `feedback` | 비행 로그 → 지터 보고 | `feedback --log <sim_result.mat>` | 비행 로그 | §3 attitude_feedback (used:false) |
| `estimate` | 플랜트 상수 추정 | `estimate --log <sim_result.mat>` | 비행 로그 | §6 param_estimate |
| `status` | 현황 조회 | `status` | — | §5 요약 + 원장 최근 N건 + 최신 report (stdout JSON) |

공통 규약:
- **종료 코드**: `0` 성공(조정 있어도 성공 — adjustments는 report로 통지) /
  `2` 거부(§7 reject_codes 발생) / `1` 내부 오류. 상위는 코드만 보고 분기 가능.
- **stdout 마지막 줄 = 기계용 JSON 한 줄** (`{"verdict", "report_path", "trajectory_hash"}`),
  사람용 로그는 그 위/stderr. 상위 파서는 마지막 줄만 읽으면 됨.
- `splice`의 `--state` 기본값은 §0 RT 경로의 `current_state.json`. **신선도(0.5s) 위반
  시 거부** — reject_code `STATE_STALE` (§7 코드 목록에 추가 예정, 안정 계약 규칙 적용).
- `check`는 순수 함수처럼 동작 (원장·피드백 소비 없음) — RL이 후보 궤도를 대량
  질의해도 상태 오염 없음.
- 현재 임시 진입점 (구현 전까지의 대응물): `plan` = `traj_pipeline.py --input`,
  `check` ≈ `verify_pipeline.py --static`, `feedback` = `analyze_flight_log.py`,
  `estimate` = `estimate_params.py`, `splice` = `traj_pipeline.replan_splice()` 함수 직접
  호출 (CLI 없음), `status` = 없음.

## 9. 비상(emergency) 규약 — v0.2 (감독자 아키텍처, 사용자 제안 2026-07-18)

### 아키텍처: 비행 감독자 (flight supervisor)

**비행을 장악하는 단일 프로세스**가 모든 명령과 상황 보고를 받아 수락/거부/모드
전환을 판단한다 (PX4 commander 패턴). 아래 A/B 정의와 우선순위는 이 감독자가
집행하는 법이다.

```
[상위 RL/조종] --미션·비상명령--> [감독자] --수락된 미션--> [궤적 파이프라인(§8 동사)]
[컨트롤러] --current_state·이벤트--> [감독자]        └--궤적--> [컨트롤러(제어루프)]
```

- **mode의 단일 소유자 = 감독자**: `flight_state.json` (§0 RT 경로, 원자적 쓰기)
  `{written_at, mode, active_traj_hash, reason}` — mode는
  `normal | recovering | hover_latched | emergency_stopping`. current_state(§5)는
  물리 상태 보고 전용으로 남고 mode 필드는 넣지 않는다 (v0.3 확장 철회 — 소유권
  분리: 컨트롤러=물리, 감독자=판단).
- **철칙 1 — 결정 경로에만**: 감독자는 명령 수락/거부/모드 전환만 (~수 Hz).
  30Hz+ 제어 루프는 컨트롤러 내부에 있고 감독자를 경유하지 않는다.
- **철칙 2 — 반사는 컨트롤러 내장**: B(회생) 트리거·RECOVER 진입은 컨트롤러가
  즉시 자체 수행(감독자 왕복 대기 금지)하고 감독자에 **통보**한다. 감독자는
  통보받아 mode를 갱신하고 이후 명령을 게이트한다 (뇌/척수 분담).
- **철칙 3 — 감독자 부재 시 안전 강하**: 감독자 하트비트(flight_state written_at
  갱신)가 1s 이상 끊기면 컨트롤러는 현행 궤적 완주 후 §1 무명령 default(현재
  자리 래치 호버)로 강하. 감독자 사망이 추락이 아니라 호버로 죽는 구조.
- 명령 게이트: `recovering`/`emergency_stopping` 중 도착한 미션은 감독자가
  `REJECTED_RECOVERING`으로 거부. 우선순위 중재(B > C > A-1 > A-2 > 일반)도 감독자
  단일 지점에서 집행.
- 구현 힌트: C++ 미션 러너가 감독자의 골격 후보. Gazebo/ROS2 단계에선 독립
  노드로 승격 (§8 API의 종료 코드/stdout JSON이 감독자→파이프라인 호출 규약).

세 종류의 비상을 구분한다 (A/B 2026-07-18, C 2026-07-19 사용자 정의).
**우선순위: B > C > A-1 > A-2 > 일반 명령** (감독자 집행).

### 유형 A — 상위 선언형

**A-1. 비상 정지** ("지금 멈춰"): 상위가 `active/emergency_cmd.json` 원자적 쓰기:
```json
{ "written_at": "...", "type": "stop" }
```
→ 궤적 층이 §8 `emergency` 동사로 즉시 정지 궤적 생성: 현재 상태(§5 실측 —
비상은 기준 아닌 **실측** 사용, 기존 규칙)에서 최단 정지 후 그 자리 래치 호버
(§1 무명령 default와 동일한 종착 상태). **비상 레짐 규칙**: ZVD 생략(군지연
1/f0 ≈ 0.56s는 비상에 사치 — 짐 흔들림 감수), 지터 마진 반납(물리 한계
v2.0/a2.0/j10 풀사용), snap 측정만. 정지 거리 수학은 `stop_dist`(traj_smoother
내 2단 정지 정확식) 재사용.

**A-2. 금지 구역** ("거기 가면 큰일"): 미션 JSON(§1) 선택 필드 또는 emergency_cmd로 갱신:
```json
{ "type": "keep_out_update",
  "zones": [ { "shape": "box", "min": [x,y,z], "max": [x,y,z] },
             { "shape": "sphere", "center": [x,y,z], "radius_m": 1.0 } ],
  "inflate_m": 0.5 }
```
- `inflate_m`: 안전 여유 — **드론 반경 + 현수 짐 반경 + 정적 편각 최대 처짐**을
  포함해 산정할 것 (짐은 드론 위치보다 로프 길이만큼 밑에서 흔들린다).
- 검사 지점: 계획(plan)·스플라이스(splice)·비상 재계획 **모두** 게이트에서 전
  샘플 교차 검사. 위반 시 reject_code `KEEP_OUT_VIOLATION` (신규, 안정 계약).
- 비행 중 갱신으로 현행 궤적이 구역과 교차하게 되면: 즉시 회피 재계획(스플라
  이스, 새 명령 승리와 동일 통로). 회피 불가능(현재 위치가 이미 구역 내 등)이면
  A-1 정지로 강등 + 보고.

### 유형 B — 자체 회생형 (자세 상실 복구)

**소관: 컨트롤러 층** (궤적 층 아님 — 자세 상실 시 위치 기준은 무의미).
컨트롤러 상태기계 신설:
```
NORMAL → [트리거] → RECOVER → HOVER_LATCHED → [새 미션 수신] → NORMAL
```
- **트리거(잠정, 구현 세션이 실측으로 확정)**: |roll| 또는 |pitch| > 45°가
  0.3s 지속, 또는 자세 오차 발산율 기준. + 기준 대이탈(추종 오차 > 임계)은
  기존 §1 비상 재계획 경로.
- **RECOVER 동작 순서**: ① 궤적 기준 무시, 자세 수평 명령 (자세 루프만 가동)
  ② 고도 유지/확보 (추력 우선) ③ 안정화 판정(자세 RMS < 임계 지속) 후
  ④ 현재 실측 위치 1회 래치 → HOVER_LATCHED (§1 무명령 default로 합류).
- **궤적 층 역할**: RECOVER 진입 통지를 받으면 현행 trajectory_hash **무효
  선언** (원장 기록). 회복 후 재개는 상위의 새 미션으로만 (자동 재개 금지 —
  구 궤적 이어가기는 스냅백 위험).
- **회생 게인 주의**: 0kg 레짐 붕괴 실측(튜닝 세션 07-17) — 회생 모드 게인은
  질량 추정치와 함께 검증할 것. 짐이 크게 흔들리는 상태에서의 회생이 최악
  케이스 (질량 유효값 요동).

### 유형 C — 추력 부족 통제 강하 (사용자 정의 2026-07-19)

**원리: 자세 권한은 추력 여유에서 나온다.** 모터 4개가 전부 포화되면 차동(자세)
여유가 0 → 자세 상실(B)로 직행. 따라서 힘이 부족하면 **고도를 내주고 자세를
산다** — 통제 강하는 실패가 아니라 B 예방이다. 우선순위는 B > **C** > A-1 > A-2.

- **C-반사 (컨트롤러 내장, 항상 켜짐)**: 믹서 포화 시 집단 추력(고도 몫)을 깎아
  차동(자세 몫) 우선 배분 (PX4 desaturation 패턴). 판단 없는 즉각 반사 — B 반사와
  같은 척수 계열. 감독자 경유 금지.
- **C-모드 (감독자, `power_degraded`)**: 트리거(잠정, 실측 확정) — 모터 명령
  포화율 > 90%가 1s 지속 **그리고** 고도 오차 증가 추세. 동작: 안전 강하율
  (잠정 0.5m/s)로 통제 하강 — 권한 회복 고도에서 호버 재시도, 회복 불가면
  착지까지. 상위 통보 + 미션 게이트(`REJECTED_RECOVERING` 계열, reason에
  power_degraded). 전압/부하 회복으로 포화율이 내려가면 해제 → hover_latched.
- **궤적 층 연동**: `power_degraded` 중 감독자는 파이프라인 호출 시 유효 한계를
  강등해 전달 (동적 마진과 같은 통로 — 지터 대신 추력 여유 근거). 이 강등은
  §7 계약상 adjustments로 상위에 통지된다.
- flight_state.json의 mode에 `power_degraded` 추가.

### 상태 보고

mode는 감독자 소유 `flight_state.json`으로 공표 (위 아키텍처 절 — current_state
확장안은 철회, 소유권 분리 원칙). 상위는 `recovering`/`emergency_stopping` 동안
보낸 미션이 `REJECTED_RECOVERING`(신규 reject_code)으로 거부됨을 예상해야 한다.
컨트롤러의 RECOVER 진입/해제 통보는 이벤트 파일 또는 current_state 부가 신호로
구현 세션이 정하되, **판단의 원본은 항상 flight_state.json**.

### 검증 의무 (구현 세션)

MATLAB 정답 플랜트에서 최소 4편 (④ 추가): ④ C 강하 — 추력 여유 축소 주입
(질량 증가 또는 K_thrust 하향, 메모리 수정만) → C-반사가 자세 유지 + C-모드
통제 강하 진입 → 자세 RMS 유지·강하율 준수 검증. ① A-1 정지 (고속 이동 중 정지 명령 — 정지
거리·오버슈트·래치 검증) ② A-2 회피 (비행 중 구역 갱신 — 회피 재계획 + 불가 시
강등) ③ B 회생 (인위 자세 교란 주입 — RECOVER 상태기계 완주). 각 편 수치는
SESSIONS_BOARD에 보고.

## 공통 규칙

- 대상 파일/키 못 찾으면 조용히 통과 금지 — **error()로 즉사** (저장소 규칙).
- 각도 [deg]는 보고용(JSON), 계산·궤적은 [rad]/[m] SI.
- 스키마 변경 시 이 문서 버전을 올리고 양측(파이프라인·컨트롤러 후처리) 동시 반영.
