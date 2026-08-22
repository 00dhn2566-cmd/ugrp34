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
| 1 | `input/<mission>.json` | 상위 → path_time | 임무 단위 | 경로점 + 계획 스펙 (**코어** — RL seam) |
| 1b | `input/<mission>.options.json` | 상위 → path_time | 임무 단위 | 성진 확장 (선택 — 없으면 전부 기본값) |
| 2 | `output/trajectory.mat` / `trajectory.json` | path_time → 컨트롤러 | 임무 단위 | 성형 완료 궤적 (게이트 통과분만) |
| 3 | `output/attitude_feedback.json` | 컨트롤러 → path_time | 비행 후 1회 | 잔류 지터 실측 → 경로 보정 학습 |
| 4 | `output/feedback_ledger.jsonl` | path_time 전용 (append) | 소비 시마다 | 보정 이력 원장 (처리 여부·경과 시간 조회) |
| 5 | `output/current_state.json` | 컨트롤러 → 모두 | 상시 20~50Hz | 실시간 상태 (재계획 이어붙이기) |

## 1. 경로 JSON (`input/<mission>.json` + `input/<mission>.options.json`)

**코어/옵션 분리 (형식 정합 2026-08-01).** 상위(RL)가 실제로 채우는 계획 스펙은
`sample/INPUT_FORMAT.md`의 3키뿐이고, 그 파일은 윤호
`reinforcement_yunho/interface/waypoints_config.schema.json`과 **바이트 호환**이다.
그 스키마는 `additionalProperties: false`라 **확장 키를 한 개라도 섞으면 RL 측
`validate()`가 거부**하므로, 성진 확장(v0.2)은 전부 사이드카
`<mission>.options.json`으로 분리한다.

```json
// input/<mission>.json — 코어 (윤호 waypoints_config 스키마 100% 준수)
{
  "waypoints": [[x, y, z], ...],          // 필수, N>=2, 첫 점 = 출발점
  "limits": {                              // 필수 — "계획 스펙"
    "v_max": 1.0, "a_max": 0.8,            //   숫자 또는 [x,y,z]
    "j_max": 2.0, "snap_max": 10.0
  },
  "dt": 0.01                               // 선택 (기본 0.01) [s]
}
```

검증: `python3 interface/schemas.py validate input/<mission>.json --kind waypoints`
(윤호 폴더에서 실행. `input/` 예시 6종은 전부 VALID.)

```json
// input/<mission>.options.json — 성진 확장 (파일 자체가 선택. 전 키 선택)
{
  "_comment": "자유 메모 — 코어가 아니라 여기에 둔다",
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
             "rate_rad_s": 0.5,            //   **필수** — 스캔 속도의 정답은 비전
                                           //   (탐지 주기·블러·FOV)만 안다. 제어는
                                           //   모름 → 기본값 없음, 누락 시 즉사.
                                           //   물리 상한(1.0 잠정) 초과분만 클램프+통지
             "priority": "coupled"},       //   "move" | "coupled"(기본) | "scan" —
                                           //   이동↔스캔 시간 배분 정책 (아래)
    "rate_max": 1.0                        //   [rad/s] 선택 — 아래 yaw 물리 잠정치로 클램프
  },
  "strict": false,                         // 선택: true = 클램프 대신 즉시 거부 (검증용, §7)
  "trajectory": {"t": [...],               // 선택: waypoints 대신 쓰는 원시 궤적 입구
                 "pos": [[x,y,z], ...]}    //   (이미 시간 붙음 — 스무더가 재성형 후 게이트).
                                           //   RL seam이 아니므로 코어 스키마 대상 외 —
                                           //   이 입구만 쓰는 미션은 코어 파일에 둬도 된다.
}
```

**병합 규칙 (`traj_pipeline.load_mission`)**

- 옵션 파일은 코어와 **같은 이름 + `.options.json`**. 없으면 전부 기본값.
- 코어 키(`waypoints`/`limits`/`dt`)를 옵션 파일에 두면 **즉사** — 계획 스펙이 두
  파일로 흩어지면 어느 쪽이 진실인지 알 수 없다.
- 같은 키가 양쪽에 있으면 **즉사** (조용한 병합 금지, §공통규칙).
- **하위 호환**: 확장 키가 코어 파일에 인라인으로 있어도 그대로 동작한다.
  `_legacy_inline_options`로 표시하고 `[호환]` 한 줄을 통지 — 그 파일은 RL 측
  `validate()`를 통과하지 못하므로 새 미션은 분리해서 쓸 것.

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
  §7 adjustments 통지. **기본값은 "스캔 안 함"** — yaw 블록 자체가 선택이고
  기본 mode=heading.
  **[조율점 해소 — 비전 회신 2026-08-01, 원문 `overall_gilnam/docs/scan_rate_estimate.md`]**:
  **시뮬 = 1.0 rad/s** (Isaac 렌더 모션 블러 없음 → 블러 제약 비구속, 물리
  상한 그대로) / **실기 = 0.6 rad/s** (블러 제약 지배: ω ≤ blur_px/(t_exp·fx)
  = 0.67의 보수화). 프레임 커버리지 항은 3.8 rad/s로 비구속. 재검토 트리거:
  intrinsics 확정(윤호)·실기 노출시간 실측·탐지 주기 실측 — 그때 비전이 숫자만
  갱신 (일반식은 문서에).
- **스캔↔시간 배분 정책 3종 (사용자 설계 2026-07-19: "국면마다 우선이 다르다")**:
  스캔은 자기 소요시간(구간각/rate)이 있는 명령이라 이동 시간표와 결합된다.
  결합 방식은 `scan.priority`로 상위가 선택 — 숫자 우선도가 아니라 **의미가
  명확한 정책 열거** (enum — 추가만 허용, 의미 변경 금지):
  - `"move"` (이동 우선): 이동 시간표 불변(최속 도착). 스캔은 이동 중 진행하고
    잔여분은 도착 후 호버에서 완료. 도착 시점 완료율은 report에 `scan_coverage`
    (0~1)로 보고 — 상위가 학습 신호로 사용.
  - `"coupled"` (기본, 동등): 미션 시간 = max(이동, 스캔). 스캔이 길면 이동을
    균일 팽창해 이동 중 완료 (한 동작) — §7 `TIME_DILATED` 통지 (reason: scan).
  - `"scan"` (스캔 우선, 3상): ①스캔 완료까지 이동은 저속(스캔 시간 안에 도달
    가능한 만큼만 전진) ②스캔 완료 순간부터 잔여 구간 풀스피드 ③이후 이동 우선.
    탐색 국면용 — 부수 효과: 스캔 중 저속 = 패럴랙스·블러 감소로 비전 품질 유리.
  - 공통 불변: 어느 정책에서도 **스캔 rate는 요청값 유지** (비전의 제약 — 임의
    가감속 금지, 물리 상한 클램프만 예외 + yaw 성형 상한도 요청 rate로 고정 —
    따라잡기 과도가 rate를 넘으면 블러 계약 위반). 이동이 스캔보다 길면 정책
    무관하게 스캔 완료 후 마지막 각도 유지.
  - snap 정책 연동 (구현 확정 2026-07-19): `coupled`/`scan`은 시간 왜곡(재보간)
    을 거치므로 다항식 snap 보장 범주 이탈 — §7 정책대로 **snap 측정-only 강등**
    (v/a/j는 그대로 강제). `move`는 hold 패딩뿐(경계 jerk=0, 이음새 매끈)이라
    snap 강제 유지.
  - 참고: 각도 양자화(1~N단계 yaw)는 상위 재량으로 계약 밖(위 결정), 시간 배분
    정책은 의미론이라 계약 안 — 둘은 다른 층의 문제.
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

## 5b. 실시간 능력 표 (`capability.json`) — schema_version 0.1

**누가 읽나**: 상위 경로 생성기(계획기/RL). **누가 쓰나**: 컨트롤러 측 `capability.py`.
저장 경로·원자적 쓰기 규칙은 §5 와 같다 (`env UGRP_RT_DIR` → `%LOCALAPPDATA%/ugrp_drone/`).
갱신율은 30 Hz 가 필요 없다 — **~5 Hz 또는 값이 바뀔 때**로 충분하다.

**왜 필요한가**: [PERFORMANCE.md §8b](PERFORMANCE.md) 능력 카드는 **질량별 정적 표**다.
그런데 지금 줘도 되는 스펙은 (a) 현재 짐 질량, (b) 지금 감지되는 외란 크기,
(c) 지금 측정되는 시간 지연 세 가지로 달라진다. 이 파일은 그 정적 표를 기저로 삼아
**지금 이 순간 값**으로 깎아 내보낸다. 상위는 `limits` 를 그대로
`path_time.plan_waypoints(v_max, a_max, j_max, snap_max)` 에 넣으면 된다.

```json
{
  "schema_version": "0.1",
  "timestamp": "2026-08-22T09-14-07.412",
  "basis":    { "pkg_kg": 1.0, "profile": "precision", "mass_source": "scheduled" },
  "limits":   { "v": 0.444444, "a": 0.515704, "j": 1.690362, "snap": 8.86501 },
  "budget":   { "track": 0.04, "overshoot": 0.05, "settle": 2.2, "keepout_inflate": 0.1 },
  "yaw":      { "scan_rate": 1.0, "align_deg": 15.0, "step90_s": 2.1 },
  "observed": { "rho": 0.31, "yaw_err_deg": 8.0, "rho_eff": 0.31, "latency_s": 0.045 },
  "degraded": { "active": true, "time_scale": 0.6556,
                "reasons": ["disturbance", "latency"], "hold_until_recovered": true },
  "guarantees": { "recovery": "full", "pulse_dev_deg": 2.3 },
  "shaper":   { "brake_share": 0.8 },
  "valid_for_s": 1.0
}
```

### 필드

| 키 | 뜻 | 상위가 할 일 |
|---|---|---|
| `limits.{v,a,j,snap}` | **지금 줘도 되는 궤적 한계** (감쇄 이미 반영) | `plan_waypoints` 에 그대로 투입 |
| `budget.track` / `overshoot` | 추종·오버슈트 여유 [m] | keep-out inflate·통과 여유 산정 |
| `budget.settle` | 정지 후 정착 [s] | 정밀 작업 전 대기 |
| `budget.keepout_inflate` | 권장 금지구역 팽창 [m] | `keep_out` 설정 |
| `yaw.scan_rate` / `align_deg` / `step90_s` | yaw 계획 규칙 | scan 속도·heading 정확도 기대치 |
| `observed.*` | 관측 원값 | 로깅·판단 참고 (계획엔 `limits` 만 쓰면 됨) |
| `degraded.time_scale` | 감쇄 배율 `s` | 진단용 — `limits` 에 이미 반영돼 있음 |
| `degraded.reasons` | `disturbance` / `latency` | 왜 느려졌는지 |
| `degraded.hold_until_recovered` | 회복 전까지 안 되돌림 | 곧 복구될 거라 기대하고 계획하지 말 것 |
| `guarantees.recovery` | `full` / `bounded_only` | `bounded_only` 면 **외란 복구 전제 계획 금지** |
| `shaper.brake_share` | 쉐이퍼 제동 여유 | 참고 |
| `valid_for_s` | 유효 기간 [s] | 이보다 낡으면 재읽기 |

### 감쇄 규칙 — 시계 배율 `s` 하나로 표현

경로 기하를 바꾸지 않고 **시간축만 늘리는 것**과 동치라, 상위는 "s 배 느리게"만 알면 된다:

```
v ∝ s,   a ∝ s²,   j ∝ s³,   snap ∝ s⁴
```

- `s` 는 **외란 권한 점유율 `rho`** 에 선형 비례해 줄어든다 (`rho_stop` 0.90 에서 `s_min` 0.10).
- **yaw 오차도 `rho` 로 환산해서 같이 본다** (`|e_psi| / 45도`). 돌풍이 끝나도 기수가 틀어져
  있으면 스펙을 되돌리지 않기 위해서다 — "회복까지 유지" 가 별도 타이머 없이 성립한다.
- **지연은 속도에만 건다**: `v <= 0.5 * budget.track / latency`. 지연 × 속도가 곧 위치
  오차이므로, 추종 예산의 절반까지만 지연 몫으로 내준다 (나머지 절반은 게인·외란 몫).

관측 예 (`python capability.py`):

| 조건 | v | a | j | snap | s | reasons |
|---|---|---|---|---|---|---|
| 1 kg 평시 | 1.200 | 1.200 | 6.00 | 48.0 | 1.000 | — |
| 1 kg, rho 0.31 | 0.787 | 0.516 | 1.69 | 8.87 | 0.656 | disturbance |
| 1 kg, rho 0.31, 지연 40 ms | 0.500 | 0.516 | 1.69 | 8.87 | 0.656 | disturbance, latency |
| 0 kg 평시 | 0.900 | 0.750 | 6.00 | 48.0 | 1.000 | — |
| 0 kg, rho 0.7, yaw 30도 | 0.200 | 0.037 | 0.07 | 0.12 | 0.222 | disturbance |
| 1 kg agile | 1.600 | 1.600 | 8.00 | 64.0 | 1.000 | — |

### 소비 규약 (읽는 쪽이 지켜야 하는 것)

§5 의 `current_state` 와 달리 이 파일은 **낡아도 즉사시키지 않는다** — 능력 표는 천천히
변하는 값이라, 잠깐 못 읽었다고 임무를 멈추는 게 더 위험하다. 대신:

| 상황 | 상위가 할 일 |
|---|---|
| 나이 `<= valid_for_s` | 그대로 사용 |
| `valid_for_s` 초과 ~ 5 s | **마지막 값 유지**, 단 `limits` 를 0.7 배로 보수화 |
| 5 s 초과 / 파일 없음 | **정적 표(PERFORMANCE §8b)의 0 kg 열**로 폴백 — 가장 보수적인 기체로 간주 |
| `degraded.hold_until_recovered` 가 true | 곧 복구될 거라 **가정하고 계획하지 말 것** |
| `guarantees.recovery == "bounded_only"` | 외란 복구를 전제한 계획 금지 (0 kg 이 이 경우) |
| `basis.anchor_provisional` 가 true | 해당 질량 앵커가 잠정(2 kg 은 1 kg 복사본) — 보수적으로 볼 것 |

### rho 는 무엇을 넣나 (쓰는 쪽 규정)

```
rho = max( |u_yaw| / limit_yaw ,  |u_att| / limit_att )
```

제어기 PID **출력**의 클램프 대비 점유율이다. 정상상태에서 적분기는 "외란을 상쇄하는 데
필요한 명령"을 들고 있으므로, 이 값이 곧 외란 추정치다 — 별도 센서가 필요 없다.
`diagnose_yaw_final.m` 이 이미 `authSS`/`authPk` 라는 이름으로 계산하던 값과 같다.
표본은 제어 주기에서 뽑되 `capability` 갱신 주기(~5 Hz)에 맞춰 **구간 최대값**을 쓴다
(평균을 쓰면 짧은 포화를 놓친다).

물리 환산: **`tau_max ≈ 0.317 N·m`** (yaw). `rho 0.315` ↔ `0.10 N·m`.
산출 근거는 [SPEED_GOVERNOR.md §4](docs/SPEED_GOVERNOR.md).

### 스키마 진화 규칙

- **필드 추가는 마이너 올림** (`0.1` → `0.2`), 소비자는 **모르는 필드를 무시**해야 한다.
- **필드 삭제·의미 변경은 메이저 올림** (`1.0`), 소비자는 major 가 다르면 **거부**한다.
- `limits` / `budget` 키 집합은 계약의 핵심이라 마이너에서 줄이지 않는다.
- 단위는 `units` 블록에 동봉한다 — 필드명(`v`, `a`)만으로는 알 수 없다.

### 열린 항목 (2026-08-22 기준 미완)

1. **양쪽 끝이 아직 안 붙었다** — 컨트롤러가 이 파일을 쓰지 않고(MATLAB/C++ 미배선),
   계획기가 읽지도 않는다. 규격과 생성기만 있는 상태.
2. **`observed.rho` 를 실제로 채우는 경로 없음** — 위 규정대로 제어기에서 뽑아야 한다.
   현재는 호출자가 인자로 넣어주는 것만 지원.
3. **2 kg 앵커가 1 kg 복사본** (`anchor_provisional`로 표시만 해둠).
4. **`budget.overshoot` 이 쉐이퍼 설정과 연동 안 됨** — 정지 거동을 바꾸면(거리 연동
   포락선) 오버슈트 예산도 달라져야 하는데 지금은 정적 표 값 그대로다.
5. **지연 표본 주입 지점 미정** — `latency_tracker` 에 무엇을 넣을지(상태 파일 나이 /
   명령→응답 / 계획 왕복) 셋 다 가능하다고만 적혀 있고 배포 구성이 안 정해졌다.

### 연산 부하 → 지연 예측 (`compute_load.py`)

지연을 **재고 나서** 반응하면 늦다. 부하는 선행 지표라 — 무엇을 얼마나 자주 돌릴지는
미리 알 수 있으므로 — 지연이 나타나기 전에 깎을 수 있다.

```
cost(n) = fixed + per_unit * n        실측(2026-08-22, 이 노트북):
                                        traj_smoother  25.1 us/샘플 (R2 0.9995)
                                        plan_waypoints 5.3 ms/세그먼트
duty    = sum(cost_i * rate_i)
지연 R  = S + duty*S / (2*(1-duty))   (M/D/1 근사, duty->1 에서 발산)
```

**근거는 예상량과 실측값 둘 다** (`LoadGovernor.fuse`):

| 원천 | 역할 | 왜 |
|---|---|---|
| 예상 (부하 모델) | **선행** | 부하가 올라가는 순간, 지연이 나타나기 전에 잡는다 |
| 실측 | **백스톱** | 모델이 모르는 원인(OneDrive 잠금·GC·타 프로세스)까지 잡는다 |

둘 중 나쁜 쪽을 적용하고, 차이(`실측 − 예상`)를 `model_bias_s` 로 들고 있어 모델이 계속
과소예측하면 드러난다. `observed.load` 블록으로 상위에 같이 내보낸다.

**복귀도 포함**: 부하가 줄면 스펙을 되돌린다. 다만 **올릴 땐 즉시, 내릴 땐 dwell 후 지수 감쇠** —
대칭으로 두면 경계에서 스펙이 요동치고, 그게 재계획을 유발해 부하가 또 오르는 양의 되먹임이 된다.

스펙을 깎는 대신 **재계획 지평을 줄이는** 손잡이도 있다 (`horizon_for_budget`) — 상위가 고른다.

그림: [figure/10_capability/](figure/10_capability/)

### 시간 지연 추적 (`latency_tracker.py`)

지연은 한 샘플이 아니라 **평균**으로 판단한다 — 단발 스파이크로 스펙을 깎으면 순항
속도가 요동친다. EMA 두 개를 쓴다: 빠른 EMA(8표본)로 감지, 느린 EMA(60표본)를 예측치로
내보낸다. 진입에는 연속 초과 3표본(`arm_n`), 해제에는 연속 정상 30표본(`hold_n`)이 필요하다.

표본으로 무엇을 넣어도 된다 — `current_state.json` timestamp 나이(§8c T8),
명령→응답 등가 지연(T4), 계획 동사 왕복(T7). 셋 다 "상위가 본 세상이 얼마나 낡았나"라는
같은 단위다.

**§8c T3(센서·통신 지연) 이 "미측정" 으로 남아 있는데, 이 추적기가 그 자리를 채운다.**

---

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
  "command_fidelity": null | {              // 명령 수행도 (설계 2026-07-19, 구현 대기)
    "mission_completed": true,              // 완주 여부 (false면 abort_reason)
    "waypoint_hit_cm": [0.9, 1.3, 0.4],     // 요청 waypoint별 실제 최근접 통과 오차
    "duration_requested_vs_actual_s": [9.2, 9.4],
    "pointing_rms_deg": 2.1,                // look_at/scan: 비행 중 실측 주시 오차
    "scan_coverage_actual": 1.0,            // 계획이 아니라 실비행 기준 스캔 완료율
    "keep_out_min_clearance_m": 0.8,        // 구역 최소 이격 실측 (음수 = 침범)
    "fidelity_gaps": {"plan": 0.05, "track": 0.02}   // 갭 분해: 의도→궤적 / 궤적→비행
  },
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
- **명령 수행도 (`command_fidelity`, 사용자 지적 2026-07-19 "명령을 얼마나 잘
  수행했나도 지표")**: 추종 RMS는 "내 궤적 대비"라 파이프라인이 명령을 많이
  고쳤으면 추종이 완벽해도 명령 수행은 나쁠 수 있다. 그래서 **의도 대비 실비행**
  을 갭 2개로 분해해 보고: `fidelity_gaps.plan`(의도→궤적 — 클램프·팽창·재성형
  총량) + `fidelity_gaps.track`(궤적→비행 — 추종). waypoint별 통과 오차, 실측
  주시 오차(pointing_rms), 실측 스캔 완료율, 구역 이격까지 **전부 요청 기준**.
  RL 보상은 track_rms(내부 지표)보다 command_fidelity(의도 지표)를 우선 권장.
  **구현 확정사항 3건 (설계 리뷰 2026-07-19 밤 — 어기면 지표가 거짓말함):**
  1. `fidelity_gaps`는 **스칼라 금지, 성분별 dict** — `plan: {clamp_ratio,
     dilation, reshape_dev_m}` / `track: {rms_cm, endpoint_cm}`. 단위가 다른
     양(비율/시간/거리)을 한 숫자로 합치는 공식은 정의 불가 — 합성은 RL 보상
     설계자의 몫으로 넘긴다.
  2. `waypoint_hit_cm`은 **계획 통과 시각 ±수 초 시간창 내** 최근접 거리 —
     시간 무관 최근접은 왕복 경로에서 "돌아올 때 스친 것"을 통과로 오인.
     RDP 병합된 요청 점도 측정 대상 (병합은 계획 사정, 의도 아님).
  3. `mission_completed: false`의 `abort_reason`에 **`superseded`(새 명령 승리로
     대체됨)를 별도 코드로** — RL이 이를 실패 벌점으로 먹으면 "명령 변경 = 벌"을
     학습해 재계획 기피 정책이 나온다. superseded는 보상 계산에서 제외할 것.
  - 구현 참고: 주시 오차 실측은 비행 로그에 yaw 실측 채널 필요 —
    run_traj_baked.m 로그 탭에 yaw 채널 확보 여부부터 확인.
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
| `emergency` | **A-1 비상 정지** (§9, 구현 2026-07-19 비상 세션) | `emergency [--state <current_state.json>] [--out-dir] [--hold-s 2.0] [--keep-out <keep_out.json>]` | §5 (**실측** 사용) + §9 keep_out(선택, 기본 `output/keep_out.json` — 감독자 영속화) | §2 산출물 3종 (정지 궤적, 비상 레짐: ZVD 생략·마진 반납·snap 측정만) + stdout JSON에 `emergency{stop_point,stop_dist_m,stop_T_s}`·`keep_out` 동봉. 제동 경로 구역 침범 시 거부 대신 `KEEP_OUT_UNAVOIDABLE` 보고+원장 기록 |

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
  `normal | recovering | hover_latched | emergency_stopping | power_degraded`
  (C 추가 2026-07-19, 아래 유형 C 절). current_state(§5)는
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
- **적용 범위 (사용자 지적 2026-07-19: "하강 중에도 피하면서 내려가야")**:
  금지 구역은 **위치 제어가 살아 있는 모든 모드에서 유효** — 비상도 예외 아님.
  - C 통제 강하: 수직 하강이 아니라 **구역 회피 하강 궤적**으로 계획 (바로 아래가
    구역이면 측방으로 흘리며 하강 — 저추력 상태에서도 측방 기울임 비용은 작음).
  - A-1 정지: 제동 경로가 구역을 관통 예정이면 측방 회피 제동 우선, 물리적으로
    불가피하면 침범을 이벤트로 보고 (`KEEP_OUT_UNAVOIDABLE`, 원장 기록).
  - B 회생: **유일한 면제** — 자세 상실 중엔 위치 제어가 없어 준수 불가능.
    단 회생 후 래치 지점이 구역 안이면 안정화 후 최근접 경계 밖으로 저속 이탈
    (감독자가 이탈 미션 발행).
- 비행 중 갱신으로 현행 궤적이 구역과 교차하게 되면: 즉시 회피 재계획(스플라
  이스, 새 명령 승리와 동일 통로). 회피 불가능(현재 위치가 이미 구역 내 등)이면
  A-1 정지로 강등 + 보고.
- **구현 확정 (2026-07-19 비상 세션)**: 미션 JSON 선택 필드는 최상위
  `"keep_out": {"zones": [...], "inflate_m": 0.5}` (위 zones 스키마 그대로) —
  plan/splice 게이트가 최종 성형 궤적 전 샘플을 검사해 위반 시
  `KEEP_OUT_VIOLATION` 거부. 회피 재계획 프리미티브는
  `traj_shaping.keep_out_avoid_waypoints()` (재조밀화 push-out, 시작/종점이
  구역 안이면 unavoidable로 즉사 = A-1 강등 신호). 감독자 keep_out 영속화
  파일 `output/keep_out.json`을 emergency 동사가 기본 소비. 검사 리포트는
  산출물 res의 `keep_out_report{min_clearance_m, violated}`.

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
