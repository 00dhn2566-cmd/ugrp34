# control_seoungjin 통신 규격 v0.1 (2026-07-16)

path_time 파이프라인 ↔ 상위(경로계획) ↔ 하위(컨트롤러) 간 파일 인터페이스 규격.
폴더 규약: `input/` = 상위→여기 수신, `output/` = 여기↔하위 교환 (생성물, git 제외).
모든 JSON은 UTF-8, **원자적 쓰기 필수**(임시파일→`os.replace`/rename — 반쯤 써진 JSON 읽기 방지).
좌표는 world frame [m], z는 고도(+위), yaw는 [rad], 시각은 ISO 8601 로컬.

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
  "shaper": {                              // 선택
    "mode": "zvd",                         //   "zv" | "zvd" (기본) | "none"(A/B 검증용 — 운용 금지)
    "f_mode_hz": 1.8                       //   짐 모드 주파수 (피드백으로 갱신됨)
  }
}
```

### waypoint 배치 프로토콜 (상위 call 구조, 사용자 확정 2026-07-16)

- 집합 단위로 도착. 전처리 `normalize_waypoints()`: 근접점 **merge**(기본 1cm) /
  긴 구간 **divide**(옵션). 집합 종점에서는 **기본 정지**.
- **새 명령 승리 policy**: 비행 중 새 집합이 call되면 이전 집합의 잔여 구간은
  버리고 새 집합을 따른다 — `replan_splice(res1, τ, new_set, cfg)`가 τ 시점
  성형 기준 상태(p/v/a/j 연속)에서 무정지로 꺾는 결합 궤적 생성. 스무더~ZV~
  게이트는 결합 타임라인 전체에 일괄 적용(성형기 원칙 1: 상태 연속 보장).
- 비상(기준 대이탈) 단독 재계획만 `build_trajectory(v0,a0)` 경로 — snap은
  측정만(스무더 정지 초기상태 가정 특성), v/a/j는 강제.

**한계 예산 규칙**: `limits`는 지터 상쇄 오프셋 예산을 **빼고** 작성한다.
`limits ≤ (1 − JITTER_MARGIN=0.2) × 물리 한계(v2.0 / a2.0 / j10)` — 초과 시 거부(error).
상쇄 수정이 얹혀도 최종 궤적이 물리 한계 안에 남게 하기 위한 몫이다.

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

`trajectory.json`: `{dt, trajectory_hash, t[], pos[][3], yaw_rad[]}`.
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

## 5. 실시간 상태 (`output/current_state.json`)

컨트롤러가 비행 내내 **상시 덮어쓰기** (20~50Hz, 시뮬 배치에선 후처리로 흉내). 원자적 쓰기 필수.

```json
{
  "timestamp": "2026-07-16T14-30-00.123",
  "pos": [x,y,z], "vel": [vx,vy,vz], "acc": [ax,ay,az],
  "yaw_rad": 0.0,
  "ref_state": { "pos": [...], "vel": [...], "acc": [...] }   // 현재 성형 기준의 상태
}
```

- **재계획 이어붙이기**: 평시엔 **`ref_state`에서** 이어붙일 것 (측정 상태 사용 = 피드백 성형
  함정, 성형기 원칙 1 위반). 측정 상태는 비상 이탈 재계획에만 + 스플라이스 온건(Tm≥0.9s).
- **신선도 검사**: timestamp 나이 > 0.5s면 error() 즉사 (낡은 상태 이어붙이기 = 점프 = 미분킥).

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

### 넘기지 말 것 (거부 규칙 — 어겨도 안전하게 거부될 뿐이지만, 학습 효율을 위해 사전 준수 권장)

| # | 규칙 | 위반 시 코드 |
|---|---|---|
| 1 | `limits` ≤ 0.8×물리 한계 (v/a ≤ **1.6 m/s·m/s²**, j ≤ **8 m/s³**, **snap ≤ 64 m/s⁴**) | `LIMITS_OVER_BUDGET` |
| 2 | xy 동시 기동(대각) 미션은 추가 ×0.7: v/a ≤ **1.12**, j ≤ **5.6** | (성형 개입 → 편차 벌점) |
| 3 | 시간 붙은 궤적의 후방차분 v/a/j도 같은 한계 이내 (스텝·순간정지 금지) | `RESHAPED_BEYOND_TOL` (편차 > 0.3m) |
| 4 | 저크-가능 조건: 이동 진폭 A마다 최소시간 **Tm ≥ (60·A/(0.8·j_max))^⅓** | (성형 개입 → 편차 벌점) |
| 5 | `trajectory.t` 단조증가, 스키마 준수 | `TIME_NOT_MONOTONIC` / `SCHEMA_ERROR` |
| 6 | 재계획 이어붙임은 `current_state.json`의 **ref_state** 기준 (신선도 0.5s) | 파이프라인 error |

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

## 공통 규칙

- 대상 파일/키 못 찾으면 조용히 통과 금지 — **error()로 즉사** (저장소 규칙).
- 각도 [deg]는 보고용(JSON), 계산·궤적은 [rad]/[m] SI.
- 스키마 변경 시 이 문서 버전을 올리고 양측(파이프라인·컨트롤러 후처리) 동시 반영.
