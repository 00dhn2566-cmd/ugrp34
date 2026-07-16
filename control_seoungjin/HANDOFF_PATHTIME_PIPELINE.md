# 인수인계: path_time 파이프라인 구축 (다른 Claude 세션 전용)

작성 2026-07-16. 이 문서 하나로 시작 가능하도록 자립 작성. 컨트롤러/모델 세부는 자매 문서 [HANDOFF_PATH_TO_CONTROLLER.md](HANDOFF_PATH_TO_CONTROLLER.md), 튜닝 전 과정은 `controller/.../TUNING_STATUS.md` §V~§W 참고.

## 이 세션이 만들 것

상위(경로계획 RL 등)가 던진 **경로 JSON → 시간 파라미터화 → 물리 성형 → 진동 억제 → 컨트롤러가 먹는 궤적**까지의 Python 파이프라인. 즉 `path_time.py`를 축으로, 아래 "궤적 전처리 체인"을 조립하는 것이 핵심 과제. **MATLAB/Simulink 쪽은 이미 완성**(아래 모듈들)이라 손댈 필요 없음 — Python에서 같은 알고리즘을 쓰거나, MATLAB 모듈을 호출하거나 둘 중 택.

## 파이프라인 순서 (고정 — 순서 역전 금지)

```
경로 JSON (input/)
  → path_time.py         : 아크길이 재매개화 → 곡률 → 속도프로파일 → 시간매개화 (snap까지 고려)
  → traj_smoother        : min/max 도달가능성 포락선 (물리 v/a/j 한계 강제)
  → traj_zv              : ZV/ZVD 입력셰이퍼 (1.8Hz 짐 모드 상쇄)
  → traj_gate            : 최종 검증 (v/a/j 노름 초과 시 error) — 통과분만 컨트롤러로
  → 컨트롤러 (trajectory.mat / JSON)
```
- **순서 이유**: ZV는 볼록결합이라 스무더의 한계를 보존하지만, 순서를 바꾸면(ZV 먼저) 스무더가 임펄스 간격을 뭉개 상쇄가 깨진다. 게이트는 언제나 맨 끝(최종 산출물 검사).

## 이미 완성된 자산 (재사용 — 재구현 금지)

### Python
- **`control_seoungjin/path_time.py`** — 시간 파라미터화 본체. 주요 함수:
  - `reparameterize_by_arc_length(x,y,z,...)` / `compute_curvature_and_kN(...)` / `generate_velocity_profile(s,kappa,v_max,a_max,j_max)` / `generate_pid_reference(...)` / `plan_waypoints(waypoints,...)` (7차 다항식 최소시간)
- `control_seoungjin/sample/waypoints_to_maneuver_input.py` — waypoint→`trajectory.mat` 변환 참고구현
- `control_seoungjin/sample/INPUT_FORMAT.md` — 기존 config.json 입력 스펙

### MATLAB (알고리즘 참조 또는 직접 호출; Python 포팅의 정답지)
- **`controller/.../Scripts_Data/traj_smoother.m`** — min/max 포락선 성형기. 상·하한 3항 + 정확 2단 정지거리 트리거. **구현 원칙 4개**(§ 아래) 반드시 준수.
- **`controller/.../Scripts_Data/traj_zv.m`** — `traj_zv(t,pos,fMode,'zv'|'zvd')`. ZV=[½,½]@반주기, ZVD=[¼,½,¼]@반주기(주파수 강건). 현재 fMode=1.80Hz.
- **`controller/.../Scripts_Data/traj_gate.m`** — `traj_gate(t,pos,vmax,amax,doError,jmax)`. xy는 노름, z는 별도. v/a/**j 3종** 검사.

### 성형기 구현 원칙 4개 (위반 시 실측된 함정 재현 — TUNING_STATUS §V/§W)
1. **상태(v,a)는 출력의 후방차분으로 정의.** 저크 적분 병렬 전파는 데드비트 저크 한계사이클(정상 궤적에 0.37m 개입) 유발. 드론 측정값 사용 금지(피드백 성형으로 변질).
2. 한계는 envelope 실측(v/a≈2.5)보다 깎은 값(2.0/2.0/j10) 권장.
3. **각 축 독립 성형**하되 **xy 동시기동 경로는 한계 ×0.7 축배분** (대각 √2 초과 방지 — 박스투어 실증).
4. 정지거리는 sqrt 근사 말고 정확 2단(저크 스윙+정속 제동) 공식 — sqrt는 45cm 오버슈트.

## 3종 파일 인터페이스 (컨트롤러 ↔ path_time 규약)

폴더 규약: `control_seoungjin/input/`(상위→여기), `control_seoungjin/output/`(여기→하위). output/은 생성물이라 .gitignore 검토.

| 파일 | 방향 | 갱신 | 용도 |
|---|---|---|---|
| 경로 JSON | 상위 → `input/` | 임무 단위 | waypoints (mission) |
| `output/attitude_feedback.json` | 컨트롤러 → | 비행 후, **used 태그** | 롤·피치 학습 → 경로 보정 |
| `output/current_state.json` | 컨트롤러 → | **상시 20~50Hz** | 실시간 상태 (재계획 이어붙이기) |

### attitude_feedback.json (반복 학습형 경로 보정)
- 쓰기: `used:false`로 기록. 읽기: `used:false`만 소비 → 보정 반영 → **`used:true`로 재기록** (이중 보정 방지 핸드셰이크).
- 스키마(안): `{flight_id, used, trajectory_hash, mode_freq_hz, tail:{pitch_rms_deg,roll_rms_deg,amp_deg,phase_rad,t_ref_s}, moving:{att_peak_deg,track_rms_cm}, k_est:{kthrust,kdrag,confidence}}`
- 1차 추론(단순한 것부터): ① tail RMS>임계 → 해당 구간 Tm 연장(온건화, 20배 저감 실증) ② mode_freq로 traj_zv f0 갱신(잔여 1.5° 주범이 주파수 오차) ③ 수렴 시 무수정.

### current_state.json (실시간 상태 보고)
- **상시 덮어쓰기**(비행 내내), **원자적 쓰기 필수**(임시파일→rename, 반쯤 써진 JSON 읽기 방지).
- 스키마(안): `{timestamp, pos[3], vel[3], acc[3], yaw_rad, ref_state:{pos,vel,acc}}`
- 재계획 이어붙임 원칙: **평시엔 `ref_state`(성형 기준 상태)에서** 이어붙일 것 — 측정 상태로 하면 피드백 성형 함정(원칙 1 위반). 측정 상태는 비상 이탈(충돌회피 후) 재계획에만, 스플라이스 구간 온건(Tm≥0.9s)하게.
- **신선도 검사**: timestamp 나이 임계(예 0.5s) 초과 시 error() — 낡은 상태 이어붙이기 = 점프 = 미분킥 자초.

## 확정된 설계 상수 / 원칙 (실측 근거)

- **운용 envelope: v≈2.5 m/s, a≈2.5 m/s²** (성형 한계는 여유율 2.0/2.0/j10)
- **온건 이동(Tm≥0.9s)은 1.8Hz 짐 모드를 20배 덜 때림** — 예방책. 빠른 이동일수록 ZV 의존.
- **저크-가능 조건 필수**: 이동 최소시간 Tm ≥ (60·A / (0.8·jmax))^⅓. 위반하면 게이트 차단(또는 스무더 뱅뱅). path_time이 애초에 이 조건 지켜 생성해야 함.
- 짐 = 중심 용접 강체(저중심 모드). 진자 아님. 크기 0.1~0.2m/질량 1~2kg 강건 확인됨.
- 스윙(백래시) 대응 큰 그림(사용자 확정): **측정→path_time에서 경로 꼬기** 피드포워드. traj_zv가 1호기. 잔여는 추후 별도 PID.

## 권장 착수 순서

1. `path_time.py`에 traj_smoother/traj_zv/traj_gate에 대응하는 Python 함수 확보 (MATLAB 알고리즘 포팅 or 브리지). 각각 단위테스트: 정상 궤적 무개입, 살인 궤적 성형 후 게이트 통과.
2. `input/` 경로 JSON 스키마 확정 + 로더 → path_time → 체인 → `output/` 산출.
3. `controller/.../run_traj_baked.m`(미실행)와 접합: 산출 trajectory.mat을 구운 모델에 주입해 왕복 검증. **이 접합의 첫 실행이 run_traj_baked 자체의 첫 검증도 겸함** — Scope 버스 매핑 로그로 act/des 로그 형식 확정.
4. attitude_feedback.json 쓰기(컨트롤러측 후처리)와 읽기(path_time) 최소 루프 → used 태그 왕복 확인.
5. current_state.json + 재계획 이어붙임 (여유되면).

## 절대 규칙 (공통)

- 구운 `.slx`는 **`save_system` 금지**(실험은 메모리 수술만). 게인은 `Scripts_Data/quadcopter_package_parameters.m`.
- **`sample/`은 사용자 개인 테스트 케이스 폴더** — 신규 코드 넣지 말 것. 신규 산출물은 `control_seoungjin/` 바로 밑.
- 대상 못 찾으면 조용히 통과 말고 **error()로 즉사** (이 저장소 규칙).
- MATLAB 실행: `"/c/Program Files/MATLAB/R2026a/bin/matlab.exe" -batch "스크립트"` (R2026a가 실사용). Git Bash 경로는 `cygpath -w`.
- 커밋은 하되 **push는 사용자에게 명령어 제공**(Claude push 권한 제한). 서브모듈 브랜치 `fix/plate-orientation-cg`.
