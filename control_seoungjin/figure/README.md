# 컨트롤러 성능 그림 모음 (figure/)

작성: 성능 지표 세션, 2026-08-18. 생성기: [`../perf_metrics.py`](../perf_metrics.py) (+ [`../perf_battery_plots.py`](../perf_battery_plots.py), [`../perf_0kg_compare.py`](../perf_0kg_compare.py)),
MATLAB 배터리: [`../controller/Quadcopter-Drone-Model-Simscape/diagnose/perf_battery.m`](../controller/Quadcopter-Drone-Model-Simscape/diagnose/perf_battery.m), 미션 실비행: [`../verify_pipeline.py`](../verify_pipeline.py).
지표 정의와 합격선은 [PERFORMANCE_SPEC.md](../controller/Quadcopter-Drone-Model-Simscape/PERFORMANCE_SPEC.md)(1 kg 설계점) / [PERFORMANCE_SPEC_0KG.md](../controller/Quadcopter-Drone-Model-Simscape/PERFORMANCE_SPEC_0KG.md)(무하중 부록)에서 가져왔고 새로 발명한 지표는 없다. 숫자 원본은 각 폴더의 `summary_*.md` / 최상위 [summary.json](summary.json).

**폴더 구조 (경우별)** — 스크립트가 생성 후 자동 정리(`perf_metrics.organize`):

| 폴더 | 내용 | 원자료 |
|---|---|---|
| [00_pipeline/](00_pipeline/) | 파이프라인 다이어그램 (`fig_pipeline.png/.svg`, mermaid 소스 README) | `make_pipeline_figure.py` |
| [01_missions_1kg/](01_missions_1kg/) | 미션 실비행 7편, 짐 1 kg (설계점, 설득 본편) | `sim_result_<name>.mat` (07-19/08-01 로그) |
| [02_missions_0kg/](02_missions_0kg/) | 같은 7편, 짐 0 kg (08-18 신 스케줄로 실비행) | `sim_result_<name>_0kg.mat` (`UGRP_PKG_KG=0 python verify_pipeline.py --tag _0kg`) |
| [03_tuning_timeseries/](03_tuning_timeseries/) | 스텝/램프/지터 3부작 (07-19 튜닝 세션 CSV) | `diagnose/results/{step,ramp,jitctr}_ts_*.csv` |
| [04_battery_1kg/](04_battery_1kg/) | 배터리 6케이스 × 짐 1 kg — 접미 없음 = 배포 구성(ZVD), `_raw` = 셰이퍼 없음 | `perf_*_1kg.csv`, `perf_raw_*_1kg.csv` |
| [05_battery_0kg/](05_battery_0kg/) | 배터리 6케이스 × 짐 0 kg — 접미 없음 = **신 스케줄(ZVD)**, `_raw` = **구 앵커**(셰이퍼 없음) | `perf_*_0kg.csv`, `perf_raw_*_0kg.csv` |
| [06_mass_sweep/](06_mass_sweep/) | 질량 스윕 1 m 이동 0/0.25/0.5/0.75/1/1.5/2 kg (+ 구/신 대조) | `perf_move1m_*.csv` |
| [07_retune_0kg/](07_retune_0kg/) | 0 kg 재튜닝 근거: 축별 스캔, 맞교환, 전/후 비교, 후보 대조 배터리(`_tuned`, `_tuned0p5`) | `tune_0kg_r1~r4.csv`, `perf_tuned*_*.csv` |

재생성: `cd control_seoungjin && python perf_metrics.py && python perf_metrics.py --tag _0kg && python perf_0kg_compare.py` (MATLAB 불필요 — 로그·CSV만 읽음).
배터리 재실측: 모델 폴더에서 `PKG=0 SHAPER=zvd FF=sqrt matlab -batch "cd(fullfile(pwd,'diagnose')); perf_battery"` (PKG=1 로 짐 1 kg). 미션 0 kg 재실측: `UGRP_PKG_KG=0 python verify_pipeline.py --tag _0kg`.


**대표 지표(최악 조건) — `09_headline/`**: `fig_labmeeting_worst_case.png`(**랩 미팅 한 장**: 0 kg+바람 5 m/s+창문 코스+돌풍 복합 최악, x/y/z 오차·roll/pitch/yaw, `make_labmeeting_figure.py`), `fig_unlucky_case_worst_combo_0kg.png`(같은 케이스 상세), `fig_0kg_showcase_<case>.png`(0 kg 단일 최악 케이스: diag/torque_pulse/wind/worst_combo), `fig_worst_timeseries.png`(지표별 최악 조건 6행 오차 시계열), `fig_headline_ratio.png`(최악값 ÷ 조건별 적용 스펙 한 장), `fig_headline_worst.png`(지표별 패널), `headline_worst.md`. 사용자 결정 08-18: 발표 숫자는 설계점이 아니라 최악 조건값; 지연 내성은 제외(forward). 재생성 `python make_headline_figure.py` 등.

## 0. 한 장 요약 — 무엇을 보여주나

| 주장 | 근거 | 핵심 숫자 |
|---|---|---|
| 실제 미션 궤적(정지·플라이스루·yaw 4모드·34 s 장거리)을 **cm급**으로 추종한다 (1 kg) | §1 / 01 | 추종 RMS **1.5~3.4 cm** (스펙 10), 종점 오차 **≤0.25 cm**, 도착 후 잔류 자세 **≤0.085°** |
| 위치 스텝은 **오버슈트 ≤6 %, 정착 2.5 s** — 임계감쇠에 가깝다 | §2 / 03 | 1 m: rise 0.87 s / 오버 5.9 % / SSE 0.2 mm (precision) |
| 등속 추종 지연은 **프로파일로 선택 가능** (정밀 vs 민첩) | §2 / 03 | 2 m/s에서 이동 중 RMS 4.7 cm(precision) vs 1.5 cm(agile), 정지 후 잔류 0.1 cm |
| 도착 후 호버는 **지터 0.000°급** | §2 / 03, §3 / 04 | precision tail RMS 2e-5°; 12 s 호버 지터 RMS 0.001° |
| 외란·고도스텝·바람·대각 배터리 (1 kg, 배포 구성 ZVD) | §3 / 04 | 외란 펄스 이탈 2.3°/회복 0.7 s, 고도 스텝 오버슈트 0.8 cm, 바람 5 m/s 유지 1.5 cm, 대각 1.6 m/s 추종 6.1 cm |
| **0.25~2 kg 전 구간 동질** — 게인 질량 스케줄(0 kg 앵커 ↔ 1 kg 앵커 선형, 1 kg 이상 기존식) | §3 / 06 | 1 m 이동 추종 5.2/4.3/3.7/3.3/3.3/3.3 cm (0.25→2 kg), 오버슈트 9.2→5.0 cm |
| **0 kg(무하중)도 이제 난다** — 08-18 재튜닝으로 한계사이클·발산 제거 | §4 / 07, 05, 02 | 호버 지터 8°→**0.16°**, 새그 14.6→**2.5 cm**, 대각 이동 발산→**12.9 cm**, 바람 5 m/s 발산→**19.9 cm** |
| 약점을 숨기지 않는다 | §1 yaw, §3.2, §4 | yaw 루프 느림(rise 2.1 s); 0 kg 외란 펄스 이탈 19.8°·재진입 못 함(호버 정밀과 외란 강건 동시 만족 게인 없음, 실기는 배터리 저장착으로 0 kg 상태 제거 권고) |
| 이 자료 뒤에는 **검증 스크립트 220개 / 결과표 38개 / 튜닝 기록 711줄(TUNING_STATUS §A~§Z)** 이 있다 | — | 08-01 RL seam 왕복 실비행까지 전 구간 통과 |

전 그림 공통: 파선 = 목표(계획 궤적), 실선 = 실측, 회색 띠 또는 빨간 파선 = 스펙 한계, 점선 세로선 = 궤적 종료(이후 = hold/tail 구간).

---

## 1. 미션 실비행 7편 — 짐 1 kg ([01_missions_1kg/](01_missions_1kg/))

원자료: 모델 폴더 `sim_result_*.mat` — `verify_pipeline.py` 검증 매트릭스와 08-01 RL seam 실비행이 남긴 로그.
구운 모델(precision 프로파일, 짐 1 kg) 무수정. 궤적은 전부 파이프라인(plan → 스무더 → ZVD → 게이트)을 통과한 것.

| 미션 | 무엇을 시험하나 | 추종 RMS 3D | 최대 이탈 | 종점 오차 | 오버슈트 | z 이탈 | 자세 RMS p/r | 자세 피크 | tail 잔류 | 판정 |
|---|---|---|---|---|---|---|---|---|---|---|
| step | 1 m 정지→정지 (스텝 백스톱) | 2.88 cm | 5.8 | 0.25 cm | 3.0 cm | 0.3 cm | 2.51/0.01° | 4.9° | 0.056° | ✅ |
| jitter_a | 공격적 왕복 (짐 모드 가진 A) | 3.41 | 5.8 | 0.23 | 4.0 | 0.3 | 3.52/0.01 | 6.2 | 0.085 | ✅ |
| jitter_b | 공격적 왕복 B | 3.16 | 5.8 | 0.21 | 3.3 | 0.3 | 3.10/0.01 | 5.8 | 0.053 | ✅ |
| fly_through | 중간점 무정지 통과 + heading yaw | 2.72 | 5.8 | 0.10 | 3.1 | 1.1 | 2.36/1.46 | 5.6 | 0.058 | ✅ |
| look_at | fly_through + 목표 주시 yaw | 2.95 | 5.0 | 0.12 | 2.2 | 0.4 | 2.94/1.34 | 5.6 | 0.048 | ✅ |
| scan | 제자리 ±180° 스캔 (S-사다리꼴 yaw) | 1.53 | 5.0 | 0.08 | 0.2 | 0.2 | 0.37/0.44 | 1.0 | 0.003 | ✅ |
| stop_batch | 34.9 s 다중 웨이포인트 정지 배치 | 2.07 | 5.2 | 0.03 | 0.1 | 1.1 | 2.47/0.40 | 6.8 | 0.002 | ✅ |

판정 = 추종 RMS ≤10 cm / 오버슈트 ≤10 cm / z 이탈 ≤10 cm / 도착 후 드리프트 ≤5 cm 네 항목 전부.
"최대 이탈 5.8 cm"는 전 미션 공통으로 **이륙 직후 새그(Z3, t<2 s)** 이고 기동 중 z 이탈은 0.2~1.1 cm.

### fig_mission_<name>.png — 미션별 4단 그림
위에서부터 ① 위치 x/y/z 목표(파선) vs 실측(실선) — 파선이 안 보이면 실선이 덮은 것 ② 축별 추종 오차 [cm], 회색 띠 = ±10 cm 스펙
③ pitch/roll [deg] — 비행 중 RMS와 tail 잔류를 제목에 병기 ④ 모터 4개 |ω| — 포화(~825 rad/s 대역 상한) 없이 평평.
**읽는 법**: 오차가 스펙 띠 폭의 1/3 이내에서 놀고, 궤적 종료선 뒤로는 자세가 0.0x° 수준으로 눕는다 = 셰이퍼+PID 조합이 짐 모드를 남기지 않는다.

- `fig_mission_stop_batch.png` — 가장 긴 미션(34.9 s, 6 m 상승 포함). 종점 오차 0.03 cm, tail 잔류 0.002°. 대표 그림으로 쓸 것.
- `fig_mission_scan.png` — 위치는 제자리, yaw만 ±180° 스캔. 위치 추종 1.5 cm = yaw 기동이 위치를 흔들지 않는다는 증거.
- `fig_mission_fly_through.png` / `look_at.png` — 08-01 RL seam 실비행(윤호 씬 → 성진 궤적 → MATLAB) 로그. roll이 붙는 이유는 heading/look_at yaw로 기체가 돌면서 x·y가 섞이기 때문.
- `fig_mission_jitter_a/b.png` — 짐 모드(1.8 Hz)를 일부러 가진하는 공격 왕복. 자세 피크 6°까지 갔다가 도착 후 0.05~0.09°로 수렴.

### fig_path_<name>.png — xy 평면 경로
계획(파선) 위에 실측(실선)이 겹친다. 출발(●)·목표 종점(■). 코너에서도 경로 이탈이 수 cm.

### fig_summary_tracking.png — 미션 횡단 막대
좌: 축별 추종 RMS vs 스펙 10 cm(빨간 파선) — 전부 1/3 이하. 중: 비행 중 자세 RMS 막대 + tail 잔류(●, 거의 0).
우: 종점 오차 / 기동 중 z 이탈 — 전부 ≤1.1 cm.

### fig_yaw_missions.png — yaw 루프 (알려진 한계, 정직하게)
yaw 4모드 미션의 목표 vs 실측 yaw와 오차. yaw 루프는 **설계상 느리다** (PERFORMANCE_SPEC §2: rise ~1.5 s — 반토크 권한이 약해 의도적으로 완만) 라 1.0 rad/s 스캔이나 웨이포인트 정지점의 heading 점프는 1~2 s 지연 + 오버슈트가 난다: scan 오차 RMS 68°(종점 +51° 오버슈트), look_at 13.6°, fly_through 14.7°, stop_batch 15.2°(정지점 heading 점프 79° 순간 오차). 08-01 ★튜닝 세션에 yaw 대역폭/댐핑 재조정으로 이관돼 있고, **위치 추종에는 영향이 없다**(위 표: 같은 비행에서 추종 1.5~3 cm). §3 yaw 스텝 90°로 rise/오버슈트를 스펙 기준으로 정량화한다.

### 1b. 같은 7편 — 짐 0 kg, 신 스케줄 ([02_missions_0kg/](02_missions_0kg/))

원자료: `sim_result_<name>_0kg.mat` — 이 세션(08-18 18:4x~18:5x) `UGRP_PKG_KG=0 python verify_pipeline.py --tag _0kg` 로 같은 7편을 신 스케줄(0 kg 앵커)로 실비행. 판정 기준은 1 kg과 동일(추종 ≤10 / 오버슈트 ≤10 / z 이탈 ≤10 / 드리프트 ≤5 cm).

| 미션 | 추종 RMS 3D | 최대 | 종점 | 오버슈트 | 새그 | z 이탈 | 드리프트 | 자세 RMS p/r | 피크 | tail 잔류 | yaw RMS | 판정 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| step | 3.13 cm | 6.0 | 0.12 cm | 5.0 cm | 2.5 | 0.0 cm | 1.9 cm | 2.56/0.03° | 5.4° | 0.18° | 0.0° | ✅ |
| jitter_a | 4.55 | 8.6 | 0.15 | 6.9 | 2.5 | 0.1 | 3.5 | 3.54/0.03 | 6.7 | 0.21 | 0.0 | ✅ |
| jitter_b | 4.17 | 8.1 | 0.14 | 5.2 | 2.5 | 0.1 | 1.5 | 3.24/0.03 | 6.3 | 0.17 | 0.0 | ✅ |
| fly_through | 3.62 | 7.2 | 0.19 | 4.9 | 2.5 | 0.9 | 1.5 | 2.39/1.63 | 5.9 | 0.17 | 14.6 | ✅ |
| look_at | 4.43 | 8.1 | 0.11 | 4.6 | 2.5 | 0.2 | 1.3 | 3.14/1.23 | 6.0 | 0.15 | 13.3 | ✅ |
| scan | 1.12 | 2.5 | 0.06 | 0.3 | 2.5 | 0.1 | 0.2 | 0.38/0.50 | 1.2 | 0.22 | 80.4 | ✅ |
| stop_batch | 3.06 | 9.5 | 0.03 | 0.0 | 0.4 | 0.7 | 0.2 | 2.31/0.52 | 7.3 | 0.12 | 10.5 | ✅ |

**7편 전부 통과** (1 kg 대비 추종 +0.3~1.5 cm, 오버슈트 +1~3 cm, tail 잔류 0.12~0.22° = 0 kg 호버 지터 0.16° 수준). 이륙 새그 5.8 → 2.5 cm(FF √질량). scan yaw 오차 RMS 80°(1 kg 68°) — yaw 게인은 질량 동결이라 관성 절반에서 오히려 오버슈트가 커진 것, 위치엔 무해(1.1 cm). 그림: `fig_mission_<name>_0kg.png`, `fig_path_*_0kg.png`, `fig_summary_tracking_0kg.png`, `fig_yaw_missions_0kg.png`; 표 원본 `summary_missions_0kg.md`.

---

## 2. 튜닝 세션 시계열 3부작 ([03_tuning_timeseries/](03_tuning_timeseries/))

원자료: `diagnose/results/{step,ramp,jitctr}_ts_*.csv` (07-19 튜닝/C++ 세션, 882030e·8add65e). 파이프라인 성형기(스무더+게이트)를 통과한 준-스텝/램프를 precision·agile 두 프로파일로.

### fig_step_response.png — 위치 스텝 응답
| 케이스 | 프로파일 | rise 10-90 % | 오버슈트 | ±2 % 정착 | SSE |
|---|---|---|---|---|---|
| **1 m** | precision | 0.87 s | 5.9 % | 2.47 s | −0.2 mm |
| **1 m** | agile | 0.94 s | 1.3 % | 1.76 s | −0.1 mm |
| 0.1 m (참고) | precision | 0.54 s | (49 %)※ | — | 0.0 mm |
| 0.1 m (참고) | agile | 0.49 s | (47 %)※ | — | −0.6 mm |

좌(1 m): 두 프로파일 모두 오버슈트 한 자릿수 %, SSE 0.x mm. 아래 pitch는 기동 중 ±10°까지 쓰고 정착 후 0°.
※ 우(0.1 m)는 **기준 궤적 자체가 출렁이는** 케이스 — 스무더에 0.1 s 램프를 직접 넣으면 꼬리 저주파 진동이 생기는 성형기 오용 경로(SESSIONS_BOARD 07-19 ★path_time)이고, 컨트롤러는 그 출렁이는 기준을 mm급으로 따라간다(우하 pitch가 기준의 진동과 동위상). 컨트롤러 성능이 아니라 성형기 입력 규약 이슈라 참고로만.

### fig_ramp_lag.png — 등속 램프 추종 지연 (0.5 / 1.5 / 2 m/s)
| 속도 | precision 이동 중 RMS / 피크 | agile 이동 중 RMS / 피크 | 정지 후 잔류 |
|---|---|---|---|
| 0.5 m/s | 6.2 / 10.8 cm | 1.9 / 3.4 cm | ≤0.1 cm |
| 1.5 m/s | 4.8 / 8.7 | 1.6 / 2.8 | ≤0.2 |
| 2.0 m/s | 4.7 / 8.7 | 1.5 / 2.8 | ≤0.13 |

지연 = 목표−실측. 피크는 가속/감속 순간(등속 구간에서는 그 절반 이하), 정지 후 mm로 수렴. agile은 지연 1/3, precision은 호버 지터 0. 상위 계층이 임무에 따라 고른다(INTERFACE_SPEC §1 `controller_profile`).

### fig_hover_jitter.png — 도착 후 잔류 지터
0.1 m 이동 후 4 s: precision pitch RMS **2×10⁻⁵°**(호버급 복귀), agile 0.17° / 피크 0.27° (R4 스펙 0.25° 이내). 두 프로파일의 구조적 맞교환(호버 지터 vs 추종 지연)을 한 그림에서 보여준다.

---

## 3. 성능 배터리 — 이 세션 실측 ([04_battery_1kg/](04_battery_1kg/), [05_battery_0kg/](05_battery_0kg/), [06_mass_sweep/](06_mass_sweep/))

`perf_battery.m`으로 구운 모델을 무수정으로 시나리오별 실비행 (호버 12 s / 외란 토크 펄스 0.3 N·m×0.3 s (precision, agile) / 고도 스텝 1 m / yaw 스텝 90° / 대각 2 m×2 m 1.8 s / 7 m 호버 바람 0·5 m/s / 질량 스윕 1 m 이동 7점). 두 구성:
- 접미 없음 = **배포 구성**: 스무더 → ZVD 1.8 Hz 셰이퍼 → FF 호버 트림 √질량 → 08-18 질량 스케줄 게인 (파이프라인이 항상 적용하는 실사용 성능).
- `_raw` = 셰이퍼 없음(스무더만), 구 게인 — 튜닝 세션 refine 하네스와 동일 구성. **기록 재현 검증**: 1 m 이동 추종 1 kg 4.08 / 0.5 kg 4.21 / 2 kg 3.95 cm, 꼬리 4.42° = refine_linear_law.csv와 소수점까지 일치 → 하네스 신뢰 가능.

케이스당 소요: 첫 편 ~100 s(컴파일), 이후 7~60 s (이 머신, R2026a). 배터리 15케이스 = 281 s.

### 3.1 짐 1 kg (설계점) — 배포 구성, 전부 스펙 안 (`04_battery_1kg/`)

| 케이스 | 그림 | 결과 (ZVD 배포 구성) | 스펙 |
|---|---|---|---|
| 12 s 호버 | `fig_bat_hover_1kg.png` | 자세 지터 RMS **0.001°** / 피크 0.007°, 고도 1.001~1.002 m, 이륙 새그 5.0 cm, 드리프트 0.20 cm, yaw 배회 0.02° | R4 ≤0.25 ✅ R5 ≤0.8 ✅ Z2 ✅ Z3 ≤5 (경계) 드리프트 ≤5 ✅ |
| 외란 토크 펄스 0.3 N·m×0.3 s | `fig_bat_torque_pulse_1kg.png` | precision 최대 이탈 **2.33°** / 회복 0.74 s, agile **1.58° / 0.44 s**; 고도 이탈 ≤1 cm; 수평 밀림 4.3 / 1.6 cm; 모터 차동 177/341 rad/s | R6 ≤5° ✅ R7 ≤1.5 s ✅ R10 ✅ |
| 고도 스텝 1 m | `fig_bat_alt_step_1kg.png` | rise 0.96 s, 오버슈트 **0.8 cm**, 수평 이탈 0.55 cm, 자세 피크 0.01° | Z1 ≤5 cm ✅ |
| yaw 스텝 90° | `fig_bat_yaw_step_1kg.png` | rise 2.1 s, 오버슈트 15.8°, 최대 오차 62°; 위치 커플링 수평 0.54 cm / z 0.17 cm | §2 rise ~1.5 s ❌ (약점, §1 yaw 그림 참조) — 위치엔 무해 |
| 대각 이동 2 m×2 m (1.8 s) | `fig_bat_diag_move_1kg.png` | 추종 RMS **6.1 cm** / 최대 11.9 cm, 종점 0.16 cm, z 이탈 0.23 cm, roll/pitch 대칭비 0.97 | 추종 ≤10 ✅ (벡터 속도 1.6 m/s 공격 기동) |
| 7 m 호버 바람 0 → 5 m/s | `fig_bat_wind_1kg.png` | 수평 유지 0.3 → **1.5 cm**, 고도 0.2 cm, 자세 트림 0.48° / 지터 0.57° | 지속 외란 하 위치 유지 ✅ (I항이 트림 소거) |

`_raw`(셰이퍼 없음)와의 차이: 호버·외란·고도·바람은 사실상 동일(정지 상태라 셰이퍼 무관), 이동 케이스만 ZVD가 짐 모드 잔류를 없앤다 — 질량 스윕 1 kg 꼬리 잔류 4.4°(raw) → **0.64°**(ZVD), 오버슈트 7.7 → 4.9 cm, 추종 4.1 → 3.3 cm.

### 3.2 짐 0 kg — 신 스케줄(배포 구성) vs 구 앵커 (`05_battery_0kg/`)

접미 없음 = **08-18 재튜닝 스케줄**(sA 0.40 / kd:kp 0.6 / limit_att 100 / kp_pos 5 / filtPz 0.005 / FF √질량), `_raw` = 구 앵커(sA 0.75, 선형 FF). 스펙은 [PERFORMANCE_SPEC_0KG.md](../controller/Quadcopter-Drone-Model-Simscape/PERFORMANCE_SPEC_0KG.md).

| 케이스 | 그림 | 구 앵커 (`_raw`) | **신 스케줄** | 0 kg 스펙 |
|---|---|---|---|---|
| 12 s 호버 | `fig_bat_hover_0kg.png` | roll/pitch ±8° 5 Hz 한계사이클 (RMS 5.4°), 새그 14.6 cm, 드리프트 12.7 cm | 지터 RMS **0.156°** / 피크 0.29°, 새그 **2.5 cm**, 드리프트 0.05 cm | H1 ≤0.25 ✅ H2 ✅ H3 ≤5 ✅ H4 ✅ |
| 외란 펄스 0.3 N·m | `fig_bat_torque_pulse_0kg.png` | 이탈 13.4°, 재진입 없음 (한계사이클 위) | 이탈 **19.8°**, ±1° 재진입 없음, 수평 밀림 7 m | R1 ≤25°·전복 금지 ✅ / **R2·R3 ❌** (§4 참조) |
| 고도 스텝 1 m | `fig_bat_alt_step_0kg.png` | rise 1.6 s, 오버슈트 22.9 cm, 수평 이탈 11 cm | rise 0.95 s, 오버슈트 **0.9 cm**, 수평 0.1 cm | Z1 ≤5 ✅ |
| yaw 스텝 90° | `fig_bat_yaw_step_0kg.png` | rise 3.6 s, 최대 오차 57° | rise 2.4 s, 오버슈트 14.5°, 위치 커플링 0.1 cm | Y1 ✅ |
| 대각 이동 2 m×2 m | `fig_bat_diag_move_0kg.png` | **발산** (x −38 m) | 추종 RMS **12.9 cm** / 최대 26.6, 종점 0.4 cm, 자세 피크 19.3° | P3 ≤15·발산 금지 ✅ |
| 7 m 호버 바람 0/5 m/s | `fig_bat_wind_0kg.png` | 상승 중 **발산** (20~36 m 이탈) | 바람 0: 0.03 cm / 바람 5: **19.9 cm**, 자세 트림 1.0° | W1 ≤25 ✅ |

### 3.3 질량 스윕 1 m 이동 (`06_mass_sweep/`)

`fig_bat_mass_sweep.png` (신 스케줄, ZVD, precision) 7점: 0~1 kg은 두 앵커의 선형 보간, 1 kg 이상은 기존식(sA 1.0→1.25, sZ 0.56+0.44 m).

| m_pkg [kg] | 0 | 0.25 | 0.5 | 0.75 | 1.0 | 1.5 | 2.0 |
|---|---|---|---|---|---|---|---|
| 추종 RMS [cm] | 6.9 | 5.2 | 4.3 | 3.7 | 3.3 | 3.3 | 3.3 |
| 오버슈트 [cm] | 13.1 | 9.2 | 7.1 | 5.8 | 4.9 | 5.0 | 5.0 |
| 이동 중 자세 피크 [°] | 13.3 | 12.8 | 12.4 | 12.1 | 11.8 | 12.0 | 11.8 |
| 꼬리 잔류 [°] / 드리프트 [cm] | 0.58 / 0.6 | 0.63 / 0.7 | 0.68 / 0.7 | 0.65 / 0.6 | 0.64 / 0.6 | 0.65 / 0.8 | 0.66 / 0.8 |

읽는 법: 보간 구간(0.25/0.5/0.75)이 양 끝 사이에서 **단조롭게** 이어진다 = 선형 스케일에 구멍이 없다 (스펙 M1 ✅; 0.25 kg 이상은 1 kg 스펙 P1/P2 ≤10 cm 통과). 0 kg 행 오버슈트 13 cm는 0 kg 스펙 P2(≤15)로 판정.
`fig_0kg_mass_sweep_before_after.png` — 구 앵커(raw, 0 kg 24 cm/36 cm) 대 신 스케줄 대조. (구 앵커 곡선은 셰이퍼 없음 구성이라 1 kg 이상에서 신 곡선과 차이가 있는 것은 셰이퍼 효과.)

---

## 4. 0 kg 재튜닝 ([07_retune_0kg/](07_retune_0kg/)) — 진단 → 스캔 → 맞교환 → 채택 → 검증

**진단** (구 앵커 `05_battery_0kg/*_raw.png`): 짐을 떼면 CG가 추력면 아래 8.1 cm → 위 0.8 cm로 올라와 진자 복원 모멘트가 사라지고 자세 관성이 절반(1.71→0.93e-2)이 되는데 게인은 25 %만 감쇠돼 실효 루프 이득이 1.37배 → **5 Hz 세차 한계사이클**(roll·pitch 동진폭·−92° 위상): 호버 400 rad/s 대비 자세 변조 423 rad/s라 저속 모터가 0에 포화되며 자기유지. FF 호버 트림도 선형 배분(56.5 rev/s)이 √질량 정답(75.5)보다 25 % 부족해 새그 14.7 cm.

**튜닝 축 스캔** — `fig_tune0kg_r2_axes.png` (좌표하강 27축, `tune_0kg_r2.m`, 호버 6 s + 1 m 이동 10 s 단축 시뮬): 유효 축 = sA↓, kd/kp↓, limit_att↓(≤100), 위치 측정필터, Bias Chassis↑(FF), filtPz↓. 무효/악화 = sZ, filtD, posErrSat, tilt 한계, ki, yaw 게인, kp/ki/kd_alt; 자세 측정필터(0.05) 단축은 전부 발산. 모터 내부 루프는 사용자 지시로 제외.

**맞교환** — `fig_tune0kg_r3_tradeoff.png` (결합 격자 r3/r3b): sA를 내릴수록 호버 지터·이동 오버슈트는 좋아지고 외란 이탈은 커진다. 0.35~0.42가 호버(H1)·이동(P1/P2) 스펙 교집합; sA 0.5는 외란 15.5°/밀림 46 cm로 나아지지만 호버 지터 0.56°(H1 초과). `tune_0kg_r4_limit.csv`: limit_att 20~40은 외란 권한 부족(이탈 26~78°), ≥50 동일 → 100 채택(이동 중 자세 명령 <50).

**채택 앵커** (parameters.m `sA_0kg` 등, C++ `qc_scales` 동기): sA **0.40** / kd:kp **0.6** / limit_att **100** / kp_pos **5**(kd 2.0) / filtM_pos 0.005 / filtPz **0.005** / FF **√질량**. 0~1 kg 선형 보간, 1 kg 수치 불변(1 kg 회귀 §3.1 전부 재확인).

**검증** — `fig_0kg_before_after.png` (4케이스 × 자세/수평오차, 구 vs 신), `fig_bat_*_0kg_tuned.png`(sA 0.40 후보 배터리) / `_tuned0p5.png`(sA 0.5 대조), `summary_0kg_compare.md`:

| 케이스 | 구 앵커 → 신 스케줄 |
|---|---|
| 호버 자세 RMS / 수평오차 | 7.65° / 11 cm → **0.22° / 0.1 cm** |
| 대각 이동 자세 RMS / 수평오차 최대 | 16.8° / 38.8 m(발산) → **6.2° / 26.6 cm** |
| 바람 5 m/s 자세 RMS / 수평오차 | 20.3° / 35.8 m(발산) → **2.1° / 20.4 cm** |
| 외란 펄스 이탈 / 밀림 | 13.3° / 17 cm → 19.8° / **7 m** ❌ |

**정직한 한계**: 0 kg 자세 루프(sA 0.40)는 0.3 N·m 펄스(각가속 32 rad/s², 1 kg의 2배)에 재진입 못 하고 수평 7 m 밀린다 — 위치 루프가 posErrSat(0.24 m)로 잘린 기울기 명령을 반복하는 톱니 형태(전복은 안 함). 호버 정밀(H1)과 외란 강건(R2/R3)을 동시에 만족하는 게인은 r3b/r4 격자 안에 없다. 결론: (a) 0 kg 운용은 임무 밖(투하 없음)이므로 생존 스펙으로 관리, (b) 실기는 **배터리 저장착으로 0 kg 상태 자체를 제거**(0.5 kg이면 1 kg급 스펙 전부 통과 — §3.3), (c) 그래도 0 kg 외란 강건이 필요하면 임무별 sA 선택(호버 정밀 vs 외란)이 남은 카드.
