# 성능 지표 세션 상태 (PERF_STATUS) — 2026-08-18 착수, 지니 노트북(i7-13650HX/16 GB/RTX 5060, MATLAB R2026a)

목적(사용자): 컨트롤러 성능을 **그래프 + 설명**으로 설득 자료화 (`figure/`), 이어서 **0 kg 재튜닝**(사용자 지시) + 최대 적재 튜닝.
규칙: SESSIONS_BOARD 프로토콜(MATLAB 점유 선언, ★ 이관), save_system 금지, 1 스크립트 = 1 프로세스, 병렬 MATLAB 금지(실측: 1개 3.5 GB, 여유 2.3 GB).

## 산출물 지도

| 파일 | 역할 |
|---|---|
| `perf_metrics.py` | 기존 로그(`sim_result_*.mat`, `diagnose/results/{step,ramp,jitctr}_ts_*.csv`) → 지표·그림·`figure/summary_missions.md` |
| `perf_battery_plots.py` | 배터리 CSV → `fig_bat_*` (prefix `perf_`=ZVD 배포 구성 / `perf_raw_`=스무더만 / `perf_tuned_`=0 kg 신 튜닝 / `perf_tuned0p5_`) |
| `figure/README.md` + 폴더 `01_missions_1kg … 07_retune_0kg` | 그림별 설명(§0~§4 완성) — 경우별 폴더는 `perf_metrics.organize`(ORGANIZE_RULES)가 자동 정리 |
| `perf_0kg_compare.py` | 0 kg 구 앵커(perf_raw_) vs 신 스케줄(perf_) 전/후 그림 2장 + `summary_0kg_compare.md` (07 폴더) |
| `verify_pipeline.py --tag` + `parameters.m` `UGRP_PKG_KG` | 질량별 미션 실비행 (`sim_result_<name>_0kg.mat`, `output/verification_matrix_0kg.json`), `perf_metrics.py --tag _0kg` 로 02 폴더 |
| `PERFORMANCE_SPEC.md` v0.2 / `PERFORMANCE_SPEC_0KG.md` v0.2 | 실측 열 전면 갱신·미션/질량/바람 항목 신설 / 0 kg 앵커·채점표·미션 7편 |
| `controller/.../diagnose/perf_battery.m` | 배터리 13케이스. env: `PKG`(짐 kg) `SHAPER`(zvd/none) `FF`(sqrt/linear) `TUNE0KG`(1) `TUNE0KG_SA` `ONLY` |
| `diagnose/tune_0kg.m` (r1) `tune_0kg_r2.m`(좌표하강 27축) `tune_0kg_r3.m`/`_r3b.m`(결합 격자) `tune_0kg_r4.m`(limit_att 임계+외란) | 0 kg 재튜닝 라운드. 결과 `results/tune_0kg_r1.csv, r2.csv, r2b.csv, r3.csv, r3b.csv, r4_limit.csv`, BASE `results/tune_0kg_base_r4.json` |
| `Scripts_Data/qc_ff_trim_apply.m` + `parameters.m` (`bias_hover_rps`, `use_ff_sqrt`) + `run_traj_baked.m` 훅 | **FF 호버 트림 √질량 법칙** (1 kg 수치 불변 100.9) |
| `controller_cpp/include/qc_controller.hpp` (`ffSqrt/hoverRpsRef/mTotFfRef`) + `src/qc_controller.cpp` (`biasHover`) | C++ 동기 — **이 머신엔 컴파일러 없어 미빌드** ★튜닝/C++ 세션: msys64에서 build.ps1 + 골든 트레이스(1 kg 무변화 기대) |
| `results/chain_0818.ps1` / `chain_0818.log` | 자율 체인: 0 kg 튜닝 배터리 → sA 0.5 대조 → 1 kg ZVD → `python perf_metrics.py` |

복구 이력: OneDrive 손상 플레이스홀더 3건(루트 `CLAUDE.md` 재작성, `parameters.m`·`SESSIONS_BOARD.md` git HEAD 복원), `diagnose/slprj` 손상 → 캐시 `%LOCALAPPDATA%/ugrp_drone/slprj_perf`. **파일 정리(08-18 19:3x)**: 손상 플레이스홀더 삭제, `-지니` 충돌 사본 3건(CLAUDE/SESSIONS_BOARD/parameters, 전부 stale)·`slprj.stale` → `%LOCALAPPDATA%/ugrp_drone/archive_0818/`, 모델 루트 `run_*.txt` 25건 → `diagnose/results/logs/`(verify_pipeline도 이제 거기 기록).

## 확정 결과

### 1 kg (설계점) — 전부 스펙 안 (raw 구성; 기록 재현 소수점 일치)
호버 지터 0.001°/피크 0.007°, 새그 5.0 cm, 드리프트 0.2 cm · 외란 0.3 N·m: precision 이탈 2.33°/회복 0.74 s, agile 1.58°/0.44 s · 고도 스텝 1 m 오버 1.5 cm · 대각 2 m×2 m 추종 7.0 cm · 바람 5 m/s 유지 1.5 cm · 질량 0.5~2 kg 이동 4.0~4.2 cm. yaw 스텝 90° rise 2.1 s/오버 15.8°(약점, 위치 무해). 미션 로그 7편 추종 1.5~3.4 cm.

### 0 kg 현행 앵커(sA 0.75/sZ 0.56) — 스펙 밖
호버 **5 Hz 세차 한계사이클 ±8°**(모터 0~800 rad/s 왕복 = 저속 모터 0 포화), 새그 14.7 cm, 1 m 이동 24 cm/오버 35, 대각·7 m 상승 **발산**.

### 0 kg 재튜닝 (r1~r4)
- 무효/악화 축(r2 73점): sZ, filtD_att/alt, posErrSat(Z), tiltLimit, ki_att/pos, kp_alt/ki_alt/kd_alt, filtM_alt(=roll/pitch 필터에만 배선됨, z는 Filter pz 0.01 하드코딩), yaw 게인, 자세 측정필터 단축(0.005~0.03 전부 발산), 길게(0.07~0.1) 무효.
- 효과 축: **sA↓, kd/kp↓, limit_att↓(≤100), filtM_pos 0.05(호버만, 이동엔 발산), biasChassis↑(새그), filtPz↓(새그)**.
- **FF 트림 오류**: 56.5+44.4·m는 선형, 정답 ω∝√m → 0 kg 75.5 rev/s. bias 75 → 새그 4.4 cm. 반영 완료(위 표).
- r3/r3b 맞교환(`figure/fig_tune0kg_r3_tradeoff.png`): sA 0.25→0.50: 호버 0.001→1.1°, 이동 21→8.6 cm. 교집합 **sA 0.35~0.42**. kd/kp 0.4 이동 불안정, 0.8 호버 초과 → **0.6**.
- **채택 후보** `sA 0.40 / kd:kp 0.6 / limit_att 100 / kp_pos 5(kd 2.0, posErrSat 0.24) / filtM_pos 0.005 / filtPz 0.005 / FF √질량`: 호버 0.086°, 새그 2.5, 드리프트 0.04, 1 m 이동(한계 가속) 9.8 cm/오버 20.6, 자세 피크 16°.
- r4 limit_att 임계: 20/30/40 → 외란 이탈 78/63/26° 권한 부족; **≥50 → 18.6°·±1° 미회복(limit 무관)** = sA 0.40 루프의 외란 강건성 한계 (1 kg 2.3°, R6 스펙 5°). 호버·이동은 limit 20~300 전부 동일 → limit_att는 100 권장(이동 중 자세 명령 <50).

### 0 kg 튜닝 후보 배터리 실측 (perf_tuned_*.csv, ZVD+FF √질량, 17:53 완료)
| 항목 | 0 kg 현행 | 0 kg 튜닝(sA 0.40) | 1 kg raw | 스펙 |
|---|---|---|---|---|
| 호버 12 s 지터/새그/드리프트 | 5.4° / 14.7 / 12.7 cm | 0.156° / 2.5 / 0.05 | 0.001°/5.0/0.2 | 0.25°/5/5 |
| 고도 스텝 1 m 오버슈트 | 22.9 cm | 0.9 cm (rise 0.95 s) | 1.5 | ≤5 |
| 대각 2 m×2 m 추종/종점 | 발산 | 12.9 cm / 0.4 cm | 7.0 | ≤10 |
| 1 m 이동 추종/오버 (스윕 0 kg 행) | 24 / 35 | 6.9 / 13 | 3.3 / 4.9 (ZVD) | ≤10/≤10 |
| 7 m 호버 바람 5 m/s 유지 | 발산 | 19.9 cm (트림 1.0°) | 1.5 cm | — |
| 외란 펄스 0.3 N·m | 13° 미회복 | **19.8° 이탈·수평 7 m 밀림·미회복** (agile 23.8°) | 2.3°/0.74 s | R6 ≤5°, R7 ≤1.5 s |
| yaw 스텝 90° | rise 3.6 s | rise 2.4 s (yaw 미변경) | 2.1 s | ~1.5 s |
→ 호버·고도·기동은 스펙권, **외란 강건성이 대가**. sA 0.5 대조 배터리(`perf_tuned0p5_*.csv`) 결과와 비교해 최종 0 kg 앵커 결정 필요 (판단 지점).

### sA 0.5 대조 배터리 (perf_tuned0p5_*.csv, 18:03 완료) vs sA 0.40
| 항목 | sA 0.40 | sA 0.50 |
|---|---|---|
| 호버 지터 RMS/피크 | 0.156° / 0.29° | 0.56° / 0.85° (R4·R5 초과, 회귀 기준 0.48/1.05 근처) |
| 외란 0.3 N·m precision | 19.8°, 수평 7 m, 미회복 | **15.5°, 수평 46 cm, 회복 7.5 s** |
| 외란 agile | 23.8°, 6.7 m | 12.8°, 25 cm, 미회복 |
| 대각 2 m×2 m | 12.9 cm | 12.1 cm |
| 바람 5 m/s 유지 | 19.9 cm | 15.2 cm |
| 1 m 이동 (0 kg 행) | 6.9 / 오버 13 | 6.5 / 11.8 |
| 고도 스텝 오버 | 0.9 cm | 0.9 cm |
→ sA↑ 는 외란·바람을 조금 되찾지만(여전히 R6 밖) 호버 지터를 R4 밖으로 밀어냄. **두 스펙을 0 kg에서 동시에 만족하는 sA는 없음** — 0 kg 은 (a) 호버 정밀 우선 sA 0.40, (b) 외란 우선 sA 0.5 중 임무별 선택 또는 실기 배터리 저장착으로 0 kg 상태 자체를 없애는 것이 정답. 다음 세션에서 사용자에게 선택 제시.

### 1 kg 배포 구성(ZVD + FF √질량) 배터리 (perf_*.csv, 18:16 완료) — 체인 종료, MATLAB 슬롯 해제
질량 스윕 1 m 이동: 0.5/1/1.5/2 kg 추종 3.3~3.4 cm / 오버 4.9~5.0 / 꼬리 잔류 0.64~1.09° (raw 4.1/7.7/4.4° → **ZVD 효과 실증**). 0 kg 행(구 게인+FF √+ZVD) 13.1/23.8/꼬리 5.2°. 호버·외란·대각·바람은 raw와 동일(2.33°/0.74 s, 0.001°, 6.1 cm, 1.5 cm). 그림은 `figure/fig_bat_*_1kg.png`(접미 없음) / `_raw` / `_tuned` / `_tuned0p5` 4벌 생성됨(`python perf_metrics.py` 재실행으로 재생성).

### 질량 스케줄 배포 배터리 (perf_*.csv, PKG=0 15케이스 281 s, 18:5x) + 0 kg 미션 7편 (19:0x)
0 kg: 호버 0.156°/0.29°, 새그 2.5, 드리프트 0.05 cm; 펄스 19.8° 재진입 없음 밀림 7 m(❌ R2/R3); 고도 스텝 0.9 cm; yaw rise 2.4 s; 대각 12.9 cm; 바람 5 m/s 19.9 cm. 질량 스윕 0/0.25/0.5/0.75/1/1.5/2 kg 추종 6.9/5.2/4.3/3.7/3.3/3.3/3.3, 오버 13.1/9.2/7.1/5.8/4.9/5.0/5.0 cm. 미션 7편(0 kg) 1.1~4.6 cm 전부 ✅ (scan yaw RMS 80°). 1 kg 회귀 불변.

### 19:1x~ 추가 작업 (2 kg 스윕 병행)
- **사후 학습(ILC) 층** 신설(사용자 요청 "로그 보고 경로 살짝씩 조정"): `traj_learn.py` + `traj_pipeline.py learn` 동사 + `plan --correction` (INTERFACE_SPEC §8 표 추가). 위치 항(1 Hz 4차 영위상 LPF, L 0.6, ±10 cm, 이륙 1.5 s/끝 1 s smoothstep 테이퍼, hold 1.5 s 연장, 게이트 위반 시 배율 ½) + 자세 항(2호기 `counter_swing_offset` 배선: 꼬리 잔류 amp/phase → 역위상 오프셋, `output/swing_calib.json` 07-19 교정 소비). 07-19 step 로그로 iter1 보정 2.2 cm 생성·게이트 통과. 단위 테스트 `tests/test_traj_learn.py`(합성 지연 플랜트 수렴). **한계(사용자에 설명)**: 호버 중 자려 진동(0 kg 5 Hz 등)은 경로로 못 없앰 — 기동 유발 잔류만 대상.
- **0 kg 자려 지터 감축**(사용자 "떨림 줄이자"): `sA_0kg` 0.40→**0.35** (r3 격자: 호버 0.086→0.006°, 이동 +0.7 cm/오버 +1.7 cm) — parameters.m·perf_battery 미러(`sA_0kg` 승계)·C++ 동기. 배터리 재확인 chain c 큐.
- 자율 체인: `chain_0818b.ps1`(2 kg 스윕 종료 대기 → agile 질량 스윕 배터리 7점) → `chain_0818c.ps1`(0 kg sA 0.35 배터리 16케이스 → ILC step 반복 비행 2회: plan --correction → run_traj_baked → learn). 로그 `chain_0818b/c.log`, `tune_2kg_r1.log/csv`, `perf_battery_agile_sweep.log`, `perf_battery_0kg_sA035.log`, `logs/run_ilc_step_iter{1,2}.txt`, `sim_result_step_ilc{1,2}.mat`.
- 문서: `PERFORMANCE.md` §7 질량 한계 표(T/W 1.69@1 kg, 1.17@2 kg, ~1.0@2.5 kg → 운용 상한 2 kg) + §8b 상위 계층 능력 카드(EXTERNAL_INTERFACE 링크), agile 질량 스케일 명시(5+19m / 2+8.8m — 이미 구현돼 있던 식), perf_battery 케이스 8(agile 스윕) + `fig_bat_mass_sweep_agile.png`.


### 21:1x~22:0x 결과·진행 (두 번째 스냅샷 — 최신, 여기부터 읽을 것)
- **22:0x~22:3x**: 사용자 지시로 지연 배터리 chain g 중단(1 kg 전 축·0 kg A0/A10 까지만; `figure/08_delay/summary_delay.md`), 지연은 forward 과제. **chain h 완료**: 0 kg 비선형 자세 게인 배포 배터리 — 토크 펄스 34°→**10.1°**/밀림 10.2→**0.26 m**/재진입 5.3 s, 바람 5 m/s 유지 23.3→**11 cm**, 대각 13.3 cm, 호버 0.005°; 1 kg 회귀 3케이스 동일(항등 확인). **복합 최악 케이스** `worst_combo_0kg`(perf_battery 케이스 9: 0 kg+정상풍 5+창문 코스 지그재그 v1.2/a1.0+이동 중 돌풍 0.3 N·m @14 s) 실비행: 위치 RMS 8.4/최대 15.7/종점 5.2 cm, 돌풍 13.3°→0.4 s 복귀, 발산 없음. 그림: `09_headline/fig_labmeeting_worst_case.png`(랩 미팅 한 장), `fig_unlucky_case_worst_combo_0kg.png`, `fig_0kg_showcase_*.png`, `fig_worst_timeseries.png`, `fig_headline_ratio/worst.png`(0 kg 외란·바람 통과로 갱신). 미완 안내: `input/absurd_mission.json`(한계 밖 명령 전시, 미비행), 2 kg 앵커 조합 확인, C++ 미빌드.
- **지연 배터리 1 kg 자세 축 결과**: 10 ms OK / **20 ms 호버 1.9° 진동·외란 복귀 없음** / 40 ms 8° / 80 ms 발산 → 자세 루프 지연 여유 10~20 ms (kd −127.5·filtD 2500 고이득 미분). **사용자 결정: 지연은 대표 지표에서 제외, forward 과제** = 제어기 내부 평균 지연 추정 → 그 지연 포함 PID 재튜닝(자세 kd/filtD, 지연 여유 ≥2×). 실기 자세는 IMU 직결 경로 규약.
- **r5 (0 kg 비선형 자세 게인) 완료 → 채택**: 1차 실행 전부 NaN(Fcn 블록 min/max 미지원) → sat01=(|x|−|x−1|+1)/2 로 수정 재실행. 결과 gmax/e0: 1.0 → 이탈 34.2°/밀림 10.3 m/재진입 없음; **2.1/3° → 10.1° / 0.26 m / 5.3 s (채택)**; 1.6/3° 13.6°/0.35 m/3.3 s; 2.9/3° 10.0°/0.24 m/재진입 없음; 2.1/6° 12.1°/2.4 s; 2.1/1.5° 10.6°/재진입 없음. 호버 지터 0.005° 전 격자 불변, 1 m 이동 오버슈트 22→17 cm(하네스). 반영: `parameters.m` `nl_gmax_0kg 2.1`(→1 @1 kg)·`nl_e0=nl_e1=3°`, `run_traj_baked.m`/`perf_battery.m`(env `NLATT=0` 대조) 에 `qc_nl_att_apply` 배선, C++ `qc_nl_att_gain` + `QcScales.nlGmax`/`QcState.nlGmax`(1 kg 항등 → 골든 불변, 미빌드). 0 kg 부록 v0.3 R1~R3 갱신(배포 배터리 재실측 chain h 대기).
- **2 kg r2 완료** (`tune_2kg_r2.csv` 20점, base sA1.0/r_att1.2/filtM_pos0.05/sZ1.2/kd_alt0.08): 지터 0.003°, 추종 4.6, 오버 8.6, 새그 7.9(불변). 개선 축 **kp_pos 12**(추종 3.9/오버 6.5/자세피크 18°), **filtM_pos 0.02**(추종 4.3, 지터 동일 → 지연 20 ms, T2 스펙 안); kd_alt ≥0.3·filtPz 0.02 악화; sA 1.5~1.75/r_att 1.8~2.2 소폭. **앵커 후보**: sA 1.0~1.5 / r_att 1.2 / filtM_pos 0.02 / kp_pos 10~12 / sZ 1.2 / kd_alt 0.08 — 조합 1회 확인 후 parameters.m 1~2 kg 법칙(★미완).
- **지연·타이밍 스펙 편입 (SPEC v0.3 전면 재편)**: §0 [시간] 역산, §0.5 시간 예산, R12~R14/Y6~Y7/Z6~Z7/§3.5 등가 지연·필터/§4 T·계획 벽시계/M6/W3, §7 T1~T10 종합, PERFORMANCE.md §8c. 실측: plan 0.6~0.7 s, 비상 서브프로세스 0.5~0.75 s (**STATE_STALE 0.5 s 와 충돌 3회 중 2회** → 처방 인프로세스 or 비상 한정 1.0 s, 미결정), A-1 정지 0.65/0.90/1.27/1.52 s @0.5/1/1.6/2 m/s (0.12/0.39/0.92/1.41 m). 미측: 센서·통신 지연 내성(T3/R14) → **지연 주입 배터리** 신설: `qc_delay_apply.m`(Filter Pitch/Roll/Yaw/pz 입력 + Position Control Mux p→Subtract2 에 Transport Delay, base `dly_att_s`/`dly_pos_s`), `perf_battery.m` env `DLY_ATT_MS/DLY_POS_MS`(파일 `perf_dlyA<a>_P<p>_*.csv`), `perf_delay_metrics.py`(→ `figure/08_delay/`), `chain_0818g.ps1`(자세 0/10/20/40/80 · 위치 0/50/100/200 · 조합 40+100, 1 kg→0 kg, 호버/토크/1 m, ~54회) **실행 중**. 주의: A0_P0 행은 1 µs 지연 블록 삽입 상태라 무블록 기준(perf_hover_1kg 0.001°)과 다름(0.018°) — 블록 자체 영향, 해석 시 참고.
- **대표 지표 = 최악 조건 성능 (사용자 결정)**: SPEC §0.6 + PERFORMANCE §8d 표, `make_headline_figure.py` → `figure/09_headline/fig_headline_ratio.png`(최악값 ÷ 조건별 적용 스펙 한 장), `fig_headline_worst.png`(10패널), `headline_worst.md`. 현재 미달 대표값: 이동 직후 잔류 0.68°(R11), 2 kg 새그 7.9 cm, 0 kg 외란(현행 선형; r5 채택 시 통과). 0 kg 대각 13.6 cm(부록 P3 15) — PERFORMANCE §4.2 표의 6.96 은 agile 값이었음(정정).
- **체인**: g(지연 배터리) → h(`chain_0818h.ps1`: 0 kg 전 케이스 NL 게인 배포 배터리 + 1 kg 회귀 3케이스) 자동. 완료 파일 `chain_0818g.done` / `chain_0818h.done`. 그 후 할 일: `python perf_delay_metrics.py` → T3/R14 실측치 스펙 기입 + 실기 예상 지연(IMU 1~3 ms, 모터 20~40 ms, VIO 30~100 ms) 대비 여유 판정(여유 < 예상×2 면 지연 넣고 재튜닝); `python perf_metrics.py --tag _0kg`/`make_headline_figure.py` 재생성 → 0 kg 부록 R1~R3·§0.6 교체; 2 kg 앵커 조합 확인.
- **OneDrive**: `SESSIONS_BOARD.md` 또 손상(읽기 불가) — 복원 `git checkout -- control_seoungjin/SESSIONS_BOARD.md`(HEAD 64abfca 가 최신) 는 사용자에게 요청. MATLAB 점유 선언은 이 문서로 갈음.

### 20:3x~21:0x 결과·진행 (compact 직전 스냅샷)
- **0 kg sA 0.35 배터리(chain c ①, 16/16)**: 호버 지터 0.156→**0.005°**/피크 0.01, 새그 2.5, 이동 7.3/오버 14.1(+0.4/+1.0), 대각 13.6, 바람 5 m/s 23.3 cm(경계), **외란 펄스 34°·밀림 10 m(악화, 전복 없음)**. agile 질량 스윕(chain b): 0.25 kg 3.1/4.3, 0.5 kg 1.8/1.8, 0.75 kg 1.2/0.7, 1 kg 0.95/0.27, 1.5 kg 1.5/1.0, 2 kg = precision — 0.25 kg부터 유효 ✅ (`fig_bat_mass_sweep_agile.png` 재생성 필요: `python perf_metrics.py`).
- **ILC step(chain c ②)**: 원 명령 대비 RMS 2.24 → 1.38 → 1.23 cm (2회, 보정 ≤2.2 cm) ✅ — `sim_result_step_ilc{1,2}.mat`, `output/traj_correction.json`.
- **chain d(jitter_a 역위상 상쇄)**: ZVD 구성 꼬리 0.12°라 상쇄 대상 없음(자세 항 생략) — 실증은 SHAPER=none/바람 가진으로 재설계 필요.
- **사용자 결정(20:4x)**: 0 kg 외란은 "복귀 포기, 이탈 크기 범위 한정(목표 이탈 ≤20°·밀림 ≤1 m·전복 금지)" → **오차 의존 비선형 자세 게인** g(|e|)=1+(gmax−1)·sat((|e|−e0)/e1) (`Scripts_Data/qc_nl_att_apply.m` Fcn 블록 삽입, `diagnose/tune_0kg_r5.m` 격자 gmax{1,1.6,2.1,2.9}×e0{1.5,3,6}°, 결과 `tune_0kg_r5_nl.csv`) — chain f 큐(2 kg r2 뒤).
- **2 kg r2**(chain e, BASE sA1.0/r_att1.2/filtM_pos0.05/sZ1.2/kd_alt0.08, `tune_2kg_r2.csv`) 실행 중.
- **counter_swing 온라인판(C++ SwingDamper)**: 실시간 규약 보강(O(1)·무할당·dt 가드·스테일 워치독·NaN 가드·지연 위상 보상 latencyS·상수 사전계산 prepare()·출력 레이트 제한·텔레메트리) — 미빌드·미검증. 다음: Simulink 동일 블록(memory surgery) + jitter_a SHAPER=none 실증, 부호/위상 확정.
- **체인 로그 함정**: PowerShell `Add-Content`가 OneDrive 잠금으로 조용히 실패 → 체인 b/c/d의 "완료" 표식이 안 남아 다음 체인이 대기 → 산출물(csv/mat) 존재로 판단하고 수동으로 완료 줄 추가해 풀었음. 이후 체인은 산출물 파일 존재로 대기(chain f).
- **파일 정리·발표 그림**: `figure/00_pipeline/slide_{1,2,3}_*.png` (16:9), `fig_pipeline.png`, mermaid README (`make_pipeline_slides.py`, `make_pipeline_figure.py`).
- 미커밋: 이번 절 전부(perf_battery agile 케이스, verify_pipeline 로그 경로, traj_learn/learn 동사, allowed_limits, PERFORMANCE.md, C++ SwingDamper/sA0kg 0.35, r5/qc_nl_att_apply, 슬라이드).

## 진행 중 / 다음 (우선순위)
1. ~~체인/그림/README/스펙/스케줄 반영/C++ 동기~~ 완료 (19:10). 커밋만 남음(푸시는 사용자: 서브모듈 → 부모).
2. **2 kg 앵커 튜닝** 진행 중(`tune_2kg.m` 18축 47점, ~19:50 종료 예상): BASE(외삽) 호버 0.158°/새그 8.0 cm/이동 4.08 cm; 초기 관측 sA 1.0 → 지터 0.046°, r_att 1.2 개선, limit_att 무영향, **새그 8 cm는 자세 게인 무관**(sZ/altCmdSat/biasChassis/kp_alt 축 결과 대기). 결과 → parameters.m 1~2 kg 식(sA 상한·sZ·FF) + PERFORMANCE.md §7 + SPEC M5.
2b. chain b/c 결과 반영: agile 질량 스윕(0.25~0.75 kg 신 자세 스케줄 하 유효성), 0 kg sA 0.35 배터리(호버 12 s 지터·이동·외란), ILC step 2회 수렴(2.9 cm → ?) → PERFORMANCE.md §9b(사후 학습) + figure 08 폴더.
3. C++ 빌드/골든 트레이스(msys64 머신 ★), yaw Y1(★튜닝 세션), 자세 스텝 하네스(R1~R3).

## 사용자 결정 기록
- 0 kg은 스펙 밖이라도 정직하게 기록; 설득 본편은 1 kg. 배터리 저장착 = 실기 설계 정답(무하중 상태 제거).
- 모터 내부 루프는 튜닝 제외. Saturation·z·필터 전 항목은 튜닝 대상.
- 시뮬 길이 단축(호버 6 s/이동 10 s) 승인. FF √질량 변경 승인. 결과 좋으면 C++도 수정.
