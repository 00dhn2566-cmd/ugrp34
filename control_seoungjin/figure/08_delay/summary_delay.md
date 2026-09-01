# 측정 지연 주입 배터리 — 지연 여유 (스펙 T3/R14)

지연은 측정 경로에 Transport Delay 로 주입 (qc_delay_apply: 자세/yaw = 자세 지연, 위치/z = 위치 지연). 판정 = R4/R6/R7/§3.5 유지 (0 kg 는 R6/R7 제외).

## 1kg — 자세 지연 여유: 마지막 통과 10.0 ms / 첫 실패 20.0 ms · 위치 지연 여유: 마지막 통과 0.0 ms / 첫 실패 50.0 ms

| 자세 지연 [ms] | 위치 지연 [ms] | 호버 지터 RMS/피크 [°] | 새그 [cm] | 외란 피크 [°] / 복귀 [s] / 밀림 [m] | 추종 RMS / 오버슈트 [cm] | 자세 피크 [°] | 잔류 [°] | 판정 |
|---|---|---|---|---|---|---|---|---|
| 0 | 0 | 0.018 / 0.031 | 5.0 | 2.35 / 0.74 / 0.04 | 3.28 / 4.9 | 11.8 | 0.457 | OK |
| 10 | 0 | 0.002 / 0.009 | 5.0 | 2.55 / 0.88 / 0.04 | 3.25 / 4.8 | 12.0 | 0.518 | OK |
| 20 | 0 | 1.908 / 2.958 | 5.0 | 4.26 / — / 0.04 | 3.53 / 5.2 | 13.5 | 2.125 | FAIL:hover_att_rms_deg,hover_att_peak_deg,pulse_recover_s |
| 40 | 0 | 7.983 / 12.852 | 5.0 | 14.80 / — / 0.13 | 6.49 / 9.7 | 22.6 | 7.371 | FAIL:hover_att_rms_deg,hover_att_peak_deg,pulse_peak_deg,pulse_recover_s |
| 80 | 0 | 14.872 / 42.674 | 5.0 | 44.39 / — / 45.06 | 86.19 / 112.4 | 50.3 | 15.546 | FAIL:hover_att_rms_deg,hover_att_peak_deg,pulse_peak_deg,pulse_recover_s,track_rms_cm,overshoot_cm,drift_cm |
| 0 | 50 | 1.415 / 3.875 | 13.4 | 5.68 / — / 0.08 | 4.60 / 5.4 | 13.8 | 1.489 | FAIL:hover_att_rms_deg,hover_att_peak_deg,pulse_peak_deg,pulse_recover_s,drift_cm |
