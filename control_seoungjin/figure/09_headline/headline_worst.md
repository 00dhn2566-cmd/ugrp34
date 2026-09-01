# 대표 지표 = 최악 조건 성능 (자동 생성: make_headline_figure.py)

| 지표 | 최악값 | 조건 | 스펙 | 판정 | 전 조건 (값) |
|---|---|---|---|---|---|
| 호버 자세 지터 RMS | **0.00512 °** | 0 kg 무풍 | 0.25 | ✅ | 1 kg 무풍 0.00136; 0 kg 무풍 0.00512; 1 kg 바람 5 m/s 0.57; 0 kg 바람 5 m/s 1.71 |
| 추종 RMS 3D | **13.3 cm** | 대각 1.6 m/s 0 kg | 15.0 | ✅ | 미션 step 1 kg 2.88; 미션 jitter_a 1 kg 3.41; 미션 jitter_b 1 kg 3.16; 미션 fly_through 1 kg 2.72; 미션 look_at 1 kg 2.95; 미션 scan 1 kg 1.53; 미션 stop_batch 1 kg 2.07; 미션 step 0 kg 3.13; 미션 jitter_a 0 kg 4.55; 미션 jitter_b 0 kg 4.17; 미션 fly_through 0 kg 3.62; 미션 look_at 0 kg 4.42; 미션 scan 0 kg 1.12; 미션 stop_batch 0 kg 3.06; 대각 1.6 m/s 1 kg 6.06; 대각 1.6 m/s 0 kg 13.3; 1 m 이동 0 kg 6.96; 1 m 이동 0.25 kg 5.24; 1 m 이동 0.5 kg 4.29; 1 m 이동 0.75 kg 3.7; 1 m 이동 1 kg 3.27; 1 m 이동 1.5 kg 3.31; 1 m 이동 2 kg 3.31 |
| 오버슈트 | **9.32 cm** | 1 m 이동 0.25 kg | 10.0 | ✅ | 미션 step 1 kg 3; 미션 jitter_a 1 kg 4.02; 미션 jitter_b 1 kg 3.28; 미션 fly_through 1 kg 3.09; 미션 look_at 1 kg 2.21; 미션 scan 1 kg 0.235; 미션 stop_batch 1 kg 0.077; 미션 step 0 kg 5.01; 미션 jitter_a 0 kg 6.88; 미션 jitter_b 0 kg 5.19; 미션 fly_through 0 kg 4.88; 미션 look_at 0 kg 4.62; 미션 scan 0 kg 0.273; 미션 stop_batch 0 kg 0.033; 1 m 이동 0 kg 13.1; 1 m 이동 0.25 kg 9.32; 1 m 이동 0.5 kg 7.18; 1 m 이동 0.75 kg 5.82; 1 m 이동 1 kg 4.85; 1 m 이동 1.5 kg 4.99; 1 m 이동 2 kg 4.99 |
| 이륙 새그 | **7.88 cm** | 2 kg | 5.0 | ❌ | 0 kg 2.46; 1 kg 4.99; 2 kg 7.88 |
| 도착 후 잔류 자세 RMS | **0.676 °** | 1 m 이동 0.5 kg | 0.25 | ❌ | 미션 step 1 kg 0.056; 미션 jitter_a 1 kg 0.085; 미션 jitter_b 1 kg 0.053; 미션 fly_through 1 kg 0.058; 미션 look_at 1 kg 0.048; 미션 scan 1 kg 0.003; 미션 stop_batch 1 kg 0.002; 미션 step 0 kg 0.182; 미션 jitter_a 0 kg 0.208; 미션 jitter_b 0 kg 0.174; 미션 fly_through 0 kg 0.174; 미션 look_at 0 kg 0.145; 미션 scan 0 kg 0.223; 미션 stop_batch 0 kg 0.12; 1 m 이동 0 kg 0.581; 1 m 이동 0.25 kg 0.614; 1 m 이동 0.5 kg 0.676; 1 m 이동 0.75 kg 0.653; 1 m 이동 1 kg 0.643; 1 m 이동 1.5 kg 0.65; 1 m 이동 2 kg 0.661 |
| 외란 펄스 0.3 N·m 최대 이탈 | **10.1 °** | 0 kg agile | 20.0 | ✅ | 1 kg precision 2.33; 1 kg agile 1.58; 0 kg precision (비선형 자세 게인 배포) 10.1; 0 kg agile 10.1 |
| 외란 후 수평 밀림 | **0.257 m** | 0 kg (비선형 자세 게인 배포) | 1.0 | ✅ | 1 kg precision 0.0431; 0 kg (비선형 자세 게인 배포) 0.257 |
| 정상풍 5 m/s 위치 유지 | **11 cm** | 0 kg | 25.0 | ✅ | 1 kg 1.46; 0 kg 11 |
| 응답 시간 | **2.41 s** | yaw 90° rise 0 kg | — | — | 고도 1 m rise 1 kg 0.955; 고도 1 m rise 0 kg 0.945; yaw 90° rise 1 kg 2.12; yaw 90° rise 0 kg 2.41; 외란 복귀 1 kg 0.74; 정착 ±5 cm 1 kg 2.2; 정착 ±5 cm 0 kg 2.4; plan 벽시계 0.7; 비상 서브프로세스 0.75; 정지 v0 0.5 m/s 0.65; 정지 v0 1 m/s 0.9; 정지 v0 1.6 m/s 1.27; 정지 v0 2 m/s 1.52 |
| 비상 정지 거리 | **0.924 m** | v0 1.6 m/s | 1.0 | ✅ | v0 0.5 m/s 0.116; v0 1 m/s 0.391; v0 1.6 m/s 0.924; v0 2 m/s 1.41 |
