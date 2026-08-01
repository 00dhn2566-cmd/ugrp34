# scan.rate_rad_s 산정 (비전 → 제어 조율점 회신)

> 배경: 제어 yaw `scan` 모드의 회전 속도 `rate_rad_s`는 비전이 산정하는 필수 입력
> (`control_seoungjin/EXTERNAL_INTERFACE.md` 3·6절 — 제어는 물리 상한 1.0 rad/s만 집행).
> 작성: 2026-08-01 류길남. intrinsics 확정(윤호) 시 숫자만 재계산.

## 회신 값

| 조건 | rate_rad_s |
|---|---|
| **시뮬 (이번 학기)** | **1.0** — 물리 상한 그대로 (Isaac 렌더는 모션 블러 없음 → 비전 제약 비구속) |
| **실기 대비 보수값** | **0.6** — 모션 블러 제약이 지배 |

## 근거

가정: fx=600px (placeholder intrinsics, 1280×720), 수평 FOV = 2·atan(640/600) ≈ 1.63 rad(93.7°),
탐지 주기 10Hz, 노출시간 5ms(실기·밝은 실내), corner 오차 예산 3px 중 블러 몫 2px.

1. **모션 블러**: blur_px = ω·t_exp·fx ≤ 2px → ω ≤ 2/(0.005×600) = **0.67 rad/s**
   (시뮬 렌더는 블러가 없어 이 제약 자체가 사라짐)
2. **프레임 커버리지**: 정책 A 모델은 4-corner 완전 가시 시에만 검출 → 스캔 중 창문이
   완전 가시 구간(FOV − 창문 각폭, 근거리 최악 ≈ 1.63−0.5 = 1.13 rad)에서 K=3회 이상
   검출되어야: ω ≤ 1.13×10/3 ≈ **3.8 rad/s** → 비구속

일반식 (조건 바뀔 때 재계산용):

```text
rate_max = min( 1.0(물리 상한),
                blur_budget_px / (t_exp × fx),
                (FOV − θ_window) × f_det / K )
```

## 재검토 트리거

- 윤호 intrinsics 확정 (fx 변경 시 블러·FOV 항 재계산)
- 실기 카메라 노출시간 실측 (5ms 가정 대체)
- 탐지 주기 실측 (목표 하드웨어에서 `infer_stream.py` 벤치 — CPU 참고치 3.1fps, GPU에서 재측정 필요)
