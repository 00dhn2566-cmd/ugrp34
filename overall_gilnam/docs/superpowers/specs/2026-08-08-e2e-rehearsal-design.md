# 파이프라인 E2E 리허설 + corner 유도 normal 부호 확정 — 설계

> 2026-08-08 · 담당: 류길남 · 체크리스트 4번 "전체 파이프라인 통합 검증 주도" 실행
> 근거 문서: `state_window_interface_spec_v0_1.md`, `vision/sample_stream/README_stream.md`(winding 계약), `planning/`(계획기), `vision/eval_recon3d.py`(삼각측량 재현)

## 배경·목적

비전→복원→계획 각 단계는 개별 검증됐지만 전 구간을 이은 적이 없다. 오늘 만든 조각(노이즈 주입·삼각측량 재현·웨이포인트 계획기)을 한 스크립트로 연결해 **노이즈 수준별로 최종 계획 품질이 어떻게 열화되는지** 측정한다 — 특히 "계획 경로의 실제 창문 통과점이 개구부 중심에서 얼마나 벗어나는가"는 margin 논의의 끝단 근거가 된다.

## 결정: corner 유도 normal 부호 (잠정 확정, 2026-08-08 비동기)

corner winding은 이미 확정된 계약이다 (v0.2 §4.3: **접근측에서 본** TL→TR→BR→BL — 접근측에서 보면 시계방향). 오른손 법칙에 따라 접근측을 향하는 법선은:

```
n̂ = normalize(cross(c3 − c0, c1 − c0))    # (BL−TL) × (TR−TL) → 접근측
```

- 별도 합의가 필요한 새 관례가 아니라 **기존 winding 계약의 따름정리** — `cross(c1−c0, c3−c0)`(윤호 rl/README가 antiparallel로 지적한 그 순서)의 인자를 뒤집은 것.
- §5가 corner 순서를 보존하므로 **삼각측량 복원 corner에도 그대로 유효**.
- 처리: `state_window_interface_spec_v0_1.md` §3.1의 미결 2건(normal ±, winding 재정의)을 "잠정 확정 — 이의 없으면 v1.0 반영"으로 기입, 팀 통보 목록에 추가 (윤호 rl/README의 OPEN 지적 해소 통보 포함).

## 범위

- `overall_gilnam/integration/` 신설 + 계획기·eval_recon3d 소폭 확장 (아래). 다른 팀원 코드 무변경.
- 재계획 주기·실시간성·RL 연동은 범위 밖. 성진 궤적 계획기 실행(MATLAB)도 범위 밖 — waypoints_config 스키마 검증까지.

## 구성요소

| 파일 | 변경 | 역할 |
|---|---|---|
| `planning/window_waypoint_planner.py` | 확장 | `normal_from_corners(corners_3d)` 헬퍼 + `gate_points` 폴백 (normal 부재 시 corners_3d에서 유도, 둘 다 없으면 에러 유지) |
| `overall_gilnam/docs/state_window_interface_spec_v0_1.md` | 갱신 | §3.1·§7 미결 2건 잠정 확정 기입 |
| `vision/eval_recon3d.py` | 확장 | 공개 API `reconstruct_windows(records, scene_gt, ...)` — 창문별 복원 corner 원본 반환 (`evaluate_records`가 내부적으로 재사용, 기존 동작 불변) |
| `integration/e2e_rehearsal.py` | 신설 | 전 구간 실행 + 스케일 스윕 결과표 CLI |
| `integration/tests/test_e2e.py` | 신설 | 무노이즈 게이트 + 이음새 계약 테스트 |
| `overall_gilnam/docs/e2e_rehearsal_report.md` | 신설 | 결과 보고 (비동기 공유용) |

## 데이터 흐름

```
sample_stream (GT §5+pose, seed 42, 302프레임)
  ── noisy_stream.make_noisy_records(×scale, seed 1234) ──▶ 노이즈 스트림
  ── eval_recon3d.reconstruct_windows ──▶ 창문별 복원 corners_3d
  ── [이음새] 창문 맵 조립: center=corner 평균, size_wh=변 길이 평균,
              normal=normal_from_corners (부호 확정 공식) ──▶ §6.2 window_map
  ── planning.plan_waypoints ──▶ waypoints_config (성진 스키마 검증 포함)
```

## 지표 (스케일 {0, 0.5, 1.0, 1.5, 2.0}, 시드 1234)

1. **게이트점 오차**: GT 창문 맵으로 만든 계획 대비 접근·이탈점 위치 오차 (창문별·mm)
2. **통과점 여유 잠식** ★: 계획의 접근→이탈 구간이 **GT 창문 평면**을 교차하는 점의 개구부 중심 이탈 (|u|, |v|) → 잔여 여유 min(w/2−|u|, h/2−|v|) (mm) — margin 논의 직결 지표
3. **안전 경고**: `crossing_warnings(계획, GT 창문, clearance_margin)` 발생 수 + 스키마 검증 통과 여부

## 검증 게이트

scale 0(무노이즈 복원)의 계획이 GT 창문 맵의 계획과 **게이트점 오차 ≤ 1mm**로 일치해야 스윕을 신뢰 (삼각측량 무노이즈 게이트와 동일 패턴 — 이번엔 이음새·부호 공식까지 포함한 전 구간 검증이 된다).

## 성공 기준

1. `normal_from_corners`가 synth 씬 GT corner에서 기존 normal과 일치 (내적 > 0.999) — 부호 공식 검증
2. 계획기 폴백: normal 없는 창문(corners_3d만)도 계획 성공, 결과 동일
3. scale 0 게이트 통과 (≤ 1mm)
4. 스케일 스윕 표 + 보고 문서 산출, 전 스케일 스키마 검증 통과
5. 신규 포함 planning·vision·integration 테스트 전체 green (기존 environment 실패 1건 제외)

## 한계·가정 (보고서에 동일 기재)

- 단일 시드(1234) — 앞선 스팟 체크에서 판정의 시드 강건성은 확인됐으므로 리허설은 대표 실현치로 충분하다고 판단.
- 복원 창문 맵은 전 프레임 일괄 삼각측량 (S-2 맵 유지·스무딩은 태민 몫 — 리허설은 태민 대역의 보수적 근사).
- 미검출 드롭이 특정 창문의 n_pairs를 줄여 복원 품질에 반영됨 — 창문이 통째로 복원 불가(n_pairs=0)면 해당 스케일 행에 "복원 불가"로 기록 (계획은 해당 창문 제외로 진행하되 실패로 집계).
