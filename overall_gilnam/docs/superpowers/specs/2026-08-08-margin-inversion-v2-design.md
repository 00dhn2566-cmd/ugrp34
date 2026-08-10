# margin 역산 재계산 (v2) — 태민 실방식 병행·worst-window 판정·실현 가능성 상한

> 2026-08-08 · 담당: 류길남 · 전 인원 감사(scratchpad/audit/ 5편)에서 발견된 v1 결함 3건의 정정
> 근거: `visual_imaging_taemin/window_recon_node.py`(태민 실방식 — 코드로 확인), `overall_gilnam/docs/eval_target_derivation.md`(v1), 감사 보고 gilnam.md·seams.md

## 배경 — v1의 결함 3건 (감사 확인)

1. **집계 방식 가정 오류**: v1은 "태민 원본 코드는 리포에 없음"을 전제로 쌍별 삼각측량+중앙값을 가정했다. 사실 태민의 실코드 `window_recon_node.py`는 리포에 있으며(7/4 커밋 — v1 작성 시 grep을 "길남" 키워드로만 해 놓친 착오), 방식은 **corner별 전 관측 시선의 최소자승 교점**(평균 성격, det_conf ≥ 0.7 필터·corner_vis=1만·시차각 2° 문턱)이다. 중앙값보다 꼬리 노이즈에 민감 → 허용 픽셀이 더 엄격해질 것.
2. **worst-window 희석**: v1 침범은 3창문 평균 — 실제 임무는 모든 창문을 통과해야 하므로 판정은 최악 창문 기준이어야 한다 (v1 데이터로도 blue x1 침범 107.6mm > 100mm).
3. **실현 가능성 미검토**: 창문 크기 − 기체 유효 반경 − 추종 예산을 빼면 margin 100mm가 이 씬에서 기하적으로 가능한지 자체가 의문 (감사 주장 슬랙 68.9~95.5mm — 본 작업에서 독립 재산출).

## 범위

- `vision/eval_recon3d.py` 확장(태민 방식 재현 + method 분기), 스윕·시드 체크 재실행, 문서 개정. 다른 팀원 코드 무변경 (태민 노드는 읽기만 — 재현 대상).
- CLI 확장 없음 — 재실행은 스크래치 스크립트로 (YAGNI).

## 구성요소

| 항목 | 내용 |
|---|---|
| `reconstruct_windows_rays(records, scene_gt, det_conf_min=0.7, min_parallax_deg=2.0)` | 태민 수치 경로 재현: corner_vis=1 관측만, 시선 d=normalize(((u−cx)/fx, (v−cy)/fy, 1)), world 변환은 **body≡camera 항등** (오프라인 재현 — T_IC 대체가 아니라 방식 차이만 측정), corner별 A+=I−ddᵀ·b+=(I−ddᵀ)c 누적 후 3×3 해. 시차각·관측수·4-corner 완성 조건 태민 코드와 동일 (시차각 = corner별 min-dot의 arccos → 창문은 corner별 시차각의 최소가 문턱 이상일 때만 채택 — 태민 코드 그대로 복제). 반환 형식은 `reconstruct_windows`와 동일 (`n_pairs` 키에 관측 수 기록 — 의미 차이는 docstring 명시) |
| `evaluate_records(..., method="pairs_median")` | `"rays_ls"` 분기 추가. 크기 계산도 방식별: pairs_median = 양변 평균(기존), rays_ls = 태민식 단일 변 (w=\|c1−c0\|, h=\|c2−c1\|) |
| 재실행 | 동일 노이즈 스트림(시드 1234, 배율 {0.25,0.5,1,1.5,2,3})을 두 집계로 평가 + 시드 4개({1234,7,99,2026}) × 경계 배율({0.5,0.75,1.0,1.25,1.5}) 스팟 체크 재실행 |
| 판정 규칙 v2 | 창문별 침범_i = center_err_i + mean(\|Δw_i\|, \|Δh_i\|)/2 (v1 공식을 창문 평균 없이 창문별로 적용), **판정값 = max_i 침범_i** (worst window). margin별 허용 배율은 max-침범 곡선의 선형 보간, 허용 픽셀 = 8.87 × 배율. 과외삽 금지 |
| 실현 가능 margin 상한 | 창문별 상한 = min(w,h)/2 − r_body − e_track. r_body = 0.35m (planner clearance와 동일 파라미터), e_track ∈ {0, 50mm} 병기. 씬 최악 창문 기준 상한 명시 |
| 문서 개정 | `eval_target_derivation.md` 전면 개정 (결론 = 두 집계 × worst-window 판정표 + 실현 가능성 절 + 오류 정정 고지), `eval_recon3d.py` docstring의 "태민 원본 코드는 리포에 없음" 정정, `e2e_rehearsal_report.md`의 "100mm PASS" 인용 문장 갱신, 체크리스트 0번 갱신 |

## 검증 게이트

1. **태민 방식 재현 게이트**: 무노이즈 스트림에서 rays_ls corner 오차 ≤ 1mm — 태민 7/4 표(0.01~0.07mm)가 이 방식의 산출물이므로 자릿수 정합이 재현의 증명. FAIL이면 스윕 진행 금지.
2. 기존 pairs_median 경로·테스트 전부 불변 (회귀 없음).
3. 감사가 제시한 수치(허용 픽셀 4.75/6.93/8.22px, 슬랙 68.9~95.5mm)는 **참조만 하고 전부 독립 재산출** — 불일치 시 그대로 보고.

## 성공 기준

1. 게이트 1 통과 + 기존 테스트 green.
2. 두 집계 × worst-window 역산표 + 시드 스팟 체크 + 실현 가능성 표 산출, 전부 실측치.
3. 문서 4곳 개정 일관성 (상호 인용 모순 없음), 오류 정정 고지 포함.

## 한계 (문서에 기재)

- rays_ls 재현은 태민의 ROS 배관(포즈 버퍼 20ms 매칭, 2초 주기 보고, T_IC)을 생략한 수치 경로 재현 — 방식 차이 측정이 목적이며 태민 실행값과 완전 동일은 아님. 교차검증 요청(noisy_stream_x1)은 여전히 유효.
- det_conf 필터는 우리 스트림에서 무효(GT값 1.0) — 실모델 출력에서는 추가 드롭 요인임을 명시.
- 시드·씬은 v1과 동일 (단일 씬 한계 유지).
