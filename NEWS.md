# 팀 소식 (NEWS)

> `git pull` 후 이 파일부터 보면 최근 변동을 따라잡을 수 있습니다. 최신이 맨 위. 상세는 링크된 문서가 기준.

---

## 2026-08-22 — 제어: yaw 외란 강건화 + 상위용 실시간 스펙표 (성진)

**TL;DR**: 구운 모델에 **yaw 오차 ±pi 랩이 없던 결함**을 찾아 고쳤고(C++ 이식본은 원래 있어서 둘이 달랐다),
anti-windup 을 켰다. 미션 7편 회귀 게이트 7/7 통과, 추종 RMS 전부 동일하거나 소폭 개선.
그리고 **상위 경로 생성기가 읽을 실시간 스펙표 `capability.json`** 을 신설했다
([INTERFACE_SPEC §5b](control_seoungjin/INTERFACE_SPEC.md)).

- **윤호/길남**: 계획기가 이제 **정적 표 대신 `capability.json` 의 `limits` 를 읽으면 된다**
  (`plan_waypoints(v,a,j,snap)` 에 그대로 투입). 질량·외란·연산지연에 따라 이미 깎인 값이다.
  감쇄는 시계 배율 `s` 하나로 표현: `v∝s, a∝s², j∝s³, snap∝s⁴`.
  **아직 컨트롤러가 파일을 쓰지 않는다** — 규격·생성기만 있는 상태라 배선은 다음 작업.
- **전원**: 알아둘 실측 두 개
  - **폐루프 오버슈트 ≈ 3.13·v²** [cm, m/s] → 5 cm 목표면 순항 `v ≤ 1.26 m/s`
  - **지연 여유: 자세 10~20 ms / 위치 0~50 ms** — VIO 30~100 ms 는 현재 한계 밖 (태민 참고)
- **성진(본인)**: 08-18 성능 지표 세션의 `parameters.m` 0 kg 재튜닝분이 이 머신에 미동기라
  `tune_0kg_r5.m` 에서 복원해 쓰는 중 (`qc_0kg_tuned_apply.m`, 원본 회수되면 폐기)

세부: `control_seoungjin/docs/YAW_DISTURBANCE_I.md`, `docs/SPEED_GOVERNOR.md`,
그림 `control_seoungjin/figure/10_capability/`

---

## 2026-08-18 — 윤호 PyBullet 프로토타입 + 안건 격상 (길남 정리)

**TL;DR**: 윤호가 `feat/pybullet-prototype-pipeline`(미병합)에서 **진짜 이미지 → 실추론 → 복원 → 계획 E2E를 최초 달성**. 그 과정에서 "창문을 채우면 앞 창문이 뒤를 가려 색 판정·삼각측량이 깨진다"를 실증 → 회의 안건 A2(창문 시각 정의)를 **테두리 기준 확정 제안**으로 격상, intrinsics 후보(fx=763, HFOV 80°) 확정 안건 A2-b 추가. [meeting_brief](overall_gilnam/docs/meeting_brief_2026-08-11.md) 갱신됨.

- **윤호**: 병합 요청 2건 (`feat/pybullet-prototype-pipeline`, `feat/rl-seam-format-alignment`) / PyBullet 파인튜닝은 2차 학습 선행 실험으로, 본 학습은 외관 랜덤화 스펙과 합치기
- **태민**: 복원 LS의 conf 미가중·아웃라이어 제거 부재 (윤호 발견, `prototype_demo/overrides/`에 대안 — 반영 여부는 태민 판단)
- **전원**: 8/11 절의 액션은 그대로 유효

---

## 2026-08-11 — 8월 둘째 주 묶음 (길남)

**TL;DR**: margin 역산이 v2로 정정되어 **현 검출 모델은 사실상 전 margin FAIL → 재학습 공식 채택**. 웨이포인트 계획기·E2E 리허설 신설, 전 코드 감사에서 이음새 이슈 다수 발견, 잠정 확정 4건. **회의 전이라도 [meeting_brief_2026-08-11](overall_gilnam/docs/meeting_brief_2026-08-11.md) 한 장만 읽으면 됩니다.**

### 잠정 확정 4건 (이의 있으면 회의에서 재론)

1. 창문-창문 **가림 corner는 visibility=0** 마킹 (루트 spec §7, 정책 C 통합)
2. **2차 데이터셋은 창문 외관 랜덤화** (채움↔테두리↔반투명 개구부 — 현 모델이 "채운 색판"만 인식하는 문제)
3. **ⓑ 재학습 공식 채택** — 순서: 가림 정책 → 증강(+조명 버그 수정) → imgsz=960
4. 계획기 **limits = 물리 한계의 80%** 잠정 (성진 승인 시 복원)

### 멤버별 — 나한테 걸린 것

- **윤호**: ① `feat/rl-seam-format-alignment`(456fa5b) 병합 요청 (main과 겹침 0) ② CPU 렌더러 조명 랜덤화 키 불일치 수정 (`brightness/direction`→`intensity/azimuth_rad`) ③ `rl/state_window_adapter.py` corner-normal 폴백 부호 플립 (확정 부호: 접근측 = cross(c3−c0, c1−c0) — [state_window spec §3.1](overall_gilnam/docs/state_window_interface_spec_v0_1.md)) ④ 2차 데이터셋 생성기에 잠정 확정 1·2번 반영
- **태민**: ① `overall_gilnam/vision/noisy_stream_x1/` 교차검증 회신 ② `/window_positions` 필드명 → spec §6.2 정렬 ③ `window_recon_node` T_IC → body≡camera(항등) ④ §5 center = 4점 평균 ⑤ 검출 주기 2Hz가 scan 속도를 0.75 rad/s로 구속 — 상향 검토
- **성진**: ① scan.rate 회신값 = 시뮬 **0.75** / 실기 0.6 ([scan_rate_estimate](overall_gilnam/docs/scan_rate_estimate.md) 정정판) ② 계획기 limits 80% 잠정 — 100% 가능하면 회신 ③ EXTERNAL_INTERFACE 비상 서술 현행화 확인
- **전원**: 서브모듈 워크트리가 비었으면 `git submodule update --force` (단, Simscape 팀 커밋은 `git -C <서브모듈> fetch <ugrp34 원격> fix/plate-orientation-cg` 후 체크아웃 — NEWS 하단 참고)

### 새 문서·모듈 (8/8~8/11)

| 문서/모듈 | 내용 |
|---|---|
| [eval_target_derivation.md](overall_gilnam/docs/eval_target_derivation.md) (v2) | margin 역산 — 두 집계 × worst-window, **100mm는 기하적으로 불가능** |
| [miss_tail_diagnosis.md](overall_gilnam/docs/miss_tail_diagnosis.md) | 미검출 31건 = 가림 채점 61% + 원거리 저대비 39%, 진짜 미검출 0 |
| [domain_gap_quantification.md](overall_gilnam/docs/domain_gap_quantification.md) | 도메인 갭 원인 = 창문 채움 표현 단일 요인 (9%→92%) |
| [e2e_rehearsal_report.md](overall_gilnam/docs/e2e_rehearsal_report.md) | 비전→복원→계획 전 구간 리허설 (scale 0 게이트 1mm 통과) |
| `overall_gilnam/planning/` | 창문 통과 웨이포인트 계획기 → 성진 waypoints_config 출력 |
| `overall_gilnam/integration/` | E2E 리허설 실행기 |
| `overall_gilnam/vision/` 추가 | noisy_stream(노이즈 주입)·eval_recon3d(삼각측량 평가, 태민 방식 재현 포함)·noisy_stream_x1(태민 패키지) |

### 스펙 변경 (공유 의무 항목)

- 루트 spec v0.2: §4.3 corner 순서 "**접근측에서 본 기준**" 명시, §7 완료 2건 체크 + 가림 정책·창문 시각 정의 항목 추가
- state_window spec §3.1: corner 유도 normal 부호 잠정 확정 (winding 계약의 따름정리)

---

*파일 운영: 새 묶음이 생기면 위에 날짜 절을 추가. 한 절은 TL;DR + 멤버별 액션 + 링크만 — 상세는 각 문서로.*
