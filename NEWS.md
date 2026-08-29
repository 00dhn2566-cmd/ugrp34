# 팀 소식 (NEWS)

> `git pull` 후 이 파일부터 보면 최근 변동을 따라잡을 수 있습니다. 최신이 맨 위. 상세는 링크된 문서가 기준.

---

## 2026-08-29 — 2학기 개강 직전 현황 정리 + 회의 안건 (길남)

**TL;DR**: 방학 checkpoint("Isaac 시뮬 + RL 훈련 완료")는 원래 정의대로는 미달 — Isaac 렌더러 6주째 차단. 대신 윤호 PyBullet 프로토타입으로 파이프라인 전 구간은 뚫렸고, 성진은 08-18~23에 동역학 문서화·0kg 재튜닝·지연 강건화를 진행했다(0kg 앵커 선택·`measAgeS` 생산자는 미결). **시뮬레이터 후보가 Isaac/PyBullet/Gazebo 3개**가 된 상태라 2학기 마일스톤·역할 분담을 첫 안건으로 올렸다. 안건 전체: [meeting_agenda_2026-08-29](overall_gilnam/docs/meeting_agenda_2026-08-29.md) (08-11 브리핑의 A1~A7을 잇는 문서). Isaac 전환 시 필요 요소 표: [ISAAC_MIGRATION_CHECKLIST](reinforcement_yunho/docs/ISAAC_MIGRATION_CHECKLIST.md).

- **윤호**: `rl/state_window_adapter.py` corner→normal **폴백** 부호가 아직 `cross(c1−c0, c3−c0)` (08-11 요청 미반영 — 확정 부호는 `cross(c3−c0, c1−c0)`; 명시 `normal` 필드가 있으면 무해) / 로터 인덱스·CW/CCW 매핑 확정 (성진 C++ 믹서표 대조가 이것 때문에 보류) / 2차 데이터셋 생성기에 잠정 확정 ①②⑤ 반영 여부 회신
- **태민**: 08-11 액션 5건 회신 대기 중 — 단 T_IC 항목은 08-18 잠정 확정 ⑥(루트 spec §6.1 표준 R_IC)으로 **대체**됨, "항등" 지시는 폐기. 7월 이후 기록이 없어 근황·블로킹 여부부터 확인 필요
- **성진**: `capability.json`을 계획기(길남)가 읽도록 연동 — 착수 시점 회의에서 / `measAgeS` 생산자 지정 필요 / rpm↔rad/s 단위 결착은 Isaac·Gazebo 이식 시 주의 사항으로 체크리스트에 반영해 둠
- **전원**: 잠정 확정 6건 일괄 추인 예정 (안건 2-2) / 최종보고서 12/12, 9월 연구노트 9/30

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
