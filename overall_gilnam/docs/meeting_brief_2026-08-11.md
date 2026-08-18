# 회의 안건 통합 브리핑 (2026-08-11 기준)

> 류길남(총괄) 작성. 회의가 잡히기 전이라도 비동기로 읽고 의견 줄 수 있게 안건별 근거 문서를 연결해 둠.
> 8월 첫 주~둘째 주 진행분(margin 역산 v1→v2, 웨이포인트 계획기, E2E 리허설, 전 인원 코드 감사, 미검출 진단, 도메인 갭 정량화)이 반영된 목록.

> **2026-08-11 갱신 — 잠정 확정 4건** (규격 관리자/총괄 결정, 이의 있으면 회의에서 재론): ① 가림 corner vis=0 (A3 → 추인만) ② 2차 데이터셋 창문 외관 랜덤화 (A2의 실무 절반 → 실물 정의만 회의 잔여) ③ ⓑ 재학습 공식 채택 (A1의 방침 절반 → margin 전제·자원만 잔여) ④ 계획기 limits 물리 한계의 80% 잠정 적용 (A6 → 성진 승인 시 복원 가능).

## A. 결정이 필요한 안건 (우선순위순)

### A1. 비전 목표치·margin — 재학습 여부가 걸림 ★최우선
- **현황**: margin 역산 v2 결과, 현 모델(corner 평균 8.87px)은 두 집계(쌍별+중앙값 / 태민 실방식 LS) 모두에서 margin 50·100mm **FAIL**. 나아가 **margin 100mm는 씬 최악 창문(red, 반치수 418.9mm)에서 기체 유효 반경 0.35m 기준 기하적으로 불가능** (여유 68.9mm, 추종 오차 50mm 반영 시 18.9mm).
- **결정할 것**: ① margin 값 (사실상 "몇 mm가 가능한가"의 문제) ② ⓑ 재학습 착수 여부 ③ 씬 전제(창문 0.8~1.2m) 또는 기체 여유(0.35m) 자체의 재검토
- **근거**: `overall_gilnam/docs/eval_target_derivation.md` (v2)

### A2. 창문의 시각 정의 (신규) — 실기 리스크 → **확정 제안으로 격상 (2026-08-18)**
- **현황**: 두 합성 도메인이 서로 다른 가정 사용 중 — 윤호 렌더(학습 데이터) = **채운 색판**, 길남 토이 = **테두리 프레임**. 정량화 결과 모델은 채움 표현에만 반응 (테두리 창문 재현율 9%, 채우면 92%).
- **8/18 실증 (윤호 PyBullet 프로토타입, `feat/pybullet-prototype-pipeline`)**: 창문을 채우면 검출은 되지만 **앞 창문이 뒤 창문을 가려 color_judge가 세 창문을 전부 red로 판정, 삼각측량이 창문들을 하나로 융합(복원 오차 207mm)** — 채움 표현은 검출 단독으로만 유리하고 색 판정·복원에서 깨짐. 윤호는 뚫린 테두리로 되돌리고 모델을 파인튜닝하는 방향으로 진행 중.
- **결정할 것**: 열린 질문이 아니라 **"채움 불가 → 테두리 프레임(+개구부 투명) 기준으로 확정" 제안을 추인**. 2차 데이터셋의 외관 랜덤화(잠정 확정 ②)는 이 기준을 중심으로 폭만 두는 것으로 조정.
- **근거**: `overall_gilnam/docs/domain_gap_quantification.md` (반박 각주 포함), `prototype_demo/README.md` (윤호 브랜치)

### A2-b. 카메라 intrinsics 후보 확정 (신규, 2026-08-18)
- **현황**: 팀 전체가 쓰는 fx=fy=600(HFOV 93.7°)은 근거 없는 placeholder. 윤호가 spec §7 "§6 intrinsics 기입" 항목의 후보로 **fx=fy=763 (HFOV 80°, 1280×720)** 을 근거와 함께 제시 (`prototype_demo/config/camera.yaml`, status: provisional).
- **결정할 것**: 후보값 확정 여부. 확정 시 삼각측량 오차·margin 역산·scan rate 전부 재계산 필요 (학습 가중치는 무영향).
- **근거**: `prototype_demo/config/camera.yaml`

### A3. 창문-창문 가림 라벨 정책 (신규)
- **현황**: 미검출 31건 중 19건(61%)은 모델 결함이 아니라 **가려진 창문에 라벨이 붙어 있어서** 생기는 채점 문제 (규격 §4 공백). HSV 색 오판 19건도 같은 뿌리(겹침).
- **결정할 것**: 가림 corner를 visibility=0으로 마킹(정책 C 통합, 권장) vs 고가림 창문 라벨 제외. **이 결정 없이 재학습하면 미검출 61%는 그대로.**
- **근거**: `overall_gilnam/docs/miss_tail_diagnosis.md`, 루트 spec §7에 항목 추가됨

### A4. state_window 인터페이스 spec v1.0 확정
- **현황**: corner 유도 normal 부호는 08-08 잠정 확정(winding 계약의 따름정리 — §3.1) → **추인만 필요**. 잔여: 축 선택(A/B/S), 전송 방식, passed 소유권.
- **연동 이슈**: 태민 `/window_positions` 필드명(center_w/corners_w/width/height)이 spec §6.2(center/corners_3d/size_wh/normal)와 불일치 — 확정 시 함께 정렬 (~20줄 수정).
- **근거**: `overall_gilnam/docs/state_window_interface_spec_v0_1.md`

### A5. 담당 지정 2건
- RL 경로계획(궤적 생성) 실무 담당 — 참고: 고전 웨이포인트 계획기(`overall_gilnam/planning/`)가 생겨 "RL 없이도 파이프라인은 뚫린" 상태. RL은 성능 개선 트랙으로 논의 가능
- 보상 함수 설계 담당 (`rl/configs/reward_default.yaml` 스텁)

### A6. 제어 접점 파라미터 (성진)
- 계획기 limits 기본값(v_max 2.0 등)이 성진 물리 한계의 100% — 예산 규칙(80%) 적용 여부 협의, `planning/planner_limits.yaml`만 갱신하면 됨
- scan.rate 회신값: **0.75 rad/s로 정정** (as-built 탐지 주기 2Hz 반영 — `docs/scan_rate_estimate.md` 2026-08-11 정정판). 태민 노드 주기 상향 시 1.0 회복

### A7. 클러스터 드라이버(595→580) 팔로업 — 재학습(A1)이 결정되면 다시 전체 병목

## B. 통보·확인 사항 (결정 불요, 회의에서 확인만)

- **감사 후속 수정 목록** (각 담당에게 개별 통보 진행 중): 윤호 — 456fa5b 클린 머지, CPU 렌더러 조명 랜덤화 키 불일치 수정, RL 어댑터 corner-normal 부호 플립 / 태민 — /window_positions 필드명, EuRoC T_IC↔body≡camera 충돌, §5 center 정의(가시-only 평균 → 4점 평균), noisy_stream_x1 교차검증 회신
- 루트 spec v0.2 갱신 공유: §4.3 corner 순서 "접근측 기준" 명시, §7 완료 2건 체크 + 신규 2건(A2·A3) 등록
- **윤호 PyBullet 프로토타입 (8/18, `feat/pybullet-prototype-pipeline`)**: 진짜 이미지→실추론→복원→계획 E2E 최초 달성. 병합 요청 2건(이 브랜치 + `feat/rl-seam-format-alignment`). 복원단 발견사항(태민 LS가 conf 미가중·중복 검출 중복표·아웃라이어 제거 없음 → `overrides/`에 conf가중+Huber 대안, center 오차 268→63mm)은 태민 통보에 반영
- 8월 진행분 문서 위치: margin v2 · E2E 리허설 보고 · 미검출 진단 · 도메인 갭 정량화 — 전부 `overall_gilnam/docs/`

## C. 권고 요약 (길남 의견)

1. A2·A3(창문 정의·가림 정책)를 먼저 확정해야 A1의 재학습이 한 번에 끝난다 — 재학습 스펙 순위: 가림 정책 → 2차 데이터셋 증강(창문 외관 랜덤화 + 조명 버그 수정) → imgsz=960 (`miss_tail_diagnosis.md`).
2. margin은 "고르는 값"이 아니라 씬·기체 전제에 의해 상한이 결정되는 값임이 확인됐으므로, A1은 margin 숫자 논쟁 대신 **전제(창문 크기 스펙·기체 여유·추종 예산) 확정**으로 접근할 것.
