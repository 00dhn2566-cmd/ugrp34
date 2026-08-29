# 회의 안건 — 2026-08-29 (2학기 개강 직전 현황 + 결정 목록)

> 류길남(총괄) 작성. [meeting_brief_2026-08-11](meeting_brief_2026-08-11.md)(08-18 갱신본)을 잇는 문서 — 그 이후 들어온 진행분(윤호 PyBullet 전 구간 프로토타입 병합, 성진 08-18~23 동역학·지연 강건화, 08/03 랩미팅)을 반영하고, **2학기 마일스톤 재설정**을 최상위 안건으로 올렸다.
> 기준: `main` 9b79fe1 (2026-08-23) + 08/03 랩미팅 슬라이드 + UGRP 운영 일정.

---

## 0. 한 장 요약

- **방학 checkpoint(README §6 "Isaac 시뮬 환경 + RL 훈련 완료")는 원래 정의대로는 미달.** Isaac RTX 렌더러는 07/17부터 드라이버 버그로 차단 상태. 대신 윤호가 **PyBullet 위에서 진짜 이미지→검출→복원→계획→성진 제어기→물리 전 구간을 완주**(10창문, [prototype_demo/README.md](../../prototype_demo/README.md)) — 파이프라인 자체는 뚫렸다.
- 팀에 시뮬레이터 후보가 **3개** 생겼다: Isaac(차단, 포토리얼 렌더), PyBullet(프로토타입 완주, CF2X 27g — 팀 기체 아님), Gazebo(성진 SDF 작성 단계, RTX 5060 머신 예정). **하나로 정하거나 역할을 나눠 명시하지 않으면 2학기 통합이 셋으로 갈라진다.**
- 비전: 1차 모델(mAP 0.927)은 corner 8.87px → margin 역산 v2에서 **50/100mm 모두 FAIL, 100mm는 기체 반경 0.35m 기준 기하적으로 불가능**. 재학습 공식 채택(08-11). 잠정 확정 6건이 추인 대기.
- 일정: 최종보고서 **12/12(금)**, 9월 연구노트 9/30, 11월에는 신청 예산의 10%만 집행 가능(잔액 회수·재료 구매 불가), 장비 구매는 과제 종료 2개월 전까지 (UGRP 운영 일정 PDF — `UGRP/공지사항/`, 리포 밖).

## 1. 파트별 현황 (08-29)

| 파트 | 08-18 이후 된 것 | 막힌 것 / 열린 것 |
|---|---|---|
| 비전·총괄(길남) | 계획기 v2→v3(`overall_gilnam/planning/`, 짧은 구간 병합·이탈측 가드, 3000씬 스윕 회귀 0), E2E 리허설 통과, 미검출·도메인 갭 진단, margin v2, NEWS.md 운영 시작 | 재학습 실행(GPU·데이터 도메인 미정), A2-b intrinsics·A2-d 검출 클래스 미결, **계획기가 성진 `capability.json`을 아직 안 읽음**(08-23 성진 미완 항목) |
| 시뮬·RL(윤호) | PyBullet 전 구간 프로토타입 병합(4faa8eb), 3클래스 파인튜닝, 복원단 대안(`prototype_demo/overrides/` conf가중+Huber, center 오차 268→63mm), EuRoC 덤프 | Isaac 여전히 차단. **`rl/state_window_adapter.py`의 corner→normal 폴백 부호가 아직 `cross(c1−c0, c3−c0)`(확정 부호의 반대, 08-11 요청 미반영 — 명시 `normal` 필드가 있으면 그걸 쓰므로 GT 스트림에선 무해, 필드 없는 입력에서 뒤집힘)**. CPU 렌더러 조명 키 수정·2차 데이터셋 생성기 반영 여부 미확인. 로터 인덱스/CW·CCW 매핑 미결 |
| VIO(태민) | — (07-04 이후 커밋 없음) | 걸린 액션 5건 미회신(§4 참조). 윤호 발견: LS 복원이 conf 미가중·아웃라이어 제거 없음(같은 관측에서 center 오차 268mm) — 반영 여부 태민 판단 |
| 제어(성진) | 동역학 문서화(`docs/DYNAMICS*.md`, 지도교수 요구) — 믹서 부호표·유효 모멘트 암 0.093m 실측 확정, **프로펠러 단위 결착(Simscape 로그는 rpm, C++는 rad/s — 이식 시 주의)**; 0kg 재튜닝(sA 0.40 vs 0.50 판단 지점); 성능 지표 그림 108장(`figure/`); ILC 궤적 보정(`traj_learn.py`); **08-22~23 지연 강건화** — 외란 연동 속도 조속기, `capability.json` 생산자(`spec_governor.py`), 재계획 다리(`traj_bridge.py` — 외란 수단 아님으로 반증), C++ 지연 관측기, 테스트 254 통과; RL seam 형식 정합(코어/옵션 분리, 456fa5b) `main` 병합; Gazebo 기체 SDF 1건(`gazebo/fx450_test.sdf`, 구동 확인 전) | `measAgeS` 채우는 쪽 없음, 계획기 미연동, C++ 믹서표 대조는 로터 매핑 확정 대기. 남의 영역 파일 침범→회수 사건(08-19~23) |

**문서 불일치 1건(계속)**: 08/03 랩미팅 슬라이드는 "Yunho – Path planning through RL"로 발표됐으나 저장소 문서는 전부 "RL 담당 미정". 아래 §2-7에서 정리.

## 2. 결정이 필요한 안건 (우선순위순)

### 2-1. 2학기 마일스톤·시뮬레이터 역할 재설정 ★최우선
- **결정할 것**: ① Isaac 드라이버 해소를 기다릴지(관리자 다운그레이드 595→580), 외부 GPU를 구독할지(08/03 공통 과제, 예산 11월 전 집행), Isaac을 이번 학기 범위에서 뺄지 ② PyBullet·Gazebo·Isaac의 **역할 분담**을 명문화 — 권장안: PyBullet = 계획/RL 대리 실험 + 비전 E2E, Gazebo = 성진 제어 폐루프(FX450 파라미터), Isaac = 드라이버 해소 시 포토리얼 데이터셋·VIO 데이터 전용 ③ 그에 맞춰 README §6 마일스톤과 최종보고서의 "검증 환경" 문구 갱신
- **근거**: [ISAAC_CLUSTER_NOTES.md](../../reinforcement_yunho/sim/ISAAC_CLUSTER_NOTES.md), [prototype_demo/README.md](../../prototype_demo/README.md)("CF2X는 팀 기체가 아니다"), [gazebo_setup_log.md](../../control_seoungjin/docs/gazebo_setup_log.md), Isaac 전환 시 필요 요소: [ISAAC_MIGRATION_CHECKLIST.md](../../reinforcement_yunho/docs/ISAAC_MIGRATION_CHECKLIST.md)

### 2-2. 잠정 확정 6건 일괄 추인 (이의 없으면 5분)
① 가림 corner vis=0 ② 2차 데이터셋 창문 외관 랜덤화 ③ ⓑ 재학습 공식 ④ 계획기 limits 80% ⑤ 창문 시각 정의 = 테두리+뚫린 개구부(루트 spec §2.5) ⑥ 카메라-IMU 프레임 규약(루트 spec §6.1) + corner 유도 normal 부호(state spec §3.1). 근거는 [meeting_brief_2026-08-11](meeting_brief_2026-08-11.md).

### 2-3. A1 — margin 전제 확정 + 재학습 착수 조건
- margin은 고르는 값이 아니라 **씬 전제(창문 0.8~1.2m) − 기체 유효 반경(0.35m) − 추종 오차**가 상한을 정한다(최악 창문 슬랙 68.9mm). **결정**: 창문 크기 스펙을 키울지 / 기체 반경 가정을 낮출지 / 추종 예산(e_track)을 얼마로 둘지 → 그 뒤에 픽셀 목표치가 따라 나온다.
- 재학습 실행 조건 **결정**: 데이터 도메인(CPU 절차 렌더러 vs PyBullet 렌더 vs 둘 다 — 윤호 파인튜닝은 이미 PyBullet 도메인), GPU(클러스터 RTX PRO 6000 — 학습은 가능, RTX 렌더만 드라이버 차단 / 윤호 로컬), 스펙 순서(가림 정책→증강+조명 버그→imgsz 960).
- 근거: [eval_target_derivation.md](eval_target_derivation.md) v2, [miss_tail_diagnosis.md](miss_tail_diagnosis.md)

### 2-4. A2-b — 카메라 intrinsics 확정 (+ 태민 OpenVINS 입력 묶음)
- 후보 **fx=fy=763 (HFOV 80°, 1280×720)**, `prototype_demo/config/camera.yaml`. 확정 시 삼각측량 오차·margin·scan rate 재계산(가중치 무영향).
- 같은 자리에서: 카메라-IMU extrinsics 숫자, IMU 노이즈 4개, update_rate, mono/stereo — 태민 `calib/` 입력이 이것 때문에 두 달째 비어 있음.

### 2-5. A2-d — 검출 클래스 설계
- 단일클래스+HSV(현 spec §3) vs 3클래스 직접 분류(윤호 파인튜닝 방향). HSV는 겹침 오판 19건, 3클래스는 §4.1 "같은 색 중복" 시 식별 불가. **먼저 정할 것: 한 씬에 같은 색 창문이 둘 이상 올 수 있는가** — 이게 답이면 클래스 설계는 따라온다.

### 2-6. A4 — state_window 인터페이스 v1.0 잔여
- 축 선택(A-1/2/3, B-1/2/3, S-1/2/3), 전송 방식, `passed` 소유권. 권장 조합은 문서 §5. S-2(창문 맵을 태민이 유지)는 역할 결정이라 태민 동의 필수.
- 함께 처리: 태민 `/window_positions` 필드명 → spec §6.2 정렬(~20줄), 복원단 안전장치(`overrides/` 반영 여부).
- 근거: [state_window_interface_spec_v0_1.md](state_window_interface_spec_v0_1.md)

### 2-7. A5 — 담당 2건 + RL의 위치
- RL 경로계획 실무·보상 설계 담당 확정(슬라이드와 문서 정합). 고전 계획기 v3가 기준선이 됐으므로 **RL은 "개선 트랙"** — 최종보고서에서 RL을 어디까지 주장할지(대리 환경 PyBullet 결과로 충분한가)도 같이 정할 것.

### 2-8. 제어 접점 (성진 ↔ 길남·윤호·태민)
- 길남: 계획기가 `capability.json`(시계 배율)을 읽도록 연동 — 성진 08-23 미완 항목, 착수 시점 확정
- 누가: `measAgeS`(측정 나이) 생산자 — 시뮬 측(윤호/Gazebo) 또는 VIO 측(태민)
- 윤호: 로터 인덱스→기하 + CW/CCW 매핑 확정(성진 C++ 믹서표 대조가 이것 때문에 보류), Isaac/Gazebo 궤적 JSON 스키마 서명
- 성진 보고: 0kg 앵커 sA 0.40 vs 0.50(외란 강건성 trade-off), limits 80% 승인 여부, rpm/rad/s 단위 주의 공지

### 2-9. 저장소 운영 규칙 명문화
- 08-19~23 "남의 폴더 침범 → 회수" 사건 후속. 제안: ① 자기 폴더 밖 수정은 PR 또는 총괄 경유 ② 공용 문서(README·NEWS·spec) 편집은 총괄 ③ 서브모듈 포인터는 해당 파트만 갱신 ④ 개인용 ignore는 `.git/info/exclude`. 이의 없으면 CLAUDE.md/README에 4줄 추가.

### 2-10. Novelty 방향 + 최종보고서 골격 (12/12)
- 후보(저장소에 실측이 있는 것): 성진 지연 강건화·외란 연동 조속기 / 창문 시각 정의와 색→순서 검출 파이프라인의 실패 모드 분석 / 노이즈 전파(픽셀→3D→통과 여유) 정량화. 이번 회의에선 "다음 회의까지 각자 1개 제출"로만 정해도 됨.

## 3. 팔로업 (결정 불요)

- 윤호: `state_window_adapter.py` normal 부호 플립(08-11 요청, 미반영 확인), CPU 렌더러 조명 키 수정 여부, 2차 데이터셋 생성기에 ①②⑤ 반영 여부
- 태민: 근황 확인(07-04 이후 기록 없음). 액션 5건 — `noisy_stream_x1/` 교차검증 회신 / `/window_positions` 필드명 → spec §6.2 / **T_IC: 08-11의 "body≡camera(항등)" 지시는 폐기 — 08-18 잠정 확정 ⑥(루트 spec §6.1)대로 노드의 EuRoC 하드코딩 값을 표준 R_IC=[[0,0,1],[−1,0,0],[0,−1,0]]로 교체, 이 상수가 단일 진실** / §5 center 4점 평균 / 검출 주기 2Hz→상향(scan rate 0.75→1.0 회복)
- 성진: MATLAB 검증 큐(비상 ①②), Gazebo 첫 월드 구동 시점, PX4 전환 트리거(07-19 구상) 재확인
- 전원: 서브모듈 워크트리 비었으면 `git submodule update --force`(Simscape 제외 — NEWS 하단), 9월 연구노트(9/30) 정리 담당, 회의록 md화 여부(3월 이후 hwp 회의록 없음)

## 4. 일정

| 항목 | 날짜 |
|---|---|
| 9월 연구노트 제출 | 2026-09-30 |
| 예산 잔액 10% 규칙 시작 / 장비 구매 마감 | 2026-11 / 과제 종료 2개월 전 |
| 최종보고서 제출 | 2026-12-12(금) |
| 최종평가 결과 확인 | 2026-12-21~23 |

*2026-08-29 작성. 회의 후 결정 사항은 이 문서 하단에 "결정 기록" 절로 추가하고 NEWS.md에 한 줄 요약.*
