# 세션 상황판 (SESSIONS_BOARD)

여러 클로드 세션이 병행 작업할 때의 **단일 보고 창구** (사용자 요청 2026-07-16:
"이런 보고체계 있어가지고 쭉 이어서 진행할 수 있었으면 좋겠다").

## 규칙
- 각 세션은 **자기 섹션만** 갱신 (한 줄 = 사건 1건, 최신이 위, `날짜 시각 — 내용 (커밋)`)
- 다른 세션에 넘길 것/받을 것은 ★로 표시 — 상대 세션은 소비 후 ★를 지우고 자기 줄로 기록
- 상세는 각자 문서(TUNING_STATUS/PIPELINE_STATUS/README)에, 여기는 헤드라인만
- MATLAB 사용 전 여기서 점유 여부 확인 후 자기 줄로 선언 (1대 규칙, RAM 16GB)

## MATLAB 점유
- (비어 있음) — 07-17 02:05 path_time 스텝 재비행 완료·해제. 다음: 튜닝 세션 잔여 큐 (대기/예약 참조).

## 튜닝/C++ 세션 (17차 계열)
- 07-17 01:45 — **⚠ 전 세션 공유: 0kg(생 드론) 레짐 붕괴 실측** — 정규화 ON/OFF 무관 준발산(오버슈트 2m/43m, ON이 그나마 방어). 1kg 튜닝 시스템이 0kg에서 무너짐 = **드롭 후 복귀 구간 미지원**. 붕괴는 0.5~0kg 사이(0.5kg은 정상) — 중간 질량 탐침으로 국소화 예정. 드롭/복귀 시나리오 테스트 금지 권고 (validate_phys_ab0.csv)
- 07-17 01:10 — **위치 게인 결정(사용자): 프로파일 3종 + 상위 선택** — precision(8/3.2, 기본)/balanced(12/4.8)/agile(24/10.8), 임무 단위 전환(v1). parameters.m `ctrl_profile` switch + C++ `qc_apply_profile` 동기 구현·검증. ★path_time 세션: 경로 JSON에 `controller_profile` 필드(§1) 추가 협의 — 미지정=precision, 값은 trajectory 산출물에 동봉해 컨트롤러 측에 전달 필요
- 07-17 00:50 — ★소비: current_state 저장 경로 규칙 반영 완료 (qc_io::resolve_rt_dir — UGRP_RT_DIR→LOCALAPPDATA\ugrp_drone, 실검증). smoother 백포팅 재검증(diagnose_smoother)은 MATLAB 큐에 추가
- 07-16 23:00 — current_state.json **v0.2 생산자 구현 완료** (C++ qc_io: jerk/traj_hash/t_on_traj/motors, 파이썬 교차 파싱 통과)
- 07-16 22:40 — docker/ 신설: Dockerfile.pathtime(경로 층) + Dockerfile.cpp(리눅스 이식성 검증) — Docker Desktop 꺼져 있어 컨테이너 검증 보류 (4979ef2)
- 07-16 22:30 — ★사용자 결정 대기: 위치 게인 A(8/3.2 유지, 추천)/B(12/4.8 절충)/C(게인 스케줄링). r8 실측: 호버 지터 범인=kp, 평탄부 없음 (d7f55c2)
- 07-16 22:00 — 위치 후보 24/10.8 관문 반려 (호버 자세 지터 0.002→0.26° 퇴행). parameters.m은 8/3.2 유지 중 — **성능 측정 세션 주의: 현행 게인 오염 없음** (c402454)
- 07-16 21:30 — C++ 몸통 완성: 제어 체인+인터페이스 4계약+모터 플랜트(진실 주입)+미션 러너+골든 도구 3종. 골든 대조 전 Gazebo 폐루프 금지 (docs/HANDOFF_CPP_GAZEBO.md)
- 07-16 20:00 — 자세 게인 채택(-85/-127.5/2500, 지터 38배), 물성 정규화(sIa/sIz/sM+관성 실측), 타당성 축A 통과. main 반영·푸시됨

## path_time 세션
- 07-17 02:05 — **스텝 백스톱 합격 → 매트릭스 v3 전 항목(6/6) 완결** — 추종 2.79cm / tail 0.018°, path_vis 서브미터 패치 유효 확인. 재시간화도 정상 작동(스텝 → S-커브, 팽창 x0.30, 경로 이탈 0.0cm). 세션 종료
- 07-17 02:00 — 슬롯 인수(튜닝 세션 해제 확인) → 스텝 백스톱 재비행 백그라운드 착수. 판독 기준: verification_matrix.json의 step 항목 ✅ + 추종 RMS cm급이면 매트릭스 v3 완결
- 07-17 01:55 — **세션 마감 정리**: ★소비: `controller_profile` 필드 반영 완료 — §1 스키마 + 산출물 3종(.mat/.json/meta) 동봉, 기본 precision, 테스트 74개 통과. README 모듈 지도/문서표 갱신(v0.2), PIPELINE_STATUS 매트릭스 v3 성적표 확정. 스텝 재비행은 미실행(01:40 착수분은 튜닝 세션 MATLAB 감지로 가드 중단) → 대기/예약 재등록
- 07-17 01:40 — MATLAB 슬롯 인수 시도 → 튜닝 세션 선점 감지로 가드 중단 (verify_pipeline 타 MATLAB 감지 정상 작동). 스텝 재비행은 큐 유지
- 07-17 01:05 — 스텝 실패 진범: `quadcopter_waypoints_to_path_vis.m`의 `floor(dist)*4` — **1m 미만 세그먼트 = 시각화 점 0개** → Spline 컴파일 거부. 최소 2점 가드 패치 (서브미터 경로 쓰는 모든 세션 해당). 매트릭스 v3: fly_through 다항식판 합격(추종 1.3cm/tail 0.017°), ZVD tail 8배 저감(0.12→0.015°), 질량 0.06% 재현
- 07-17 00:35 — current_state **저장 경로 규칙 확정** — 30Hz 파일은 OneDrive 밖 `env UGRP_RT_DIR → %LOCALAPPDATA%\ugrp_drone\` (repo output/은 sync 잠금으로 원자적 rename 실패 위험). INTERFACE_SPEC §5 [★소비됨: C++ 반영 완료 07-17 00:50]
- 07-17 00:30 — traj_smoother.m에 vmax 저크 스파이크 테이퍼 백포팅 (6f43567) — Python판 등가 검증 완료 [★소비됨: 튜닝 세션 MATLAB 큐에 diagnose_smoother 재검증 추가]
- 07-17 00:25 — 다항식 fly-through(통과 속도+구심 가속 BC, 중간점 정확 통과, 일직선 -30%) + 완화 계약 v0.2(클램프/재시간화, 거부 최소화) + 동적 지터 예산 + RDP 전처리. 테스트 72개
- 07-17 00:15 — current_state v0.2 스키마 확정(INTERFACE_SPEC §5) + 소비 측 스플라이스 jerk 승계 반영 (a636c2a). 매트릭스 v3(fly-through 5편+추정기) 백그라운드 비행 중
- (이하 이 세션이 직접 기록)

## Gazebo/C++ 검증 세션
- (미착수 — 착수 시 docs/HANDOFF_CPP_GAZEBO.md 필독, 여기 첫 줄 기록)

## 대기/예약 (세션 무관)
- [MATLAB] 튜닝 세션 잔여: 골든 로그(diagnose_golden_trace) → smoother 백포팅 재검증(diagnose_smoother) → 0kg 붕괴 국소화 탐침(0.3/0.1/0.03kg) → agile 프로파일 외란/질량 관문 — 스크립트 준비 완료 (0kg A/B·명세 덤프는 완료됨)
- [사용자] 푸시 2줄 (서브모듈 → 부모 순서)
- [Docker] qc-cpp 컨테이너 빌드 (Desktop 기동 시)
