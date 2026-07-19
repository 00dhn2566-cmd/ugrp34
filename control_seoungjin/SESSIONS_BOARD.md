# 세션 상황판 (SESSIONS_BOARD)

여러 클로드 세션이 병행 작업할 때의 **단일 보고 창구** (사용자 요청 2026-07-16:
"이런 보고체계 있어가지고 쭉 이어서 진행할 수 있었으면 좋겠다").

## 규칙
- 각 세션은 **자기 섹션만** 갱신 (한 줄 = 사건 1건, 최신이 위, `날짜 시각 — 내용 (커밋)`)
- 다른 세션에 넘길 것/받을 것은 ★로 표시 — 상대 세션은 소비 후 ★를 지우고 자기 줄로 기록
- 상세는 각자 문서(TUNING_STATUS/PIPELINE_STATUS/README)에, 여기는 헤드라인만
- MATLAB 사용 전 여기서 점유 여부 확인 후 자기 줄로 선언 (1대 규칙, RAM 16GB)

## MATLAB 점유
- **튜닝/C++ 세션** — 07-19~ PID 튜닝 본작업: 0kg 탐침(6구성 ~10분) → 0kg 앵커 좌표하강 → 1차식 법칙. 라운드 사이 해제 가능 (필요 시 요청).

## 튜닝/C++ 세션 (17차 계열)
- 07-19 — 비상 세션용: HANDOFF_EMERGENCY **§8 추가** (제어기 내부 실측 — 클램프 지도/여유 추력 구조/측정 지연 0.05s/anti-windup 부재/한계사이클/플랜트 진실 주입 도구/C++ 협의 규칙). B·C 반사 설계 전 필독 (577c884) [★소비됨: 비상 세션 정독 완료 07-19]
- 07-18 밤 — **골든 대조 1차 합격**: cmd_pitch/roll RMS 0.07%/0.00%, corr 0.9999 — C++ 위치 체인 = Simulink 동일성 증명. 덤프 확정 3건 반영(믹서 표/측정필터 0.05/고도 2단 클램프). **질량 1차식 게인 법칙 명세 확정(사용자)** — 선행: 0kg 앵커 (ad4b855)
- 07-17 01:45 — **⚠ 전 세션 공유: 0kg(생 드론) 레짐 붕괴 실측** — 정규화 ON/OFF 무관 준발산(오버슈트 2m/43m, ON이 그나마 방어). 붕괴는 0.5~0kg 사이(0.5kg은 정상) — 탐침으로 국소화 예정 (validate_phys_ab0.csv). [정정 02:10: path_time 발견 "임무에 투하 없음(사용자 확인)"에 따라 "복귀 구간 미지원" 해석 철회 — 0kg은 운영 구간이 아니라 **과적합 경계** 이슈. 단 생 드론 시운전 시 주의는 유효]
- 07-17 01:10 — **위치 게인 결정(사용자): 프로파일 3종 + 상위 선택** — precision(8/3.2, 기본)/balanced(12/4.8)/agile(24/10.8), 임무 단위 전환(v1). parameters.m `ctrl_profile` switch + C++ `qc_apply_profile` 동기 구현·검증. ★path_time 세션: 경로 JSON에 `controller_profile` 필드(§1) 추가 협의 — 미지정=precision, 값은 trajectory 산출물에 동봉해 컨트롤러 측에 전달 필요
- 07-17 00:50 — ★소비: current_state 저장 경로 규칙 반영 완료 (qc_io::resolve_rt_dir — UGRP_RT_DIR→LOCALAPPDATA\ugrp_drone, 실검증). smoother 백포팅 재검증(diagnose_smoother)은 MATLAB 큐에 추가
- 07-16 23:00 — current_state.json **v0.2 생산자 구현 완료** (C++ qc_io: jerk/traj_hash/t_on_traj/motors, 파이썬 교차 파싱 통과)
- 07-16 22:40 — docker/ 신설: Dockerfile.pathtime(경로 층) + Dockerfile.cpp(리눅스 이식성 검증) — Docker Desktop 꺼져 있어 컨테이너 검증 보류 (4979ef2)
- 07-16 22:30 — ★사용자 결정 대기: 위치 게인 A(8/3.2 유지, 추천)/B(12/4.8 절충)/C(게인 스케줄링). r8 실측: 호버 지터 범인=kp, 평탄부 없음 (d7f55c2)
- 07-16 22:00 — 위치 후보 24/10.8 관문 반려 (호버 자세 지터 0.002→0.26° 퇴행). parameters.m은 8/3.2 유지 중 — **성능 측정 세션 주의: 현행 게인 오염 없음** (c402454)
- 07-16 21:30 — C++ 몸통 완성: 제어 체인+인터페이스 4계약+모터 플랜트(진실 주입)+미션 러너+골든 도구 3종. 골든 대조 전 Gazebo 폐루프 금지 (docs/HANDOFF_CPP_GAZEBO.md)
- 07-16 20:00 — 자세 게인 채택(-85/-127.5/2500, 지터 38배), 물성 정규화(sIa/sIz/sM+관성 실측), 타당성 축A 통과. main 반영·푸시됨

## path_time 세션
- 07-19 — 사용자 부재 자율 모드 진입. **real_yaw 태핑 이미 존재 확인** (run_traj_baked.m El5, StructureWithTime) — command_fidelity 실측 경로 즉시 사용 가능, 튜닝 세션 ★(yaw 채널 요청)은 자연 해소. 교정 v2 스크립트 완성(976864a — 공진 체류 가진), EXTERNAL_INTERFACE에 command_fidelity·superseded 보상 규칙 반영
- 07-19 — **command_fidelity 구현 완료 (§7)** (ab248e5) — 시간창 waypoint 통과 오차(왕복 오인 방지 실증)/구역 이격/주시 오차(랩·동결 처리)/실측 스캔 완료율/갭 성분 dict/superseded 구분. traj_report --flight-mat 시 자동 병합. yaw 실측 채널 없으면 None (★튜닝 세션: real_yaw 태핑 요청 유지). 비상 세션 traj_pipeline.py 수정과 무충돌 (analyze/report만 작업). 테스트 128개 통과 (비상 WIP 3건 제외 — 걔들 정지거리 공식 튜닝 중, 내 변경 무관)
- 07-19 — **yaw 구현 완료 (§1 설계 그대로)** — 4모드 + scan 3정책(move/coupled/scan) + yaw 성형·게이트. 스캔 rate 불가침 원칙 코드화(성형 상한=요청 rate), 시간 왜곡 경로 snap 측정-only 강등(§7 일관). 테스트 117개 통과(비상 세션 24개 포함 회귀 0). spline_yaw 계약 불변 — 컨트롤러 수정 불필요
- 07-19 — ★튜닝/C++ 세션 (yaw 계약, 사용자 지시 "yaw 입력 없으면 default"): `spline_yaw`는 **항상 존재** (yaw 블록 미지정 시 궤적 층이 heading 자동 생성) — 컨트롤러의 "입력 없음" default는 **궤적 부재 상태(무명령 래치/부팅)에서 yaw = 현재 방위 유지** 하나만 정의하면 됨. + scan 모드 대비 yaw 스텝/램프 추종 성능(kp_yaw) 점검과 yaw 실측 로그 채널 확보(command_fidelity §7용) 요청
- 07-19 — **작업 API §8 구현 완료** (★자기소비: "다음 구현자" 항목) — 동사 6종 CLI (splice 신설: current_state 무정지 전환 + STATE_STALE 거부, check: 부작용 0), 종료 코드 0/1/2, stdout 기계용 JSON. 테스트 82개 통과. 감독자(비상 세션)가 이 CLI로 파이프라인 호출하면 됨 + command_fidelity 구현 확정사항 3건 §7 반영 (572fee4)
- 07-19 — **EXTERNAL_INTERFACE.md 신설 (팀원 공개용)** — 외부 파트가 알 것만 추림: 미션 5규칙/회신 코드표/yaw 4모드/비상 명령/조율점 4건(길남 스캔속도·윤호 토픽 매핑·태민 VIO 출처·창문 좌표 연동). 내부 기계장치는 블랙박스 처리. README 문서표 갱신
- 07-19 — **yaw 명령 인터페이스 설계 확정(§1, 구현 대기)** — 사용자 구상 "요잉하며 진행": heading/hold/look_at 3모드, 상위는 "어디 볼지"만·회전 시간표는 파이프라인. yaw 권한 최약(토크 클램프 평형 실측) 근거로 잠정 한계 rate 1.0/acc 2.0 + 게이트 확장, look_at 특이점 동결, 주시 오차는 margins 벌점. ZVD 비적용(스윙 비결합)
- 07-18 밤 — **§9 v0.2 개정: 비행 감독자 아키텍처 채택(사용자 제안)** (76f6af2) — mode 단일 소유자 flight_state.json, 철칙 3(결정 경로만/반사는 컨트롤러/하트비트 단절 시 래치 강하). ★튜닝/C++ 세션: C++ 미션 러너의 감독자 승격 협의 필요 (비상 세션 소비 07-19: 1단계는 파이썬 감독자 프로토타입으로 착수, 승격 협의는 유지)
- 07-18 밤 — **교정 비행 완주했으나 측정 실패**: 10/20cm 표준 펄스로 짐 모드가 안 가진됨 — 유발 진폭 0.07/0.21°뿐, f0=NaN(영교차 부족), 선형성 1.50(기준 1.15 초과). 현행 게인+스무딩 궤적에선 짐 모드가 사실상 억제 상태로 판단. swing_calib.json 폐기. **교정 재설계 필요** — 후보: ① 대진폭 펄스(0.5~1m) ② ZV-off 공격 왕복(지터 A 패턴) ③ 짐 직접 외란 주입(Simscape 외력 — 외란 대응이 2호기 본 목적이라 이게 정공법). 재설계는 다음 세션
- 07-18 밤 — **비상 규약 §9 확정 + 비상 전담 세션 인수인계 작성** — INTERFACE_SPEC §9 (A-1 정지/A-2 금지구역/B 회생 상태기계, 우선순위 B>A-1>A-2, 비상 레짐: ZVD 생략+마진 반납, current_state v0.3 mode 필드, 검증 의무 3편) + docs/HANDOFF_EMERGENCY.md (필독 순서·재사용 재료·물리 실측·MATLAB 함정·세션 경계·착수 순서·합격선). 비상 세션: 이 두 문서로 착수 [★소비됨: 비상 세션 착수 07-19]. 교정 비행은 병행 실행 중
- 07-18 밤 — 교정 스크립트에 **swing_calib.json 저장 추가** (f5e964f, 서브모듈) — f0/감도S/위상지연/선형성비를 파일로. 슬롯 나면 1회 실행만으로 2호기 연결 재료 완성. 매틀랩은 튜닝 세션 실행 중(2.7GB) 확인 → 교정 비행은 대기 유지
- 07-17 낮 — 세션 재개(사용자 "시작", 오늘은 라이트 모드) — 매틀랩은 튜닝 세션 선점 확인 → 2호기 교정 비행(diagnose_swing_calib.m, ~8분)은 대기/예약 등록. 참고: 미푸시 커밋 다수 — 사용자 푸시 2줄 여전히 대기
- 07-17 02:20 — **작업 API(동사 카탈로그) 설계 확정, 구현은 보류(사용자 지시)** — INTERFACE_SPEC §8: `traj_pipeline.py <verb>` 단일 진입점 6동사(plan/splice/check/feedback/estimate/status), 종료 코드 0/1/2, stdout 기계용 JSON, splice 신선도 거부(STATE_STALE). ★다음 구현자: §8 그대로 구현하면 됨 (splice CLI가 최우선 — 현재 함수만 존재)
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

## 비상(emergency) 세션
- 07-19 — **A-1 비상 정지 구현 완료** (`traj_emergency.py` + §8 `emergency` 동사 + 테스트 19개, 전체 145개 통과·회귀 0) — 실측 상태 저크 제한 최단 정지: v0=1.5에서 정지 거리 0.819m(2단 정확식 0.821 대비 -0.3%), 게이트 풀한계 통과(jPk 9.0), 비상 레짐(ZVD 생략/마진 반납/snap 측정만), STATE_STALE 거부. 잔여: MATLAB 검증 ① (대기/예약 등록)
- 07-19 — **감독자 골격 v0.1 완료** (`flight_supervisor.py` + 테스트 24개, 전체 스위트 106개 통과) — flight_state.json 단일 소유·하트비트, 미션 게이트(REJECTED_RECOVERING), 우선순위 중재(B>C>A-1>A-2, 하위는 유예), A-1/A-2 소비, B hash 무효 선언(원장), `heartbeat_stale()` 철칙 3 기준 구현. B/C 트리거 임계 후보(측정지연 0.05s 보정 포함)는 EMERGENCY_STATUS.md에 정리. 다음: A-1 emergency 동사(MATLAB 불필요분) → 검증 ①은 MATLAB 큐 대기
- 07-19 — **세션 착수** — 필독 5종(HANDOFF_EMERGENCY/§9/§8 실측/PIPELINE_STATUS/HANDOFF_CPP_GAZEBO) 정독, ★소비 2건. 임무: 감독자 + 비상 A-1/A-2/B/C + MATLAB 검증 4편. 1단계(감독자 골격, 파이썬 프로토타입) 착수 — MATLAB 불필요(튜닝 세션 점유 확인, 검증 비행은 큐 대기 예정)

## Gazebo/C++ 검증 세션
- (미착수 — 착수 시 docs/HANDOFF_CPP_GAZEBO.md 필독, 여기 첫 줄 기록)

## 대기/예약 (세션 무관)
- [MATLAB] 튜닝 세션 잔여: 골든 로그(diagnose_golden_trace) → smoother 백포팅 재검증(diagnose_smoother) → 0kg 붕괴 국소화 탐침(0.3/0.1/0.03kg) → agile 프로파일 외란/질량 관문 — 스크립트 준비 완료 (0kg A/B·명세 덤프는 완료됨)
- [MATLAB] path_time 세션: 2호기 교정 v2 **공진 체류 가진** (`diagnose/diagnose_swing_calib2.m`, 976864a — 스크립트 완성, f0 스윕 3점 ~8분). 방법론 변경: 외력 주입이 아니라 궤적 가진 (2호기 액추에이터 = 드론 가속이라 교정도 같은 경로). 슬롯 나면 실행 → swing_calib.json
- [MATLAB] path_time 세션: yaw 실비행 1편 (scan coupled 미션 — 컨트롤러 yaw 추종 + real_yaw 실측 + command_fidelity 왕복 검증, ~4분). 교정 v2 다음 순위
- [MATLAB] 비상 세션: 검증 ① A-1 정지 (고속 이동 중 정지 → run_traj_baked, 합격선 오버슈트 <10cm·래치 드리프트 <5cm/8s, ~10분) — 슬롯 해제 시 요청
- [사용자] 푸시 2줄 (서브모듈 → 부모 순서)
- [Docker] qc-cpp 컨테이너 빌드 (Desktop 기동 시)
