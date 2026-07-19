# 세션 상황판 (SESSIONS_BOARD)

여러 클로드 세션이 병행 작업할 때의 **단일 보고 창구** (사용자 요청 2026-07-16:
"이런 보고체계 있어가지고 쭉 이어서 진행할 수 있었으면 좋겠다").

## 규칙
- 각 세션은 **자기 섹션만** 갱신 (한 줄 = 사건 1건, 최신이 위, `날짜 시각 — 내용 (커밋)`)
- 다른 세션에 넘길 것/받을 것은 ★로 표시 — 상대 세션은 소비 후 ★를 지우고 자기 줄로 기록
- 상세는 각자 문서(TUNING_STATUS/PIPELINE_STATUS/README)에, 여기는 헤드라인만
- MATLAB 사용 전 여기서 점유 여부 확인 후 자기 줄로 선언 (1대 규칙, RAM 16GB)

## MATLAB 점유
- **path_time 세션(자율)** — 07-19 밤: 큐 순차 2건 — ①교정 v2(diagnose_swing_calib2, ~8분) ②yaw 실비행 2편(verify_pipeline --only scan,look_at, ~8분). 끝나면 이 줄 비움. 비상 세션 검증 ①은 그 다음 순서.

## 튜닝/C++ 세션 (17차 계열)
- 07-19 — **agile 최종 채택: 삼각 법칙 + z분리** (9026688) — kp_xy=24-16|m-1| (양끝 precision 수렴), z는 8/3.2 고정(벡터 게인+posErrSatZ). 삼각 경사 1.5/1.75/2kg 합격(1.91~3.96cm, z꼬리<1cm — 2kg 한계사이클 소멸), 최종 관문(채택 경로) 외란 1.55도/0.44s + 1kg 이동 1.28cm 합격. C++ qc_apply_profile 동기(precision 골든 비트 동일). MATLAB 해제. ★전 세션: `run_traj_baked.m`에 `qc_zsplit_apply(mdl)` 호출 추가됨 — 직접 load_system 하는 스크립트로 agile 돌릴 땐 이 헬퍼 호출 필수 (비-agile은 무해·선택). ★path_time/RL: agile 유효구간 0.5~2kg, 0.5kg 미만은 precision 권장 (혼돈 구간 실측)
- 07-19 — smoother 백포팅 재검증 **합격** (diagnose_smoother: 발산 궤적 적발/정상 무개입 0.0000m/성형 후 안정 비행 |x| 1.08m·자세 13.2도·모터 81% 무포화) — path_time ★큐 소비 완료
- 07-19 — agile 1차식 A/B/C 실측 (8734bf1): A(외삽) 1.5kg부터 발산 — 위치 절벽 33 불변. B(1kg캡) 수평 전 질량 해결, z피크 잔존. C(z분리, 사용자 "z가 문제") 이동 중 z 해결(42→1.3cm), 2kg 85cm는 **정착 후 한계사이클**로 판명. 0~0.25kg는 혼돈 구간(재현성 없음). 삼각 법칙(kp_xy=24-16|m-1|, 2kg=precision 수렴) 경사 검증 중
- 07-19 — **agile 0kg 격자 전멸 (9/9)** — 위치 24/10.8 고정 시 sA/sZ 어떤 조합도 0kg 생존 불가 (최선 54cm/오버 483cm) → **범인 = 위치 kp 자체** (z축 결합: 위치 kp가 z 위치 오차에도 걸림). 사용자 확정 "위치 PID도 똑같이 1차식" — kp(m)=8+16m, kd(m)=3.2+7.6m (0kg=precision 앵커와 일치=실측 생존점, 1kg=agile 24/10.8). A(2kg 외삽 kp40)/B(위치 1kg 캡) 8구성 검증 실행 중
- 07-19 — **agile 관문 판정: 외란 합격 / 질량 불합격** — 외란 펄스 이탈 1.65도·회복 0.44s(여유 큼), 1kg 이동 1.32cm(r6 재현). 그러나 **0.5kg 완전 발산**(추종 193cm/자세 48도) + 2kg z피크 85cm(위치 kp가 z축에도 걸려 고도 과출력). agile은 1kg 전용임이 실증 — 처분(제거/1kg 제한/agile-lite 교체)은 사용자 결정 대기 (verify_agile_gates.csv)
- 07-19 — **질량 1차식 게인 법칙 채택 완료** (사용자 지시 이행): 0kg 앵커 sA=0.75/sZ=0.56 (r2 재현 확인) → 6질량 검증 합격(0~2kg 무발산, 1kg 회귀 무결 4.08cm, 0.5 내삽 비열등, 2kg 외삽 우세) → parameters.m `sA_mass/sZ_mass` 반영 + C++ `qc_scales` 동기(1kg 골든 재생 비트 동일). yaw는 질량 동결. 약점: 0~0.25kg 전이 20~29cm(안정 유지). 상세 TUNING_STATUS §Y 18차
- 07-19 — **0kg 앵커 r1 완료** (9점 격자): 승자 sA=0.75/sZ=0.56 (추종 21.4cm/오버 37cm, 유일 무발산). 예측 뒤집힘 — 자세도 25% 감쇠가 최선 (동결 sA=1.0은 오버 759~4294cm 발산). sZ 하단 모서리라 r2 (sA 0.65~0.85 × sZ 0.40~0.56) 실행 중 (~14분). 이후: 1차식 6점 검증 → parameters.m 반영
- 07-19 — 질량 탐침 완료: 붕괴 경계 0.3~0.1kg. xy 붕괴=자세 게인 과소(sIa≤0.61), z 붕괴=고도 게인 과대(sM=1.0, z피크 83~84cm) — 채널 분리 확인, 0kg 앵커 설계 근거 (refine_mass_probe.csv)
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
- 07-19 저녁 — **yunho/sim-rl-scaffold 병합** (ac29e20, 사용자 지시) — reinforcement_yunho/ 신규 49파일(+8387: Isaac Sim 씬/데이터셋/RL 환경 스캐폴드), 타 폴더 무접촉·무충돌. + yaw 검증 미션 2종 등록(정적 통과) + fly-through 촘촘 병리 조사(세그당 4s 진범 추적 — 수정 2건 커밋, 저크 폭주 미해명분은 PIPELINE_STATUS에 승계 기록). 세션 마감
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
- 07-19 — ★튜닝/C++ 세션: 비상 협의 의제 3건 — ① 컨트롤러 측 하트비트 감시(철칙 3: flight_state.json written_at 나이>1.0s → 현행 궤적 완주 후 래치 호버. 기준 구현 `flight_supervisor.heartbeat_stale()`, C++ 미션 러너/qc_io에 이식 요청) ② B RECOVER 상태기계 C++ 이식(골든 확장 후 — 트리거 후보 EMERGENCY_STATUS 참조) ③ C-반사 믹서 자세 우선 배분(qc_controller.hpp 수정이라 골든 재대조 필수, 믹서 부호표 §8 기반. w_sat 임계 실측은 qc_trace Ct 0.8 열화로 비상 세션이 수행 예정)
- 07-19 — **A-2 금지 구역 + 감독자 v0.2 완료 (사용자 부재 자율 모드)** — keep_out 이격 검사(box/sphere, 전 샘플)·게이트 연동(plan/splice/emergency, KEEP_OUT_VIOLATION)·회피 재계획(재조밀화 push-out — 정관통 퇴화 2건 실측 해결)·감독자 러너(action→§8 CLI subprocess 실전 왕복)·C-모드 트리거 감시(옵트인, w_sat 실측 대기)·검증 ①② 오케스트레이터 `verify_emergency.py` 작성(미실행, MATLAB 슬롯 대기). §8 표에 emergency 동사 등재 + §9 A-2 구현 확정 추기. **테스트 169개 전체 통과** (신규 40, 회귀 0). 상세 EMERGENCY_STATUS.md
- 07-19 — **A-1 비상 정지 구현 완료** (`traj_emergency.py` + §8 `emergency` 동사 + 테스트 19개, 전체 145개 통과·회귀 0) — 실측 상태 저크 제한 최단 정지: v0=1.5에서 정지 거리 0.819m(2단 정확식 0.821 대비 -0.3%), 게이트 풀한계 통과(jPk 9.0), 비상 레짐(ZVD 생략/마진 반납/snap 측정만), STATE_STALE 거부. 잔여: MATLAB 검증 ① (대기/예약 등록)
- 07-19 — **감독자 골격 v0.1 완료** (`flight_supervisor.py` + 테스트 24개, 전체 스위트 106개 통과) — flight_state.json 단일 소유·하트비트, 미션 게이트(REJECTED_RECOVERING), 우선순위 중재(B>C>A-1>A-2, 하위는 유예), A-1/A-2 소비, B hash 무효 선언(원장), `heartbeat_stale()` 철칙 3 기준 구현. B/C 트리거 임계 후보(측정지연 0.05s 보정 포함)는 EMERGENCY_STATUS.md에 정리. 다음: A-1 emergency 동사(MATLAB 불필요분) → 검증 ①은 MATLAB 큐 대기
- 07-19 — **세션 착수** — 필독 5종(HANDOFF_EMERGENCY/§9/§8 실측/PIPELINE_STATUS/HANDOFF_CPP_GAZEBO) 정독, ★소비 2건. 임무: 감독자 + 비상 A-1/A-2/B/C + MATLAB 검증 4편. 1단계(감독자 골격, 파이썬 프로토타입) 착수 — MATLAB 불필요(튜닝 세션 점유 확인, 검증 비행은 큐 대기 예정)

## Gazebo/C++ 검증 세션
- (미착수 — 착수 시 docs/HANDOFF_CPP_GAZEBO.md 필독, 여기 첫 줄 기록)

## 대기/예약 (세션 무관)
- [MATLAB] 튜닝 세션 잔여: agile 전용 1차식 (0kg 격자 → 6질량 재검증 → 관문 재실행) — 실행 중. 완료분: 골든 로그·smoother 재검증·질량 탐침·표준 1차식·agile 관문 전부 07-19 소화
- [MATLAB] path_time 세션: 2호기 교정 v2 **공진 체류 가진** (`diagnose/diagnose_swing_calib2.m`, 976864a — 스크립트 완성, f0 스윕 3점 ~8분). 방법론 변경: 외력 주입이 아니라 궤적 가진 (2호기 액추에이터 = 드론 가속이라 교정도 같은 경로). 슬롯 나면 실행 → swing_calib.json
- [MATLAB] path_time 세션: yaw 실비행 1편 (scan coupled 미션 — 컨트롤러 yaw 추종 + real_yaw 실측 + command_fidelity 왕복 검증, ~4분). 교정 v2 다음 순위
- [MATLAB] 비상 세션: 검증 ①·② (`python verify_emergency.py` — A-1 정지 오버슈트/드리프트 + A-2 회피 실측 이격, MATLAB 3회 ~20분, 타 MATLAB 가드 내장) — 슬롯 해제 시 실행. 첫 실행이라 로그 전체 확인 필요
- [사용자] 푸시 2줄 (서브모듈 → 부모 순서)
- [Docker] qc-cpp 컨테이너 빌드 (Desktop 기동 시)
