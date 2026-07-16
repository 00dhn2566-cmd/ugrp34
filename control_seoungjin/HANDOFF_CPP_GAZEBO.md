# HANDOFF: C++ 제어기 → Gazebo 검증 세션

작성: 17차 튜닝 세션 (2026-07-16). 대상: Gazebo 검증을 맡은 클로드 세션.
전제 문서: [INTERFACE_SPEC.md](INTERFACE_SPEC.md) (통신 규격 v0.1),
[controller_cpp/README.md](controller_cpp/README.md) (이식 상태),
[gazebo_setup_log.md](../gazebo_setup_log.md) (환경 구축 이력).

## 임무

`controller_cpp/`의 C++ 제어기를 Gazebo 플랜트에 붙여 검증한다.
MATLAB 구운 모델(Simscape)은 "정답 플랜트"로 계속 남고, Gazebo는 **독립 물리로
교차 검증**하는 역할 — 두 플랜트에서 같은 궤적이 같은 성적이면 제어기가
플랜트-독립적으로 옳다는 증거가 된다.

## 지금 상태 (믿어도 되는 것 / 아직 아닌 것)

**검증 완료 (믿어도 됨):**
- 제어 체인 구조·게인·클램프 7종 (`qc_controller.hpp/cpp`) — 산수 검증: 물성 스케일
  5종=1.000000 (MATLAB parameters.m과 비트단위 일치), posErrSat 곱 불변식 16.8° 재현
- 파일 인터페이스 3종 (`qc_io`) — 실파일 왕복 + 파이썬 파이프라인 교차 파싱 통과
- 모터 기준속도 스케일: motorRef = mixDir × 2π×(고도PID+56.5+44.4·m_pkg) —
  실비행 재생에서 실측 모터속도와 스케일비 0.98~1.02 확인 (호버 평형 ~634 rad/s)
- 모터 방향 규약: **모터 2·3 내장 역회전** (실측 w 음수) — `mixDir={+1,-1,-1,+1}` 반영됨

**미확정 ([TODO-verify] — 코드 주석에 표시, 골든 대조 전까지 의심할 것):**
- 믹서 차동 부호표 (mixPitch/mixRoll/mixYaw) — 재생 상관이 낮은 원인
- 측정 필터 계수·배선 위치 (Filter Pitch TransferFcn ↔ filtM_attitude 0.01 가정)
- 고도 PID 클램프: parameters.m `limit_altitude=10` vs 9차 기록 ".slx ±30 rev/s" 관계
- RBI 회전 완전성 (현재 yaw만 반영한 1차 근사) / Dir P/R 부호
- 확정 수단: `diagnose/dump_controller_spec.m` (작성돼 있음, MATLAB 차례 오면 실행)
  + 완전 골든 대조 (아래 §검증 절차)

## 절대 규칙 (튜닝 세션 피의 교훈 — 위반 시 재현 불가 버그)

1. **자세 게인 음수는 의도** (플랜트 이득 음수 b=−0.0296). "부호 수정" 금지 —
   고치면 즉시 발산.
2. **앵커 불변**: QcConfig의 `*Ref` 값들(kThrustRef 9.79, kDragRef 0.597,
   droneMassRef 1.2726, pkgSizeRef 0.14³)은 튜닝 당시 값 — 갱신 금지.
3. **입력 계약**: 이 제어기는 스무더+게이트 통과 궤적 전제. 날것 스텝을 주면
   원본도 릴레이 한계사이클로 왕복한다 (구조 한계, 게인 문제 아님). Gazebo에서
   스텝 응답으로 "불안정하다" 판정하지 말 것 — 반드시 `output/trajectory.json`
   경유 (INTERFACE_SPEC §1~2 파이프라인 산출물).
4. **anti-windup 없음 유지** — 원본과 동일해야 골든 대조가 성립. 개선은 검증 후.
5. **qc_phys 동기화**: `quadcopter_package_parameters.m`의 qc_phys와
   `qc_controller.hpp`의 qc_phys는 1:1 사본 — 한쪽 바꾸면 양쪽 갱신.
6. **게인 수치는 parameters.m이 원본**: 위치 게인이 곧 확정·변경된다
   (현행 8/3.2 → 절벽 33 확인, 22~26 영역 채택 임박). QcConfig 숫자를
   parameters.m 최신과 대조 후 사용할 것.

## Gazebo 쪽 구성 유의

- **플랜트 물성 일치**: Gazebo 모델의 질량/관성/추력계수를 QcConfig(=qc_phys 합성:
  m_tot 2.2726kg, I_att 1.713e-2, I_yaw 2.124e-2 kg·m², 프롭 Kthrust 9.79/Kdrag 0.597
  계열)와 맞출 것. 게인만 스케일되고 플랜트가 다르면 미스매치 (INTERFACE_SPEC §6
  가드레일 2 — 시뮬 플랜트 일관성).
- 모터 2·3 역회전 규약 반영 (X쿼드, r_arm=0.159m, 휠베이스 450mm).
- 제어 주기: 고정스텝 1kHz 권장 (골든 대조 기준). PID 이산화는 후방차분 —
  가변 스텝 금지.
- 현수 짐: 1kg 패키지가 **용접(강체)** — 진자 조인트가 아님. 저중심 강체 모드
  1.8Hz가 정상 (질량 불변, ω²=g/L, L=8.1cm). Gazebo에서 조인트로 만들면 다른
  물리가 된다.
- 이 개발 머신(MX450)은 Gazebo 못 돌림 — RTX 5060 머신 또는 Paperspace Core
  (cloud_gpu_ssh_setup.md 참조).

## 빌드/실행

```powershell
cd control_seoungjin\controller_cpp
.\build.ps1            # msys64 g++ — mingw64\bin PATH 필수 (없으면 무음 실패 0xC0000135)
.\qc_trace.exe --smoke                                   # 산수 정상 확인
.\qc_trace.exe --io-test ..\output\trajectory.json <임시폴더>   # 인터페이스 왕복
```

## 검증 절차 (골든 트레이스)

1. 입력 추출: `python make_golden_input.py <sim_result_baked.mat> input.csv`
   (act/des/rpy/모터속도 → C++ 입력 CSV)
2. 재생: `qc_trace.exe input.csv cpp_out.csv`
3. 개연성: `make_golden_input.py ... --check cpp_out.csv` — motorRef↔실측 w
   상관/스케일비 (현재: 스케일비 4모터 +0.98~1.02 통과, 차동 상관은 믹서 확정 대기)
4. 완전 대조 (cmd 탭 로그 확보 후): `python compare_trace.py golden.csv cpp_out.csv`
   — 합격선: 채널 RMS ≤ 풀스케일 2%, 상관 ≥ 0.99. 불합격 시 "첫이탈 시각/채널"로
   어느 블록에서 갈라졌는지 국소화.
5. Gazebo 폐루프: 같은 trajectory.json으로 비행 → attitude_feedback.json 생산
   (qc_io가 이미 씀) → MATLAB 성적표(§W/§X: 1m 이동 추종 2.7cm RMS, 호버 지터
   0.002°, 토크 펄스 회복 0.73s)와 나란히 비교.

## 세션 조율

- **MATLAB은 전 세션 공용 1대** (RAM 16GB, 동시 2개 금지) — 필요하면 사용자에게
  차례 요청. 현재 대기열: 위치 튜닝 마무리(이 세션) → path_time 세션.
- git: 서브모듈 푸시는 사용자 손으로 (클로드 권한 차단). 산출물은
  `control_seoungjin/` 바로 밑, `sample/`은 사용자 개인 폴더 — 건드리지 말 것.
- 진행 기록은 이 파일에 덧붙이지 말고 별도 상태 md (예: `GAZEBO_STATUS.md`) 생성.
