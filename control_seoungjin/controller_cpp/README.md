# controller_cpp — Simulink 제어기의 C++ 이식 (17차 착수)

구운 Simscape 모델(`quadcopter_package_delivery.slx`)의 Maneuver Controller를
독립 실행 C++로 떼어내는 작업. 실기/Isaac Sim 연동용 — MATLAB 없이 도는 제어기.

## 상태 (2026-07-16)

| 단계 | 상태 |
|---|---|
| 1. 경계 확정 (참조+센서 → 모터 명령 4개) | 완료 |
| 2. 블록 명세 자동 덤프 (`dump_controller_spec.m`) | 예정 — MATLAB 한가할 때 |
| 3. C++ 뼈대 (PID/필터/체인/물성 정규화) | **빌드+스모크 통과** |
| 4. 골든 트레이스 대조 (Simulink=정답) | 예정 — [TODO-verify] 확정 후 |
| 5. 경로 층 (smoother/zv 등) | **범위 제외** — 나중에 도커 기반으로 별도 작성 (사용자 결정 17차) |

스모크에서 확인된 것: qc_phys 물성 합성이 MATLAB과 일치(스케일 5종=1.0),
posErrSat 곱 불변식(최대 명령 기울기 16.8°) 재현.

## 인터페이스 계약 (INTERFACE_SPEC v0.1 대응 — qc_io)

| 스펙 | 파일 | 역할 | 상태 |
|---|---|---|---|
| §2 | `output/trajectory.json` | 소비 — 참조 궤적 로드 + 선형보간 샘플러 (`sample_trajectory`, 후방차분 vel/acc) | ✅ 실파일 검증 |
| §5 | `output/current_state.json` | 생산 — 상시 20~50Hz, 원자적 쓰기(tmp→rename), ref_state 동봉 | ✅ 파이썬 파서 교차 검증 |
| §3 | `output/attitude_feedback.json` | 생산 — 비행 후 1회 (`FlightLogger`: tail RMS·영교차 주파수·사인 피팅), used:false, 유효성 게이트(추종 RMS>30cm 거부) | ✅ 합성 지터 왕복 검증 |
| §6 | `output/param_estimate.json` | (예정) 비율 소비 — 가드레일 3종 준수 필수 | 미착수 |

타임스탬프는 `traj_pipeline.py`의 `TS_FMT`(`%Y-%m-%dT%H-%M-%S[.mmm]`, 콜론→하이픈)와
동일 — C++ 산출물을 파이썬 파이프라인이 그대로 읽음을 확인함.
경계: `qc_controller`(비행 루프)는 힙 금지, `qc_io`(컴패니언 측)는 std 컨테이너 허용.
제어 루프 안에서 IO 호출 금지 — 궤적은 이륙 전 로드, 상태 쓰기는 저주기/별도 스레드.

## 파일

- `include/qc_controller.hpp` — 설정(QcConfig)/상태(QcState)/스텝(qc_step). 물성 정규화
  `qc_phys()`는 parameters.m 17차와 1:1 (그쪽 바뀌면 여기도 갱신).
- `src/qc_controller.cpp` — 제어 체인. `[TODO-verify]` 주석 = 덤프/골든트레이스로 확정할 배선.
- `src/main_trace.cpp` — 골든 트레이스 하니스 (CSV in→out) + `--smoke`.
- `build.ps1` — 빌드. **mingw64\bin PATH 필수** (없으면 cc1plus가 DLL 못 찾아 무음 실패).

## 실비행 재생 발견 (sim_result_baked.mat 1707샘플, make_golden_input.py)

- **확정**: motorRef = 2π×(고도PID + 56.5 + 44.4·m_pkg) 스케일이 실측 모터속도와 일치
  (스케일비 0.98~1.02, 호버 평형 ~634 rad/s) — [TODO-verify] 2π/바이어스 항목 해소.
- **발견**: 모터 2·3은 내장 역회전(9차 규명)이라 실측 w가 음수 — C++ motorRef에
  모터별 부호(1,4:+ / 2,3:−) 반영 필요. 믹서 차동 부호표는 dump_controller_spec.m으로 확정.
- 제어기 출력(cmd/u) 골든 로그는 아직 없음 — run_traj_baked에 cmd 탭 추가 후
  compare_trace.py로 완전 대조 (그 전까지는 --check 개연성 검사만).

## 설계 규약

- 임베디드 강등 가능한 C++17 부분집합: 힙 없음(초기화 후), 예외/가상함수 없음, 고정 배열.
- 자세 게인 음수는 의도된 것 (플랜트 이득 음수) — "수정" 금지.
- anti-windup 없음 — 원본 Simulink와 동일하게 출력만 클램프 (골든 트레이스 일치가 우선,
  개선은 이식 검증 후).
- 입력 계약: 이 제어기는 스무더+게이트 통과 궤적 전제. 날것 스텝은 원본도 못 버틴다
  (릴레이 한계사이클) — 해법은 속도 캐스케이드 (이식 후 1순위 개선 항목).

## 빌드/실행

```powershell
.\build.ps1
.\qc_trace.exe --smoke                      # 수치 정상 확인
.\qc_trace.exe golden_in.csv cpp_out.csv    # 골든 트레이스 (입력 형식은 main_trace.cpp 머리)
```
