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

## 파일

- `include/qc_controller.hpp` — 설정(QcConfig)/상태(QcState)/스텝(qc_step). 물성 정규화
  `qc_phys()`는 parameters.m 17차와 1:1 (그쪽 바뀌면 여기도 갱신).
- `src/qc_controller.cpp` — 제어 체인. `[TODO-verify]` 주석 = 덤프/골든트레이스로 확정할 배선.
- `src/main_trace.cpp` — 골든 트레이스 하니스 (CSV in→out) + `--smoke`.
- `build.ps1` — 빌드. **mingw64\bin PATH 필수** (없으면 cc1plus가 DLL 못 찾아 무음 실패).

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
