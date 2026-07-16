// qc_io.hpp — INTERFACE_SPEC v0.1 파일 계약의 C++ 구현 (컨트롤러 몫 3종)
//
//   §2 output/trajectory.json  : 소비 (참조 궤적 {dt, trajectory_hash, t[], pos[][3], yaw_rad[]})
//   §5 output/current_state.json: 생산 (상시 20~50Hz, 원자적 쓰기, ref_state 동봉)
//   §3 output/attitude_feedback.json: 생산 (비행 후 1회, used:false, tail/moving 지표)
//
// 경계 규약: qc_controller.hpp(비행 크리티컬 루프)는 힙 금지지만, 이 IO 계층은
//   컴패니언 컴퓨터 측이라 std::string/vector 허용. 제어 루프에 IO를 넣지 말 것 —
//   trajectory는 이륙 전 로드, current_state는 쓰기 스레드/저주기로 분리.
// 타임스탬프: "%Y-%m-%dT%H-%M-%S.mmm" 로컬 (traj_pipeline.py TS_FMT과 동일 — 콜론 대신 하이픈).
// 공통 규칙: 필수 키 누락/파일 없음 = 조용히 통과 금지, false 반환 + err 메시지 (저장소 error() 규칙).

#pragma once
#include <string>
#include <vector>

namespace qcio {

// ---------- §2 참조 궤적 ----------

struct Trajectory {
    double dt = 0.01;
    std::string hash;                 // trajectory_hash — 피드백 대조 열쇠
    std::vector<double> t;
    std::vector<double> px, py, pz;
    std::vector<double> yaw;

    bool empty() const { return t.empty(); }
    double duration() const { return t.empty() ? 0.0 : t.back(); }
};

// trajectory.json 로드. 실패 시 false + err에 사유.
bool load_trajectory(const std::string& path, Trajectory& out, std::string& err);

// 시각 tq에서 참조 샘플 (선형 보간 + 후방차분 vel/acc — ref_state용).
// 범위 밖은 끝점 유지 (성형 궤적은 끝점 정지 전제).
struct RefSample {
    double pos[3], vel[3], acc[3], yaw;
};
RefSample sample_trajectory(const Trajectory& tr, double tq);

// ---------- §5 실시간 상태 ----------

struct CurrentState {
    double pos[3], vel[3], acc[3];
    double yaw = 0;
    RefSample ref;                    // 현재 성형 기준의 상태 (재계획 이어붙이기용)
};

// 원자적 쓰기 (tmp → rename). 호출 주기 조절은 호출자 몫 (스펙 20~50Hz).
bool write_current_state(const std::string& path, const CurrentState& st, std::string& err);

// ---------- §3 잔류 지터 보고 ----------

// 비행 로그 축적기: (t, pitch[rad], roll[rad], 추종오차[m]) 스트림 → tail/moving 지표.
// analyze_flight_log.py의 검출 로직 대응 (RMS + 영교차 주파수 + 고정주파수 사인 피팅).
struct FlightLogger {
    std::vector<double> t, pitch, roll, trackErr;
    double tArrive = 0;               // 도착 시각 (이후 = tail 구간). 궤적 duration으로 설정.

    void push(double time, double pitchRad, double rollRad, double trackErrM);

    struct Feedback {
        double modeFreqHz = 0;
        double tailPitchRmsDeg = 0, tailRollRmsDeg = 0;
        double ampDeg = 0, phaseRad = 0, tRefS = 0;
        double movingAttPeakDeg = 0, movingTrackRmsCm = 0;
        bool valid = false;           // 유효성 게이트: 추종 RMS>30cm면 false (발산 오염 방지)
    };
    Feedback analyze() const;
};

// attitude_feedback.json 기록 (used:false, written_at=now). flight_id는 now 타임스탬프.
bool write_attitude_feedback(const std::string& path, const FlightLogger::Feedback& fb,
                             const std::string& trajectoryHash, std::string& err);

// 현재 시각 문자열 "%Y-%m-%dT%H-%M-%S" (+ withMs면 ".mmm")
std::string now_string(bool withMs);

// ---------- §6 플랜트 상수 추정 소비 ----------
// INTERFACE_SPEC §6 가드레일 3종을 코드로 강제:
//   1) 앵커(*Ref) 불변 — 이 함수는 현재값만 갱신, ref는 절대 안 건드림
//   2) 질량/관성은 기본 미적용 (시뮬 플랜트 일관성) — allowMassInertia는 실기 전용
//   3) 비율로만 — k_*_lumped(집중계수)는 이전 추정 대비 '비율'로 kThrust/kDrag에 적용
// confident:true(R²>=문턱) 항목만 반영. maxStepFrac로 급변 방지 램프 (기본 ±10%/호출).
struct ParamEstimateApply {
    bool appliedThrust = false, appliedDrag = false, appliedMass = false;
    double thrustRatio = 1.0, dragRatio = 1.0;   // 이번 호출로 곱해진 비율
    std::string note;
};
// prevLumpedThrust/Drag: 직전 추정값 (비율의 분모). 최초 호출이면 <=0 → 기록만 하고 미적용.
bool apply_param_estimate(const std::string& path,
                          double prevLumpedThrust, double prevLumpedDrag,
                          double maxStepFrac, bool allowMassInertia,
                          double& kThrustInOut, double& kDragInOut,
                          ParamEstimateApply& out, std::string& err);

} // namespace qcio
