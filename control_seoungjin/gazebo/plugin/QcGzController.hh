// QcGzController.hh — Gazebo Harmonic (gz-sim8) 시스템 플러그인
//
// 역할: `control_seoungjin/controller_cpp` 의 제어기(`qc_step`)를 Gazebo 물리와
//       **같은 프로세스 / 같은 고정 스텝**으로 돌린다. 토픽 왕복을 쓰지 않는 이유는
//       전송 지터가 섞이면 "지연을 얼마 넣었을 때 무슨 일이 나는가"를 재는 실험이
//       오염되기 때문이다. 지연은 오직 플러그인이 **의도적으로** 넣는 것만 존재한다.
//
// 이 플러그인이 하는 일 (순서대로, PreUpdate 한 스텝 안에서):
//   1) ECM 에서 base_link 의 위치/자세/속도를 읽는다
//   2) 링버퍼로 posDelayS / attDelayS 만큼 **묵힌** 측정을 만든다 (지연 주입)
//   3) qc_step() 1회 (controlRateHz 로 데시메이션 — 기본 1 kHz = 물리와 동일)
//   4) qc_motor 로 motorCmd -> 회전속도 -> 추력/반토크
//   5) 로터 좌표에서 합력/합토크를 만들어 AddWorldWrench 로 인가
//   6) 외란(토크 펄스 / 정상풍)을 더한다
//   7) CSV 한 줄 기록 (logRateHz)
//
// 플랜트 독립성에 대한 정직한 범위: 강체 적분·접촉·좌표변환은 Gazebo(DART) 가
// Simscape 와 **독립**으로 한다. 반면 모터 1차 지연과 추력계수는 `qc_motor.hpp` 를
// 공유한다 — 즉 이 하네스는 "제어 사슬 + 강체 동역학"의 교차검증이지 모터 모델의
// 교차검증은 아니다. 모터까지 독립으로 보고 싶으면 SDF 의 MulticopterMotorModel
// 경로를 써야 하는데, 그건 별개 실험이다 (README "무엇을 증명하는가" 참조).
//
// 설정: SDF 자식 태그로 주고, 같은 이름을 대문자로 한 `QC_*` 환경변수가 **덮어쓴다**
//       (예: <armXY> -> QC_ARMXY). scripts/run_case.sh 는 이 규칙만 쓰므로 월드
//       파일을 매번 다시 쓸 필요가 없다.

#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "qc_controller.hpp"
#include "qc_motor.hpp"
#include "qc_io.hpp"

namespace qc_gz {

/// 한 스텝의 측정 스냅샷 (지연 링버퍼 원소)
struct MeasSample {
    double pos[3] = {0, 0, 0};
    double rpy[3] = {0, 0, 0};
};

class QcGzController
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
    QcGzController() = default;
    ~QcGzController() override;

    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr) override;

    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &ecm) override;

private:
    void OpenLog();
    /// mode=traj 일 때의 기준. 이륙 구간은 초기 위치에서 궤적 첫 점까지 7차 스무스스텝.
    void ReferenceAt(double t, double refPos[3], double &refYaw) const;

    // --- 엔티티 ---
    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Entity linkEntity_{gz::sim::kNullEntity};
    bool ready_ = false;
    bool initialPoseCaptured_ = false;
    double initialPos_[3] = {0, 0, 0};

    // --- 제어기 ---
    qc::QcConfig cfg_;
    qc::QcState st_;
    qc::MotorParams mp_;
    qc::Motor motors_[4];

    // --- 설정값 ---
    std::string linkName_ = "base_link";
    std::string mode_ = "hover";           // hover | traj | probe
    std::string mixerTable_ = "measured";  // measured | header
    double armXY_ = 0.1125;
    double rotorZ_ = 0.02;
    double comZ_ = -0.031754;
    double hoverZ_ = 1.0;
    double refYawFixed_ = 0.0;
    double controlRateHz_ = 1000.0;
    double logRateHz_ = 200.0;
    double takeoffS_ = 3.0;
    std::string trajPath_;
    std::string logPath_ = "gz_run.csv";

    // 지연 주입 [s]
    double posDelayS_ = 0.0;
    double attDelayS_ = 0.0;

    // 외란
    double pulseTorque_ = 0.0;
    std::string pulseAxis_ = "y";
    double pulseStartS_ = 0.0;
    double pulseDurS_ = 0.3;
    double wind_[3] = {0, 0, 0};
    std::string windFrame_ = "world";      // world = 기체가 기울어도 방향 고정 (진짜 바람)
    double pulseForce_ = 0.0;              // 힘 펄스 [N] — 축 하나 지정용 약식
    std::string pulseForceAxis_ = "x";
    // 3축 벡터형. 약식(pulseTorque/pulseAxis, pulseForce/pulseForceAxis)과 **더해진다**.
    // 힘과 토크를 동시에 거는 실제 외란(옆에서 때리는 돌풍 등)은 이쪽으로 준다.
    double pulseTorqueVec_[3] = {0, 0, 0};   // [N*m] 기체 좌표
    double pulseForceVec_[3] = {0, 0, 0};    // [N]   기체 좌표
    // 외란 힘의 **작용점** (링크 원점 기준 기체 좌표 [m]).
    // 무게중심이 아니라 여기에 걸리므로 r x F 만큼 모멘트가 같이 생긴다.
    double distPoint_[3] = {0, 0, 0};

    // 플랜트 진실 주입 (제어기는 공칭 유지 — main_trace 의 ct/cq 배율과 같은 뜻)
    double ctScale_ = 1.0;
    double cqScale_ = 1.0;

    // probe 모드
    std::string probeChannel_ = "pitch";   // pitch | roll | yaw
    double probeU_ = 1.0;                  // 명령 크기 [rev/s 차동]
    double probeStartS_ = 1.0;
    double probeDurS_ = 0.5;

    // --- 런타임 ---
    std::vector<MeasSample> ring_;
    std::size_t ringHead_ = 0;
    bool ringFilled_ = false;
    double ctrlAcc_ = 0.0;
    double logAcc_ = 0.0;
    // 사용 전력량 추정 누적 [Wh]. control_seoungjin/energy.py 와 **같은 식**이라
    // MATLAB(verify_worstcase) / Gazebo / 계획기 셋이 같은 수를 낸다.
    double energyWh_ = 0.0;
    double powerW_ = 0.0;
    qc::QcOutput lastOut_{};
    qcio::Trajectory traj_;
    std::ofstream log_;
    long logRows_ = 0;
};

}  // namespace qc_gz
