// QcGzController.cc — 구현. 설계 근거는 QcGzController.hh 머리말 참조.

#include "QcGzController.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>

#include <sdf/Element.hh>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>

namespace qc_gz {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.81;

/// SDF 태그명을 환경변수 이름으로: armXY -> QC_ARMXY
bool env_lookup(const std::string &tag, std::string &val) {
    std::string key = "QC_";
    for (char c : tag) key += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    const char *e = std::getenv(key.c_str());
    if (e == nullptr || *e == '\0') return false;
    val = e;
    return true;
}

double param_d(const std::shared_ptr<const sdf::Element> &sdf,
               const std::string &tag, double dflt) {
    std::string raw;
    if (env_lookup(tag, raw)) {
        try { return std::stod(raw); }
        catch (...) { gzerr << "[qc] " << tag << " 환경변수 파싱 실패: " << raw << "\n"; }
    }
    if (sdf && sdf->HasElement(tag)) return sdf->Get<double>(tag, dflt).first;
    return dflt;
}

std::string param_s(const std::shared_ptr<const sdf::Element> &sdf,
                    const std::string &tag, const std::string &dflt) {
    std::string raw;
    if (env_lookup(tag, raw)) return raw;
    if (sdf && sdf->HasElement(tag)) return sdf->Get<std::string>(tag, dflt).first;
    return dflt;
}

bool param_b(const std::shared_ptr<const sdf::Element> &sdf,
             const std::string &tag, bool dflt) {
    const std::string s = param_s(sdf, tag, dflt ? "true" : "false");
    return s == "1" || s == "true" || s == "True" || s == "TRUE" || s == "on";
}

/// 7차 스무스스텝 S(u) = 35u^4 - 84u^5 + 70u^6 - 20u^7 (양끝 C3).
/// traj_bridge.py 와 같은 식을 쓴다 — 이륙 램프가 궤적 층과 다른 매끄러움을 갖지
/// 않게 하려는 것. 날것 스텝 금지 규칙(HANDOFF_CPP_GAZEBO "절대 규칙 3")의 이행이다.
double smoothstep7(double u) {
    if (u <= 0.0) return 0.0;
    if (u >= 1.0) return 1.0;
    const double u2 = u * u, u3 = u2 * u, u4 = u3 * u;
    return 35.0 * u4 - 84.0 * u4 * u + 70.0 * u4 * u2 - 20.0 * u4 * u3;
}

}  // namespace

QcGzController::~QcGzController() {
    if (this->log_.is_open()) {
        this->log_.flush();
        this->log_.close();
        gzmsg << "[qc] 로그 " << this->logRows_ << " 행 -> " << this->logPath_ << "\n";
    }
}

// ---------------------------------------------------------------- Configure

void QcGzController::Configure(const gz::sim::Entity &entity,
                               const std::shared_ptr<const sdf::Element> &sdf,
                               gz::sim::EntityComponentManager &ecm,
                               gz::sim::EventManager &) {
    this->model_ = gz::sim::Model(entity);
    if (!this->model_.Valid(ecm)) {
        gzerr << "[qc] 모델 엔티티가 유효하지 않음 — 플러그인은 <model> 안에 두어야 한다\n";
        return;
    }

    this->linkName_ = param_s(sdf, "link", this->linkName_);
    this->linkEntity_ = this->model_.LinkByName(ecm, this->linkName_);
    if (this->linkEntity_ == gz::sim::kNullEntity) {
        gzerr << "[qc] 링크 없음: " << this->linkName_ << "\n";
        return;
    }
    gz::sim::Link link(this->linkEntity_);
    link.EnableVelocityChecks(ecm, true);

    // --- 설정 읽기 (SDF -> QC_* 환경변수 우선) ---
    this->mode_         = param_s(sdf, "mode", this->mode_);
    this->mixerTable_   = param_s(sdf, "mixerTable", this->mixerTable_);
    this->armXY_        = param_d(sdf, "armXY", this->armXY_);
    this->rotorZ_       = param_d(sdf, "rotorZ", this->rotorZ_);
    this->comZ_         = param_d(sdf, "comZ", this->comZ_);
    this->hoverZ_       = param_d(sdf, "hoverZ", this->hoverZ_);
    this->refYawFixed_  = param_d(sdf, "refYaw", this->refYawFixed_);
    this->controlRateHz_ = param_d(sdf, "controlRateHz", this->controlRateHz_);
    this->logRateHz_    = param_d(sdf, "logRateHz", this->logRateHz_);
    this->takeoffS_     = param_d(sdf, "takeoffS", this->takeoffS_);
    this->trajPath_     = param_s(sdf, "trajectory", this->trajPath_);
    this->logPath_      = param_s(sdf, "log", this->logPath_);
    this->posDelayS_    = param_d(sdf, "posDelayS", this->posDelayS_);
    this->attDelayS_    = param_d(sdf, "attDelayS", this->attDelayS_);
    this->pulseTorque_  = param_d(sdf, "pulseTorque", this->pulseTorque_);
    this->pulseAxis_    = param_s(sdf, "pulseAxis", this->pulseAxis_);
    this->pulseStartS_  = param_d(sdf, "pulseStartS", this->pulseStartS_);
    this->pulseDurS_    = param_d(sdf, "pulseDurS", this->pulseDurS_);
    this->wind_[0]      = param_d(sdf, "windX", 0.0);
    this->wind_[1]      = param_d(sdf, "windY", 0.0);
    this->wind_[2]      = param_d(sdf, "windZ", 0.0);
    this->windFrame_    = param_s(sdf, "windFrame", this->windFrame_);
    this->pulseForce_   = param_d(sdf, "pulseForce", this->pulseForce_);
    this->pulseForceAxis_ = param_s(sdf, "pulseForceAxis", this->pulseForceAxis_);
    this->pulseTorqueVec_[0] = param_d(sdf, "pulseTorqueX", 0.0);
    this->pulseTorqueVec_[1] = param_d(sdf, "pulseTorqueY", 0.0);
    this->pulseTorqueVec_[2] = param_d(sdf, "pulseTorqueZ", 0.0);
    this->pulseForceVec_[0]  = param_d(sdf, "pulseForceX", 0.0);
    this->pulseForceVec_[1]  = param_d(sdf, "pulseForceY", 0.0);
    this->pulseForceVec_[2]  = param_d(sdf, "pulseForceZ", 0.0);
    // 외란 힘의 작용점 (링크 원점 기준 기체 좌표). 무게중심이 아니라 여기에 걸린다.
    this->distPoint_[0] = param_d(sdf, "distPointX", 0.0);
    this->distPoint_[1] = param_d(sdf, "distPointY", 0.0);
    this->distPoint_[2] = param_d(sdf, "distPointZ", 0.0);
    this->ctScale_      = param_d(sdf, "ctScale", 1.0);
    this->cqScale_      = param_d(sdf, "cqScale", 1.0);
    this->probeChannel_ = param_s(sdf, "probeChannel", this->probeChannel_);
    this->probeU_       = param_d(sdf, "probeU", this->probeU_);
    this->probeStartS_  = param_d(sdf, "probeStartS", this->probeStartS_);
    this->probeDurS_    = param_d(sdf, "probeDurS", this->probeDurS_);

    // --- 제어기 구성 ---
    this->cfg_.pkgMass = param_d(sdf, "pkgMass", this->cfg_.pkgMass);
    const std::string prof = param_s(sdf, "profile", "precision");
    if (prof == "agile")         qc::qc_apply_profile(this->cfg_, qc::Profile::Agile);
    else if (prof == "balanced") qc::qc_apply_profile(this->cfg_, qc::Profile::Balanced);
    else                         qc::qc_apply_profile(this->cfg_, qc::Profile::Precision);

    // 질량 1차식 (qc_mass_lerp / MATLAB qc_mass_lerp_apply.m 의 짝).
    // 0 kg 은 이게 없으면 **못 뜬다**: 무보정 biasChassis=56.5 rev/s 는 추력 6.98 N 인데
    // 생 드론 무게가 12.48 N 이다. 1차식의 0 kg 앵커 75.5 가 필요한 75.6 과 맞는다.
    // 프로파일 충돌 주의: 1차식의 kpPos 앵커(5->8)는 precision 기준이라 agile 과 겹친다
    // (QcConfig::massLerpOn 주석). 그래서 precision 이 아니면 경고한다.
    this->cfg_.massLerpOn = param_b(sdf, "massLerpOn", false);
    if (this->cfg_.massLerpOn) {
        if (prof != "precision")
            gzwarn << "[qc] massLerpOn + profile=" << prof
                   << " — 1차식 kpPos 앵커는 precision 기준이라 프로파일 설정을 덮어쓴다
";
        const qc::MassLerp L = qc::qc_mass_lerp(this->cfg_.pkgMass);
        this->cfg_.biasChassis = L.biasChassis;
        this->cfg_.limAtt      = L.limAtt;
        this->cfg_.kdAtt       = this->cfg_.kpAtt * L.rAtt;   // rAtt = kd/kp 비
        this->cfg_.kpPos       = L.kpPos;   this->cfg_.kdPos  = L.kdPos;
        this->cfg_.kpPosZ      = L.kpPos;   this->cfg_.kdPosZ = L.kdPos;
        this->cfg_.tauMeasAlt  = L.filtPz;
        // nlGmax 는 대응 필드가 확정되지 않아 적용하지 않는다 (추측 반영 금지).
    }
    // 개별 덮어쓰기는 1차식 뒤에 온다 — 손으로 준 값이 항상 이긴다.
    this->cfg_.biasChassis = param_d(sdf, "biasChassis", this->cfg_.biasChassis);
    this->cfg_.altCmdSat   = param_d(sdf, "altCmdSat", this->cfg_.altCmdSat);

    this->cfg_.specOn = param_b(sdf, "specOn", true);
    this->cfg_.recOn  = param_b(sdf, "recOn", false);
    this->cfg_.govOn  = param_b(sdf, "govOn", false);
    this->cfg_.latencyAttS = this->attDelayS_ > 0.0 ? this->attDelayS_ : this->cfg_.latencyAttS;

    // 믹서 표. 기본은 08-18 골든 트레이스 실측표다.
    //
    // 왜 헤더 기본값을 그대로 안 쓰나: qc_controller.hpp 의 mixYaw = {-1,+1,-1,+1} 은
    // mixDir = {+1,-1,-1,+1} 과 **직교**한다. yaw 토크는 로터 반토크의 차동으로만
    // 생기므로 sum(mixDir_i * mixYaw_i) = 0 이면 yaw 권한이 원리적으로 0 이다.
    // 또 그 표에서 유도되는 로터 사분면은 대각 로터의 회전방향이 서로 반대가 되어
    // X 쿼드 규약에도 어긋난다. 실측표는 mixYaw = -mixDir 로 정확히 정렬된다.
    // 헤더는 골든 트레이스 불변을 위해 건드리지 않고, 여기서만 갈아 끼운다.
    if (this->mixerTable_ == "measured") {
        const double p[4] = {+1, +1, -1, -1};
        const double r[4] = {-1, +1, -1, +1};
        const double y[4] = {-1, +1, +1, -1};
        for (int i = 0; i < 4; ++i) {
            this->cfg_.mixPitch[i] = p[i];
            this->cfg_.mixRoll[i]  = r[i];
            this->cfg_.mixYaw[i]   = y[i];
        }
    } else {
        gzwarn << "[qc] mixerTable=header — qc_controller.hpp 기본표를 쓴다. "
                  "yaw 권한이 0 인 표라 yaw 시험은 무의미하다 (의도적 대조용).\n";
    }

    qc::qc_bind(this->st_, this->cfg_);
    this->mp_.Ct *= this->ctScale_;
    this->mp_.Cq *= this->cqScale_;

    // --- 호버 가능성 방어 ---
    // 고도 PID 가 권한을 다 써도 무게를 못 이기면 어떤 게인으로도 못 뜬다. 그 상태로
    // 날려서 "제어기가 나쁘다"는 결론을 내는 것을 막는다 (실제로 altCmdSat 배선 오류로
    // 이 상황이 있었다 — 2026-08-26 정정).
    {
        const qc::PhysOut phc = qc::qc_phys(this->cfg_.droneMass, this->cfg_.pkgMass,
                                            this->cfg_.pkgSize);
        const double nMax = this->cfg_.biasChassis
                          + this->cfg_.biasLoadGain * this->cfg_.pkgMass
                          + this->cfg_.limAlt;                       // rev/s
        const double tMax = 4.0 * this->mp_.Ct * this->mp_.rho * nMax * nMax
                          * std::pow(this->mp_.D, 4);
        const double weight = phc.m_tot * kG;
        if (tMax < weight) {
            gzerr << "[qc] 이 구성으로는 뜰 수 없다: 최대 추력 " << tMax << " N < 무게 "
                  << weight << " N (base 상한 " << nMax << " rev/s)
";
            gzerr << "[qc]   0 kg 이면 QC_MASSLERPON=1 (biasChassis 75.5), "
                     "아니면 QC_BIASCHASSIS 를 직접 줄 것
";
            return;
        }
    }

    // 모터 초기 회전속도를 호버 평형으로 놓는다. 0 에서 시작하면 첫 0.3 초 동안
    // 자유낙하해 지면에 부딪히고, 그 접촉 과도가 모든 지표를 오염시킨다.
    const double hoverW = 2.0 * kPi * (this->cfg_.biasChassis
                                       + this->cfg_.biasLoadGain * this->cfg_.pkgMass);
    for (auto &m : this->motors_) m.w = hoverW;

    if (this->mode_ == "traj") {
        std::string err;
        if (this->trajPath_.empty() ||
            !qcio::load_trajectory(this->trajPath_, this->traj_, err)) {
            gzerr << "[qc] mode=traj 인데 궤적을 못 읽었다: "
                  << (this->trajPath_.empty() ? "<trajectory> 미지정" : err) << "\n";
            return;
        }
    }

    this->OpenLog();
    if (!this->log_.is_open()) return;

    const qc::PhysOut ph = qc::qc_phys(this->cfg_.droneMass, this->cfg_.pkgMass,
                                       this->cfg_.pkgSize);
    gzmsg << "[qc] mode=" << this->mode_ << " mixer=" << this->mixerTable_
          << " profile=" << prof << " m_pkg=" << this->cfg_.pkgMass << "\n";
    gzmsg << "[qc] m_tot=" << ph.m_tot << " I_att=" << ph.I_att << " I_yaw=" << ph.I_yaw
          << " armXY=" << this->armXY_ << " hoverW=" << hoverW << " rad/s\n";
    gzmsg << "[qc] 지연 pos=" << this->posDelayS_ * 1e3 << " ms att="
          << this->attDelayS_ * 1e3 << " ms | 펄스 " << this->pulseTorque_ << " Nm ("
          << this->pulseAxis_ << ") @" << this->pulseStartS_ << "s x" << this->pulseDurS_
          << "s | 바람 " << this->wind_[0] << "," << this->wind_[1] << "," << this->wind_[2]
          << " N (" << this->windFrame_ << ") | 힘펄스 " << this->pulseForce_ << " N (" << this->pulseForceAxis_ << ")"
          << " | 작용점 " << this->distPoint_[0] << "," << this->distPoint_[1]
          << "," << this->distPoint_[2] << " m\n";
    if (this->ctScale_ != 1.0 || this->cqScale_ != 1.0)
        gzmsg << "[qc] 플랜트 진실 주입 Ct x" << this->ctScale_
              << " Cq x" << this->cqScale_ << " (제어기는 공칭 유지)\n";

    this->ready_ = true;
}

// ------------------------------------------------------------------- 로그

void QcGzController::OpenLog() {
    this->log_.open(this->logPath_, std::ios::out | std::ios::trunc);
    if (!this->log_.is_open()) {
        gzerr << "[qc] 로그 파일을 못 연다: " << this->logPath_ << "\n";
        return;
    }
    this->log_ << "t,ref_x,ref_y,ref_z,ref_yaw,"
                  "x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,"
                  "cmd_pitch,cmd_roll,mref1,mref2,mref3,mref4,w1,w2,w3,w4,"
                  "thrust,tau_x,tau_y,tau_z,rho,s_clock,"
                  "spec_scale,spec_v,spec_a,spec_lat_pos,spec_lat_att,spec_rec,"
                  "P_est_W,E_est_Wh,lat_applied,mission_allowed,dist_on\n";
    this->log_ << std::setprecision(9);
}

// ------------------------------------------------------------------ 기준

void QcGzController::ReferenceAt(double t, double refPos[3], double &refYaw) const {
    // 공통: 초기 위치에서 목표 시작점까지 7차 스무스스텝으로 올린다.
    double target[3];
    double targetYaw = this->refYawFixed_;
    if (this->mode_ == "traj" && !this->traj_.empty()) {
        const double tTraj = t - this->takeoffS_;
        if (tTraj >= 0.0) {
            const qcio::RefSample rs = qcio::sample_trajectory(this->traj_, tTraj);
            refPos[0] = rs.pos[0]; refPos[1] = rs.pos[1]; refPos[2] = rs.pos[2];
            refYaw = rs.yaw;
            return;
        }
        const qcio::RefSample rs0 = qcio::sample_trajectory(this->traj_, 0.0);
        target[0] = rs0.pos[0]; target[1] = rs0.pos[1]; target[2] = rs0.pos[2];
        targetYaw = rs0.yaw;
    } else {
        target[0] = this->initialPos_[0];
        target[1] = this->initialPos_[1];
        target[2] = this->hoverZ_;
    }
    const double s = this->takeoffS_ > 1e-9 ? smoothstep7(t / this->takeoffS_) : 1.0;
    for (int i = 0; i < 3; ++i)
        refPos[i] = this->initialPos_[i] + (target[i] - this->initialPos_[i]) * s;
    refYaw = targetYaw * s;
}

// --------------------------------------------------------------- PreUpdate

void QcGzController::PreUpdate(const gz::sim::UpdateInfo &info,
                               gz::sim::EntityComponentManager &ecm) {
    if (!this->ready_ || info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0.0) return;
    const double t = std::chrono::duration<double>(info.simTime).count();

    gz::sim::Link link(this->linkEntity_);
    const gz::math::Pose3d pose = gz::sim::worldPose(this->linkEntity_, ecm);
    const gz::math::Quaterniond q = pose.Rot();
    const gz::math::Vector3d rpy = q.Euler();

    const auto velOpt = link.WorldLinearVelocity(ecm);
    const auto omgOpt = link.WorldAngularVelocity(ecm);
    const gz::math::Vector3d vel = velOpt ? *velOpt : gz::math::Vector3d::Zero;
    const gz::math::Vector3d omgW = omgOpt ? *omgOpt : gz::math::Vector3d::Zero;
    const gz::math::Vector3d omgB = q.RotateVectorReverse(omgW);

    if (!this->initialPoseCaptured_) {
        this->initialPos_[0] = pose.Pos().X();
        this->initialPos_[1] = pose.Pos().Y();
        this->initialPos_[2] = pose.Pos().Z();
        this->initialPoseCaptured_ = true;
    }

    // --- 지연 주입: 링버퍼에 넣고, 원하는 만큼 과거를 꺼낸다 ---
    if (this->ring_.empty()) {
        const double worst = std::max(this->posDelayS_, this->attDelayS_);
        const std::size_t n = static_cast<std::size_t>(worst / dt + 0.5) + 2;
        this->ring_.assign(std::max<std::size_t>(n, 2), MeasSample{});
    }
    MeasSample now;
    now.pos[0] = pose.Pos().X(); now.pos[1] = pose.Pos().Y(); now.pos[2] = pose.Pos().Z();
    now.rpy[0] = rpy.X(); now.rpy[1] = rpy.Y(); now.rpy[2] = rpy.Z();
    this->ring_[this->ringHead_] = now;
    const std::size_t N = this->ring_.size();
    auto delayed = [&](double delayS) -> const MeasSample & {
        std::size_t back = static_cast<std::size_t>(delayS / dt + 0.5);
        if (!this->ringFilled_) {
            // 아직 과거가 없으면 있는 만큼만 (초기 몇 스텝은 지연이 덜 걸린다).
            back = std::min(back, this->ringHead_);
        }
        back = std::min(back, N - 1);
        const std::size_t idx = (this->ringHead_ + N - back) % N;
        return this->ring_[idx];
    };
    const MeasSample &mPos = delayed(this->posDelayS_);
    const MeasSample &mAtt = delayed(this->attDelayS_);

    // --- 제어 (controlRateHz 로 데시메이션) ---
    this->ctrlAcc_ += dt;
    const double ctrlPeriod = this->controlRateHz_ > 1e-9 ? 1.0 / this->controlRateHz_ : dt;
    double refPos[3] = {0, 0, 0};
    double refYaw = 0.0;
    this->ReferenceAt(t, refPos, refYaw);

    qc::QcInput in{};
    for (int i = 0; i < 3; ++i) {
        in.refPos[i] = refPos[i];
        in.measPos[i] = mPos.pos[i];
        in.measRpy[i] = mAtt.rpy[i];
    }
    in.refYaw = refYaw;
    in.measAlt = mPos.pos[2];
    in.measAgeS = this->posDelayS_;          // 지연 추적기에 실제 주입량을 그대로 준다
    in.refWithinLimits = true;
    for (int i = 0; i < 4; ++i) in.motorSpd[i] = this->cfg_.mixDir[i] * this->motors_[i].w;

    const bool probing = (this->mode_ == "probe");
    if (!probing && this->ctrlAcc_ >= ctrlPeriod) {
        this->ctrlAcc_ -= ctrlPeriod;
        this->lastOut_ = qc::qc_step(this->st_, this->cfg_, in, ctrlPeriod);
    }

    // --- 모터 플랜트 ---
    double thrust[4] = {0, 0, 0, 0};
    double dragQ[4] = {0, 0, 0, 0};
    if (probing) {
        // 개루프 프로브: 호버 바이어스 + 한 채널 차동. 병진은 일부러 없앤다
        // (중력을 상쇄해 띄워 두고 각가속도만 본다 — 회전 플랜트 이득 측정용).
        const double base = this->cfg_.biasChassis + this->cfg_.biasLoadGain * this->cfg_.pkgMass;
        const double *mix = this->cfg_.mixPitch;
        if (this->probeChannel_ == "roll") mix = this->cfg_.mixRoll;
        else if (this->probeChannel_ == "yaw") mix = this->cfg_.mixYaw;
        const bool on = (t >= this->probeStartS_) && (t < this->probeStartS_ + this->probeDurS_);
        for (int i = 0; i < 4; ++i) {
            const double u = on ? this->probeU_ : 0.0;
            const double mref = 2.0 * kPi * (base + mix[i] * u);
            const double cmd = this->st_.pidMot[i].step(mref - this->motors_[i].w, dt);
            const qc::MotorOut mo = this->motors_[i].step(cmd, this->mp_, dt);
            thrust[i] = mo.thrust;
            dragQ[i] = mo.dragQ;
            this->lastOut_.motorRef[i] = mref;
        }
    } else {
        for (int i = 0; i < 4; ++i) {
            const qc::MotorOut mo = this->motors_[i].step(this->lastOut_.motorCmd[i], this->mp_, dt);
            thrust[i] = mo.thrust;
            dragQ[i] = mo.dragQ;
        }
    }

    // --- 로터 좌표 -> 합력/합토크 (기체 좌표) ---
    // 로터 사분면은 믹서 부호에서 유도한다 (gen_worlds.mixer_geometry 와 같은 규칙):
    //   +pitch (tau_y>0) 는 x<0 쪽을, +roll (tau_x>0) 는 y>0 쪽을 올린다.
    // 추력은 기체 +z 성분뿐이라 r x F 의 z 성분(로터 높이)은 토크에 기여하지 않는다.
    // 그래서 rotorZ/comZ 는 여기서 쓰이지 않는다 — 합력을 무게중심에 인가하고
    // 토크를 따로 주는 형태(AddWorldWrench)라 오프셋이 정확히 소거된다.
    double Fz = 0.0, Tx = 0.0, Ty = 0.0, Tz = 0.0;
    for (int i = 0; i < 4; ++i) {
        const double x = (this->cfg_.mixPitch[i] > 0 ? -1.0 : +1.0) * this->armXY_;
        const double y = (this->cfg_.mixRoll[i] > 0 ? +1.0 : -1.0) * this->armXY_;
        Fz += thrust[i];
        Tx += y * thrust[i];
        Ty += -x * thrust[i];
        // 반토크는 로터 회전방향의 반대로 기체에 걸린다.
        Tz += -this->cfg_.mixDir[i] * dragQ[i];
    }

    // --- 외란 ---
    // **작용점을 갖는다.** 무게중심에 순수 힘을 걸면 기체가 평행이동만 하는데, 실제로
    // 바람이 기체 옆면/짐에 걸리면 힘과 **모멘트가 같이** 생긴다. 어느 쪽으로 얼마나
    // 도는지가 곧 자세 루프가 감당할 몫이라, 작용점을 못 지정하면 외란 시험이 실제보다
    // 쉬워진다. distPoint 는 링크 원점 기준 기체 좌표이고, 무게중심 기준으로 환산해서
    //   tau_dist = r x F,   r = distPoint - (0, 0, comZ)
    // 를 더한다.
    int distOn = 0;
    double dFx = 0, dFy = 0, dFz = 0;      // 외란 힘   (기체 좌표)
    double dTx = 0, dTy = 0, dTz = 0;      // 외란 토크 (기체 좌표)

    // pulseDurS <= 0 이면 창 없이 상시 (계단 외란).
    const bool inWindow = (this->pulseDurS_ <= 0.0)
                        || (t >= this->pulseStartS_ &&
                            t < this->pulseStartS_ + this->pulseDurS_);
    if (this->pulseTorque_ != 0.0 && inWindow) {
        distOn = 1;
        if (this->pulseAxis_ == "x")      dTx += this->pulseTorque_;
        else if (this->pulseAxis_ == "z") dTz += this->pulseTorque_;
        else                              dTy += this->pulseTorque_;
    }
    if (this->pulseForce_ != 0.0 && inWindow) {
        distOn = 1;
        if (this->pulseForceAxis_ == "y")      dFy += this->pulseForce_;
        else if (this->pulseForceAxis_ == "z") dFz += this->pulseForce_;
        else                                   dFx += this->pulseForce_;
    }
    // 3축 벡터형 (약식과 더해진다). 힘과 토크를 **동시에** 거는 경우가 이쪽.
    if (inWindow) {
        for (int k = 0; k < 3; ++k) {
            if (this->pulseTorqueVec_[k] != 0.0 || this->pulseForceVec_[k] != 0.0) distOn = 1;
        }
        dTx += this->pulseTorqueVec_[0];
        dTy += this->pulseTorqueVec_[1];
        dTz += this->pulseTorqueVec_[2];
        dFx += this->pulseForceVec_[0];
        dFy += this->pulseForceVec_[1];
        dFz += this->pulseForceVec_[2];
    }
    // 바람은 창과 무관하게 상시. windFrame=world 면 기체가 기울어도 방향이 고정이고
    // (진짜 바람), body 면 기체에 붙어 돈다 (편의용).
    if (this->wind_[0] != 0.0 || this->wind_[1] != 0.0 || this->wind_[2] != 0.0) {
        distOn = 1;
        gz::math::Vector3d wb(this->wind_[0], this->wind_[1], this->wind_[2]);
        if (this->windFrame_ != "body") wb = q.RotateVectorReverse(wb);
        dFx += wb.X(); dFy += wb.Y(); dFz += wb.Z();
    }
    // 작용점 -> 모멘트
    if (dFx != 0.0 || dFy != 0.0 || dFz != 0.0) {
        const double rx = this->distPoint_[0];
        const double ry = this->distPoint_[1];
        const double rz = this->distPoint_[2] - this->comZ_;
        dTx += ry * dFz - rz * dFy;
        dTy += rz * dFx - rx * dFz;
        dTz += rx * dFy - ry * dFx;
    }

    gz::math::Vector3d Fw, Tw;
    if (probing) {
        const qc::PhysOut ph = qc::qc_phys(this->cfg_.droneMass, this->cfg_.pkgMass,
                                           this->cfg_.pkgSize);
        Fw = gz::math::Vector3d(0, 0, ph.m_tot * kG);        // 중력만 상쇄 = 제자리 부양
        Tw = q.RotateVector(gz::math::Vector3d(Tx, Ty, Tz));
    } else {
        Fw = q.RotateVector(gz::math::Vector3d(0, 0, Fz)
                            + gz::math::Vector3d(dFx, dFy, dFz));
        Tw = q.RotateVector(gz::math::Vector3d(Tx + dTx, Ty + dTy, Tz + dTz));
    }
    // 밖에서 gz-sim-apply-link-wrench-system 으로 찔러 넣는 렌치가 있으면 같은 스텝에
    // 합산된다 (scripts/poke.sh). AddWorldWrench 는 누적이라 두 경로가 서로를 안 지운다.
    // --- 사용 전력량 추정 (운동량 이론; energy.py 의 electrical_power 와 동일) ---
    // Gazebo 는 총 추력 Fz 를 정확히 알고 있으므로 여기서는 추정이 아니라 모델 내
    // 실측에 가깝다. 그래도 효율(FM*eta)은 여전히 미측정 상수라 '추정'으로 부른다.
    {
        const double kRho = 1.225, kRprop = 0.127, kFM = 0.70, kEta = 0.80;
        const double A = kPi * kRprop * kRprop;
        const double Ttot = Fz > 0.0 ? Fz : 0.0;
        const double per = Ttot / 4.0;
        const double pIdeal = 4.0 * std::pow(per, 1.5) / std::sqrt(2.0 * kRho * A);
        this->powerW_ = pIdeal / (kFM * kEta);
        this->energyWh_ += this->powerW_ * dt / 3600.0;
    }
    link.AddWorldWrench(ecm, Fw, Tw);

    // --- 로그 ---
    this->logAcc_ += dt;
    const double logPeriod = this->logRateHz_ > 1e-9 ? 1.0 / this->logRateHz_ : dt;
    if (this->logAcc_ >= logPeriod && this->log_.is_open()) {
        this->logAcc_ -= logPeriod;
        const qc::QcOutput &o = this->lastOut_;
        this->log_
            << t << ',' << refPos[0] << ',' << refPos[1] << ',' << refPos[2] << ',' << refYaw
            << ',' << now.pos[0] << ',' << now.pos[1] << ',' << now.pos[2]
            << ',' << now.rpy[0] << ',' << now.rpy[1] << ',' << now.rpy[2]
            << ',' << vel.X() << ',' << vel.Y() << ',' << vel.Z()
            << ',' << omgB.X() << ',' << omgB.Y() << ',' << omgB.Z()
            << ',' << o.cmdPitch << ',' << o.cmdRoll
            << ',' << o.motorRef[0] << ',' << o.motorRef[1]
            << ',' << o.motorRef[2] << ',' << o.motorRef[3]
            << ',' << this->motors_[0].w << ',' << this->motors_[1].w
            << ',' << this->motors_[2].w << ',' << this->motors_[3].w
            << ',' << Fz << ',' << Tx << ',' << Ty << ',' << Tz
            << ',' << o.rho << ',' << o.sClock
            << ',' << o.spec.timeScale << ',' << o.spec.v << ',' << o.spec.a
            << ',' << o.spec.scaleLatPos << ',' << o.spec.scaleLatAtt
            << ',' << o.spec.scaleRecovery
            << ',' << this->powerW_ << ',' << this->energyWh_
            << ',' << o.spec.latencyAppliedS
            << ',' << (o.spec.missionAllowed ? 1 : 0)
            << ',' << distOn << '\n';
        if ((++this->logRows_ % 200) == 0) this->log_.flush();
    }

    this->ringHead_ = (this->ringHead_ + 1) % N;
    if (this->ringHead_ == 0) this->ringFilled_ = true;
}

}  // namespace qc_gz

GZ_ADD_PLUGIN(qc_gz::QcGzController,
              gz::sim::System,
              qc_gz::QcGzController::ISystemConfigure,
              qc_gz::QcGzController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(qc_gz::QcGzController, "qc::QcGzController")
