// 성진 제어기(qc_controller/qc_motor)를 파이썬(ctypes)에서 부르기 위한 얇은 C 래퍼.
//
// **그의 파일은 안 고친다.** 여기서 헤더만 include 하고 extern "C" 진입점을 연다.
// 접합 지점은 그가 main_trace.cpp:198 에 [PLANT HOOK] 주석으로 지정해 둔 그대로:
//   "아래 모터 플랜트의 thrust/dragQ 를 기체에 인가할 것. 모터 속도 피드백은
//    이미 플랜트(qc_motor) 적분값을 쓴다 (mixDir 부호 복원 포함)."
//
// 빌드: control/build.sh
#include <cmath>
#include <cstring>

#include "qc_controller.hpp"
#include "qc_motor.hpp"

namespace {

struct Session {
    qc::QcConfig cfg;
    qc::QcState st;
    qc::MotorParams mp;
    qc::Motor motors[4];
    bool bound = false;
};

Session g;   // 단일 세션 — 데모용으로 충분 (동시 다중 기체는 필요해지면 핸들화)

}  // namespace

extern "C" {

// profile: 0=Precision 1=Balanced 2=Agile
// altCmdSat < 0 이면 그의 기본값(30) 유지. 양수면 그 값으로 덮어쓴다.
//
// 왜 이걸 열어두나: qc_controller.cpp:89 는
//     base = clamp(uA + 56.5 + 44.4*pkgMass, ±altCmdSat)
// 인데 기본값 pkgMass=1 에서 괄호 안이 100.9 rev/s 라 ±30 클램프에 상시 포화한다.
// 그러면 고도 PID(uA)가 base 에 전혀 반영되지 못하고 추력이 호버의 8%에 묶인다.
// 한편 100.9 rev/s × 2π = 634 rad/s 는 그가 README 에 적은 호버 평형과 정확히 같다.
// 즉 "클램프는 PID 출력에만 걸고 바이어스는 그 뒤에 더한다"가 맞는 배선으로 보인다.
// 그의 HANDOFF 가 [TODO-verify] 로 남긴 "고도 PID 클램프: limit_altitude=10 vs
// .slx ±30 rev/s 관계" 가 바로 이 항목이다. 그의 파일은 안 고치고 설정값만 연다.
void qcb_init(double droneMass, double pkgMass, double pkgSz, int profile,
              double altCmdSat) {
    g = Session{};
    g.cfg.droneMass = droneMass;
    g.cfg.pkgMass = pkgMass;
    g.cfg.pkgSize[0] = g.cfg.pkgSize[1] = g.cfg.pkgSize[2] = pkgSz;
    if (altCmdSat > 0) g.cfg.altCmdSat = altCmdSat;
    qc::Profile p = profile == 0 ? qc::Profile::Precision
                  : profile == 2 ? qc::Profile::Agile
                                 : qc::Profile::Balanced;
    qc::qc_apply_profile(g.cfg, p);   // 프로파일 먼저 (질량 의존 게인)
    qc::qc_bind(g.st, g.cfg);         // 그 다음 바인드 — 그의 주석대로의 순서
    g.bound = true;
}

// 물성 정규화 결과를 파이썬에서 확인 (스모크가 1.000000 을 찍는 그 값들)
void qcb_phys(double* out_Iatt, double* out_Iyaw, double* out_mtot) {
    qc::PhysOut ph = qc::qc_phys(g.cfg.droneMass, g.cfg.pkgMass, g.cfg.pkgSize);
    *out_Iatt = ph.I_att; *out_Iyaw = ph.I_yaw; *out_mtot = ph.m_tot;
}

void qcb_arm_geometry(double* out_rarm, double* out_D) {
    *out_rarm = 0.225 / 1.4142135623730951;   // qc_phys 의 r_arm 과 동일 정의
    *out_D = g.mp.D;
}

// 한 스텝. 모터 속도 피드백은 내부 플랜트 적분값을 쓴다 (main_trace.cpp 와 동일).
//   refPos[3], refYaw, measPos[3], measRpy[3], dt
//   -> thrust[4] (N), dragQ[4] (N·m), w[4] (rad/s), cmd[4] (정규화), cmdPitchRoll[2]
void qcb_step(const double* refPos, double refYaw,
              const double* measPos, const double* measRpy, double dt,
              double* out_thrust, double* out_dragQ, double* out_w,
              double* out_cmd, double* out_cmdPR) {
    qc::QcInput in{};
    std::memcpy(in.refPos, refPos, 3 * sizeof(double));
    in.refYaw = refYaw;
    std::memcpy(in.measPos, measPos, 3 * sizeof(double));
    std::memcpy(in.measRpy, measRpy, 3 * sizeof(double));
    in.measAlt = measPos[2];
    for (int i = 0; i < 4; ++i) in.motorSpd[i] = g.cfg.mixDir[i] * g.motors[i].w;

    qc::QcOutput out = qc::qc_step(g.st, g.cfg, in, dt);

    for (int i = 0; i < 4; ++i) {
        qc::MotorOut mo = g.motors[i].step(out.motorCmd[i], g.mp, dt);
        out_thrust[i] = mo.thrust;
        out_dragQ[i] = mo.dragQ;
        out_w[i] = mo.w;
        out_cmd[i] = out.motorCmd[i];
    }
    out_cmdPR[0] = out.cmdPitch;
    out_cmdPR[1] = out.cmdRoll;
}

// mixDir 를 파이썬 쪽에서 알아야 yaw 반토크 부호를 맞출 수 있다.
void qcb_mixdir(double* out4) {
    for (int i = 0; i < 4; ++i) out4[i] = g.cfg.mixDir[i];
}

// --- 임의 오버라이드 진입점 -------------------------------------------------
// 그가 [TODO-verify] 로 남긴 항목들은 전부 QcConfig/MotorParams **멤버**다.
// 즉 로직이 아니라 설정값이라, 그의 소스를 한 줄도 안 고치고 여기서 덮을 수 있다.
// qcb_init 직후 · 비행 전에 부를 것 (믹서표와 모터 파라미터는 qc_bind 대상이 아니다).

// 믹서 차동 부호표 — 그의 최우선 미확정 항목. 각 인자는 길이 4 배열 (NULL 이면 유지).
void qcb_set_mix(const double* pitch4, const double* roll4,
                 const double* yaw4, const double* dir4) {
    for (int i = 0; i < 4; ++i) {
        if (pitch4) g.cfg.mixPitch[i] = pitch4[i];
        if (roll4)  g.cfg.mixRoll[i]  = roll4[i];
        if (yaw4)   g.cfg.mixYaw[i]   = yaw4[i];
        if (dir4)   g.cfg.mixDir[i]   = dir4[i];
    }
}

// 모터 플랜트 — 기본값에서는 최대추력이 정확히 mg 라 상승 여력이 0 이다.
// 인자가 <= 0 이면 해당 항목 유지.
void qcb_set_motor(double maxTorque, double limitCmd, double Ct, double Cq,
                   double Vbatt, double maxPower) {
    if (maxTorque > 0) g.mp.maxTorque = maxTorque;
    if (limitCmd  > 0) g.mp.limitCmd  = limitCmd;
    if (Ct        > 0) g.mp.Ct        = Ct;
    if (Cq        > 0) g.mp.Cq        = Cq;
    if (Vbatt     > 0) g.mp.Vbatt     = Vbatt;
    if (maxPower  > 0) g.mp.maxPower  = maxPower;
}

// 명령 경로 상수 — Dir P/R 부호와 각도 리밋도 미확정 항목이다.
void qcb_set_cmd(double dirGain, double cmdLimDeg) {
    if (dirGain != 0) g.cfg.dirGain = dirGain;
    if (cmdLimDeg > 0) g.cfg.cmdLimDeg = cmdLimDeg;
}

// 모터 스핀 상태를 직접 세팅 (패드 스핀업 대신 평형 회전수에서 출발시킬 때)
void qcb_preset_w(double w) {
    for (int i = 0; i < 4; ++i) g.motors[i].w = w;
}

// 게인 오버라이드. 인자가 NaN 이면 해당 항목 유지. **바꾼 뒤 qc_bind 를 다시 부른다**
// (게인은 스케일과 곱해져 QcState 의 Pid 로 들어가므로 재바인드가 필수).
// 자세 게인이 음수인 것은 의도된 설계다 (플랜트 이득 b=-0.0296 음수) — 부호를 뒤집으면
// 그의 원본 플랜트에서는 즉시 발산한다. 다만 우리 PyBullet 플랜트는 부호 규약이
// 다를 수 있어서, 여기서는 부호까지 포함해 자유롭게 덮을 수 있게 열어 둔다.
static bool nn(double v) { return v == v; }   // NaN 아님

void qcb_set_gains(double kpPos, double kiPos, double kdPos,
                   double kpPosZ, double kdPosZ, double pos2att,
                   double kpAtt, double kiAtt, double kdAtt, double limAtt,
                   double kpAlt, double kiAlt, double kdAlt, double limAlt,
                   double kpYaw, double kiYaw, double kdYaw) {
    if (nn(kpPos))  g.cfg.kpPos = kpPos;
    if (nn(kiPos))  g.cfg.kiPos = kiPos;
    if (nn(kdPos))  g.cfg.kdPos = kdPos;
    if (nn(kpPosZ)) g.cfg.kpPosZ = kpPosZ;
    if (nn(kdPosZ)) g.cfg.kdPosZ = kdPosZ;
    if (nn(pos2att))g.cfg.pos2att = pos2att;
    if (nn(kpAtt))  g.cfg.kpAtt = kpAtt;
    if (nn(kiAtt))  g.cfg.kiAtt = kiAtt;
    if (nn(kdAtt))  g.cfg.kdAtt = kdAtt;
    if (nn(limAtt)) g.cfg.limAtt = limAtt;
    if (nn(kpAlt))  g.cfg.kpAlt = kpAlt;
    if (nn(kiAlt))  g.cfg.kiAlt = kiAlt;
    if (nn(kdAlt))  g.cfg.kdAlt = kdAlt;
    if (nn(limAlt)) g.cfg.limAlt = limAlt;
    if (nn(kpYaw))  g.cfg.kpYaw = kpYaw;
    if (nn(kiYaw))  g.cfg.kiYaw = kiYaw;
    if (nn(kdYaw))  g.cfg.kdYaw = kdYaw;
    qc::qc_bind(g.st, g.cfg);       // 재바인드 필수
}

// 제어기 자체 명령 클램프 limMot / yaw 차동 클램프 limYaw. 둘 다 재바인드 필요.
// limMot 이 진짜 병목이다: 모터 플랜트의 limitCmd 를 올려도 제어기가 0.25 로 먼저
// 자르므로 소용없다. 최대추력 = min(limMot, limitCmd) x maxTorque 로 결정된다.
void qcb_set_lims(double limMot, double limYaw) {
    if (limMot > 0) g.cfg.limMot = limMot;
    if (limYaw > 0) g.cfg.limYaw = limYaw;
    qc::qc_bind(g.st, g.cfg);
}

// 모터 PI 게인. yaw 는 오직 모터 회전수 변화로만 모멘트가 생기므로 (자세는 추력
// 차이로 직접 모멘트가 나온다) 이 루프가 느리면 yaw 만 죽는다. 기본 kpMot=0.00375.
void qcb_set_motor_pi(double kpMot, double kiMot) {
    if (kpMot > 0) g.cfg.kpMot = kpMot;
    if (kiMot > 0) g.cfg.kiMot = kiMot;
    qc::qc_bind(g.st, g.cfg);
}

// 현재 게인을 읽어온다 (기본값 확인·상대 튜닝용)
void qcb_get_gains(double* out17) {
    const qc::QcConfig& c = g.cfg;
    const double v[17] = {c.kpPos, c.kiPos, c.kdPos, c.kpPosZ, c.kdPosZ, c.pos2att,
                          c.kpAtt, c.kiAtt, c.kdAtt, c.limAtt,
                          c.kpAlt, c.kiAlt, c.kdAlt, c.limAlt,
                          c.kpYaw, c.kiYaw, c.kdYaw};
    for (int i = 0; i < 17; ++i) out17[i] = v[i];
}

// 진단: 직전 스텝의 내부 신호 (고도 루프가 왜 못 잡는지 보려면 base 를 봐야 한다)
void qcb_debug(double* out3) {
    out3[0] = g.cfg.altCmdSat;
    out3[1] = g.cfg.biasChassis + g.cfg.biasLoadGain * g.cfg.pkgMass;
    out3[2] = g.cfg.limAlt;
}

// 현재 모터 플랜트로 낼 수 있는 최대 추력 [N] (4개 합) — 추력대중량비 계산용
// 명령 상한은 **제어기의 limMot(0.25)** 이 먼저 건다 — 플랜트의 limitCmd 를 쓰면
// 제어기가 결코 내지 않는 명령으로 재게 되어 과대평가된다.
double qcb_max_thrust() {
    const double cmd = std::fmin(g.cfg.limMot, g.mp.limitCmd);
    qc::Motor m;
    for (int k = 0; k < 40000; ++k) m.step(cmd, g.mp, 0.001);
    qc::MotorOut o = m.step(cmd, g.mp, 0.001);
    return 4.0 * o.thrust;
}

}  // extern "C"
