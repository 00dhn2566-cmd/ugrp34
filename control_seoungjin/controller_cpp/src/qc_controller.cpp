// qc_controller.cpp — 제어 체인 구현 (Simulink Maneuver Controller 대응)
//
// 체인 (실측 아키텍처, TUNING_STATUS/세션 기록 기반):
//   위치: err(world) → 축별 클램프 ±posErrSat → PID_pos → RBI 회전(world→body)
//         → 경로 필터 → Dir P/R(±1/9.81) → ×err2rp(2.4) → ±60° 클램프 → pitch/roll 명령
//   자세: cmd − 측정필터(측정 rpy) → PID_att(음수 게인, ±800) → 믹서
//   yaw/고도: 동일 패턴. 고도 출력 + 바이어스(56.5 + 44.4·m_pkg) → 모터 기준속도
//   모터: (ref − meas) PI (±0.25)
//
// [TODO-verify] 배선 세부(부호/필터 위치/2π 스케일)는 dump_controller_spec.m +
//               골든 트레이스 대조로 확정한다. 여기 초안은 구조를 고정하는 용도.

#include "qc_controller.hpp"

namespace qc {

static constexpr double kPi = 3.14159265358979323846;

void qc_bind(QcState& st, const QcConfig& c) {
    const QcScales s = qc_scales(c);

    st.pidPosX = Pid{c.kpPos, c.kiPos, c.kdPos, c.filtDPos, 0};
    st.pidPosY = Pid{c.kpPos, c.kiPos, c.kdPos, c.filtDPos, 0};
    st.pidPosZ = Pid{c.kpPos, c.kiPos, c.kdPos, c.filtDPos, 0};

    st.pidAttP = Pid{c.kpAtt * s.sT * s.sIa, c.kiAtt * s.sT * s.sIa,
                     c.kdAtt * s.sT * s.sIa, c.filtDAtt, c.limAtt};
    st.pidAttR = st.pidAttP;

    st.pidYaw = Pid{c.kpYaw * s.sQ * s.sIz, c.kiYaw * s.sQ * s.sIz,
                    c.kdYaw * s.sQ * s.sIz, c.filtDYaw, c.limYaw};
    st.pidAlt = Pid{c.kpAlt * s.sT * s.sM, c.kiAlt * s.sT * s.sM,
                    c.kdAlt * s.sT * s.sM, c.filtDAlt, c.limAlt};

    for (int i = 0; i < 4; ++i)
        st.pidMot[i] = Pid{c.kpMot, c.kiMot, 0, 100, c.limMot};

    st.fMeasP.tau = c.tauMeasAtt;
    st.fMeasR.tau = c.tauMeasAtt;
    for (auto& f : st.fPosPath) f.tau = c.tauPosPath;
}

QcOutput qc_step(QcState& st, const QcConfig& c, const QcInput& in, double dt) {
    const QcScales s = qc_scales(c);
    QcOutput out{};

    // ---- 위치 루프 (world) ----
    double e[3];
    for (int i = 0; i < 3; ++i) {
        e[i] = in.refPos[i] - in.measPos[i];
        e[i] = clamp(e[i], -s.posErrSat, +s.posErrSat);   // PosErr Sat X/Y/Z (15차 채택)
    }
    double u[3] = { st.pidPosX.step(e[0], dt),
                    st.pidPosY.step(e[1], dt),
                    st.pidPosZ.step(e[2], dt) };

    // RBI 회전: world → body (측정 rpy 사용; z-오차 누수 봉인은 위 축별 클램프가 담당)
    // [TODO-verify] 원본 Matrix Multiply의 회전 규약(ZYX 가정) 및 완전성
    const double cy = std::cos(in.measRpy[2]), sy = std::sin(in.measRpy[2]);
    double bx =  cy * u[0] + sy * u[1];    // yaw만 반영한 수평 성분 (1차 근사)
    double by = -sy * u[0] + cy * u[1];

    bx = st.fPosPath[0].step(bx, dt);
    by = st.fPosPath[1].step(by, dt);

    // Dir P/R → err2rp → ±60° 클램프
    const double limCmd = c.cmdLimDeg * kPi / 180.0;
    out.cmdPitch = clamp( bx * c.dirGain * c.pos2att, -limCmd, +limCmd);
    out.cmdRoll  = clamp(-by * c.dirGain * c.pos2att, -limCmd, +limCmd);  // [TODO-verify 부호]

    // ---- 자세 루프 ----
    const double measP = st.fMeasP.step(in.measRpy[1], dt);  // Filter Pitch (기전 ②: 최대 7° 지연 실측)
    const double measR = st.fMeasR.step(in.measRpy[0], dt);
    const double uP = st.pidAttP.step(out.cmdPitch - measP, dt);
    const double uR = st.pidAttR.step(out.cmdRoll  - measR, dt);

    // ---- yaw / 고도 ----
    const double uY = st.pidYaw.step(in.refYaw - in.measRpy[2], dt);
    const double uA = st.pidAlt.step(in.refPos[2] - in.measAlt, dt);

    // ---- 추력 바이어스 + 믹서 ----
    // [TODO-verify] ref = 2π×(고도PID + BiasChassis + BiasLoad·m_pkg) 구조 (9차 규명) 재확인
    const double base = uA + c.biasChassis + c.biasLoadGain * c.pkgMass;
    for (int i = 0; i < 4; ++i) {
        // mixDir: 모터 2·3 내장 역회전 (실측 w 음수) — 크기 성분에 방향 부호를 입힘
        out.motorRef[i] = c.mixDir[i] * 2.0 * kPi *
            (base + c.mixPitch[i] * uP + c.mixRoll[i] * uR + c.mixYaw[i] * uY);
        out.motorCmd[i] = st.pidMot[i].step(c.mixDir[i] * (out.motorRef[i] - in.motorSpd[i]), dt);
    }
    return out;
}

} // namespace qc
