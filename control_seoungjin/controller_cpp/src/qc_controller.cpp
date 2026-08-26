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
    st.pidPosZ = Pid{c.kpPosZ, c.kiPos, c.kdPosZ, c.filtDPos, 0};   // 18차 z분리

    // 18차: 자세/고도 질량 의존 = 1차식(sAMass/sZMass), yaw = 질량 동결(sQ만)
    st.pidAttP = Pid{c.kpAtt * s.sT * s.sAMass, c.kiAtt * s.sT * s.sAMass,
                     c.kdAtt * s.sT * s.sAMass, c.filtDAtt, c.limAtt};
    st.pidAttR = st.pidAttP;

    st.pidYaw = Pid{c.kpYaw * s.sQ, c.kiYaw * s.sQ,
                    c.kdYaw * s.sQ, c.filtDYaw, c.limYaw};
    st.pidYaw.antiWindup = c.yawAntiWindup;    // Simulink qc_antiwindup_apply('clamping') 과 동기
    st.gov.on      = c.govOn;                  // Simulink qc_clock_gov_apply 와 동기
    st.gov.rf      = c.govRf;
    st.gov.rs      = c.govRs;
    st.gov.sMin    = c.govSmin;
    st.gov.ws      = c.govWs;
    st.gov.tauRho  = c.govTauRho;
    st.gov.tauPsi  = c.govTauPsi;
    st.gov.psiStop = c.govPsiStop;
    st.gov.bind();
    st.yawDistI.gmax     = c.yawDistGmax;      // 1.0 이면 항등 (기존 동작)
    st.yawDistI.e0       = c.yawDistE0;
    st.yawDistI.e1       = c.yawDistE1;
    st.yawDistI.tau      = c.yawDistTau;
    st.yawDistI.rateGate = c.yawDistRateGate;
    st.yawDistI.relax    = c.yawDistRelax;
    st.pidAlt = Pid{c.kpAlt * s.sT * s.sZMass, c.kiAlt * s.sT * s.sZMass,
                    c.kdAlt * s.sT * s.sZMass, c.filtDAlt, c.limAlt};

    for (int i = 0; i < 4; ++i)
        st.pidMot[i] = Pid{c.kpMot, c.kiMot, 0, 100, c.limMot};

    st.fMeasP.tau = c.tauMeasAtt;
    st.fMeasR.tau = c.tauMeasAtt;
    st.fMeasY.tau = c.tauMeasYaw;
    st.fMeasZ.tau = c.tauMeasAlt;
    for (auto& f : st.fPosPath) f.tau = c.tauPosPath;
}

QcOutput qc_step(QcState& st, const QcConfig& c, const QcInput& in, double dt) {
    const QcScales s = qc_scales(c);
    QcOutput out{};

    // ---- 위치 루프 (world) ----
    double e[3];
    for (int i = 0; i < 3; ++i) {
        const double lim = (i == 2) ? s.posErrSatZ : s.posErrSat;  // 18차 z분리 클램프
        e[i] = in.refPos[i] - in.measPos[i];
        e[i] = clamp(e[i], -lim, +lim);                   // PosErr Sat X/Y/Z (15차 채택)
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

    // ---- yaw / 고도 (측정 필터: 덤프 확정 배선) ----
    const double measY = st.fMeasY.step(in.measRpy[2], dt);   // Filter Yaw (0.01)
    const double measZ = st.fMeasZ.step(in.measAlt, dt);      // Filter pz (0.01)
    // yaw 외란 적응 적분: 참조 각속도로 슬루 여부를 가른 뒤, 지속 오차에만 적분률을 올린다.
    // (gmax=1 기본값이면 refRate 계산만 돌고 kiScale 은 항상 1 -> 골든 트레이스 불변)
    const double eYaw = wrapPi(in.refYaw - measY);                     // yaw 오차 랩 (18차: yaw 입력 지원)
    const double refRate = st.refYawFirst ? 0.0 : wrapPi(in.refYaw - st.refYawPrev) / dt;
    st.refYawPrev = in.refYaw; st.refYawFirst = false;
    st.pidYaw.kiScale = st.yawDistI.step(eYaw, refRate, dt);
    const double uY = st.pidYaw.step(eYaw, dt);

    // ---- 속도 조속기 + 진단 (상위 capability 입력) ----
    // govOn=false 면 s≡1 이라 여기 계산은 출력에 영향을 주지 않는다 (진단만).
    out.uYaw   = uY;
    out.eYaw   = eYaw;
    const double rhoYaw = (c.limYaw > 0.0) ? std::fabs(uY) / c.limYaw : 0.0;
    const double uAttMax = std::fabs(uP) > std::fabs(uR) ? std::fabs(uP) : std::fabs(uR);
    const double rhoAtt = (c.limAtt > 0.0) ? uAttMax / c.limAtt : 0.0;
    out.rho    = rhoYaw > rhoAtt ? rhoYaw : rhoAtt;   // INTERFACE_SPEC §5b 정의
    out.sClock = st.gov.step(uY, c.limYaw, eYaw, dt);
    out.rhoEff = st.gov.rhoEff;
    st.tauClock += out.sClock * dt;

    // ── 지연 -> 상위 보고 스펙 (관측 전용; 제어 출력에 영향 없음) ──────────
    // 지연 추적은 조속기와 **분리**되어 있다. 조속기는 외란(권한 점유)에 반응하는
    // 즉시 반사이고, 지연 감쇄는 계획을 다시 짜야 하는 느린 판단이라 성격이 다르다.
    // 여기서는 재기만 하고, 실제로 느리게 갈지는 상위가 새 궤적으로 결정한다.
    // 보고는 ~5 Hz 로 데시메이션한다 (QcConfig::specRateHz 주석 참조 — 제어 주기로
    // 돌리면 EMA 시정수가 표본수 기준이라 의미가 달라진다).
    if (c.specOn) {
        // 회복 감시는 **제어 주기마다** 관측한다. 지연 추적기와 달리 시정수가
        // 표본수가 아니라 **시간**(dt)으로 정의돼 있어 주기가 달라져도 뜻이 같다.
        // 판단은 아래 데시메이션 지점에서 한 번만.
        if (c.recOn) {
            st.rec.trackBandM = c.recTrackBandM;
            st.rec.settleS    = c.recSettleS;
            double e = 0.0;
            for (int i = 0; i < 3; ++i) {
                const double d = in.refPos[i] - in.measPos[i];
                e += d * d;
            }
            st.rec.observe(std::sqrt(e), in.refWithinLimits, dt);
        }
        if (in.measAgeS > st.specAgeMax) st.specAgeMax = in.measAgeS;
        st.specAcc += dt;
        const double period = (c.specRateHz > 1e-9) ? (1.0 / c.specRateHz) : dt;
        if (st.specAcc >= period) {
            st.specAcc -= period;
            const double tauPos = st.lat.update(st.specAgeMax);
            st.specAgeMax = 0.0;
            const double sRec = c.recOn ? st.rec.decide(c.bridgeLeadS) : 1.0;
            st.specLast = qc_spec_report(st.specRule,
                                         c.specBaseV, c.specBaseA, c.specBaseJ,
                                         c.specBaseSnap, c.specLimitScale,
                                         out.sClock, tauPos, c.latencyAttS, sRec);
        }
        out.spec = st.specLast;
    }
    const double uA = st.pidAlt.step(in.refPos[2] - measZ, dt);

    // ---- 추력 바이어스 + 2단 클램프 + 믹서 ----
    // 2π 스케일·바이어스 구조는 실비행 재생으로 실증.
    //
    // [2026-08-26 정정] Alt Cmd Sat(±30)은 **바이어스를 더하기 전**, 고도 PID 출력에만
    // 걸린다. Simulink 배선이 `cmd -> Alt Cmd Sat -> Bias Chassis` 이다
    // (diagnose/bake_tuned_model.m (3) 고도 클램프: sat 출력이 Bias Chassis 의 입력).
    // 이전 코드는 바이어스를 합산한 뒤 클램프해서 base 가 항상 30 rev/s 로 잘렸고,
    // 그러면 추력이 1.97 N 밖에 안 나와 22.29 N 인 1 kg 기체는 **뜰 수가 없었다**.
    // 고친 뒤: base = 100.9 rev/s -> 추력 22.26 N (필요 22.29 N, 0.1% 일치) =
    // SESSIONS_BOARD 가 기록한 호버 평형 634 rad/s 와 일치.
    // 교차 확인: 0 kg 에서 필요한 base 75.6 rev/s 가 qc_mass_lerp 의 0 kg 앵커
    // biasChassis=75.5 와 0.1% 로 맞는다 (독립 유도인데 같은 값).
    // ⚠ 이 정정은 motorRef/motorCmd 채널의 골든 트레이스를 바꾼다 (cmd_pitch/cmd_roll
    //   위치 체인은 불변). 모터 채널 대조는 다시 떠야 한다.
    const double base = clamp(uA, -c.altCmdSat, c.altCmdSat)
                        + c.biasChassis + c.biasLoadGain * c.pkgMass;
    for (int i = 0; i < 4; ++i) {
        // mixDir: 모터 2·3 내장 역회전 (실측 w 음수) — 크기 성분에 방향 부호를 입힘
        out.motorRef[i] = c.mixDir[i] * 2.0 * kPi *
            (base + c.mixPitch[i] * uP + c.mixRoll[i] * uR + c.mixYaw[i] * uY);
        out.motorCmd[i] = st.pidMot[i].step(c.mixDir[i] * (out.motorRef[i] - in.motorSpd[i]), dt);
    }
    return out;
}

} // namespace qc
