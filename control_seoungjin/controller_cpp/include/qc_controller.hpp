// qc_controller.hpp — Simulink Maneuver Controller의 C++ 이식 (17차 착수)
//
// 원본: controller/Quadcopter-Drone-Model-Simscape/Models/quadcopter_package_delivery.slx
//       (구운 모델) + Scripts_Data/quadcopter_package_parameters.m
//
// 설계 규약 (임베디드 강등 가능한 보수적 C++):
//   - 초기화 이후 힙 할당 없음 / 예외 없음 / 가상함수 없음
//   - 상태는 전부 명시적 struct, 매 스텝 qc_step() 순수 호출
//   - 게인·물성은 QcConfig로 분리 (물성 정규화 sT/sQ/sIa/sIz/sM 포함 — parameters.m §17차)
//
// 검증 계약: 구운 Simulink 모델을 정답 플랜트로, 같은 입력의 골든 트레이스와 대조.
//   [TODO-verify] 표시 항목은 dump_controller_spec.m 결과로 확정할 것 — 손으로 "수정" 금지.
//   특히 자세 게인 음수는 플랜트 이득이 음수라 의도된 것 (TUNING_STATUS 참조).

#pragma once
#include <cmath>
#include <cstdint>

namespace qc {

// ---------- 기초 부품 ----------

inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// 병렬형 PID + 필터드 미분 (Simulink PID 블록 대응: P + I/s + D·N/(1+N/s))
// anti-windup 없음 — 원본과 동일 (출력만 클램프, 적분기는 계속 적분). TUNING_STATUS 명시.
struct Pid {
    // 파라미터
    double kp = 0, ki = 0, kd = 0;
    double N = 100;          // 미분 필터 계수 (filtD)
    double outLim = 0;       // 출력 클램프 ±outLim (0이면 무제한)
    // 상태
    double integ = 0;        // 적분기
    double dFilt = 0;        // 필터드 미분 상태
    double ePrev = 0;
    bool   first = true;

    void reset() { integ = 0; dFilt = 0; ePrev = 0; first = true; }

    double step(double e, double dt) {
        integ += ki * e * dt;                       // 전진 오일러 적분
        double de = first ? 0.0 : (e - ePrev) / dt; // 후방차분
        first = false;
        ePrev = e;
        double a = 1.0 / (1.0 + N * dt);            // 미분 1차 필터 (후방 오일러)
        dFilt = a * dFilt + (1.0 - a) * kd * de;
        double u = kp * e + integ + dFilt;
        if (outLim > 0) u = clamp(u, -outLim, outLim);
        return u;
    }
};

// 1차 저역 필터 (Simulink filtM_* 측정 필터 대응; tau = 시정수)
// [TODO-verify] Filter Pitch/Roll TransferFcn 계수 — filtM_attitude=0.01 가정, 덤프로 확정
struct Lpf1 {
    double tau = 0.01;
    double y = 0;
    bool first = true;
    void reset() { y = 0; first = true; }
    double step(double u, double dt) {
        if (first) { y = u; first = false; return y; }  // 초기 과도 방지
        y += dt / (tau + dt) * (u - y);                 // 후방 오일러
        return y;
    }
};

// ---------- 물성 정규화 (parameters.m qc_phys의 1:1 이식) ----------
// 섀시 실측(Inertia Sensor) + 로터 기하 추정 + 패키지 해석항 합성 (CoM 기준).
// parameters.m 쪽이 바뀌면 여기도 함께 갱신할 것.

struct PhysOut { double I_att, I_yaw, m_tot; };

inline PhysOut qc_phys(double m_drone, double m_pkg, const double pkgSz[3]) {
    const double m_ch  = 0.9650346;
    const double z_ch  = +0.0038181;
    const double I_ch[3] = {1.488e-3, 1.538e-3, 2.399e-3};
    const double m_rot = m_drone - m_ch;
    const double r_arm = 0.225 / std::sqrt(2.0);
    const double z_rot = +0.02;
    const double z_pkg = -0.012 - pkgSz[2] / 2.0;
    const double m_tot = m_drone + m_pkg;
    const double z_cg  = (m_ch * z_ch + m_rot * z_rot + m_pkg * z_pkg) / m_tot;
    const double dch2  = (z_ch - z_cg) * (z_ch - z_cg);
    const double drot2 = (z_rot - z_cg) * (z_rot - z_cg);
    const double dpkg2 = (z_pkg - z_cg) * (z_pkg - z_cg);
    const double Ix = I_ch[0] + m_ch * dch2 + m_rot * r_arm * r_arm + m_rot * drot2
                    + m_pkg / 12.0 * (pkgSz[1]*pkgSz[1] + pkgSz[2]*pkgSz[2]) + m_pkg * dpkg2;
    const double Iy = I_ch[1] + m_ch * dch2 + m_rot * r_arm * r_arm + m_rot * drot2
                    + m_pkg / 12.0 * (pkgSz[0]*pkgSz[0] + pkgSz[2]*pkgSz[2]) + m_pkg * dpkg2;
    const double I_att = 0.5 * (Ix + Iy);
    const double I_yaw = I_ch[2] + m_rot * 2.0 * r_arm * r_arm
                       + m_pkg / 12.0 * (pkgSz[0]*pkgSz[0] + pkgSz[1]*pkgSz[1]);
    return {I_att, I_yaw, m_tot};
}

// ---------- 설정 ----------

struct QcConfig {
    // 물성 (현재 기체) — 갱신 지점
    double droneMass = 1.2726;
    double pkgMass   = 1.0;
    double pkgSize[3] = {0.14, 0.14, 0.14};
    double kThrust = 9.79, kDrag = 0.597;

    // 앵커 (튜닝 당시 — 절대 갱신 금지)
    double kThrustRef = 9.79, kDragRef = 0.597;
    double droneMassRef = 1.2726, pkgMassRef = 1.0;
    double pkgSizeRef[3] = {0.14, 0.14, 0.14};

    // 기저 게인 (parameters.m 채택치; 스케일 곱하기 전)
    // 위치 (16차 현행 — 위치 미세조정 확정 시 갱신 예정)
    double kpPos = 8, kiPos = 0.04, kdPos = 3.2, filtDPos = 100;
    double pos2att = 2.4;                    // err2rp
    double posErrSatCoef = 1.2;              // posErrSat = 1.2/kpPos (곱 불변식)
    // 자세 (16차 채택: -85/-10/-127.5, filtD 2500) — 음수 필수 (플랜트 이득 음수)
    double kpAtt = -85, kiAtt = -10, kdAtt = -127.5, filtDAtt = 2500, limAtt = 800;
    // yaw (12차)
    double kpYaw = 15, kiYaw = 1.5, kdYaw = 4, filtDYaw = 100, limYaw = 20;
    // 고도 (11~12차)
    double kpAlt = 0.5, kiAlt = 0.1, kdAlt = 0.15, filtDAlt = 1000, limAlt = 10;
    // 모터 PI (per-motor 속도 루프)
    double kpMot = 0.00375, kiMot = 4.5e-4, limMot = 0.25;

    // 측정 필터 시정수 [TODO-verify: Simulink filtM_* 대응 및 실제 배선 위치 확인]
    double tauMeasAtt = 0.01;    // filtM_attitude — Filter Pitch/Roll
    double tauPosPath = 0.005;   // filtM_position — 위치 명령 경로 Filter

    // 명령 경로 상수 [TODO-verify: Dir P/R 부호, Pitch/Roll Limit]
    double dirGain = 1.0 / 9.81; // Dir P/R (±1/9.81)
    double cmdLimDeg = 60.0;     // Pitch/Roll Limit ±60°

    // 추력 바이어스 (구운 모델 재스케일 계열) [TODO-verify: 2π 배선, Bias Load 식]
    double biasChassis = 56.5;               // rev/s
    double biasLoadGain = 44.4;              // × pkgMass (44.4·pkgSize³·pkgDensity = 44.4·m_pkg)

    // 믹서 부호표 [TODO-verify: 차동 성분 부호는 덤프/골든트레이스로 확정]
    //             모터:      1     2     3     4
    double mixPitch[4] = { +1,   +1,   -1,   -1 };
    double mixRoll[4]  = { +1,   -1,   -1,   +1 };
    double mixYaw[4]   = { +1,   -1,   +1,   -1 };
    // 모터 회전 방향 (실비행 재생으로 확정: 모터 2·3 내장 역회전 — 실측 w 부호가 음수.
    // 9차 "믹서 원래 부호 + direction 전부 Positive" 구성에서 모터 2,3이 스스로 음회전)
    double mixDir[4]   = { +1,   -1,   -1,   +1 };
};

// 스케일 적용된 실효 게인 계산 (parameters.m 로직 대응)
struct QcScales { double sT, sQ, sIa, sIz, sM, posErrSat; };

inline QcScales qc_scales(const QcConfig& c) {
    PhysOut now = qc_phys(c.droneMass, c.pkgMass, c.pkgSize);
    PhysOut ref = qc_phys(c.droneMassRef, c.pkgMassRef, c.pkgSizeRef);
    QcScales s;
    s.sT  = c.kThrustRef / c.kThrust;
    s.sQ  = c.kDragRef / c.kDrag;
    s.sIa = now.I_att / ref.I_att;
    s.sIz = now.I_yaw / ref.I_yaw;
    s.sM  = now.m_tot / ref.m_tot;
    s.posErrSat = c.posErrSatCoef / c.kpPos;
    return s;
}

// ---------- 제어기 본체 ----------

struct QcInput {
    double refPos[3];    // 궤적 참조 위치 (world, m) — 스무더+게이트 통과 전제!
    double refYaw;       // 참조 yaw (rad)
    double measPos[3];   // 측정 위치 (world, m)
    double measRpy[3];   // 측정 roll/pitch/yaw (rad)
    double measAlt;      // 측정 고도 (m) — 보통 measPos[2]
    double motorSpd[4];  // 측정 모터 속도 (rev/s) — 모터 PI 루프용
};

struct QcOutput {
    double cmdPitch, cmdRoll;   // 위치→자세 명령 (rad)
    double motorRef[4];         // 믹서 후 모터 속도 참조 (rev/s)
    double motorCmd[4];         // 모터 PI 출력 (정규화 토크 명령)
};

struct QcState {
    Pid pidPosX, pidPosY, pidPosZ;   // 위치 3축
    Pid pidAttP, pidAttR;            // 자세 pitch/roll
    Pid pidYaw, pidAlt;
    Pid pidMot[4];
    Lpf1 fMeasP, fMeasR;             // 자세 측정 필터
    Lpf1 fPosPath[3];                // 위치 명령 경로 필터
    void reset() {
        pidPosX.reset(); pidPosY.reset(); pidPosZ.reset();
        pidAttP.reset(); pidAttR.reset(); pidYaw.reset(); pidAlt.reset();
        for (auto& p : pidMot) p.reset();
        fMeasP.reset(); fMeasR.reset();
        for (auto& f : fPosPath) f.reset();
    }
};

// 게인을 config+스케일로부터 상태에 주입 (초기화 시 1회, 물성 변경 시 재호출)
void qc_bind(QcState& st, const QcConfig& c);

// 한 스텝 실행. dt[s] 고정 스텝 권장 (골든 트레이스는 1kHz 기준으로 대조).
QcOutput qc_step(QcState& st, const QcConfig& c, const QcInput& in, double dt);

} // namespace qc
