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

// 각도를 [-pi, pi]로 랩 (yaw 오차용 — ref +179도/측정 -179도 = 오차 +2도가 되게)
inline double wrapPi(double a) {
    constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
    a = std::fmod(a + 3.14159265358979323846, kTwoPi);
    if (a < 0) a += kTwoPi;
    return a - 3.14159265358979323846;
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
    // 위치 x/y (프로파일 결정). z는 18차 z분리로 별도 (agile에서만 달라짐 — 이동
    // 시작/끝 기울기 추력손실로 인한 z 낙하를 z축 게인 고정으로 억제, 실증 42→1.3cm)
    double kpPos = 8, kiPos = 0.04, kdPos = 3.2, filtDPos = 100;
    double kpPosZ = 8, kdPosZ = 3.2;         // z축 위치 게인 (precision/balanced = xy와 동일값)
    double pos2att = 2.4;                    // err2rp
    double posErrSatCoef = 1.2;              // posErrSat = 1.2/kpPos (곱 불변식, z는 1.2/kpPosZ)
    // 자세 (16차 채택: -85/-10/-127.5, filtD 2500) — 음수 필수 (플랜트 이득 음수)
    double kpAtt = -85, kiAtt = -10, kdAtt = -127.5, filtDAtt = 2500, limAtt = 800;
    // yaw (12차)
    double kpYaw = 15, kiYaw = 1.5, kdYaw = 4, filtDYaw = 100, limYaw = 20;
    // 고도 (11~12차)
    double kpAlt = 0.5, kiAlt = 0.1, kdAlt = 0.15, filtDAlt = 1000, limAlt = 10;
    // 모터 PI (per-motor 속도 루프)
    double kpMot = 0.00375, kiMot = 4.5e-4, limMot = 0.25;

    // 측정 필터 시정수 (명세 덤프로 확정, 17차말 controller_spec.txt):
    //   Filter Pitch/Roll = 1/(altitude_filtM·s+1) — filtM_attitude가 아니라 0.05!
    //   (기전 ② "자세 측정 7도 지연"의 정체 = 이 50ms 필터)
    double tauMeasAtt = 0.05;    // altitude_filtM (덤프 확정)
    double tauMeasYaw = 0.01;    // yaw_filtM (Filter Yaw, 덤프 확정)
    double tauMeasAlt = 0.01;    // Filter pz (고도 측정 필터, 덤프 확정)
    double tauPosPath = 0.005;   // Position Control/Filter 3종 = 1/(filt_const·s+1)
                                 // [TODO-verify: filt_const 수치 — 골든 트레이스로 확정]

    // 명령 경로 상수 [TODO-verify: Dir P/R 부호, Pitch/Roll Limit]
    double dirGain = 1.0 / 9.81; // Dir P/R (±1/9.81)
    double cmdLimDeg = 60.0;     // Pitch/Roll Limit ±60°

    // 추력 바이어스 (구운 모델 재스케일 계열; 2π·바이어스 구조는 실비행 재생으로 실증)
    double biasChassis = 56.5;               // rev/s
    double biasLoadGain = 44.4;              // × pkgMass (덤프 확정: Bias Load = 44.4·pkgSize³·pkgDensity)
    double altCmdSat = 30.0;                 // Alt Cmd Sat ±30 (덤프 확정: PID ±10과 별개, 바이어스 합산 뒤 2단 클램프)

    // 믹서 부호표 (명세 덤프 확정: Motor Mixer Add4~7 signs = 모터1 +--+ / 2 --++ / 3 -+-+ / 4 ++++)
    // [TODO-verify: 입력 포트 순서 (pitch,roll,yaw,base) 가정 — 골든 트레이스가 최종 판정]
    //             모터:      1     2     3     4
    double mixPitch[4] = { +1,   -1,   -1,   +1 };
    double mixRoll[4]  = { -1,   -1,   +1,   +1 };
    double mixYaw[4]   = { -1,   +1,   -1,   +1 };
    // 모터 회전 방향 (실비행 재생으로 확정: 모터 2·3 내장 역회전 — 실측 w 부호가 음수.
    // 9차 "믹서 원래 부호 + direction 전부 Positive" 구성에서 모터 2,3이 스스로 음회전)
    double mixDir[4]   = { +1,   -1,   -1,   +1 };
};

// --- 컨트롤러 프로파일 (17차 사용자 설계): 상위 계층이 임무 특성으로 선택 ---
// r8 실측: 호버 지터(범인=kp)와 이동 추종의 구조적 맞교환 -> 검증된 선택지 3종.
// 전환은 임무 단위(비행 전, v1). parameters.m의 ctrl_profile switch와 1:1 동기.
// [18차 agile 재설계] 고정 24/10.8은 1kg 밖 붕괴 실측 -> 삼각 법칙 + z분리:
//   x/y: kp = 24 - 16·|m_pkg-1| (1kg 정점, 0/2kg에서 precision 수렴 - 양끝 검증점)
//   z  : 8/3.2 고정 (이동 시작/끝 기울기 추력손실 z 낙하 억제, 42->1.3cm 실증)
//   유효 0.5~2kg (1.91~3.96cm, z꼬리 <1cm). 0.5kg 미만 혼돈 구간 - precision 권장.
// 질량 의존이라 pkgMass 변경 후엔 qc_apply_profile 재호출 필요 (qc_bind 전).
enum class Profile { Precision, Balanced, Agile };

inline void qc_apply_profile(QcConfig& c, Profile p) {
    switch (p) {
        case Profile::Precision:
            c.kpPos = 8;  c.kdPos = 3.2;   break;  // 호버 0.002도/이동 4.1cm (기본)
        case Profile::Balanced:
            c.kpPos = 12; c.kdPos = 4.8;   break;  // 0.10도/2.7cm
        case Profile::Agile: {
            const double d = c.pkgMass < 1.0 ? 1.0 - c.pkgMass : c.pkgMass - 1.0;
            const double tri = d > 1.0 ? 0.0 : 1.0 - d;
            c.kpPos = 8 + 16 * tri;  c.kdPos = 3.2 + 7.6 * tri;  // 1kg: 24/10.8 (1.25cm)
            c.kpPosZ = 8; c.kdPosZ = 3.2;
            return;
        }
    }
    c.kpPosZ = c.kpPos; c.kdPosZ = c.kdPos;   // precision/balanced: z = xy 동일 (기존 거동)
    // posErrSat = 1.2/kpPos (z는 1.2/kpPosZ) 곱 불변식은 qc_scales()가 자동 연동
}

// 스케일 적용된 실효 게인 계산 (parameters.m 로직 대응)
// 18차: 자세/고도의 질량 의존은 물성비(sIa/sM)가 아니라 질량 1차식(sAMass/sZMass)을 쓴다
// — 물성비는 0kg 레짐 붕괴로 반증(refine_mass_probe), 1차식은 0~2kg 6점 검증 통과
// (refine_linear_law: 전 질량 무발산, 1kg 회귀 무결, 0.5 내삽 비열등, 2kg 외삽 우세).
// sIa/sIz/sM은 진단/비교용으로만 유지. yaw는 질량 동결(sQ만 적용, 검증 구성 그대로).
struct QcScales { double sT, sQ, sIa, sIz, sM, sAMass, sZMass, posErrSat, posErrSatZ; };

inline QcScales qc_scales(const QcConfig& c) {
    PhysOut now = qc_phys(c.droneMass, c.pkgMass, c.pkgSize);
    PhysOut ref = qc_phys(c.droneMassRef, c.pkgMassRef, c.pkgSizeRef);
    QcScales s;
    s.sT  = c.kThrustRef / c.kThrust;
    s.sQ  = c.kDragRef / c.kDrag;
    s.sIa = now.I_att / ref.I_att;
    s.sIz = now.I_yaw / ref.I_yaw;
    s.sM  = now.m_tot / ref.m_tot;
    // 질량 1차식 (18차): 배율(m) = s0 + (1-s0)·m_pkg, 0kg 앵커 실측 sA=0.75/sZ=0.56,
    // 1kg(앵커 탑재)에서 정확히 1 = 현행 채택 게인. 외삽은 2kg 캡(검증 상한).
    const double mClamped = c.pkgMass < 2.0 ? c.pkgMass : 2.0;
    s.sAMass = 0.75 + 0.25 * mClamped;
    s.sZMass = 0.56 + 0.44 * mClamped;
    s.posErrSat  = c.posErrSatCoef / c.kpPos;
    s.posErrSatZ = c.posErrSatCoef / c.kpPosZ;   // 18차 z분리 (비-agile은 kpPosZ==kpPos라 동일)
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
    Lpf1 fMeasP, fMeasR;             // 자세 측정 필터 (tau=0.05, 덤프 확정)
    Lpf1 fMeasY, fMeasZ;             // yaw(0.01)/고도(0.01) 측정 필터 (덤프 확정)
    Lpf1 fPosPath[3];                // 위치 명령 경로 필터
    void reset() {
        pidPosX.reset(); pidPosY.reset(); pidPosZ.reset();
        pidAttP.reset(); pidAttR.reset(); pidYaw.reset(); pidAlt.reset();
        for (auto& p : pidMot) p.reset();
        fMeasP.reset(); fMeasR.reset(); fMeasY.reset(); fMeasZ.reset();
        for (auto& f : fPosPath) f.reset();
    }
};

// 게인을 config+스케일로부터 상태에 주입 (초기화 시 1회, 물성 변경 시 재호출)
void qc_bind(QcState& st, const QcConfig& c);

// 한 스텝 실행. dt[s] 고정 스텝 권장 (골든 트레이스는 1kHz 기준으로 대조).
QcOutput qc_step(QcState& st, const QcConfig& c, const QcInput& in, double dt);

} // namespace qc
