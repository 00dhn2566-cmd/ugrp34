// qc_motor.hpp — 모터+프로펠러 근사 플랜트 (사용자 요청 17차: "전압 출력 + 대충 모터
// dynamics + 모터→추력")
//
// 용도: ① Gazebo 접합 시 모터 명령→추력/토크 변환 사슬 제공 ② C++ 단독 건식 주행에서
//       모터 동역학 포함 폐루프 ③ 호버 평형 자가검증 (스모크).
// 근거 (전부 튜닝 세션 실측):
//   - 추력  T = Ct·ρ·n²·D⁴  (Aerodynamic Propeller 표준계수, 11차: Ct=0.1072)
//     검산: n=100.9 rev/s → 모터당 5.56N ×4 = 22.3N ≈ m_tot·g (2.2726·9.81) ✓
//   - 항력토크 Q = Cq·ρ·n²·D⁵ — 9차 "평형속도=토크측정기": 토크 클램프 0.2 N·m에서
//     평형 ~634 rad/s가 되도록 Cq 교정 (모터 PI는 호버에서 상시 클램프에 걸려 있음)
//   - 동역학 J·dw/dt = τ − Q − b·w, J는 qc_motor.time_const=0.02s가 나오게 역산
//   - 전압: V = duty·V_batt (duty = |cmd|/limit) — 개략 모델, 정밀화는 실기 단계
// 주의: 이것은 "플랜트 근사"다 — 골든 트레이스의 정답은 여전히 Simscape. 이 모델은
//       Gazebo/단독 실행용이며 Simscape 대체가 아니다.

#pragma once
#include <cmath>

namespace qc {

struct MotorParams {
    double maxTorque = 0.8;      // N·m (qc_motor.max_torque)
    double maxPower  = 160.0;    // W   (qc_motor.max_power)
    double limitCmd  = 0.25;     // 정규화 명령 클램프 (limit_motor)
    double rho       = 1.225;    // kg/m^3
    double D         = 0.254;    // m (프로펠러 직경)
    double Ct        = 0.1072;   // 추력 표준계수 (11차)
    double Cq        = 0.01517;  // 항력 표준계수 (9차 평형 교정: Q=0.2 @ 634 rad/s)
    double Jrotor    = 1.26e-5;  // kg·m^2 (시정수 0.02s 역산)
    double damping   = 1e-7;     // N·m/(rad/s) (qc_motor.rotor_damping)
    double Vbatt     = 22.2;     // V — 개략 (배터리 스펙 확정 시 갱신)
};

struct MotorOut {
    double voltage;   // V (개략: duty×Vbatt)
    double torque;    // N·m (클램프·파워 제한 후 실제 인가)
    double w;         // rad/s (적분 후)
    double thrust;    // N (프로펠러 추력)
    double dragQ;     // N·m (공력 반토크 — yaw 반작용에 사용)
};

// 모터 1개. cmdNorm = 제어기 motorCmd (부호는 회전방향, 크기 0~limitCmd).
struct Motor {
    double w = 0;     // rad/s (크기 — 방향 부호는 mixDir가 관리)

    MotorOut step(double cmdNorm, const MotorParams& p, double dt) {
        MotorOut o{};
        double mag = std::fabs(cmdNorm);
        if (mag > p.limitCmd) mag = p.limitCmd;

        o.voltage = (mag / p.limitCmd) * p.Vbatt;

        // 명령→토크 (0.25 → 0.2 N·m: 9차 평형 실측과 정합) + 파워 제한 τ·w ≤ maxPower
        double tau = mag * p.maxTorque;
        if (w > 1.0 && tau * w > p.maxPower) tau = p.maxPower / w;

        // 공력 부하: n[rev/s] 기준 표준식
        const double n = w / (2.0 * 3.14159265358979323846);
        const double n2 = n * n;
        o.dragQ  = p.Cq * p.rho * n2 * std::pow(p.D, 5);
        o.thrust = p.Ct * p.rho * n2 * std::pow(p.D, 4);

        // J·dw/dt = τ − Q − b·w  (w<0 방지: 추력용 크기 상태)
        w += dt * (tau - o.dragQ - p.damping * w) / p.Jrotor;
        if (w < 0) w = 0;

        o.torque = tau;
        o.w = w;
        return o;
    }
};

} // namespace qc
