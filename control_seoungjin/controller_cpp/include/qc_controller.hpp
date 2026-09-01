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
// anti-windup: 구운 모델은 `AntiWindupMode='none'` 이었다 (출력만 클램프, 적분기는 계속 적분).
// 2026-08-22 외란 강건화 세션에서 Simulink 쪽 `Control Yaw` 를 'clamping' 으로 바꿨고
// (`qc_antiwindup_apply` + `run_traj_baked` 기본 켬), 여기도 같은 식으로 맞춘다.
// 포화하지 않는 구간에서는 완전히 항등이라 골든 트레이스는 불변이다.
struct Pid {
    // 파라미터
    double kp = 0, ki = 0, kd = 0;
    double N = 100;          // 미분 필터 계수 (filtD)
    double outLim = 0;       // 출력 클램프 ±outLim (0이면 무제한)
    double kiScale = 1.0;    // 적분 '누적률' 한시 배율 (외란 적응용; 1.0 = 항등)
                             // ki 자체가 아니라 누적률만 곱한다 — 이미 쌓인 integ 는
                             // 건드리지 않으므로 배율이 바뀌어도 출력에 점프가 없다.
    bool antiWindup = false; // true = Simulink 'clamping' 과 동일 (조건부 적분).
                             // 출력이 포화 중이고 오차가 포화를 **더 미는** 방향일 때만
                             // 적분을 멈춘다. 포화 없으면 항등.
    // 상태
    double integ = 0;        // 적분기
    double dFilt = 0;        // 필터드 미분 상태
    double ePrev = 0;
    bool   first = true;

    void reset() { integ = 0; dFilt = 0; ePrev = 0; first = true; }

    double step(double e, double dt) {
        // 조건부 적분 판정은 '이번 스텝 적분을 더하기 전' 상태로 한다.
        // Simulink clamping 과 같은 규칙: (출력 포화 중) AND (오차가 포화를 더 미는 방향)
        bool hold = false;
        if (antiWindup && outLim > 0) {
            double de0 = first ? 0.0 : (e - ePrev) / dt;
            double a0 = 1.0 / (1.0 + N * dt);
            double dPre = a0 * dFilt + (1.0 - a0) * kd * de0;
            double uPre = kp * e + integ + dPre;
            const bool sat = (uPre > outLim) || (uPre < -outLim);
            hold = sat && ((uPre > 0.0 && ki * e > 0.0) || (uPre < 0.0 && ki * e < 0.0));
        }
        if (!hold) {
            integ += ki * kiScale * e * dt;         // 전진 오일러 적분 (kiScale=1 이면 원본과 동일)
        }
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

// ---------- yaw 외란 적응 적분 (2026-08-22, 사용자 설계) ----------
// 목적: yaw 는 약권한 채널이라 지속 외란이 걸리면 PD 만으로는 영구 고착한다(12차 실측 42~65도).
//       그래서 ki_yaw=1.5(Ti=10s)를 상시 켜 두는데, 이 값은 '평시 지터를 안 건드리는 선'에서
//       고른 것이라 외란 소거가 느리다. 여기서는 외란이 감지된 동안만 적분 누적률을 올린다.
//
// 설계 요지 (왜 |e| 가 아니라 저역통과 ē 인가):
//   scan 미션은 yaw 를 0.6~1.0 rad/s 로 의도적으로 슬루한다. 그 구간의 추종 오차는 외란이 아니다.
//   |e| 만 보면 슬루마다 적분을 키워 오버슈트를 만든다. 그래서 두 겹으로 거른다.
//     (a) 지속성  : ē = LPF(e, tau) — 순간 오차가 아니라 '남아 있는' 오차만 통과
//     (b) 명령 게이트: |psi_ref_dot| > rateGate 이면 슬루 중으로 보고 아예 판정하지 않음
//
// 배율 궤적: k in [0,1] 로 sat((|ē|-e0)/e1). 상승은 즉시(외란 대응은 빨라야 함),
//            하강은 기울기 -relax [1/s] 로 제한 (경계에서 채터링하면 적분률이 요동친다).
//            Simulink 측은 같은 자리에 Rate Limiter(RisingSlewLimit=inf, Falling=-relax) 를 쓴다.
//   반환값 g = 1 + (gmax-1)*k  ->  Pid::kiScale 에 그대로 물린다.
//
// gmax = 1.0 (기본값) 이면 항상 g=1 -> 골든 트레이스 비트 동일. 켤 때만 값을 올릴 것.
struct YawDistI {
    // 파라미터
    double gmax     = 1.0;    // 최대 적분률 배율 (1.0 = 기능 꺼짐)
    double e0       = 0.0349; // rad (2.0도) — 판정 시작 문턱
    double e1       = 0.0349; // rad (2.0도) — 문턱 위로 이만큼 더 가면 gmax 포화 (총 4.0도)
    double tau      = 1.0;    // s — 오차 저역통과(지속성 판정) 시정수
    double rateGate = 0.05;   // rad/s — |psi_ref_dot| 이 값 초과면 슬루로 보고 게이트 닫음
    double relax    = 0.5;    // 1/s — 해제 시 k 하강 기울기 제한 (Simulink Rate Limiter 와 같은 식)
    // 상태
    double eLpf = 0, k = 0;
    bool   first = true;

    void reset() { eLpf = 0; k = 0; first = true; }

    // e: yaw 오차 [rad] (wrapPi 적용된 값), refRate: 참조 yaw 각속도 [rad/s]
    double step(double e, double refRate, double dt) {
        if (first) { eLpf = e; first = false; }
        else       { eLpf += dt / (tau + dt) * (e - eLpf); }

        double target = 0.0;
        if (std::fabs(refRate) <= rateGate)
            target = clamp((std::fabs(eLpf) - e0) / e1, 0.0, 1.0);

        // 상승 무제한 / 하강 -relax [1/s] — Simulink 쪽 Rate Limiter 와 동일 의미
        const double kLo = k - relax * dt;
        k = (target > kLo) ? target : kLo;

        return 1.0 + (gmax - 1.0) * k;
    }
};

// ---------- 외란 연동 속도 조속기 (2026-08-22) ----------
// Simulink `qc_clock_gov_apply.m` 의 1:1 이식. 설계: docs/SPEED_GOVERNOR.md
//
//   rho_eff = max( LPF(|u_yaw|/limYaw, tauRho),  LPF(|wrapPi(e_yaw)|, tauPsi)/psiStop )
//   d*      = (1 - sMin) * sat01( (rho_eff - rf) / (rs - rf) )      <- 벗어난 양에 선형 비례
//   d       = 3차 임계감쇠 필터(d*)                                   <- (s/w+1)^-3
//   s       = 1 - govOn * d
//
// ★ 필터를 s 가 아니라 '1로부터의 편차 d' 에 건다. s 를 직접 필터하면 초기값이 0 이라
//   시작부터 시계가 멎는다 (Simulink 구현에서 같은 함정을 겪었다).
//
// 이 구조체는 **s 만 낸다.** tau 적분과 참조 조회는 호출자 몫이다 —
// Simulink 에서도 Integrator 출력이 Lookup 을 먹이는 구조라 같은 분업이다.
// govOn = false (기본) 이면 s ≡ 1 -> 기존 동작과 완전히 동일.
struct SpeedGovernor {
    // 파라미터 (qc_clock_gov_defaults.m 와 동일 기본값)
    bool   on      = false;
    double rf      = 0.00;      // 무개용 문턱 (0 = 문턱 없이 선형 비례)
    double rs      = 1.00;      // 정규화 1.0 에서 s = sMin
    double sMin    = 0.00;      // 최저 시계 배율 (0 = 완전 정지 허용)
    double ws      = 0.50;      // 3차 임계감쇠 대역 [rad/s] (스냅 예산 §6)
    double tauRho  = 0.20;      // rho 저역통과 [s]
    double tauPsi  = 0.20;      // yaw 오차 저역통과 [s]
    double psiStop = 0.7853981633974483;   // 45 deg [rad] — 안정성 경계 90 도의 절반
    // 상태
    Lpf1 fRho, fPsi;
    Lpf1 f1, f2, f3;            // 3차 = 1차 3단 (각 시정수 1/ws)
    double rhoEff = 0, sOut = 1.0;

    void bind() {
        fRho.tau = tauRho;
        fPsi.tau = tauPsi;
        f1.tau = f2.tau = f3.tau = 1.0 / (ws > 1e-9 ? ws : 1e-9);
    }
    void reset() {
        fRho.reset(); fPsi.reset(); f1.reset(); f2.reset(); f3.reset();
        // 편차 필터의 초기값은 0 이어야 s(0)=1 이 된다
        f1.y = f2.y = f3.y = 0.0; f1.first = f2.first = f3.first = false;
        rhoEff = 0.0; sOut = 1.0;
    }

    // uYaw: yaw PID 출력, limYaw: 그 클램프, eYaw: yaw 오차(rad, 랩 전/후 무관)
    double step(double uYaw, double limYaw, double eYaw, double dt) {
        const double rho = (limYaw > 0.0) ? std::fabs(uYaw) / limYaw : 0.0;
        const double rBar = fRho.step(rho, dt);
        const double pBar = fPsi.step(std::fabs(wrapPi(eYaw)), dt) / psiStop;
        rhoEff = rBar > pBar ? rBar : pBar;

        const double den = (rs - rf) > 1e-9 ? (rs - rf) : 1e-9;
        double u = (rhoEff - rf) / den;
        u = clamp(u, 0.0, 1.0);
        const double dStar = (1.0 - sMin) * u;

        const double d = f3.step(f2.step(f1.step(dStar, dt), dt), dt);
        sOut = 1.0 - (on ? d : 0.0);
        return sOut;
    }
};

// ---------- 시간 지연 추적 (2026-08-23) ----------
// Python `latency_tracker.LatencyTracker` 의 1:1 이식.
//
// 표본 하나로 판단하지 않는다 — 단발 스파이크(스케줄러 지터, 파일 잠금)로 스펙을
// 깎으면 순항 속도가 계속 요동친다. 그래서 EMA 두 개를 쓴다:
//   빠른 EMA(8) 로 '걸렸다'를 감지하고, 보고값은 **둘 중 큰 쪽**을 쓴다.
//
// ★ 왜 max(느린, 빠른) 인가 (2026-08-23 수정) —
//   느린 EMA(60) 만 내보내면 지연이 붙은 직후 30 표본쯤 늦게 반영된다. 그런데
//   `traj_bridge` 분석에 따르면 감쇄를 결정한 뒤에도 새 한계 안으로 들어가는 데
//   0.7~4 s 가 더 걸린다 — 늦은 감지 + 늦은 수렴은 두 번 늦는 것이다.
//   max 를 쓰면 붙는 순간은 빠른 쪽이, 빠지는 순간은 느린 쪽이 이겨
//   "즉시 깎고 천천히 되돌린다" 는 비대칭이 공짜로 나온다.
struct LatencyTracker {
    double baselineS = 0.017;   // 이 아래는 '지연 없음' (30 Hz 상태 주기의 절반)
    double triggerS  = 0.040;   // 이 위면 '걸림' 판정
    int    tauFastN  = 8;
    int    tauSlowN  = 60;
    int    armN      = 3;       // 감지 진입에 필요한 연속 초과 표본수
    int    holdN     = 30;      // 해제 전 연속 정상 표본수
    // 상태
    double emaFast = 0, emaSlow = 0, peakS = 0;
    int    n = 0, cleanRun = 0, overRun = 0;
    bool   detected = false;

    void reset() {
        emaFast = emaSlow = peakS = 0.0;
        n = cleanRun = overRun = 0;
        detected = false;
    }

    double update(double sampleS) {
        const double x = sampleS > 0.0 ? sampleS : 0.0;
        ++n;
        if (x > peakS) peakS = x;
        if (n == 1) {
            emaFast = emaSlow = x;
        } else {
            const double af = 2.0 / (tauFastN + 1.0);
            const double as = 2.0 / (tauSlowN + 1.0);
            emaFast += af * (x - emaFast);
            emaSlow += as * (x - emaSlow);
        }
        overRun = (x > triggerS) ? (overRun + 1) : 0;

        if (emaFast > triggerS && overRun >= armN) {
            detected = true;
            cleanRun = 0;
        } else if (detected) {
            if (emaFast <= baselineS) {
                if (++cleanRun >= holdN) { detected = false; cleanRun = 0; }
            } else {
                cleanRun = 0;
            }
        }
        return predicted();
    }

    double predicted() const {
        if (!detected) return 0.0;
        return emaSlow > emaFast ? emaSlow : emaFast;
    }
};

// ---------- 지연 -> 스펙 (2026-08-23 MATLAB 실측) ----------
// Python `capability` / `spec_governor` 의 지연 규칙과 **같은 수를 내야 한다.**
// 실측 출처: diagnose/sweep_delay_margin.m (자세), diagnose/sweep_delay_spec.m (위치).
//
// 두 경로를 다르게 다룬다 — 이게 요점이다.
//   자세(IMU->제어기): **게이트**. 20 ms 부터는 기동을 안 해도 호버가 2.4° 로 떨린다
//                     (0/8/12/16/20/24 ms 에서 RMS 0.021/0.004/0.004/0.211/2.437/4.614°).
//                     정지해 있는데도 불안정하니 궤적을 느리게 해도 안 고쳐진다 -> 임무 거부.
//   위치(VIO->제어기): **감쇄**. 오차가 속도에 비례하므로 느려지면 준다.
struct SpecLatencyRule {
    // 자세 경로 게이트
    double attCleanS  = 0.012;   // 이 아래 무보정
    double attMaxS    = 0.016;   // 이 위 운용 불가
    double attMargin  = 0.60;    // 그 사이 구간의 고정 배율

    // 위치 경로 실측표. 표 밖은 **외삽하지 않는다** — 안 재본 구간의 추정치를 내면
    // 그게 곧 사고다. 마지막 값을 유지하고 플래그를 세운다.
    //
    // ★ 두 벌인 이유 (사용자 정정): **기본은 외란 없음.** 상위에 늘 내보내는 스펙은
    //   지연만 반영해야 한다. 돌풍 표는 외란이 실제로 감지될 때만, 그것도 rho 크기로
    //   기본표와 **보간**해서 쓴다 (한 점에서 잰 값을 약한 돌풍에 그대로 쓰면 과감쇄).
    static constexpr int kMaxAnchors = 8;
    double posTau[kMaxAnchors]      = {0.000, 0.020, 0.040, 0.060, 0.080, 0.120, 0.160, 0.0};
    double posScale[kMaxAnchors]    = {1.00, 1.00, 0.88, 0.75, 0.37, 0.00, 0.00, 0.0};
    int    posN = 7;         // sync_delay_anchors.py 가 생성 — 손으로 고치지 말 것

    // 0 kg 무외란 표 (2026-08-28 실측). 질량은 지연 내성을 크게 바꾼다 —
    // 120/160 ms 에서 1 kg 은 운용 불가인데 0 kg 은 0.55/0.40 으로 산다. 원인은
    // 질량이 아니라 그 질량의 튜닝 강도다 (0 kg 구성이 물러서 위상 여유가 남는다).
    // 이 표가 없으면 0 kg 비행이 1 kg 표에 묶여 80 ms 에서 0.37 로 간다 (실측 0.75).
    double posTau0[kMaxAnchors]     = {0.000, 0.020, 0.040, 0.060, 0.080, 0.120, 0.160, 0.0};
    double posScale0[kMaxAnchors]   = {1.00, 1.00, 1.00, 0.83, 0.75, 0.55, 0.40, 0.0};
    int    posN0 = 7;        // sync_delay_anchors.py 가 생성 — 손으로 고치지 말 것

    // 짐 질량 [kg]. 게인 스케줄과 **같은 값**을 이륙 전에 한 번 넣는다.
    // 이 모델은 비행 중 질량을 추정하지 않는다 (qc_pkg_mass_set 주석과 같은 규약).
    double pkgKg = 1.0;

    // 돌풍 표 (0.3 N*m x 0.3 s 를 이동 중 맞고도 복귀가 사는 배율). sync_delay_anchors.py 생성.
    double gustTau[kMaxAnchors]     = {0.000, 0.020, 0.030, 0.040, 0.060, 0.080, 0.0, 0.0};
    double gustScale[kMaxAnchors]   = {1.00, 1.00, 1.00, 0.55, 0.28, 0.00, 0.0, 0.0};
    int    gustN = 6;        // sync_delay_anchors.py 가 생성 — 손으로 고치지 말 것
    // 돌풍 표를 잰 외란 크기에 대응하는 rho (0.3 N*m = yaw 권한의 94.6%).
    double gustRhoRef = 0.90;
    // ⚠ 돌풍 표에는 질량 축이 없다. 0 kg 돌풍 표는 **다른 복귀 게이트**로 쟀기
    //   때문이다 (그 질량의 tau=0 복귀의 2배 = 약 18 s vs 1 kg 3 s). 이으면 서로
    //   다른 기준이 한 표에 섞인다. 게이트 표기 방식이 정해지면 그때 연다.

    static double lookup(const double* tau, const double* sc, int n, double t,
                         bool* extrapolated) {
        if (extrapolated) *extrapolated = (n > 0 && t > tau[n - 1]);
        if (n <= 0) return 1.0;
        if (t <= tau[0]) return sc[0];
        if (t >= tau[n - 1]) return sc[n - 1];
        for (int i = 1; i < n; ++i) {
            if (t <= tau[i]) {
                const double d = tau[i] - tau[i - 1];
                const double w = d > 1e-12 ? (t - tau[i - 1]) / d : 0.0;
                return sc[i - 1] + (sc[i] - sc[i - 1]) * w;
            }
        }
        return sc[n - 1];
    }

    // 무외란 표를 질량으로 보간한다 (파이썬 `capability._lat_table_for_pkg` 대응).
    //
    // 순서가 중요하다: **질량 먼저, tau 나중**이다. 파이썬이 그렇게 한다 —
    // 0.00(운용 불가) 흡수를 앵커 단계에서 하기 때문이다. 순서를 뒤집으면
    // (tau 먼저 -> 질량) 0.00 이 이미 다른 값과 섞여 흡수가 안 걸리고,
    // 0.5 kg / 100 ms 에서 0.28 대신 0.42 가 나온다 (더 빠르게 가라는 쪽 = 위험).
    //
    // 두 표의 tau 격자가 다르면 질량 보간을 포기하고 1 kg 표로 물러난다 (보수적).
    double posNominalFor(double t, bool* extrapolated) const {
        bool sameGrid = (posN0 == posN);
        for (int i = 0; sameGrid && i < posN; ++i) {
            const double d = posTau0[i] - posTau[i];
            if ((d < 0 ? -d : d) > 1e-12) sameGrid = false;
        }
        if (!sameGrid) return lookup(posTau, posScale, posN, t, extrapolated);
        const double w = clamp(pkgKg, 0.0, 1.0);
        double sc[kMaxAnchors] = {};
        for (int i = 0; i < posN; ++i) {
            const double a = posScale0[i], b = posScale[i];
            sc[i] = (a == 0.0 || b == 0.0) ? 0.0 : a + (b - a) * w;
        }
        return lookup(posTau, sc, posN, t, extrapolated);
    }

    // rhoEff = 관측된 유효 외란 점유율 (0 이면 기본표 그대로).
    double posScaleFor(double tauS, double rhoEff = 0.0,
                       bool* extrapolated = nullptr) const {
        const double t = tauS > 0.0 ? tauS : 0.0;
        const double sNom = posNominalFor(t, extrapolated);
        if (rhoEff <= 0.0) return sNom;
        const double sG = lookup(gustTau, gustScale, gustN, t, nullptr);
        double u = rhoEff / (gustRhoRef > 1e-9 ? gustRhoRef : 1e-9);
        u = clamp(u, 0.0, 1.0);
        return sNom + (sG - sNom) * u;      // 잰 조합이므로 가산이 아니라 보간
    }

    // 반환 0.0 = 운용 불가 (임무 거부)
    double attScaleFor(double tauAttS) const {
        const double t = tauAttS > 0.0 ? tauAttS : 0.0;
        if (t > attMaxS) return 0.0;
        if (t > attCleanS) return attMargin;
        return 1.0;
    }
};

// 상위 계획기에 보고할 스펙 한 장 (Python `capability.json` 의 C++ 대응물).
// 한계는 **배율 하나**로 깎는다: v∝s, a∝s², j∝s³, snap∝s⁴.
// 속도만 자르면 "느린데 급격한" 궤적이 나오고, 그건 경로 기하가 바뀐다는 뜻이라
// 금지구역 판정과 다리 궤적(traj_bridge)의 전제가 함께 깨진다.
struct SpecReport {
    double timeScale = 1.0;
    double v = 0, a = 0, j = 0, snap = 0;
    double scaleDisturb = 1.0, scaleLatPos = 1.0, scaleLatAtt = 1.0;
    double scaleRecovery = 1.0;   // 회복 감시(폐루프 교정)
    double latencyAppliedS = 0.0;
    bool   missionAllowed = true;
    bool   latencyExtrapolated = false;
};

// baseV/A/J/Snap: 질량 앵커 기저 한계, limitScale: 프로파일 배율(precision 0.75)
// sDisturb: 외란 쪽 배율 (SpeedGovernor 의 s 를 그대로 넣으면 된다)
// sDisturb: 외란 쪽 배율 (SpeedGovernor 의 s). 1 - sDisturb 를 rhoEff 로 써서
// 위치 지연표(기본/돌풍) 사이를 보간한다.
inline SpecReport qc_spec_report(const SpecLatencyRule& rule,
                                 double baseV, double baseA, double baseJ, double baseSnap,
                                 double limitScale, double sDisturb,
                                 double latencyPosS, double latencyAttS,
                                 double sRecovery = 1.0) {
    SpecReport r;
    r.scaleRecovery = clamp(sRecovery, 0.0, 1.0);
    r.latencyAppliedS = latencyPosS;
    r.scaleDisturb = clamp(sDisturb, 0.0, 1.0);
    r.scaleLatPos  = rule.posScaleFor(latencyPosS, 1.0 - r.scaleDisturb,
                                      &r.latencyExtrapolated);
    r.scaleLatAtt  = rule.attScaleFor(latencyAttS);
    r.missionAllowed = (r.scaleLatAtt > 0.0);

    // **가산**으로 합친다 (사용자 지적): 배율 s 가 아니라 깎인 양 1-s 가 소모량이라,
    // 서로 다른 원인이면 더해야 맞다. min 은 두 원인이 같은 제약을 다르게 표현한
    // 경우에만 옳고, 그렇지 않으면 낙관적이다.
    // 여유(제어 권한/위상 여유)라는 **하나의 자원**을 속도·외란·지연이 나눠 쓰는 모델.
    double d = 0.0;
    d += 1.0 - clamp(r.scaleDisturb, 0.0, 1.0);
    d += 1.0 - clamp(r.scaleLatPos, 0.0, 1.0);
    d += 1.0 - clamp(r.scaleRecovery, 0.0, 1.0);
    if (r.missionAllowed) d += 1.0 - clamp(r.scaleLatAtt, 0.0, 1.0);
    double s = r.missionAllowed ? (1.0 - d) : 0.0;
    if (s < 0.0) s = 0.0;

    r.timeScale = s;
    r.v    = baseV    * limitScale * s;
    r.a    = baseA    * limitScale * s * s;
    r.j    = baseJ    * limitScale * s * s * s;
    r.snap = baseSnap * limitScale * s * s * s * s;
    return r;
}

// ---------- 회복 감시 (2026-08-23, 사용자 설계) ----------
// Python `recovery_watcher.RecoveryWatcher` 의 1:1 이식.
//
// 실측표(SpecLatencyRule)는 특정 조합에서 잰 것이라 실제 운용이 그 격자 위에 놓일
// 이유가 없다. 그래서 **표 = 피드포워드, 이 감시 = 피드백**.
//
// 관측량은 **밴드 초과 지속시간**: |측정-기준| 이 track 밴드를 연속으로 넘고 있는 시간.
// 비행 중에는 "외란이 끝났다" 는 이벤트가 없으므로, MATLAB 게이트로 쓴 복귀 시간의
// 온라인 대응물이 이것이다. 에피소드가 끝나길 기다리지 않고 넘고 있는 동안 반응한다.
//
// 안전장치 둘 — 둘 다 없으면 이 루프가 스스로 사고를 낸다:
//  ① 판단 주기 > 다리 수렴 시간. 감쇄 결정 뒤에도 새 한계 안으로 드는 데 0.66~3.94 s
//     가 걸린다(실측). 그보다 짧게 판단하면 앞 결정이 반영되기 전에 또 결정 = 발진.
//  ② 기준이 한계 밖이면 계상 안 함. 계획이 과한 것을 제어기 탓으로 돌려 스펙을 깎으면
//     잘못된 계획이 기체 능력을 갉아먹는 되먹임이 된다.
//
// 바닥은 kRecFloor 이지 0 이 아니다 — 정지 판단은 감시의 권한이 아니다(감독자 몫).
struct RecoveryWatcher {
    static constexpr double kLeadMargin = 1.5;   // 다리 수렴 시간에 곱할 여유
    static constexpr double kRecFloor   = 0.15;  // 감시가 낼 수 있는 최저 배율

    // 파라미터 (Python 기본값과 동일)
    double trackBandM  = 0.04;   // capability.budget.track
    double settleS     = 2.2;    // capability.budget.settle
    double cutGain     = 0.5;
    double maxCut      = 0.25;   // 한 판단당 최대 감쇄. 없으면 두세 번에 바닥을 친다
    // 깎아도 안 나아진 판단이 이만큼 연속되면 깎기를 멈춘다 (파이썬과 1:1).
    // 근거: 0 kg / 토크 0.3 N*m / 20 ms 에서 배율을 1.00 -> 0.37 로 내려 순항을
    // 2.7배 낮췄는데 복귀가 9.87 -> 9.79 s 로 꿈쩍도 안 했다 (PERFORMANCE 8g).
    // 이탈은 위치오차 클램프 포화가 정하므로 속도와 무관하기 때문이다. 그 영역에서
    // 계속 깎으면 에너지만 1/s 로 늘고(바닥에서 10배) 회복은 그대로다. 멈추고 알린다 —
    // 다음 수는 감쇄가 아니라 재계획/임무축소/착륙이고 그건 상위가 고를 일이다.
    // ★ 효과 판정에 lastRatio(밴드 초과 **지속시간**)를 쓰면 안 된다. 오차가 밴드 위에
    //   머무는 한 그 값은 계속 커지기만 해서, 깎기가 잘 듣고 있어도 무효로 오판한다
    //   (파이썬 시험에서 오차 0.20 -> 0.05 로 4배 줄었는데 ratio 는 3.65 -> 7.29 로 올랐다).
    //   그래서 **평균 오차 크기**의 상대 변화를 본다.
    int    futileN     = 3;
    double futileEps   = 0.05;   // 상대 개선폭 (5% 미만이면 '안 나아졌다')
    double minPeriodS  = 4.0;
    double cleanHoldS  = 3.0;
    double restoreTauS = 6.0;
    // 상태
    double s = 1.0;
    double tAbove = 0, worstAbove = 0, tClean = 0, tSince = 0;
    double lastRatio = 0;
    double errSum = 0;               // 이번 판단 창의 |오차| 합
    long   errN = 0;
    double meanErr = 0;              // 직전 판단의 평균 |오차|
    double prevMeanErr = 0;
    bool   havePrev = false;
    int    futileCuts = 0;           // 연속 무효 깎기
    bool   derateIneffective = false;
    long   nObs = 0, nSkipped = 0, cuts = 0;
    bool   restoring = false;

    void reset() {
        s = 1.0;
        tAbove = worstAbove = tClean = tSince = lastRatio = 0.0;
        errSum = 0.0; errN = 0; meanErr = 0.0;
        prevMeanErr = 0.0; havePrev = false; futileCuts = 0;
        derateIneffective = false;
        nObs = nSkipped = cuts = 0;
        restoring = false;
    }

    // 제어 주기마다. refOk = '지금 기준이 현재 limits 안인가'.
    void observe(double errM, bool refOk, double dt) {
        if (dt < 0.0) dt = 0.0;
        tSince += dt;                     // 판단 주기는 실제 시간으로 센다 (버린 표본도 포함)
        if (!refOk) { ++nSkipped; return; }
        ++nObs;
        const double e = std::fabs(errM);
        errSum += e;
        ++errN;
        if (e > trackBandM) {
            tAbove += dt;
            tClean = 0.0;
            if (tAbove > worstAbove) worstAbove = tAbove;
        } else {
            tAbove = 0.0;
            tClean += dt;
        }
    }

    // bridgeLeadS <= 0 이면 '모름' 으로 보고 minPeriodS 만 쓴다.
    double periodS(double bridgeLeadS) const {
        double p = minPeriodS;
        if (bridgeLeadS > 0.0 && bridgeLeadS == bridgeLeadS) {   // NaN 방어
            const double q = bridgeLeadS * kLeadMargin;
            if (q > p) p = q;
        }
        return p;
    }

    double decide(double bridgeLeadS) {
        const double period = periodS(bridgeLeadS);
        if (tSince < period) return s;
        const double elapsed = tSince;
        tSince = 0.0;

        lastRatio = worstAbove / (settleS > 1e-9 ? settleS : 1e-9);
        worstAbove = tAbove;              // 아직 넘고 있으면 그 시간은 이월
        meanErr = errN > 0 ? errSum / static_cast<double>(errN) : 0.0;
        errSum = 0.0; errN = 0;

        if (lastRatio > 1.0) {
            if (havePrev && cuts > 0) {
                const double base = prevMeanErr > 1e-9 ? prevMeanErr : 1e-9;
                const double gain = (prevMeanErr - meanErr) / base;   // 상대 개선
                if (gain < futileEps) ++futileCuts;
                else                  futileCuts = 0;
            }
            if (futileCuts >= futileN) {
                derateIneffective = true;   // 배율 동결 + 밖에 알림
                restoring = false;
            } else {
                double step = cutGain * (lastRatio - 1.0);
                if (step > maxCut) step = maxCut;
                s -= step;
                if (s < kRecFloor) s = kRecFloor;
                ++cuts;
                restoring = false;
            }
            prevMeanErr = meanErr; havePrev = true;
        } else if (tClean >= cleanHoldS && s < 1.0) {
            futileCuts = 0; derateIneffective = false; havePrev = false;
            double a = elapsed / (restoreTauS > 1e-9 ? restoreTauS : 1e-9);
            a = clamp(a, 0.0, 1.0);
            s += a * (1.0 - s);
            restoring = true;
            if (1.0 - s < 1e-6) { s = 1.0; restoring = false; }
        } else {
            restoring = false;
        }
        return s;
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
    // 속도 조속기 (SpeedGovernor). govOn=false 면 s≡1 = 기존 동작.
    bool   govOn = false;
    double govRf = 0.00, govRs = 1.00, govSmin = 0.00, govWs = 0.50;
    double govTauRho = 0.20, govTauPsi = 0.20;
    double govPsiStop = 0.7853981633974483;   // 45 deg
    // yaw 적분 와인드업 방지 (Simulink 'clamping' 대응, run_traj_baked 기본 켬 = YAWAW).
    // 포화 없는 구간 항등 -> 골든 트레이스 불변.
    bool yawAntiWindup = true;
    // ── 지연 -> 스펙 보고 (2026-08-23). **관측 전용**: 제어 출력에 아무 영향이 없다.
    //    (다리 궤적과 재계획은 계획측이 하는 일이라, 여기서는 '무엇을 줘도 되는지'만 낸다.
    //     그래서 golden trace 는 이 기능을 켜도 불변이다.)
    bool   specOn = true;
    // 스펙 보고 주기 [Hz]. **제어 주기(1 kHz)로 돌리면 안 된다** — LatencyTracker 의
    // EMA 시정수는 표본 수(빠른 8 / 느린 60)로 정의돼 있어서, 1 kHz 에서 돌리면
    // 8 ms / 60 ms 짜리가 되어 스케줄러 지터 하나하나에 반응한다. 파이썬 조속기와
    // 같은 ~5 Hz 로 데시메이션해야 두 구현이 같은 수를 낸다.
    double specRateHz = 5.0;
    double specBaseV = 1.6, specBaseA = 1.6, specBaseJ = 8.0, specBaseSnap = 64.0;
    double specLimitScale = 0.75;    // 프로파일 precision (agile = 1.00)
    double latencyAttS = 0.003;      // 자세 경로 지연 — 하드웨어 상수에 가깝다 (구성값)
    // 질량 1차식을 08-18 채택 0 kg 앵커 기준으로 바꾼다 (qc_mass_lerp_apply.m 의 짝).
    // false(기본) = 07-19 18차 법칙 그대로 = 기존 동작. **1 kg 에서는 둘이 같다.**
    //
    // ⚠ 켜기 전에 풀어야 할 것 — **프로파일과 충돌한다.** 1차식의 kp_pos 앵커
    //   (5 -> 8)는 precision 기준이다. agile 은 프로파일이 kpPos 를 24 로 올리는데,
    //   massLerpOn 이 켜지면 qc_scales 가 posErrSat 을 1차식의 kpPos 로 계산해
    //   프로파일 설정을 덮어쓴다 (곱 불변식 C·kp = 1.2 가 깨진다).
    //   MATLAB `qc_mass_lerp_apply.m` 도 같은 전제(precision)로 쓰여 있다.
    //   -> 채택 시 결정할 것: 1차식을 precision 기저로 두고 프로파일이 그 위에
    //      곱해지게 할지, 아니면 프로파일별 앵커를 따로 잴지.
    //   MATLAB verify_mass_lerp.m 은 통과했다 (0 kg 호버 11.58 -> 0.0088도,
    //   1 kg 완전 동일). 남은 것은 이 프로파일 문제뿐이다. [TODO-채택]
    bool massLerpOn = false;
    // 회복 감시. recOn=false 면 배율 1 고정 = 기존 동작과 동일.
    bool   recOn = false;
    double recTrackBandM = 0.04;     // capability.budget.track
    double recSettleS = 2.2;         // capability.budget.settle
    // 계획측이 알려주는 직전 다리의 수렴 시간 [s]. 0 이면 '모름' -> minPeriodS 만 쓴다.
    double bridgeLeadS = 0.0;
    // yaw 외란 적응 적분 (YawDistI). yawDistGmax = 1 이면 완전히 꺼진 상태 = 기존 동작.
    double yawDistGmax = 1.0, yawDistE0 = 0.0349, yawDistE1 = 0.0349;
    double yawDistTau = 1.0, yawDistRateGate = 0.05, yawDistRelax = 0.5;
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

// 질량 1차식 — 두 실측 앵커를 잇는다. MATLAB `Scripts_Data/qc_mass_lerp_apply.m` 의 짝.
//
// 왜 두 법칙이 있나: 07-19 18차가 0 kg 앵커를 sA=0.75 로 잡았는데, 08-18 성능 세션이
// 0 kg 을 다시 튜닝해 **sA=0.35** 를 채택했다 (0.75 는 5 Hz 세차 한계사이클 ±8°,
// 0.40 은 자려 지터 0.156° -> 0.35 에서 0.005°). 그 결과가 parameters.m 에 동기되지
// 않아 0 kg 만 별도 이산 구성으로 갈라져 있었다. 이 구조체가 그 둘을 하나의 1차식으로
// 잇는다. **1 kg 에서는 두 법칙이 같으므로 1 kg 골든 트레이스는 불변이다.**
struct MassLerp {
    double sAMass, sZMass, kpPos, kdPos, rAtt, limAtt, filtPz, biasChassis, nlGmax;
};

inline MassLerp qc_mass_lerp(double pkgMass) {
    const double u = clamp(pkgMass, 0.0, 1.0);   // 앵커 밖은 클램프 (2 kg 은 1 kg 복사본)
    auto L = [u](double a0, double a1) { return a0 + (a1 - a0) * u; };
    MassLerp m;
    m.sAMass      = L(0.35,  1.00);
    m.sZMass      = L(0.56,  1.00);
    m.rAtt        = L(0.60,  1.50);   // kd/kp 비
    m.limAtt      = L(100.0, 800.0);
    m.kpPos       = L(5.0,   8.0);
    m.filtPz      = L(0.005, 0.01);
    m.biasChassis = L(75.5,  56.5);
    m.nlGmax      = L(2.1,   1.0);
    // 파생은 보간하지 않고 **다시 계산**한다 — 보간한 값끼리 어긋나면 불변식이 깨진다.
    m.kdPos = 0.4 * m.kpPos;          // 두 앵커 모두 비가 0.4 (2.0/5 == 3.2/8)
    return m;                         // posErrSat = 1.2/kpPos 는 qc_scales 가 계산
}

// 질량 1차식을 **설정에 실제로 반영**한다.
//
// 왜 따로 필요한가: `qc_scales` 는 const 참조라 설정을 못 고친다. 그래서 지금까지
// `qc_mass_lerp` 가 계산한 여덟 값 중 **셋만**(sAMass/sZMass/posErrSat) 소비됐고,
// 나머지는 `main_spec_trace` 가 찍기만 했다. 그 결과 0 kg 비행에서
//   biasChassis 56.5 (실측 앵커 **75.5**) — 호버 추력 바이어스 25% 부족
//   limAtt 800 (0 kg 채택 **100**) / kpPos 8 (채택 **5**)
// 이 그대로 남았다. MATLAB 쪽(`qc_mass_lerp_apply.m`)은 이미 다 넣고 있어서
// 두 구현이 0 kg 에서 다른 기체가 돼 있었다.
//
// 1 kg 에서는 1차식이 현행 기본값과 정확히 일치하므로 **골든 트레이스는 불변**이다
//   limAtt 800 / kdAtt = kpAtt·1.5 = -127.5 / biasChassis 56.5 / kpPos 8 / kdPos 3.2.
//
// kpPos/kdPos 는 **precision 에서만** 덮는다 — MATLAB 1차식도 precision 전제로
// 쓰여 있고(qc_mass_lerp_apply.m 주석), agile 은 프로파일이 삼각식으로 따로 정한다.
// filtPz / nlGmax 는 C++ 에 해당 필드가 없다 (기능 미구현) — 여기서 지어내지 않는다.
inline void qc_apply_mass_lerp(QcConfig& c, Profile p) {
    if (!c.massLerpOn) return;
    const MassLerp m = qc_mass_lerp(c.pkgMass);
    c.limAtt      = m.limAtt;
    c.kdAtt       = c.kpAtt * m.rAtt;     // 부호는 kpAtt 가 갖는다 (음수 필수)
    c.biasChassis = m.biasChassis;
    if (p == Profile::Precision) {
        c.kpPos  = m.kpPos;  c.kdPos  = m.kdPos;
        c.kpPosZ = c.kpPos;  c.kdPosZ = c.kdPos;
    }
}

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
            qc_apply_mass_lerp(c, p);
            return;
        }
    }
    c.kpPosZ = c.kpPos; c.kdPosZ = c.kdPos;   // precision/balanced: z = xy 동일 (기존 거동)
    // posErrSat = 1.2/kpPos (z는 1.2/kpPosZ) 곱 불변식은 qc_scales()가 자동 연동
    qc_apply_mass_lerp(c, p);
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
    if (c.massLerpOn) {
        const MassLerp m = qc_mass_lerp(c.pkgMass);
        s.sAMass = m.sAMass;
        s.sZMass = m.sZMass;
        s.posErrSat = s.posErrSatZ = c.posErrSatCoef / m.kpPos;
    } else {
        s.sAMass = 0.75 + 0.25 * mClamped;
        s.sZMass = 0.56 + 0.44 * mClamped;
        s.posErrSat  = c.posErrSatCoef / c.kpPos;
        s.posErrSatZ = c.posErrSatCoef / c.kpPosZ;   // 18차 z분리 (비-agile은 동일)
    }
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
    // 이 측정이 얼마나 낡았나 [s] — 상태 타임스탬프 나이 (INTERFACE_SPEC §8c T8).
    // 0 이면 '지연 없음'. 위치(VIO) 경로 지연 추적의 입력이다.
    double measAgeS = 0.0;
    // 회복 감시 입력. refWithinLimits=false 면 그 표본은 버린다 — 기준이 과한 것을
    // 제어기 탓으로 돌려 스펙을 깎으면 잘못된 계획이 기체 능력을 갉아먹는다.
    // (기본 true. 상위가 게이트 결과를 안 주면 감시가 보수적으로 작동한다.)
    bool refWithinLimits = true;
};

struct QcOutput {
    double cmdPitch, cmdRoll;   // 위치→자세 명령 (rad)
    double motorRef[4];         // 믹서 후 모터 속도 참조 (rev/s)
    double motorCmd[4];         // 모터 PI 출력 (정규화 토크 명령)
    // ── 진단 / 상위 노출 (capability.json `observed` 입력) ──
    // INTERFACE_SPEC §5b: rho = max(|u_yaw|/limYaw, |u_att|/limAtt).
    // 정상상태 적분기가 곧 외란 추정치라 별도 센서가 필요 없다.
    double uYaw;                // yaw PID 출력
    double eYaw;                // yaw 오차 (wrapPi 적용, rad)
    double rho;                 // 권한 점유율 [0,1] — 그 스텝 순간값
    double rhoEff;              // 조속기가 본 유효 점유율 (yaw 오차 환산 포함)
    double sClock;              // 가상 시계 배율 s  (호출자가 tau += s*dt 로 적분)
    // 상위 계획기에 그대로 올릴 스펙 한 장 (Python capability.json 과 같은 수).
    SpecReport spec;
};

struct QcState {
    Pid pidPosX, pidPosY, pidPosZ;   // 위치 3축
    Pid pidAttP, pidAttR;            // 자세 pitch/roll
    Pid pidYaw, pidAlt;
    Pid pidMot[4];
    Lpf1 fMeasP, fMeasR;             // 자세 측정 필터 (tau=0.05, 덤프 확정)
    Lpf1 fMeasY, fMeasZ;             // yaw(0.01)/고도(0.01) 측정 필터 (덤프 확정)
    Lpf1 fPosPath[3];                // 위치 명령 경로 필터
    YawDistI yawDistI;               // yaw 외란 적응 적분 (기본 항등)
    SpeedGovernor gov;               // 속도 조속기 (기본 항등, s≡1)
    LatencyTracker lat;              // 위치 경로 지연 추적 (measAgeS 표본)
    SpecLatencyRule specRule;        // 지연 -> 배율 실측 규칙
    RecoveryWatcher rec;             // 회복 감시 (폐루프 교정, recOn 으로 켠다)
    double specAcc = 0;              // 보고 주기 데시메이션 누적기
    SpecReport specLast;             // 마지막 보고 (틱 사이에는 이걸 그대로 낸다)
    // 데시메이션 구간의 measAgeS **최대값**. 평균을 넣으면 짧은 스파이크가 지워진다
    // (rho 를 '구간 최대'로 넣으라는 INTERFACE_SPEC §5b 규정과 같은 이유).
    double specAgeMax = 0;
    double tauClock = 0;             // 가상 시계 누적 (편의 — 참조 조회는 호출자 몫)
    double refYawPrev = 0;           // 참조 yaw 각속도 산출용
    bool   refYawFirst = true;
    void reset() {
        pidPosX.reset(); pidPosY.reset(); pidPosZ.reset();
        pidAttP.reset(); pidAttR.reset(); pidYaw.reset(); pidAlt.reset();
        for (auto& p : pidMot) p.reset();
        fMeasP.reset(); fMeasR.reset(); fMeasY.reset(); fMeasZ.reset();
        for (auto& f : fPosPath) f.reset();
        yawDistI.reset(); refYawPrev = 0; refYawFirst = true;
        gov.reset(); tauClock = 0; lat.reset();
        specAcc = 0; specAgeMax = 0; specLast = SpecReport(); rec.reset();
    }
};

// 게인을 config+스케일로부터 상태에 주입 (초기화 시 1회, 물성 변경 시 재호출)
void qc_bind(QcState& st, const QcConfig& c);

// 한 스텝 실행. dt[s] 고정 스텝 권장 (골든 트레이스는 1kHz 기준으로 대조).
QcOutput qc_step(QcState& st, const QcConfig& c, const QcInput& in, double dt);

} // namespace qc
