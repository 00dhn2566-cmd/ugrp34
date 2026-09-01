// main_spec_trace.cpp — 지연 추적 + 스펙 보고의 언어 간 대조용 트레이스 (2026-08-23)
//
// 목적: C++ `LatencyTracker` / `qc_spec_report` 가 Python `latency_tracker` /
//       `capability` / `spec_governor` 와 **같은 수**를 내는지 확인한다.
//       제어 본체에 대해 골든 트레이스를 대조하는 것과 같은 규율을, 새로 들어온
//       지연 경로에도 적용하는 것 — 두 구현이 조용히 갈라지면 상위가 받는 스펙이
//       기체마다 달라진다.
//
// 출력: CSV (stdout).
//   기본 (인자 1개)  : 지연 추적 + 스펙 보고
//   회복 감시 (--rec) : RecoveryWatcher 단독 트레이스
// 대조: tests/test_cpp_spec_parity.py

#include <cstdio>
#include <cstdlib>
#include <string>

#include "qc_controller.hpp"

static int rec_trace() {
    // Python recovery_watcher self-test 와 **같은 수열**: 0~20 s 평온 /
    // 20~50 s 밴드 위에 머묾 / 50 s~ 정상. dt = 0.01, bridgeLead = 2.0.
    qc::RecoveryWatcher w;
    std::printf("k,err,t_above,t_clean,scale,ratio,cuts\n");
    double t = 0.0;
    for (int k = 0; k < 9000; ++k) {
        const double err = (t >= 20.0 && t < 50.0) ? 0.09 : 0.01;
        w.observe(err, true, 0.01);
        const double s = w.decide(2.0);
        std::printf("%d,%.9f,%.9f,%.9f,%.9f,%.9f,%ld\n",
                    k, err, w.tAbove, w.tClean, s, w.lastRatio, w.cuts);
        t += 0.01;
    }
    return 0;
}

static int mass_trace() {
    // 질량 1차식 덤프. MATLAB qc_mass_lerp_apply.m 의 앵커와 대조하기 위한 것.
    std::printf("m,sA,sZ,rAtt,limAtt,kpPos,kdPos,filtPz,bias,nlGmax,sA_old,sZ_old\n");
    for (int i = 0; i <= 8; ++i) {
        const double m = i * 0.125;
        const qc::MassLerp L = qc::qc_mass_lerp(m);
        std::printf("%.4f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                    m, L.sAMass, L.sZMass, L.rAtt, L.limAtt, L.kpPos, L.kdPos,
                    L.filtPz, L.biasChassis, L.nlGmax,
                    0.75 + 0.25 * m, 0.56 + 0.44 * m);
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--rec") return rec_trace();
    if (argc > 1 && std::string(argv[1]) == "--mass") return mass_trace();
    // 시나리오: 평온 -> 지연 급증 -> 해소. Python 쪽 self-test 와 같은 수열이어야 한다.
    const double attS = (argc > 1) ? atof(argv[1]) : 0.003;

    qc::LatencyTracker lat;
    qc::SpecLatencyRule rule;

    std::printf("k,sample,detected,ema_fast,ema_slow,predicted,scale,v,a,j,snap,mission\n");
    for (int k = 0; k < 120; ++k) {
        const double sample = (k >= 30 && k < 70) ? 0.075 : 0.012;
        const double pred = lat.update(sample);
        const qc::SpecReport r = qc::qc_spec_report(
            rule, 1.6, 1.6, 8.0, 64.0, 0.75, /*sDisturb=*/1.0, pred, attS);
        std::printf("%d,%.9f,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d\n",
                    k, sample, lat.detected ? 1 : 0, lat.emaFast, lat.emaSlow, pred,
                    r.timeScale, r.v, r.a, r.j, r.snap, r.missionAllowed ? 1 : 0);
    }
    return 0;
}
