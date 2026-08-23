// main_spec_trace.cpp — 지연 추적 + 스펙 보고의 언어 간 대조용 트레이스 (2026-08-23)
//
// 목적: C++ `LatencyTracker` / `qc_spec_report` 가 Python `latency_tracker` /
//       `capability` / `spec_governor` 와 **같은 수**를 내는지 확인한다.
//       제어 본체에 대해 골든 트레이스를 대조하는 것과 같은 규율을, 새로 들어온
//       지연 경로에도 적용하는 것 — 두 구현이 조용히 갈라지면 상위가 받는 스펙이
//       기체마다 달라진다.
//
// 출력: CSV (stdout). 열 = k,sample,detected,ema_fast,ema_slow,predicted,
//                          scale,v,a,j,snap,mission
// 대조: tools/compare_spec_trace.py

#include <cstdio>

#include "qc_controller.hpp"

int main(int argc, char** argv) {
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
