// main_trace.cpp — 골든 트레이스 하니스
//
// 용도: 구운 Simulink 모델의 로그(run_traj_baked 태핑: 참조/측정/모터)와 같은 입력을
//       C++ 제어기에 먹여 출력을 CSV로 남긴다. 대조는 Python(compare_trace.py)에서.
//
// 입력 CSV (헤더 1줄, 콤마):
//   t, ref_x, ref_y, ref_z, ref_yaw, meas_x, meas_y, meas_z,
//   roll, pitch, yaw, w1, w2, w3, w4
// 출력 CSV:
//   t, cmd_pitch, cmd_roll, mref1..4, u1..4
//
// 사용: qc_trace.exe input.csv output.csv
// 스모크: qc_trace.exe --smoke   (합성 스텝 입력 100스텝, 수치 정상 여부만 확인)

#include "qc_controller.hpp"
#include <cstdio>
#include <cstring>

static int run_smoke() {
    qc::QcConfig cfg;
    qc::QcState st;
    qc::qc_bind(st, cfg);

    const qc::QcScales s = qc::qc_scales(cfg);
    std::printf("스케일: sT=%.6f sQ=%.6f sIa=%.6f sIz=%.6f sM=%.6f posErrSat=%.4f\n",
                s.sT, s.sQ, s.sIa, s.sIz, s.sM, s.posErrSat);

    qc::QcInput in{};
    in.refPos[0] = 1.0; in.refPos[2] = 1.0;   // x로 1m 스텝 (스모크 전용 — 실전은 성형 궤적)
    in.measPos[2] = 1.0; in.measAlt = 1.0;
    for (int i = 0; i < 4; ++i) in.motorSpd[i] = 400.0;

    const double dt = 1e-3;
    qc::QcOutput out{};
    for (int k = 0; k < 100; ++k) out = qc::qc_step(st, cfg, in, dt);

    std::printf("100스텝 후: cmdP=%.4f rad, cmdR=%.4f rad\n", out.cmdPitch, out.cmdRoll);
    std::printf("motorRef = %.2f %.2f %.2f %.2f rev/s(x2pi rad/s)\n",
                out.motorRef[0], out.motorRef[1], out.motorRef[2], out.motorRef[3]);
    std::printf("motorCmd = %.5f %.5f %.5f %.5f\n",
                out.motorCmd[0], out.motorCmd[1], out.motorCmd[2], out.motorCmd[3]);
    const bool ok = std::isfinite(out.cmdPitch) && std::isfinite(out.motorCmd[0]);
    std::printf("%s\n", ok ? "스모크 통과 (수치 유한)" : "스모크 실패 (NaN/Inf)");
    return ok ? 0 : 1;
}

int main(int argc, char** argv) {
    if (argc >= 2 && std::strcmp(argv[1], "--smoke") == 0) return run_smoke();
    if (argc < 3) {
        std::fprintf(stderr, "사용: qc_trace <input.csv> <output.csv> | --smoke\n");
        return 2;
    }
    std::FILE* fi = std::fopen(argv[1], "r");
    if (!fi) { std::fprintf(stderr, "입력 열기 실패: %s\n", argv[1]); return 1; }
    std::FILE* fo = std::fopen(argv[2], "w");
    if (!fo) { std::fprintf(stderr, "출력 열기 실패: %s\n", argv[2]); std::fclose(fi); return 1; }

    qc::QcConfig cfg;
    qc::QcState st;
    qc::qc_bind(st, cfg);

    char line[1024];
    if (!std::fgets(line, sizeof line, fi)) { std::fclose(fi); std::fclose(fo); return 1; } // 헤더
    std::fprintf(fo, "t,cmd_pitch,cmd_roll,mref1,mref2,mref3,mref4,u1,u2,u3,u4\n");

    double tPrev = 0; bool first = true;
    long n = 0;
    while (std::fgets(line, sizeof line, fi)) {
        double v[15];
        int got = std::sscanf(line,
            "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7],
            &v[8], &v[9], &v[10], &v[11], &v[12], &v[13], &v[14]);
        if (got != 15) continue;
        const double t = v[0];
        const double dt = first ? 1e-3 : (t - tPrev);
        first = false; tPrev = t;
        if (dt <= 0) continue;

        qc::QcInput in{};
        in.refPos[0] = v[1]; in.refPos[1] = v[2]; in.refPos[2] = v[3];
        in.refYaw = v[4];
        in.measPos[0] = v[5]; in.measPos[1] = v[6]; in.measPos[2] = v[7];
        in.measRpy[0] = v[8]; in.measRpy[1] = v[9]; in.measRpy[2] = v[10];
        in.measAlt = v[7];
        for (int i = 0; i < 4; ++i) in.motorSpd[i] = v[11 + i];

        const qc::QcOutput out = qc::qc_step(st, cfg, in, dt);
        std::fprintf(fo, "%.6f,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g\n",
                     t, out.cmdPitch, out.cmdRoll,
                     out.motorRef[0], out.motorRef[1], out.motorRef[2], out.motorRef[3],
                     out.motorCmd[0], out.motorCmd[1], out.motorCmd[2], out.motorCmd[3]);
        ++n;
    }
    std::fclose(fi); std::fclose(fo);
    std::printf("%ld행 처리 완료 -> %s\n", n, argv[2]);
    return 0;
}
