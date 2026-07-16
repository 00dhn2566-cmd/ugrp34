"""sim_result_baked.mat → qc_trace.exe 입력 CSV 변환 + 재생 개연성 검사.

골든 트레이스의 전반부: MATLAB 실비행 로그에서 C++ 제어기 입력(참조/측정/모터속도)을
추출한다. 제어기 출력(cmd/u) 골든은 아직 로깅에 없으므로, 후반부(완전 대조)는
run_traj_baked에 cmd 탭 추가 후 compare_trace.py로. 그 전까지는 이 스크립트의
--check 모드가 C++ motorRef ↔ 실측 모터속도(prop_w)의 상관/스케일로 개연성만 판정.

사용:
    python make_golden_input.py <sim_result_baked.mat> <out_input.csv>
    python make_golden_input.py <mat> <input.csv> --check <cpp_out.csv>
"""
import argparse
import csv
import sys

import numpy as np
from scipy.io import loadmat

if hasattr(sys.stdout, "reconfigure"):   # cp949 콘솔 방어 (PIPELINE_STATUS 교훈)
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def ts_series(entry):
    """To Workspace StructureWithTime → (t, y) 1D."""
    rec = entry[0, 0]
    t = np.asarray(rec["time"]).ravel()
    y = np.asarray(rec["signals"]["values"][0, 0]).squeeze()
    if y.ndim > 1:
        y = y[:, 0]
    return t, y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mat")
    ap.add_argument("out_csv")
    ap.add_argument("--check", help="qc_trace 출력 CSV — motorRef↔실측 w 개연성 검사")
    args = ap.parse_args()

    m = loadmat(args.mat)
    need = ["sim_time", "act_x1", "act_y1", "act_z1", "des_x1", "des_y1", "des_z1",
            "real_pitch", "real_roll", "real_yaw",
            "prop1_w", "prop2_w", "prop3_w", "prop4_w",
            "timespot_spl", "spline_yaw"]
    missing = [k for k in need if k not in m]
    if missing:
        sys.exit(f"[즉사] mat 변수 누락: {missing}")

    t = m["sim_time"].ravel()
    ref = {ax: m[f"des_{ax}1"].ravel() for ax in "xyz"}
    act = {ax: m[f"act_{ax}1"].ravel() for ax in "xyz"}

    def resamp(key):
        tt, yy = ts_series(m[key])
        return np.interp(t, tt, yy)

    pitch = resamp("real_pitch")
    roll = resamp("real_roll")
    yaw = resamp("real_yaw")
    w = [resamp(f"prop{i}_w") for i in range(1, 5)]

    ref_yaw = np.interp(t, m["timespot_spl"].ravel(), m["spline_yaw"].ravel())

    with open(args.out_csv, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["t", "ref_x", "ref_y", "ref_z", "ref_yaw",
                     "meas_x", "meas_y", "meas_z", "roll", "pitch", "yaw",
                     "w1", "w2", "w3", "w4"])
        for i in range(len(t)):
            wr.writerow([f"{t[i]:.6f}",
                         f"{ref['x'][i]:.8g}", f"{ref['y'][i]:.8g}", f"{ref['z'][i]:.8g}",
                         f"{ref_yaw[i]:.8g}",
                         f"{act['x'][i]:.8g}", f"{act['y'][i]:.8g}", f"{act['z'][i]:.8g}",
                         f"{roll[i]:.8g}", f"{pitch[i]:.8g}", f"{yaw[i]:.8g}",
                         f"{w[0][i]:.8g}", f"{w[1][i]:.8g}", f"{w[2][i]:.8g}", f"{w[3][i]:.8g}"])
    print(f"입력 CSV 생성: {args.out_csv} ({len(t)}행, {t[0]:.2f}~{t[-1]:.2f}s)")
    print(f"모터속도 w1 범위: {w[0].min():.1f}~{w[0].max():.1f} (단위 확인용 — rad/s면 ~2500, rev/s면 ~400)")

    if args.check:
        rows = list(csv.DictReader(open(args.check, newline="", encoding="utf-8")))
        tc = np.array([float(r["t"]) for r in rows])
        for mi in range(4):
            mref = np.array([float(r[f"mref{mi+1}"]) for r in rows])
            wi = np.interp(tc, t, w[mi])
            # 상관 + 스케일비 (motorRef 단위 가설 검증: rad/s vs rev/s)
            c = np.corrcoef(mref, wi)[0, 1]
            scale = np.median(wi / np.where(np.abs(mref) < 1e-9, np.nan, mref))
            print(f"모터{mi+1}: corr(motorRef, 실측w)={c:+.4f}  중앙값 스케일비 w/mref={scale:.4f}")
        print("(해석: corr 높고 스케일비 일정하면 체인 개연성 OK. 완전 대조는 cmd 탭 추가 후)")


if __name__ == "__main__":
    main()
