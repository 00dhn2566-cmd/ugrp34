"""골든 트레이스 대조기: MATLAB(구운 모델) 제어기 로그 vs C++(qc_trace.exe) 출력.

사용:
    python compare_trace.py golden.csv cpp_out.csv [--tol-motor 0.02] [--tol-cmd 0.02]

입력 형식:
    golden.csv : t, cmd_pitch, cmd_roll, mref1..4, u1..4  (MATLAB 채취 — 골든)
    cpp_out.csv: 동일 컬럼 (qc_trace.exe 출력)

판정 (채널별):
    - RMS 오차 / 풀스케일  <= tol   (모터 명령 풀스케일 = limit_motor*2 = 0.5)
    - 상관계수 >= 0.99
    어긋나면 처음 갈라지는 시각과 채널을 보고 — 단계별 배선 오류 국소화용.
저장소 규칙: 파일/컬럼 불일치 시 조용히 통과 금지, 즉사.
"""
import argparse
import csv
import math
import sys

COLS = ["t", "cmd_pitch", "cmd_roll",
        "mref1", "mref2", "mref3", "mref4",
        "u1", "u2", "u3", "u4"]
FULLSCALE = {"cmd_pitch": 2.0944, "cmd_roll": 2.0944,      # ±60도 = 1.047rad -> 폭 2.094
             "mref1": 1600.0, "mref2": 1600.0, "mref3": 1600.0, "mref4": 1600.0,  # 대략 rev/s*2pi 스팬
             "u1": 0.5, "u2": 0.5, "u3": 0.5, "u4": 0.5}   # limit_motor ±0.25


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        rd = csv.DictReader(f)
        missing = [c for c in COLS if c not in (rd.fieldnames or [])]
        if missing:
            sys.exit(f"[즉사] {path} 컬럼 누락: {missing} (있는 것: {rd.fieldnames})")
        rows = [{c: float(r[c]) for c in COLS} for r in rd]
    if len(rows) < 10:
        sys.exit(f"[즉사] {path} 행 부족: {len(rows)}")
    return rows


def interp_series(rows, key, tq):
    ts = [r["t"] for r in rows]
    vs = [r[key] for r in rows]
    if tq <= ts[0]:
        return vs[0]
    if tq >= ts[-1]:
        return vs[-1]
    lo, hi = 0, len(ts) - 1
    while hi - lo > 1:
        m = (lo + hi) // 2
        if ts[m] <= tq:
            lo = m
        else:
            hi = m
    w = (tq - ts[lo]) / (ts[hi] - ts[lo])
    return vs[lo] + w * (vs[hi] - vs[lo])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("golden")
    ap.add_argument("cpp")
    ap.add_argument("--tol-motor", type=float, default=0.02)   # 풀스케일 2%
    ap.add_argument("--tol-cmd", type=float, default=0.02)
    args = ap.parse_args()

    g = load(args.golden)
    c = load(args.cpp)

    t0 = max(g[0]["t"], c[0]["t"])
    t1 = min(g[-1]["t"], c[-1]["t"])
    if t1 - t0 < 1.0:
        sys.exit(f"[즉사] 시간 겹침 부족: [{t0}, {t1}]")
    n = 2000
    ts = [t0 + (t1 - t0) * i / (n - 1) for i in range(n)]

    print(f"대조 구간 [{t0:.2f}, {t1:.2f}]s, {n} 샘플")
    print(f"{'채널':>10} | {'RMS/FS %':>9} {'상관':>7} {'최대오차':>10} {'첫이탈(s)':>9} | 판정")
    all_ok = True
    for key in COLS[1:]:
        gv = [interp_series(g, key, t) for t in ts]
        cv = [interp_series(c, key, t) for t in ts]
        errs = [a - b for a, b in zip(gv, cv)]
        rms = math.sqrt(sum(e * e for e in errs) / n)
        fs = FULLSCALE[key]
        tol = args.tol_cmd if key.startswith("cmd") else args.tol_motor
        # 상관계수
        mg = sum(gv) / n
        mc = sum(cv) / n
        cov = sum((a - mg) * (b - mc) for a, b in zip(gv, cv))
        vg = math.sqrt(sum((a - mg) ** 2 for a in gv))
        vc = math.sqrt(sum((b - mc) ** 2 for b in cv))
        corr = cov / (vg * vc) if vg > 1e-12 and vc > 1e-12 else float("nan")
        # 첫 이탈 시각 (풀스케일 tol 초과)
        first_dev = next((ts[i] for i, e in enumerate(errs) if abs(e) > tol * fs), None)
        max_err = max(abs(e) for e in errs)
        ok = (rms / fs <= tol) and (math.isnan(corr) or corr >= 0.99)
        all_ok &= ok
        print(f"{key:>10} | {100*rms/fs:>8.3f}% {corr:>7.4f} {max_err:>10.4g} "
              f"{('-' if first_dev is None else f'{first_dev:.2f}'):>9} | {'합격' if ok else '불합격'}")

    print("\n>>", "골든 트레이스 일치 — 이식 검증 통과" if all_ok
          else "불일치 — 첫이탈 시각/채널로 배선 오류 국소화할 것 ([TODO-verify] 목록 대조)")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
