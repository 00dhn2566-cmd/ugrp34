#!/usr/bin/env python3
"""Gazebo 로그(out/*.csv) -> 지표. 표준 라이브러리만 쓴다 (numpy 없어도 돈다).

    python3 analyze/gz_metrics.py out/*.csv
    python3 analyze/gz_metrics.py out/*.csv --json out/metrics.json

두 종류를 구분해 해석한다:
  probe  개루프 프로브 — 각가속도 선형 적합으로 플랜트 이득 b 를 뽑는다.
  flight 폐루프 — 호버 지터 / 추종 / 외란 이탈·복귀 / 스펙 보고.

복귀 시간 정의는 `diagnose/verify_bridge_sim.m` 와 같다: 펄스가 끝난 뒤,
남은 구간 **전체**가 밴드 안에 들어오는 첫 시각. (한 번 스쳤다 다시 나가는 것을
복귀로 세지 않으려는 것.)
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys

# 윈도우 콘솔(cp949)에서도 한글/기호가 깨지지 않게. 리눅스에서는 무해한 no-op.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001 - 출력 인코딩은 실패해도 본 작업을 막지 않는다
    pass

BAND_M = 0.02        # 복귀 판정 밴드 [m] — 능력카드 track 예산의 절반
SETTLE_SKIP_S = 2.0  # 이륙 램프가 끝난 뒤 이만큼 더 버리고 정상상태로 본다


def load(path):
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError("빈 로그: %s" % path)
    cols = {}
    for key in rows[0]:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[key]))
            except (TypeError, ValueError):
                vals.append(float("nan"))
        cols[key] = vals
    return cols


def rms(xs):
    xs = [x for x in xs if math.isfinite(x)]
    if not xs:
        return float("nan")
    return math.sqrt(sum(x * x for x in xs) / len(xs))


def linfit_slope(t, y):
    """최소제곱 기울기. 프로브의 각가속도 = d(omega)/dt 를 여기서 뽑는다."""
    pts = [(a, b) for a, b in zip(t, y) if math.isfinite(a) and math.isfinite(b)]
    n = len(pts)
    if n < 3:
        return float("nan")
    st = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    stt = sum(p[0] * p[0] for p in pts)
    sty = sum(p[0] * p[1] for p in pts)
    den = n * stt - st * st
    if abs(den) < 1e-15:
        return float("nan")
    return (n * sty - st * sy) / den


def probe_metrics(name, c):
    """개루프 프로브: 차동 명령 u 와 그때의 각가속도로 플랜트 이득을 낸다."""
    t = c["t"]
    mref = [c["mref%d" % (i + 1)] for i in range(4)]
    # u 는 로그에 없으니 명령에서 되뽑는다: mref_i = 2*pi*(base + mix_i*u)
    diff = [(max(m[k] for m in mref) - min(m[k] for m in mref)) / (2 * math.pi) / 2.0
            for k in range(len(t))]
    u_on = max(diff) if diff else 0.0
    # 차동이 켜진 구간
    on = [k for k, d in enumerate(diff) if d > 0.5 * u_on and u_on > 1e-9]
    res = {"case": name, "kind": "probe", "u": u_on}
    if not on or u_on <= 1e-9:
        res["note"] = "차동이 안 걸렸다 (probeU=0?)"
        return res
    # 모터 1차 지연(0.02 s)이 지난 뒤부터 적합한다.
    t0 = t[on[0]] + 0.05
    t1 = t[on[-1]]
    idx = [k for k in on if t0 <= t[k] <= t1]
    if len(idx) < 5:
        res["note"] = "적합 구간이 너무 짧다"
        return res
    tt = [t[k] for k in idx]
    for axis, key in (("x", "wx"), ("y", "wy"), ("z", "wz")):
        alpha = linfit_slope(tt, [c[key][k] for k in idx])
        res["alpha_%s" % axis] = alpha
        res["b_%s" % axis] = alpha / u_on
    # 어느 축이 주축인지 + 교차결합
    mags = {a: abs(res["b_%s" % a]) for a in ("x", "y", "z")}
    main = max(mags, key=mags.get)
    other = sum(v for a, v in mags.items() if a != main)
    res["main_axis"] = main
    res["b_main"] = res["b_%s" % main]
    res["crosstalk"] = other / mags[main] if mags[main] > 1e-12 else float("inf")
    res["tau_main"] = {"x": rms(c["tau_x"]), "y": rms(c["tau_y"]), "z": rms(c["tau_z"])}[main]
    return res


def flight_metrics(name, c):
    t = c["t"]
    n = len(t)
    ex = [c["ref_x"][k] - c["x"][k] for k in range(n)]
    ey = [c["ref_y"][k] - c["y"][k] for k in range(n)]
    ez = [c["ref_z"][k] - c["z"][k] for k in range(n)]
    err = [math.sqrt(ex[k] ** 2 + ey[k] ** 2 + ez[k] ** 2) for k in range(n)]
    lat = [math.hypot(ex[k], ey[k]) for k in range(n)]
    tilt = [math.degrees(math.hypot(c["roll"][k], c["pitch"][k])) for k in range(n)]

    dist = c.get("dist_on", [0.0] * n)
    on = [k for k in range(n) if dist[k] > 0.5]
    t_pulse0 = t[on[0]] if on else None
    t_pulse1 = t[on[-1]] if on else None

    # 정상상태 구간: 이륙이 끝나고 외란이 오기 전
    t_settle = min(x for x in t) + 0.0
    ramp_end = max(t) * 0.0
    # 이륙 시간은 로그에 없다. 기준이 움직이지 않게 된 시점을 정상상태 시작으로 본다.
    for k in range(1, n):
        moved = (abs(c["ref_x"][k] - c["ref_x"][k - 1]) + abs(c["ref_y"][k] - c["ref_y"][k - 1])
                 + abs(c["ref_z"][k] - c["ref_z"][k - 1]))
        if moved > 1e-6:
            ramp_end = t[k]
    t_settle = ramp_end + SETTLE_SKIP_S
    hi = t_pulse0 if t_pulse0 is not None else t[-1]
    ss = [k for k in range(n) if t_settle <= t[k] < hi]
    if len(ss) < 5:
        ss = list(range(int(n * 0.6), n))

    res = {
        "case": name, "kind": "flight", "T": t[-1], "rows": n,
        "hover_att_rms_deg": rms([math.degrees(c["roll"][k]) for k in ss]
                                 + [math.degrees(c["pitch"][k]) for k in ss]),
        "hover_att_max_deg": max((tilt[k] for k in ss), default=float("nan")),
        "track_rms_cm": 100.0 * rms([err[k] for k in ss]),
        "track_max_cm": 100.0 * max((err[k] for k in ss), default=float("nan")),
        "z_err_rms_cm": 100.0 * rms([ez[k] for k in ss]),
        "tilt_max_deg": max(tilt),
        "diverged": (not all(math.isfinite(v) for v in err)) or max(err) > 5.0,
    }

    if t_pulse0 is not None:
        after = [k for k in range(n) if t[k] >= t_pulse0]
        res["dist_t"] = t_pulse0
        res["dist_lat_max_cm"] = 100.0 * max(lat[k] for k in after)
        res["dist_tilt_max_deg"] = max(tilt[k] for k in after)
        # 복귀: 펄스 종료 후, 남은 구간 전체가 밴드 안인 첫 시각
        rec = None
        post = [k for k in range(n) if t[k] > t_pulse1]
        ok = [lat[k] < BAND_M for k in range(n)]
        for j, k in enumerate(post):
            if all(ok[m] for m in post[j:]):
                rec = t[k] - t_pulse1
                break
        res["dist_recover_s"] = rec          # None = 밴드로 못 돌아옴
    # 권한 점유율 rho: 정상풍처럼 지속되는 외란은 여기서 드러난다
    # (INTERFACE_SPEC 5b — 정상상태 적분기가 곧 외란 추정치다).
    if "rho" in c:
        res["rho_max"] = max((c["rho"][k] for k in ss), default=float("nan"))
        res["rho_max_all"] = max(c["rho"])
    # 사용 전력량 (energy.py 와 같은 식). MATLAB verify_worstcase 와 나란히 볼 수 있다.
    if "E_est_Wh" in c:
        res["energy_wh"] = c["E_est_Wh"][-1]
        res["power_mean_w"] = (res["energy_wh"] * 3600.0 / t[-1]) if t[-1] > 1e-9 else 0.0
        res["power_peak_w"] = max(c["P_est_W"])
    # 스펙 보고 (관측 전용 사슬이 실제로 돌았는지)
    if "spec_scale" in c:
        res["spec_scale_end"] = c["spec_scale"][-1]
        res["spec_lat_pos_end"] = c["spec_lat_pos"][-1]
        res["spec_lat_att_end"] = c["spec_lat_att"][-1]
        res["spec_lat_applied_s"] = c["lat_applied"][-1]
        res["mission_allowed"] = bool(c["mission_allowed"][-1] > 0.5)
        res["spec_v_end"] = c["spec_v"][-1]
    return res


def fmt(v, spec="%.3f"):
    if v is None:
        return "  -  "
    if isinstance(v, bool):
        return "예" if v else "아니오"
    if isinstance(v, float) and not math.isfinite(v):
        return " nan "
    return spec % v if isinstance(v, float) else str(v)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Gazebo 로그 -> 지표")
    ap.add_argument("csv", nargs="+", help="out/*.csv")
    ap.add_argument("--json", help="지표를 JSON 으로도 저장")
    args = ap.parse_args(argv)

    paths = []
    for pat in args.csv:
        hits = sorted(glob.glob(pat))
        paths.extend(hits if hits else [pat])
    if not paths:
        print("로그가 없다", file=sys.stderr)
        return 1

    out = []
    for p in paths:
        name = os.path.splitext(os.path.basename(p))[0]
        try:
            c = load(p)
        except Exception as exc:            # noqa: BLE001 - 어느 파일이 깨졌는지 알려야 한다
            print("  ! %s: %s" % (name, exc))
            continue
        out.append(probe_metrics(name, c) if name.startswith("probe") else flight_metrics(name, c))

    probes = [r for r in out if r["kind"] == "probe"]
    flights = [r for r in out if r["kind"] == "flight"]

    if probes:
        print()
        print("== 개루프 프로브 (플랜트 이득) ==")
        print("%-26s %6s %8s %10s %10s %10s %9s" %
              ("케이스", "u", "주축", "b_main", "b_x", "b_y", "b_z"))
        for r in probes:
            if "b_main" not in r:
                print("%-26s %6s  %s" % (r["case"], fmt(r.get("u"), "%.2f"), r.get("note", "")))
                continue
            print("%-26s %6.2f %8s %10.4f %10.4f %10.4f %9.4f" %
                  (r["case"], r["u"], r["main_axis"], r["b_main"],
                   r["b_x"], r["b_y"], r["b_z"]))
        print()
        print("  읽는 법: b = (각가속도)/(차동 명령 u[rev/s]) [rad/s^2 per rev/s].")
        print("  - 부호가 예상과 반대면 게인을 뒤집지 말 것 (절대 규칙 1). 월드의 로터")
        print("    사분면(= 믹서 표) 을 고쳐야 한다.")
        print("  - yaw 프로브의 b_z 가 ~0 이면 mixYaw 가 mixDir 와 직교한 것이다.")
        print("    probe_yaw 와 probe_yaw_headertable 을 나란히 볼 것.")

    if flights:
        print()
        print("== 폐루프 ==")
        hdr = ("%-22s %7s %8s %8s %8s %9s %9s %8s %7s" %
               ("케이스", "자세RMS", "추종RMS", "추종max", "경사max",
                "외란이탈", "복귀", "스펙s", "발산"))
        print(hdr)
        print("%-22s %7s %8s %8s %8s %9s %9s %8s %7s" %
              ("", "[deg]", "[cm]", "[cm]", "[deg]", "[cm]", "[s]", "", ""))
        for r in flights:
            print("%-22s %7.3f %8.2f %8.2f %8.2f %9s %9s %8s %7s" %
                  (r["case"], r["hover_att_rms_deg"], r["track_rms_cm"],
                   r["track_max_cm"], r["tilt_max_deg"],
                   fmt(r.get("dist_lat_max_cm"), "%.2f"),
                   fmt(r.get("dist_recover_s"), "%.2f"),
                   fmt(r.get("spec_scale_end"), "%.2f"),
                   "예" if r["diverged"] else ""))
        print()
        print("  복귀가 '-' 면 밴드(%.0f cm)로 못 돌아온 것이다 — 시간 초과이지 0 이 아니다."
              % (BAND_M * 100))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)
        print()
        print("JSON: %s" % args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
