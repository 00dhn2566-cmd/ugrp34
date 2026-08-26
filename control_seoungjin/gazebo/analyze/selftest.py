#!/usr/bin/env python3
"""분석 사슬 자체 시험 — **Gazebo 없이** 돈다.

    python3 analyze/selftest.py

합성 로그를 만들어 gz_metrics.py 가 아는 답을 내는지 확인한다. 지표 코드가
깨졌는지, 실제 비행이 나쁜지를 구분하려면 이게 먼저 있어야 한다. Gazebo 머신에
가기 전에 이 노트북에서도 돌려 볼 수 있고, 가서도 실측 전에 한 번 돌리면 좋다.

검사 항목:
  1. probe  — 알고 넣은 각가속도를 그대로 되뽑는가 (b = alpha/u)
  2. probe  — yaw 권한 0 인 경우를 0 으로 보고하는가
  3. flight — 호버 지터 RMS / 추종 RMS
  4. flight — 외란 복귀 시간 (한 번 스쳤다 다시 나가는 것을 복귀로 세지 않는가)
  5. flight — 밴드로 못 돌아오면 None (0 이 아니라)
"""
from __future__ import annotations

import csv
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gz_metrics  # noqa: E402

# 윈도우 콘솔(cp949)에서도 한글/기호가 깨지지 않게. 리눅스에서는 무해한 no-op.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001 - 출력 인코딩은 실패해도 본 작업을 막지 않는다
    pass

COLS = ("t,ref_x,ref_y,ref_z,ref_yaw,x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,"
        "cmd_pitch,cmd_roll,mref1,mref2,mref3,mref4,w1,w2,w3,w4,"
        "thrust,tau_x,tau_y,tau_z,rho,s_clock,"
        "spec_scale,spec_v,spec_a,spec_lat_pos,spec_lat_att,spec_rec,"
        "P_est_W,E_est_Wh,lat_applied,mission_allowed,dist_on").split(",")


def write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(COLS)
        for r in rows:
            w.writerow([r.get(c, 0.0) for c in COLS])


def make_probe(path, alpha_y, u, dur=1.0, t_on=1.0, T=3.0, dt=0.001):
    """t_on 부터 dur 동안 차동 u 를 넣고 wy 가 alpha_y 로 증가하는 로그."""
    base = 100.9
    rows = []
    t = 0.0
    wy = 0.0
    while t <= T + 1e-9:
        on = t_on <= t < t_on + dur
        uu = u if on else 0.0
        if on and t >= t_on + 0.05:      # 모터 지연 뒤부터 선형 증가
            wy += alpha_y * dt
        mix = (+1, +1, -1, -1)
        r = {"t": t, "wy": wy, "ref_z": 1.0, "z": 1.0}
        for i in range(4):
            r["mref%d" % (i + 1)] = 2 * math.pi * (base + mix[i] * uu)
        rows.append(r)
        t += dt
    write_csv(path, rows)


def make_flight(path, jitter_deg=0.5, track_cm=3.0, pulse_t=None,
                recover_s=1.0, never_recover=False, T=16.0, dt=0.005,
                takeoff_s=3.0):
    rows = []
    t = 0.0
    while t <= T + 1e-9:
        s = min(1.0, t / takeoff_s)
        ramp = 35 * s ** 4 - 84 * s ** 5 + 70 * s ** 6 - 20 * s ** 7
        ref_z = ramp * 1.0
        # 정상상태 지터/추종 오차 (결정론적 톱니 — 난수 안 씀)
        ph = math.sin(2 * math.pi * 3.0 * t)
        jit = math.radians(jitter_deg) * math.sqrt(2.0) * ph
        ex = (track_cm / 100.0) * math.sqrt(2.0) * ph
        y = 0.0
        dist_on = 0
        if pulse_t is not None and t >= pulse_t:
            if t < pulse_t + 0.3:
                dist_on = 1
            dtp = t - (pulse_t + 0.3)
            if dtp >= 0:
                amp = 0.10
                if never_recover:
                    y = amp * math.cos(2 * math.pi * 0.5 * dtp)   # 안 잦아드는 진동
                else:
                    # recover_s 에 정확히 2 cm 밴드로 들어오게
                    tau = recover_s / math.log(amp / 0.0199)
                    y = amp * math.exp(-dtp / tau)
            else:
                y = 0.10 * (t - pulse_t) / 0.3
        rows.append({
            "t": t, "ref_x": 0.0, "ref_y": 0.0, "ref_z": ref_z, "ref_yaw": 0.0,
            "x": ex, "y": y, "z": ref_z, "roll": jit, "pitch": jit, "yaw": 0.0,
            "dist_on": dist_on, "spec_scale": 1.0, "spec_v": 1.6, "spec_a": 1.6,
            "spec_lat_pos": 1.0, "spec_lat_att": 1.0, "spec_rec": 1.0,
            "lat_applied": 0.0, "mission_allowed": 1,
        })
        t += dt
    write_csv(path, rows)


def close(a, b, tol, what):
    ok = a is not None and b is not None and math.isfinite(a) and abs(a - b) <= tol
    print("  %-46s 기대 %-10s 실제 %-10s %s"
          % (what,
             "%.4g" % b,
             ("%.4g" % a) if isinstance(a, float) and math.isfinite(a) else str(a),
             "통과" if ok else "실패"))
    return ok


def main():
    tmp = tempfile.mkdtemp(prefix="qcgz_")
    fails = 0
    print("합성 로그 위치: %s" % tmp)

    print()
    print("[1] probe — b = alpha/u 를 되뽑는가")
    p = os.path.join(tmp, "probe_pitch.csv")
    make_probe(p, alpha_y=2.9, u=1.0)
    r = gz_metrics.probe_metrics("probe_pitch", gz_metrics.load(p))
    fails += not close(r.get("u"), 1.0, 1e-6, "차동 u 되뽑기")
    fails += not close(r.get("b_y"), 2.9, 0.05, "b_y (= alpha_y / u)")
    fails += not close(1.0 if r.get("main_axis") == "y" else 0.0, 1.0, 0.0, "주축이 y")

    print()
    print("[2] probe — yaw 권한 0 을 0 으로 보고하는가")
    p = os.path.join(tmp, "probe_yaw.csv")
    make_probe(p, alpha_y=0.0, u=1.0)
    r = gz_metrics.probe_metrics("probe_yaw", gz_metrics.load(p))
    fails += not close(abs(r.get("b_y", 9.9)) < 1e-6, True, 0, "b_y ~ 0")

    print()
    print("[3] flight — 호버 지터 / 추종")
    p = os.path.join(tmp, "hover10.csv")
    make_flight(p, jitter_deg=0.5, track_cm=3.0, pulse_t=None, T=13.0)
    r = gz_metrics.flight_metrics("hover10", gz_metrics.load(p))
    fails += not close(r["hover_att_rms_deg"], 0.5, 0.05, "자세 RMS [deg]")
    fails += not close(r["track_rms_cm"], 3.0, 0.3, "추종 RMS [cm]")
    fails += not close(1.0 if not r["diverged"] else 0.0, 1.0, 0.0, "발산 아님")

    print()
    print("[4] flight — 외란 복귀 시간")
    p = os.path.join(tmp, "pulse_y.csv")
    make_flight(p, jitter_deg=0.1, track_cm=0.5, pulse_t=8.0, recover_s=1.0, T=16.0)
    r = gz_metrics.flight_metrics("pulse_y", gz_metrics.load(p))
    fails += not close(r.get("dist_recover_s"), 1.0, 0.06, "복귀 [s]")
    fails += not close(r.get("dist_lat_max_cm"), 10.0, 0.6, "외란 이탈 [cm]")

    print()
    print("[5] flight — 못 돌아오면 None (0 아님)")
    p = os.path.join(tmp, "pulse_bad.csv")
    make_flight(p, jitter_deg=0.1, track_cm=0.5, pulse_t=8.0, never_recover=True, T=16.0)
    r = gz_metrics.flight_metrics("pulse_bad", gz_metrics.load(p))
    got = r.get("dist_recover_s")
    ok = got is None
    print("  %-46s 기대 %-10s 실제 %-10s %s"
          % ("복귀 실패 표기", "None", str(got), "통과" if ok else "실패"))
    fails += not ok

    print()
    if fails:
        print("실패 %d 건 — 지표 코드를 고치기 전에는 실측 숫자를 믿지 말 것." % fails)
        return 1
    print("전부 통과. 지표 코드는 정상이다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
