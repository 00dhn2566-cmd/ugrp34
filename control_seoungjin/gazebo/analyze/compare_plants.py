#!/usr/bin/env python3
"""Gazebo 지표 vs Simulink 기록값. 두 플랜트가 같은 소리를 하는지 본다.

    python3 analyze/gz_metrics.py out/*.csv --json out/metrics.json
    python3 analyze/compare_plants.py                     # out/metrics.json 을 읽는다

원칙: **조건이 같은 것만 맞댄다.** simulink_ref.json 의 comparable=false 는 참고
표시로만 찍고 판정하지 않는다. 조건이 다른 숫자를 나란히 놓고 '일치/불일치' 라고
쓰는 것이 이 프로젝트에서 제일 위험한 종류의 거짓말이다.

무엇을 증명하려는가:
  1) 부호 사슬 — Gazebo 에서도 같은 방향으로 난다 (probe)
  2) 무외란 기준 — 두 플랜트의 성적이 같은 자릿수다 (base)
  3) 지연 절벽 — 08-23 지연->스펙 표가 Simscape 과적합이 아니다 (delay)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

# 윈도우 콘솔(cp949)에서도 한글/기호가 깨지지 않게. 리눅스에서는 무해한 no-op.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001 - 출력 인코딩은 실패해도 본 작업을 막지 않는다
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_METRICS = os.path.join(HERE, "..", "out", "metrics.json")
DEFAULT_REF = os.path.join(HERE, "simulink_ref.json")


def by_case(metrics):
    return {m["case"]: m for m in metrics}


def show_probe(m):
    print()
    print("== 1. 부호 사슬 / 플랜트 이득 (개루프) ==")
    rows = [(ch, m.get("probe_" + ch)) for ch in ("pitch", "roll", "yaw")]
    if not any(r[1] for r in rows):
        print("  프로브 결과 없음 — bash scripts/run_matrix.sh probe")
        return
    # 기대: pitch 는 y축, roll 은 x축, yaw 는 z축이 주축이고 부호가 +.
    want = {"pitch": ("y", +1), "roll": ("x", +1), "yaw": ("z", +1)}
    print("  %-8s %8s %8s %12s %10s  %s" % ("채널", "기대축", "주축", "b_main", "교차결합", "판정"))
    for ch, r in rows:
        if not r or "b_main" not in r:
            print("  %-8s %8s   (결과 없음)" % (ch, want[ch][0]))
            continue
        ax, sgn = want[ch]
        ok_axis = r["main_axis"] == ax
        ok_sign = (r["b_main"] > 0) == (sgn > 0)
        if not ok_axis:
            verdict = "축이 다름 — 로터 사분면(믹서 표) 재확인"
        elif not ok_sign:
            verdict = "부호 반대 — 게인 말고 월드 기하를 고칠 것"
        elif abs(r["b_main"]) < 1e-3:
            verdict = "권한 거의 0 — mixYaw/mixDir 정렬 확인"
        else:
            verdict = "통과"
        print("  %-8s %8s %8s %12.4f %10.3f  %s"
              % (ch, ax, r["main_axis"], r["b_main"], r["crosstalk"], verdict))
    hdr = m.get("probe_yaw_headertable")
    yaw = m.get("probe_yaw")
    if hdr and yaw and "b_main" in hdr and "b_main" in yaw:
        print()
        print("  대조군 (C++ 헤더 믹서표): b_z = %.5f  vs  실측표 b_z = %.5f"
              % (hdr.get("b_z", float("nan")), yaw.get("b_z", float("nan"))))
        if abs(hdr.get("b_z", 0.0)) < 0.05 * max(abs(yaw.get("b_z", 1e-9)), 1e-9):
            print("  -> 예측대로 헤더 표는 yaw 권한이 0 이다 "
                  "(sum(mixDir_i*mixYaw_i)=0). qc_controller.hpp 의 mixYaw 를 고쳐야 한다.")


def show_base(m, ref):
    print()
    print("== 2. 무외란 기준 (Simulink 기록값과) ==")
    print("  %-22s %10s %10s %8s  %s" % ("항목", "Simulink", "Gazebo", "비", "조건"))
    for key, spec in ref["base"].items():
        case = m.get(spec["case"])
        gz = None
        if case:
            field = {
                "hover_att_rms_deg": "hover_att_rms_deg",
                "hover_jitter_deg": "hover_att_rms_deg",
                "move_track_rms_cm": "track_rms_cm",
                "pulse_recover_s": "dist_recover_s",
                "pulse_tilt_deg": "dist_tilt_max_deg",
                "recover_baseline_1kg_s": "dist_recover_s",
            }.get(key)
            gz = case.get(field) if field else None
        mark = "" if spec["comparable"] else "  [조건 다름 — 참고만]"
        if gz is None or (isinstance(gz, float) and not math.isfinite(gz)):
            print("  %-22s %10.3f %10s %8s  %s%s"
                  % (key, spec["value"], "-", "-", spec["condition"][:40], mark))
            continue
        ratio = gz / spec["value"] if spec["value"] else float("inf")
        print("  %-22s %10.3f %10.3f %8.2f  %s%s"
              % (key, spec["value"], gz, ratio, spec["condition"][:40], mark))
    print()
    print("  비 = Gazebo / Simulink. 같은 자릿수(0.3~3)면 두 플랜트가 같은 소리를 하는 것이다.")
    print("  10배 이상 벌어지면 물성/기하가 어긋난 것이지 제어기 문제가 아닐 가능성이 높다.")


def interp_anchor(anchors, tau):
    ks = sorted(float(k) for k in anchors)
    if tau <= ks[0]:
        return anchors[("%.3f" % ks[0])]
    if tau >= ks[-1]:
        return anchors[("%.3f" % ks[-1])]
    for a, b in zip(ks, ks[1:]):
        if a <= tau <= b:
            va, vb = anchors["%.3f" % a], anchors["%.3f" % b]
            w = (tau - a) / (b - a) if b > a else 0.0
            return va + (vb - va) * w
    return float("nan")



def fmt_rho(r):
    """로그의 rho(권한 점유율) 요약. gz_metrics 가 안 내면 '-'."""
    v = r.get("rho_max")
    return "-" if v is None else "%.2f" % v

def show_delay(m, ref):
    print()
    print("== 3. 지연 절벽 (08-23 스펙표의 독립 재측정) ==")
    anchors = {k: v for k, v in ref["lat_pos_anchors"].items() if not k.startswith("_")}
    rows = []
    for ms in (0, 20, 40, 60, 80, 120):
        rows.append((ms, m.get("tau%d_clean" % ms), m.get("tau%d_pulse" % ms)))
    if not any(r[1] or r[2] for r in rows):
        print("  지연 스윕 결과 없음 — bash scripts/run_matrix.sh delay")
        return
    base_rec = None
    b0 = m.get("tau0_pulse")
    if b0:
        base_rec = b0.get("dist_recover_s")
    print("  %5s %9s %10s %10s %10s  %s"
          % ("tau", "표의 s", "추종RMS", "외란이탈", "복귀", "판정"))
    print("  %5s %9s %10s %10s %10s" % ("[ms]", "(Simu)", "[cm]", "[cm]", "[s]"))
    for ms, clean, pulse in rows:
        s_tab = interp_anchor(anchors, ms / 1000.0)
        trk = clean.get("track_rms_cm") if clean else None
        dev = pulse.get("dist_lat_max_cm") if pulse else None
        rec = pulse.get("dist_recover_s") if pulse else None
        div = (clean and clean.get("diverged")) or (pulse and pulse.get("diverged"))
        # 판정: 표가 '괜찮다'(s>=0.9)는데 Gazebo 가 못 버티면 표가 낙관적이고,
        #      표가 '못 난다'(s<=0.05)는데 Gazebo 가 멀쩡하면 표가 보수적이다.
        if div:
            verdict = "발산"
        elif rec is None and s_tab >= 0.9:
            verdict = "표 낙관 — Gazebo 는 복귀 실패"
        elif rec is not None and s_tab <= 0.05:
            verdict = "표 보수 — Gazebo 는 복귀함"
        elif rec is not None and base_rec and rec > 3.0 * base_rec and s_tab >= 0.9:
            verdict = "표 낙관 — 복귀가 기준선의 %.1f배" % (rec / base_rec)
        else:
            verdict = "정합"
        print("  %5d %9.2f %10s %10s %10s  %s"
              % (ms, s_tab,
                 "-" if trk is None else "%.2f" % trk,
                 "-" if dev is None else "%.2f" % dev,
                 "-" if rec is None else "%.2f" % rec,
                 verdict))
    print()
    print("  주의: 표의 s 는 '이만큼 깎으면 버틴다'는 값이고, 여기 Gazebo 행은 전부")
    print("  s=1.00(안 깎은 상태)로 돌린 것이다. 그래서 두 열을 직접 빼면 안 되고,")
    print("  '표가 깎으라고 한 구간에서 실제로 힘들어하는가' 만 본다.")

    # --- 지연 x 돌풍 ---
    gust_rows = [(ms, m.get("tau%d_gust" % ms)) for ms in (0, 20, 40, 60, 80)]
    if any(r[1] for r in gust_rows):
        ganch = {k: v for k, v in ref.get("lat_pos_anchors_gust", {}).items()
                 if not k.startswith("_")}
        print()
        print("== 3a. 지연 x 정상풍 (돌풍표) ==")
        print("  정상풍은 펄스와 달리 적분기를 계속 민다. 08-23 이 무외란표와 따로 둔 표가 이것.")
        print("  %5s %9s %9s %10s %10s %8s  %s"
              % ("tau", "돌풍s", "무외란s", "추종RMS", "경사max", "rho", "판정"))
        print("  %5s %9s %9s %10s %10s %8s" % ("[ms]", "(Simu)", "(Simu)", "[cm]", "[deg]", ""))
        for ms, r in gust_rows:
            if not r:
                continue
            s_g = interp_anchor(ganch, ms / 1000.0) if ganch else float("nan")
            s_c = interp_anchor({k: v for k, v in ref["lat_pos_anchors"].items()
                                 if not k.startswith("_")}, ms / 1000.0)
            trk = r.get("track_rms_cm")
            if r.get("diverged"):
                verdict = "발산"
            elif s_g <= 0.05:
                verdict = "표 보수 — 못 난다는데 버팀" if trk is not None and trk < 20 else "정합"
            elif s_g >= 0.9 and trk is not None and trk > 15:
                verdict = "표 낙관 — 안 깎아도 된다는데 추종 %.0f cm" % trk
            else:
                verdict = "정합"
            print("  %5d %9.2f %9.2f %10s %10.2f %8s  %s"
                  % (ms, s_g, s_c,
                     "-" if trk is None else "%.2f" % trk,
                     r.get("tilt_max_deg", float("nan")),
                     fmt_rho(r), verdict))
        print()
        print("  돌풍표가 무외란표보다 가파른 것이 Gazebo 에서도 재현되는지가 요점이다:")
        print("  같은 tau 라도 바람이 있으면 더 나빠져야 표의 분리가 정당화된다.")

    print()
    print("== 3b. 자세 지연 관문 ==")
    gate = ref["lat_att_gate"]
    worst_ok = None
    first_bad = None
    for ms in (3, 8, 12, 16, 20):
        r = m.get("att%d" % ms)
        if not r:
            continue
        bad = r.get("diverged") or r.get("tilt_max_deg", 0) > 30
        print("  att %3d ms: 자세RMS %6.3f deg / 경사max %6.2f deg%s"
              % (ms, r["hover_att_rms_deg"], r["tilt_max_deg"], "   <- 무너짐" if bad else ""))
        if bad and first_bad is None:
            first_bad = ms
        if not bad:
            worst_ok = ms
    if first_bad is not None:
        print("  Gazebo 무너짐 시작 %d ms  vs  표의 관문 %.0f ms"
              % (first_bad, gate["max_s"] * 1000))
    elif worst_ok is not None:
        print("  %d ms 까지 전부 버팀 — 표의 관문 %.0f ms 보다 관대하다 (표가 보수적)"
              % (worst_ok, gate["max_s"] * 1000))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Gazebo vs Simulink")
    ap.add_argument("--metrics", default=DEFAULT_METRICS)
    ap.add_argument("--ref", default=DEFAULT_REF)
    args = ap.parse_args(argv)

    if not os.path.isfile(args.metrics):
        print("지표 파일이 없다: %s" % args.metrics)
        print("  python3 analyze/gz_metrics.py out/*.csv --json out/metrics.json")
        return 1
    with open(args.metrics, encoding="utf-8") as fh:
        metrics = by_case(json.load(fh))
    with open(args.ref, encoding="utf-8") as fh:
        ref = json.load(fh)

    print("Gazebo 케이스 %d개 / 기준표 %s" % (len(metrics), os.path.basename(args.ref)))
    show_probe(metrics)
    show_base(metrics, ref)
    show_delay(metrics, ref)
    print()
    print("== 결론 기록 ==")
    print("  이 출력을 그대로 GAZEBO_STATUS.md 에 붙이고, 어긋난 항목마다")
    print("  '어느 쪽을 믿을 것인가'를 한 줄로 남길 것. 두 플랜트가 다르면")
    print("  기본값은 Simscape(정답 플랜트)이지만, 물성/기하가 어긋난 것이")
    print("  원인이면 Gazebo 쪽을 고치는 게 맞다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
