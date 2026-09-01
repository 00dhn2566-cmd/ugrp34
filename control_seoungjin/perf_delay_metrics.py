# -*- coding: utf-8 -*-
"""측정 지연 주입 배터리(perf_battery.m DLY_ATT_MS/DLY_POS_MS) → 지연 여유(delay margin) 표 + 그림. 스펙 T3/R14.

입력: diagnose/results/perf_dlyA<att>_P<pos>_{hover,torque_pulse,move1m_1.0kg|move1m_0.0kg}_<tag>.csv
출력: figure/08_delay/summary_delay.csv|md, fig_delay_margin_<tag>.png (지연 vs 지표), fig_delay_hover_ts_<tag>.png (호버 자세 시계열)
판정: 스펙 R4(호버 지터 ≤0.25°)/R6(외란 이탈 ≤5°)/R7(복귀 ≤1.5 s)/§3.5(추종 ≤10 cm, 오버슈트 ≤10 cm) 유지 여부 → 지연 여유 = 마지막 통과 지연.
사용: python perf_delay_metrics.py [--tag 1kg,0kg] [--out figure/08_delay]
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from perf_metrics import RESULTS_DIR, _rms, C, AXIS_C  # noqa: E402

SPEC = {"hover_att_rms_deg": 0.25, "hover_att_peak_deg": 0.8, "pulse_peak_deg": 5.0, "pulse_recover_s": 1.5,
        "track_rms_cm": 10.0, "overshoot_cm": 10.0, "drift_cm": 5.0}
# 0 kg 는 외란 복귀 스펙이 없다 (부록: 이탈 크기 한정만) — 판정에서 제외
SPEC_0KG_SKIP = {"pulse_peak_deg", "pulse_recover_s"}


def _read(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()}


def hover_metrics(d):
    w = (d["t"] >= 2.0)
    r = d["roll_deg"][w] - d["roll_deg"][w].mean()
    p = d["pitch_deg"][w] - d["pitch_deg"][w].mean()
    return {"hover_att_rms_deg": _rms(np.concatenate([r, p])), "hover_att_peak_deg": float(np.max(np.hypot(r, p))),
            "sag_cm": float((1.0 - d["z"][d["t"] < 2.0].min()) * 100),
            "drift_cm": float(np.max(np.hypot(d["x"][w] - d["x"][w].mean(), d["y"][w] - d["y"][w].mean())) * 100)}


def pulse_metrics(d):
    pre = (d["t"] >= 2.0) & (d["t"] < 4.0)
    post = d["t"] >= 4.0
    r0, p0 = d["roll_deg"][pre].mean(), d["pitch_deg"][pre].mean()
    dev = np.hypot(d["roll_deg"] - r0, d["pitch_deg"] - p0)
    peak = float(dev[post].max())
    ok = dev < 1.0
    t_rec = float("nan")
    for i in range(int(np.argmax(d["t"] >= 4.3)), len(ok)):
        if ok[i:].all():
            t_rec = float(d["t"][i] - 4.0)
            break
    return {"pulse_peak_deg": peak, "pulse_recover_s": t_rec,
            "pulse_xy_m": float(np.max(np.hypot(d["x"][post] - d["x"][pre].mean(), d["y"][post] - d["y"][pre].mean())))}


def move_metrics(d):
    t = d["t"]
    seg = (t >= 3.0) & (t < 7.0)
    err = np.hypot(np.hypot(d["x"] - d["x_ref"], d["y"] - d["y_ref"]), d["z"] - d["z_ref"])
    tail = t >= t[-1] - 6.0
    return {"track_rms_cm": _rms(err[seg]) * 100, "overshoot_cm": float(max(0.0, d["x"].max() - 1.0) * 100),
            "att_peak_deg": float(max(np.abs(d["roll_deg"]).max(), np.abs(d["pitch_deg"]).max())),
            "tail_att_rms_deg": _rms(np.concatenate([d["roll_deg"][tail] - d["roll_deg"][tail].mean(),
                                                     d["pitch_deg"][tail] - d["pitch_deg"][tail].mean()]))}


def collect(tag):
    """tag('1kg'|'0kg') 의 (att_ms, pos_ms) 별 지표 dict 목록."""
    mv = "move1m_1.0kg" if tag == "1kg" else "move1m_0.0kg"
    pat = re.compile(r"perf_dlyA(?P<a>[\d.]+)_P(?P<p>[\d.]+)_hover_" + tag + r"\.csv$")
    rows = []
    for path in sorted(glob.glob(os.path.join(RESULTS_DIR, f"perf_dlyA*_P*_hover_{tag}.csv"))):
        m = pat.search(os.path.basename(path))
        if not m:
            continue
        a, p = float(m["a"]), float(m["p"])
        pre = os.path.join(RESULTS_DIR, f"perf_dlyA{m['a']}_P{m['p']}_")
        row = {"tag": tag, "att_ms": a, "pos_ms": p}
        row.update(hover_metrics(_read(path)))
        f2 = pre + f"torque_pulse_{tag}.csv"
        if os.path.exists(f2):
            row.update(pulse_metrics(_read(f2)))
        f3 = pre + f"{mv}.csv"
        if os.path.exists(f3):
            row.update(move_metrics(_read(f3)))
        row["pass"] = judge(row, tag)
        rows.append(row)
    rows.sort(key=lambda r: (r["pos_ms"] > 0 and r["att_ms"] == 0, r["att_ms"] + r["pos_ms"], r["att_ms"], r["pos_ms"]))
    return rows


def judge(row, tag):
    fails = []
    for k, lim in SPEC.items():
        if tag == "0kg" and k in SPEC_0KG_SKIP:
            continue
        v = row.get(k)
        if v is None:
            continue
        if not np.isfinite(v) or v > lim:
            fails.append(k)
    return "OK" if not fails else "FAIL:" + ",".join(fails)


def margins(rows):
    """자세 지연 축(pos=0)·위치 지연 축(att=0)에서 마지막으로 통과한 지연 [ms] (= 실측 지연 여유)."""
    def last_ok(sel):
        okv = [r["att_ms"] + r["pos_ms"] for r in sel if r["pass"] == "OK"]
        allv = [r["att_ms"] + r["pos_ms"] for r in sel]
        first_fail = min([v for v in allv if v not in okv], default=None)
        return (max(okv) if okv else None), first_fail
    att_axis = [r for r in rows if r["pos_ms"] == 0]
    pos_axis = [r for r in rows if r["att_ms"] == 0]
    return {"att_last_ok_ms": last_ok(att_axis), "pos_last_ok_ms": last_ok(pos_axis)}


def fig_margin(rows, tag, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from perf_metrics import _style
    _style()
    att = sorted([r for r in rows if r["pos_ms"] == 0], key=lambda r: r["att_ms"])
    pos = sorted([r for r in rows if r["att_ms"] == 0], key=lambda r: r["pos_ms"])
    combo = [r for r in rows if r["att_ms"] > 0 and r["pos_ms"] > 0]
    fig, axs = plt.subplots(2, 3, figsize=(13, 7))
    panels = [("hover_att_rms_deg", "호버 자세 지터 RMS [°]", SPEC["hover_att_rms_deg"]),
              ("pulse_peak_deg", "외란 펄스 최대 이탈 [°]", SPEC["pulse_peak_deg"] if tag == "1kg" else None),
              ("pulse_recover_s", "외란 복귀 시간 [s]", SPEC["pulse_recover_s"] if tag == "1kg" else None),
              ("track_rms_cm", "1 m 이동 추종 RMS [cm]", SPEC["track_rms_cm"]),
              ("overshoot_cm", "1 m 이동 오버슈트 [cm]", SPEC["overshoot_cm"]),
              ("tail_att_rms_deg", "이동 후 잔류 자세 RMS [°]", 0.25)]
    for ax, (k, lab, lim) in zip(axs.ravel(), panels):
        if att:
            ax.plot([r["att_ms"] for r in att], [r.get(k, np.nan) for r in att], "o-", color=C["blue"], label="자세 측정 지연 (위치 0)")
        if pos:
            ax.plot([r["pos_ms"] for r in pos], [r.get(k, np.nan) for r in pos], "s-", color=C["orange"], label="위치 측정 지연 (자세 0)")
        for r in combo:
            ax.plot([r["att_ms"] + r["pos_ms"]], [r.get(k, np.nan)], "D", color=C["magenta"], ms=8,
                    label=f"조합 자세 {r['att_ms']:g}+위치 {r['pos_ms']:g} ms" )
        if lim is not None:
            ax.axhline(lim, color=C["red"], lw=0.9, ls="--", label=f"스펙 {lim:g}")
        ax.set_title(lab, loc="left")
        ax.set_xlabel("주입 지연 [ms]")
        ax.grid(True, alpha=0.4)
    axs[0, 0].legend(fontsize=8, loc="upper left")
    fig.suptitle(f"측정 지연 주입 배터리 ({tag}) — 센서·통신 지연 내성 (스펙 T3/R14): 빨간 점선 = 스펙 한계", x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_delay_margin_{tag}.png"), dpi=150)
    plt.close(fig)


def fig_hover_ts(rows, tag, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from perf_metrics import _style
    _style()
    att = sorted([r for r in rows if r["pos_ms"] == 0], key=lambda r: r["att_ms"])
    if not att:
        return
    fig, ax = plt.subplots(figsize=(10, 4))
    cols = [C["blue"], C["aqua"], C["violet"], C["orange"], C["red"], C["magenta"]]
    for r, col in zip(att, cols * 3):
        d = _read(os.path.join(RESULTS_DIR, f"perf_dlyA{r['att_ms']:g}_P{r['pos_ms']:g}_hover_{tag}.csv"))
        w = d["t"] >= 2.0
        ax.plot(d["t"][w], d["pitch_deg"][w] - d["pitch_deg"][w].mean(), color=col, lw=0.9,
                label=f"자세 지연 {r['att_ms']:g} ms (RMS {r['hover_att_rms_deg']:.3f}°)")
    ax.set_xlabel("시간 [s]")
    ax.set_ylabel("pitch 편차 [°]")
    ax.set_title(f"호버 pitch 지터 vs 자세 측정 지연 ({tag})", loc="left")
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"fig_delay_hover_ts_{tag}.png"), dpi=150)
    plt.close(fig)


def write_tables(all_rows, out):
    keys = ["tag", "att_ms", "pos_ms", "hover_att_rms_deg", "hover_att_peak_deg", "sag_cm", "drift_cm",
            "pulse_peak_deg", "pulse_recover_s", "pulse_xy_m", "track_rms_cm", "overshoot_cm", "att_peak_deg", "tail_att_rms_deg", "pass"]
    with open(os.path.join(out, "summary_delay.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in all_rows:
            w.writerow(r)
    lines = ["# 측정 지연 주입 배터리 — 지연 여유 (스펙 T3/R14)", "",
             "지연은 측정 경로에 Transport Delay 로 주입 (qc_delay_apply: 자세/yaw = 자세 지연, 위치/z = 위치 지연). 판정 = R4/R6/R7/§3.5 유지 (0 kg 는 R6/R7 제외).", ""]
    for tag in ("1kg", "0kg"):
        rows = [r for r in all_rows if r["tag"] == tag]
        if not rows:
            continue
        mg = margins(rows)
        lines.append(f"## {tag} — 자세 지연 여유: 마지막 통과 {mg['att_last_ok_ms'][0]} ms / 첫 실패 {mg['att_last_ok_ms'][1]} ms · "
                     f"위치 지연 여유: 마지막 통과 {mg['pos_last_ok_ms'][0]} ms / 첫 실패 {mg['pos_last_ok_ms'][1]} ms")
        lines.append("")
        lines.append("| 자세 지연 [ms] | 위치 지연 [ms] | 호버 지터 RMS/피크 [°] | 새그 [cm] | 외란 피크 [°] / 복귀 [s] / 밀림 [m] | 추종 RMS / 오버슈트 [cm] | 자세 피크 [°] | 잔류 [°] | 판정 |")
        lines.append("|---|---|---|---|---|---|---|---|---|")
        for r in rows:
            g = lambda k, f="{:.2f}": (f.format(r[k]) if (k in r and np.isfinite(r[k])) else "—")
            lines.append(f"| {r['att_ms']:g} | {r['pos_ms']:g} | {g('hover_att_rms_deg','{:.3f}')} / {g('hover_att_peak_deg','{:.3f}')} | {g('sag_cm','{:.1f}')} | "
                         f"{g('pulse_peak_deg')} / {g('pulse_recover_s')} / {g('pulse_xy_m')} | {g('track_rms_cm')} / {g('overshoot_cm','{:.1f}')} | "
                         f"{g('att_peak_deg','{:.1f}')} | {g('tail_att_rms_deg','{:.3f}')} | {r['pass']} |")
        lines.append("")
    with open(os.path.join(out, "summary_delay.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="1kg,0kg")
    ap.add_argument("--out", default=os.path.join(HERE, "figure", "08_delay"))
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    all_rows = []
    for tag in a.tag.split(","):
        rows = collect(tag.strip())
        if not rows:
            print(f"[{tag}] 지연 배터리 CSV 없음 (perf_dlyA*_P*_hover_{tag}.csv)")
            continue
        fig_margin(rows, tag, a.out)
        fig_hover_ts(rows, tag, a.out)
        all_rows.extend(rows)
        mg = margins(rows)
        print(f"[{tag}] {len(rows)}점  자세 지연 여유 {mg['att_last_ok_ms']}  위치 지연 여유 {mg['pos_last_ok_ms']}")
    if all_rows:
        write_tables(all_rows, a.out)
        print("→", a.out)


if __name__ == "__main__":
    main()
