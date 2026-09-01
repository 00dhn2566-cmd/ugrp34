# -*- coding: utf-8 -*-
"""대표 지표 = 최악 조건 성능 그림 (사용자 결정 08-18: "내세워야 하는 metric을 그래프로").

입력: diagnose/results/perf_*.csv (배터리 최신 CSV — perf_battery_plots 로 재계산), figure/01|02 summary_missions*.csv,
      diagnose/results/tune_2kg_r2.csv (2 kg 새그), traj_emergency (정지 시간), tune_0kg_r5_nl.csv (있으면 비선형 게인 후보)
출력: figure/09_headline/
  fig_headline_worst.png     — 지표별 패널: 조건별 막대, 최악값 강조(빨강), 스펙 선
  fig_headline_ratio.png     — 한 장 요약: 최악값 / 스펙 (정규화, ≤1 = 통과), 조건 라벨
  headline_worst.csv|md      — 표 (SPEC §0.6 갱신 원자료)
사용: python make_headline_figure.py [--out figure/09_headline]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import perf_battery_plots as pbp                      # noqa: E402
from perf_metrics import C, RESULTS_DIR, _style       # noqa: E402

FIG = os.path.join(HERE, "figure")


def _missions(tag):
    p = os.path.join(FIG, "01_missions_1kg", "summary_missions.csv") if tag == "1kg" else \
        os.path.join(FIG, "02_missions_0kg", "summary_missions_0kg.csv")
    if not os.path.exists(p):
        return []
    with open(p, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        d = {"mission": r["mission"]}
        for k, v in r.items():
            if k == "mission":
                continue
            try:
                d[k] = float(v)
            except ValueError:
                d[k] = v
        out.append(d)
    return out


def _sag_2kg():
    p = os.path.join(RESULTS_DIR, "tune_2kg_r2.csv")
    if not os.path.exists(p):
        return None
    with open(p, newline="") as f:
        for r in csv.DictReader(f):
            if r["axis"] == "BASE":
                return float(r["sag_cm"])
    return None


def _r5_best():
    p = os.path.join(RESULTS_DIR, "tune_0kg_r5_nl.csv")
    if not os.path.exists(p):
        return None
    with open(p, newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["dist_peak_deg"] not in ("NaN", "")]
    cand = [r for r in rows if float(r["gmax"]) > 1.0 and float(r["dist_xy_m"]) < 1.0 and float(r["dist_peak_deg"]) < 20]
    if not cand:
        return None
    fin = lambda x: (x not in ("NaN", "")) and np.isfinite(float(x))
    r = min(cand, key=lambda r: float(r["dist_peak_deg"]) + float(r["dist_xy_m"]) + (0.0 if fin(r["dist_recover_s"]) else 5.0))   # 복귀되는 점 우선
    return {"gmax": float(r["gmax"]), "e0": float(r["e0_deg"]), "peak": float(r["dist_peak_deg"]),
            "xy_m": float(r["dist_xy_m"]), "rec": float(r["dist_recover_s"]) if r["dist_recover_s"] not in ("NaN", "") else float("nan")}


def _stop_times():
    import traj_emergency as te
    out = []
    for v0 in (0.5, 1.0, 1.6, 2.0):
        r = te.build_emergency_stop({"pos": [0, 0, 1.0], "vel": [v0, 0, 0], "acc": [0, 0, 0], "jerk": [0, 0, 0]})
        e = r["emergency"] if isinstance(r, dict) else [x for x in r if isinstance(x, dict)][0]["emergency"]
        out.append((v0, e["stop_T_s"], e["stop_dist_m"]))
    return out


def collect():
    """지표 → [(조건 라벨, 값)] 와 스펙. 배터리는 최신 CSV 로 재계산 (그림은 임시 폴더로 버림)."""
    tmp = tempfile.mkdtemp(prefix="headline_")
    B = {}
    for tag in ("0kg", "1kg"):
        B[f"hover_{tag}"] = pbp.hover(tmp, tag)
        B[f"pulse_{tag}"] = pbp.pulse(tmp, tag)
        B[f"alt_{tag}"] = pbp.alt_step(tmp, tag)
        B[f"yaw_{tag}"] = pbp.yaw_step(tmp, tag)
        B[f"diag_{tag}"] = pbp.diag(tmp, tag)
        B[f"wind_{tag}"] = pbp.wind(tmp, tag)
    mass = pbp.mass_sweep(tmp) or []
    M1, M0 = _missions("1kg"), _missions("0kg")
    sag2 = _sag_2kg()
    r5 = _r5_best()
    stops = _stop_times()

    P = {}   # key -> dict(title, unit, spec, items=[(label, value, group)], lower_better=True, log=False)

    def add(key, title, unit, spec, items, log=False, note=""):
        norm = []
        for it in items:
            l, v, gname = it[0], it[1], it[2]
            sp = it[3] if len(it) > 3 else spec          # 항목별 적용 스펙 (0 kg 부록 등); None = 참고값(비교 제외)
            if v is None or not np.isfinite(v):
                continue
            norm.append((l, float(v), gname, sp))
        P[key] = {"title": title, "unit": unit, "spec": spec, "items": norm, "log": log, "note": note}

    g = lambda d, *ks: (None if d is None else _dig(d, ks))
    add("hover", "호버 자세 지터 RMS", "°", 0.25, [
        ("1 kg 무풍", g(B["hover_1kg"], "att_rms_deg"), "1kg"),
        ("0 kg 무풍", g(B["hover_0kg"], "att_rms_deg"), "0kg"),
        ("1 kg 바람 5 m/s", g(B["wind_1kg"], "바람 5 m/s", "att_rms_deg"), "wind", None),
        ("0 kg 바람 5 m/s", g(B["wind_0kg"], "바람 5 m/s", "att_rms_deg"), "wind", None)], log=True,
        note="R4 ≤0.25° 는 무풍 조건 (바람 중 자세 = 트림+지터, 스펙 없음 → 참고)")
    tr = [(f"미션 {m['mission']} 1 kg", m["track_rms_3d_cm"], "1kg") for m in M1] + \
         [(f"미션 {m['mission']} 0 kg", m["track_rms_3d_cm"], "0kg") for m in M0] + \
         [("대각 1.6 m/s 1 kg", g(B["diag_1kg"], "track_rms_3d_cm"), "1kg"), ("대각 1.6 m/s 0 kg", g(B["diag_0kg"], "track_rms_3d_cm"), "0kg", 15.0)] + \
         [(f"1 m 이동 {m['m_pkg']:g} kg", m["track_rms_cm"], "mass") for m in mass]
    add("track", "추종 RMS 3D", "cm", 10.0, tr, note="0 kg 대각은 부록 P3 ≤15 cm")
    ov = [(f"미션 {m['mission']} 1 kg", m["overshoot_cm"], "1kg") for m in M1] + \
         [(f"미션 {m['mission']} 0 kg", m["overshoot_cm"], "0kg", 15.0) for m in M0] + \
         [(f"1 m 이동 {m['m_pkg']:g} kg", m["overshoot_cm"], "mass", 15.0 if m["m_pkg"] < 0.05 else 10.0) for m in mass]
    add("overshoot", "오버슈트", "cm", 10.0, ov, note="0 kg 은 부록 P2 ≤15 cm")
    add("sag", "이륙 새그", "cm", 5.0, [
        ("0 kg", g(B["hover_0kg"], "z_sag_cm"), "0kg"), ("1 kg", g(B["hover_1kg"], "z_sag_cm"), "1kg"),
        ("2 kg", sag2, "mass")], note="2 kg 는 모터 스핀업 과도 (게인 무관)")
    tail = [(f"미션 {m['mission']} 1 kg", m["att_tail_rms_deg"], "1kg") for m in M1] + \
           [(f"미션 {m['mission']} 0 kg", m["att_tail_rms_deg"], "0kg") for m in M0] + \
           [(f"1 m 이동 {m['m_pkg']:g} kg", m["tail_att_rms_deg"], "mass") for m in mass]
    add("tail", "도착 후 잔류 자세 RMS", "°", 0.25, tail, log=True)
    pulse = [("1 kg precision", g(B["pulse_1kg"], "precision", "peak_dev_deg"), "1kg"),
             ("1 kg agile", g(B["pulse_1kg"], "agile", "peak_dev_deg"), "1kg"),
             ("0 kg precision (비선형 자세 게인 배포)", g(B["pulse_0kg"], "precision", "peak_dev_deg"), "0kg", 20.0),
             ("0 kg agile", g(B["pulse_0kg"], "agile", "peak_dev_deg"), "0kg", 20.0)]
    add("pulse", "외란 펄스 0.3 N·m 최대 이탈", "°", 5.0, pulse, note="0 kg 목표(사용자): 이탈 ≤20°·밀림 ≤1 m, 복귀 보장 없음")
    push = [("1 kg precision", (g(B["pulse_1kg"], "precision", "xy_excursion_cm") or 0) / 100, "1kg"),
            ("0 kg (비선형 자세 게인 배포)", (g(B["pulse_0kg"], "precision", "xy_excursion_cm") or 0) / 100, "0kg")]
    add("push", "외란 후 수평 밀림", "m", 1.0, push, log=True)
    add("wind", "정상풍 5 m/s 위치 유지", "cm", 5.0, [
        ("1 kg", g(B["wind_1kg"], "바람 5 m/s", "xy_hold_cm"), "1kg"), ("0 kg", g(B["wind_0kg"], "바람 5 m/s", "xy_hold_cm"), "0kg", 25.0)], note="0 kg 은 부록 W1 ≤25 cm")
    add("time", "응답 시간", "s", None, [
        ("고도 1 m rise 1 kg", g(B["alt_1kg"], "rise_s"), "1kg"), ("고도 1 m rise 0 kg", g(B["alt_0kg"], "rise_s"), "0kg"),
        ("yaw 90° rise 1 kg", g(B["yaw_1kg"], "rise_s"), "1kg"), ("yaw 90° rise 0 kg", g(B["yaw_0kg"], "rise_s"), "0kg"),
        ("외란 복귀 1 kg", g(B["pulse_1kg"], "precision", "t_recover_s"), "1kg"),
        ("정착 ±5 cm 1 kg", 2.2, "1kg"), ("정착 ±5 cm 0 kg", 2.4, "0kg"),
        ("plan 벽시계", 0.7, "plan"), ("비상 서브프로세스", 0.75, "plan")] +
        [(f"정지 v0 {v:g} m/s", T, "stop") for (v, T, _) in stops],
        note="스펙: yaw ≤3 s(Y6) · 정착 ≤3 s · 정지 ≤1.5 s @1.6 m/s · plan ≤1 s")
    add("stopdist", "비상 정지 거리", "m", 1.0, [(f"v0 {v:g} m/s", D, "stop", (1.0 if v <= 1.6 else None)) for (v, _, D) in stops], note="스펙 ≤1 m @≤1.6 m/s (2.0 은 게이트 밖 참고)")
    return P


def _dig(d, ks):
    for k in ks:
        if d is None:
            return None
        d = d.get(k) if isinstance(d, dict) else None
    return d


GROUP_C = {"1kg": C["blue"], "0kg": C["violet"], "wind": C["aqua"], "mass": C["green"], "plan": C["yellow"], "stop": C["orange"]}


def fig_worst(P, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style()
    keys = ["hover", "track", "overshoot", "tail", "pulse", "push", "wind", "sag", "time", "stopdist"]
    fig, axs = plt.subplots(5, 2, figsize=(15, 19))
    for ax, k in zip(axs.ravel(), keys):
        p = P[k]
        items = p["items"]
        if not items:
            ax.set_visible(False)
            continue
        vals = np.array([v for _, v, _, _ in items])
        ratios = np.array([(v / sp) if sp else np.nan for _, v, _, sp in items])
        worst_i = int(np.nanargmax(ratios)) if np.isfinite(ratios).any() else int(np.nanargmax(vals))
        cols = [GROUP_C.get(gname, C["muted"]) for _, _, gname, _ in items]
        cols[worst_i] = C["red"]
        x = np.arange(len(items))
        ax.bar(x, vals, color=cols, width=0.7)
        ax.set_xticks(x)
        ax.set_xticklabels([l for l, _, _, _ in items], rotation=60, ha="right", fontsize=7.5)
        if p["log"]:
            ax.set_yscale("log")
        # 항목별 적용 스펙 = 짧은 빨간 선 (조건마다 스펙이 다르면 다르게 그려짐; None = 참고 항목)
        for xi, (_, _, _, sp) in zip(x, items):
            if sp is not None:
                ax.hlines(sp, xi - 0.42, xi + 0.42, color=C["red"], lw=1.4, ls="--")
        if p["spec"] is not None:
            ax.text(-0.4, p["spec"], f"스펙 {p['spec']:g} {p['unit']} (빨간 점선; 조건별 상이)", color=C["red"], fontsize=7.5, va="bottom", ha="left")
        wl, wv, _, wsp = items[worst_i]
        rtxt = f" = 스펙×{wv / wsp:.2f}" if wsp else " (참고)"
        ax.annotate(f"최악 {wv:.3g} {p['unit']}{rtxt}\n({wl})", xy=(worst_i, wv), xytext=(0, 14), textcoords="offset points",
                    ha="center", fontsize=8.5, color=C["red"], weight="bold",
                    arrowprops=dict(arrowstyle="-", color=C["red"], lw=0.8))
        ax.set_title(f"{p['title']} [{p['unit']}]" + (f"  —  {p['note']}" if p["note"] else ""), loc="left", fontsize=9.5)
        ax.set_ylabel(p["unit"])
        ax.margins(y=0.35)
    fig.suptitle("컨트롤러 대표 지표 = 측정한 모든 조건 중 최악값 (빨간 막대) · 빨간 점선 = 스펙 · 색 = 조건군 (파랑 1 kg / 보라 0 kg / 청록 바람 / 초록 질량 스윕 / 주황 정지 / 노랑 계획)",
                 x=0.01, ha="left", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(os.path.join(out, "fig_headline_worst.png"), dpi=150)
    plt.close(fig)


def fig_ratio(P, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style()
    rows = []
    for k, p in P.items():
        sc = [(l, v, sp) for (l, v, _, sp) in p["items"] if sp]
        if not sc:
            continue
        l, v, sp = max(sc, key=lambda t: t[1] / t[2])
        rows.append((p["title"], v, p["unit"], sp, l, v / sp))
    # 시간 항목은 개별 스펙으로 분해
    T = dict((l, v) for l, v, _, _ in P["time"]["items"])
    for lab, spec in (("yaw 90° rise 0 kg", 3.0), ("정착 ±5 cm 0 kg", 3.0), ("정지 v0 1.6 m/s", 1.5), ("plan 벽시계", 1.0)):
        if lab in T:
            rows.append((lab.replace(" 0 kg", " (0 kg 최악)"), T[lab], "s", spec, lab, T[lab] / spec))
    rows.sort(key=lambda r: r[5], reverse=True)
    fig, ax = plt.subplots(figsize=(11, 0.45 * len(rows) + 2))
    y = np.arange(len(rows))
    cols = [C["red"] if r[5] > 1 else C["blue"] for r in rows]
    ax.barh(y, [r[5] for r in rows], color=cols, height=0.62)
    ax.axvline(1.0, color=C["ink"], lw=1.2)
    for i, r in enumerate(rows):
        ax.text(r[5] + 0.03, i, f"{r[1]:.3g} {r[2]}  (적용 스펙 {r[3]:g}) — {r[4]}", va="center", fontsize=8.5, color=C["ink2"])
    ax.set_yticks(y)
    ax.set_yticklabels([r[0] for r in rows], fontsize=9)
    ax.invert_yaxis()
    ax.set_xlabel("최악값 / 스펙  (1.0 = 스펙 경계, 왼쪽 = 통과)")
    ax.set_xlim(0, max(1.4, max(r[5] for r in rows) * 1.35))
    ax.set_title("대표 지표 한 장 — 모든 측정 조건 중 (값 ÷ 그 조건의 적용 스펙) 최악 (파랑 통과 / 빨강 미달; 0 kg 은 부록 스펙)", loc="left")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_headline_ratio.png"), dpi=150)
    plt.close(fig)
    return rows


def write_tables(P, out):
    lines = ["# 대표 지표 = 최악 조건 성능 (자동 생성: make_headline_figure.py)", "",
             "| 지표 | 최악값 | 조건 | 스펙 | 판정 | 전 조건 (값) |", "|---|---|---|---|---|---|"]
    with open(os.path.join(out, "headline_worst.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["metric", "unit", "worst_value", "worst_condition", "spec", "pass", "n_conditions"])
        for k, p in P.items():
            if not p["items"]:
                continue
            sc = [(l, v, sp) for (l, v, _, sp) in p["items"] if sp]
            if sc:
                l, v, sp = max(sc, key=lambda t: t[1] / t[2])
            else:
                l, v, _, sp = max(p["items"], key=lambda t: t[1])
            ok = "—" if sp is None else ("✅" if v <= sp else "❌")
            w.writerow([p["title"], p["unit"], f"{v:.4g}", l, sp if sp is not None else "", ok, len(p["items"])])
            allv = "; ".join(f"{ll} {vv:.3g}" for ll, vv, _, _ in p["items"])
            lines.append(f"| {p['title']} | **{v:.3g} {p['unit']}** | {l} | {sp if sp is not None else '—'} | {ok} | {allv} |")
    with open(os.path.join(out, "headline_worst.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(FIG, "09_headline"))
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    P = collect()
    fig_worst(P, a.out)
    rows = fig_ratio(P, a.out)
    write_tables(P, a.out)
    with open(os.path.join(a.out, "headline_data.json"), "w", encoding="utf-8") as f:
        json.dump(P, f, ensure_ascii=False, indent=1)
    for r in rows:
        print(f"{r[0]:<22s} {r[1]:8.3g} {r[2]:<3s} spec {r[3]:<5g} ratio {r[5]:.2f}  ({r[4]})")
    print("→", a.out)


if __name__ == "__main__":
    main()
