"""극한 상황 배터리 그림 — 지연 x 스펙배율 x 외란 + 사용 전력량.

2026-08-26. 입력: diagnose/results/worstcase/{summary.csv, ts_*.csv}
                  (MATLAB `diagnose/verify_worstcase.m` 이 만든다)
출력: figure/13_worstcase/*.png

    python make_worstcase_figure.py

실측이 없으면 그리지 않는다 — 숫자를 지어내지 않는다는 저장소 규칙 그대로다.
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = os.path.join("figure", "13_worstcase")
RES = os.path.join("controller", "Quadcopter-Drone-Model-Simscape",
                   "diagnose", "results", "worstcase")

for cand in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
    try:
        matplotlib.font_manager.findfont(cand, fallback_to_default=False)
        plt.rcParams["font.family"] = cand
        break
    except Exception:
        continue
plt.rcParams["axes.unicode_minus"] = False
# 한글 폰트에 U+2212 / 위첨자 글리프가 없다 — 수식 글꼴만 DejaVu 로 (11_delay 와 같은 처방)
plt.rcParams["mathtext.fontset"] = "dejavusans"
plt.rcParams["figure.dpi"] = 130
plt.rcParams["savefig.bbox"] = "tight"

C_BASE, C_BAD, C_GOOD, C_EXTRA, C_GRAY = "#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#888888"


def _save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p)
    plt.close(fig)
    print("  " + p)
    return p


def load_summary():
    p = os.path.join(RES, "summary.csv")
    if not os.path.isfile(p):
        return None
    # MATLAB 쪽이 desc 를 따옴표 없이 쓰는데 그 안에 쉼표가 있다 -> DictReader 로는
    # 필드가 밀린다. desc 는 두 번째 자유문자열 필드 하나뿐이므로, 남는 조각을 다시
    # 붙여서 복구한다. (verify_worstcase.m 은 이후 쉼표를 안 쓰도록 고쳤다.)
    with open(p, newline="", encoding="utf-8") as fh:
        raw = list(csv.reader(fh))
    head = raw[0]
    rows = []
    for parts in raw[1:]:
        if not parts:
            continue
        extra = len(parts) - len(head)
        if extra > 0:
            parts = [parts[0], ",".join(parts[1:2 + extra])] + parts[2 + extra:]
        rows.append(dict(zip(head, parts)))
    out = {}
    for r in rows:
        d = dict(r)
        for k in ("tau_pos_ms", "tau_att_ms", "s", "pulse_Nm", "v_peak", "end_cm",
                  "track_cm", "dev_y_cm", "recover_s", "t_pulse", "T_end", "wall_s",
                  "energy_est_Wh", "energy_act_Wh", "energy_ratio", "est_Wh_per_m",
                  "P_mean_W", "P_peak_W"):
            v = d.get(k, "")
            d[k] = float(v) if v not in ("", None) else float("nan")
        out[d["label"]] = d
    return out


def load_ts(label):
    p = os.path.join(RES, "ts_%s.csv" % label)
    if not os.path.isfile(p):
        return None
    with open(p, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def fig_timeseries(S):
    """핵심 그림: 같은 지연에서 스펙을 안 깎았을 때 vs 표대로 깎았을 때."""
    want = [("B", "지연 60ms · 스펙 안 깎음 (s=1.00)", C_BAD),
            ("C", "지연 60ms · 표대로 (s=0.75)", C_GOOD),
            ("A", "무지연 기준선 (s=1.00)", C_BASE)]
    have = [(l, t, c) for l, t, c in want if load_ts(l) is not None]
    if not have:
        return None
    fig, ax = plt.subplots(3, 1, figsize=(9.5, 8.4), sharex=True)

    for lab, ttl, col in have:
        d = load_ts(lab)
        row = S.get(lab, {}) if S else {}
        tp = row.get("t_pulse", float("nan"))
        ax[0].plot(d["t"], d["x"], color=col, lw=1.6, label=ttl)
        ax[0].plot(d["t"], d["xref"], color=col, lw=0.8, ls="--", alpha=0.45)
        ax[1].plot(d["t"], 100 * d["y"], color=col, lw=1.6)
        ax[2].plot(d["t"], d["P_est_W"], color=col, lw=1.4)
        if math.isfinite(tp):
            for a in ax:
                a.axvspan(tp, tp + 0.3, color=C_GRAY, alpha=0.18, lw=0)

    ax[0].set_ylabel("x [m]")
    ax[0].set_title("극한 조합: 위치 지연 60 ms + 자세 지연 12 ms + 외란 0.3 N·m × 0.3 s (이동 한복판)")
    ax[0].legend(loc="lower right", fontsize=8.5)
    ax[0].grid(alpha=0.3)
    ax[0].text(0.012, 0.93, "실선 = 실제, 파선 = 기준", transform=ax[0].transAxes,
               fontsize=8, color=C_GRAY, va="top")

    ax[1].axhline(2, color=C_GRAY, lw=0.8, ls=":")
    ax[1].axhline(-2, color=C_GRAY, lw=0.8, ls=":")
    ax[1].set_ylabel("외란 방향 y [cm]")
    ax[1].grid(alpha=0.3)
    ax[1].text(0.012, 0.92, "점선 = 복귀 판정 밴드 ±2 cm", transform=ax[1].transAxes,
               fontsize=8, color=C_GRAY, va="top")

    ax[2].set_ylabel("전력 추정 [W]")
    ax[2].set_xlabel("시간 [s]")
    ax[2].grid(alpha=0.3)
    ax[2].text(0.012, 0.92, "회색 띠 = 외란 인가 구간", transform=ax[2].transAxes,
               fontsize=8, color=C_GRAY, va="top")
    fig.tight_layout()
    return _save(fig, "fig_worst_timeseries.png")


def fig_recover(S):
    """지연·배율 조합별 복귀 시간과 추종 이탈 — 표대로 깎는 것이 사는 길인가."""
    order = ["A", "B", "C", "D", "E", "F", "G"]
    rows = [S[l] for l in order if l in S]
    if not rows:
        return None
    fig, ax = plt.subplots(1, 2, figsize=(11.5, 4.2))
    labs = [r["label"] for r in rows]
    xs = np.arange(len(rows))

    rec = [r["recover_s"] for r in rows]
    fail = [not math.isfinite(v) for v in rec]
    top = max([v for v in rec if math.isfinite(v)] + [1.0]) * 1.35
    plot_rec = [top if f else v for v, f in zip(rec, fail)]
    cols = [C_BAD if f else (C_GOOD if v <= 3.0 else C_EXTRA)
            for v, f in zip(rec, fail)]
    ax[0].bar(xs, plot_rec, color=cols)
    ax[0].axhline(3.0, color=C_GRAY, ls="--", lw=1.0)
    ax[0].text(len(rows) - 0.4, 3.05, "게이트 3 s", ha="right", fontsize=8, color=C_GRAY)
    for i, (v, f) in enumerate(zip(rec, fail)):
        ax[0].text(i, plot_rec[i] * 1.02, "복귀실패" if f else "%.2f" % v,
                   ha="center", fontsize=8)
    ax[0].set_xticks(xs); ax[0].set_xticklabels(labs)
    ax[0].set_ylabel("외란 복귀 [s]")
    ax[0].set_title("복귀 시간 (빨강 = 밴드로 못 돌아옴)")
    ax[0].grid(axis="y", alpha=0.3)

    trk = [r["track_cm"] for r in rows]
    ax[1].bar(xs, trk, color=[C_GOOD if v <= 10 else C_BAD for v in trk])
    ax[1].axhline(10.0, color=C_GRAY, ls="--", lw=1.0)
    ax[1].text(len(rows) - 0.4, 10.2, "게이트 10 cm", ha="right", fontsize=8, color=C_GRAY)
    for i, v in enumerate(trk):
        ax[1].text(i, v * 1.02, "%.1f" % v, ha="center", fontsize=8)
    ax[1].set_xticks(xs); ax[1].set_xticklabels(labs)
    ax[1].set_ylabel("최대 추종 이탈 [cm]")
    ax[1].set_title("기준 대비 추종")
    ax[1].grid(axis="y", alpha=0.3)

    note = " / ".join("%s: tau %g·%g ms, s=%.2f" %
                      (r["label"], r["tau_pos_ms"], r["tau_att_ms"], r["s"]) for r in rows)
    fig.suptitle(note, fontsize=8, color=C_GRAY, y=1.02)
    fig.tight_layout()
    return _save(fig, "fig_worst_gates.png")


def fig_energy(S):
    """사용 전력량: 추정치 vs 실측, 그리고 스펙을 깎으면 에너지는 어떻게 되나."""
    order = ["A", "B", "C", "D", "E", "F", "G", "H", "I"]
    rows = [S[l] for l in order if l in S]
    if not rows:
        return None
    fig, ax = plt.subplots(1, 2, figsize=(11.5, 4.2))
    labs = [r["label"] for r in rows]
    xs = np.arange(len(rows))
    w = 0.38

    est = [r["energy_est_Wh"] for r in rows]
    act = [r["energy_act_Wh"] for r in rows]
    ax[0].bar(xs - w / 2, est, w, color=C_BASE, label="추정치 (운동량 이론)")
    if any(math.isfinite(v) for v in act):
        ax[0].bar(xs + w / 2, [0 if not math.isfinite(v) else v for v in act], w,
                  color=C_EXTRA, label="실측 (Simscape 배터리)")
    ax[0].set_xticks(xs); ax[0].set_xticklabels(labs)
    ax[0].set_ylabel("에너지 [Wh]")
    ax[0].set_title("사용 전력량: 추정치 vs 실측")
    ax[0].legend(fontsize=8.5)
    ax[0].grid(axis="y", alpha=0.3)

    # 스펙 배율 대 에너지 — "느리게 가면 더 드는가 덜 드는가"
    pairs = [(r["s"], r["energy_est_Wh"], r["label"]) for r in rows
             if math.isfinite(r["s"])]
    ax[1].scatter([p[0] for p in pairs], [p[1] for p in pairs], s=48, color=C_BASE)
    for sv, ev, lb in pairs:
        ax[1].annotate(lb, (sv, ev), textcoords="offset points", xytext=(5, 4), fontsize=8)
    ax[1].set_xlabel("스펙 배율 s")
    ax[1].set_ylabel("추정 에너지 [Wh]")
    ax[1].set_title("스펙을 깎을수록 임무가 길어져 에너지는 늘어난다")
    ax[1].grid(alpha=0.3)
    ax[1].invert_xaxis()
    fig.tight_layout()
    return _save(fig, "fig_worst_energy.png")



def alt_limit_cycle(label, t_from=6.0):
    """정착 후 고도 한계사이클 (p-p 진폭 / 영교차 주파수).

    08-26 발견: 위치(VIO) 경로 지연이 ~2.5 Hz 고도 한계사이클을 만든다. 무지연
    0.15 cm 대 60 ms 4.72 cm. **스펙을 깎아도 안 잡힌다** (0.75 에서 오히려 5.76 cm) —
    감쇄 s 는 기준 궤적의 시간축을 늘리는 것이고 이 진동은 고도 폐루프의 성질이라
    지연이 그대로면 동특성도 그대로다. 기존 지연 표는 횡방향 복귀·추종으로만
    채점해서 이 축을 못 봤다.
    """
    d = load_ts(label)
    if d is None:
        return None
    m = d["t"] > t_from
    if m.sum() < 20:
        return None
    z = d["z"][m]
    tt = d["t"][m]
    zm = float(np.mean(z))
    dz = z - zm
    nz = int(np.sum(dz[1:] * dz[:-1] < 0))
    span = float(tt[-1] - tt[0])
    return {
        "z_mean_m": zm,
        "z_ripple_cm": float(100.0 * (z.max() - z.min())),
        "z_freq_hz": (nz / (2.0 * span)) if span > 0 else float("nan"),
        "p_min_w": float(d["P_est_W"][m].min()),
        "p_max_w": float(d["P_est_W"][m].max()),
        "p_mean_w": float(d["P_est_W"][m].mean()),
    }


def fig_altitude(S):
    """지연이 만드는 고도 한계사이클 — 감쇄로 안 잡히는 축."""
    order = ["A", "B", "C", "D", "E", "F", "G", "H", "I"]
    rows = []
    for l in order:
        if l not in S:
            continue
        lc = alt_limit_cycle(l)
        if lc:
            rows.append((l, S[l], lc))
    if len(rows) < 2:
        return None
    fig, ax = plt.subplots(1, 2, figsize=(11.5, 4.2))
    xs = np.arange(len(rows))
    labs = [r[0] for r in rows]

    rip = [r[2]["z_ripple_cm"] for r in rows]
    cols = [C_GOOD if v < 1.0 else (C_EXTRA if v < 3.0 else C_BAD) for v in rip]
    ax[0].bar(xs, rip, color=cols)
    for i, (v, r) in enumerate(zip(rip, rows)):
        ax[0].text(i, v * 1.02, "%.2f" % v, ha="center", fontsize=8)
    ax[0].set_xticks(xs)
    ax[0].set_xticklabels(["%s\n%g ms\ns=%.2f"
                           % (r[0], r[1]["tau_pos_ms"], r[1]["s"])
                           for r in rows], fontsize=7.5)
    ax[0].set_ylabel("정착 후 고도 진동 p-p [cm]")
    ax[0].set_title("위치 지연이 만드는 고도 한계사이클")
    ax[0].grid(axis="y", alpha=0.3)

    # 같은 지연에서 s 를 깎아도 안 줄어든다는 것을 직접 보인다
    g = [(r[1]["s"], r[2]["z_ripple_cm"], r[0]) for r in rows
         if abs(r[1]["tau_pos_ms"] - 60) < 1e-6]
    if len(g) >= 2:
        g.sort()
        ax[1].plot([p[0] for p in g], [p[1] for p in g], "o-", color=C_BAD, lw=1.8)
        for sv, rv, lb in g:
            ax[1].annotate(lb, (sv, rv), textcoords="offset points", xytext=(5, 5), fontsize=8)
        ax[1].set_xlabel("스펙 배율 s (지연 60 ms 고정)")
        ax[1].set_ylabel("고도 진동 p-p [cm]")
        ax[1].set_title("스펙을 깎아도 줄지 않는다\n(감쇄는 기준 궤적의 축, 진동은 고도 폐루프의 축)")
        ax[1].invert_xaxis()
        ax[1].grid(alpha=0.3)
    else:
        ax[1].axis("off")
    fig.tight_layout()
    return _save(fig, "fig_worst_altitude.png")



def write_readme(S):
    """figure/13_worstcase/README.md 를 실측에서 직접 뽑는다 (손으로 옮겨 적지 않게)."""
    lines = []
    A = lines.append
    A("# 13_worstcase — 극한 조합 실측 (2026-08-26)")
    A("")
    A("생성: `python make_worstcase_figure.py` (입력 `diagnose/verify_worstcase.m` 산출).")
    A("**이 README 는 자동 생성이다** — 숫자를 손으로 고치지 말 것.")
    A("")
    A("조건: 짐 1 kg, precision, 3 m 직선 이동, 외란 0.3 N·m x 0.3 s 를 이동 한복판에.")
    A("")
    A("## 케이스")
    A("")
    A("| | 시나리오 | tauP [ms] | tauA [ms] | s | 추종 [cm] | 외란y [cm] | 복귀 [s] | est [Wh] | act [Wh] |")
    A("|---|---|---|---|---|---|---|---|---|---|")
    for lab in ("A", "B", "C", "D", "E", "F", "G", "H", "I"):
        r = S.get(lab)
        if not r:
            continue
        rec = "-" if not math.isfinite(r["recover_s"]) else "%.2f" % r["recover_s"]
        act = "-" if not math.isfinite(r["energy_act_Wh"]) else "%.4f" % r["energy_act_Wh"]
        A("| %s | %s | %g | %g | %.2f | %.2f | %.2f | %s | %.4f | %s |"
          % (lab, r["desc"], r["tau_pos_ms"], r["tau_att_ms"], r["s"],
             r["track_cm"], r["dev_y_cm"], rec, r["energy_est_Wh"], act))
    A("")
    A("## 고도 한계사이클 (정착 후, t > 6 s)")
    A("")
    A("| | tauP [ms] | s | z 진동 p-p [cm] | 주파수 [Hz] | 전력 [W] |")
    A("|---|---|---|---|---|---|")
    for lab in ("A", "B", "C", "D", "E", "F", "G", "H", "I"):
        lc = alt_limit_cycle(lab)
        r = S.get(lab)
        if not lc or not r:
            continue
        A("| %s | %g | %.2f | %.2f | %.2f | %.0f ~ %.0f (평균 %.0f) |"
          % (lab, r["tau_pos_ms"], r["s"], lc["z_ripple_cm"], lc["z_freq_hz"],
             lc["p_min_w"], lc["p_max_w"], lc["p_mean_w"]))
    A("")
    A("## 읽는 법 / 주의")
    A("")
    A("- **복귀는 배율에 단조롭지 않다.** 08-23 에도 같았다 (tau=60 에서 s 1.00/0.75/0.55")
    A("  -> 6.17/12.19/11.06 s). 그래서 `sweep_delay_spec` 은 이분탐색을 금지하고")
    A("  내림차순 전수로 돈다. 여기 B/C/D 도 같은 모양이다.")
    A("- **표를 두 벌 쓴다.** `_LAT_POS_ANCHORS`(무외란) 60 ms = 0.75 이고,")
    A("  `_LAT_POS_ANCHORS_GUST`(외란 중) 60 ms = **0.28** 이다. 여기 C/D 라벨의")
    A("  '표대로'는 **무외란표** 값이라 외란 조건에서는 덜 깎은 것이다 — 표 자체의")
    A("  반증이 아니라 케이스 설계가 표를 잘못 고른 것. (돌풍표 0.28 확인은 미실행.)")
    A("- **고도 한계사이클은 감쇄로 안 잡힌다.** 감쇄 s 는 기준 궤적의 시간축을 늘리는")
    A("  것이고 이 진동은 고도 폐루프의 성질이라, 지연이 그대로면 동특성도 그대로다.")
    A("  자세 지연처럼 **관문**으로 다뤄야 할 후보다 (기존 표는 횡방향 복귀·추종으로만")
    A("  채점해서 이 축을 못 봤다).")
    A("- **act(Simscape 배터리)로 추정기를 교정하지 말 것.** 그 전기 모델은 SOC 표시용")
    A("  이상화라 25 g/W 라는 불가능한 효율을 낸다. `energy.TRUSTED_SOURCES` 참조.")
    A("")
    A("## 그림")
    A("")
    A("- `fig_worst_timeseries.png` — x 추종 / 외란 방향 y / 전력, 세 구성 겹쳐 그림")
    A("- `fig_worst_gates.png` — 복귀 시간과 추종 이탈 (게이트 대비)")
    A("- `fig_worst_energy.png` — 추정 vs 실측 에너지, 배율 대 에너지")
    A("- `fig_worst_altitude.png` — 고도 한계사이클 (지연별 / 배율별)")
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "README.md")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(chr(10).join(lines) + chr(10))
    print("  " + path)


def main():
    S = load_summary()
    if not S:
        print("실측이 없다: %s/summary.csv" % RES)
        print("  MATLAB 에서 `verify_worstcase` 를 먼저 돌릴 것.")
        return 1
    print("극한 상황 그림:")
    made = [f for f in (fig_timeseries(S), fig_recover(S), fig_energy(S),
                        fig_altitude(S)) if f]

    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as fh:
        lc = {k: alt_limit_cycle(k) for k in S}
        json.dump({"source": "diagnose/verify_worstcase.m", "cases": S,
                   "altitude_limit_cycle": {k: v for k, v in lc.items() if v}},
                  fh, ensure_ascii=False, indent=2)
    print("  %s/summary.json" % OUT)
    write_readme(S)
    print("그림 %d장" % len(made))
    return 0


if __name__ == "__main__":
    sys.exit(main())
