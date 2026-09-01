"""시간 지연 → 스펙 그림 생성.

2026-08-23. 출력: figure/11_delay/*.png

MATLAB 실측(diagnose/results/sweep_delay_*.mat)을 읽어 그리고, 없으면 이 파일에
박아둔 실측표로 그린다 (다른 머신에서도 그림이 재현되게 — 숫자 출처는 각 함수 주석).

사용:
    python make_delay_figures.py
"""
from __future__ import annotations

import json
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import capability as cap
from traj_bridge import BASE_LIMITS, _smoothstep7, plan_bridge
from latency_tracker import LatencyTracker

OUT = os.path.join("figure", "11_delay")
RES = os.path.join("controller", "Quadcopter-Drone-Model-Simscape", "diagnose", "results")

for cand in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
    try:
        matplotlib.font_manager.findfont(cand, fallback_to_default=False)
        plt.rcParams["font.family"] = cand
        break
    except Exception:
        continue
plt.rcParams["axes.unicode_minus"] = False
# 한글 폰트에는 U+2212(수학 마이너스)와 위첨자 글리프가 없다. 로그축 기본 포매터가
# 그걸 쓰기 때문에 눈금이 "10ㅁ1" 처럼 깨진다 -> 수식 글꼴만 DejaVu 로 돌리고,
# 로그 눈금은 아래에서 평문으로 직접 박는다.
plt.rcParams["mathtext.fontset"] = "dejavusans"
plt.rcParams["figure.dpi"] = 130
plt.rcParams["savefig.bbox"] = "tight"

C_OK, C_BAD, C_ACC, C_GRAY = "#1f77b4", "#d62728", "#ff7f0e", "#888888"

# A상 실측 (diagnose/sweep_delay_margin.m, 1 kg, 제자리 호버 + 0.3 N·m x 0.3 s)
ATT_MS = [0, 8, 12, 16, 20, 24]
ATT_RMS = [0.0209, 0.0043, 0.0040, 0.2106, 2.4367, 4.6137]
ATT_PK = [0.0247, 0.0104, 0.0100, 0.4172, 2.9513, 4.9069]
ATT_DIST = [2.32, 2.50, 2.60, 3.08, 4.59, 7.10]
ATT_GATE = 0.25          # 호버 RMS 판정선 [deg]


def _save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p)
    plt.close(fig)
    print(f"  {p}")
    return p


def _read_progress(which="gust"):
    """스윕 진행 파일 파싱 -> [{tau, s, end_cm, over_cm, devy_cm, rec_s, ok}].

    which: 'gust'(0.3 N*m 배터리) | 'nominal'(외란 없음, 기본표)
    """
    names = {"gust": ["sweep_delay_spec_progress_1kg_gust.txt"],
             "nominal": ["sweep_delay_spec_progress_1kg_nominal.txt",
                         "sweep_delay_spec_progress.txt"]}[which]
    for name in names:
        p = os.path.join(RES, name)
        if os.path.exists(p):
            break
    else:
        return []
    rows = []
    with open(p, encoding="utf-8", errors="replace") as f:
        for ln in f:
            m = re.match(r"\s*(\d+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+"
                         r"([\d.]+)\s+(\S+)\s+(OK|FAIL)", ln)
            if m:
                rec = float(m.group(7)) if m.group(7) != "NaN" else float("nan")
                rows.append(dict(tau=int(m.group(1)), s=float(m.group(2)),
                                 end_cm=float(m.group(4)), over_cm=float(m.group(5)),
                                 devy_cm=float(m.group(6)), rec_s=rec,
                                 ok=(m.group(8) == "OK")))
    return rows


def fig_att_gate():
    """자세 경로 지연 = 감쇄가 아니라 게이트. 왜 그런지가 한 장에 보여야 한다."""
    fig, ax = plt.subplots(1, 2, figsize=(9.4, 3.4))

    ax[0].semilogy(ATT_MS, ATT_RMS, "o-", color=C_OK, label="호버 RMS")
    ax[0].semilogy(ATT_MS, ATT_PK, "s--", color=C_GRAY, ms=4, label="호버 피크")
    ax[0].axhline(ATT_GATE, color=C_BAD, ls=":", lw=1.2)
    ax[0].text(0.3, ATT_GATE * 1.25, f"판정선 {ATT_GATE}°", color=C_BAD, fontsize=8)
    ax[0].axvspan(cap.LAT_ATT_MAX_S * 1000, 26, color=C_BAD, alpha=.10)
    ax[0].axvspan(cap.LAT_ATT_CLEAN_S * 1000, cap.LAT_ATT_MAX_S * 1000,
                  color=C_ACC, alpha=.12)
    ax[0].text(21.5, 0.012, "운용 불가\n(임무 거부)", color=C_BAD, fontsize=8, ha="center")
    ax[0].text(14, 0.012, "여유\n감쇄", color=C_ACC, fontsize=8, ha="center")
    ax[0].set_xlabel("자세 경로 지연 [ms]"); ax[0].set_ylabel("자세 이탈 [deg]")
    ax[0].set_title("제자리 호버 — 기동을 전혀 안 한 상태")
    ax[0].legend(fontsize=8, loc="lower right"); ax[0].grid(alpha=.3, which="both")
    ax[0].set_xlim(-1, 26); ax[0].set_ylim(2e-3, 12)
    ticks = [0.003, 0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0]
    ax[0].set_yticks(ticks)
    ax[0].set_yticklabels([("%g" % t) for t in ticks])
    ax[0].minorticks_off()

    ax[1].plot(ATT_MS, ATT_DIST, "o-", color=C_ACC)
    ax[1].set_xlabel("자세 경로 지연 [ms]"); ax[1].set_ylabel("외란 최대 이탈 [deg]")
    ax[1].set_title("0.3 N·m × 0.3 s 펄스 응답")
    ax[1].grid(alpha=.3); ax[1].set_xlim(-1, 26)
    for x, y in zip(ATT_MS, ATT_DIST):
        ax[1].annotate(f"{y:.1f}", (x, y), textcoords="offset points",
                       xytext=(0, 6), ha="center", fontsize=7)

    fig.suptitle("자세 경로 지연은 '깎아서' 못 버틴다 — 정지 상태에서 이미 불안정",
                 fontsize=10)
    fig.tight_layout()
    return _save(fig, "fig_att_gate.png")


def _smax(rows):
    taus = sorted({r["tau"] for r in rows})
    out = []
    for t in taus:
        ok = [r["s"] for r in rows if r["tau"] == t and r["ok"]]
        out.append(max(ok) if ok else 0.0)
    return taus, out


def fig_pos_spec():
    """위치 경로: 왜 복귀 게이트가 필요했나 + 기본표 vs 돌풍표."""
    rows = _read_progress("gust")
    nom = _read_progress("nominal")
    fig, ax = plt.subplots(1, 2, figsize=(9.4, 3.4))

    if rows:
        # 왼쪽: 각 (tau, s) 시도의 복귀 시간.
        # 복귀가 아예 없는 경우(NaN)를 빼고 그리면 최악 사례가 그림에서 사라진다 —
        # 맨 위에 빈 삼각형으로 올려 '안 돌아옴'을 보이게 한다.
        fin = [r["rec_s"] for r in rows if r["rec_s"] == r["rec_s"]]
        top = (max(fin) if fin else 3.0) * 1.18
        for r in rows:
            nan = r["rec_s"] != r["rec_s"]
            yv = top if nan else r["rec_s"]
            ax[0].scatter(r["tau"], yv, s=48,
                          facecolors=("none" if nan else (C_OK if r["ok"] else C_BAD)),
                          edgecolors=C_BAD if (nan or not r["ok"]) else C_OK,
                          marker=("^" if nan else ("o" if r["ok"] else "x")), zorder=3)
            ax[0].annotate(f"s={r['s']:.2f}", (r["tau"], yv),
                           textcoords="offset points", xytext=(6, 3), fontsize=6.5)
        if any(r["rec_s"] != r["rec_s"] for r in rows):
            ax[0].text(2, top, "△ = 끝까지 복귀 없음", color=C_BAD, fontsize=8, va="center")
        ax[0].set_ylim(0, top * 1.12)
        ax[0].axhline(3.0, color=C_BAD, ls=":", lw=1.2)
        ax[0].text(41, 3.3, "복귀 게이트 3 s", color=C_BAD, fontsize=8)
        ax[0].axhline(1.73, color=C_GRAY, ls="--", lw=1.0)
        ax[0].text(41, 0.9, "무지연 기준 1.73 s", color=C_GRAY, fontsize=8)
        ax[0].set_xlabel("위치 경로 지연 [ms]"); ax[0].set_ylabel("외란 복귀 시간 [s]")
        ax[0].set_title("외란 배터리: 복귀 시간 — 종단오차만 보면 놓친다")
        ax[0].grid(alpha=.3)

        # 오른쪽: 기본표(외란 없음) vs 돌풍표. 둘의 간격이 곧 '이중 감쇄' 의 크기다.
        tg, sg = _smax(rows)
        ax[1].plot(tg, [1.6 * v for v in sg], "s--", color=C_BAD,
                   label="돌풍표 (0.3 N·m)")
        if nom:
            tn, sn = _smax(nom)
            ax[1].plot(tn, [1.6 * v for v in sn], "o-", color=C_OK,
                       label="기본표 (외란 없음)")
            # 60 ms 간격을 화살표로 짚는다
            if 60 in tn and 60 in tg:
                a = 1.6 * sn[tn.index(60)]
                b = 1.6 * sg[tg.index(60)]
                ax[1].annotate("", xy=(60, a), xytext=(60, b),
                               arrowprops=dict(arrowstyle="<->", color="k", lw=1.0))
                ax[1].text(63, (a + b) / 2, f"{a / max(b, 1e-9):.1f}배", fontsize=8)
        ax[1].set_xlabel("위치 경로 지연 [ms]"); ax[1].set_ylabel("허용 속도 [m/s]")
        ax[1].set_title("기본표 vs 돌풍표 — 섞어 재면 이만큼 과감쇄")
        ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
    else:
        for a in ax:
            a.text(.5, .5, "스윕 결과 없음\n(sweep_delay_spec.m 먼저 실행)",
                   ha="center", va="center", fontsize=9, color=C_GRAY)
            a.set_xticks([]); a.set_yticks([])

    fig.suptitle("위치 경로 지연은 스펙 감쇄로 흡수된다 (자세와 다르다)", fontsize=10)
    fig.tight_layout()
    return _save(fig, "fig_pos_spec.png")


def _ref(dx=6.0, v0=1.6, tail=8.0, dt=0.01):
    tm = 2.1875 * dx / v0
    t = np.arange(0.0, tm + tail + dt, dt)
    return t, np.column_stack([dx * _smoothstep7(t / tm),
                               np.zeros_like(t), np.ones_like(t)]), tm


def fig_bridge():
    """다리 궤적 — 시계 배율, 위치, 그리고 '새 한계 안에 언제 들어오나'."""
    t, base, tm = _ref()
    t0 = tm / 2
    lim = {k: BASE_LIMITS[k] * 0.5 ** p
           for k, p in (("v", 1), ("a", 2), ("j", 3), ("snap", 4))}
    br = plan_bridge(t, base, t_now=t0, limits_new=lim, replan_budget_s=0.25)

    fig, ax = plt.subplots(1, 3, figsize=(12.4, 3.4))
    ax[0].plot(br.t - t0, br.s_of_t, color=C_OK)
    ax[0].axvline(br.failsafe_from_s - t0, color=C_BAD, ls=":", lw=1.2)
    ax[0].text(br.failsafe_from_s - t0 + .05, .55,
               "계획 미도착\n→ 정지 갈래", color=C_BAD, fontsize=8)
    ax[0].axvline(br.t_handoff - t0, color=C_ACC, ls="--", lw=1.2)
    ax[0].text(br.t_handoff - t0 + .05, .2, "인계", color=C_ACC, fontsize=8)
    ax[0].set_xlabel("감쇄 시작 후 [s]"); ax[0].set_ylabel("시계 배율 s")
    ax[0].set_title(f"7차 스무드스텝 하강 (램프 {br.t_ramp:.2f} s)")
    ax[0].grid(alpha=.3)

    i0 = int(round(t0 / 0.01))
    n = len(br.t)
    ax[1].plot(br.t - t0, br.pos[:, 0], color=C_OK, label="다리")
    ax[1].plot(br.t - t0, base[i0:i0 + n, 0] if i0 + n <= len(base)
               else np.pad(base[i0:, 0], (0, i0 + n - len(base)), mode="edge"),
               color=C_GRAY, ls="--", label="옛 기준 (감쇄 없이)")
    ax[1].set_xlabel("감쇄 시작 후 [s]"); ax[1].set_ylabel("x [m]")
    ax[1].set_title("기하는 그대로 — 경로 위에서 느려질 뿐")
    ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)

    # 감쇄 깊이 vs 새 한계 진입 시간 (= 필요한 선행 경보 시간)
    fracs = [0.85, 0.75, 0.60, 0.50, 0.40, 0.30, 0.22]
    lead, phys = [], []
    for f in fracs:
        L = {k: BASE_LIMITS[k] * f ** p
             for k, p in (("v", 1), ("a", 2), ("j", 3), ("snap", 4))}
        b = plan_bridge(t, base, t_now=t0, limits_new=L, replan_budget_s=0.25)
        lead.append(b.compliant_after_s)
        phys.append(max(b.phys_use.values()))
    ax[2].plot([BASE_LIMITS["v"] * f for f in fracs], lead, "o-", color=C_ACC,
               label="새 한계 진입까지")
    ax[2].axhline(0.25, color=C_BAD, ls=":", lw=1.2)
    ax[2].text(0.4, 0.33, "재계획 예산 0.25 s", color=C_BAD, fontsize=8)
    a2 = ax[2].twinx()
    a2.plot([BASE_LIMITS["v"] * f for f in fracs], phys, "s--", color=C_GRAY, ms=4,
            label="물리 한계 사용")
    a2.axhline(1.0, color=C_GRAY, ls=":", lw=.8)
    a2.set_ylabel("물리 한계 사용 [배]", color=C_GRAY); a2.set_ylim(0, 1.2)
    ax[2].set_xlabel("새 속도 한계 [m/s]"); ax[2].set_ylabel("시간 [s]", color=C_ACC)
    ax[2].set_title("깊이 깎을수록 늦게 든다 → 감쇄는 선행이어야")
    ax[2].grid(alpha=.3); ax[2].legend(fontsize=8, loc="upper right")

    fig.suptitle("재계획 인터벌 다리 — 기하 불변, 시계만 감속", fontsize=10)
    fig.tight_layout()
    return _save(fig, "fig_bridge.png")


def fig_tracker_lag():
    """지연 추적기: 느린 EMA 만 vs max(느린, 빠른). 왜 바꿨는지가 보여야 한다."""
    N, T_ON, T_OFF = 160, 40, 100
    xs = [0.075 if T_ON <= k < T_OFF else 0.012 for k in range(N)]

    tr = LatencyTracker()
    slow_only, both = [], []
    for x in xs:
        tr.update(x)
        slow_only.append(tr.ema_slow if tr.detected else 0.0)
        both.append(tr.predicted_s)

    fig, ax = plt.subplots(figsize=(7.2, 3.2))
    ax.plot(xs, color=C_GRAY, lw=.9, alpha=.7, label="실측 표본")
    ax.plot(slow_only, color=C_BAD, ls="--", label="구판: 느린 EMA 만")
    ax.plot(both, color=C_OK, label="신판: max(느린, 빠른)")
    ax.axvline(T_ON, color="k", lw=.7, alpha=.4)
    ax.axvline(T_OFF, color="k", lw=.7, alpha=.4)

    def first_at(seq, thr, start):
        for i in range(start, len(seq)):
            if seq[i] >= thr:
                return i
        return None
    thr = 0.05
    a = first_at(slow_only, thr, T_ON)
    b = first_at(both, thr, T_ON)
    if a and b:
        ax.annotate(f"{a - b} 표본 빨라짐", xy=(b, thr), xytext=(b + 12, thr + 0.018),
                    fontsize=8, color=C_OK,
                    arrowprops=dict(arrowstyle="->", color=C_OK, lw=.9))
    ax.set_xlabel("표본"); ax.set_ylabel("예측 지연 [s]")
    ax.set_title("지연 감지 지체 — 늦게 감지하면 다리 수렴까지 두 번 늦는다")
    ax.legend(fontsize=8); ax.grid(alpha=.3)
    fig.tight_layout()
    return _save(fig, "fig_tracker_lag.png")


def main():
    print("figure/11_delay/ 생성:")
    paths = [fig_att_gate(), fig_pos_spec(), fig_bridge(), fig_tracker_lag()]
    summary = {
        "att_sweep": {"ms": ATT_MS, "hover_rms_deg": ATT_RMS,
                      "hover_peak_deg": ATT_PK, "dist_peak_deg": ATT_DIST},
        "att_rule": {"clean_s": cap.LAT_ATT_CLEAN_S, "max_s": cap.LAT_ATT_MAX_S,
                     "margin_scale": cap.LAT_ATT_MARGIN_SCALE},
        "pos_sweep_gust": _read_progress("gust"),
        "pos_sweep_nominal": _read_progress("nominal"),
        "pos_anchors_gust": {str(k): v for k, v in cap._LAT_POS_ANCHORS_GUST.items()},
        "pos_anchors": {str(k): v for k, v in cap._LAT_POS_ANCHORS.items()},
        "figures": [os.path.basename(p) for p in paths],
    }
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(f"  {os.path.join(OUT, 'summary.json')}")


if __name__ == "__main__":
    main()
