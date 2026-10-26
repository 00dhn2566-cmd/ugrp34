"""Markdown tables from an evaluation matrix folder (eval_matrix.sh output).

    python rl_planner/scripts/results_table.py /data/runs/rl_planner/eval_main > tables.md
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys

PLANNERS = (("fsm", "상태기계"), ("rl", "RL"))
STRESS_NAMES = {
    "outliers": "코너 아웃라이어 3%",
    "noise_x1p7": "코너 잡음 ×1.7",
    "calib_3pct": "초점거리 보정 오차 3%",
    "vio_noise": "VIO 자세 잡음 (3 cm, 0.5°)",
    "wind_strong": "강풍 0.6~1.0 m/s²",
    "heavy": "과중량 (질량 ×1.1~1.2, 추력 ×0.85~0.9)",
    "blackout": "카메라 끊김 3~6 s",
    "blind": "검출 전무 (성공 불가 — 충돌 여부만 의미)",
}


def load(d):
    p = os.path.join(d, "summary.json")
    if not os.path.exists(p):
        return None
    with open(p) as f:
        s = json.load(f)["summary"]
    s["_paired"] = paired(d)
    return s


def paired(d):
    """Same-scene comparison from episodes.csv: (#RL-only successes, #FSM-only successes,
    McNemar exact two-sided p, mean RL/FSM time ratio on scenes both solved)."""
    p = os.path.join(d, "episodes.csv")
    if not os.path.exists(p):
        return None
    ep = {}
    with open(p, newline="") as f:
        for r in csv.DictReader(f):
            ep.setdefault(r["planner"], {})[int(r["env"])] = (r["success"] == "True", float(r["time_s"]))
    if "fsm" not in ep or "rl" not in ep:
        return None
    f, r = ep["fsm"], ep["rl"]
    common = sorted(set(f) & set(r))
    rl_only = sum(1 for e in common if r[e][0] and not f[e][0])
    fsm_only = sum(1 for e in common if f[e][0] and not r[e][0])
    both = [e for e in common if r[e][0] and f[e][0]]
    ratio = sum(r[e][1] / f[e][1] for e in both) / len(both) if both else float("nan")
    return rl_only, fsm_only, mcnemar_p(rl_only, fsm_only), ratio


def mcnemar_p(b, c):
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def fmt_p(p):
    return "p<1e-4" if p < 1e-4 else f"p={p:.2g}"


def pct(v):
    return f"{100 * v:.1f}%"


def row(label, s):
    f, r = s["fsm"], s["rl"]
    pr = s.get("_paired")
    pair = (f"{pr[0]} / {pr[1]} ({fmt_p(pr[2])}) | {pr[3]:.2f} |" if pr else "– | – |")
    return (f"| {label} | {pct(f['success_rate'])} / **{pct(r['success_rate'])}** | {pair} "
            f"{pct(f['crash_rate'])} / {pct(r['crash_rate'])} | "
            f"{f['mean_time_success_s']:.1f} / **{r['mean_time_success_s']:.1f}** | "
            f"{f['mean_clearance_m']:.3f} / {r['mean_clearance_m']:.3f} | "
            f"{f['est_err_at_pass_m']:.3f} / {r['est_err_at_pass_m']:.3f} |")


HEAD = ("| 조건 | 성공률 (상태기계 / RL) | 같은 장면: RL만 / 상태기계만 성공 | 시간 비 RL/상태기계 | 충돌률 | 완료 시간 [s] "
        "| 통과 여유 [m] | 통과 시 추정 오차 [m] |\n"
        "|---|---|---|---|---|---|---|---|")


def main(root: str) -> int:
    out = []
    for pol, pol_name in (("isaac", "Isaac(PhysX) 학습 정책"), ("torch", "torch 학습 정책")):
        out.append(f"\n### {pol_name}\n")
        out.append(HEAD)
        for be, be_name in (("isaac", "Isaac"), ("torch", "torch")):
            for L in (3, 4, 5):
                s = load(os.path.join(root, f"{pol}pol_{be}_L{L}"))
                if s:
                    out.append(row(f"{be_name} 평가, L{L}", s))
        out.append(f"\n**학습에 없던 조건 (torch 평가, L4)** — {pol_name}\n")
        out.append(HEAD)
        s = load(os.path.join(root, f"{pol}pol_torch_L4"))
        if s:
            out.append(row("기준 (명목 L4)", s))
        for key, name in STRESS_NAMES.items():
            s = load(os.path.join(root, f"{pol}pol_torch_L4_{key}"))
            if s:
                out.append(row(name, s))
    print("\n".join(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1] if len(sys.argv) > 1 else "/data/runs/rl_planner/eval_main"))
