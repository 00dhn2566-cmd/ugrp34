"""Markdown tables from an evaluation matrix folder (eval_matrix.sh output).

    python rl_planner/scripts/results_table.py /data/runs/rl_planner/eval_main > tables.md
    python rl_planner/scripts/results_table.py --safety /data/runs/rl_planner/eval_safety
    python rl_planner/scripts/results_table.py --ablation /data/runs/rl_planner/eval_ablation
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


def num(v, nd):
    return "–" if v != v else f"{v:.{nd}f}"       # NaN (no successful episode) -> dash


def row(label, s):
    f, r = s["fsm"], s["rl"]
    pr = s.get("_paired")
    pair = (f"{pr[0]} / {pr[1]} ({fmt_p(pr[2])}) | {num(pr[3], 2)} |" if pr else "– | – |")
    return (f"| {label} | {pct(f['success_rate'])} / **{pct(r['success_rate'])}** | {pair} "
            f"{pct(f['crash_rate'])} / {pct(r['crash_rate'])} | "
            f"{num(f['mean_time_success_s'], 1)} / **{num(r['mean_time_success_s'], 1)}** | "
            f"{num(f['mean_clearance_m'], 3)} / {num(r['mean_clearance_m'], 3)} | "
            f"{num(f['est_err_at_pass_m'], 3)} / {num(r['est_err_at_pass_m'], 3)} |")


HEAD = ("| 조건 | 성공률 (상태기계 / RL) | 같은 장면: RL만 / 상태기계만 성공 | 시간 비 RL/상태기계 | 충돌률 | 완료 시간 [s] "
        "| 통과 여유 [m] | 통과 시 추정 오차 [m] |\n"
        "|---|---|---|---|---|---|---|---|")


def main(root: str) -> int:
    out = []
    for pol, pol_name in (("isaac", "Isaac(PhysX) 학습 정책"), ("torch", "torch 학습 정책"),
                          ("robust", "Isaac 정책 + 카메라 끊김 DR 미세조정")):
        if not any(os.path.isdir(os.path.join(root, d)) for d in os.listdir(root) if d.startswith(f"{pol}pol_")):
            continue
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


SAFETY_ROWS = (("L4", "명목 L4"), ("L5", "명목 L5 (스트레스)"), ("blackout_L4", "카메라 끊김 3~6 s (L4)"),
               ("blind_L4", "검출 전무 (L4)"))
SAFETY_PLANNERS = (("fsm", "상태기계"), ("fsm+safe", "상태기계 + 안전 필터"), ("rl", "RL"), ("rl+safe", "RL + 안전 필터"))


def safety_table(root: str) -> int:
    out = ["| 조건 | 계획기 | 성공 | 충돌 | 시간 초과(충돌 없이 버팀) | 완료 시간 [s] |", "|---|---|---|---|---|---|"]
    for key, label in SAFETY_ROWS:
        p = os.path.join(root, key, "summary.json")
        if not os.path.exists(p):
            continue
        with open(p) as f:
            s = json.load(f)["summary"]
        for pk, pn in SAFETY_PLANNERS:
            if pk in s:
                x = s[pk]
                out.append(f"| {label} | {pn} | {pct(x['success_rate'])} | {pct(x['crash_rate'])} | "
                           f"{pct(x['timeout_rate'])} | {num(x['mean_time_success_s'], 1)} |")
    print("\n".join(out))
    return 0


ABLATIONS = (("full_s1", "전체 (시드 1)"), ("full_s2", "전체 (시드 2)"), ("full_s3", "전체 (시드 3)"),
             ("noshaping_s1", "추정 개선 보상 없음"), ("nocurr_s1", "커리큘럼 없음 (L5 고정)"),
             ("symcritic_s1", "대칭 비평가 (특권 정보 없음)"))


def first_iter(run_dir: str, tag: str, thr: float):
    """First training iteration at which the 10-iteration running mean of ``tag`` reaches ``thr``."""
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except ImportError:
        return None
    import glob
    for f in sorted(glob.glob(os.path.join(run_dir, "events.out.tfevents.*"))):
        ea = EventAccumulator(f, size_guidance={"scalars": 0})
        ea.Reload()
        if tag not in ea.Tags().get("scalars", []):
            continue
        v = [e.value for e in ea.Scalars(tag)]
        st = [e.step for e in ea.Scalars(tag)]
        for i in range(len(v)):
            w = v[max(0, i - 9):i + 1]
            if sum(w) / len(w) >= thr:
                return st[i]
    return None


def ablation_table(root: str, runs_root: str = "/data/runs/rl_planner/window_mission") -> int:
    import glob
    out = ["| 학습 설정 (각 500 반복) | L5 도달 반복 | L4 성공 / 충돌 / 시간 [s] | L5 성공 / 충돌 / 시간 [s] |",
           "|---|---|---|---|"]
    for key, label in ABLATIONS:
        cells = []
        runs = sorted(glob.glob(os.path.join(runs_root, f"*_abl_{key}")))
        it = first_iter(runs[-1], "Metrics/curriculum_level", 4.9) if runs and key != "nocurr_s1" else None
        cells.append("–" if it is None else str(it))
        for L in (4, 5):
            p = os.path.join(root, f"{key}_L{L}", "summary.json")
            if not os.path.exists(p):
                cells.append("–")
                continue
            with open(p) as f:
                r = json.load(f)["summary"]["rl"]
            cells.append(f"{pct(r['success_rate'])} / {pct(r['crash_rate'])} / {num(r['mean_time_success_s'], 1)}")
        out.append(f"| {label} | " + " | ".join(cells) + " |")
    print("\n".join(out))
    return 0


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--safety":
        raise SystemExit(safety_table(sys.argv[2]))
    if len(sys.argv) > 2 and sys.argv[1] == "--ablation":
        raise SystemExit(ablation_table(sys.argv[2]))
    raise SystemExit(main(sys.argv[1] if len(sys.argv) > 1 else "/data/runs/rl_planner/eval_main"))
