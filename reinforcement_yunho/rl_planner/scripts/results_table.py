"""Markdown tables from an evaluation matrix folder (eval_matrix.sh output).

    python rl_planner/scripts/results_table.py /data/runs/rl_planner/eval_main > tables.md
"""
from __future__ import annotations

import json
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
}


def load(d):
    p = os.path.join(d, "summary.json")
    if not os.path.exists(p):
        return None
    with open(p) as f:
        return json.load(f)["summary"]


def pct(v):
    return f"{100 * v:.1f}%"


def row(label, s):
    f, r = s["fsm"], s["rl"]
    return (f"| {label} | {pct(f['success_rate'])} / **{pct(r['success_rate'])}** | "
            f"{pct(f['crash_rate'])} / {pct(r['crash_rate'])} | "
            f"{f['mean_time_success_s']:.1f} / **{r['mean_time_success_s']:.1f}** | "
            f"{f['mean_clearance_m']:.3f} / {r['mean_clearance_m']:.3f} | "
            f"{f['est_err_at_pass_m']:.3f} / {r['est_err_at_pass_m']:.3f} |")


HEAD = ("| 조건 | 성공률 (상태기계 / RL) | 충돌률 | 완료 시간 [s] | 통과 여유 [m] | 통과 시 추정 오차 [m] |\n"
        "|---|---|---|---|---|---|")


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
