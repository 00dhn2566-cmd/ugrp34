"""Flight-behaviour statistics from recorded trajectories (evaluate.py --record N).

    python rl_planner/scripts/analyze_traj.py <eval_dir> [<eval_dir> ...]

Per planner, over the recorded scenes: mean speed, path length / straight-line
distance, share of time nearly stopped (< 0.3 m/s), lateral and vertical travel
per metre of forward progress (how much the planner weaves to gather parallax).
Prints a markdown table and writes traj_stats.json next to the trajectories.
"""
from __future__ import annotations

import glob
import json
import os
import sys

import torch

DT = 0.1          # decision period of the recorded positions [s]
SLOW = 0.3        # "nearly stopped" speed [m/s]


def stats(paths) -> dict:
    v_mean, ratio, slow, lat, vert = [], [], [], [], []
    for P in paths:
        P = torch.tensor(P, dtype=torch.float64)
        if len(P) < 3:
            continue
        d = P[1:] - P[:-1]
        seg = d.norm(dim=-1)
        L = float(seg.sum())
        chord = float((P[-1] - P[0]).norm())
        fwd = float(P[-1, 0] - P[0, 0])
        v_mean.append(L / (DT * len(seg)))
        ratio.append(L / max(chord, 1e-6))
        slow.append(float((seg / DT < SLOW).double().mean()))
        lat.append(float(d[:, 1].abs().sum()) / max(fwd, 1e-6))
        vert.append(float(d[:, 2].abs().sum()) / max(fwd, 1e-6))
    m = lambda x: sum(x) / len(x) if x else float("nan")  # noqa: E731
    return {"scenes": len(v_mean), "mean_speed_mps": m(v_mean), "path_over_chord": m(ratio),
            "time_nearly_stopped": m(slow), "lateral_per_forward": m(lat), "vertical_per_forward": m(vert)}


def main(dirs) -> int:
    print("| eval | planner | scenes | mean speed [m/s] | path / straight line | time < 0.3 m/s | lateral m per forward m | vertical m per forward m |")
    print("|---|---|---|---|---|---|---|---|")
    for d in dirs:
        out = {}
        for f in sorted(glob.glob(os.path.join(d, "traj_*.pt"))):
            name = os.path.basename(f)[5:-3]
            out[name] = s = stats(torch.load(f)["paths"])
            print(f"| {os.path.basename(d.rstrip('/'))} | {name} | {s['scenes']} | {s['mean_speed_mps']:.2f} | "
                  f"{s['path_over_chord']:.2f} | {100 * s['time_nearly_stopped']:.0f}% | "
                  f"{s['lateral_per_forward']:.2f} | {s['vertical_per_forward']:.2f} |")
        with open(os.path.join(d, "traj_stats.json"), "w") as fh:
            json.dump(out, fh, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
