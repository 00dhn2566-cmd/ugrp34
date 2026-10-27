"""Grid-search the state-machine baseline on VALIDATION seeds (not the eval seeds),
so RL is compared against a tuned baseline rather than a hand-picked one.

    python rl_planner/scripts/tune_fsm.py --levels 4 5 --num-scenes 512 --seed 4242 --out /data/runs/rl_planner/fsm_tuning

Objective: mean success over the levels, tie-break by mean completion time.
Writes results.csv, best.yaml (an fsm: block usable with --config).
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

GRID = {
    "min_parallax_deg": [4.0, 8.0],
    "sigma_thr": [0.08, 0.15],
    "scan_amp": [0.2, 0.3],
    "d_app": [0.4, 0.6],
    "max_scan_rounds": [1, 2],
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--levels", type=int, nargs="+", default=[4, 5])
    ap.add_argument("--num-scenes", type=int, default=512)
    ap.add_argument("--seed", type=int, default=4242)
    ap.add_argument("--out", default="/data/runs/rl_planner/fsm_tuning")
    ap.add_argument("--set", action="append", default=[], help="dotted config override (e.g. perception.ray_forget=0.8)")
    ap.add_argument("--grid", action="append", default=[], help="replace one grid axis: key=v1,v2,...")
    ap.add_argument("--device", default=None, help="cpu or cuda:0 (default: config)")
    a = ap.parse_args(argv)
    for g in a.grid:
        k, v = g.split("=", 1)
        GRID[k] = [type(GRID[k][0])(x) for x in v.split(",")] if k in GRID else [float(x) for x in v.split(",")]

    import yaml

    from rl_planner.core.config import load_config
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.eval_harness import run_planner, summarize
    from rl_planner.planners.fsm import FSMPlanner

    os.makedirs(a.out, exist_ok=True)
    keys = list(GRID)
    rows = []
    for vals in itertools.product(*(GRID[k] for k in keys)):
        params = dict(zip(keys, vals))
        res = {}
        for L in a.levels:
            cfg = load_config(overrides=[f"seed={a.seed}", f"sim.num_envs={a.num_scenes}", *a.set])
            cfg["fsm"].update(params)
            env = TorchWindowEnv(cfg, num_envs=a.num_scenes, seed=a.seed, device=a.device)
            env.core.fixed_level = L
            env.core.reseed(a.seed)          # same scene set as evaluate.py --seed for this level
            env._reset_all()
            s = summarize(run_planner(env, FSMPlanner(cfg, a.num_scenes, env.device)))
            res[L] = s
        succ = sum(res[L]["success_rate"] for L in a.levels) / len(a.levels)
        t = sum(res[L]["mean_time_success_s"] for L in a.levels) / len(a.levels)
        row = {**params, "mean_success": round(succ, 4), "mean_time_s": round(t, 3),
               **{f"success_L{L}": round(res[L]["success_rate"], 4) for L in a.levels},
               **{f"time_L{L}": round(res[L]["mean_time_success_s"], 3) for L in a.levels}}
        rows.append(row)
        print(json.dumps(row), flush=True)
    rows.sort(key=lambda r: (-r["mean_success"], r["mean_time_s"]))
    with open(os.path.join(a.out, "results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    best = {k: rows[0][k] for k in keys}
    with open(os.path.join(a.out, "best.yaml"), "w") as f:
        yaml.safe_dump({"fsm": best}, f, sort_keys=False)
    print("BEST", json.dumps(rows[0]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
