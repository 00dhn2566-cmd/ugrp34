"""Safety-filter settings on the validation seed: planner x (hold point, blind slow-down, retreat) -> mean success /
crash / timeout over the conditions every setting of that planner has, key conditions, crash rate with no detections.

    python rl_planner/scripts/filter_grid.py gen_mlp:rl+safe speed_a:rl+safe [--base /data/runs/rl_planner]

Result roots (all written by scripts/benchmark.py --seed 4242, see /data/runs/val_cand.sh, val_grid.sh, val_retreat.sh):
bench_val (hold where it began, no slow-down), bench_val_slow1 / _slow2 (+ slow-down 1 / 2 s) and
bench_val_grid/b<brake>_s<slow>[_r<retreat>] (braking stop point, slow-down, retreat_after_s). RESULTS.md §10, §11.
"""
from __future__ import annotations

import argparse
import glob
import json
import os

KEY = ["blackout3", "blackout7", "blackout10", "combo_hard", "geom_wide", "yaw55", "nominal_L5"]


def roots(base: str) -> list:
    out = [("latch", "none", "–", f"{base}/bench_val"), ("latch", "1.0", "–", f"{base}/bench_val_slow1"),
           ("latch", "2.0", "–", f"{base}/bench_val_slow2")]
    for d in sorted(glob.glob(f"{base}/bench_val_grid/b*_s*")):
        parts = os.path.basename(d)[1:].split("_")             # 1.5, s2.0[, r1.0]
        retreat = parts[2][1:] if len(parts) > 2 else "–"
        out.append((f"brake{parts[0]}", parts[1][1:], retreat, d))
    return out


def load(root: str, stem: str):
    res, blind = {}, None
    for f in glob.glob(f"{root}/*/{stem}.json"):
        d = json.load(open(f, encoding="utf-8"))
        s = d["summary"]
        if d["meta"].get("tier") == "safety":
            blind = s["crash_rate"]
        else:
            res[os.path.basename(os.path.dirname(f))] = (s["success_rate"], s["crash_rate"], s["timeout_rate"])
    return res, blind


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("planners", nargs="+", help="planner as in benchmark.py (e.g. gen_mlp:rl+safe, fsm+safe)")
    ap.add_argument("--base", default="/data/runs/rl_planner")
    a = ap.parse_args(argv)
    for planner in a.planners:
        stem = planner.replace(":", "__").replace("+", "_plus_")
        rows = [(h, s, r, *load(root, stem)) for h, s, r, root in roots(a.base)]
        rows = [x for x in rows if x[3]]
        if not rows:
            print(f"\n### {planner}: no results")
            continue
        common = set.intersection(*(set(x[3]) for x in rows))
        print(f"\n### {planner}  (mean over {len(common)} common conditions)")
        print("| hold | slow | retreat | mean succ / crash / timeout | " + " | ".join(KEY) + " | blind crash |")
        print("|---" * (len(KEY) + 5) + "|")
        for h, s, r, res, blind in rows:
            m = [res[c] for c in common]
            ms, mc, mt = (sum(v[i] for v in m) / len(m) * 100 for i in range(3))
            cells = " | ".join(f"{res[k][0] * 100:.1f}" if k in res else "–" for k in KEY)
            print(f"| {h} | {s} | {r} | {ms:.2f} / {mc:.2f} / {mt:.2f} | {cells} | "
                  f"{'–' if blind is None else f'{blind * 100:.1f}'} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
