"""How long do planners fly with no detection and no valid target estimate? (safety-filter tuning)

    python rl_planner/scripts/perception_gaps.py --policy <run>/export --level 4 [--planner rl|fsm]

Runs one episode per scene (same seeded scenes as evaluate.py) WITHOUT intervening and
records, per scene, the longest streak of decisions in which no window was detected
and the current target had no valid estimate — the state planners/safety.py treats as
"lost" — split into streaks that start before the first detection and later ones.
Prints the share of scenes whose streak exceeds each candidate threshold, i.e. how
often a hold rule with that threshold would intervene in nominal flight.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", required=True)
    ap.add_argument("--planner", default="rl", choices=("rl", "fsm"))
    ap.add_argument("--level", type=int, default=4)
    ap.add_argument("--num-scenes", type=int, default=512)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--config", action="append", default=[])
    ap.add_argument("--sigma-safe", type=float, default=None,
                    help="count a valid target estimate looser than this [m] as unknown (safety.sigma_safe_m)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)

    import torch

    from rl_planner.core.config import config_for_policy
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.fsm import FSMPlanner
    from rl_planner.planners.policy import PolicyPlanner

    with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
        cfg = config_for_policy(json.load(f)["config"], a.config)
    n = a.num_scenes
    seed = int(cfg["eval"]["seed"] if a.seed is None else a.seed)
    env = TorchWindowEnv(cfg, num_envs=n, seed=seed)
    core = env.core
    core.fixed_level = a.level
    core.reseed(seed)
    env._reset_all()
    planner = (PolicyPlanner(a.policy, device=env.device, cfg=cfg) if a.planner == "rl"
               else FSMPlanner(cfg, n, env.device))
    planner.reset(torch.arange(n, device=env.device))

    E, dev = n, env.device
    streak = torch.zeros(E, dtype=torch.long, device=dev)
    ever_seen = torch.zeros(E, dtype=torch.bool, device=dev)
    max_start = torch.zeros(E, dtype=torch.long, device=dev)   # before the first detection
    max_later = torch.zeros(E, dtype=torch.long, device=dev)
    active = torch.ones(E, dtype=torch.bool, device=dev)       # still in its first episode
    success = torch.zeros(E, dtype=torch.bool, device=dev)
    for _ in range(core.max_episode_length + 5):
        seen = core.det.detected.any(-1)
        valid = core.est.valid.gather(1, core.target_idx[:, None]).squeeze(1)
        if a.sigma_safe is not None:
            valid &= core.est.sigma.gather(1, core.target_idx[:, None]).squeeze(1) <= a.sigma_safe
        ever_seen |= seen & active
        lost = ~seen & ~valid & active
        streak = torch.where(lost, streak + 1, torch.zeros_like(streak))
        max_start = torch.where(~ever_seen, torch.maximum(max_start, streak), max_start)
        max_later = torch.where(ever_seen, torch.maximum(max_later, streak), max_later)
        _, _, d, _ = env.step(planner.act(core))
        d = d.bool()
        if d.any():
            ids = d.nonzero(as_tuple=False).squeeze(-1)
            planner.reset(ids)
            success |= d & active & core.episode_record["success"]
            active &= ~d
        if not bool(active.any()):
            break
    dt = core.step_dt
    res = {"planner": a.planner, "level": a.level, "scenes": n, "success_rate": float(success.float().mean()),
           "max_lost_s_start": {"p50": float(max_start.float().median()) * dt, "max": float(max_start.max()) * dt},
           "max_lost_s_later": {"p50": float(max_later.float().median()) * dt, "max": float(max_later.max()) * dt},
           "share_exceeding": {}}
    print(f"{a.planner} L{a.level}: success {100 * res['success_rate']:.1f}%  "
          f"longest lost streak before first detection max {res['max_lost_s_start']['max']:.1f}s, "
          f"after max {res['max_lost_s_later']['max']:.1f}s")
    print("| threshold [s] | scenes with a longer streak before first detection | after first detection |")
    print("|---|---|---|")
    for thr in (0.3, 0.5, 1.0, 1.5, 2.0, 3.0):
        k = int(round(thr / dt))
        s0, s1 = float((max_start > k).float().mean()), float((max_later > k).float().mean())
        res["share_exceeding"][str(thr)] = {"start": s0, "later": s1}
        print(f"| {thr} | {100 * s0:.1f}% | {100 * s1:.1f}% |")
    if a.out:
        os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
        with open(a.out, "w") as f:
            json.dump(res, f, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
