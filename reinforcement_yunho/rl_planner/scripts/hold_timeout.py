"""Why do safety-filter holds end in timeouts? Policy behind the filter, one condition, outcome by episode.

    python rl_planner/scripts/hold_timeout.py --policy <run>/export --config rl_planner/configs/general/blackout7.yaml \
        --level 4 [--set safety.retreat_after_s=null ...]

Same seeded scenes as the benchmark (validation seed 4242, CPU). Per outcome (success / crash / timeout): how many
episodes held at least once, were still holding at the end, hold time and number of holds, and where the drone was at
the end relative to the target window — bearing off the camera axis, distance, signed distance along the normal,
lateral and vertical offset, whether the target had a valid estimate and its sigma. A timeout that ends holding a
few metres short of an unestimated window, too close or too far off-axis to see its four corners, is the deadlock
the retreat (safety.retreat_after_s) resolves. RESULTS.md §11.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", required=True, help="export dir")
    ap.add_argument("--level", type=int, default=4)
    ap.add_argument("--config", action="append", default=[], help="condition yaml merged over the policy config")
    ap.add_argument("--set", action="append", default=[], help="dotted override (filter settings)")
    ap.add_argument("--num-scenes", type=int, default=512)
    ap.add_argument("--seed", type=int, default=4242, help="4242 = validation seed (RESULTS.md §10)")
    ap.add_argument("--threads", type=int, default=4)
    a = ap.parse_args(argv)
    import torch
    from rl_planner.core.config import config_for_policy
    from rl_planner.core.observations import _gather
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.policy import PolicyPlanner
    from rl_planner.planners.safety import SafePlanner
    torch.set_num_threads(a.threads)
    with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
        cfg = config_for_policy(json.load(f)["config"], a.config, a.set)
    E = a.num_scenes
    env = TorchWindowEnv(cfg, num_envs=E, device="cpu", seed=a.seed)
    core = env.core
    core.fixed_level = a.level
    core.reseed(a.seed)
    env._reset_all()
    sp = SafePlanner(PolicyPlanner(a.policy, device="cpu", cfg=cfg), cfg, E, "cpu", core.step_dt)
    sp.reset(torch.arange(E))
    done = torch.zeros(E, dtype=torch.bool)
    held = torch.zeros(E, dtype=torch.bool)
    hold_steps, n_holds = torch.zeros(E), torch.zeros(E)
    was = torch.zeros(E, dtype=torch.bool)
    end = {k: torch.zeros(E) for k in ("hold", "bear", "dist", "n", "lat", "up", "valid", "sigma")}
    out = torch.zeros(E, dtype=torch.long)
    for _ in range(int(core.episode_length_s / core.step_dt) + 5):
        act = sp.act(core)
        h = sp.safety.holding & ~done
        held |= h
        hold_steps += h.float()
        n_holds += (h & ~was).float()
        was = h.clone()
        tg = core.target_idx.clamp_max(core.W - 1)
        c, nrm = _gather(core.scene.center, tg), _gather(core.scene.normal, tg)
        d = c - core.state.pos
        bear = torch.atan2(d[:, 1], d[:, 0]) - core.yaw
        rel = core.state.pos - c
        lat = rel - (rel * nrm).sum(-1, keepdim=True) * nrm
        lat[:, 2] = 0
        now = {"hold": h.float(), "bear": torch.atan2(torch.sin(bear), torch.cos(bear)).abs() * 180 / math.pi,
               "dist": d.norm(dim=-1), "n": (rel * nrm).sum(-1), "lat": lat.norm(dim=-1), "up": rel[:, 2],
               "valid": _gather(core.est.valid, tg).float(), "sigma": _gather(core.est.sigma, tg)}
        live = ~done
        for k, v in now.items():
            end[k][live] = v[live]
        _, _, dn, _ = env.step(act)
        dn = dn.bool() & ~done
        rec = core.episode_record
        out[dn & rec["success"]] = 1
        out[dn & rec["crashed"]] = 2
        out[dn & ~rec["success"] & ~rec["crashed"]] = 3
        done |= dn
        if bool(done.all()):
            break
    print(f"{' '.join(a.config + a.set)} L{a.level}: held at least once {int(held.sum())}/{E}")
    for o, name in ((1, "success"), (2, "crash"), (3, "timeout")):
        m = out == o
        if not bool(m.any()):
            continue
        hm = m & held
        v = m & (end["valid"] > 0)
        print(f"  {name:8s} n={int(m.sum()):3d} held={int(hm.sum()):3d} holding at end={int((m & (end['hold'] > 0)).sum()):3d} "
              f"hold {float(hold_steps[hm].mean()) * core.step_dt if hm.any() else 0:.1f} s in {float(n_holds[hm].mean()) if hm.any() else 0:.1f} holds"
              f" | at the end: target bearing median {float(end['bear'][m].median()):.0f} deg (>35: {int((m & (end['bear'] > 35)).sum())}), "
              f"distance {float(end['dist'][m].median()):.2f} m, along normal {float(end['n'][m].median()):+.2f}, "
              f"lateral {float(end['lat'][m].median()):.2f}, up {float(end['up'][m].median()):+.2f} (|up|>0.35: "
              f"{int((m & (end['up'].abs() > 0.35)).sum())}), valid estimate {int(v.sum())}, sigma median "
              f"{float(end['sigma'][v].median()) if v.any() else float('nan'):.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
