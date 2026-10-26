"""Where do planners cross the window plane, and how does that relate to their estimate? (precision)

    python rl_planner/scripts/pass_geometry.py --policy <run>/export --level 4 [--config stress.yaml ...]

Runs one episode per scene (same seeded scenes as evaluate.py) for the tuned state machine
and the policy, and at every window pass records: the target estimate's error split into
the window-normal (depth) part and the in-plane part, the drone's in-plane offset from the
TRUE opening centre (mean / 95th percentile), the crossing angle to the normal, and the
correlation between the left-right crossing offset and the left-right estimate error
(1 = the planner follows its final estimate exactly). RESULTS.md §4.
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


def run(env, planner, n: int) -> dict:
    import torch
    core = env.core
    active = torch.ones(n, dtype=torch.bool, device=env.device)
    rec = {k: [] for k in ("depth", "inplane", "offset", "angle", "e_r", "o_r")}
    post = core.post_decision

    def hooked():                        # read the pass before an auto-reset clears the mission
        post()
        sp = core.mission.step_pass & active[:, None]
        if not sp.any():
            return
        e, w = sp.nonzero(as_tuple=True)
        c, nrm, right = core.scene.center[e, w], core.scene.normal[e, w], core.scene.right[e, w]
        err = core.est.center[e, w] - c
        dn = (err * nrm).sum(-1)
        rel = core.state.pos[e] - c
        v = core.state.vel_w[e]
        rec["depth"].append(dn.abs())
        rec["inplane"].append((err - dn[:, None] * nrm).norm(dim=-1))
        rec["offset"].append((rel - (rel * nrm).sum(-1)[:, None] * nrm).norm(dim=-1))
        rec["angle"].append(torch.rad2deg(torch.acos(((v * nrm).sum(-1).abs() / v.norm(dim=-1).clamp_min(1e-6)).clamp(max=1.0))))
        rec["e_r"].append((err * right).sum(-1))
        rec["o_r"].append((rel * right).sum(-1))

    core.post_decision = hooked
    planner.reset(torch.arange(n, device=env.device))
    for _ in range(core.max_episode_length + 5):
        _, _, d, _ = env.step(planner.act(core))
        d = d.bool()
        if d.any():
            planner.reset(d.nonzero(as_tuple=False).squeeze(-1))
            active &= ~d
        if not bool(active.any()):
            break
    core.post_decision = post
    r = {k: torch.cat(v) for k, v in rec.items()}
    return {"passes": int(len(r["depth"])), "est_err_depth_m": float(r["depth"].mean()),
            "est_err_inplane_m": float(r["inplane"].mean()), "offset_mean_m": float(r["offset"].mean()),
            "offset_p95_m": float(r["offset"].quantile(0.95)), "crossing_angle_deg": float(r["angle"].mean()),
            "corr_lateral": float(torch.corrcoef(torch.stack((r["e_r"], r["o_r"])))[0, 1])}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", required=True)
    ap.add_argument("--level", type=int, default=4)
    ap.add_argument("--num-scenes", type=int, default=512)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--config", action="append", default=[], help="extra yaml (FSM tuning, stress condition)")
    ap.add_argument("--planners", default="fsm,rl")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)

    from rl_planner.core.config import config_for_policy
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.fsm import FSMPlanner
    from rl_planner.planners.policy import PolicyPlanner

    with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
        cfg = config_for_policy(json.load(f)["config"], a.config)
    n = a.num_scenes
    seed = int(cfg["eval"]["seed"] if a.seed is None else a.seed)
    res = {}
    print("| planner | passes | est. error, depth [m] | est. error, in-plane [m] | offset from true centre, mean / p95 [cm] "
          "| crossing angle [deg] | corr(left-right offset, left-right est. error) |")
    print("|---|---|---|---|---|---|---|")
    for name in a.planners.split(","):
        env = TorchWindowEnv(cfg, num_envs=n, seed=seed)
        env.core.fixed_level = a.level
        env.core.reseed(seed)
        env._reset_all()
        pl = PolicyPlanner(a.policy, device=env.device, cfg=cfg) if name == "rl" else FSMPlanner(cfg, n, env.device)
        res[name] = s = run(env, pl, n)
        print(f"| {name} | {s['passes']} | {s['est_err_depth_m']:.3f} | {s['est_err_inplane_m']:.3f} | "
              f"{100 * s['offset_mean_m']:.1f} / {100 * s['offset_p95_m']:.1f} | {s['crossing_angle_deg']:.1f} | {s['corr_lateral']:.2f} |")
    if a.out:
        os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
        with open(a.out, "w") as f:
            json.dump(res, f, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
