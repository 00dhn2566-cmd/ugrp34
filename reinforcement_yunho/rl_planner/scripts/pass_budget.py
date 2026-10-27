"""Why is a fast policy less robust? Per window pass: speed, camera frames of that window, estimate error and aim error.

    python rl_planner/scripts/pass_budget.py --policy gen_mlp=<run>/export --policy speed_a=<run>/export \
        --level 4 [--config rl_planner/configs/general/yaw55.yaml] [--set k=v ...]

Runs every policy alone (no safety filter) on the same seeded scenes (validation seed 4242 by default, CPU) and at
every window pass records, in the window plane: the estimate error (estimated - true centre), the aim error (drone -
estimated centre) and the drone's offset from the true centre, with the speed and the number of camera frames that
went into that window's estimate. At every crash: reason, which window, speed, frames, estimate error, distance to
the plane and in-plane offset. Last, the estimate error binned by frames — equal bins mean the estimator is the same
and only the number of frames differs. RESULTS.md §11.
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


def _inplane(v, n):
    return (v - (v * n).sum(-1, keepdim=True) * n).norm(dim=-1)


def run(export: str, extra, sets, level: int, n: int, seed: int):
    import torch
    from rl_planner.core.config import config_for_policy
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.policy import PolicyPlanner
    with open(os.path.join(export, "policy_meta.json"), encoding="utf-8") as f:
        cfg = config_for_policy(json.load(f)["config"], extra, sets)
    env = TorchWindowEnv(cfg, num_envs=n, device="cpu", seed=seed)
    core = env.core
    core.fixed_level = level
    core.reseed(seed)
    env._reset_all()
    planner = PolicyPlanner(export, device="cpu", cfg=cfg)
    planner.reset(torch.arange(n))
    active = torch.ones(n, dtype=torch.bool)
    was_crashed = torch.zeros(n, dtype=torch.bool)
    passes, crashes = [], []
    post = core.post_decision

    def hooked():                        # read passes and crashes before an auto-reset clears the mission
        post()
        m = core.mission
        sp = m.step_pass & active[:, None]
        if sp.any():
            e, w = sp.nonzero(as_tuple=True)
            c, nrm, est = core.scene.center[e, w], core.scene.normal[e, w], core.est.center[e, w]
            passes.append(torch.stack([core.state.vel_w[e].norm(dim=-1), core.perception.n_obs[e, w].float(),
                                       _inplane(est - c, nrm), _inplane(core.state.pos[e] - est, nrm),
                                       _inplane(core.state.pos[e] - c, nrm)], -1))
        new = m.crashed & ~was_crashed & active
        was_crashed.copy_(m.crashed)
        if new.any():
            e = new.nonzero(as_tuple=True)[0]
            w = m.target[e].clamp_max(core.W - 1)
            c, nrm, est = core.scene.center[e, w], core.scene.normal[e, w], core.est.center[e, w]
            valid = core.est.valid[e, w].float()
            crashes.append(torch.stack([m.crash_reason[e].float(), w.float(), core.state.vel_w[e].norm(dim=-1),
                                        core.perception.n_obs[e, w].float(), _inplane(est - c, nrm) * valid - (1 - valid),
                                        ((core.state.pos[e] - c) * nrm).sum(-1), _inplane(core.state.pos[e] - c, nrm)], -1))
    core.post_decision = hooked
    for _ in range(int(core.episode_length_s / core.step_dt) + 5):
        _, _, d, _ = env.step(planner.act(core))
        d = d.bool()
        if d.any():
            planner.reset(d.nonzero(as_tuple=False).squeeze(-1))
            was_crashed &= ~d
            active &= ~d
        if not bool(active.any()):
            break
    return (torch.cat(passes) if passes else torch.zeros(0, 5)), (torch.cat(crashes) if crashes else torch.zeros(0, 7))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", action="append", required=True, help="name=<export dir>")
    ap.add_argument("--level", type=int, default=4)
    ap.add_argument("--config", action="append", default=[], help="condition yaml merged over the policy config")
    ap.add_argument("--set", action="append", default=[], help="dotted override")
    ap.add_argument("--num-scenes", type=int, default=512)
    ap.add_argument("--seed", type=int, default=4242, help="4242 = validation seed (RESULTS.md §10)")
    ap.add_argument("--threads", type=int, default=4)
    a = ap.parse_args(argv)
    import torch
    from rl_planner.core.mission import CRASH_NAMES
    torch.set_num_threads(a.threads)
    pooled = []
    for spec in a.policy:
        name, export = spec.split("=", 1)
        p, k = run(export, a.config, a.set, a.level, a.num_scenes, a.seed)
        pooled.append((name, p))
        q = lambda v, x: float(v.quantile(x)) if len(v) else float("nan")  # noqa: E731
        print(f"\n== {name}  L{a.level} {' '.join(a.config + a.set)}: passes {len(p)}, crashes {len(k)}")
        print(f"   at pass: speed {p[:, 0].mean():.2f} m/s | frames of that window {p[:, 1].mean():.1f} (p10 {q(p[:, 1], 0.1):.0f}) | "
              f"estimate error {p[:, 2].mean():.3f} (p95 {q(p[:, 2], 0.95):.3f}) | aim error {p[:, 3].mean():.3f} "
              f"(p95 {q(p[:, 3], 0.95):.3f}) | offset from true centre {p[:, 4].mean():.3f} (p95 {q(p[:, 4], 0.95):.3f}) m")
        for r in sorted(set(k[:, 0].long().tolist())):
            c = k[k[:, 0] == r]
            ok = c[:, 4] >= 0
            print(f"   crash {CRASH_NAMES[r]:9s} n={len(c):3d}: window #{c[:, 1].mean() + 1:.1f}, speed {c[:, 2].mean():.2f}, "
                  f"frames {c[:, 3].mean():.1f}, estimate error {c[ok, 4].mean() if ok.any() else float('nan'):.3f} "
                  f"(no estimate {int((~ok).sum())}), to plane {c[:, 5].median():+.2f}, in-plane offset {c[:, 6].median():.2f} m")
    print("\n   estimate error at the pass by frames of that window:")
    for lo, hi in ((0, 8), (8, 12), (12, 16), (16, 24), (24, 10**6)):
        cells = []
        for name, p in pooled:
            m = (p[:, 1] >= lo) & (p[:, 1] < hi)
            cells.append(f"{name} {p[m, 2].mean() if m.any() else float('nan'):.3f} (n={int(m.sum())})")
        print(f"   frames [{lo},{hi if hi < 10**6 else 'inf'}): " + " | ".join(cells))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
