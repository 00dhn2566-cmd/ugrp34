"""Distil the teacher's demonstrations (scripts/teach_cem.py) into a policy checkpoint (RESULTS.md §15).

Behaviour cloning of the actor's mean action on the observations the policy itself sees along each demonstration
(replayed in the teacher's env: its scene, nominal model, the level's perception noise), from the start until
--tail-s after the load is through. A self-distillation term on the policy's own states (training config, its
curriculum) keeps the rest of what it knows: search, approach, later windows. The critic and the observation
normaliser are kept; PPO continues from the result with train.py --resume.

    python rl_planner/scripts/bc_warmstart.py --resume model.pt --demos 'DIR/batch_*.pt' --config ... --set OBS --out bc.pt

--demos may repeat (teachers with different settings, e.g. teach_cem.py --passes 1 and 2): each teacher env is
replayed on its own.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time

import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from rl_planner.scripts.teach_cem import SCENE_KEYS  # noqa: E402


def load_demos(patterns) -> list:
    """-> [(robust demonstrations, batch meta)] per teacher env (configs, settings, horizon, level, passes)."""
    files = sorted({f for p in ([patterns] if isinstance(patterns, str) else patterns) for f in glob.glob(p)})
    if not files:
        raise FileNotFoundError(patterns)
    env_of = lambda d: (tuple(d["configs"]), tuple(s for s in d["sets"] if not s.startswith("device=")), d["T"],  # noqa: E731
                        d["level"], d.get("args", {}).get("passes", 1))
    groups = {}
    for f in files:
        d = torch.load(f, weights_only=False)
        g = groups.setdefault(env_of(d), ([], d))
        g[0].extend(x for x in d["demos"] if x["robust"])
    return [g for g in groups.values() if g[0]]


@torch.no_grad()
def replay(demos, meta, device: str, tail_s: float):
    """Open-loop replay of every demonstration in the teacher's env -> (obs, action) pairs up to tail_s after the
    load is through the teacher's K windows, and how many still pass robustly (deterministic: all of them should)."""
    from rl_planner.core.config import load_config
    from rl_planner.envs.torch_env import TorchWindowEnv
    sets = [s for s in meta["sets"] if not s.startswith("device=")] + [f"device={device}"]
    cfg = load_config(meta["configs"], sets)
    E, T, K = len(demos), meta["T"], meta.get("args", {}).get("passes", 1)
    env = TorchWindowEnv(cfg, num_envs=E, device=device, seed=0)
    core = env.core
    core.fixed_level = meta["level"]
    env._reset_all()
    sc = core.scene
    for k in SCENE_KEYS:
        getattr(sc, k).copy_(torch.stack([d["scene"][k] for d in demos]).to(getattr(sc, k).device))
    m, s = core.mission, core.state
    m.prev_d = ((s.pos[:, None, :] - sc.center) * sc.normal).sum(-1)
    m.prev_dL = ((core.slung.pos[:, None, :] - sc.center) * sc.normal).sum(-1)
    acts = torch.stack([d["actions"] for d in demos]).to(device)
    fin = torch.zeros(E, dtype=torch.bool, device=device)
    through_t = torch.full((E,), float("inf"), device=device)
    obs_l, act_l = [], []
    for t in range(T):
        keep = ~fin & (t * core.step_dt <= through_t + tail_s)
        o = env.get_observations()["policy"]
        obs_l.append(o[keep].clone())
        act_l.append(acts[keep, t].clone())
        _, _, d, _ = env.step(acts[:, t])
        fin |= d.bool()
        both = (m.target >= K) & m.load_past[:, :K].all(-1) & ~fin
        through_t = torch.where(both & torch.isinf(through_t), torch.full_like(through_t, (t + 1) * core.step_dt), through_t)
    ok = ~fin & (m.target >= K) & m.load_past[:, :K].all(-1)
    return torch.cat(obs_l), torch.cat(act_l), int(ok.sum())


@torch.no_grad()
def own_states(env, actor, steps: int):
    """The policy's own (obs, deterministic action) pairs in the training env."""
    from tensordict import TensorDict
    obs_l, act_l = [], []
    obs = env.get_observations()
    for _ in range(steps):
        a = actor(obs)
        obs_l.append(obs["policy"].clone())
        act_l.append(a.clone())
        obs, _, _, _ = env.step(a)
        if not isinstance(obs, TensorDict):
            obs = env.get_observations()
    return torch.cat(obs_l), torch.cat(act_l)


def interval_mse(y: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
    """The env flies clip(y, -1, 1): a target at a bound is met by anything beyond it, any other target only by itself."""
    lo = torch.where(t <= -1.0, torch.full_like(t, -torch.inf), t)
    hi = torch.where(t >= 1.0, torch.full_like(t, torch.inf), t)
    return (y - y.clamp(min=lo, max=hi)).pow(2).mean()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--resume", required=True, help="model_*.pt to start from")
    ap.add_argument("--demos", action="append", required=True, help="glob of teach_cem.py batch files (repeatable)")
    ap.add_argument("--config", action="append", required=True, help="the training configs (PPO continues with them)")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--tail-s", type=float, default=0.5, help="clone until this long after the load is through [s]")
    ap.add_argument("--self-weight", type=float, default=0.1, help="weight of the self-distillation term")
    ap.add_argument("--self-envs", type=int, default=1024)
    ap.add_argument("--self-steps", type=int, default=200)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch", type=int, default=1024)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    from rsl_rl.runners import OnPolicyRunner
    from tensordict import TensorDict

    from rl_planner.core.config import load_config
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.scripts.train import runner_cfg
    t0 = time.time()
    parts = []
    for demos, meta in load_demos(a.demos):
        o, ac, ok = replay(demos, meta, a.device, a.tail_s)
        K = meta.get("args", {}).get("passes", 1)
        print(f"[bc] level {meta['level']} passes {K}: {len(demos)} demonstrations, {o.shape[0]} (obs, action) pairs; "
              f"replay passes robustly: {ok}/{len(demos)}", flush=True)
        parts.append((o, ac, len(demos)))
    obs_d, act_d = torch.cat([x[0] for x in parts]), torch.cat([x[1] for x in parts])
    n_demos = sum(x[2] for x in parts)
    cfg = load_config(a.config, a.set + [f"device={a.device}"])
    env = TorchWindowEnv(cfg, num_envs=a.self_envs, device=a.device, seed=1)
    runner = OnPolicyRunner(env, runner_cfg(cfg), log_dir=None, device=a.device)
    runner.load(a.resume)
    actor = runner.alg.actor
    actor.eval()
    obs_s, act_s = own_states(env, actor, a.self_steps)
    print(f"[bc] {obs_s.shape[0]} own (obs, action) pairs [{time.time() - t0:.0f}s]", flush=True)
    td = lambda x: TensorDict({"policy": x}, batch_size=[x.shape[0]])  # noqa: E731
    with torch.no_grad():
        before = float(interval_mse(actor(td(obs_d)), act_d))
    opt = torch.optim.Adam(actor.mlp.parameters(), lr=a.lr)
    nd, ns = obs_d.shape[0], obs_s.shape[0]
    for ep in range(a.epochs):
        perm = torch.randperm(nd, device=a.device)
        tot_d = tot_s = 0.0
        for i in range(0, nd, a.batch):
            j = perm[i:i + a.batch]
            k = torch.randint(0, ns, (len(j) * 4,), device=a.device)
            with torch.enable_grad():
                ld = interval_mse(actor(td(obs_d[j])), act_d[j])
                ls = interval_mse(actor(td(obs_s[k])), act_s[k].clamp(-1.0, 1.0))
                loss = ld + a.self_weight * ls
                opt.zero_grad()
                loss.backward()
                opt.step()
            tot_d += ld.item() * len(j)
            tot_s += ls.item() * len(j)
        if ep % 5 == 0 or ep == a.epochs - 1:
            print(f"[bc] epoch {ep:3d} demo mse {tot_d / nd:.4f} (start {before:.4f}) own mse {tot_s / nd:.5f}", flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    runner.save(a.out, infos={"bc_warmstart": dict(resume=a.resume, demos=a.demos, n=n_demos, pairs=nd)})
    print(f"[bc] saved {a.out} [{time.time() - t0:.0f}s]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
