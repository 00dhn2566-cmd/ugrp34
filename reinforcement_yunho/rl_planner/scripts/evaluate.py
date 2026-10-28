"""Evaluate planners on one fixed, seeded scene set (README §7.4: RL vs baseline).

    # state machine baseline only (torch backend)
    python rl_planner/scripts/evaluate.py --planners fsm
    # RL policy vs state machine, Isaac backend, curriculum level 3, 512 scenes
    python rl_planner/scripts/evaluate.py --backend isaac --planners fsm,rl --policy <run>/export --level 3

Every planner runs the same scenes and the same sensor-noise draws (fresh env with
the same seed per planner). Outputs to --out (default <policy run>/eval or
/data/runs/rl_planner/eval/<stamp>): summary.json, episodes.csv, table.md.

With --policy every planner shares the policy's evaluation config, so the state machine also runs with the
policy's perception settings (e.g. perception.ray_forget 0.8 of the general policies, which hurts it —
RESULTS.md §10). scripts/benchmark.py gives each planner its own config on the same scenes.
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


class _IsaacAdapter:
    """Gives the wrapped Isaac env the (core, num_envs, device, step) shape eval_harness expects."""

    def __init__(self, wrapper, raw):
        self.wrapper, self.core = wrapper, raw.core
        self.num_envs, self.device = wrapper.num_envs, raw.device

    def step(self, act):
        return self.wrapper.step(act)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--backend", choices=("torch", "isaac"), default="torch")
    ap.add_argument("--planners", default="fsm,rl")
    ap.add_argument("--policy", default=None, help="export dir with policy.pt + policy_meta.json")
    ap.add_argument("--config", action="append", default=[])
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--num-scenes", type=int, default=None)
    ap.add_argument("--level", type=int, default=None)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--device", default=None)
    ap.add_argument("--record", type=int, default=8, help="save trajectories of the first N scenes")
    a = ap.parse_args(argv)

    from rl_planner.core.config import load_config
    planners = [p.strip() for p in a.planners.split(",") if p.strip()]   # "+safe" = behind SafetyFilter, "+layer" = convex layer
    if any(p.split("+")[0] == "rl" for p in planners) and not a.policy:
        raise SystemExit("--policy <export dir> is required for the rl planner")

    if a.policy:   # the config the policy was trained with (training-only augmentations off) + overrides
        from rl_planner.core.config import config_for_policy
        with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
            cfg = config_for_policy(json.load(f)["config"], a.config, a.set)
    else:
        cfg = load_config(a.config, a.set)
    if a.policy and any(p.split("+")[0] == "fsm" for p in planners) and float(cfg["perception"].get("ray_forget", 1.0)) != 1.0:
        print(f"[eval] note: the state machine runs with the policy's perception.ray_forget={cfg['perception']['ray_forget']} "
              "(its own baseline uses 1.0 — compare with scripts/benchmark.py)")
    ev = cfg["eval"]
    n = int(a.num_scenes or ev["num_scenes"])
    level = int(ev["level"] if a.level is None else a.level)
    seed = int(ev["seed"] if a.seed is None else a.seed)
    cfg["seed"] = seed
    cfg["sim"]["num_envs"] = n
    device = a.device or ("cuda:0" if cfg["device"] == "cuda" else "cpu")

    app = None
    if a.backend == "isaac":
        from isaaclab.app import AppLauncher
        app = AppLauncher(headless=True, device=device).app

    import torch

    from rl_planner.eval_harness import per_env_rows, run_planner, save_json, scene_fingerprint, summarize, write_csv
    from rl_planner.planners.fsm import FSMPlanner

    def make_env():
        if a.backend == "torch":
            from rl_planner.envs.torch_env import TorchWindowEnv
            env = TorchWindowEnv(cfg, num_envs=n, device=device, seed=seed)
            env.core.fixed_level = level
            env._reset_all()
            return env, env.close
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
        from rl_planner.envs.isaaclab_env import WindowMissionIsaacEnv, make_env_cfg
        raw = WindowMissionIsaacEnv(make_env_cfg(cfg, num_envs=n, device=device))
        raw.core.fixed_level = level
        raw._reset_idx(None)
        w = RslRlVecEnvWrapper(raw)
        return _IsaacAdapter(w, raw), w.close

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = a.out or (os.path.join(os.path.dirname(a.policy.rstrip("/")), f"eval_{a.backend}_L{level}_{stamp}") if a.policy
                    else os.path.join("/data/runs/rl_planner/eval", f"{stamp}_{a.backend}_L{level}"))
    summary, rows, scene_fp = {}, [], {}
    env, closer = make_env()          # Isaac allows one env per process: re-seed between planners instead
    for name in planners:
        env.core.reseed(seed)          # identical scenes + camera noise for every planner (eval_harness)
        if a.backend == "torch":
            env._reset_all()
        else:
            env.core.fixed_level = level
            env.wrapper.unwrapped._reset_idx(None)
        scene_fp[name] = scene_fingerprint(env.core.scene)
        if scene_fp[name] != scene_fp[planners[0]]:
            raise SystemExit(f"[eval] planners would see different scenes: {scene_fp}")
        base, _, guard = name.partition("+")
        if base == "fsm":
            planner = FSMPlanner(cfg, n, env.device)
        elif base == "rl":
            from rl_planner.planners.policy import PolicyPlanner
            planner = PolicyPlanner(a.policy, device=env.device, cfg=cfg)
        else:
            raise SystemExit(f"unknown planner {name!r}")
        guards = guard.split("+") if guard else []
        if set(guards) - {"safe", "layer"}:
            raise SystemExit(f"unknown planner suffix {guard!r} (+safe, +layer)")
        if "safe" in guards:
            from rl_planner.planners.safety import SafePlanner
            planner = SafePlanner(planner, cfg, n, env.device, env.core.step_dt)
        if "layer" in guards:                  # slung-load convex layer, last (planners/convex_layer.py)
            from rl_planner.planners.convex_layer import SlungLayerPlanner
            planner = SlungLayerPlanner(planner, cfg, n, env.device, env.core.step_dt)
        rec = run_planner(env, planner, record_k=a.record)
        if "traj" in rec:
            os.makedirs(out, exist_ok=True)
            torch.save(rec["traj"], os.path.join(out, f"traj_{name}.pt"))
        summary[name] = summarize(rec)
        rows += per_env_rows(name, rec)
        print(f"[eval] {name:4s} " + "  ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}"
                                                for k, v in summary[name].items()))
    closer()
    meta = {"backend": a.backend, "level": level, "seed": seed, "num_scenes": n, "policy": a.policy,
            "planners": planners, "scene_fingerprint": scene_fp[planners[0]]}
    save_json(os.path.join(out, "summary.json"), {"meta": meta, "summary": summary})
    write_csv(os.path.join(out, "episodes.csv"), rows)
    keys = ["success_rate", "crash_rate", "timeout_rate", "windows_passed_frac", "mean_time_success_s",
            "mean_clearance_m", "min_clearance_m", "est_err_at_pass_m", "crash_wall", "crash_order"]
    with open(os.path.join(out, "table.md"), "w", encoding="utf-8") as f:
        f.write(f"backend={a.backend} level={level} seed={seed} scenes={n}\n\n")
        f.write("| metric | " + " | ".join(planners) + " |\n|---|" + "---|" * len(planners) + "\n")
        for k in keys:
            f.write(f"| {k} | " + " | ".join(f"{summary[p][k]:.3f}" for p in planners) + " |\n")
    print(f"[eval] wrote {out}")
    if app is not None:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
