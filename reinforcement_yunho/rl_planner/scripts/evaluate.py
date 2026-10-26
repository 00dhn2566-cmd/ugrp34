"""Evaluate planners on one fixed, seeded scene set (README §7.4: RL vs baseline).

    # state machine baseline only (torch backend)
    python rl_planner/scripts/evaluate.py --planners fsm
    # RL policy vs state machine, Isaac backend, curriculum level 3, 512 scenes
    python rl_planner/scripts/evaluate.py --backend isaac --planners fsm,rl --policy <run>/export --level 3

Every planner runs the same scenes and the same sensor-noise draws (fresh env with
the same seed per planner). Outputs to --out (default <policy run>/eval or
/data/runs/rl_planner/eval/<stamp>): summary.json, episodes.csv, table.md.
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

    from rl_planner.core.config import Cfg, load_config
    planners = [p.strip() for p in a.planners.split(",") if p.strip()]   # "+safe" suffix = behind SafetyFilter
    if any(p.split("+")[0] == "rl" for p in planners) and not a.policy:
        raise SystemExit("--policy <export dir> is required for the rl planner")

    if a.policy:   # evaluate with the exact config the policy was trained with (+ explicit overrides)
        import yaml
        from rl_planner.core.config import DEFAULT_CONFIG, apply_override, deep_merge
        with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
            trained = json.load(f)["config"]
        with open(DEFAULT_CONFIG, encoding="utf-8") as f:
            base = yaml.safe_load(f)
        # training values win; keys added to default.yaml later (e.g. stress knobs) keep their defaults
        base = deep_merge(base, trained)
        for p in a.config:
            with open(p, encoding="utf-8") as f:
                base = deep_merge(base, yaml.safe_load(f) or {})
        for o in a.set:
            apply_override(base, o)
        cfg = Cfg(base)
    else:
        cfg = load_config(a.config, a.set)
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
        if guard == "safe":
            from rl_planner.planners.safety import SafePlanner
            planner = SafePlanner(planner, cfg, n, env.device, env.core.step_dt)
        elif guard:
            raise SystemExit(f"unknown planner suffix {guard!r} (only +safe)")
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
