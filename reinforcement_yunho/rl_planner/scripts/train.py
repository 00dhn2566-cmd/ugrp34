"""Train the RL window-mission planner with rsl_rl PPO (the Isaac Lab default runner).

    # Isaac Sim / Isaac Lab (PhysX) backend
    python rl_planner/scripts/train.py --backend isaac --num-envs 1024 --max-iterations 3000
    # pure torch backend (no Isaac; same MDP)
    python rl_planner/scripts/train.py --backend torch --num-envs 4096
    # experiments: extra yaml files and dotted overrides, all recorded in the run folder
    python rl_planner/scripts/train.py --config rl_planner/configs/exp_xxx.yaml --set reward.crash=-30

Run folder: <train.log_root>/<experiment>/<date>_<backend>_<cfghash>_<git>/
  config.yaml (merged), git.txt, model_*.pt, events.* (tensorboard), export/policy.pt
  export/policy.onnx, export/policy_meta.json
"""
from __future__ import annotations

import argparse
import datetime
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))          # reinforcement_yunho/
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--backend", choices=("torch", "isaac"), default=None)
    ap.add_argument("--config", action="append", default=[], help="extra yaml merged over default.yaml")
    ap.add_argument("--set", action="append", default=[], help="dotted override, e.g. reward.crash=-30")
    ap.add_argument("--num-envs", type=int, default=None)
    ap.add_argument("--max-iterations", type=int, default=None)
    ap.add_argument("--run-name", default="")
    ap.add_argument("--resume", default=None, help="path to a model_*.pt to continue from")
    ap.add_argument("--device", default=None)
    ap.add_argument("--no-export", action="store_true")
    return ap.parse_args(argv)


def git_info(path: str) -> str:
    try:
        h = subprocess.run(["git", "-C", path, "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", path, "status", "--porcelain"], capture_output=True, text=True).stdout.strip()
        return h + ("-dirty" if dirty else "") if h else "nogit"
    except Exception:
        return "nogit"


def runner_cfg(cfg) -> dict:
    t = cfg["train"]
    pol = t["policy"]
    alg = dict(t["algorithm_cfg"])
    return {
        "num_steps_per_env": int(t["num_steps_per_env"]),
        "save_interval": int(t["save_interval"]),
        "check_for_nan": True,
        "logger": "tensorboard",
        "obs_groups": {"actor": ["policy"], "critic": ["policy", "critic"]},
        "actor": {
            "class_name": "MLPModel",
            "hidden_dims": list(pol["actor_hidden_dims"]),
            "activation": pol["activation"],
            "obs_normalization": bool(t["empirical_normalization"]),
            "distribution_cfg": {"class_name": "GaussianDistribution", "init_std": float(pol["init_noise_std"]),
                                 "std_type": "scalar"},
            **({"class_name": "RNNModel", "rnn_type": pol["actor_type"], "rnn_hidden_dim": int(pol["rnn_hidden_dim"]),
                "rnn_num_layers": int(pol["rnn_num_layers"])} if pol.get("actor_type", "mlp") != "mlp" else {}),
        },
        "critic": {
            "class_name": "MLPModel",
            "hidden_dims": list(pol["critic_hidden_dims"]),
            "activation": pol["activation"],
            "obs_normalization": bool(t["empirical_normalization"]),
        },
        "algorithm": {"class_name": "PPO", "rnd_cfg": None, "symmetry_cfg": None, **alg},
        "multi_gpu": None,
    }


def export_recurrent_jit(actor, path: str) -> None:
    """TorchScript of an rsl_rl RNNModel actor as a pure function ``(obs (B,D), state (S,B,H)) -> (action, state)``.
    rsl_rl's own JIT export keeps a batch-1 hidden state inside the module; evaluation runs 512 envs at
    once and resets them one by one, so the state lives in the caller (PolicyPlanner) instead."""
    import torch

    from rl_planner.planners.policy import RecurrentExport
    mod = RecurrentExport(actor)
    mod.eval()
    torch.jit.script(mod).save(path)


def recurrent_meta(actor) -> dict:
    """``state_shape`` = (S, H): the caller keeps a zero-initialised (S, num_envs, H) state per env."""
    rnn = actor.rnn.rnn
    kind = "lstm" if type(rnn).__name__ == "LSTM" else "gru"
    return {"type": kind, "state_shape": [rnn.num_layers * (2 if kind == "lstm" else 1), rnn.hidden_size],
            "call": "action, state = policy(obs, state); state[:, done_env_ids] = 0 at every episode start"}


def make_env(cfg, backend: str, num_envs: int, device: str):
    """Returns (rsl_rl VecEnv, core, closer). For isaac the Kit app must already be running."""
    if backend == "torch":
        from rl_planner.envs.torch_env import TorchWindowEnv
        env = TorchWindowEnv(cfg, num_envs=num_envs, device=device)
        return env, env.core, env.close
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from rl_planner.envs.isaaclab_env import WindowMissionIsaacEnv, make_env_cfg
    env_cfg = make_env_cfg(cfg, num_envs=num_envs, device=device)
    raw = WindowMissionIsaacEnv(env_cfg)
    env = RslRlVecEnvWrapper(raw)
    return env, raw.core, env.close


def main(argv=None) -> int:
    a = parse_args(argv)
    from rl_planner.core.config import config_hash, load_config, save_config
    overrides = list(a.set)
    cfg = load_config(a.config, overrides)
    backend = a.backend or cfg["sim"]["backend"]
    cfg["sim"]["backend"] = backend
    if a.num_envs:
        cfg["sim"]["num_envs"] = a.num_envs
    if a.max_iterations:
        cfg["train"]["max_iterations"] = a.max_iterations
    device = a.device or ("cuda:0" if cfg["device"] == "cuda" else "cpu")

    app = None
    if backend == "isaac":
        from isaaclab.app import AppLauncher
        app = AppLauncher(headless=True, device=device).app

    import torch
    from rsl_rl.runners import OnPolicyRunner

    from rl_planner.planners.policy import JIT_NAME, META_NAME, build_meta

    torch.manual_seed(int(cfg["seed"]))
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    git = git_info(ROOT)
    name = f"{stamp}_{backend}_{config_hash(cfg)}_{git}" + (f"_{a.run_name}" if a.run_name else "")
    run_dir = os.path.join(cfg["train"]["log_root"], cfg["train"]["experiment_name"], name)
    os.makedirs(run_dir, exist_ok=True)
    save_config(cfg, os.path.join(run_dir, "config.yaml"))
    with open(os.path.join(run_dir, "git.txt"), "w") as f:
        f.write(f"commit {git}\nargv {' '.join(sys.argv)}\n")
    print(f"[train] run dir: {run_dir}")

    env, core, close = make_env(cfg, backend, int(cfg["sim"]["num_envs"]), device)
    print(f"[train] backend={backend} envs={env.num_envs} obs(policy)={core.num_policy_obs} "
          f"obs(critic)={core.num_critic_obs} actions={core.action_dim} decision_dt={core.step_dt}s")
    rcfg = runner_cfg(cfg)
    runner = OnPolicyRunner(env, rcfg, log_dir=run_dir, device=device)
    if a.resume:
        runner.load(a.resume)
        print(f"[train] resumed from {a.resume}")
    runner.learn(num_learning_iterations=int(cfg["train"]["max_iterations"]), init_at_random_ep_len=True)

    if not a.no_export:
        exp = os.path.join(run_dir, "export")
        os.makedirs(exp, exist_ok=True)
        actor = runner.alg.get_policy()
        if actor.is_recurrent:                 # batched export with explicit state (planners/policy.py)
            export_recurrent_jit(actor, os.path.join(exp, JIT_NAME))
        else:
            runner.export_policy_to_jit(exp, filename=JIT_NAME)
        try:
            runner.export_policy_to_onnx(exp, filename="policy.onnx")
        except Exception as e:  # noqa: BLE001 — ONNX is a convenience copy
            print(f"[train] onnx export skipped: {e!r}")
        import json
        meta = build_meta(cfg, core, {"run_dir": run_dir, "git": git, "backend": backend,
                                      "iterations": int(cfg["train"]["max_iterations"])})
        if actor.is_recurrent:
            meta["recurrent"] = recurrent_meta(actor)
        with open(os.path.join(exp, META_NAME), "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)
        print(f"[train] exported policy to {exp}")
    close()
    if app is not None:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
