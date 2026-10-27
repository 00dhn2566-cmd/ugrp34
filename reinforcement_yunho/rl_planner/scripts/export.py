"""Export any checkpoint of a training run (policy.pt + policy.onnx + policy_meta.json), e.g. a mid-training one.

    python rl_planner/scripts/export.py --run <run_dir> --ckpt model_2900.pt --out <run_dir>/export_2900

train.py exports only the final model; this rebuilds the same runner (on CPU, a few envs) from the run's
config.yaml, loads the checkpoint and writes the same three files, so evaluate.py / benchmark.py /
OnboardPlanner can use it.
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
    ap.add_argument("--run", required=True)
    ap.add_argument("--ckpt", required=True, help="checkpoint file name in --run, e.g. model_2900.pt")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    import yaml
    from rsl_rl.runners import OnPolicyRunner

    from rl_planner.core.config import Cfg
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.policy import JIT_NAME, META_NAME, build_meta
    from rl_planner.scripts.train import export_recurrent_jit, recurrent_meta, runner_cfg

    with open(os.path.join(a.run, "config.yaml"), encoding="utf-8") as f:
        cfg = Cfg(yaml.safe_load(f))
    cfg["device"] = "cpu"
    env = TorchWindowEnv(cfg, num_envs=4, device="cpu")
    runner = OnPolicyRunner(env, runner_cfg(cfg), log_dir=None, device="cpu")
    path = os.path.join(a.run, a.ckpt)
    runner.load(path, map_location="cpu")
    os.makedirs(a.out, exist_ok=True)
    actor = runner.alg.get_policy()
    if actor.is_recurrent:
        export_recurrent_jit(actor, os.path.join(a.out, JIT_NAME))
    else:
        runner.export_policy_to_jit(a.out, filename=JIT_NAME)
    try:
        runner.export_policy_to_onnx(a.out, filename="policy.onnx")
    except Exception as e:  # noqa: BLE001 — ONNX is a convenience copy
        print(f"[export] onnx skipped: {e!r}")
    with open(os.path.join(a.run, "git.txt"), encoding="utf-8") as f:
        git = f.readline().split()[-1]
    meta = build_meta(cfg, env.core, {"run_dir": a.run, "git": git, "backend": cfg["sim"]["backend"],
                                      "iterations": int(a.ckpt.rsplit("_", 1)[1][:-3]), "checkpoint": a.ckpt})
    if actor.is_recurrent:
        meta["recurrent"] = recurrent_meta(actor)
    with open(os.path.join(a.out, META_NAME), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print(f"[export] {path} -> {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
