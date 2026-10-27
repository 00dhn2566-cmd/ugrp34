"""Warm-start a trained checkpoint on a larger action scale (action.step_max) without changing its behaviour.

    python rl_planner/scripts/rescale_action.py --run <run_dir> [--ckpt model_3997.pt] --step-max 1.25 0.9 0.5 --out new.pt

An action a in [-1, 1] commands the waypoint displacement a * step_max, and the Lee controller turns a waypoint
0.8 m ahead into kp_pos * 0.8 = 1.28 m/s, so step_max caps the speed (RESULTS.md §11). With a larger step_max the
same displacement needs the smaller action a * old/new: the actor's output layer, its exploration std and their
Adam moments are scaled by s = old/new per action, and the previous action in the observation (``last_action``,
now in the new units) is re-normalised (mean x s, std so that (x - mean)/(std + eps) is unchanged) in the actor and
the critic. The rescaled policy commands exactly the old displacement wherever the old action was inside [-1, 1].
Where the old policy saturated (asked for more than step_max, which a speed-capped policy does all the time) it
now gets more, up to the new step: the main policy rescaled 0.8 -> 1.25 m flew nominal L5 in 7.1 s instead of 10.4 s
before any training, but lost strong-wind robustness, which the fine-tune has to bring back. Train with the printed
``--set action.step_max=[...]`` and ``--resume new.pt``.
"""
from __future__ import annotations

import argparse
import copy
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import torch  # noqa: E402

from rl_planner.scripts.expand_obs import layouts  # noqa: E402

NORM_EPS = 1e-2          # rsl_rl EmpiricalNormalization: (x - mean) / (std + eps)


def _renormalise(sd: dict, sl: slice, s: torch.Tensor) -> None:
    """Inputs x[sl] become s * x: keep (x - mean) / (std + eps) exactly."""
    if "obs_normalizer._mean" not in sd:
        return
    mean, std = sd["obs_normalizer._mean"].clone(), sd["obs_normalizer._std"].clone()
    mean[..., sl] = mean[..., sl] * s
    std[..., sl] = s * (std[..., sl] + NORM_EPS) - NORM_EPS
    if bool((std[..., sl] <= 0).any()):
        raise ValueError("an input std is too small to rescale exactly (std < eps/s - eps)")
    sd["obs_normalizer._mean"], sd["obs_normalizer._std"] = mean, std
    if "obs_normalizer._var" in sd:
        sd["obs_normalizer._var"] = std ** 2


def rescale_checkpoint(ck: dict, cfg_old, step_new) -> dict:
    step_old = torch.tensor([float(v) for v in cfg_old["action"]["step_max"]])
    step_new = torch.tensor([float(v) for v in step_new])
    if step_old.shape != step_new.shape:
        raise ValueError(f"step_max has {len(step_old)} entries, got {len(step_new)}")
    s = step_old / step_new
    A = len(s)
    out = copy.copy(ck)
    actor = dict(ck["actor_state_dict"])
    # output layer = the last 2-D weight with A rows (MLP head of the MLP or the recurrent model)
    heads = [k for k, v in actor.items() if k.endswith(".weight") and v.dim() == 2 and v.shape[0] == A]
    if not heads:
        raise ValueError("no output layer with one row per action found")
    w = heads[-1]
    b = w[:-len("weight")] + "bias"
    actor[w] = actor[w] * s[:, None]
    actor[b] = actor[b] * s
    if "distribution.std_param" in actor:
        actor["distribution.std_param"] = actor["distribution.std_param"] * s
    elif "distribution.log_std_param" in actor:
        actor["distribution.log_std_param"] = actor["distribution.log_std_param"] + torch.log(s)
    else:
        raise ValueError("no std parameter in the actor")
    critic = dict(ck["critic_state_dict"])
    a_lay, c_lay = layouts(cfg_old)
    for sd, lay in ((actor, a_lay), (critic, c_lay)):
        for name, lo, hi in lay:
            if name == "last_action":
                if hi - lo != A:
                    raise ValueError(f"last_action has {hi - lo} entries, the action {A}")
                _renormalise(sd, slice(lo, hi), s)
    out["actor_state_dict"], out["critic_state_dict"] = actor, critic
    opt = ck.get("optimizer_state_dict")
    if opt:                                    # Adam moments follow the parameters: grad scales by 1/s
        names = [k for k, v in ck["actor_state_dict"].items() if not k.startswith("obs_normalizer")]
        opt = copy.deepcopy(opt)
        order = [i for g in opt["param_groups"] for i in g["params"]]
        for idx, name in zip(order, names):
            st = opt["state"].get(idx)
            if not st or name not in (w, b, "distribution.std_param"):
                continue
            if tuple(st["exp_avg"].shape) != tuple(ck["actor_state_dict"][name].shape):
                raise ValueError(f"optimizer state {idx} does not match {name}")
            g = (1.0 / s)[:, None] if name == w else 1.0 / s
            st["exp_avg"] = st["exp_avg"] * g
            st["exp_avg_sq"] = st["exp_avg_sq"] * g ** 2
        out["optimizer_state_dict"] = opt
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run", required=True, help="training run dir (config.yaml + model_*.pt)")
    ap.add_argument("--ckpt", default=None, help="checkpoint file name in --run (default: the latest)")
    ap.add_argument("--step-max", type=float, nargs="+", required=True, help="new action.step_max [m]")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    import glob

    import yaml
    with open(os.path.join(a.run, "config.yaml"), encoding="utf-8") as f:
        cfg_old = yaml.safe_load(f)
    if a.ckpt:
        path = os.path.join(a.run, a.ckpt)
    else:
        path = max(glob.glob(os.path.join(a.run, "model_*.pt")), key=lambda p: int(p.rsplit("_", 1)[1][:-3]))
    ck = torch.load(path, map_location="cpu", weights_only=False)
    out = rescale_checkpoint(ck, cfg_old, a.step_max)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    torch.save(out, a.out)
    print(f"{path} -> {a.out}: step_max {cfg_old['action']['step_max']} -> {a.step_max}")
    print("train with:  --resume", a.out, f'--set "action.step_max=[{",".join(str(v) for v in a.step_max)}]"')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
