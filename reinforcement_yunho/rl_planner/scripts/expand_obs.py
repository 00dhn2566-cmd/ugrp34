"""Warm-start a trained checkpoint on a longer observation list (add terms without retraining).

    python rl_planner/scripts/expand_obs.py --run <run_dir> [--ckpt model_1999.pt] --add detection_gap --out new.pt

New policy terms (``--add``) go to the end of the policy group, new critic terms (``--add-critic``)
to the end of the critic group. They get zero input weights, so the expanded policy acts exactly
like the old one until training moves them; observation normaliser mean 0 / var 1 (the defaults)
and zero Adam moments. Every existing term keeps its weights wherever it lands in the new layout.
Then train with the printed ``--set observation.policy=[...]`` and ``--resume new.pt``.

``--yaw`` adds the yaw-rate action (action.yaw): a fourth actor output with zero weights and bias (so the
policy still holds its heading until training moves it), exploration std ``--yaw-std``, and the matching
fourth ``last_action`` input (zero weights). RESULTS.md §12.
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

from rl_planner.core.observations import build_spec  # noqa: E402


def layouts(cfg) -> tuple:
    """Input layouts [(name, start, end)] of the actor (policy group) and the critic (policy + critic)."""
    pol = build_spec(cfg, "policy")
    n = pol[-1][2] if pol else 0
    crit = [(f"critic:{k}", s + n, e + n) for k, s, e in build_spec(cfg, "critic")]
    return pol, pol + crit


GROWS = {"last_action", "critic:last_action"}     # terms that may gain trailing entries (the yaw action)


def _widen(t: torch.Tensor, old, new, fill: float) -> torch.Tensor:
    """Re-lay the last dimension of ``t`` from layout ``old`` to ``new``; new terms (and new trailing entries of a
    term in GROWS) get ``fill``."""
    out = torch.full((*t.shape[:-1], new[-1][2]), fill, dtype=t.dtype, device=t.device)
    src = {k: (s, e) for k, s, e in old}
    for k, s, e in new:
        if k in src:
            a, b = src[k]
            if b - a != e - s and not (k in GROWS and e - s > b - a):
                raise ValueError(f"term {k!r} changed size {b - a} -> {e - s}")
            out[..., s:s + (b - a)] = t[..., a:b]
    return out


def grow_actions(ck: dict, new_dim: int, init_std: float) -> dict:
    """Append actor outputs (zero weights / bias, exploration std ``init_std``) up to ``new_dim``, with zero Adam
    moments for the new entries."""
    sd = dict(ck["actor_state_dict"])
    last = max(int(k.split(".")[1]) for k in sd if k.startswith("mlp.") and k.endswith(".weight"))
    wk, bk = f"mlp.{last}.weight", f"mlp.{last}.bias"
    old_dim = sd[wk].shape[0]
    if new_dim <= old_dim:
        return ck
    add = new_dim - old_dim
    shapes = {}
    for k in (wk, bk, "distribution.std_param", "distribution.log_std_param"):
        if k in sd:
            shapes[tuple(sd[k].shape)] = add
    sd[wk] = torch.cat((sd[wk], torch.zeros(add, sd[wk].shape[1], dtype=sd[wk].dtype)), 0)
    sd[bk] = torch.cat((sd[bk], torch.zeros(add, dtype=sd[bk].dtype)), 0)
    if "distribution.std_param" in sd:
        sd["distribution.std_param"] = torch.cat((sd["distribution.std_param"], torch.full((add,), init_std)), 0)
    if "distribution.log_std_param" in sd:
        sd["distribution.log_std_param"] = torch.cat((sd["distribution.log_std_param"],
                                                      torch.full((add,), float(torch.log(torch.tensor(init_std))))), 0)
    out = copy.copy(ck)
    out["actor_state_dict"] = sd
    opt = ck.get("optimizer_state_dict")
    if opt:                                    # moments of the output layer and the std, found by shape
        opt = copy.deepcopy(opt)
        for st in opt["state"].values():
            if "exp_avg" in st and tuple(st["exp_avg"].shape) in shapes and st["exp_avg"].shape[0] == old_dim:
                for m in ("exp_avg", "exp_avg_sq"):
                    st[m] = torch.cat((st[m], torch.zeros(add, *st[m].shape[1:], dtype=st[m].dtype)), 0)
        out["optimizer_state_dict"] = opt
    return out


def expand_state(sd: dict, old, new) -> dict:
    """An actor/critic state_dict (rsl_rl MLPModel + EmpiricalNormalization) on the new input layout."""
    sd = dict(sd)
    sd["mlp.0.weight"] = _widen(sd["mlp.0.weight"], old, new, 0.0)
    for k, fill in (("obs_normalizer._mean", 0.0), ("obs_normalizer._var", 1.0), ("obs_normalizer._std", 1.0)):
        if k in sd:
            sd[k] = _widen(sd[k], old, new, fill)
    return sd


def expand_checkpoint(ck: dict, cfg_old, cfg_new) -> dict:
    (a_old, c_old), (a_new, c_new) = layouts(cfg_old), layouts(cfg_new)
    out = copy.copy(ck)
    out["actor_state_dict"] = expand_state(ck["actor_state_dict"], a_old, a_new)
    out["critic_state_dict"] = expand_state(ck["critic_state_dict"], c_old, c_new)
    opt = ck.get("optimizer_state_dict")
    if opt:                                    # Adam moments of the two first layers, found by shape
        shapes = {tuple(ck["actor_state_dict"]["mlp.0.weight"].shape): (a_old, a_new),
                  tuple(ck["critic_state_dict"]["mlp.0.weight"].shape): (c_old, c_new)}
        if len(shapes) != 2:
            raise ValueError("actor and critic first layers have the same shape; cannot tell their moments apart")
        opt = copy.deepcopy(opt)
        for st in opt["state"].values():
            lay = shapes.get(tuple(st["exp_avg"].shape)) if "exp_avg" in st else None
            if lay:
                for m in ("exp_avg", "exp_avg_sq"):
                    st[m] = _widen(st[m], *lay, 0.0)
        out["optimizer_state_dict"] = opt
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run", required=True, help="training run dir (config.yaml + model_*.pt)")
    ap.add_argument("--ckpt", default=None, help="checkpoint file name in --run (default: the latest)")
    ap.add_argument("--add", action="append", default=[], help="policy observation term to append")
    ap.add_argument("--add-critic", action="append", default=[], help="critic observation term to append")
    ap.add_argument("--yaw", action="store_true", help="add the yaw-rate action (action.yaw: true)")
    ap.add_argument("--yaw-std", type=float, default=0.5, help="exploration std of the new yaw output")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    import glob

    import yaml
    with open(os.path.join(a.run, "config.yaml"), encoding="utf-8") as f:
        cfg_old = yaml.safe_load(f)
    cfg_new = copy.deepcopy(cfg_old)
    cfg_new["observation"]["policy"] = list(cfg_old["observation"]["policy"]) + a.add
    cfg_new["observation"]["critic"] = list(cfg_old["observation"]["critic"] or []) + a.add_critic
    if a.yaw:
        cfg_new["action"]["yaw"] = True
    if a.ckpt:
        path = os.path.join(a.run, a.ckpt)
    else:
        path = max(glob.glob(os.path.join(a.run, "model_*.pt")), key=lambda p: int(p.rsplit("_", 1)[1][:-3]))
    ck = torch.load(path, map_location="cpu", weights_only=False)
    out = expand_checkpoint(ck, cfg_old, cfg_new)
    if a.yaw:
        out = grow_actions(out, 4, a.yaw_std)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    torch.save(out, a.out)
    a_new, c_new = layouts(cfg_new)
    print(f"{path} -> {a.out}: actor inputs {layouts(cfg_old)[0][-1][2]} -> {a_new[-1][2]}, "
          f"critic {layouts(cfg_old)[1][-1][2]} -> {c_new[-1][2]}")
    print("train with:  --resume", a.out, f'--set "observation.policy=[{",".join(cfg_new["observation"]["policy"])}]"'
          + (f' --set "observation.critic=[{",".join(cfg_new["observation"]["critic"])}]"' if a.add_critic else "")
          + (" --set action.yaw=true" if a.yaw else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
