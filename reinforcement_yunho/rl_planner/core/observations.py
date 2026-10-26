"""Observation terms (registry). The YAML list order IS the vector layout.

Each term is ``fn(core) -> (E, dim)`` plus a fixed ``dim``. ``build_spec`` records
names/slices so an exported policy carries its own layout (policy_meta.json),
and the deployment adapter (planners/policy_planner.py) rebuilds the same vector
from real interface messages.

Policy terms use only what the real stack provides (VIO pose/velocity, IMU,
detector output, triangulated window estimates, own pass count). Critic terms
(asymmetric actor-critic) may use ground truth — they are dropped at export.
"""
from __future__ import annotations

from typing import Callable, Dict, List, Mapping, Tuple

import torch

from . import math3d as m3

POS_SCALE = 5.0      # [m] relative positions are divided by this
VEL_SCALE = 2.0
ANG_SCALE = 3.0


class Term:
    def __init__(self, fn: Callable, dim: Callable[[Mapping], int], privileged: bool):
        self.fn, self.dim, self.privileged = fn, dim, privileged


REGISTRY: Dict[str, Term] = {}


def register(name: str, dim, privileged: bool = False):
    def deco(fn):
        REGISTRY[name] = Term(fn, dim if callable(dim) else (lambda cfg, d=dim: d), privileged)
        return fn
    return deco


def _heading(core, v_w):
    return m3.world_to_heading(v_w, core.yaw)


def _noise(core, x, key):
    s = float(core.cfg["observation"]["noise"].get(key, 0.0))
    if s <= 0 or not core.obs_noise_enabled:
        return x
    return x + torch.randn(x.shape, generator=core.gen, device=x.device) * s


def _gather(t: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
    """t (E, W, ...) , idx (E,) -> (E, ...)"""
    shape = (t.shape[0], 1) + tuple(t.shape[2:])
    return t.gather(1, idx.view(-1, *([1] * (t.dim() - 1))).expand(shape)).squeeze(1)


# ----------------------------------------------------------------------------- policy
@register("lin_vel_heading", 3)
def lin_vel_heading(core):
    return _noise(core, _heading(core, core.state.vel_w) / VEL_SCALE, "lin_vel")


@register("gravity_body", 3)
def gravity_body(core):
    g = torch.tensor((0.0, 0.0, -1.0), device=core.device).expand(core.E, 3)
    return _noise(core, m3.quat_rotate_inverse(core.state.quat, g), "gravity")


@register("ang_vel_body", 3)
def ang_vel_body(core):
    return _noise(core, core.state.omega_b / ANG_SCALE, "ang_vel")


@register("altitude", 1)
def altitude(core):
    return _noise(core, (core.state.pos[:, 2:3] - 1.2), "altitude")


@register("last_action", lambda cfg: 4 if cfg["action"]["yaw"] else 3)
def last_action(core):
    return core.actions


def _estimate_block(core, idx, exists):
    est = core.est
    valid = _gather(est.valid, idx) & exists
    rel = _heading(core, _gather(est.center, idx) - core.state.pos) / POS_SCALE
    nrm = _heading(core, _gather(est.normal, idx))
    size = _gather(est.size, idx) - 1.0
    sig = torch.log10(_gather(est.sigma, idx).clamp(1e-3, 10.0)) / 3.0      # [-1, 0.33]
    par = _gather(est.parallax_deg, idx) / 20.0
    nob = (_gather(est.n_obs, idx) / 30.0).clamp_max(2.0)
    v = valid.float()[:, None]
    return valid, v, rel * v, nrm * v, size * v, sig[:, None] * v, par[:, None] * v, nob[:, None]


@register("target_estimate", 12)
def target_estimate(core):
    exists = torch.ones(core.E, dtype=torch.bool, device=core.device)
    _, v, rel, nrm, size, sig, par, nob = _estimate_block(core, core.target_idx, exists)
    return torch.cat((v, rel, nrm, size, sig, par, nob), -1)


@register("target_detection", 6)
def target_detection(core):
    pr = core.perception
    idx = core.target_idx
    det = _gather(core.det.detected, idx).float()[:, None]
    age = (_gather(pr.det_age, idx) / 20.0).clamp_max(1.0)[:, None]
    seen = (_gather(pr.det_age, idx) < 1e3).float()[:, None]
    bearing = _heading(core, _gather(pr.last_bearing, idx)) * seen
    side = (_gather(pr.last_side, idx) / 200.0).clamp_max(2.0)[:, None] * seen
    return torch.cat((det, age, bearing, side), -1)


@register("next_estimate", 7)
def next_estimate(core):
    nxt = (core.target_idx + 1).clamp_max(core.W - 1)
    exists = (core.target_idx + 1) < core.scene.n_windows
    _, v, rel, nrm, *_ = _estimate_block(core, nxt, exists)
    return torch.cat((v, rel, nrm), -1)


@register("progress", 3)
def progress(core):
    n = core.scene.n_windows.float()
    passed = core.mission.target.float()
    return torch.stack((passed / n, (n - passed) / 5.0, core.episode_time / core.episode_length_s), -1)


# ----------------------------------------------------------------------------- critic
@register("target_gt", 8, privileged=True)
def target_gt(core):
    sc, idx = core.scene, core.target_idx
    rel = _heading(core, _gather(sc.center, idx) - core.state.pos) / POS_SCALE
    return torch.cat((rel, _heading(core, _gather(sc.normal, idx)), _gather(sc.size, idx) - 1.0), -1)


@register("next_gt", 4, privileged=True)
def next_gt(core):
    sc = core.scene
    nxt = (core.target_idx + 1).clamp_max(core.W - 1)
    exists = ((core.target_idx + 1) < sc.n_windows).float()[:, None]
    rel = _heading(core, _gather(sc.center, nxt) - core.state.pos) / POS_SCALE
    return torch.cat((rel * exists, exists), -1)


@register("estimate_error", 4, privileged=True)
def estimate_error(core):
    err = _gather(core.est_err, core.target_idx)[:, None]
    rel_start = (core.state.pos - core.scene.start) / 10.0
    return torch.cat((err, rel_start), -1)


# ----------------------------------------------------------------------------- layout
def build_spec(cfg: Mapping, group: str) -> List[Tuple[str, int, int]]:
    """[(name, start, end)] for an observation group."""
    names = list(cfg["observation"][group])
    spec, i = [], 0
    for n in names:
        if n not in REGISTRY:
            raise KeyError(f"unknown observation term {n!r}; registered: {sorted(REGISTRY)}")
        if group == "policy" and REGISTRY[n].privileged:
            raise ValueError(f"privileged term {n!r} cannot be in the policy group")
        d = REGISTRY[n].dim(cfg)
        spec.append((n, i, i + d))
        i += d
    return spec


def compute_group(core, spec) -> torch.Tensor:
    return torch.cat([REGISTRY[n].fn(core) for n, _, _ in spec], dim=-1)
