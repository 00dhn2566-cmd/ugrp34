"""Reward terms (registry). Weights come ONLY from ``cfg.reward`` (README §7.2).

Each term returns an un-weighted (E,) tensor; the env multiplies by the weight and
logs every weighted term per episode, so reward hacking shows up in the curves.
A term with weight 0 is skipped. Add a term: decorate a function with
``@register("name")`` and add ``name: <weight>`` to the yaml.
"""
from __future__ import annotations

from typing import Callable, Dict, Mapping

import torch

from .observations import _gather

REGISTRY: Dict[str, Callable] = {}
NON_TERM_KEYS = {"estimate_error_cap", "load_clearance_range"}


def register(name: str):
    def deco(fn):
        REGISTRY[name] = fn
        return fn
    return deco


@register("progress")
def progress(core):
    """Decrease of distance to the current target window centre (GT) [m/step]."""
    return core.progress_delta


@register("window_pass")
def window_pass(core):
    return core.mission.step_pass.any(-1).float()


@register("pass_clearance")
def pass_clearance(core):
    """On a pass: slack / max-possible-slack in [0, 1] (1 = straight through the middle)."""
    m = core.mission
    hit = m.step_pass
    if not hit.any():
        return torch.zeros(core.E, device=core.device)
    half = core.scene.size.min(dim=-1).values / 2 - m.r
    frac = (m.clearance / half.clamp_min(1e-3)).clamp(0.0, 1.0)
    return (frac * hit.float()).sum(-1)


@register("mission_success")
def mission_success(core):
    return (core.mission.success & ~core.success_rewarded).float()


@register("crash")
def crash(core):
    return (core.mission.crashed & ~core.crash_rewarded).float()


@register("perception")
def perception(core):
    """exp(-delta^4), delta = angle between optical axis (body +x) and the target (Swift)."""
    sc = core.scene
    to_t = _gather(sc.center, core.target_idx) - core.state.pos
    to_t = to_t / to_t.norm(dim=-1, keepdim=True).clamp_min(1e-6)
    axis = core.body_x_w
    delta = torch.acos((to_t * axis).sum(-1).clamp(-1.0, 1.0))
    alive = ~core.mission.success
    return torch.exp(-delta.pow(4)) * alive.float()


@register("estimate_improve")
def estimate_improve(core):
    """Potential-based (Ng et al. 1999): Phi = -min(err, cap) of the target estimate."""
    return core.est_potential_delta


@register("time")
def time(core):
    return torch.ones(core.E, device=core.device)


@register("action_rate")
def action_rate(core):
    return (core.actions - core.prev_actions).pow(2).sum(-1)


@register("action_excess")
def action_excess(core):
    """Squared excess of the policy's raw action over the [-1, 1] the env flies (it is clipped): a mean parked far
    outside the box no longer explores — every sample clips to the same bound (rl_games' bounds loss, as a reward).
    Costs nothing inside the box, so it removes no behaviour."""
    return (core.raw_actions.abs() - 1.0).clamp_min(0.0).pow(2).sum(-1)


@register("ang_vel")
def ang_vel(core):
    return core.state.omega_b.pow(2).sum(-1)


@register("load_swing")
def load_swing(core):
    """Horizontal load offset from the hook / cable length, squared (core/slung.py): a calm pendulum."""
    st = core.state
    hook, _ = core.slung.hook(st.pos, st.quat)
    return ((core.slung.pos - hook)[:, :2] / core.slung.L).pow(2).sum(-1)


@register("load_clearance")
def load_clearance(core):
    """Slung load: the room it had in the opening when it first reached a wall (core/mission.py; < 0 = by how much
    it missed the sill or lintel), mapped to [0, 1] over reward.load_clearance_range [m] — graded credit for nearly
    clearing that crash and success alone do not give (a cable as long as the window is tall, RESULTS.md §15)."""
    m = core.mission
    lo, hi = (float(x) for x in core.cfg["reward"]["load_clearance_range"])
    val = ((m.load_clear - lo) / (hi - lo)).clamp(0.0, 1.0)
    return (val * m.step_load_arrive.float()).sum(-1)


@register("approach_turn")
def approach_turn(core):
    """|yaw rate|^2 [rad^2/s^2] while the chosen window (core/selector.py) is in view: look around while searching,
    then fly to the window without swinging the camera off it."""
    sel = core.selector
    in_view = core.det.detected.gather(1, sel.sel[:, None]).squeeze(1) & sel.has
    return core.state.omega_b[:, 2].pow(2) * in_view.float()


def active_terms(cfg: Mapping) -> Dict[str, float]:
    out = {}
    for k, w in cfg["reward"].items():
        if k in NON_TERM_KEYS:
            continue
        if k not in REGISTRY:
            raise KeyError(f"unknown reward term {k!r}; registered: {sorted(REGISTRY)}")
        if float(w) != 0.0:
            out[k] = float(w)
    return out
