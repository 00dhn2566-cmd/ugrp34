"""Runtime safety filter around any planner (perception-loss hold + geofence).

Neither planner was trained or tuned for a camera that sees nothing: with every
detection suppressed the state machine's SEARCH flies straight into the first
wall and the RL policy backs out of the arena (RESULTS.md). This filter is the
standard runtime guard for a learned planner and changes nothing while the
mission is nominal:

* hold   — no window detected for ``lost_hold_s`` (or none yet this episode) AND
           no valid estimate of the target window: latch the current position
           and command it until the next detection, then hand control back.
* fence  — clamp every waypoint into the arena box (scene.bounds shrunk by
           ``fence_margin_m``).

It reads only what the real stack has (detections, own pose, own estimates), so
the same class runs in simulation (:class:`SafePlanner`) and on board
(planners/onboard.py).
"""
from __future__ import annotations

from typing import Mapping

import torch

from ..core import math3d as m3


class SafetyFilter:
    def __init__(self, cfg: Mapping, num_envs: int, device, step_dt: float):
        s = dict(cfg.get("safety") or {})
        self.enabled = bool(s.get("enabled", True))
        self.lost_steps = max(1, int(round(float(s.get("lost_hold_s", 1.0)) / float(step_dt))))
        b, m = s.get("bounds") or cfg["scene"]["bounds"], float(s.get("fence_margin_m", 0.5))
        self.lo = torch.tensor([float(b["x_min"]) + m, -float(b["y_abs"]) + m, float(b["z_min"]) + m], device=device)
        self.hi = torch.tensor([float("inf"), float(b["y_abs"]) - m, float(b["z_max"]) - m], device=device)
        E = int(num_envs)
        self.since_seen = torch.full((E,), self.lost_steps + 1, dtype=torch.long, device=device)
        self.holding = torch.zeros(E, dtype=torch.bool, device=device)
        self.hold_pos = torch.zeros(E, 3, device=device)
        self.hold_steps = torch.zeros(E, dtype=torch.long, device=device)   # per episode, for logging

    def reset(self, env_ids) -> None:
        self.since_seen[env_ids] = self.lost_steps + 1      # hold until the first detection
        self.holding[env_ids] = False
        self.hold_steps[env_ids] = 0

    @torch.no_grad()
    def filter(self, actions: torch.Tensor, seen: torch.Tensor, target_valid: torch.Tensor,
               pos: torch.Tensor, yaw: torch.Tensor, step_max: torch.Tensor) -> torch.Tensor:
        """actions (E,A) in [-1,1]; seen (E,) any window detected since the last decision;
        target_valid (E,) the current target window has a valid estimate."""
        self.since_seen = torch.where(seen, torch.zeros_like(self.since_seen), self.since_seen + 1)
        if not self.enabled:
            return actions
        hold = (self.since_seen > self.lost_steps) & ~target_valid
        self.hold_pos = torch.where((hold & ~self.holding)[:, None], pos, self.hold_pos)
        self.holding = hold
        self.hold_steps += hold.long()
        wp = pos + m3.heading_to_world(actions[:, :3] * step_max, yaw)
        wp = torch.where(hold[:, None], self.hold_pos, wp)
        wp = torch.maximum(torch.minimum(wp, self.hi), self.lo)
        out = actions.clone()
        out[:, :3] = (m3.world_to_heading(wp - pos, yaw) / step_max).clamp(-1.0, 1.0)
        if out.shape[1] > 3:
            out[:, 3] = torch.where(hold, torch.zeros_like(out[:, 3]), out[:, 3])
        return out


class SafePlanner:
    """A simulation planner (``reset(ids)`` / ``act(core)``) behind :class:`SafetyFilter`."""

    def __init__(self, base, cfg: Mapping, num_envs: int, device, step_dt: float):
        self.base = base
        self.safety = SafetyFilter(cfg, num_envs, device, step_dt)

    def reset(self, env_ids) -> None:
        self.base.reset(env_ids)
        self.safety.reset(env_ids)

    @torch.no_grad()
    def act(self, core) -> torch.Tensor:
        a = self.base.act(core).clamp(-1.0, 1.0)
        target_valid = core.est.valid.gather(1, core.target_idx[:, None]).squeeze(1)
        return self.safety.filter(a, core.det.detected.any(-1), target_valid,
                                  core.state.pos, core.yaw, core.step_max)
