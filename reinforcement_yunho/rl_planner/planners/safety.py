"""Runtime safety filter around any planner (perception-loss hold + geofence).

Neither planner was trained or tuned for a camera that sees nothing: with every
detection suppressed the state machine's SEARCH flies straight into the first
wall and the RL policy backs out of the arena (RESULTS.md). This filter is the
standard runtime guard for a learned planner and changes nothing while the
mission is nominal:

* hold   — no window detected for ``lost_hold_s`` (``start_grace_s`` before the first
           detection of the episode) AND the target window is not known (no valid
           estimate, or one looser than ``sigma_safe_m``): latch the current position
           and command it until the next detection, then hand control back.
           Thresholds come from nominal flights (scripts/perception_gaps.py): the RL
           policies' longest such streak was 0.3 s before the first detection and
           2.1 s after it (L5, 512 scenes, sigma_safe 0.3 m). Defaults (3 s, 0.3 m)
           were picked on outage tests (RESULTS.md §5).
           ``hold_brake_acc`` [m/s^2] latches the braking stop point instead of the current position:
           pos + v|v|/(2a) along the horizontal velocity (PX4 Hold brakes and holds where it stops). A hold
           that begins at ~1.1 m/s otherwise overshoots by its stopping distance and is pulled back, through
           the window it just flew through when one was that close (RESULTS.md §10).
* slow   — (``slow_after_s``) after that long without any detection while the target is unknown, the
           waypoint step shrinks linearly to ``slow_min_scale`` by the time the hold starts, so the hold
           does not begin at full speed next to a wall: the RL policies fly ~1.1 m/s and were 0.5 m from
           the next wall when a 3 s outage hold began, closer than their stopping distance (RESULTS.md §10).
* fence  — clamp every waypoint into the arena box: ``fence_margin_m`` inside the x/y
           bounds; altitude [z_min + 0.25, z_max - 0.3], the same waypoint clamp the
           policy was trained with (so nominal flight is never touched).

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
        self.lost_steps = max(1, int(round(float(s.get("lost_hold_s", 3.0)) / float(step_dt))))
        self.start_steps = max(1, int(round(float(s.get("start_grace_s", 0.5)) / float(step_dt))))
        so = s.get("slow_after_s")                     # None: no slow-down before the hold
        self.slow_steps = None if so is None else max(0, int(round(float(so) / float(step_dt))))
        self.slow_min = float(s.get("slow_min_scale", 0.25))
        hb = s.get("hold_brake_acc")                  # None: hold where the hold began
        self.brake_acc = None if hb is None else float(hb)
        ss = s.get("sigma_safe_m", 0.3)               # None: any valid target estimate counts as known
        self.sigma_safe = float("inf") if ss is None else float(ss)
        b, m = s.get("bounds") or cfg["scene"]["bounds"], float(s.get("fence_margin_m", 0.5))
        self.lo = torch.tensor([float(b["x_min"]) + m, -float(b["y_abs"]) + m, float(b["z_min"]) + 0.25], device=device)
        self.hi = torch.tensor([float("inf"), float(b["y_abs"]) - m, float(b["z_max"]) - 0.3], device=device)
        E = int(num_envs)
        self.since_seen = torch.zeros(E, dtype=torch.long, device=device)
        self.ever_seen = torch.zeros(E, dtype=torch.bool, device=device)
        self.holding = torch.zeros(E, dtype=torch.bool, device=device)
        self.hold_pos = torch.zeros(E, 3, device=device)
        self.hold_steps = torch.zeros(E, dtype=torch.long, device=device)   # per episode, for logging

    def reset(self, env_ids) -> None:
        self.since_seen[env_ids] = 0
        self.ever_seen[env_ids] = False
        self.holding[env_ids] = False
        self.hold_steps[env_ids] = 0

    @torch.no_grad()
    def filter(self, actions: torch.Tensor, seen: torch.Tensor, target_valid: torch.Tensor,
               pos: torch.Tensor, yaw: torch.Tensor, step_max: torch.Tensor,
               target_sigma: torch.Tensor | None = None, vel: torch.Tensor | None = None) -> torch.Tensor:
        """actions (E,A) in [-1,1]; seen (E,) any window detected since the last decision;
        target_valid (E,) the current target window has a valid estimate; target_sigma (E,) its
        1-sigma position uncertainty [m] (a valid estimate looser than sigma_safe_m counts as unknown);
        vel (E,3) world-frame velocity (needed with hold_brake_acc)."""
        self.since_seen = torch.where(seen, torch.zeros_like(self.since_seen), self.since_seen + 1)
        self.ever_seen |= seen
        if not self.enabled:
            return actions
        limit = torch.where(self.ever_seen, torch.full_like(self.since_seen, self.lost_steps),
                            torch.full_like(self.since_seen, self.start_steps))
        known = target_valid if target_sigma is None else target_valid & (target_sigma <= self.sigma_safe)
        hold = (self.since_seen > limit) & ~known
        latch = pos
        if self.brake_acc is not None:                # braking stop point (horizontal), as PX4 Hold
            vh = torch.cat((vel[:, :2], torch.zeros_like(vel[:, 2:])), -1)
            latch = pos + vh * (vh.norm(dim=-1, keepdim=True) / (2.0 * self.brake_acc))
        self.hold_pos = torch.where((hold & ~self.holding)[:, None], latch, self.hold_pos)
        self.holding = hold
        self.hold_steps += hold.long()
        wp_raw = pos + m3.heading_to_world(actions[:, :3] * step_max, yaw)
        wp = wp_raw
        slowed = torch.zeros_like(hold)
        if self.slow_steps is not None:               # blind and unknown target: ramp the step down to the hold
            span = max(1, self.lost_steps - self.slow_steps)
            scale = ((self.lost_steps - self.since_seen).float() / span).clamp(self.slow_min, 1.0)
            slowed = self.ever_seen & ~known & ~hold & (self.since_seen > self.slow_steps)
            wp = torch.where(slowed[:, None], pos + m3.heading_to_world(actions[:, :3] * step_max * scale[:, None], yaw), wp)
        wp = torch.where(hold[:, None], self.hold_pos, wp)
        wp = torch.maximum(torch.minimum(wp, self.hi), self.lo)
        touched = hold | slowed | (wp != wp_raw).any(-1)   # untouched envs keep the planner's action bit-for-bit
        out = actions.clone()
        out[:, :3] = torch.where(touched[:, None], (m3.world_to_heading(wp - pos, yaw) / step_max).clamp(-1.0, 1.0),
                                 actions[:, :3])
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
        idx = core.target_idx[:, None]
        target_valid = core.est.valid.gather(1, idx).squeeze(1)
        target_sigma = core.est.sigma.gather(1, idx).squeeze(1)
        return self.safety.filter(a, core.det.detected.any(-1), target_valid,
                                  core.state.pos, core.yaw, core.step_max, target_sigma, core.state.vel_w)
