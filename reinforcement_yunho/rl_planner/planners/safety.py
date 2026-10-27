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
           ``retreat_after_s``: a hold that has seen nothing for that long backs up along the path flown since the
           last window pass (breadcrumbs every ``crumb_spacing_m``, none within ``crumb_clear_m`` of the pass point)
           at ``retreat_speed`` [m/s], at most ``retreat_max_m`` — the path flown is free, and further back the
           window fits in the image again. A fast policy otherwise holds ~1.6 m short of a window it has not
           estimated yet, too close to see all four corners, and waits for a detection that never comes
           (RESULTS.md §11; drones that need vision to navigate retrace their route on signal loss the same way).
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
        rr = s.get("retreat_after_s")                 # None: hold in place until the next detection
        self.retreat_steps = None if rr is None else max(0, int(round(float(rr) / float(step_dt))))
        self.retreat_step_m = float(s.get("retreat_speed", 0.3)) * float(step_dt)
        self.retreat_max = float(s.get("retreat_max_m", 3.0))
        self.crumb_m = float(s.get("crumb_spacing_m", 0.1))
        self.crumb_clear = float(s.get("crumb_clear_m", 0.6))
        self.scan_hold = bool(s.get("scan_hold", False))    # holds keep the planner's yaw and altitude (look-around)
        self.scan_dz = float(s.get("scan_hold_dz", 1.0))
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
        self.hold_age = torch.zeros(E, dtype=torch.long, device=device)     # steps into the current hold
        if self.retreat_steps is not None:            # breadcrumb ring buffer (newest at head - 1)
            N = max(1, int(s.get("crumb_buffer", 64)))
            self.crumbs = torch.zeros(E, N, 3, device=device)
            self.head = torch.zeros(E, dtype=torch.long, device=device)
            self.n_crumbs = torch.zeros(E, dtype=torch.long, device=device)
            self.pass_pos = torch.zeros(E, 3, device=device)
            self.last_target = torch.full((E,), -1, dtype=torch.long, device=device)
            self.retreated = torch.zeros(E, device=device)

    def reset(self, env_ids) -> None:
        self.since_seen[env_ids] = 0
        self.ever_seen[env_ids] = False
        self.holding[env_ids] = False
        self.hold_steps[env_ids] = 0
        self.hold_age[env_ids] = 0
        if self.retreat_steps is not None:
            self.n_crumbs[env_ids] = 0
            self.last_target[env_ids] = -1
            self.retreated[env_ids] = 0.0

    @torch.no_grad()
    def filter(self, actions: torch.Tensor, seen: torch.Tensor, target_valid: torch.Tensor,
               pos: torch.Tensor, yaw: torch.Tensor, step_max: torch.Tensor,
               target_sigma: torch.Tensor | None = None, vel: torch.Tensor | None = None,
               target_idx: torch.Tensor | None = None) -> torch.Tensor:
        """actions (E,A) in [-1,1]; seen (E,) any window detected since the last decision;
        target_valid (E,) the current target window has a valid estimate; target_sigma (E,) its
        1-sigma position uncertainty [m] (a valid estimate looser than sigma_safe_m counts as unknown);
        vel (E,3) world-frame velocity (for hold_brake_acc; without it the hold latches the current position);
        target_idx (E,) index of the window to pass next (for retreat_after_s: the path is kept since the last pass)."""
        self.since_seen = torch.where(seen, torch.zeros_like(self.since_seen), self.since_seen + 1)
        self.ever_seen |= seen
        if not self.enabled:
            return actions
        limit = torch.where(self.ever_seen, torch.full_like(self.since_seen, self.lost_steps),
                            torch.full_like(self.since_seen, self.start_steps))
        known = target_valid if target_sigma is None else target_valid & (target_sigma <= self.sigma_safe)
        hold = (self.since_seen > limit) & ~known
        latch = pos
        if self.brake_acc is not None and vel is not None:   # braking stop point (horizontal), as PX4 Hold
            vh = torch.cat((vel[:, :2], torch.zeros_like(vel[:, 2:])), -1)
            latch = pos + vh * (vh.norm(dim=-1, keepdim=True) / (2.0 * self.brake_acc))
        self.hold_pos = torch.where((hold & ~self.holding)[:, None], latch, self.hold_pos)
        self.holding = hold
        self.hold_steps += hold.long()
        self.hold_age = torch.where(hold, self.hold_age + 1, torch.zeros_like(self.hold_age))
        if self.retreat_steps is not None and target_idx is not None:
            self._retrace(hold, pos, target_idx)
        wp_raw = pos + m3.heading_to_world(actions[:, :3] * step_max, yaw)
        wp = wp_raw
        slowed = torch.zeros_like(hold)
        if self.slow_steps is not None:               # blind and unknown target: ramp the step down to the hold
            span = max(1, self.lost_steps - self.slow_steps)
            scale = ((self.lost_steps - self.since_seen).float() / span).clamp(self.slow_min, 1.0)
            slowed = self.ever_seen & ~known & ~hold & (self.since_seen > self.slow_steps)
            wp = torch.where(slowed[:, None], pos + m3.heading_to_world(actions[:, :3] * step_max * scale[:, None], yaw), wp)
        wp = torch.where(hold[:, None], self.hold_pos, wp)
        if self.scan_hold:                            # a hold may still climb / descend to look around
            zc = torch.maximum(torch.minimum(wp_raw[:, 2], self.hold_pos[:, 2] + self.scan_dz), self.hold_pos[:, 2] - self.scan_dz)
            wp = torch.cat((wp[:, :2], torch.where(hold, zc, wp[:, 2])[:, None]), -1)
        wp = torch.maximum(torch.minimum(wp, self.hi), self.lo)
        touched = hold | slowed | (wp != wp_raw).any(-1)   # untouched envs keep the planner's action bit-for-bit
        out = actions.clone()
        out[:, :3] = torch.where(touched[:, None], (m3.world_to_heading(wp - pos, yaw) / step_max).clamp(-1.0, 1.0),
                                 actions[:, :3])
        if out.shape[1] > 3 and not self.scan_hold:
            out[:, 3] = torch.where(hold, torch.zeros_like(out[:, 3]), out[:, 3])
        return out


    def _retrace(self, hold: torch.Tensor, pos: torch.Tensor, target_idx: torch.Tensor) -> None:
        """Record breadcrumbs while flying; during a hold older than retreat_after_s move the hold point back along them."""
        N = self.crumbs.shape[1]
        ar = torch.arange(pos.shape[0], device=pos.device)
        passed = target_idx != self.last_target          # a window was passed (or the episode began): forget the path
        self.last_target = target_idx.clone()
        self.n_crumbs = torch.where(passed, torch.zeros_like(self.n_crumbs), self.n_crumbs)
        self.pass_pos = torch.where(passed[:, None], pos, self.pass_pos)
        newest = self.crumbs[ar, (self.head - 1) % N]
        moved = (self.n_crumbs == 0) | ((pos - newest).norm(dim=-1) >= self.crumb_m)
        rec = ~hold & moved & ((pos - self.pass_pos).norm(dim=-1) >= self.crumb_clear)
        self.crumbs[ar, self.head] = torch.where(rec[:, None], pos, self.crumbs[ar, self.head])
        self.head = torch.where(rec, (self.head + 1) % N, self.head)
        self.n_crumbs = torch.where(rec, (self.n_crumbs + 1).clamp_max(N), self.n_crumbs)
        self.retreated = torch.where(hold, self.retreated, torch.zeros_like(self.retreated))
        go = hold & (self.hold_age > self.retreat_steps) & (self.n_crumbs > 0) & (self.retreated < self.retreat_max)
        if not bool(go.any()):
            return
        d = self.crumbs[ar, (self.head - 1) % N] - self.hold_pos
        dist = d.norm(dim=-1)
        step = torch.minimum(dist, torch.full_like(dist, self.retreat_step_m))
        self.hold_pos = torch.where(go[:, None], self.hold_pos + d / dist.clamp_min(1e-6)[:, None] * step[:, None], self.hold_pos)
        self.retreated = self.retreated + torch.where(go, step, torch.zeros_like(step))
        reached = go & (dist <= self.retreat_step_m)
        self.head = torch.where(reached, (self.head - 1) % N, self.head)
        self.n_crumbs = torch.where(reached, self.n_crumbs - 1, self.n_crumbs)


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
        idx = core.selector.sel[:, None]                 # the planner's target (mission.selection)
        target_valid = core.est.valid.gather(1, idx).squeeze(1) & core.selector.has
        target_sigma = core.est.sigma.gather(1, idx).squeeze(1)
        return self.safety.filter(a, core.det.detected.any(-1), target_valid,
                                  core.state.pos, core.yaw, core.step_max, target_sigma, core.state.vel_w,
                                  core.target_idx)
