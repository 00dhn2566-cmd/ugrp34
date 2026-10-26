"""Mission bookkeeping: target window, pass / collision events, success.

Geometry (all from GROUND TRUTH — this is the referee, never an observation):
  signed distance d = (p - c) . n   (n points to the approach side, so d > 0 before)
  crossing a wall plane between two physics sub-steps is detected by a sign
  change; the crossing point is interpolated and expressed in window (u, v).
  The drone is a sphere of radius r. Near a wall plane (|d| < r) it touches the
  wall if the disk where the sphere cuts the plane leaves the opening rectangle.

Events per decision step (OR-ed over the physics sub-steps):
  pass        target window crossed front->back with the centre inside the opening
  crash       wall/frame contact, ground, out of bounds, tilt, wrong-order or
              backwards passage (reason code kept for logging)
  success     all windows passed
"""
from __future__ import annotations

import math
from typing import Mapping

import torch

from . import math3d as m3
from .scene import SceneTensors

CRASH_NONE, CRASH_WALL, CRASH_GROUND, CRASH_BOUNDS, CRASH_TILT, CRASH_ORDER, CRASH_BACK = range(7)
CRASH_NAMES = ("none", "wall", "ground", "bounds", "tilt", "order", "backwards")


class MissionTracker:
    def __init__(self, cfg: Mapping, num_envs: int, max_windows: int, device):
        sc = cfg["scene"]
        self.r = float(cfg["drone"]["collision_radius"])
        self.margin = float(cfg["drone"]["safety_margin"])
        self.wall_half = float(sc["wall_half_extent"])
        b = sc["bounds"]
        self.x_min, self.y_abs = float(b["x_min"]), float(b["y_abs"])
        self.z_min, self.z_max = float(b["z_min"]), float(b["z_max"])
        self.cos_tilt = math.cos(math.radians(float(sc["max_tilt_deg"])))
        E, W = num_envs, max_windows
        self.E, self.W, self.device = E, W, device
        self.target = torch.zeros(E, dtype=torch.long, device=device)
        self.passed = torch.zeros(E, W, dtype=torch.bool, device=device)
        self.clearance = torch.zeros(E, W, device=device)      # slack at pass [m] (>= 0 safe for r)
        self.pass_time = torch.zeros(E, W, device=device)
        self.prev_d = torch.zeros(E, W, device=device)
        self.prev_pos = torch.zeros(E, 3, device=device)
        # per decision step
        self.step_pass = torch.zeros(E, W, dtype=torch.bool, device=device)
        self.crashed = torch.zeros(E, dtype=torch.bool, device=device)
        self.crash_reason = torch.zeros(E, dtype=torch.long, device=device)
        self.success = torch.zeros(E, dtype=torch.bool, device=device)

    def reset(self, env_ids, pos, scene: SceneTensors) -> None:
        self.target[env_ids] = 0
        self.passed[env_ids] = False
        self.clearance[env_ids] = 0.0
        self.pass_time[env_ids] = 0.0
        self.prev_pos[env_ids] = pos
        self.prev_d[env_ids] = ((pos[:, None, :] - scene.center[env_ids]) * scene.normal[env_ids]).sum(-1)
        self.step_pass[env_ids] = False
        self.crashed[env_ids] = False
        self.crash_reason[env_ids] = CRASH_NONE
        self.success[env_ids] = False

    def begin_step(self) -> None:
        self.step_pass[:] = False

    def _flag_crash(self, mask: torch.Tensor, code: int) -> None:
        new = mask & ~self.crashed & ~self.success
        self.crash_reason = torch.where(new, torch.full_like(self.crash_reason, code), self.crash_reason)
        self.crashed |= new

    @torch.no_grad()
    def substep(self, pos: torch.Tensor, quat: torch.Tensor, scene: SceneTensors, t_now: torch.Tensor) -> None:
        """Call after every physics sub-step with the new body position."""
        live = ~(self.crashed | self.success)
        c, n = scene.center, scene.normal
        d = ((pos[:, None, :] - c) * n).sum(-1)                       # (E,W)
        dp = self.prev_d
        crossed_fwd = (dp > 0) & (d <= 0) & scene.valid
        crossed_back = (dp <= 0) & (d > 0) & scene.valid
        crossed = crossed_fwd | crossed_back
        t = dp / (dp - d).where(crossed, torch.ones_like(d))
        X = self.prev_pos[:, None, :] + t[..., None] * (pos - self.prev_pos)[:, None, :]
        u = ((X - c) * scene.right).sum(-1)
        v = ((X - c) * scene.up).sum(-1)
        hw, hh = scene.size[..., 0] / 2, scene.size[..., 1] / 2
        inside = (u.abs() <= hw) & (v.abs() <= hh)
        on_wall = (u.abs() <= self.wall_half) & (v.abs() <= self.wall_half)

        # sphere touching the wall around the current position
        uc = ((pos[:, None, :] - c) * scene.right).sum(-1)
        vc = ((pos[:, None, :] - c) * scene.up).sum(-1)
        rho = (self.r ** 2 - d ** 2).clamp_min(0.0).sqrt()
        near = (d.abs() < self.r) & scene.valid
        leaves = (uc.abs() + rho > hw) | (vc.abs() + rho > hh)
        wall_touch = near & leaves & (uc.abs() <= self.wall_half + self.r) & (vc.abs() <= self.wall_half + self.r)

        self._flag_crash(live & (wall_touch.any(-1) | (crossed & ~inside & on_wall).any(-1)), CRASH_WALL)
        self._flag_crash(live & (crossed_back & inside).any(-1), CRASH_BACK)

        tgt_onehot = torch.nn.functional.one_hot(self.target.clamp_max(self.W - 1), self.W).bool()
        through = crossed_fwd & inside
        wrong = through & ~tgt_onehot & ~self.passed
        self._flag_crash(live & wrong.any(-1), CRASH_ORDER)

        good = through & tgt_onehot & live[:, None] & ~self.crashed[:, None]
        if good.any():
            slack = torch.minimum(hw - u.abs(), hh - v.abs()) - self.r
            self.clearance = torch.where(good, slack, self.clearance)
            self.pass_time = torch.where(good, t_now[:, None].expand_as(self.pass_time), self.pass_time)
            self.passed |= good
            self.step_pass |= good
            self.target = self.target + good.any(-1).long()
            self.success |= self.target >= scene.n_windows

        # bounds / ground / tilt
        R22 = m3.quat_to_rotmat(quat)[..., 2, 2]
        self._flag_crash(live & (pos[:, 2] < self.z_min), CRASH_GROUND)
        self._flag_crash(live & ((pos[:, 0] < self.x_min) | (pos[:, 1].abs() > self.y_abs) | (pos[:, 2] > self.z_max)),
                         CRASH_BOUNDS)
        self._flag_crash(live & (R22 < self.cos_tilt), CRASH_TILT)

        self.prev_d = d
        self.prev_pos = pos.clone()

    def target_index(self) -> torch.Tensor:
        """(E,) current target slot clamped to a valid index (after success: last window)."""
        return self.target.clamp_max(self.W - 1)
