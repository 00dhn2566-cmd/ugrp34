"""Mission bookkeeping: target window, pass / collision events, success.

Geometry (all from GROUND TRUTH — this is the referee, never an observation):
  signed distance d = (p - c) . n   (n points to the approach side, so d > 0 before)
  crossing a wall plane between two physics sub-steps is detected by a sign
  change; the crossing point is interpolated and expressed in window (u, v).
  The drone is a sphere of radius r. Near a wall plane (|d| < r) it touches the
  wall if the disk where the sphere cuts the plane leaves the opening rectangle.
  drone.shape: airframe replaces the sphere with the real airframe (core/airframe.py): it
  touches the wall if any part's cut leaves the opening, and the clearance at a pass is
  measured from its shadow on the window plane.

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
from .airframe import Airframe
from .scene import SceneTensors

CRASH_NONE, CRASH_WALL, CRASH_GROUND, CRASH_BOUNDS, CRASH_TILT, CRASH_ORDER, CRASH_BACK, CRASH_LOAD, CRASH_CABLE = range(9)
CRASH_NAMES = ("none", "wall", "ground", "bounds", "tilt", "order", "backwards", "load", "cable")


class MissionTracker:
    def __init__(self, cfg: Mapping, num_envs: int, max_windows: int, device):
        sc = cfg["scene"]
        self.r = float(cfg["drone"]["collision_radius"])
        shape = str(cfg["drone"].get("shape", "sphere"))
        if shape not in ("sphere", "airframe"):
            raise ValueError(f"drone.shape: {shape} (sphere | airframe)")
        self.airframe = Airframe(cfg, device) if shape == "airframe" else None
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
        # slung load (core/slung.py), when one hangs from the drone: it must clear every window too
        self.load_past = torch.zeros(E, W, dtype=torch.bool, device=device)   # load through the opening
        self.prev_dL = torch.zeros(E, W, device=device)
        self.prev_load = torch.zeros(E, 3, device=device)
        # per decision step
        self.step_pass = torch.zeros(E, W, dtype=torch.bool, device=device)
        self.crashed = torch.zeros(E, dtype=torch.bool, device=device)
        self.crash_reason = torch.zeros(E, dtype=torch.long, device=device)
        self.success = torch.zeros(E, dtype=torch.bool, device=device)

    def reset(self, env_ids, pos, scene: SceneTensors, load_pos=None) -> None:
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
        if load_pos is not None:
            self.load_past[env_ids] = False
            self.prev_load[env_ids] = load_pos
            self.prev_dL[env_ids] = ((load_pos[:, None, :] - scene.center[env_ids]) * scene.normal[env_ids]).sum(-1)

    def begin_step(self) -> None:
        self.step_pass[:] = False

    def _flag_crash(self, mask: torch.Tensor, code: int) -> None:
        new = mask & ~self.crashed & ~self.success
        self.crash_reason = torch.where(new, torch.full_like(self.crash_reason, code), self.crash_reason)
        self.crashed |= new

    @torch.no_grad()
    def substep(self, pos: torch.Tensor, quat: torch.Tensor, scene: SceneTensors, t_now: torch.Tensor,
                load=None) -> None:
        """Call after every physics sub-step with the new body position; ``load`` = (load position (E,3), hook
        position (E,3), load radius) when a load hangs from the drone."""
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

        if self.airframe is None:
            # sphere touching the wall around the current position
            uc = ((pos[:, None, :] - c) * scene.right).sum(-1)
            vc = ((pos[:, None, :] - c) * scene.up).sum(-1)
            rho = (self.r ** 2 - d ** 2).clamp_min(0.0).sqrt()
            near = (d.abs() < self.r) & scene.valid
            leaves = (uc.abs() + rho > hw) | (vc.abs() + rho > hh)
            wall_touch = near & leaves & (uc.abs() <= self.wall_half + self.r) & (vc.abs() <= self.wall_half + self.r)
        else:
            # any part of the airframe meeting a wall plane outside the opening, on the panel; only the three
            # panels nearest the drone are checked (distance to the finite panel, not to its infinite plane:
            # on a 50-window course far panels' planes pass close by all the time)
            Rb = m3.quat_to_rotmat(quat)
            rel = pos[:, None, :] - c
            off = torch.stack(((rel * scene.right).sum(-1), (rel * scene.up).sum(-1)), -1)
            gap2 = d ** 2 + ((off.abs() - self.wall_half).clamp_min(0.0) ** 2).sum(-1)
            _, k = torch.where(scene.valid, gap2, torch.full_like(d, math.inf)).topk(min(3, self.W), -1,
                                                                                      largest=False)
            dk = d.gather(1, k).abs()
            pick = lambda t: t.gather(1, k[..., None].expand(-1, -1, t.shape[-1]))  # noqa: E731
            meets, ctr, half = self.airframe.cut(pos, Rb, pick(c), pick(n), pick(scene.right), pick(scene.up))
            lim = torch.stack((hw.gather(1, k), hh.gather(1, k)), -1)[:, :, None]
            leaves = ((ctr - half < -lim) | (ctr + half > lim)).any(-1)
            on_panel = ((ctr + half >= -self.wall_half) & (ctr - half <= self.wall_half)).all(-1)
            touch = (dk < self.airframe.reach) & (meets & leaves & on_panel).any(-1)
            wall_touch = torch.zeros_like(scene.valid).scatter(1, k, touch)

        self._flag_crash(live & (wall_touch.any(-1) | (crossed & ~inside & on_wall).any(-1)), CRASH_WALL)
        self._flag_crash(live & (crossed_back & inside).any(-1), CRASH_BACK)

        tgt_onehot = torch.nn.functional.one_hot(self.target.clamp_max(self.W - 1), self.W).bool()
        through = crossed_fwd & inside
        wrong = through & ~tgt_onehot & ~self.passed
        self._flag_crash(live & wrong.any(-1), CRASH_ORDER)

        good = through & tgt_onehot & live[:, None] & ~self.crashed[:, None]
        if good.any():
            if self.airframe is None:
                slack = torch.minimum(hw - u.abs(), hh - v.abs()) - self.r
            else:                                   # room left around the airframe's shadow at the crossing point
                g = good.float().argmax(-1, keepdim=True)          # the target window: at most one pass per env
                pick = lambda t: t.gather(1, g[..., None].expand(-1, -1, t.shape[-1]))  # noqa: E731
                lo, hi = self.airframe.shadow(pick(X), Rb, pick(c), pick(scene.right), pick(scene.up))
                lim = torch.stack((hw.gather(1, g), hh.gather(1, g)), -1)
                slack = torch.zeros_like(d).scatter(1, g, torch.minimum(lim - hi, lim + lo).amin(-1))
            self.clearance = torch.where(good, slack, self.clearance)
            self.pass_time = torch.where(good, t_now[:, None].expand_as(self.pass_time), self.pass_time)
            self.passed |= good
            self.step_pass |= good
            self.target = self.target + good.any(-1).long()
            if load is None:
                self.success |= self.target >= scene.n_windows
        if load is not None:
            self._load_substep(*load, scene, live)

        # bounds / ground / tilt
        R22 = m3.quat_to_rotmat(quat)[..., 2, 2]
        self._flag_crash(live & (pos[:, 2] < self.z_min), CRASH_GROUND)
        self._flag_crash(live & ((pos[:, 0] < self.x_min) | (pos[:, 1].abs() > self.y_abs) | (pos[:, 2] > self.z_max)),
                         CRASH_BOUNDS)
        self._flag_crash(live & (R22 < self.cos_tilt), CRASH_TILT)

        self.prev_d = d
        self.prev_pos = pos.clone()

    def _load_substep(self, pL, hook, rL: float, scene: SceneTensors, live) -> None:
        """The load (a ball of radius rL) and the cable (a segment hook -> load) against every wall: the load is
        refereed like the sphere drone (touching a wall outside the opening, or its centre crossing a wall plane
        outside the opening), the cable must cut a wall plane inside the opening. Success needs the load through
        the last window as well."""
        c, n = scene.center, scene.normal
        hw, hh = scene.size[..., 0] / 2, scene.size[..., 1] / 2
        rel = pL[:, None, :] - c
        dL = (rel * n).sum(-1)
        uc, vc = (rel * scene.right).sum(-1), (rel * scene.up).sum(-1)
        rho = (rL ** 2 - dL ** 2).clamp_min(0.0).sqrt()
        leaves = (uc.abs() + rho > hw) | (vc.abs() + rho > hh)
        touch = (dL.abs() < rL) & scene.valid & leaves & (uc.abs() <= self.wall_half + rL) & (vc.abs() <= self.wall_half + rL)
        dp = self.prev_dL
        fwd = (dp > 0) & (dL <= 0) & scene.valid
        crossed = fwd | ((dp <= 0) & (dL > 0) & scene.valid)
        t = dp / (dp - dL).where(crossed, torch.ones_like(dL))
        X = self.prev_load[:, None, :] + t[..., None] * (pL - self.prev_load)[:, None, :]
        u, v = ((X - c) * scene.right).sum(-1), ((X - c) * scene.up).sum(-1)
        inside = (u.abs() <= hw) & (v.abs() <= hh)
        on_wall = (u.abs() <= self.wall_half) & (v.abs() <= self.wall_half)
        self._flag_crash(live & (touch.any(-1) | (crossed & ~inside & on_wall).any(-1)), CRASH_LOAD)
        self.load_past |= fwd & inside
        dH = ((hook[:, None, :] - c) * n).sum(-1)
        cut = (dH * dL < 0) & scene.valid
        s = dH / (dH - dL).where(cut, torch.ones_like(dL))
        Xc = hook[:, None, :] + s[..., None] * (pL - hook)[:, None, :]
        uk, vk = ((Xc - c) * scene.right).sum(-1), ((Xc - c) * scene.up).sum(-1)
        out = (uk.abs() > hw) | (vk.abs() > hh)
        self._flag_crash(live & (cut & out & (uk.abs() <= self.wall_half) & (vk.abs() <= self.wall_half)).any(-1),
                         CRASH_CABLE)
        self._flag_crash(live & (pL[:, 2] < rL), CRASH_GROUND)
        last = (scene.n_windows - 1).clamp_min(0)[:, None]
        self.success |= (self.target >= scene.n_windows) & self.load_past.gather(1, last).squeeze(1) & ~self.crashed
        self.prev_dL = dL
        self.prev_load = pL.clone()

    def target_index(self) -> torch.Tensor:
        """(E,) current target slot clamped to a valid index (after success: last window)."""
        return self.target.clamp_max(self.W - 1)
