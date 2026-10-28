"""A load hanging from the drone on a cable (a pendulum): the next mission after the airframe.

Model (the standard quadrotor slung-load setup, Sreenath, Lee & Kumar 2013, with a cable that can go slack):
  load     point mass ``load_mass`` at p_L, gravity, linear drag, the same wind acceleration as the drone
  cable    massless, from the hook (``attach`` in the drone body frame, below the centre of mass) to the load;
           it only pulls: tension T = max(0, k (|r| - L) + c d|r|/dt), r = p_L - p_hook. The stiffness is a real
           rope's (stretch at hover ~2 mm) and keeps the explicit coupling stable at the 10 ms physics step of
           both backends (omega h ~ 0.9, c h / mu ~ 0.9).
  drone    gets T r/|r| at the hook: a force (world) and a torque (body) on top of the controller's wrench.

Backend-agnostic like the referee: the core computes the cable force once per physics step from the mirrored
drone state, the backend integrates the drone with it, and the core integrates the load with the opposite force
(semi-implicit Euler), so torch and Isaac fly the same pendulum.
"""
from __future__ import annotations

from typing import Mapping

import torch

from . import math3d as m3


class SlungLoad:
    def __init__(self, cfg: Mapping, num_envs: int, device):
        s = cfg["slung"]
        self.L = float(s["cable_length"])
        self.m = float(s["load_mass"])
        self.radius = float(s["load_radius"])
        self.k = float(s["cable_stiffness"])
        self.c = float(s["cable_damping"])
        self.drag = float(s["load_drag"])
        self.attach = torch.tensor([float(v) for v in s["attach"]], device=device)
        self.g = float(cfg["sim"]["gravity"])
        self.device = device
        z = lambda *sh: torch.zeros(*sh, device=device)  # noqa: E731
        self.pos = z(num_envs, 3)
        self.vel = z(num_envs, 3)
        self.tension = z(num_envs)
        self.force_on_load = z(num_envs, 3)

    def hook(self, pos, quat):
        """Hook position (world) and the body -> world rotation."""
        R = m3.quat_to_rotmat(quat)
        return pos + R @ self.attach, R

    def reset(self, env_ids, pos, quat) -> None:
        """Load hanging still, straight below the hook, the cable stretched by its static load."""
        h, _ = self.hook(pos, quat)
        sag = self.L + self.m * self.g / self.k
        self.pos[env_ids] = h - torch.tensor([0.0, 0.0, sag], device=self.device)
        self.vel[env_ids] = 0.0
        self.tension[env_ids] = self.m * self.g
        self.force_on_load[env_ids] = torch.tensor([0.0, 0.0, self.m * self.g], device=self.device)

    def cable_force(self, pos, vel, quat, omega_b):
        """Cable force on the drone for the coming physics step: (force world (E,3), torque body (E,3)).
        Also stores the opposite force for :meth:`integrate`."""
        h, R = self.hook(pos, quat)
        v_hook = vel + (R @ torch.cross(omega_b, self.attach.expand_as(omega_b), dim=-1)[..., None])[..., 0]
        r = self.pos - h
        dist = r.norm(dim=-1).clamp_min(1e-6)
        e = r / dist[:, None]
        stretch_rate = ((self.vel - v_hook) * e).sum(-1)
        T = (self.k * (dist - self.L) + self.c * stretch_rate).clamp_min(0.0) * (dist > self.L)
        self.tension = T
        f_drone = T[:, None] * e
        self.force_on_load = -f_drone
        torque_b = torch.cross(self.attach.expand_as(f_drone), (R.transpose(-1, -2) @ f_drone[..., None])[..., 0],
                               dim=-1)
        return f_drone, torque_b

    def integrate(self, dt: float, wind_acc=None) -> None:
        """Advance the load one physics step with the force stored by :meth:`cable_force`."""
        acc = self.force_on_load / self.m - self.drag / self.m * self.vel
        acc[:, 2] -= self.g
        if wind_acc is not None:
            acc = acc + wind_acc
        self.vel = self.vel + acc * dt
        self.pos = self.pos + self.vel * dt

    def features(self, pos, vel, quat, yaw):
        """(E,6): load offset from the hook / cable length and its velocity relative to the hook [m/s], both in
        the heading frame (what a 2-axis cable-angle sensor at the hook plus its rate gives)."""
        h, R = self.hook(pos, quat)
        r = m3.world_to_heading(self.pos - h, yaw) / self.L
        dv = m3.world_to_heading(self.vel - vel, yaw)
        return torch.cat((r, dv), -1)
