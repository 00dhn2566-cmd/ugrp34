"""Batched 6-DOF rigid-body quadrotor (torch backend physics).

Collective thrust along body +z, body torque, gravity and an external world force
(drag + wind, computed by the core once per physics step — exactly what the Isaac
backend hands to PhysX as an external wrench). Semi-implicit Euler with optional
sub-steps. The Isaac backend replaces exactly this class with PhysX.
"""
from __future__ import annotations

import torch

from . import math3d as m3


class RigidBodyState:
    def __init__(self, num_envs: int, device):
        z = lambda *s: torch.zeros(*s, device=device)  # noqa: E731
        self.pos = z(num_envs, 3)
        self.vel_w = z(num_envs, 3)
        self.quat = z(num_envs, 4)
        self.quat[:, 0] = 1.0
        self.omega_b = z(num_envs, 3)


class TorchQuadrotor:
    def __init__(self, num_envs: int, device, gravity: float = 9.81, substeps: int = 2):
        self.state = RigidBodyState(num_envs, device)
        self.g = gravity
        self.substeps = max(1, int(substeps))
        self.device = device

    def reset(self, env_ids, pos, quat):
        s = self.state
        s.pos[env_ids] = pos
        s.vel_w[env_ids] = 0.0
        s.omega_b[env_ids] = 0.0
        s.quat[env_ids] = quat

    def step(self, thrust, torque_b, f_ext_w, dt, mass, inertia):
        s = self.state
        h = dt / self.substeps
        gvec = torch.tensor((0.0, 0.0, -self.g), device=self.device)
        inv_m = (1.0 / mass)[:, None]
        for _ in range(self.substeps):
            R = m3.quat_to_rotmat(s.quat)
            acc = R[..., :, 2] * thrust[:, None] * inv_m + gvec + f_ext_w * inv_m
            s.vel_w = s.vel_w + acc * h
            s.pos = s.pos + s.vel_w * h
            Jw = inertia * s.omega_b
            dw = (torque_b - torch.cross(s.omega_b, Jw, dim=-1)) / inertia
            s.omega_b = s.omega_b + dw * h
            s.quat = m3.quat_integrate(s.quat, s.omega_b, h)
        return s
