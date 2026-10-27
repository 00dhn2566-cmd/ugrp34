"""Low-level controllers: waypoint -> collective thrust + body torque (torch, batched).

The default is the geometric SE(3) position controller of Lee, Leok & McClamroch
(CDC 2010) — the same family Isaac Lab ships for its multirotor tasks
(``isaaclab_contrib.controllers.LeePositionController``). It lives here as plain
torch so the torch backend, the Isaac backend and unit tests share one
implementation. To plug in another controller (e.g. 성진's PID through ctypes),
subclass :class:`Controller` and add it to ``CONTROLLERS``.
"""
from __future__ import annotations

import math
from typing import Dict, Mapping, Type

import torch

from . import math3d as m3


class Controller:
    def __init__(self, cfg: Mapping, num_envs: int, device):
        self.cfg, self.E, self.device = cfg, num_envs, device

    def reset(self, env_ids: torch.Tensor) -> None:
        pass

    def compute(self, pos, vel_w, quat, omega_b, target_pos, target_yaw, mass, inertia, thrust_max):
        """-> (thrust (E,), torque_b (E,3)) commands before motor lag."""
        raise NotImplementedError

    def set_variation(self, env_ids, kp_scale, kv_scale, ki) -> None:
        """Per-env gain mismatch for DR / stress (domain_randomization.ctrl_*)."""
        raise NotImplementedError(f"{type(self).__name__} does not support controller variation")

    def variation(self):
        """-> (kp scale, kv scale, ki) per env (E,) — the privileged critic term reads it."""
        one = torch.ones(self.E, device=self.device)
        return one, one, torch.zeros_like(one)


class LeePositionController(Controller):
    def __init__(self, cfg: Mapping, num_envs: int, device):
        super().__init__(cfg, num_envs, device)
        c = cfg["controller"]
        t = lambda k: torch.tensor(c[k], dtype=torch.float32, device=device)  # noqa: E731
        self.kp_pos, self.kv = t("kp_pos"), t("kv")
        self.k_rot, self.k_omega = t("k_rot"), t("k_omega")
        self.v_max = t("v_max")
        self.a_max = float(c["a_max"])
        self.tan_tilt = math.tan(math.radians(float(c["max_tilt_deg"])))
        self.g = float(cfg["sim"]["gravity"])
        self.torque_max = torch.tensor(cfg["drone"]["torque_max"], dtype=torch.float32, device=device)
        self.dt = float(cfg["sim"]["physics_dt"])
        self.i_max = float(c.get("i_max_acc", 1.5))
        self.kp_s = self.kv_s = self.ki = self.i_acc = None   # per-env variation (set_variation), off by default

    def set_variation(self, env_ids, kp_scale, kv_scale, ki) -> None:
        """Per-env controller mismatch (domain randomisation / stress): position and velocity gains
        scaled, plus a velocity-error integral (PX4-style velocity PID ``I``) with gain ``ki`` [1/s^2]."""
        if self.kp_s is None:
            z = lambda v: torch.full((self.E, 1), v, device=self.device)  # noqa: E731
            self.kp_s, self.kv_s, self.ki = z(1.0), z(1.0), z(0.0)
            self.i_acc = torch.zeros(self.E, 3, device=self.device)
        self.kp_s[env_ids] = kp_scale[:, None]
        self.kv_s[env_ids] = kv_scale[:, None]
        self.ki[env_ids] = ki[:, None]
        self.i_acc[env_ids] = 0.0

    def variation(self):
        """-> (kp scale, kv scale, ki) per env, (E,) each."""
        if self.kp_s is None:
            one = torch.ones(self.E, device=self.device)
            return one, one, torch.zeros_like(one)
        return self.kp_s[:, 0], self.kv_s[:, 0], self.ki[:, 0]

    def reset(self, env_ids: torch.Tensor) -> None:
        if self.i_acc is not None:
            self.i_acc[env_ids] = 0.0

    def compute(self, pos, vel_w, quat, omega_b, target_pos, target_yaw, mass, inertia, thrust_max):
        e_p = target_pos - pos
        if self.kp_s is None:
            v_cmd = torch.maximum(torch.minimum(self.kp_pos * e_p, self.v_max), -self.v_max)
            a_cmd = self.kv * (v_cmd - vel_w)
        else:
            v_cmd = torch.maximum(torch.minimum(self.kp_pos * self.kp_s * e_p, self.v_max), -self.v_max)
            e_v = v_cmd - vel_w
            self.i_acc = (self.i_acc + self.ki * e_v * self.dt).clamp(-self.i_max, self.i_max)
            a_cmd = self.kv * self.kv_s * e_v + self.i_acc
        a_xy = a_cmd[:, :2]
        n_xy = a_xy.norm(dim=-1, keepdim=True)
        a_cmd = torch.cat((a_xy * (self.a_max / n_xy.clamp_min(1e-9)).clamp_max(1.0),
                           a_cmd[:, 2:].clamp(-0.6 * self.g, 1.5 * self.a_max)), dim=-1)
        F = mass[:, None] * (a_cmd + torch.tensor((0.0, 0.0, self.g), device=self.device))
        # tilt limit: keep F_z, shrink the horizontal part
        Fz = F[:, 2:].clamp_min(1e-3)
        Fh = F[:, :2]
        h_max = Fz * self.tan_tilt
        Fh = Fh * (h_max / Fh.norm(dim=-1, keepdim=True).clamp_min(1e-9)).clamp_max(1.0)
        F = torch.cat((Fh, Fz), dim=-1)

        R = m3.quat_to_rotmat(quat)
        b3d = m3.normalize(F)
        b1c = torch.stack((torch.cos(target_yaw), torch.sin(target_yaw), torch.zeros_like(target_yaw)), -1)
        b2d = m3.normalize(torch.cross(b3d, b1c, dim=-1))
        b1d = torch.cross(b2d, b3d, dim=-1)
        Rd = torch.stack((b1d, b2d, b3d), dim=-1)
        e_R = 0.5 * m3.vee(Rd.transpose(-1, -2) @ R - R.transpose(-1, -2) @ Rd)
        e_w = omega_b
        Jw = inertia * omega_b
        torque = inertia * (-self.k_rot * e_R - self.k_omega * e_w) + torch.cross(omega_b, Jw, dim=-1)
        torque = torch.maximum(torch.minimum(torque, self.torque_max), -self.torque_max)
        thrust = (F * R[..., :, 2]).sum(-1).clamp(0.0, None)
        thrust = torch.minimum(thrust, thrust_max)
        return thrust, torque


CONTROLLERS: Dict[str, Type[Controller]] = {"lee_position": LeePositionController}


def make_controller(cfg: Mapping, num_envs: int, device) -> Controller:
    kind = cfg["controller"]["type"]
    if kind not in CONTROLLERS:
        raise KeyError(f"unknown controller.type {kind!r}; registered: {sorted(CONTROLLERS)}")
    return CONTROLLERS[kind](cfg, num_envs, device)


class MotorLag:
    """First-order lag on (thrust, torque) — stands in for rotor spin-up (DR time constant)."""

    def __init__(self, num_envs: int, device):
        self.f = torch.zeros(num_envs, device=device)
        self.tau = torch.zeros(num_envs, 3, device=device)
        self.tc = torch.full((num_envs,), 0.03, device=device)

    def reset(self, env_ids, hover_thrust, tc=None):
        self.f[env_ids] = hover_thrust
        self.tau[env_ids] = 0.0
        if tc is not None:
            self.tc[env_ids] = tc

    def step(self, f_cmd, tau_cmd, dt: float):
        a = (dt / self.tc.clamp_min(dt)).clamp_max(1.0)
        self.f = self.f + a * (f_cmd - self.f)
        self.tau = self.tau + a[:, None] * (tau_cmd - self.tau)
        return self.f, self.tau
