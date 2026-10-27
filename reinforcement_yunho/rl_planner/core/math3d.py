"""Batched rotation helpers (torch). Quaternions are (w, x, y, z) — the Isaac Lab /
USD convention. Interfaces that use XYZW (state_window_interface, ROS poses) must
be reordered at the boundary (CONVENTIONS.md "Quaternion order is per-interface").
"""
from __future__ import annotations

import math

import torch

UP = (0.0, 0.0, 1.0)

#: CV camera (x right, y down, z forward) -> body (x forward, y left, z up).
#: Same matrix as root spec §6.1 R_IC (camera looks along body +x).
R_BODY_CV = ((0.0, 0.0, 1.0),
             (-1.0, 0.0, 0.0),
             (0.0, -1.0, 0.0))


def normalize(v: torch.Tensor, eps: float = 1e-9) -> torch.Tensor:
    return v / v.norm(dim=-1, keepdim=True).clamp_min(eps)


def quat_mul(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    aw, ax, ay, az = a.unbind(-1)
    bw, bx, by, bz = b.unbind(-1)
    return torch.stack((aw * bw - ax * bx - ay * by - az * bz,
                        aw * bx + ax * bw + ay * bz - az * by,
                        aw * by - ax * bz + ay * bw + az * bx,
                        aw * bz + ax * by - ay * bx + az * bw), dim=-1)


def quat_to_rotmat(q: torch.Tensor) -> torch.Tensor:
    w, x, y, z = normalize(q).unbind(-1)
    return torch.stack((
        torch.stack((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)), -1),
        torch.stack((2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)), -1),
        torch.stack((2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)), -1),
    ), dim=-2)


def quat_from_rotmat(R: torch.Tensor) -> torch.Tensor:
    """Rotation matrix (..., 3, 3) -> unit quaternion (w, x, y, z), w >= 0."""
    m = R
    t = m[..., 0, 0] + m[..., 1, 1] + m[..., 2, 2]
    qw = 0.5 * torch.sqrt((1 + t).clamp_min(0.0))
    qx = 0.5 * torch.sqrt((1 + m[..., 0, 0] - m[..., 1, 1] - m[..., 2, 2]).clamp_min(0.0))
    qy = 0.5 * torch.sqrt((1 - m[..., 0, 0] + m[..., 1, 1] - m[..., 2, 2]).clamp_min(0.0))
    qz = 0.5 * torch.sqrt((1 - m[..., 0, 0] - m[..., 1, 1] + m[..., 2, 2]).clamp_min(0.0))
    qx = torch.copysign(qx, m[..., 2, 1] - m[..., 1, 2])
    qy = torch.copysign(qy, m[..., 0, 2] - m[..., 2, 0])
    qz = torch.copysign(qz, m[..., 1, 0] - m[..., 0, 1])
    return normalize(torch.stack((qw, qx, qy, qz), -1))


def quat_rotate(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate v by q (body -> world when q is the body orientation)."""
    return (quat_to_rotmat(q) @ v.unsqueeze(-1)).squeeze(-1)


def quat_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate v by q^-1 (world -> body)."""
    return (quat_to_rotmat(q).transpose(-1, -2) @ v.unsqueeze(-1)).squeeze(-1)


def quat_from_yaw(yaw: torch.Tensor) -> torch.Tensor:
    z = torch.zeros_like(yaw)
    return torch.stack((torch.cos(yaw / 2), z, z, torch.sin(yaw / 2)), -1)


def yaw_from_quat(q: torch.Tensor) -> torch.Tensor:
    w, x, y, z = q.unbind(-1)
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def tilt_angle(q: torch.Tensor) -> torch.Tensor:
    """Angle between body z and world z [rad]."""
    R = quat_to_rotmat(q)
    return torch.acos(R[..., 2, 2].clamp(-1.0, 1.0))


def rot_z(yaw: torch.Tensor) -> torch.Tensor:
    c, s = torch.cos(yaw), torch.sin(yaw)
    z, o = torch.zeros_like(yaw), torch.ones_like(yaw)
    return torch.stack((torch.stack((c, -s, z), -1),
                        torch.stack((s, c, z), -1),
                        torch.stack((z, z, o), -1)), -2)


def rot_axis_angle(axis: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    """Rodrigues: unit axis (..., 3), angle (...) [rad] -> rotation matrix (..., 3, 3)."""
    x, y, z = axis.unbind(-1)
    c, s = torch.cos(angle), torch.sin(angle)
    C = 1.0 - c
    return torch.stack((torch.stack((c + x * x * C, x * y * C - z * s, x * z * C + y * s), -1),
                        torch.stack((y * x * C + z * s, c + y * y * C, y * z * C - x * s), -1),
                        torch.stack((z * x * C - y * s, z * y * C + x * s, c + z * z * C), -1)), -2)


def world_to_heading(v_w: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    """World vector -> yaw-only (gravity aligned) heading frame. v_w (..., 3), yaw (...)."""
    R = rot_z(yaw)
    return (R.transpose(-1, -2) @ v_w.unsqueeze(-1)).squeeze(-1)


def heading_to_world(v_h: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    return (rot_z(yaw) @ v_h.unsqueeze(-1)).squeeze(-1)


def quat_integrate(q: torch.Tensor, omega_b: torch.Tensor, dt: float) -> torch.Tensor:
    """q_{t+dt} = q ⊗ exp(omega_b dt / 2), omega in body frame."""
    ang = omega_b.norm(dim=-1, keepdim=True) * dt
    axis = normalize(omega_b)
    half = 0.5 * ang
    dq = torch.cat((torch.cos(half), axis * torch.sin(half)), dim=-1)
    return normalize(quat_mul(q, dq))


def vee(M: torch.Tensor) -> torch.Tensor:
    return torch.stack((M[..., 2, 1], M[..., 0, 2], M[..., 1, 0]), -1)


def deg2rad(x: float) -> float:
    return x * math.pi / 180.0


def det3(A: torch.Tensor) -> torch.Tensor:
    return (A[..., 0, 0] * (A[..., 1, 1] * A[..., 2, 2] - A[..., 1, 2] * A[..., 2, 1])
            - A[..., 0, 1] * (A[..., 1, 0] * A[..., 2, 2] - A[..., 1, 2] * A[..., 2, 0])
            + A[..., 0, 2] * (A[..., 1, 0] * A[..., 2, 1] - A[..., 1, 1] * A[..., 2, 0]))


def solve3(A: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """Batched 3x3 solve by the adjugate (no cuSOLVER workspace; fine for SPD + ridge)."""
    a, bb, c = A[..., 0, 0], A[..., 0, 1], A[..., 0, 2]
    d, e, f = A[..., 1, 0], A[..., 1, 1], A[..., 1, 2]
    g, h, i = A[..., 2, 0], A[..., 2, 1], A[..., 2, 2]
    adj = torch.stack((
        torch.stack((e * i - f * h, c * h - bb * i, bb * f - c * e), -1),
        torch.stack((f * g - d * i, a * i - c * g, c * d - a * f), -1),
        torch.stack((d * h - e * g, bb * g - a * h, a * e - bb * d), -1),
    ), -2)
    det = det3(A)
    det = torch.where(det.abs() < 1e-12, torch.full_like(det, 1e-12), det)
    return (adj @ b.unsqueeze(-1)).squeeze(-1) / det.unsqueeze(-1)


def eigmin_sym3(A: torch.Tensor) -> torch.Tensor:
    """Smallest eigenvalue of symmetric 3x3 matrices (closed form, Smith 1961)."""
    p1 = A[..., 0, 1] ** 2 + A[..., 0, 2] ** 2 + A[..., 1, 2] ** 2
    q = (A[..., 0, 0] + A[..., 1, 1] + A[..., 2, 2]) / 3.0
    p2 = (A[..., 0, 0] - q) ** 2 + (A[..., 1, 1] - q) ** 2 + (A[..., 2, 2] - q) ** 2 + 2.0 * p1
    p = torch.sqrt((p2 / 6.0).clamp_min(1e-30))
    eye = torch.eye(3, device=A.device, dtype=A.dtype)
    B = (A - q[..., None, None] * eye) / p[..., None, None]
    r = (det3(B) / 2.0).clamp(-1.0, 1.0)
    phi = torch.acos(r) / 3.0
    lam_min = q + 2.0 * p * torch.cos(phi + 2.0 * math.pi / 3.0)
    return torch.where(p2 < 1e-24, q, lam_min)
