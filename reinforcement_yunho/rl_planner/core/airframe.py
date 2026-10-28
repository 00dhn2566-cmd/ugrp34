"""The real airframe — the team's FX450, a DJI F450-class X quad — for the referee and the renderers.

Up to now the referee treated the drone as a sphere of radius ``drone.collision_radius``. With
``drone.shape: airframe`` it uses the airframe's parts (``drone.airframe``) instead: four 10-inch propeller discs
on a 450 mm wheelbase X, the body (plates, flight computer, battery) and the 1 kg package hanging below.
Body frame = centre of mass (the point the dynamics integrate), x forward (the camera), y left, z up; the chassis
origin sits ``chassis_z`` above the centre of mass (the package pulls it down).

Per window plane (centre c, normal n, in-plane axes u = right, v = up) the referee needs
  cut     where each part meets the plane, as an interval box in (u, v) about c: exact for a propeller disc (the
          chord the plane cuts from it, widened by half the disc thickness), the whole projected box for a box
          (conservative; the boxes sit near the axis, well inside the propellers)
  shadow  the whole airframe projected along n onto the plane — the room left in the opening at a pass, the
          counterpart of the sphere's radius.
"""
from __future__ import annotations

import math
from typing import Mapping

import torch

PROP_RGB = (0.12, 0.12, 0.14)
ARM_RGB = {"front": (0.80, 0.12, 0.10), "rear": (0.92, 0.92, 0.90)}     # F450 kit: red arms at the front


class Airframe:
    def __init__(self, cfg: Mapping, device):
        af = cfg["drone"]["airframe"]
        zc = float(af["chassis_z"])
        self.chassis_z = zc
        self.rotor_radius = rr = float(af["rotor_radius"])
        self.angles = [math.radians(float(a)) for a in af["rotor_angles_deg"]]
        self.spin = [int(s) for s in af.get("rotor_spin", [1] * len(self.angles))]
        rz = float(af["rotor_z"]) + zc
        self.rotors = torch.tensor([[rr * math.cos(a), rr * math.sin(a), rz] for a in self.angles], device=device)
        self.rho = float(af["prop_radius"])
        self.ht = 0.5 * float(af["prop_thickness"])
        self.box_names = [k for k, v in af["boxes"].items() if v is not None]     # null: part taken off
        b = torch.tensor([[float(x) for x in af["boxes"][k]] for k in self.box_names], device=device)
        self.box_c = b[:, :3] + torch.tensor([0.0, 0.0, zc], device=device)
        self.box_a = b[:, 3:] / 2
        # farthest point from the centre of mass: no part can reach a plane farther away than this
        self.reach = max(float(self.rotors.norm(dim=-1).max()) + self.rho + self.ht,
                         float((self.box_c.norm(dim=-1) + self.box_a.norm(dim=-1)).max()))

    @staticmethod
    def _rel(pos, R, offsets, c):
        """World offsets of body points from each window centre: pos (E,3) or (E,W,3), offsets (K,3) -> (E,W,K,3)."""
        p = pos if pos.dim() == 3 else pos[:, None]
        return p[:, :, None] + torch.einsum("eij,kj->eki", R, offsets)[:, None] - c[:, :, None]

    def cut(self, pos, R, c, n, u, v):
        """Where the parts meet each window plane.

        pos (E,3), R (E,3,3) body -> world, window c, n, u, v (E,W,3).
        -> meets (E,W,P), centre (E,W,P,2) and half-size (E,W,P,2) of each part's cut in (u, v) about c;
        P = the propellers, then the boxes."""
        ax = torch.stack((u, v), -2)                                             # (E,W,2,3)
        m = R[:, None, :, 2].expand_as(n)                                        # disc normal = body z
        mn = (m * n).sum(-1)
        g = n - mn[..., None] * m                                                # n within the disc plane
        s = g.norm(dim=-1)
        gh = g / s.clamp_min(1e-6)[..., None]
        chord = torch.cross(m, gh, dim=-1)                                       # the cut line's direction
        rel = self._rel(pos, R, self.rotors, c)                                  # (E,W,K,3)
        d = (rel * n[:, :, None]).sum(-1)
        meets = d.abs() <= self.rho * s[..., None] + self.ht * mn.abs()[..., None]
        delta = (-d / s.clamp_min(1e-6)[..., None]).clamp(-self.rho, self.rho)   # chord offset from the disc centre
        half_len = (self.rho ** 2 - delta ** 2).clamp_min(0.0).sqrt()
        ctr = torch.einsum("ewkj,ewaj->ewka", rel + delta[..., None] * gh[:, :, None], ax)
        half = half_len[..., None] * torch.einsum("ewj,ewaj->ewa", chord, ax).abs()[:, :, None] + self.ht
        flat = (s < 0.05)[..., None, None]                                        # disc ~ in the plane: all of it
        ma = torch.einsum("ewj,ewaj->ewa", m, ax)
        whole = (self.rho * (1 - ma ** 2).clamp_min(0.0).sqrt() + self.ht * ma.abs())[:, :, None]
        ctr = torch.where(flat, torch.einsum("ewkj,ewaj->ewka", rel, ax), ctr)
        half = torch.where(flat, whole.expand_as(half), half)
        relb = self._rel(pos, R, self.box_c, c)                                  # (E,W,B,3)
        nb = torch.einsum("eji,ewj->ewi", R, n).abs()                            # |n| in body axes
        meets_b = (relb * n[:, :, None]).sum(-1).abs() <= (self.box_a * nb[:, :, None]).sum(-1)
        ctr_b = torch.einsum("ewbj,ewaj->ewba", relb, ax)
        half_b = torch.einsum("bi,ewai->ewba", self.box_a, torch.einsum("eji,ewaj->ewai", R, ax).abs())
        return torch.cat((meets, meets_b), -1), torch.cat((ctr, ctr_b), -2), torch.cat((half, half_b), -2)

    def shadow(self, pos, R, c, u, v):
        """The whole airframe projected onto each window plane: lo, hi (E,W,2) in (u, v) about c.
        pos (E,3) or (E,W,3) (e.g. the crossing points), R (E,3,3)."""
        ax = torch.stack((u, v), -2)
        ma = torch.einsum("ej,ewaj->ewa", R[..., 2], ax)
        half_d = (self.rho * (1 - ma ** 2).clamp_min(0.0).sqrt() + self.ht * ma.abs())[:, :, None]
        ctr_d = torch.einsum("ewkj,ewaj->ewka", self._rel(pos, R, self.rotors, c), ax)
        ctr_b = torch.einsum("ewbj,ewaj->ewba", self._rel(pos, R, self.box_c, c), ax)
        half_b = torch.einsum("bi,ewai->ewba", self.box_a, torch.einsum("eji,ewaj->ewai", R, ax).abs())
        ctr, half = torch.cat((ctr_d, ctr_b), -2), torch.cat((half_d.expand_as(ctr_d), half_b), -2)
        return (ctr - half).amin(-2), (ctr + half).amax(-2)

    def visual_parts(self) -> list:
        """Primitives for the renderers in the body frame: dicts with name, kind ('box': size; 'cylinder':
        radius, length along its local z), centre, R (3x3 nested list, local -> body), rgb, opacity."""
        eye = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        zc = self.chassis_z
        parts = []
        for name, c, a in zip(self.box_names, self.box_c.tolist(), (2 * self.box_a).tolist()):
            rgb = (0.78, 0.58, 0.32) if name == "package" else (0.14, 0.14, 0.15)
            parts.append(dict(name=name, kind="box", center=c, size=a, R=eye, rgb=rgb, opacity=1.0))
        for k, (a, p) in enumerate(zip(self.angles, self.rotors.tolist())):
            ca, sa = math.cos(a), math.sin(a)
            arm_R = [[0.0, -sa, ca], [0.0, ca, sa], [-1.0, 0.0, 0.0]]            # local z along the arm
            parts.append(dict(name=f"arm{k + 1}", kind="cylinder", center=[p[0] / 2, p[1] / 2, zc], radius=0.011,
                              length=self.rotor_radius, R=arm_R, rgb=ARM_RGB["front" if ca > 0 else "rear"],
                              opacity=1.0))
            parts.append(dict(name=f"motor{k + 1}", kind="cylinder", center=[p[0], p[1], p[2] - 0.016],
                              radius=0.014, length=0.028, R=eye, rgb=(0.06, 0.06, 0.06), opacity=1.0))
            parts.append(dict(name=f"prop{k + 1}", kind="cylinder", center=p, radius=self.rho,
                              length=2 * self.ht, R=eye, rgb=PROP_RGB, opacity=0.5))
        body = self.box_c[self.box_names.index("body")].tolist() if "body" in self.box_names else [0.0, 0.0, zc]
        half_x = float(self.box_a[self.box_names.index("body"), 0]) if "body" in self.box_names else 0.05
        parts.append(dict(name="camera", kind="box", center=[body[0] + half_x + 0.012, body[1], body[2]],
                          size=[0.024, 0.05, 0.035], R=eye, rgb=(0.03, 0.03, 0.03), opacity=1.0))
        return parts
