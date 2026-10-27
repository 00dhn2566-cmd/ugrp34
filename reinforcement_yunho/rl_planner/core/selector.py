"""Which window the planner flies to next (mission.selection), and how far it has looked around since the last pass.

``order``   — the next window in course order (the referee's target; windows are colour-coded by order).
``nearest`` — the "nearest exit" rule: at the start and right after every pass, the next window is the nearest
              window not passed yet, measured from the window just passed (its triangulated centre; the drone's
              position before the first pass). Only windows the perception has seen are candidates. A candidate's
              position is its triangulated centre when valid, otherwise a monocular guess from its latest detection:
              range = fx * mission.nominal_size_m / longest side [px] along the detection bearing. So right after a
              pass the choice is only as good as the look-around: a nearer window outside the view so far is not a
              candidate yet. scene.layout = scatter builds courses whose order follows this rule (every other window
              at least scene.nearest_margin times farther), and the referee still checks that order.

Both use only what the real stack has (own pose, the windows passed, detections and estimates), so the same class
runs in simulation and on board. The look-around bookkeeping (``scan``) feeds the ``search_state`` observation:
heading relative to the direction the drone left the last window in (unwrapped), the extremes of that heading and
of the altitude change since the pass, time since the pass and how many candidates are known.
"""
from __future__ import annotations

import math
from typing import Mapping

import torch

INF = float("inf")


def _wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(a), torch.cos(a))


class TargetSelector:
    def __init__(self, cfg: Mapping, num_envs: int, max_windows: int, device):
        m = cfg.get("mission") or {}
        self.mode = str(m.get("selection", "order"))
        if self.mode not in ("order", "nearest"):
            raise ValueError(f"mission.selection must be order or nearest, got {self.mode!r}")
        self.size = float(m.get("nominal_size_m", 1.0))
        self.fx = float(cfg["perception"]["camera"]["fx"])
        E, W = int(num_envs), int(max_windows)
        self.E, self.W, self.device = E, W, device
        z = lambda *s: torch.zeros(*s, device=device)  # noqa: E731
        self.sel = torch.zeros(E, dtype=torch.long, device=device)     # observation target window
        self.has = torch.ones(E, dtype=torch.bool, device=device)
        self.nxt = torch.zeros(E, dtype=torch.long, device=device)     # the one after it
        self.has_nxt = torch.zeros(E, dtype=torch.bool, device=device)
        self.n_cand = z(E)
        # look-around since the last pass (or the start)
        self.ref_pos, self.ref_yaw, self.ref_z, self.ref_t = z(E, 3), z(E), z(E), z(E)
        self.psi = z(E)                       # unwrapped heading - ref_yaw
        self.psi_hi, self.psi_lo, self.dz_hi, self.dz_lo = z(E), z(E), z(E), z(E)

    # ------------------------------------------------------------------ look-around bookkeeping
    def reset(self, env_ids, pos: torch.Tensor, yaw: torch.Tensor, t: torch.Tensor) -> None:
        """pos (n,3), yaw (n,), t (n,) of the envs starting an episode."""
        self._set_ref(env_ids, pos, yaw, pos[:, 2], t)

    def _set_ref(self, env_ids, pos, yaw, z, t) -> None:
        self.ref_pos[env_ids] = pos
        self.ref_yaw[env_ids] = yaw
        self.ref_z[env_ids] = z
        self.ref_t[env_ids] = t
        for v in (self.psi, self.psi_hi, self.psi_lo, self.dz_hi, self.dz_lo):
            v[env_ids] = 0.0

    def on_pass(self, passed_now: torch.Tensor, pos: torch.Tensor, vel: torch.Tensor, yaw: torch.Tensor,
                t: torch.Tensor, est_center: torch.Tensor, est_valid: torch.Tensor) -> None:
        """passed_now (E,) a window was passed this decision; est_center (E,3) / est_valid (E,) the estimate of
        that window. The reference becomes that window (its estimate, else the drone's position) and the drone's
        horizontal direction of travel (its heading when hovering)."""
        ids = passed_now.nonzero(as_tuple=False).squeeze(-1)
        if ids.numel() == 0:
            return
        v = vel[ids]
        moving = v[:, :2].norm(dim=-1) > 0.2
        travel = torch.where(moving, torch.atan2(v[:, 1], v[:, 0]), yaw[ids])
        ref = torch.where(est_valid[ids, None], est_center[ids], pos[ids])
        self._set_ref(ids, ref, travel, pos[ids, 2], t[ids])
        # heading so far = current heading relative to the new reference direction
        self.psi[ids] = _wrap(yaw[ids] - travel)
        self.psi_hi[ids] = self.psi[ids].clamp_min(0.0)
        self.psi_lo[ids] = self.psi[ids].clamp_max(0.0)

    def scan(self, yaw: torch.Tensor, z: torch.Tensor) -> None:
        """Advance the unwrapped heading and the explored extremes (call once per decision)."""
        d = _wrap(yaw - self.ref_yaw)
        self.psi = self.psi + _wrap(d - _wrap(self.psi))
        self.psi_hi = torch.maximum(self.psi_hi, self.psi)
        self.psi_lo = torch.minimum(self.psi_lo, self.psi)
        dz = z - self.ref_z
        self.dz_hi = torch.maximum(self.dz_hi, dz)
        self.dz_lo = torch.minimum(self.dz_lo, dz)

    # ------------------------------------------------------------------ choice
    def candidate_positions(self, est, perception) -> torch.Tensor:
        """(E,W,3) where the planner thinks each window is: triangulated centre, else the monocular guess."""
        rng = self.fx * self.size / perception.last_side_max.clamp_min(1.0)
        mono = perception.last_cam + rng[..., None] * perception.last_bearing
        return torch.where(est.valid[..., None], est.center, mono)

    def select(self, scene, mission, est, perception) -> None:
        if self.mode == "order":
            tgt = mission.target_index()
            self.sel = tgt
            self.has = torch.ones_like(self.has)
            self.nxt = (tgt + 1).clamp_max(self.W - 1)
            self.has_nxt = (tgt + 1) < scene.n_windows
            self.n_cand = torch.ones_like(self.n_cand)
            return
        seen = (perception.det_age < 1e3) | est.valid
        cand = scene.valid & ~mission.passed & seen                       # (E,W)
        p = self.candidate_positions(est, perception)
        d = (p - self.ref_pos[:, None, :]).norm(dim=-1).masked_fill(~cand, INF)
        dmin, sel = d.min(dim=-1)
        self.has = torch.isfinite(dmin)
        self.sel = sel
        ps = p.gather(1, sel.view(-1, 1, 1).expand(-1, 1, 3))           # (E,1,3)
        d2 = (p - ps).norm(dim=-1).masked_fill(~cand, INF)
        d2 = d2.scatter(1, sel.view(-1, 1), INF)
        dmin2, nxt = d2.min(dim=-1)
        self.has_nxt = self.has & torch.isfinite(dmin2)
        self.nxt = nxt
        self.n_cand = cand.float().sum(-1)

    def search_features(self, z: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        """(E,10): sin/cos heading off the reference direction, explored heading extremes / pi, altitude off the
        reference and its explored extremes / 2 m, seconds since the reference / 10, candidates known / 3, any."""
        return torch.stack((
            torch.sin(self.psi), torch.cos(self.psi),
            (self.psi_hi / math.pi).clamp(-2.0, 2.0), (self.psi_lo / math.pi).clamp(-2.0, 2.0),
            ((z - self.ref_z) / 2.0).clamp(-2.0, 2.0), (self.dz_hi / 2.0).clamp(-2.0, 2.0), (self.dz_lo / 2.0).clamp(-2.0, 2.0),
            ((t - self.ref_t) / 10.0).clamp(0.0, 1.0), (self.n_cand / 3.0).clamp_max(1.0), self.has.float()), -1)
