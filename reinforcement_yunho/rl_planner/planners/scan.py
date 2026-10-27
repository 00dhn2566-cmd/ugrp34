"""Scripted look-around around a heading-fixed planner — the baseline for learned scanning (RESULTS.md §12).

A planner trained with a fixed heading (3 actions) only sees what is in front of the camera. This supervisor turns
the drone for it, the way a hand-written mission script would:

* search — at the start and right after every pass, stop and sweep the heading to +``sweep_deg`` and then to
  -``sweep_deg`` off the direction the drone left the window in (core/selector.py keeps that bookkeeping), at the
  yaw rate the learned planners get (``action.yaw_rate_max_deg``, at least 90 deg/s). If no candidate window was seen,
  climb ``climb_m`` above the pass altitude and sweep again, then descend the same below it, and repeat — the camera
  is fixed to the body, so up and down are scanned by changing altitude.
* face — once a sweep has finished with a candidate known, the nearest-exit choice (``core.selector``) is the
  target: still hovering, turn the camera to it (its triangulated centre, or the monocular guess) until it is
  within ``face_deg`` of the optical axis.
* approach — the wrapped planner flies the translation exactly as it was trained (its observations are
  heading-relative) while the supervisor keeps the camera on the target.

The yaw goes straight to the controller's yaw set-point (``core.yaw_cmd``), so the wrapped planner keeps its own
observation layout and action space (``action.yaw`` stays false). Committing only after a full sweep is what makes
the nearest-exit choice right: a nearer window outside the view so far would otherwise be skipped.
"""
from __future__ import annotations

import math
from typing import Mapping

import torch


def _wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(a), torch.cos(a))


class ScanSupervisor:
    name = "scan"

    def __init__(self, base, cfg: Mapping, num_envs: int, device, step_dt: float, sweep_deg: float = 100.0,
                 climb_m: float = 0.8, face_deg: float = 15.0):
        self.base = base
        E = int(num_envs)
        self.sweep = math.radians(sweep_deg)
        rate = max(90.0, float(cfg["action"].get("yaw_rate_max_deg", 60.0)))
        self.step = math.radians(rate) * float(step_dt)
        self.climb = float(climb_m)
        self.tol = math.radians(5.0)
        self.face = math.radians(face_deg)
        self.step_max_z = float(cfg["action"]["step_max"][2])
        self.phase = torch.zeros(E, dtype=torch.long, device=device)    # 0 sweep +, 1 sweep -, 2 face, 3 approach
        self.level = torch.zeros(E, dtype=torch.long, device=device)    # altitude: 0 pass, 1 up, 2 down
        self.last_target = torch.full((E,), -1, dtype=torch.long, device=device)
        self.searching_steps = torch.zeros(E, dtype=torch.long, device=device)   # for logging

    def reset(self, env_ids) -> None:
        self.base.reset(env_ids)
        self.phase[env_ids] = 0
        self.level[env_ids] = 0
        self.last_target[env_ids] = -1
        self.searching_steps[env_ids] = 0

    @torch.no_grad()
    def act(self, core) -> torch.Tensor:
        sel = core.selector
        passed = core.mission.target != self.last_target              # a pass (or a new episode): look around again
        self.last_target = core.mission.target.clone()
        self.phase = torch.where(passed, torch.zeros_like(self.phase), self.phase)
        self.level = torch.where(passed, torch.zeros_like(self.level), self.level)
        # sweep progress (heading off the direction the last window was left in, unwrapped)
        reached_pos = (self.phase == 0) & (sel.psi >= self.sweep - self.tol)
        reached_neg = (self.phase == 1) & (sel.psi <= -self.sweep + self.tol)
        self.phase = torch.where(reached_pos, torch.ones_like(self.phase), self.phase)
        done_sweep = reached_neg
        commit = done_sweep & sel.has
        self.phase = torch.where(commit, torch.full_like(self.phase, 2), self.phase)
        retry = done_sweep & ~sel.has                                  # nothing seen: next altitude, sweep again
        self.level = torch.where(retry, (self.level + 1) % 3, self.level)
        self.phase = torch.where(retry | ((self.phase >= 2) & ~sel.has), torch.zeros_like(self.phase), self.phase)
        p = sel.candidate_positions(core.est, core.perception)
        tgt = p.gather(1, sel.sel.view(-1, 1, 1).expand(-1, 1, 3)).squeeze(1)
        rel = tgt - core.state.pos
        look = torch.atan2(rel[:, 1], rel[:, 0])
        look = torch.where(rel[:, :2].norm(dim=-1) > 1.0, look, core.yaw_cmd)   # inside 1 m: keep the heading
        faced = (self.phase == 2) & (_wrap(look - core.yaw).abs() < self.face)
        self.phase = torch.where(faced, torch.full_like(self.phase, 3), self.phase)
        searching = self.phase < 2
        hovering = self.phase < 3
        self.searching_steps += searching.long()

        # yaw set-point: sweep target, or point the camera at the chosen window
        sweep_goal = sel.ref_yaw + torch.where(self.phase == 0, torch.full_like(sel.psi, self.sweep),
                                               torch.full_like(sel.psi, -self.sweep))
        # move the set-point at most one rate step: in a sweep along the unwrapped heading (so +-100 deg sweeps the
        # long way round through the exit direction), when approaching the shortest way to the chosen window
        cmd_psi = sel.psi + _wrap(core.yaw_cmd - core.yaw)
        err = torch.where(searching, (sweep_goal - sel.ref_yaw) - cmd_psi, _wrap(look - core.yaw_cmd))
        core.yaw_cmd = core.yaw_cmd + err.clamp(-self.step, self.step)

        a = self.base.act(core).clamp(-1.0, 1.0)
        # searching: hover in place, altitude to the scan level
        dz = torch.stack((torch.zeros_like(sel.ref_z), torch.full_like(sel.ref_z, self.climb),
                          torch.full_like(sel.ref_z, -self.climb)), -1).gather(1, self.level[:, None]).squeeze(1)
        z_goal = sel.ref_z + dz
        hover = torch.zeros_like(a)
        hover[:, 2] = torch.where(searching, ((z_goal - core.state.pos[:, 2]) / self.step_max_z).clamp(-1.0, 1.0),
                                  torch.zeros_like(z_goal))
        return torch.where(hovering[:, None], hover, a)
