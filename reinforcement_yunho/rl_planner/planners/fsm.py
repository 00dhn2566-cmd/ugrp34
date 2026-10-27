"""State-machine baseline — vectorised port of the team's mission state machine.

Source: ``backup/prototype_demo/traj_manager.py`` (SCAN -> RECON -> APPROACH ->
TRAVERSE -> RESCAN) with the probe logic of ``prototype_demo/window_flight.py --seq``.
It acts through the SAME action space (waypoint delta) and sees the SAME
perception estimates as the RL policy, so evaluate.py compares planners, not
sensors. Differences from the original, on purpose:

* the scan anchor comes from the detector/estimate, not ground truth
  (traj_manager.scan_anchor read the GT window centre);
* "assess" uses the estimator's own 1-sigma (sigma_thr) + parallax gate, the
  runtime proxy traj_manager approximated with spread + residual;
* every leg has a step budget so a stuck leg cannot stall the mission.

States
------
SEARCH    no detection yet: advance along +x (heading frame), hold altitude
SCAN      fly lateral/vertical probes around the anchor to build parallax
ALIGN     point on the estimated window axis, d_app + align_back in front
APPROACH  approach gate (d_app in front of the estimated centre)
TRAVERSE  exit gate (d_exit behind the estimated centre); then next window
DONE      all windows handled: hover
"""
from __future__ import annotations

from typing import Mapping

import torch

from ..core import math3d as m3
from ..core.observations import _gather

SEARCH, SCAN, ALIGN, APPROACH, TRAVERSE, DONE = range(6)
STATE_NAMES = ("SEARCH", "SCAN", "ALIGN", "APPROACH", "TRAVERSE", "DONE")
LEG_BUDGET = 40          # decisions (4 s at 10 Hz) per leg before moving on


class FSMPlanner:
    name = "fsm"

    def __init__(self, cfg: Mapping, num_envs: int, device, follow_selector: bool = False):
        """``follow_selector``: fly to the planner-side choice (core.selector, mission.selection) instead of the
        windows in course order, restarting at SEARCH whenever that choice changes (look-around courses, §12)."""
        f = cfg["fsm"]
        self.f = f
        self.follow = bool(follow_selector)
        self.E, self.device = num_envs, torch.device(device)
        self.step_max = torch.tensor(cfg["action"]["step_max"], dtype=torch.float32, device=self.device)
        self.action_dim = 4 if cfg["action"]["yaw"] else 3
        self.fx = float(cfg["perception"]["camera"]["fx"])
        z = lambda dt=torch.long: torch.zeros(num_envs, dtype=dt, device=self.device)  # noqa: E731
        self.state = z()
        self.idx = z()
        self.probe = z()
        self.rounds = z()
        self.leg = z()
        self.anchor = torch.zeros(num_envs, 3, device=self.device)
        a, az = float(f["scan_amp"]), float(f["scan_amp"]) * 0.6
        # window_flight --seq probe order: +y, -y, +y+z, -y-z (heading frame offsets)
        self.probes = torch.tensor([[0.0, a, 0.0], [0.0, -a, 0.0], [0.0, a, az], [0.0, -a, -az]], device=self.device)

    def reset(self, env_ids: torch.Tensor) -> None:
        for t in (self.state, self.idx, self.probe, self.rounds, self.leg):
            t[env_ids] = 0
        self.anchor[env_ids] = 0.0

    # ------------------------------------------------------------------
    def _go(self, core, goal_w: torch.Tensor) -> torch.Tensor:
        delta_w = goal_w - core.state.pos
        delta_h = m3.world_to_heading(delta_w, core.yaw)
        return (delta_h / self.step_max).clamp(-1.0, 1.0)

    @torch.no_grad()
    def act(self, core) -> torch.Tensor:
        f = self.f
        pos = core.state.pos
        est = core.est
        pr = core.perception
        if self.follow:
            new = core.selector.sel
            changed = new != self.idx
            self.idx = new.clone()
            for t in (self.state, self.probe, self.rounds, self.leg):
                t.masked_fill_(changed, 0)                       # SEARCH == 0
        idx = self.idx.clamp_max(core.W - 1)
        n_win = core.scene.n_windows
        finished = self.idx >= n_win
        self.state = torch.where(finished, torch.full_like(self.state, DONE), self.state)

        e_valid = _gather(est.valid, idx)
        e_c = _gather(est.center, idx)
        e_n = m3.normalize(_gather(est.normal, idx) * torch.tensor((1.0, 1.0, 0.0), device=self.device))
        e_n = torch.where(e_n.norm(dim=-1, keepdim=True) > 0.5, e_n,
                          torch.tensor((-1.0, 0.0, 0.0), device=self.device).expand_as(e_n))
        e_par = _gather(est.parallax_deg, idx)
        e_sig = _gather(est.sigma, idx)
        seen = _gather(pr.det_age, idx) < 1e3
        bearing = _gather(pr.last_bearing, idx)
        side = _gather(pr.last_side, idx).clamp_min(1.0)
        rng_guess = (self.fx * 1.0 / side).clamp(1.0, 15.0)        # monocular size cue (~1 m opening)
        good = e_valid & (e_par >= float(f["min_parallax_deg"])) & (e_sig <= float(f["sigma_thr"]))

        goal = pos.clone()
        st = self.state
        self.leg += 1
        z_lo, z_hi = f["z_clip"]

        # SEARCH -> SCAN once the target has been seen
        search = st == SEARCH
        to_scan = search & (seen | e_valid)
        fwd = m3.heading_to_world(torch.tensor((1.0, 0.0, 0.0), device=self.device).expand(self.E, 3), core.yaw)
        goal = torch.where(search[:, None], pos + fwd * float(f["search_step"]), goal)
        goal[:, 2] = torch.where(search, torch.full_like(goal[:, 2], 1.2), goal[:, 2])

        # (re)enter SCAN: anchor in front of the estimate, else along the bearing
        anchor_est = e_c + e_n * float(f["scan_radius"])
        anchor_det = pos + bearing * (rng_guess - float(f["scan_radius"])).clamp_min(0.0)[:, None]
        new_anchor = torch.where(e_valid[:, None], anchor_est, anchor_det)
        new_anchor[:, 2] = new_anchor[:, 2].clamp(z_lo, z_hi)
        # never anchor behind the current position along the window axis
        ahead = ((new_anchor - pos) * fwd).sum(-1, keepdim=True)
        new_anchor = torch.where(ahead < 0, new_anchor - fwd * ahead, new_anchor)
        entering = to_scan
        self.anchor = torch.where(entering[:, None], new_anchor, self.anchor)
        self.probe = torch.where(entering, torch.zeros_like(self.probe), self.probe)
        self.leg = torch.where(entering, torch.zeros_like(self.leg), self.leg)
        st = torch.where(to_scan, torch.full_like(st, SCAN), st)

        # SCAN: assess first (traj_manager RECON), else keep probing
        scan = st == SCAN
        go_align = scan & good
        n_probe = self.probes.shape[0]
        exhausted = scan & (self.probe >= n_probe * int(f["max_scan_rounds"]))
        go_align = go_align | (exhausted & e_valid)
        restart = exhausted & ~e_valid
        off_h = self.probes[self.probe % n_probe]
        off_w = m3.heading_to_world(off_h, core.yaw)
        probe_goal = self.anchor + off_w
        probe_goal[:, 2] = probe_goal[:, 2].clamp(z_lo, z_hi)
        reached = (probe_goal - pos).norm(dim=-1) < float(f["reach_tol"])
        advance = scan & ~go_align & (reached | (self.leg > LEG_BUDGET // 2))
        self.probe = self.probe + advance.long()
        self.leg = torch.where(advance, torch.zeros_like(self.leg), self.leg)
        goal = torch.where((scan & ~go_align)[:, None], probe_goal, goal)
        st = torch.where(go_align, torch.full_like(st, ALIGN), st)
        st = torch.where(restart, torch.full_like(st, SEARCH), st)
        self.leg = torch.where(go_align | restart, torch.zeros_like(self.leg), self.leg)

        # ALIGN / APPROACH / TRAVERSE use the latest estimate
        d_app, d_exit, back = float(f["d_app"]), float(f["d_exit"]), float(f["align_back"])
        d_now = ((pos - e_c) * e_n).sum(-1)
        align_pt = e_c + e_n * (d_app + back)
        app_pt = e_c + e_n * d_app
        exit_pt = e_c - e_n * d_exit
        for p in (align_pt, app_pt, exit_pt):
            p[:, 2] = p[:, 2].clamp(z_lo, z_hi)

        align = st == ALIGN
        skip_align = align & (d_now < d_app + back)            # already past the align point
        done_align = align & (((align_pt - pos).norm(dim=-1) < float(f["reach_tol"])) | skip_align | (self.leg > LEG_BUDGET))
        goal = torch.where((align & ~done_align)[:, None], align_pt, goal)
        st = torch.where(done_align, torch.full_like(st, APPROACH), st)
        self.leg = torch.where(done_align, torch.zeros_like(self.leg), self.leg)

        app = st == APPROACH
        done_app = app & (((app_pt - pos).norm(dim=-1) < float(f["reach_tol"])) | (d_now < d_app) | (self.leg > LEG_BUDGET))
        goal = torch.where((app & ~done_app)[:, None], app_pt, goal)
        st = torch.where(done_app, torch.full_like(st, TRAVERSE), st)
        self.leg = torch.where(done_app, torch.zeros_like(self.leg), self.leg)

        trav = st == TRAVERSE
        done_trav = trav & ((d_now < -0.5 * d_exit) | (self.leg > LEG_BUDGET))
        goal = torch.where((trav & ~done_trav)[:, None], exit_pt, goal)
        # traversal finished -> next window (RESCAN starts from SEARCH; traj_manager RESCAN)
        self.idx = self.idx + done_trav.long()
        st = torch.where(done_trav, torch.full_like(st, SEARCH), st)
        self.leg = torch.where(done_trav, torch.zeros_like(self.leg), self.leg)
        goal = torch.where(done_trav[:, None], exit_pt, goal)

        done = st == DONE
        goal = torch.where(done[:, None], pos, goal)
        self.state = st
        act = self._go(core, goal)
        if self.action_dim == 4:
            act = torch.cat((act, torch.zeros(self.E, 1, device=self.device)), -1)
        return act
