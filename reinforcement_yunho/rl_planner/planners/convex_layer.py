"""Convex layer for the slung-load mission: the network's action, projected onto what the pendulum allows.

The network stays as it is. Behind it a small model-predictive QP (core/qp.py, all envs in one batch on the
GPU) picks the action closest to the network's whose predicted flight over the next ``horizon_s`` keeps
  * the load's swing within ``max_swing_deg`` of the vertical,
  * the drone and the load inside the target window's opening (shrunk by their sizes and ``margin_m``) while
    they are within ``gate_band_m`` of its plane, and the load inside the window the drone has just flown
    through until it is through as well (crossing times from the network's own action held over the horizon),
  * every waypoint step inside the action box.
Prediction model: the waypoint controller's closed loop (v_cmd = kp (w - p), a_cmd = kv (v_cmd - v); its thrust
is computed for the total mass) with the linearised cable pendulum (taut cable, small angles), per horizontal axis
    m_Q xdd_Q = m a_cmd + m_L g/L (x_L - x_Q),        xdd_L = -g/L (x_L - x_Q)
and zdd_Q = a_cmd,z vertically with the load L below; discretised exactly over a decision (zero-order hold).
The waypoints after the first are free variables in blocks of ``block`` decisions (receding horizon: only the
first is flown). It reads only what the drone has on board — its pose, the cable sensor (load offset and rate
from the hook) and its window estimates — like the safety filter, so the same class runs in training (in the
env), in evaluation and on board.
"""
from __future__ import annotations

import math
from typing import Mapping

import torch

from ..core import math3d as m3
from ..core.qp import solve_qp
from ..core.scene import window_frame

INF = float("inf")


def _zoh(F: torch.Tensor, G: torch.Tensor, dt: float):
    """Exact discretisation of xdot = F x + G u with u held over dt."""
    n, m = G.shape
    M = torch.zeros(n + m, n + m, dtype=torch.float64)
    M[:n, :n], M[:n, n:] = F, G
    E = torch.linalg.matrix_exp(M * dt)
    return E[:n, :n], E[:n, n:]


class SlungConvexLayer:
    # state (10): x_Q, vx_Q, x_L, vx_L, y_Q, vy_Q, y_L, vy_L, z_Q, vz_Q
    IQ, IL = (0, 4, 8), (2, 6, 8)

    def __init__(self, cfg: Mapping, num_envs: int, device, step_dt: float):
        s, c = cfg["slung"], cfg["controller"]
        cl = s.get("convex_layer") or {}
        g = float(cfg["sim"]["gravity"])
        self.device, self.E = device, num_envs
        m_tot, m_L = float(cfg["drone"]["mass"]), float(s["load_mass"])
        m_Q = m_tot - m_L
        self.L = float(s["cable_length"]) + m_L * g / float(s["cable_stiffness"])     # pendulum length (stretched)
        self.drop = self.L - float(s["attach"][2])       # load below the centre of mass: hook offset + cable
        self.r_load = float(s["load_radius"])
        self.drone_half = [float(v) for v in cl.get("drone_half_m", [0.32, 0.10])]   # (across, up/down) in a window
        self.margin = float(cl.get("margin_m", 0.05))
        self.band = float(cl.get("gate_band_m", 0.35))
        self.max_swing = math.sin(math.radians(float(cl.get("max_swing_deg", 25.0))))
        self.iters = int(cl.get("iters", 40))
        self.rho = float(cl.get("rho", 1.0))
        self.w_future = float(cl.get("future_weight", 1.0))
        # pull toward the middle of the opening near a window [per m^2] (0: off): the drone-plus-load stack centred
        # leaves the same room above the drone and below the load, robust to a biased window estimate
        self.w_center = float(cl.get("center_weight", 0.0))
        self.N = max(1, int(round(float(cl.get("horizon_s", 1.0)) / step_dt)))
        blk = max(1, int(cl.get("block", 3)))
        self.nb = math.ceil((self.N - 1) / blk) if self.N > 1 else 0
        self.step_max = torch.tensor([float(v) for v in cfg["action"]["step_max"]], device=device)
        kp, kv = [float(v) for v in c["kp_pos"]], [float(v) for v in c["kv"]]
        mu, nu, w2 = m_tot / m_Q, m_L / m_Q, g / self.L
        F = torch.zeros(10, 10, dtype=torch.float64)
        G = torch.zeros(10, 3, dtype=torch.float64)
        for ax, o in ((0, 0), (1, 4)):                   # horizontal: drone + pendulum
            F[o, o + 1] = 1.0
            F[o + 1, o] = -mu * kv[ax] * kp[ax] - nu * w2
            F[o + 1, o + 1] = -mu * kv[ax]
            F[o + 1, o + 2] = nu * w2
            F[o + 2, o + 3] = 1.0
            F[o + 3, o] = w2
            F[o + 3, o + 2] = -w2
            G[o + 1, ax] = mu * kv[ax] * kp[ax]
        F[8, 9], F[9, 8], F[9, 9], G[9, 2] = 1.0, -kv[2] * kp[2], -kv[2], kv[2] * kp[2]
        Ad, Bd = _zoh(F, G, step_dt)
        Cp = torch.zeros(3, 10, dtype=torch.float64)
        for i, j in enumerate(self.IQ):
            Cp[i, j] = 1.0
        At = Ad + Bd @ Cp                                # waypoint = own position + the step
        Phi, P = [], torch.eye(10, dtype=torch.float64)
        for _ in range(self.N):
            P = At @ P
            Phi.append(P)
        Gam = torch.zeros(self.N, self.N, 10, 3, dtype=torch.float64)    # state k+1 from the step at decision j <= k
        for k in range(self.N):
            for j in range(k + 1):
                Gam[k, j] = torch.linalg.matrix_power(At, k - j) @ Bd
        self.Phi = torch.stack(Phi).float().to(device)                   # (N,10,10): states after decisions 1..N
        self.G0 = Gam[:, 0].float().to(device)                           # (N,10,3)
        sm = [float(v) for v in cfg["action"]["step_max"]]
        self.sW = torch.tensor([max(sm[:2]), max(sm[:2]), sm[2]], device=device)   # world step bounds of later steps
        Gb = torch.zeros(self.N, max(self.nb, 1), 10, 3, dtype=torch.float64)
        for b in range(self.nb):
            for j in range(1 + b * blk, min(self.N, 1 + (b + 1) * blk)):
                Gb[:, b] += Gam[:, j]
        self.Gb = (Gb[:, : self.nb].float().to(device) * self.sW)        # (N,nb,10,3), block steps in [-1,1]
        self.n = 3 + 3 * self.nb
        self.warm = None

    def reset(self, env_ids=None) -> None:
        if self.warm is None or env_ids is None:
            self.warm = None
            return
        for t in self.warm:                              # restart only those envs' iterates
            t[env_ids] = 0.0

    # ------------------------------------------------------------------ the QP
    def project(self, a, pos, vel, yaw, load_rel, load_vel, gates):
        """a (E,4) network action; pos/vel (E,3) drone; load_rel/load_vel (E,3) load offset from the drone's centre
        of mass and its velocity relative to the drone (world); gates: list of (centre (E,3), normal (E,3),
        half size (E,2), active (E,), load_only bool). -> (projected action (E,4), changed (E,) bool)."""
        E, dev, N, n = a.shape[0], a.device, self.N, self.n
        a3 = a[:, :3].clamp(-1.0, 1.0)
        c, s_ = torch.cos(yaw), torch.sin(yaw)
        Rz = torch.zeros(E, 3, 3, device=dev)
        Rz[:, 0, 0], Rz[:, 0, 1], Rz[:, 1, 0], Rz[:, 1, 1], Rz[:, 2, 2] = c, -s_, s_, c, 1.0
        M = Rz * self.step_max                                           # world step from the normalised action
        s0 = torch.stack((pos[:, 0], vel[:, 0], pos[:, 0] + load_rel[:, 0], vel[:, 0] + load_vel[:, 0],
                          pos[:, 1], vel[:, 1], pos[:, 1] + load_rel[:, 1], vel[:, 1] + load_vel[:, 1],
                          pos[:, 2], vel[:, 2]), -1)
        free = torch.einsum("kij,ej->eki", self.Phi, s0)                 # (E,N,10)
        J0 = torch.einsum("kij,ejl->ekil", self.G0, M)                   # (E,N,10,3)
        if self.nb:
            Jb = self.Gb.permute(0, 2, 1, 3).reshape(N, 10, 3 * self.nb).expand(E, N, 10, 3 * self.nb)
            J = torch.cat((J0, Jb), -1)                                  # (E,N,10,n)
        else:
            J = J0
        # reference: the network's action now and its world step held for the rest of the horizon
        d_ref = (M @ a3[..., None])[..., 0] / self.sW
        x_ref = torch.cat((a3, d_ref.clamp(-1, 1).repeat(1, self.nb)), -1)
        nom = free + (J @ x_ref[:, None, :, None])[..., 0]               # (E,N,10)
        rows, lo, hi = [], [], []
        cP = torch.zeros(E, n, n, device=dev) if self.w_center > 0 else None
        cq = torch.zeros(E, n, device=dev) if self.w_center > 0 else None
        v_mid = 0.5 * (self.drop + self.r_load - self.drone_half[1])   # drone height that centres the stack

        def add(row, val, lo_b, hi_b):                                    # lo_b <= val + row x <= hi_b
            rows.append(row)
            lo.append(lo_b - val)
            hi.append(hi_b - val)

        # swing: horizontal load offset from the drone
        for iq, il in ((0, 2), (4, 6)):
            r = J[:, :, il] - J[:, :, iq]
            v = free[:, :, il] - free[:, :, iq]
            add(r, v, torch.full_like(v, -self.L * self.max_swing), torch.full_like(v, self.L * self.max_swing))
        down = torch.tensor([0.0, 0.0, -self.drop], device=dev)
        for (gc, gn, ghalf, gact, load_only) in gates:
            right, up = window_frame(gn)
            objs = [(self.IL, down, (self.r_load, self.r_load))]
            if not load_only:
                objs.append((self.IQ, torch.zeros(3, device=dev), tuple(self.drone_half)))
            for idx, off, (hu, hv) in objs:
                Jp = J[:, :, list(idx)]                                    # (E,N,3,n)
                fp = free[:, :, list(idx)] + off - gc[:, None]             # (E,N,3)
                npos = nom[:, :, list(idx)] + off - gc[:, None]
                near = (npos * gn[:, None]).sum(-1).abs() <= self.band
                act = near & gact[:, None]
                for axis, half_i, h_obj in ((right, 0, hu), (up, 1, hv)):
                    row = (Jp * axis[:, None, :, None]).sum(2)              # (E,N,n)
                    val = (fp * axis[:, None]).sum(-1)
                    lim = (ghalf[:, half_i:half_i + 1] - h_obj - self.margin).clamp_min(0.0).expand_as(val)
                    add(row, val, torch.where(act, -lim, -INF), torch.where(act, lim, INF))
                    if cP is not None and idx == self.IQ:
                        ra = row * act[..., None]
                        tgt = v_mid if half_i == 1 else 0.0
                        cP = cP + self.w_center * torch.einsum("ekn,ekm->enm", ra, ra)
                        cq = cq + self.w_center * torch.einsum("ekn,ek->en", ra, (val - tgt) * act)
        A = torch.cat([r.reshape(E, -1, n) for r in rows], 1)
        l = torch.cat([v.reshape(E, -1) for v in lo], 1)
        u = torch.cat([v.reshape(E, -1) for v in hi], 1)
        eye = torch.eye(n, device=dev).expand(E, n, n)
        A = torch.cat((A, eye), 1)
        l = torch.cat((l, -torch.ones(E, n, device=dev)), 1)
        u = torch.cat((u, torch.ones(E, n, device=dev)), 1)
        w = torch.ones(n, device=dev)
        w[3:] = self.w_future
        P = torch.diag_embed(w.expand(E, n))
        q = -w * x_ref
        if cP is not None:
            P, q = P + cP, q + cq
        warm = self.warm if self.warm is not None and self.warm[1].shape == l.shape else None
        x, z, y, _, _ = solve_qp(P, q, A, l, u, iters=self.iters, rho=self.rho, warm=warm)
        self.warm = (x.detach(), z.detach(), y.detach())
        out = a.clone()
        out[:, :3] = x[:, :3].clamp(-1.0, 1.0)
        changed = (out[:, :3] - a3).abs().amax(-1) > 1e-3
        return out, changed


class SlungLayerFilter:
    """The convex layer fed from the core with what the drone has on board (own state, cable sensor, window
    estimates): in the env loop (slung.convex_layer.in_env) and behind any planner (SlungLayerPlanner). Besides the
    target window it keeps the window the drone has just flown through until the load is through as well."""

    def __init__(self, cfg: Mapping, num_envs: int, device, step_dt: float):
        self.layer = SlungConvexLayer(cfg, num_envs, device, step_dt)
        self.prev = torch.full((num_envs,), -1, dtype=torch.long, device=device)   # window the load still owes
        self.last_sel = torch.full((num_envs,), -1, dtype=torch.long, device=device)
        self.changed = torch.zeros(num_envs, dtype=torch.bool, device=device)

    def reset(self, env_ids) -> None:
        self.layer.reset(env_ids)
        self.prev[env_ids] = -1
        self.last_sel[env_ids] = -1

    @torch.no_grad()
    def filter(self, a: torch.Tensor, core) -> torch.Tensor:
        sl, st, est = core.slung, core.state, core.est
        if est is None:
            return a
        sel = core.selector.sel
        switched = (self.last_sel >= 0) & (sel != self.last_sel)
        self.prev = torch.where(switched, self.last_sel, self.prev)
        self.last_sel = sel.clone()

        def g(t, i):
            idx = i.clamp_min(0).view(-1, 1, *([1] * (t.dim() - 2))).expand(-1, 1, *t.shape[2:])
            return t.gather(1, idx).squeeze(1)

        prev_c, prev_n = g(est.center, self.prev), g(est.normal, self.prev)
        owes = (self.prev >= 0) & (((sl.pos - prev_c) * prev_n).sum(-1) > -(sl.radius + self.layer.margin))
        self.prev = torch.where(owes, self.prev, torch.full_like(self.prev, -1))
        tgt_ok = core.selector.has & g(est.valid, sel)
        gates = [(g(est.center, sel), g(est.normal, sel), g(est.size, sel) / 2, tgt_ok, False),
                 (prev_c, prev_n, g(est.size, self.prev) / 2, owes & g(est.valid, self.prev), True)]
        out, self.changed = self.layer.project(a, st.pos, st.vel_w, core.yaw, sl.pos - st.pos, sl.vel - st.vel_w,
                                               gates)
        return out


class SlungLayerPlanner:
    """Any planner with the convex layer behind it (simulation side; reads the core like SafePlanner)."""

    def __init__(self, base, cfg: Mapping, num_envs: int, device, step_dt: float):
        self.base = base
        self.filt = SlungLayerFilter(cfg, num_envs, device, step_dt)

    def reset(self, env_ids) -> None:
        self.base.reset(env_ids)
        self.filt.reset(env_ids)

    @torch.no_grad()
    def act(self, core) -> torch.Tensor:
        if getattr(core, "layer", None) is not None:      # the env already projects every action (in_env): once only
            return self.base.act(core)
        return self.filt.filter(self.base.act(core), core)
