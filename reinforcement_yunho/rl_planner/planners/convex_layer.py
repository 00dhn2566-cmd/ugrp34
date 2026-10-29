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

``convex_layer.model: scp`` (SlungSCPLayer) is the same layer for a cable as long as the window is tall: a
nonlinear prediction (the simulator's tension-only cable, so the load may fly free on a slack cable, and the
attitude the cable tips) linearised along its own predicted trajectory and re-solved — sequential convex
programming, the crossing events re-read from each prediction (RESULTS.md §15).
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
    def project(self, a, pos, vel, yaw, load_rel, load_vel, gates, quat=None, omega_b=None):
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


class SlungSCPLayer:
    """The same projection for a cable as long as the window is tall (RESULTS.md §15): the load no longer fits
    under the drone in the opening and has to be swung — or tossed on a slack cable — through, so the small-angle
    model above cannot hold the solution: sequential convex programming instead (``convex_layer.model: scp``).

    Prediction: the nonlinear closed loop of the simulator, reduced. The waypoint controller with its saturations
    (core/controller.py) commands a force F; the thrust f = F.b3 acts along the body axis b3, whose tilt b_h (its
    horizontal part) follows F/|F| through the attitude loop and is tipped by the cable pulling at the hook below
    the centre of mass,
        b_h'' = -k_rot (b_h - F_h/|F|) - k_omega b_h' - s (h T / J) e_h     (h hook depth, J roll/pitch inertia;
    s = ``tip_scale``, identified on simulator flights); drone and load are point masses joined by the simulator's cable
    (core/slung.py) — tension-only, T = max(0, k (|r| - L) + c d|r|/dt) along e = r/|r|, r = load - hook — so the
    cable may go slack and the load fly free (a toss) and the jerk when it tightens again is in the prediction:
        m_Q a_Q = f b3 - c_d v + m_Q g + T e,      m_L a_L = -T e - c_L v_L + m_L g,
    integrated with the simulator's semi-implicit Euler at its physics step (``substeps`` per decision).
    Each decision: roll it out under a reference (first the network's action held, as above), linearise along
    that trajectory (forward-mode autodiff), solve the QP, roll the solution out and linearise again
    (``scp_iters``), with a trust region around the reference. The crossing events are read from each new
    prediction, so the convex constraints follow the solution:
      * the load centre where it crosses a window plane (interpolated between predicted steps) or within its
        radius of the plane: inside the opening shrunk by its radius and ``margin_m``,
      * the cable (the segment hook -> load, as the referee sees it) where it cuts a window plane: inside the
        opening shrunk by ``margin_m``,
      * the drone within ``gate_band_m`` of the target's plane: inside the opening shrunk by its size, as above,
      * the load off the ground.
    No swing limit and no taut-cable assumption: swinging the load up, or throwing it, is left to the network."""

    def __init__(self, cfg: Mapping, num_envs: int, device, step_dt: float):
        s, c = cfg["slung"], cfg["controller"]
        cl = s.get("convex_layer") or {}
        self.g = float(cfg["sim"]["gravity"])
        self.device, self.E, self.dt = device, num_envs, step_dt
        self.m_tot, self.m_L = float(cfg["drone"]["mass"]), float(s["load_mass"])
        self.m_Q = self.m_tot - self.m_L
        self.L0 = float(s["cable_length"])
        # the cable in the model: the simulator's by default; a softer one with the same damping ratio (the same
        # rebound when it snaps tight) lets the model take longer steps (``substeps``)
        self.k = float(cl.get("model_stiffness") or s["cable_stiffness"])
        mu = self.m_L * self.m_Q / self.m_tot
        zeta = float(s["cable_damping"]) / (2.0 * math.sqrt(float(s["cable_stiffness"]) * mu))
        self.c = 2.0 * zeta * math.sqrt(self.k * mu)
        self.drag_L = float(s["load_drag"])
        self.off = torch.tensor([float(v) for v in s["attach"]], device=device)   # hook from the CoM (level drone)
        self.drag = float(cfg["drone"].get("linear_drag", 0.0))
        self.r_load = float(s["load_radius"])
        self.drone_half = [float(v) for v in cl.get("drone_half_m", [0.32, 0.10])]
        self.margin = float(cl.get("margin_m", 0.05))
        self.band = float(cl.get("gate_band_m", 0.35))
        self.iters = int(cl.get("iters", 40))
        self.rho = float(cl.get("rho", 1.0))
        self.w_future = float(cl.get("future_weight", 1.0))
        self.w_trust = float(cl.get("trust_weight", 0.5))
        self.scp_iters = max(1, int(cl.get("scp_iters", 2)))
        self.sub = max(1, int(cl.get("substeps", 10)))
        self.N = max(1, int(round(float(cl.get("horizon_s", 1.0)) / step_dt)))
        self.blk = max(1, int(cl.get("block", 3)))
        self.nb = math.ceil((self.N - 1) / self.blk) if self.N > 1 else 0
        self.n = 3 + 3 * self.nb
        sm = [float(v) for v in cfg["action"]["step_max"]]
        self.step_max = torch.tensor(sm, device=device)
        self.sW = torch.tensor([max(sm[:2]), max(sm[:2]), sm[2]], device=device)
        t = lambda k: torch.tensor([float(v) for v in c[k]], device=device)  # noqa: E731
        self.kp, self.kv, self.v_max = t("kp_pos"), t("kv"), t("v_max")
        self.a_max = float(c["a_max"])
        self.az = (-0.6 * self.g, 1.5 * self.a_max)
        self.tan_tilt = math.tan(math.radians(float(c["max_tilt_deg"])))
        self.k_rot, self.k_om = float(c["k_rot"][0]), float(c["k_omega"][0])
        J = cfg["drone"]["inertia"]
        self.tip = -float(s["attach"][2]) / (0.5 * (float(J[0]) + float(J[1]))) * float(cl.get("tip_scale", 1.0))
        self.f_max = float(cfg["drone"]["thrust_to_weight"]) * self.m_tot * self.g
        b = cfg["scene"]["bounds"]
        self.wp_z = (float(b["z_min"]) + 0.25, float(b["z_max"]) - 0.3)
        self.gvec = torch.tensor([0.0, 0.0, -self.g], device=device)
        D = torch.zeros(self.N, 3, self.n, device=device)        # world step at decision k from the block variables
        for k in range(1, self.N):
            j = 3 + 3 * ((k - 1) // self.blk)
            D[k, :, j:j + 3] = torch.diag(self.sW)
        self.D = D
        self._jac = torch.func.vmap(torch.func.jacfwd(self._step, argnums=(0, 1)))
        self.warm = None

    def reset(self, env_ids=None) -> None:
        if self.warm is None or env_ids is None:
            self.warm = None
            return
        for t in self.warm:
            t[env_ids] = 0.0

    # ------------------------------------------------------------------ the model
    def _acc(self, p, v, pL, vL, bh, bhd, w):
        """Accelerations of the drone, the load and the tilt, and the cable tension (any batch shape)."""
        vc = torch.maximum(torch.minimum(self.kp * (w - p), self.v_max), -self.v_max)
        ac = self.kv * (vc - v)
        axy = ac[..., :2]
        axy = axy * (self.a_max / axy.norm(dim=-1, keepdim=True).clamp_min(1e-9)).clamp_max(1.0)
        Fz = self.m_tot * (ac[..., 2:].clamp(*self.az) + self.g)
        Fh = self.m_tot * axy
        Fh = Fh * (Fz * self.tan_tilt / Fh.norm(dim=-1, keepdim=True).clamp_min(1e-9)).clamp_max(1.0)
        F = torch.cat((Fh, Fz), -1)
        b3 = torch.cat((bh, (1.0 - (bh * bh).sum(-1, keepdim=True)).clamp_min(1e-6).sqrt()), -1)
        f = (F * b3).sum(-1, keepdim=True).clamp(0.0, self.f_max)
        r = pL - (p + self.off)
        dist = r.norm(dim=-1, keepdim=True).clamp_min(1e-6)
        e = r / dist
        T = (self.k * (dist - self.L0) + self.c * ((vL - v) * e).sum(-1, keepdim=True)).clamp_min(0.0) \
            * (dist > self.L0)
        aQ = (f * b3 - self.drag * v + T * e) / self.m_Q + self.gvec
        aL = (-T * e - self.drag_L * vL) / self.m_L + self.gvec
        bhdd = -self.k_rot * (bh - Fh / F.norm(dim=-1, keepdim=True)) - self.k_om * bhd - self.tip * T * e[..., :2]
        return aQ, aL, bhdd

    def _step(self, X, w):
        """One decision with the waypoint held: ``substeps`` semi-implicit Euler steps (as the simulator).
        X = (p, v, load p, load v, b_h, b_h') world (any batch shape)."""
        p, v, pL, vL, bh, bhd = X[..., 0:3], X[..., 3:6], X[..., 6:9], X[..., 9:12], X[..., 12:14], X[..., 14:16]
        h = self.dt / self.sub
        for _ in range(self.sub):
            aQ, aL, bhdd = self._acc(p, v, pL, vL, bh, bhd, w)
            v = v + h * aQ
            p = p + h * v
            vL = vL + h * aL
            pL = pL + h * vL
            bhd = bhd + h * bhdd
            bh = bh + h * bhd
        return torch.cat((p, v, pL, vL, bh, bhd), -1)

    def _waypoint(self, p, u, M, k):
        st = (M @ u[:, :3, None])[..., 0] if k == 0 else (self.D[k] @ u[..., None])[..., 0]
        w = p + st
        return torch.cat((w[:, :2], w[:, 2:].clamp(*self.wp_z)), -1)

    def rollout(self, X0, u, M):
        """-> states (E,N+1,16), waypoints (E,N,3) under decision vector u (E,n)."""
        Xs, Ws = [X0], []
        X = X0
        for k in range(self.N):
            w = self._waypoint(X[:, :3], u, M, k)
            X = self._step(X, w)
            Xs.append(X)
            Ws.append(w)
        return torch.stack(Xs, 1), torch.stack(Ws, 1)

    def state(self, pos, vel, load_rel, load_vel, quat=None, omega_b=None):
        """(E,16) model state from the drone (pose, velocity, attitude and body rates; level when not given) and
        the cable sensor (load offset / velocity relative to the CoM, world)."""
        if quat is None:
            bh = torch.zeros_like(pos[:, :2])
            bhd = torch.zeros_like(bh)
        else:
            R = m3.quat_to_rotmat(quat)
            bh = R[:, :2, 2]
            w = torch.zeros_like(pos) if omega_b is None else omega_b
            bhd = (R @ torch.stack((w[:, 1], -w[:, 0], torch.zeros_like(w[:, 0])), -1)[..., None])[:, :2, 0]
        return torch.cat((pos, vel, pos + load_rel, vel + load_vel, bh, bhd), -1)

    # ------------------------------------------------------------------ the QP
    def _linearise(self, X0, u_ref, M):
        """Reference rollout and its sensitivities to u: X_k ~ Xr_k + S_k (u - u_ref)."""
        E, N, n = X0.shape[0], self.N, self.n
        Xr, Wr = self.rollout(X0, u_ref, M)
        with torch.inference_mode(False):                 # forward-mode AD; rsl_rl rolls out in inference mode
            Jx, Jw = self._jac(Xr[:, :-1].reshape(-1, 16).clone(), Wr.reshape(-1, 3).clone())
        Jx, Jw = Jx.view(E, N, 16, 16), Jw.view(E, N, 16, 3)
        A = Jx.clone()
        A[..., 0:3] = A[..., 0:3] + Jw                           # the waypoint moves with the drone: w = p + step
        D = self.D.expand(E, N, 3, n).clone()
        D[:, 0, :, :3] = M
        S = [torch.zeros(E, 16, n, device=X0.device)]
        for k in range(N):
            S.append(A[:, k] @ S[k] + Jw[:, k] @ D[:, k])
        return Xr, torch.stack(S, 1)

    def project(self, a, pos, vel, yaw, load_rel, load_vel, gates, quat=None, omega_b=None):
        """Same contract as SlungConvexLayer.project, plus the attitude (quaternion, body rates)."""
        E, dev, n = a.shape[0], a.device, self.n
        a3 = a[:, :3].clamp(-1.0, 1.0)
        c, s_ = torch.cos(yaw), torch.sin(yaw)
        Rz = torch.zeros(E, 3, 3, device=dev)
        Rz[:, 0, 0], Rz[:, 0, 1], Rz[:, 1, 0], Rz[:, 1, 1], Rz[:, 2, 2] = c, -s_, s_, c, 1.0
        M = Rz * self.step_max
        X0 = self.state(pos, vel, load_rel, load_vel, quat, omega_b)
        d_ref = (M @ a3[..., None])[..., 0] / self.sW
        x_ref = torch.cat((a3, d_ref.clamp(-1, 1).repeat(1, self.nb)), -1)   # the network's action held
        w = torch.ones(n, device=dev)
        w[3:] = self.w_future
        P = torch.diag_embed((w + self.w_trust).expand(E, n))
        u_ref = x_ref
        for _ in range(self.scp_iters):
            Xr, S = self._linearise(X0, u_ref, M)
            A, l, u = self._constraints(Xr, S, u_ref, gates)
            q = -w * x_ref - self.w_trust * u_ref
            warm = self.warm if self.warm is not None and self.warm[1].shape == l.shape else None
            x, z, y, _, _ = solve_qp(P, q, A, l, u, iters=self.iters, rho=self.rho, warm=warm, inverse=True)
            self.warm = (x.detach(), z.detach(), y.detach())
            u_ref = x.clamp(-1.0, 1.0)
        out = a.clone()
        out[:, :3] = u_ref[:, :3]
        changed = (out[:, :3] - a3).abs().amax(-1) > 1e-3
        return out, changed

    def _constraints(self, Xr, S, u_ref, gates):
        """Linear constraints l <= A u <= u around the reference (fixed row layout: unused rows get infinite
        bounds, so the ADMM warm start carries over)."""
        E, dev, n = Xr.shape[0], Xr.device, self.n
        rows, lo, hi = [], [], []

        def add(row, ref, lo_b, hi_b):                    # lo_b <= ref + row (u - u_ref) <= hi_b; row (E,K,n)
            v = ref - (row * u_ref[:, None, :]).sum(-1)
            rows.append(row)
            lo.append(lo_b - v)
            hi.append(hi_b - v)

        pQ, SQ = Xr[..., 0:3], S[:, :, 0:3]                      # (E,N+1,3), (E,N+1,3,n)
        pL, SL = Xr[..., 6:9], S[:, :, 6:9]
        pH = pQ + self.off
        full = lambda t, m: torch.where(m, t, torch.full_like(t, INF))   # noqa: E731
        add(SL[:, 1:, 2], pL[:, 1:, 2], torch.full_like(pL[:, 1:, 2], self.r_load + 0.05),   # load off the ground
            torch.full_like(pL[:, 1:, 2], INF))
        for (gc, gn, ghalf, gact, load_only) in gates:
            right, up = window_frame(gn)
            axes = ((right, ghalf[:, 0]), (up, ghalf[:, 1]))
            dist = lambda P: ((P - gc[:, None]) * gn[:, None]).sum(-1)   # noqa: E731
            dQ, dH, dL = dist(pQ), dist(pH), dist(pL)
            # load centre crossing between predicted steps k and k+1 (k = 0..N-1), interpolated
            cross = (dL[:, :-1] * dL[:, 1:] < 0) | ((dL[:, 1:] == 0) & (dL[:, :-1] != 0))
            sL = (dL[:, :-1] / (dL[:, :-1] - dL[:, 1:]).where(cross, torch.ones_like(dL[:, 1:]))).clamp(0, 1)
            Pc = pL[:, :-1] + sL[..., None] * (pL[:, 1:] - pL[:, :-1])
            Sc = SL[:, :-1] + sL[..., None, None] * (SL[:, 1:] - SL[:, :-1])
            touch = dL[:, 1:].abs() < self.r_load + self.margin
            # cable cutting the plane at predicted step k (1..N): hook and load on opposite sides
            cut = dH[:, 1:] * dL[:, 1:] < 0
            sK = (dH[:, 1:] / (dH[:, 1:] - dL[:, 1:]).where(cut, torch.ones_like(dL[:, 1:]))).clamp(0, 1)
            Pk = pH[:, 1:] + sK[..., None] * (pL[:, 1:] - pH[:, 1:])
            Sk = SQ[:, 1:] + sK[..., None, None] * (SL[:, 1:] - SQ[:, 1:])
            objs = [(Pc, Sc, cross, self.r_load), (pL[:, 1:], SL[:, 1:], touch, self.r_load), (Pk, Sk, cut, 0.0)]
            if not load_only:
                objs.append((pQ[:, 1:], SQ[:, 1:], dQ[:, 1:].abs() <= self.band, None))
            for P_, S_, m, size in objs:
                m = m & gact[:, None]
                for ax_i, (axis, half) in enumerate(axes):
                    row = (S_ * axis[:, None, :, None]).sum(2)                      # (E,N,n)
                    ref = ((P_ - gc[:, None]) * axis[:, None]).sum(-1)
                    sz = self.drone_half[ax_i] if size is None else size
                    lim = (half[:, None] - sz - self.margin).clamp_min(0.0).expand_as(ref)
                    add(row, ref, -full(lim, m), full(lim, m))
        A = torch.cat([r.reshape(E, -1, n) for r in rows], 1)
        l = torch.cat([v.reshape(E, -1) for v in lo], 1)
        u = torch.cat([v.reshape(E, -1) for v in hi], 1)
        A = torch.cat((A, torch.eye(n, device=dev).expand(E, n, n)), 1)
        l = torch.cat((l, -torch.ones(E, n, device=dev)), 1)
        u = torch.cat((u, torch.ones(E, n, device=dev)), 1)
        return A, l, u


class SlungLayerFilter:
    """The convex layer fed from the core with what the drone has on board (own state, cable sensor, window
    estimates): in the env loop (slung.convex_layer.in_env) and behind any planner (SlungLayerPlanner). Besides the
    target window it keeps the window the drone has just flown through until the load is through as well."""

    def __init__(self, cfg: Mapping, num_envs: int, device, step_dt: float):
        model = ((cfg["slung"].get("convex_layer") or {}).get("model") or "linear")
        self.layer = (SlungSCPLayer if model == "scp" else SlungConvexLayer)(cfg, num_envs, device, step_dt)
        self.keep_prev = model == "scp"           # its window constraints are event based: free while far away
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
        if self.keep_prev:                        # scp: the long cable can swing the load back through it
            owes = self.prev >= 0
        self.prev = torch.where(owes, self.prev, torch.full_like(self.prev, -1))
        tgt_ok = core.selector.has & g(est.valid, sel)
        gates = [(g(est.center, sel), g(est.normal, sel), g(est.size, sel) / 2, tgt_ok, False),
                 (prev_c, prev_n, g(est.size, self.prev) / 2, owes & g(est.valid, self.prev), True)]
        out, self.changed = self.layer.project(a, st.pos, st.vel_w, core.yaw, sl.pos - st.pos, sl.vel - st.vel_w,
                                               gates, st.quat, st.omega_b)
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
