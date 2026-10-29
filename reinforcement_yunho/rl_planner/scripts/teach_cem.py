"""Privileged teacher for the long cable (RESULTS.md §15): robust passes found by CEM over open-loop waypoint actions
in the torch env itself (true window and state, nominal model — no domain randomisation) for random scenes. These are
the demonstrations scripts/bc_warmstart.py distils into the policy.

A batch holds S scenes x P samples; every sample of a scene shares it (one draw from the level's geometry, restored
after every reset). A robust pass: drone and load through the first window and nothing hit until the end of the
horizon — a second window 8 m on keeps the episode running past the first (a pass that ends just before the load
swings back into the wall is not one). Among robust passes the score prefers room: the smaller of the referee's
clearance around the airframe and the load's room in the opening (core/mission.py).

    python rl_planner/scripts/teach_cem.py --config C/train/general.yaml --config C/slung/base.yaml \
        --config C/slung/long.yaml --config C/drone/agile.yaml --level 0 --scenes 8 --pop 512 --batches 4 --out DIR
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

SCENE_KEYS = ("n_windows", "valid", "center", "normal", "right", "up", "size", "corners", "color", "start")
DRONE_HALF_H, DRONE_HALF_SPAN = 0.10, 0.35     # FX450 body half height, arm + prop reach [m] (score shaping only)


class FixedScenes:
    """Pins env i to scene (i // P) * P of a snapshot across resets (the env draws new scenes on every reset)."""

    def __init__(self, env, P: int):
        self.env, self.core, self.P = env, env.core, P
        self._orig = env._reset_all
        env._reset_all = self.reset
        self.redraw()

    def redraw(self) -> None:
        self._orig()
        sc = self.core.scene
        idx = (torch.arange(self.core.E, device=sc.center.device) // self.P) * self.P
        self.snap = {k: getattr(sc, k)[idx].clone() for k in SCENE_KEYS}
        self.reset()

    def reset(self) -> None:
        self._orig()
        core, sc = self.core, self.core.scene
        for k, v in self.snap.items():
            getattr(sc, k).copy_(v)
        m, s = core.mission, core.state
        m.prev_d = ((s.pos[:, None, :] - sc.center) * sc.normal).sum(-1)
        if core.slung is not None:
            m.prev_dL = ((core.slung.pos[:, None, :] - sc.center) * sc.normal).sum(-1)

    def scene(self, i: int) -> dict:
        return {k: v[i].cpu() for k, v in self.snap.items()}


def rollout(env, acts: torch.Tensor) -> dict:
    """acts (E,T,3) open loop from a reset -> per-env outcome (first window along +x, as the line layout builds it)."""
    core = env.core
    env._reset_all()
    E, T, _ = acts.shape
    sc, m = core.scene, core.mission
    rL = core.slung.radius
    xw, yw, zc = sc.center[:, 0, 0].clone(), sc.center[:, 0, 1].clone(), sc.center[:, 0, 2].clone()
    hw, hh = sc.size[:, 0, 0] / 2, sc.size[:, 0, 1] / 2
    zreq, ztop = zc - hh + rL, zc + hh - DRONE_HALF_H - 0.05
    dev = acts.device
    fin = torch.zeros(E, dtype=torch.bool, device=dev)
    crash = torch.zeros_like(fin)
    reason = torch.zeros(E, dtype=torch.long, device=dev)
    passed = torch.zeros(E, dtype=torch.long, device=dev)
    o = {k: torch.full((E,), v, device=dev) for k, v in
         (("maxQ", -9.0), ("lastL", -9.0), ("hL", -1.0), ("hQ", -1.0), ("latQ", 0.0), ("latL", 0.0))}
    orig = core.after_substep

    def hook():
        live = ~(m.crashed | m.success) & ~fin
        orig()
        pQ, pL = core.state.pos, core.slung.pos
        near = (pL[:, 0] - xw).abs() < 0.3
        o["hL"].copy_(torch.where(live & near, torch.maximum(o["hL"], pL[:, 2] - zreq), o["hL"]))
        o["latL"].copy_(torch.where(live & near, torch.maximum(o["latL"], ((pL[:, 1] - yw).abs() - (hw - rL)).clamp_min(0)),
                                    o["latL"]))
        closing = (pQ[:, 0] > xw - 0.6) & (pQ[:, 0] < xw + 0.1)
        o["hQ"].copy_(torch.where(live & closing, torch.maximum(o["hQ"], (ztop - pQ[:, 2]).clamp(-1.0, 0.05)), o["hQ"]))
        o["latQ"].copy_(torch.where(live & closing, torch.maximum(
            o["latQ"], ((pQ[:, 1] - yw).abs() - (hw - DRONE_HALF_SPAN)).clamp_min(0)), o["latQ"]))
        o["maxQ"].copy_(torch.where(live, torch.maximum(o["maxQ"], pQ[:, 0]), o["maxQ"]))
        o["lastL"].copy_(torch.where(live, pL[:, 0], o["lastL"]))

    core.after_substep = hook
    try:
        for t in range(T):
            _, _, d, _ = env.step(acts[:, t])
            d = d.bool()
            rec = core.episode_record
            new = d & ~fin
            crash |= new & rec["crashed"].bool()
            reason = torch.where(new, rec["crash_reason"].long(), reason)
            passed = torch.where(new, rec["windows_passed"].long(), passed)
            fin |= d
            passed = torch.where(~fin, m.target.long(), passed)
            if bool(fin.all()):
                break
    finally:
        core.after_substep = orig
    succ = ~fin & (m.target >= 1) & m.load_past[:, 0]
    room = torch.minimum(m.clearance[:, 0], m.load_clear[:, 0])
    return dict(succ=succ, crash=crash, reason=reason, passed=passed, pQ=o["maxQ"] - xw, pLe=o["lastL"] - xw,
                hL=o["hL"], hQ=o["hQ"], lat=o["latQ"] + o["latL"], room=torch.where(succ, room, torch.zeros_like(room)))


def score(o: dict) -> torch.Tensor:
    base = (5.0 * o["pQ"].clamp(-3.0, 0.6) + 8.0 * o["pLe"].clamp(-3.0, 0.6) + 20.0 * o["hL"].clamp(-1.0, 0.1)
            + 10.0 * o["hQ"] + 10.0 * (o["passed"] >= 1).float() - 5.0 * o["crash"].float() - 20.0 * o["lat"].clamp(max=1.0))
    return torch.where(o["succ"], 100.0 + 100.0 * o["room"].clamp(max=0.15), base)


def policy_actions(env, export_dir: str, T: int) -> torch.Tensor:
    """The deterministic policy's own actions in the pinned scenes (a CEM mean to start from)."""
    from rl_planner.planners.policy import PolicyPlanner
    pol = PolicyPlanner(export_dir, device=env.core.device)
    env._reset_all()
    pol.reset(torch.arange(env.core.E))
    out = []
    for _ in range(T):
        a = pol.act(env.core).clamp(-1.0, 1.0)
        out.append(a)
        _, _, d, _ = env.step(a)
        if d.any():
            pol.reset(torch.nonzero(d).squeeze(-1))
    return torch.stack(out, 1)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", action="append", required=True)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--level", type=int, default=0)
    ap.add_argument("--opening-h", type=float, nargs=2, default=None, help="override the level's opening height range")
    ap.add_argument("--scenes", type=int, default=8, help="scenes per batch (S)")
    ap.add_argument("--pop", type=int, default=512, help="samples per scene (P)")
    ap.add_argument("--batches", type=int, default=1)
    ap.add_argument("--T", type=int, default=80, help="horizon [decisions]")
    ap.add_argument("--iters", type=int, default=50)
    ap.add_argument("--min-iters", type=int, default=20, help="stop a batch after this once every scene has a robust pass")
    ap.add_argument("--elite", type=float, default=0.03)
    ap.add_argument("--sig0", type=float, default=0.6)
    ap.add_argument("--policy", default=None, help="export dir: start each scene's mean at the policy's own actions")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    from rl_planner.core.config import load_config
    from rl_planner.envs.torch_env import TorchWindowEnv
    over = "n_windows: [2, 2], spacing: [8.0, 8.0]" + (f", opening_h: [{a.opening_h[0]}, {a.opening_h[1]}]" if a.opening_h else "")
    sets = ["domain_randomization.enabled=false", "drone.start_pos_noise=[0.0, 0.0, 0.0]", f"curriculum.fixed_level={a.level}",
            f"curriculum.override={{{over}}}", "slung.convex_layer.in_env=false", f"device={a.device}"] + a.set
    cfg = load_config(a.config, sets)
    S, P, T = a.scenes, a.pop, a.T
    E = S * P
    torch.manual_seed(a.seed)
    env = TorchWindowEnv(cfg, num_envs=E, device=a.device, seed=a.seed)
    env.core.fixed_level = a.level
    fixed = FixedScenes(env, P)
    os.makedirs(a.out, exist_ok=True)
    dev = torch.device(a.device)
    g = torch.arange(E, device=dev) // P
    k = max(8, int(a.elite * P))
    for b in range(a.batches):
        if b:
            fixed.redraw()
        t0 = time.time()
        mu = torch.zeros(S, T, 3, device=dev)
        mu[..., 0] = 0.3
        sig = torch.full((S, T, 3), a.sig0, device=dev)
        sig[..., 1] = a.sig0 / 2
        if a.policy:
            mu = policy_actions(env, a.policy, T)[::P].clone()
            sig.fill_(a.sig0 / 2)
        best = mu.clone()
        best_s = torch.full((S,), -1e9, device=dev)
        best_room = torch.zeros(S, device=dev)
        best_ok = torch.zeros(S, dtype=torch.bool, device=dev)
        for it in range(a.iters):
            eps = torch.randn(E, T, 3, device=dev)
            for t in range(1, T):                   # coloured noise: coherent manoeuvres
                eps[:, t] = 0.6 * eps[:, t - 1] + 0.8 * eps[:, t]
            acts = (mu[g] + sig[g] * eps).clamp(-1.0, 1.0)
            acts[::P] = best                         # elitism: each scene's best so far stays in the population
            o = rollout(env, acts)
            s = score(o).view(S, P)
            top = s.topk(k, dim=1).indices                                      # (S,k)
            el = acts.view(S, P, T, 3).gather(1, top[..., None, None].expand(-1, -1, T, 3))
            mu = 0.8 * el.mean(1) + 0.2 * mu
            sig = (0.8 * el.std(1) + 0.2 * sig).clamp_min(0.05)
            sb, ib = s.max(1)
            up = sb > best_s
            best = torch.where(up[:, None, None], acts.view(S, P, T, 3)[torch.arange(S, device=dev), ib], best)
            best_s = torch.where(up, sb, best_s)
            sel = torch.arange(S, device=dev) * P + ib
            best_room = torch.where(up, o["room"][sel], best_room)
            best_ok = torch.where(up, o["succ"][sel], best_ok)
            print(f"batch {b} it {it:2d} samples robust {o['succ'].float().mean():.3f} | scenes solved "
                  f"{int(best_ok.sum())}/{S} room {[round(float(x), 3) for x in best_room[best_ok]]} [{time.time() - t0:.0f}s]",
                  flush=True)
            if it + 1 >= a.min_iters and bool(best_ok.all()):
                break
        demos = [dict(scene=fixed.scene(i * P), actions=best[i].cpu(), robust=bool(best_ok[i]), room=float(best_room[i]),
                      score=float(best_s[i])) for i in range(S)]
        torch.save(dict(demos=demos, level=a.level, configs=a.config, sets=sets, T=T, args=vars(a)),
                   os.path.join(a.out, f"batch_{a.seed:03d}_{b:03d}.pt"))
        print(f"batch {b}: {int(best_ok.sum())}/{S} scenes with a robust pass [{time.time() - t0:.0f}s]", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
