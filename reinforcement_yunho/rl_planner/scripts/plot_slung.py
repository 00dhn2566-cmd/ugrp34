"""Side view of one slung-load flight (RESULTS.md §14): the drone, the cable and the load through the windows,
one row per planner on the same scene — e.g. the network alone and behind the convex layer.

    python rl_planner/scripts/plot_slung.py --policy <run>/export --level 4 --seed 3 --png slung_side.png \
        [--planner rl+safe --planner rl+safe+layer]

The rig is configs/slung/base.yaml (more yamls with --config). Horizontal axis: x along the course; vertical:
height, true to scale. Each window is its wall (grey, floor to ceiling) with the opening left open, drawn at its
centre's x. Drone path blue, load path brown; every ``--every`` s the airframe (side view, prop tip to prop tip),
the cable and the load are drawn; the end is marked green (course done) or red with the crash reason. Same seeded
CPU scenes as plot_course.py.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

SLUNG = os.path.join(os.path.dirname(HERE), "configs", "slung", "base.yaml")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", required=True, help="export dir")
    ap.add_argument("--planner", action="append", default=[], help="rl | rl+safe | rl+safe+layer (repeatable)")
    ap.add_argument("--config", action="append", default=[], help="yaml merged after configs/slung/base.yaml")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--level", type=int, default=4)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--every", type=float, default=0.6, help="draw the airframe, cable and load every this many s")
    ap.add_argument("--png", required=True)
    ap.add_argument("--title", default=None)
    a = ap.parse_args(argv)
    specs = a.planner or ["rl+safe", "rl+safe+layer"]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import torch
    from matplotlib.patches import Circle

    from rl_planner.core import math3d as m3
    from rl_planner.core.airframe import Airframe
    from rl_planner.core.config import config_for_policy
    from rl_planner.core.mission import CRASH_NAMES
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.convex_layer import SlungLayerPlanner
    from rl_planner.planners.policy import PolicyPlanner
    from rl_planner.planners.safety import SafePlanner
    torch.set_num_threads(4)
    with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
        cfg = config_for_policy(json.load(f)["config"], [SLUNG] + a.config, ["device=cpu"] + a.set)
    af = Airframe(cfg, "cpu")
    tip = float(af.rotors[:, 0].abs().max()) + af.rho                # prop tip along the body x axis (side view)

    runs = []
    for spec in specs:
        env = TorchWindowEnv(cfg, num_envs=1, device="cpu", seed=a.seed)
        core = env.core
        core.fixed_level = a.level
        core.reseed(a.seed)
        env._reset_all()
        base, _, guard = spec.partition("+")
        if base != "rl":
            raise SystemExit(f"unknown planner {spec!r}")
        planner = PolicyPlanner(a.policy, device="cpu", cfg=cfg)
        guards = guard.split("+") if guard else []
        if "safe" in guards:
            planner = SafePlanner(planner, cfg, 1, "cpu", core.step_dt)
        if "layer" in guards:
            planner = SlungLayerPlanner(planner, cfg, 1, "cpu", core.step_dt)
        planner.reset(torch.arange(1))
        sc = core.scene
        n = int(sc.n_windows[0])
        win = (sc.center[0, :n].numpy().copy(), sc.size[0, :n].numpy().copy())
        log = []                                   # t, drone x z, body x-axis (x, z), hook x z, load x z
        for _ in range(core.max_episode_length):
            s, sl = core.state, core.slung
            bx = m3.quat_to_rotmat(s.quat)[0, :, 0]
            hook, _ = sl.hook(s.pos, s.quat)
            log.append((float(core.episode_time[0]), float(s.pos[0, 0]), float(s.pos[0, 2]), float(bx[0]),
                        float(bx[2]), float(hook[0, 0]), float(hook[0, 2]), float(sl.pos[0, 0]), float(sl.pos[0, 2])))
            _, _, d, _ = env.step(planner.act(core))
            if bool(d[0]):
                break
        rec = core.episode_record
        crash = bool(rec["crashed"][0])
        outcome = "course done" if bool(rec["success"][0]) else (
            f"crash: {CRASH_NAMES[int(rec['crash_reason'][0])]}" if crash else "timeout")
        summary = f"{spec}: {int(rec['windows_passed'][0])}/{n} windows, {float(rec['time_s'][0]):.1f} s, {outcome}"
        print(f"[plot] {summary}")
        runs.append((summary, np.array(log), win, bool(rec["success"][0]), crash, core.slung.radius))
        env.close()

    z_top = float(cfg["scene"]["bounds"]["z_max"])
    x_lo = min(min(r[1][:, 1].min(), r[2][0][:, 0].min()) for r in runs) - 0.8
    x_hi = max(max(r[1][:, 1].max(), r[2][0][:, 0].max()) for r in runs) + 0.8
    k = min(0.62, 14.0 / (x_hi - x_lo))            # inches per metre, both axes (true to scale)
    fig, axes = plt.subplots(len(runs), 1, figsize=(1.2 + (x_hi - x_lo) * k, 0.5 + (z_top * k + 0.5) * len(runs)),
                             sharex=True, squeeze=False)
    for ax, (summary, L, (C, S), ok, crash, r_load) in zip(axes[:, 0], runs):
        for c, sz in zip(C, S):                     # wall with the opening
            lo, hi = c[2] - sz[1] / 2, c[2] + sz[1] / 2
            ax.plot([c[0], c[0]], [0.0, lo], "-", color="#9aa3a9", lw=5, solid_capstyle="butt")
            ax.plot([c[0], c[0]], [hi, z_top], "-", color="#9aa3a9", lw=5, solid_capstyle="butt")
        ax.plot(L[:, 1], L[:, 2], "-", color="#2b6cb0", lw=1.4, label="drone")
        ax.plot(L[:, 7], L[:, 8], "-", color="#8a6a3b", lw=1.2, label="load")
        step = max(1, int(round(a.every / (L[1, 0] - L[0, 0])))) if len(L) > 1 else 1
        for i in list(range(0, len(L), step)) + [len(L) - 1]:
            t, x, z, bxx, bxz, hx, hz, lx, lz = L[i]
            ax.plot([x - tip * bxx, x + tip * bxx], [z - tip * bxz, z + tip * bxz], "-", color="#1f2a30", lw=1.6,
                    solid_capstyle="round", zorder=5)
            ax.plot([hx, lx], [hz, lz], "-", color="#2c3e50", lw=0.7, zorder=4)
            ax.add_patch(Circle((lx, lz), r_load, facecolor="#c8a064", edgecolor="#7d5a2c", lw=0.6, alpha=0.85,
                                zorder=4))
        end = "#1e8449" if ok else "#c0392b"
        ax.plot(L[-1, 7] if crash and "load" in summary else L[-1, 1],
                L[-1, 8] if crash and "load" in summary else L[-1, 2], "X", color=end, ms=9, zorder=6)
        ax.set_title(summary, fontsize=9, loc="left")
        ax.set_aspect("equal")
        ax.set_ylim(0.0, z_top)
        ax.set_xlim(x_lo, x_hi)
        ax.set_ylabel("z [m]")
        ax.grid(alpha=0.2)
    axes[0, 0].legend(loc="upper left", fontsize=8, frameon=False, ncol=2)
    axes[-1, 0].set_xlabel("x [m]")
    if a.title:
        fig.suptitle(a.title, fontsize=10)
    fig.tight_layout()
    fig.savefig(a.png, dpi=120)
    plt.close(fig)
    print(f"[plot] wrote {a.png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
