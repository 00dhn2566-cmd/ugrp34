"""Side view of one slung-load flight (RESULTS.md §14): the drone, the cable and the load through the windows,
one row per planner on the same scene — e.g. the network alone and behind the convex layer.

    python rl_planner/scripts/plot_slung.py --policy <run>/export --level 4 --seed 3 --png slung_side.png \
        [--planner rl+safe --planner rl+safe+layer]
    python rl_planner/scripts/plot_slung.py --policy g=<export> --policy s=<export> \
        --planner g:rl+safe --planner g:rl+safe+layer --planner s:rl+safe+layer --png cmp.png   # several policies
    ... --gif slung_side.gif [--fps 20]                  # the same rows played in real time (with or without --png)

The rig is configs/slung/base.yaml (more yamls with --config). Horizontal axis: x along the course; vertical:
height, true to scale. Each window is its wall (grey, floor to ceiling) with the opening left open, drawn at its
centre's x. Drone path blue, load path brown; every ``--every`` s the airframe (side view, prop tip to prop tip),
the cable and the load are drawn; the end is marked green (course done) or red with the crash reason. The GIF
draws the paths so far and the airframe, cable and load now, with the windows passed and the cable's swing from
vertical in each row's title; a row stops at its end (a crash: the physics step it happened). Every physics step
is logged. Same seeded CPU scenes as plot_course.py.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

SLUNG = os.path.join(os.path.dirname(HERE), "configs", "slung", "base.yaml")
GIF_PX = 1000                                       # GIF width [px]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", action="append", required=True, help="export dir, or name=export dir (repeatable)")
    ap.add_argument("--planner", action="append", default=[],
                    help="[name:]rl | rl+safe | rl+safe+layer (repeatable; no name: the first policy)")
    ap.add_argument("--config", action="append", default=[], help="yaml merged after configs/slung/base.yaml")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--level", type=int, default=4)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--every", type=float, default=0.6, help="draw the airframe, cable and load every this many s")
    ap.add_argument("--png", default=None)
    ap.add_argument("--gif", default=None, help="the same rows as an animation, real time")
    ap.add_argument("--fps", type=int, default=20,
                    help="GIF frame rate (GIF delays are whole 10 ms: 10, 20, 25, 50 play in real time)")
    ap.add_argument("--title", default=None)
    a = ap.parse_args(argv)
    if not (a.png or a.gif):
        ap.error("give --png and/or --gif")
    pols = dict(p.split("=", 1) if "=" in p else ("", p) for p in a.policy)
    first = next(iter(pols))
    specs = [s_.split(":", 1) if ":" in s_ else (first, s_) for s_ in (a.planner or ["rl+safe", "rl+safe+layer"])]

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
    cfgs = {}
    for name, path in pols.items():
        with open(os.path.join(path, "policy_meta.json"), encoding="utf-8") as f:
            cfgs[name] = config_for_policy(json.load(f)["config"], [SLUNG] + a.config, ["device=cpu"] + a.set)
    cfg = cfgs[first]
    af = Airframe(cfg, "cpu")
    tip = float(af.rotors[:, 0].abs().max()) + af.rho                # prop tip along the body x axis (side view)

    runs = []
    for name, spec in specs:
        cfg = cfgs[name]
        env = TorchWindowEnv(cfg, num_envs=1, device="cpu", seed=a.seed)
        core = env.core
        core.fixed_level = a.level
        core.reseed(a.seed)
        env._reset_all()
        base, _, guard = spec.partition("+")
        if base != "rl":
            raise SystemExit(f"unknown planner {spec!r}")
        planner = PolicyPlanner(pols[name], device="cpu", cfg=cfg)
        guards = guard.split("+") if guard else []
        if "safe" in guards:
            planner = SafePlanner(planner, cfg, 1, "cpu", core.step_dt)
        if "layer" in guards:
            planner = SlungLayerPlanner(planner, cfg, 1, "cpu", core.step_dt)
        planner.reset(torch.arange(1))
        sc = core.scene
        n = int(sc.n_windows[0])
        win = (sc.center[0, :n].numpy().copy(), sc.size[0, :n].numpy().copy())
        log = []       # every physics step: t, drone x z, body x-axis (x, z), hook x z, load x z, swing deg, passed

        def record():
            s, sl = core.state, core.slung
            bx = m3.quat_to_rotmat(s.quat)[0, :, 0]
            hook, _ = sl.hook(s.pos, s.quat)
            cab = sl.pos[0] - hook[0]
            swing = math.degrees(math.acos(min(1.0, max(-1.0, float(-cab[2] / cab.norm().clamp_min(1e-9))))))
            log.append((len(log) * core.dt, float(s.pos[0, 0]), float(s.pos[0, 2]), float(bx[0]), float(bx[2]),
                        float(hook[0, 0]), float(hook[0, 2]), float(sl.pos[0, 0]), float(sl.pos[0, 2]), swing,
                        int(core.mission.target[0])))

        def after_substep(sub=core.after_substep):    # the env calls it after every physics step
            live = not bool(core.mission.crashed[0])
            sub()
            if live:                                   # the crash step is the last one logged
                record()

        core.after_substep = after_substep
        record()
        for _ in range(core.max_episode_length):
            _, _, d, _ = env.step(planner.act(core))
            if bool(d[0]):
                break
        rec = core.episode_record
        reason = CRASH_NAMES[int(rec["crash_reason"][0])] if bool(rec["crashed"][0]) else None
        outcome = "course done" if bool(rec["success"][0]) else (f"crash: {reason}" if reason else "timeout")
        label = f"{name + ' ' if name else ''}{spec}"
        summary = f"{label}: {int(rec['windows_passed'][0])}/{n} windows, {float(rec['time_s'][0]):.1f} s, {outcome}"
        print(f"[plot] {summary}")
        runs.append(dict(label=label, summary=summary, L=np.array(log), win=win, n=n, ok=bool(rec["success"][0]),
                         at_load=reason in ("load", "cable"), r=core.slung.radius))
        env.close()

    z_top = float(cfg["scene"]["bounds"]["z_max"])
    x_lo = min(min(r["L"][:, 1].min(), r["win"][0][:, 0].min()) for r in runs) - 0.8
    x_hi = max(max(r["L"][:, 1].max(), r["win"][0][:, 0].max()) for r in runs) + 0.8
    k = min(0.62, 14.0 / (x_hi - x_lo))            # inches per metre, both axes (true to scale)

    def figure():                                   # one row per run: walls with their openings, axes
        fig, axes = plt.subplots(len(runs), 1, figsize=(1.2 + (x_hi - x_lo) * k, 0.5 + (z_top * k + 0.5) * len(runs)),
                                 sharex=True, squeeze=False)
        for ax, r in zip(axes[:, 0], runs):
            for c, sz in zip(*r["win"]):
                lo, hi = c[2] - sz[1] / 2, c[2] + sz[1] / 2
                ax.plot([c[0], c[0]], [0.0, lo], "-", color="#9aa3a9", lw=5, solid_capstyle="butt")
                ax.plot([c[0], c[0]], [hi, z_top], "-", color="#9aa3a9", lw=5, solid_capstyle="butt")
            ax.set_aspect("equal")
            ax.set_ylim(0.0, z_top)
            ax.set_xlim(x_lo, x_hi)
            ax.set_ylabel("z [m]")
            ax.grid(alpha=0.2)
        axes[-1, 0].set_xlabel("x [m]")
        if a.title:
            fig.suptitle(a.title, fontsize=10)
        return fig, axes[:, 0]

    def paths(ax, L):
        return (ax.plot(L[:, 1], L[:, 2], "-", color="#2b6cb0", lw=1.4, label="drone")[0],
                ax.plot(L[:, 7], L[:, 8], "-", color="#8a6a3b", lw=1.2, label="load")[0])

    def rig(ax, row, r_load):                       # airframe (prop tip to prop tip), cable and load at one instant
        _, x, z, bxx, bxz, hx, hz, lx, lz = row[:9]
        return [ax.plot([x - tip * bxx, x + tip * bxx], [z - tip * bxz, z + tip * bxz], "-", color="#1f2a30", lw=1.6,
                        solid_capstyle="round", zorder=5)[0],
                ax.plot([hx, lx], [hz, lz], "-", color="#2c3e50", lw=0.7, zorder=4)[0],
                ax.add_patch(Circle((lx, lz), r_load, facecolor="#c8a064", edgecolor="#7d5a2c", lw=0.6, alpha=0.85,
                                    zorder=4))]

    def mark_end(ax, r):                            # at the load when the load or the cable hit
        L = r["L"]
        ax.plot(L[-1, 7] if r["at_load"] else L[-1, 1], L[-1, 8] if r["at_load"] else L[-1, 2], "X",
                color="#1e8449" if r["ok"] else "#c0392b", ms=9, zorder=6)

    if a.png:
        fig, axes = figure()
        for ax, r in zip(axes, runs):
            L = r["L"]
            paths(ax, L)
            step = max(1, int(round(a.every / (L[1, 0] - L[0, 0])))) if len(L) > 1 else 1
            for i in list(range(0, len(L), step)) + [len(L) - 1]:
                rig(ax, L[i], r["r"])
            mark_end(ax, r)
            ax.set_title(r["summary"], fontsize=9, loc="left")
        axes[0].legend(loc="upper left", fontsize=8, frameon=False, ncol=2)
        fig.tight_layout()
        fig.savefig(a.png, dpi=120)
        plt.close(fig)
        print(f"[plot] wrote {a.png}")

    if a.gif:
        from PIL import Image
        fig, axes = figure()
        trails = [paths(ax, r["L"][:0]) for ax, r in zip(axes, runs)]
        axes[0].legend(loc="upper left", fontsize=8, frameon=False, ncol=2)
        for ax, r in zip(axes, runs):               # lay out once, for the longest titles
            ax.set_title(r["summary"], fontsize=9, loc="left")
        axes[0].set_title("t = 00.0 s", fontsize=9, loc="right")
        fig.tight_layout()
        fig.set_dpi(GIF_PX / fig.get_figwidth())
        dt = float(runs[0]["L"][1, 0] - runs[0]["L"][0, 0])
        t_end = max(float(r["L"][-1, 0]) for r in runs)
        ended = [False] * len(runs)
        frames = []
        for t in list(np.arange(0.0, t_end, 1.0 / a.fps)) + [t_end]:
            now = []
            for j, (ax, r, (td, tl)) in enumerate(zip(axes, runs, trails)):
                L = r["L"]
                i = min(int(round(t / dt)), len(L) - 1)
                td.set_data(L[:i + 1, 1], L[:i + 1, 2])
                tl.set_data(L[:i + 1, 7], L[:i + 1, 8])
                now += rig(ax, L[i], r["r"])
                if ended[j]:
                    continue
                if i == len(L) - 1:
                    ended[j] = True
                    mark_end(ax, r)
                    ax.set_title(r["summary"], fontsize=9, loc="left")
                else:
                    ax.set_title(f"{r['label']}: {int(L[i, 10])}/{r['n']} windows, swing {L[i, 9]:.0f}°", fontsize=9,
                                 loc="left")
            axes[0].set_title(f"t = {t:.1f} s", fontsize=9, loc="right")
            fig.canvas.draw()
            frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()))
            for art in now:
                art.remove()
        plt.close(fig)
        pal = frames[-1].quantize(colors=128)        # one palette for every frame (the last has every colour)
        frames = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
        frames[0].save(a.gif, save_all=True, append_images=frames[1:], loop=0,
                       duration=[round(1000 / a.fps)] * (len(frames) - 1) + [2500])
        print(f"[plot] wrote {a.gif} ({len(frames)} frames, {os.path.getsize(a.gif) / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
