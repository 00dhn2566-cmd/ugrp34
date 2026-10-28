"""Top view of one look-around flight (RESULTS.md §12): the course, the path flown and where the camera looked.

    python rl_planner/scripts/plot_course.py --policy <run>/export --config rl_planner/configs/scan/long50.yaml \
        --level 4 --seed 3 --png course.png [--gif course.gif --gif-seconds 60]

Windows are drawn as their wall panels (thick: the opening) with an arrow along the pass direction, numbered in
course order (= the nearest-exit order). The path is coloured by what the drone was doing: orange while it turns
faster than 40 deg/s (looking around), blue otherwise. The GIF follows the drone, drawn true to scale as
the airframe from above (core/airframe.py: red front arms, propeller discs), with the camera's
horizontal field of view drawn as a wedge, so the look-around shows as the wedge sweeping. Same seeded scenes as
benchmark.py (CPU). ``--planner`` picks a scripted baseline instead of a policy (fsmscan, or <policy>:scan with --policy).
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

COURSE = os.path.join(os.path.dirname(HERE), "configs", "scan", "course.yaml")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", default=None, help="export dir")
    ap.add_argument("--planner", default="rl", help="rl | scan (policy behind the scripted look-around) | fsmscan")
    ap.add_argument("--config", action="append", default=[], help="yaml merged after scan/course.yaml")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--level", type=int, default=4)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--safe", action="store_true")
    ap.add_argument("--layer", action="store_true", help="behind the slung-load convex layer (planners/convex_layer.py)")
    ap.add_argument("--png", default=None)
    ap.add_argument("--gif", default=None)
    ap.add_argument("--gif-seconds", type=float, default=60.0)
    ap.add_argument("--gif-every", type=int, default=2, help="one frame every N decisions")
    ap.add_argument("--title", default=None)
    a = ap.parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import torch
    from matplotlib.patches import Circle

    from rl_planner.core.airframe import Airframe
    from rl_planner.core.config import config_for_policy, load_config
    from rl_planner.core.mission import CRASH_NAMES
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.policy import PolicyPlanner
    from rl_planner.planners.safety import SafePlanner
    torch.set_num_threads(4)
    extra = [COURSE] + a.config
    if a.policy:
        with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
            cfg = config_for_policy(json.load(f)["config"], extra, ["device=cpu"] + a.set)
    else:
        cfg = load_config(extra, ["device=cpu"] + a.set)
    env = TorchWindowEnv(cfg, num_envs=1, device="cpu", seed=a.seed)
    core = env.core
    core.fixed_level = a.level
    core.reseed(a.seed)
    env._reset_all()
    if a.planner == "fsmscan":
        from rl_planner.planners.fsm import FSMPlanner
        from rl_planner.planners.scan import ScanSupervisor
        planner = ScanSupervisor(FSMPlanner(cfg, 1, "cpu", follow_selector=True), cfg, 1, "cpu", core.step_dt)
    else:
        planner = PolicyPlanner(a.policy, device="cpu", cfg=cfg)
        if a.planner == "scan":
            from rl_planner.planners.scan import ScanSupervisor
            planner = ScanSupervisor(planner, cfg, 1, "cpu", core.step_dt)
    if a.safe:
        planner = SafePlanner(planner, cfg, 1, "cpu", core.step_dt)
    if a.layer:
        from rl_planner.planners.convex_layer import SlungLayerPlanner
        planner = SlungLayerPlanner(planner, cfg, 1, "cpu", core.step_dt)
    planner.reset(torch.arange(1))

    sc = core.scene
    n = int(sc.n_windows[0])
    C = sc.center[0, :n].numpy().copy()               # copies: the env resamples the scene in place when it resets
    N = sc.normal[0, :n].numpy().copy()
    Rt = sc.right[0, :n].numpy().copy()
    half = sc.size[0, :n, 0].numpy().copy() / 2
    panel = min(float(cfg["scene"]["wall_half_extent"]), 3.0)
    hfov = math.atan(float(cfg["perception"]["camera"]["width"]) / 2 / float(cfg["perception"]["camera"]["fx"]))
    log = []                                          # t, x, y, z, yaw, has target, target (referee)
    for _ in range(core.max_episode_length):
        p = core.state.pos[0]
        lp = core.slung.pos[0] if core.slung is not None else p
        log.append((float(core.episode_time[0]), float(p[0]), float(p[1]), float(p[2]), float(core.yaw[0]),
                    bool(core.selector.has[0]), int(core.mission.target[0]), float(lp[0]), float(lp[1])))
        _, _, d, _ = env.step(planner.act(core))
        if bool(d[0]):
            break
    rec = core.episode_record
    outcome = "success" if bool(rec["success"][0]) else (
        f"crash ({CRASH_NAMES[int(rec['crash_reason'][0])]})" if bool(rec["crashed"][0]) else "timeout")
    summary = (f"{int(rec['windows_passed'][0])}/{n} windows, {float(rec['time_s'][0]):.1f} s, {outcome}")
    print(f"[plot] {summary}")
    L = np.array([r[:5] for r in log])
    has = np.array([r[5] for r in log])
    yaw_rate = np.abs(np.diff(np.unwrap(L[:, 4]), prepend=L[0, 4])) / core.step_dt
    looking = yaw_rate > math.radians(40.0)
    name = a.title or (os.path.basename(os.path.dirname(a.policy.rstrip("/"))).split("_")[-1] if a.policy else a.planner)

    def draw_course(ax, upto=None, focus=None):
        tgt = log[upto][6] if upto is not None else n
        for k in range(n):
            c, r, nn = C[k], Rt[k], N[k]
            col = "#9aa3a9" if k < tgt else ("#d9661f" if k == tgt else "#2b6cb0")
            ax.plot([c[0] - panel * r[0], c[0] + panel * r[0]], [c[1] - panel * r[1], c[1] + panel * r[1]], "-",
                    color=col, lw=1.5, alpha=0.6, solid_capstyle="butt")
            ax.plot([c[0] - half[k] * r[0], c[0] + half[k] * r[0]], [c[1] - half[k] * r[1], c[1] + half[k] * r[1]],
                    "-", color=col, lw=4, solid_capstyle="butt")
            ax.annotate("", xy=(c[0] - 0.7 * nn[0], c[1] - 0.7 * nn[1]), xytext=(c[0], c[1]),
                        arrowprops=dict(arrowstyle="->", color=col, lw=1.0))
            if focus is None or np.hypot(*(c[:2] - focus)) < 9:
                ax.text(c[0] + 0.25, c[1] + 0.25, str(k + 1), fontsize=7 if focus is None else 9, color=col)

    af = Airframe(cfg, "cpu")
    body_half = float(af.box_a[af.box_names.index("body"), 0]) if "body" in af.box_names else 0.1

    def draw_drone(ax, x, y, yw):
        """The airframe from above, true to scale: red front arms, propeller discs, body, camera."""
        c, s = math.cos(yw), math.sin(yw)
        rot = lambda px, py: (x + c * px - s * py, y + s * px + c * py)  # noqa: E731
        for a in af.angles:
            ex, ey = rot(af.rotor_radius * math.cos(a), af.rotor_radius * math.sin(a))
            ax.plot([x, ex], [y, ey], "-", color="#c0392b" if math.cos(a) > 0 else "#7b8a8b", lw=1.8,
                    solid_capstyle="round", zorder=5)
            ax.add_patch(Circle((ex, ey), af.rho, facecolor="#5d6d7e", edgecolor="#2c3e50", alpha=0.45, lw=0.6,
                                zorder=6))
        sq = np.array([rot(px, py) for px, py in ((body_half, body_half), (-body_half, body_half),
                                                   (-body_half, -body_half), (body_half, -body_half))])
        ax.fill(sq[:, 0], sq[:, 1], color="#1f2a30", zorder=7)
        tri = np.array([rot(body_half + 0.09, 0.0), rot(body_half, 0.05), rot(body_half, -0.05)])
        ax.fill(tri[:, 0], tri[:, 1], color="#c0392b", zorder=7)

    def draw_path(ax, upto):
        seg = L[:upto + 1]
        for i in range(1, len(seg)):
            ax.plot(seg[i - 1:i + 1, 1], seg[i - 1:i + 1, 2], "-", color="#e67e22" if looking[i] else "#2b6cb0",
                    lw=2.6 if looking[i] else 1.6)

    if a.png:
        span = np.ptp(np.concatenate((C[:, :2], L[:, 1:3])), axis=0)
        fig, ax = plt.subplots(figsize=(min(18, 2 + span[0] / 4), min(9, 2 + span[1] / 4)))
        draw_course(ax)
        draw_path(ax, len(L) - 1)
        ax.plot(L[0, 1], L[0, 2], "ko", ms=5)
        ax.plot(L[-1, 1], L[-1, 2], "X", color="#c0392b" if outcome != "success" else "#1e8449", ms=9)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(f"{name}: {summary}  (orange = turning to look around)", fontsize=10)
        fig.tight_layout()
        fig.savefig(a.png, dpi=110)
        plt.close(fig)
        print(f"[plot] wrote {a.png}")
    if a.gif:
        from PIL import Image
        frames = []
        last = min(len(L) - 1, int(a.gif_seconds / core.step_dt))
        fig, ax = plt.subplots(figsize=(6.4, 4.8), dpi=80)
        for i in range(0, last + 1, a.gif_every):
            ax.clear()
            x, y, z, yw = L[i, 1], L[i, 2], L[i, 3], L[i, 4]
            draw_course(ax, upto=i, focus=np.array([x, y]))
            draw_path(ax, i)
            wedge = np.array([[x, y]] + [[x + 4.5 * math.cos(yw + s), y + 4.5 * math.sin(yw + s)]
                                         for s in np.linspace(-hfov, hfov, 12)])
            ax.fill(wedge[:, 0], wedge[:, 1], color="#f5b041", alpha=0.25, lw=0)
            if core.slung is not None:                   # the load on its cable, from above
                lx, ly = log[i][7], log[i][8]
                ax.plot([x, lx], [y, ly], "-", color="#2c3e50", lw=1.0, zorder=4)
                ax.add_patch(Circle((lx, ly), core.slung.radius, facecolor="#c8a064", edgecolor="#7d5a2c", lw=0.8,
                                    zorder=4))
            draw_drone(ax, x, y, yw)
            ax.set_xlim(x - 7, x + 7)
            ax.set_ylim(y - 5.25, y + 5.25)
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])
            state = "looking around" if (looking[i] or not has[i]) else "flying to the nearest window"
            ax.set_title(f"{name}  t={L[i, 0]:.1f}s  passed {log[i][6]}/{n}  z={z:.1f} m  {state}", fontsize=9)
            fig.canvas.draw()
            frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()))
        plt.close(fig)
        frames[0].save(a.gif, save_all=True, append_images=frames[1:], duration=int(100 * a.gif_every), loop=0,
                       optimize=True)
        print(f"[plot] wrote {a.gif} ({len(frames)} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
