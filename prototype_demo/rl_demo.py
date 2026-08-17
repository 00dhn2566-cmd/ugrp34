"""Evaluate the trained PPO policy and render the flight.

    python rl_demo.py --episodes 30

NOTE the policy was trained with ``step=0.3``; ``rl/train_pybullet.py``'s default is
0.6 and evaluating there reads as ~5% success instead of ~95%. That value is not
recorded anywhere in the repo, so it is pinned here.

Outputs to --out: flight GIF, trajectory plot, training curve.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TEAM = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(TEAM, "reinforcement_yunho"))

TRAIN_STEP = 0.3


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", default=os.path.join(HERE, "model", "ppo_window_3win.zip"))
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--n-windows", type=int, default=3)
    ap.add_argument("--clutter", type=int, default=18)
    ap.add_argument("--out", default="/home/yoonho/fig/6_render")
    ap.add_argument("--no-render", action="store_true")
    a = ap.parse_args(argv)

    if not os.path.exists(a.policy):
        print(f"[error] policy not found: {a.policy}", file=sys.stderr)
        return 2

    import pybullet as p
    from PIL import Image
    from stable_baselines3 import PPO
    from rl.pybullet_window_env import WindowTraversalAviary

    os.makedirs(a.out, exist_ok=True)
    model = PPO.load(a.policy, device="cpu")
    print(f"policy   {a.policy}")
    print(f"step     {TRAIN_STEP}  (training value — repo default 0.6 would read as ~5%)\n")

    def make(seed):
        return WindowTraversalAviary(n_windows=a.n_windows, seed=seed, step=TRAIN_STEP,
                                     pane=False, domain_match=True, clutter=a.clutter)

    # ---- batch evaluation ----------------------------------------------------
    env = make(90000)
    succ, passed = 0, []
    for e in range(a.episodes):
        obs, _ = env.reset(seed=90000 + e)
        term = trunc = False
        info = {"windows_passed": 0}
        while not (term or trunc):
            act, _ = model.predict(obs, deterministic=True)
            obs, _, term, trunc, info = env.step(act)
        succ += int(term)
        passed.append(info["windows_passed"])
    env.close()
    sr = succ / a.episodes
    print(f"episodes {a.episodes}")
    print(f"success  {sr:.1%}")
    print(f"passed   {np.mean(passed):.2f} / {a.n_windows} windows (mean)")

    if a.no_render:
        return 0

    # ---- pick a cleared seed and render it -----------------------------------
    chosen = None
    for s in range(40):
        env = make(s)
        obs, _ = env.reset(seed=s)
        term = trunc = False
        while not (term or trunc):
            act, _ = model.predict(obs, deterministic=True)
            obs, _, term, trunc, _ = env.step(act)
        env.close()
        if term:
            chosen = s
            break
    if chosen is None:
        print("\n(no fully-cleared seed in 0..39 — skipping render)")
        return 0
    print(f"\nrendering seed {chosen} ...")

    env = make(chosen)
    obs, _ = env.reset(seed=chosen)
    layout = env.window_layout
    mid = np.mean([w["center"] for w in layout], axis=0)
    R_BODY = 0.0397 + 0.023135

    def sphere(rgba, r, pos):
        v = p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=rgba,
                                physicsClientId=env.CLIENT)
        return p.createMultiBody(0, -1, v, pos, physicsClientId=env.CLIENT)

    drone = sphere([1, .55, 0, 1], R_BODY, [0, 0, -5])

    def shot(dist=3.8, yaw=42, pitch=-22, W=800, H=600):
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=mid.tolist(), distance=dist, yaw=yaw, pitch=pitch,
            roll=0, upAxisIndex=2, physicsClientId=env.CLIENT)
        proj = p.computeProjectionMatrixFOV(60, W / H, 0.05, 30, physicsClientId=env.CLIENT)
        _, _, rgb, _, _ = p.getCameraImage(W, H, view, proj, shadow=1,
                                           renderer=p.ER_TINY_RENDERER,
                                           physicsClientId=env.CLIENT)
        return Image.fromarray(np.asarray(rgb, np.uint8).reshape(H, W, 4)[:, :, :3])

    frames, traj, passed_at = [], [], []
    prev, term, trunc = 0, False, False
    while not (term or trunc):
        pos = env._getDroneStateVector(0)[0:3]
        traj.append(pos.copy())
        p.resetBasePositionAndOrientation(drone, pos, [0, 0, 0, 1], physicsClientId=env.CLIENT)
        if len(traj) % 4 == 0:
            sphere([.05, .05, .05, 1], 0.012, pos.tolist())
        act, _ = model.predict(obs, deterministic=True)
        obs, _, term, trunc, info = env.step(act)
        if info["windows_passed"] > prev:
            prev = info["windows_passed"]
            passed_at.append(len(traj))
        if len(traj) % 3 == 0:
            frames.append(shot())

    frames += [frames[-1]] * 8
    gif = os.path.join(a.out, "rl_flight.gif")
    frames[0].save(gif, save_all=True, append_images=frames[1:], duration=80, loop=0)
    shot(3.6, 42, -22).save(os.path.join(a.out, "rl_path_iso.png"))
    shot(3.4, 90, -6).save(os.path.join(a.out, "rl_path_side.png"))
    print(f"  {gif}  ({len(frames)} frames)")

    # ---- plots ---------------------------------------------------------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    traj = np.array(traj)
    cmap = {"red": "#d33", "green": "#1a1", "blue": "#36c"}
    curve = os.path.join(HERE, "model", "pass_rate.csv")
    ncol = 3 if os.path.exists(curve) else 2
    fig = plt.figure(figsize=(5 * ncol, 4.6))

    ax = fig.add_subplot(1, ncol, 1, projection="3d")
    ax.plot(*traj.T, lw=2.2, color="#222", label="PPO path")
    ax.scatter(*traj[0], s=55, color="k", label="start")
    for i in passed_at:
        ax.scatter(*traj[min(i, len(traj) - 1)], s=110, marker="*", color="#f90", zorder=6)
    for w in layout:
        c, ow, oh = w["center"], w["ow"], w["oh"]
        ys = [c[1] - ow/2, c[1] + ow/2, c[1] + ow/2, c[1] - ow/2, c[1] - ow/2]
        zs = [c[2] - oh/2, c[2] - oh/2, c[2] + oh/2, c[2] + oh/2, c[2] - oh/2]
        ax.plot([c[0]]*5, ys, zs, lw=3.5, color=cmap[w["color"]])
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_title(f"trained PPO — seed {chosen}", fontsize=10)
    ax.view_init(elev=16, azim=-62); ax.set_box_aspect((3, 2, 1.3))
    ax.legend(loc="upper left", fontsize=8)

    ax2 = fig.add_subplot(1, ncol, 2)
    ax2.plot(traj[:, 0], traj[:, 2], lw=2.2, color="#222")
    for i in passed_at:
        j = min(i, len(traj) - 1)
        ax2.scatter(traj[j, 0], traj[j, 2], s=130, marker="*", color="#f90", zorder=6)
    for w in layout:
        c, oh = w["center"], w["oh"]
        ax2.plot([c[0]]*2, [c[2]-oh/2, c[2]+oh/2], lw=5, color=cmap[w["color"]])
    ax2.set_xlabel("x [m]"); ax2.set_ylabel("z [m]")
    ax2.set_title("side view (x-z)", fontsize=10); ax2.grid(alpha=.3)

    if ncol == 3:
        rows = list(csv.DictReader(open(curve)))
        ax3 = fig.add_subplot(1, 3, 3)
        ax3.plot([int(r["timesteps"])/1e6 for r in rows],
                 [float(r["success_rate"])*100 for r in rows], "o-", lw=1.8, ms=4, color="#26c")
        ax3.axhline(sr*100, ls="--", color="#c33", lw=1.4, label=f"this eval {sr:.0%}")
        ax3.set_xlabel("timesteps [M]"); ax3.set_ylabel("success rate [%]")
        ax3.set_title("training curve", fontsize=10)
        ax3.set_ylim(0, 100); ax3.grid(alpha=.3); ax3.legend(fontsize=8)

    fig.suptitle(f"PPO window traversal — {a.n_windows} windows, CF2X, PyBullet "
                 f"(success {sr:.0%} over {a.episodes} episodes)", fontsize=12)
    fig.tight_layout()
    out_png = os.path.join(a.out, "rl_summary.png")
    fig.savefig(out_png, dpi=130)
    print(f"  {out_png}")
    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
