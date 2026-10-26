"""Figures for a training run + its evaluations.

    python rl_planner/scripts/report.py --run <run_dir> [--eval <eval_dir> ...] [--out <dir>]

Writes PNGs (English labels — matplotlib has no Korean font here) into --out
(default <run_dir>/report): training curves, RL-vs-FSM bars per eval, and
top/side trajectory overlays of the recorded scenes.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

COL = {"fsm": "#d9822b", "rl": "#2b6cd9"}
WIN_COL = ["#e03131", "#2f9e44", "#1c7ed6"]


def read_scalars(run_dir: str) -> dict:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    files = sorted(glob.glob(os.path.join(run_dir, "events.out.tfevents.*")))
    out = {}
    for f in files:
        ea = EventAccumulator(f, size_guidance={"scalars": 0})
        ea.Reload()
        for tag in ea.Tags().get("scalars", []):
            ev = ea.Scalars(tag)
            out.setdefault(tag, []).extend((e.step, e.value) for e in ev)
    return {k: np.array(sorted(v)) for k, v in out.items()}


def smooth(y, k=10):
    if len(y) < k:
        return y
    c = np.convolve(y, np.ones(k) / k, mode="valid")
    return np.concatenate([np.full(k - 1, np.nan), c])


def plot_training(sc: dict, path: str) -> None:
    panels = [("Metrics/success_rate", "success rate (at curriculum frontier)"),
              ("Metrics/curriculum_level", "mean curriculum level"),
              ("Metrics/crash_rate", "crash rate"),
              ("Metrics/windows_passed_frac", "windows passed / episode"),
              ("Train/mean_reward", "mean episode return"),
              ("Metrics/episode_time_s", "episode time [s]")]
    fig, axes = plt.subplots(2, 3, figsize=(14, 7))
    for ax, (tag, title) in zip(axes.ravel(), panels):
        if tag in sc:
            x, y = sc[tag][:, 0], sc[tag][:, 1]
            ax.plot(x, y, color="#bbb", lw=0.8)
            ax.plot(x, smooth(y), color="#2b6cd9", lw=1.6)
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("iteration")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_eval_bars(evals: list, path: str) -> None:
    keys = [("success_rate", "success rate"), ("crash_rate", "crash rate"),
            ("mean_time_success_s", "time to finish [s]"), ("mean_clearance_m", "pass clearance [m]")]
    fig, axes = plt.subplots(1, 4, figsize=(15, 3.6))
    labels = [f"{e['meta']['backend']} L{e['meta']['level']}" for e in evals]
    x = np.arange(len(evals))
    for ax, (k, title) in zip(axes, keys):
        for j, p in enumerate(("fsm", "rl")):
            vals = [e["summary"].get(p, {}).get(k, np.nan) for e in evals]
            ax.bar(x + (j - 0.5) * 0.38, vals, width=0.38, color=COL[p], label=("state machine" if p == "fsm" else "RL policy"))
        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_title(title, fontsize=10)
        ax.grid(axis="y", alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_traj(eval_dir: str, path: str, max_scenes: int = 4) -> None:
    import torch
    tr = {p: torch.load(os.path.join(eval_dir, f"traj_{p}.pt")) for p in ("fsm", "rl")
          if os.path.exists(os.path.join(eval_dir, f"traj_{p}.pt"))}
    if not tr:
        return
    any_tr = next(iter(tr.values()))
    sc = any_tr["scene"]
    n = min(max_scenes, len(any_tr["paths"]))
    fig, axes = plt.subplots(2, n, figsize=(4.2 * n, 7), squeeze=False)
    for i in range(n):
        for row, (iy, lab) in enumerate(((1, "y [m]"), (2, "z [m]"))):
            ax = axes[row, i]
            nw = int(sc["n_windows"][i])
            for w in range(nw):
                c = sc["center"][i, w].numpy()
                nrm = sc["normal"][i, w].numpy()
                half = float(sc["size"][i, w, 0 if iy == 1 else 1]) / 2
                if iy == 1:
                    right = np.cross([0, 0, 1.0], nrm)
                    right /= np.linalg.norm(right) + 1e-9
                    a, b = c - right * half, c + right * half
                    ax.plot([a[0], b[0]], [a[1], b[1]], color=WIN_COL[w % 3], lw=5, solid_capstyle="butt")
                    wall = 1.2
                    for s in (-1, 1):
                        p0, p1 = c + s * right * half, c + s * right * (half + wall)
                        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="#555", lw=2)
                else:
                    ax.plot([c[0], c[0]], [c[2] - half, c[2] + half], color=WIN_COL[w % 3], lw=5, solid_capstyle="butt")
                    for s in (-1, 1):
                        ax.plot([c[0], c[0]], [c[2] + s * half, c[2] + s * (half + 1.0)], color="#555", lw=2)
            for p, d in tr.items():
                P = np.array(d["paths"][i])
                if len(P):
                    ax.plot(P[:, 0], P[:, iy], color=COL[p], lw=1.5, label=("state machine" if p == "fsm" else "RL policy"))
            ax.set_xlabel("x [m]")
            ax.set_ylabel(lab)
            ax.grid(alpha=0.3)
            if row == 0:
                ax.set_title(f"scene {i} ({nw} windows)", fontsize=10)
                ax.set_aspect("equal", adjustable="datalim")
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Top (x-y) and side (x-z) views. Grey = wall around the opening.", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run", required=True)
    ap.add_argument("--eval", action="append", default=[])
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    out = a.out or os.path.join(a.run, "report")
    os.makedirs(out, exist_ok=True)
    sc = read_scalars(a.run)
    if sc:
        plot_training(sc, os.path.join(out, "training_curves.png"))
        last = {k: float(v[-50:, 1].mean()) for k, v in sc.items() if len(v)}
        with open(os.path.join(out, "training_last50.json"), "w") as f:
            json.dump(last, f, indent=2)
    evals = []
    for d in a.eval:
        with open(os.path.join(d, "summary.json")) as f:
            evals.append(json.load(f))
        plot_traj(d, os.path.join(out, f"traj_{os.path.basename(d.rstrip('/'))}.png"))
    if evals:
        plot_eval_bars(evals, os.path.join(out, "eval_bars.png"))
    print(f"[report] wrote {out}: {sorted(os.listdir(out))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
