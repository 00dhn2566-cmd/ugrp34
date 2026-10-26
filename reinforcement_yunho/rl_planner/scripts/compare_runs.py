"""Training curves of several runs side by side (seed sweeps, ablations) + milestone table.

    python rl_planner/scripts/compare_runs.py --run <run_dir>[=label] ... [--max-iter 500] [--out fig.png]

One small panel per run (same axes): training success rate, timeout rate (episodes that
end neither in success nor crash — the policy hovering in place) and curriculum level / 5,
all as 10-iteration running means. Prints, per run, the milestones RESULTS.md §7 uses:
first iteration at L5 (level mean >= 4.9), first iteration at L5 with >= 95% success, and
the number of iterations in which more than half of the episodes timed out.
"""
from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

from report import plt, read_scalars, smooth  # noqa: E402  (report sets the Agg backend)

# reference categorical slots 1-3 (validated all-pairs) and recessive chrome
SERIES = (("Metrics/success_rate", "success rate", "#2a78d6"),
          ("Metrics/timeout_rate", "timeout rate (hovering, no progress)", "#eb6834"),
          ("Metrics/curriculum_level", "curriculum level / 5", "#1baf7a"))
INK, INK2, MUTED, GRID, AXIS = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"


def running_mean(y: np.ndarray, k: int = 10) -> np.ndarray:
    c = np.cumsum(np.insert(y, 0, 0.0))
    n = np.minimum(np.arange(1, len(y) + 1), k)
    return (c[1:] - c[np.arange(len(y)) + 1 - n]) / n


def milestones(sc: dict, max_iter: int | None = None) -> dict:
    """Iterations at which a run reached L5 / L5 with >= 95 % success, and its hovering iterations."""
    lv, sr, to = (sc.get(t) for t in ("Metrics/curriculum_level", "Metrics/success_rate", "Metrics/timeout_rate"))
    if lv is None or sr is None:
        return {"l5": None, "l5_95": None, "hover_iters": None}
    st, lv_m = lv[:, 0].astype(int), running_mean(lv[:, 1])
    ok = lv_m >= 4.9
    first = lambda m: int(st[np.argmax(m)]) if m.any() else None  # noqa: E731
    n = min(len(sr), len(lv))
    out = {"l5": None if float(lv[0, 1]) >= 4.9 else first(ok),          # fixed-level runs start at L5
           "l5_95": first(ok[:n] & (running_mean(sr[:, 1])[:n] >= 0.95))}
    if to is not None:
        keep = to[:, 0] < max_iter if max_iter else np.ones(len(to), dtype=bool)
        out["hover_iters"] = int((to[keep, 1] > 0.5).sum())
    return out


def plot(runs: list, path: str, max_iter: int | None = None, cols: int = 4) -> None:
    rows = -(-len(runs) // cols)
    fig, axes = plt.subplots(rows, cols, figsize=(3.4 * cols, 2.5 * rows + 0.6), sharex=True, sharey=True,
                             squeeze=False)
    for ax in axes.ravel()[len(runs):]:
        ax.set_visible(False)
    for ax, (label, sc) in zip(axes.ravel(), runs):
        for tag, _, color in SERIES:
            if tag not in sc:
                continue
            x, y = sc[tag][:, 0], sc[tag][:, 1] / (5.0 if tag.endswith("curriculum_level") else 1.0)
            if max_iter:
                x, y = x[x < max_iter], y[x < max_iter]
            y = running_mean(y)
            ax.plot(x, y, color=color, lw=1.4, solid_capstyle="round", solid_joinstyle="round")
            if tag.endswith("timeout_rate"):
                ax.fill_between(x, 0, y, color=color, alpha=0.10, lw=0)
        m = milestones(sc, max_iter)
        if m["l5_95"] is not None and (not max_iter or m["l5_95"] < max_iter):
            ax.axvline(m["l5_95"], color=MUTED, lw=0.8)
            ax.text(m["l5_95"], 1.02, f" L5 & 95%: {m['l5_95']}", color=INK2, fontsize=7.5, va="bottom",
                    ha="left" if m["l5_95"] < 0.6 * (max_iter or x[-1]) else "right")
        ax.set_title(label, fontsize=9, color=INK, loc="left", pad=12)
        ax.set_ylim(-0.02, 1.08)
        ax.grid(True, color=GRID, lw=0.6)
        ax.tick_params(colors=MUTED, labelsize=7.5, length=0)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.spines["bottom"].set_visible(True)
        ax.spines["bottom"].set_color(AXIS)
    for ax in axes[-1]:
        ax.set_xlabel("training iteration", fontsize=8, color=INK2)
    handles = [plt.Line2D([], [], color=c, lw=2) for _, _, c in SERIES]
    fig.legend(handles, [n for _, n, _ in SERIES], loc="upper left", ncol=3, frameon=False, fontsize=8.5,
               labelcolor=INK2, bbox_to_anchor=(0.01, 1.0))
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, dpi=110, facecolor="#fcfcfb")
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run", action="append", required=True, help="run directory, optionally =label")
    ap.add_argument("--max-iter", type=int, default=None, help="only the first N iterations (compare unequal budgets)")
    ap.add_argument("--out", default=None, help="PNG path (omit for the table only)")
    a = ap.parse_args(argv)
    runs = []
    print("| run | L5 reached | L5 & 95% success | iterations with > 50% timeouts |")
    print("|---|---|---|---|")
    for spec in a.run:
        d, _, label = spec.partition("=")
        sc = read_scalars(d)
        label = label or os.path.basename(d.rstrip("/"))
        m = milestones(sc, a.max_iter)
        print(f"| {label} | {'–' if m['l5'] is None else m['l5']} | {'–' if m['l5_95'] is None else m['l5_95']} | "
              f"{'–' if m['hover_iters'] is None else m['hover_iters']} |")
        runs.append((label, sc))
    if a.out:
        plot(runs, a.out, a.max_iter)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
