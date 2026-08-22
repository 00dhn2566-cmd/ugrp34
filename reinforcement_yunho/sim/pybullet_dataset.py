"""PyBullet → YOLO-pose dataset (spec §4.3) for domain fine-tuning.

WHY
---
The shipped detector was trained on `sim/procedural_render.py` frames where a
window is an OPAQUE filled quad. Making PyBullet windows opaque to match it turns
every window into a wall: the near pane hides the far ones, `color_judge` samples
the near window's colour for all of them, and triangulation fuses different
windows into one (measured: all three judged red, 207 mm reconstruction error).

An open frame is what the drone actually flies through, so instead of bending the
scene to the model we fine-tune the model onto open frames — neck + head only
(`freeze=11`), keeping the backbone that produced keypoint mAP50-95 0.927.

Labels are free here: window corners are known exactly, so projecting them gives
pixel-perfect ground truth with no annotation. Classes are the real 3 (red/green/
blue = order_index 0/1/2), so the same run also un-does `single_cls` and lets the
network read colour itself instead of the occlusion-fragile HSV post-step.

LABEL FORMAT (spec §4.3, 17 tokens — identical to sim/replicator_writer.py)
    <class> <cx> <cy> <w> <h> <u1> <v1> <vis1> ... <u4> <v4> <vis4>
all coordinates normalised to [0,1]; corners TL→TR→BR→BL from the approach side.

POLICY A (the shipped model's convention): only windows whose four corners are all
inside the frame get a label. Partially-occluded windows are still labelled — the
open frame means they stay visible through the opening, which is the entire point.
"""
from __future__ import annotations

import argparse
import json
import os
import random
import sys
from typing import Dict, List, Sequence, Tuple

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TEAM = os.path.dirname(_REPO)
for _p in (_REPO, os.path.join(_TEAM, "overall_gilnam", "vision")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import pybullet as p                                    # noqa: E402
from rl import domain                                   # noqa: E402
from sim import pybullet_stream as pbs                  # noqa: E402

ORDER_INDEX = {"red": 0, "green": 1, "blue": 2}
COLORS = ("red", "green", "blue")

# Scene randomisation, aligned with spec §4.1 / sim/scene_gen.py.
N_WINDOWS = (1, 4)
SIZE_WH_M = (0.8, 1.2)          # spec §4.1 opening size
DIST_BINS = ((1.8, 3.0), (3.0, 6.0), (6.0, 9.0))   # near / mid / far, uniform
LATERAL_M = 2.2
HEIGHT_M = (0.7, 2.3)

# --- overlap curriculum (v2) ------------------------------------------------
# v1 spread windows 1.8-9 m apart with up to +-2.2 m lateral offset, so they almost
# never overlapped in the image. Through an OPEN frame the window behind is visible,
# and that is exactly where v1 fails: measured against ground truth, blue was called
# green 12 times and green was detected in only 7 of 32 in-frame chances, which drags
# green's triangulation 0.66 m toward blue. So a fixed share of scenes is generated
# deliberately stacked — small gaps, near-zero lateral offset — and colours are drawn
# so adjacent windows are often the same or confusable.
OVERLAP_FRACTION = 0.55         # share of scenes built as a stacked corridor
STACK_GAP_M = (0.7, 2.2)        # x gap when stacked (v1: 1.8-9)
STACK_JITTER_M = 0.35           # lateral/vertical wobble when stacked
SAME_COLOUR_RUN_P = 0.35        # chance the next window repeats the previous colour


def _sample_scene(rng: random.Random) -> List[dict]:
    """One frame's worth of windows. A fixed share are stacked to force overlap."""
    stacked = rng.random() < OVERLAP_FRACTION
    n = rng.randint(2, N_WINDOWS[1]) if stacked else rng.randint(*N_WINDOWS)

    cols: List[str] = []
    for i in range(n):
        if i and rng.random() < SAME_COLOUR_RUN_P:
            cols.append(cols[-1])               # same colour behind same colour
        else:
            cols.append(rng.choice(COLORS))

    wins, x = [], 0.0
    y0 = rng.uniform(-LATERAL_M, LATERAL_M) if stacked else None
    z0 = rng.uniform(*HEIGHT_M) if stacked else None
    for i in range(n):
        if stacked:
            x += rng.uniform(*STACK_GAP_M) if i else rng.uniform(1.6, 2.6)
            y = y0 + rng.uniform(-STACK_JITTER_M, STACK_JITTER_M)
            z = z0 + rng.uniform(-STACK_JITTER_M, STACK_JITTER_M)
        else:
            lo, hi = DIST_BINS[rng.randrange(len(DIST_BINS))]
            x += rng.uniform(lo, hi) if i else rng.uniform(*DIST_BINS[0])
            y = rng.uniform(-LATERAL_M, LATERAL_M)
            z = rng.uniform(*HEIGHT_M)
        wins.append({
            "order_index": ORDER_INDEX[cols[i]],
            "color": cols[i],
            "center": np.array([x, y, max(0.6, z)]),
            "ow": rng.uniform(*SIZE_WH_M),
            "oh": rng.uniform(*SIZE_WH_M),
        })
    return wins


def _camera(rng: random.Random, wins: Sequence[dict]) -> Tuple[np.ndarray, np.ndarray]:
    """A pose looking down the +x corridor, jittered (scene_gen CAM_LOOK_DEG-ish)."""
    first = wins[0]["center"]
    eye = np.array([first[0] - rng.uniform(1.5, 3.5),
                    rng.uniform(-1.2, 1.2),
                    rng.uniform(0.8, 2.0)])
    look = np.array([first[0] + rng.uniform(1.0, 6.0),
                     first[1] + rng.uniform(-0.8, 0.8),
                     first[2] + rng.uniform(-0.5, 0.5)])
    return pbs.look_at_pose(eye, look)


def _project(corners_w: np.ndarray, pos, quat, K) -> np.ndarray | None:
    from eval_recon3d import quat_xyzw_to_rot
    R = quat_xyzw_to_rot(np.asarray(quat, dtype=float))
    cam = (R.T @ (corners_w - np.asarray(pos, float)).T).T
    if (cam[:, 2] <= 0.15).any():
        return None
    return (K @ (cam / cam[:, 2:3]).T).T[:, :2]


def build_label_line(order_index: int, uv: np.ndarray, w: int, h: int) -> str | None:
    """17-token YOLO-pose line (spec §4.3), or None if any corner leaves the frame."""
    if (uv[:, 0] < 0).any() or (uv[:, 0] >= w).any() or (uv[:, 1] < 0).any() or (uv[:, 1] >= h).any():
        return None                       # policy A: all four corners must be visible
    u, v = uv[:, 0] / w, uv[:, 1] / h
    cx, cy = float(u.mean()), float(v.mean())
    bw, bh = float(u.max() - u.min()), float(v.max() - v.min())
    if bw < 0.012 or bh < 0.012:          # too small to be a useful sample
        return None
    toks = [str(order_index), f"{cx:.6f}", f"{cy:.6f}", f"{bw:.6f}", f"{bh:.6f}"]
    for (uu, vv) in zip(u, v):
        toks += [f"{uu:.6f}", f"{vv:.6f}", "2"]     # visibility 2 = labelled & visible
    return " ".join(toks)


def _iou(a, b):
    xa, ya = max(a[0], b[0]), max(a[1], b[1])
    xb, yb = min(a[2], b[2]), min(a[3], b[3])
    i = max(0.0, xb - xa) * max(0.0, yb - ya)
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - i
    return i / ua if ua > 0 else 0.0


def _build_scene(client: int, wins: Sequence[dict], tex_dir: str, pane: bool) -> None:
    xs = [float(w["center"][0]) for w in wins]
    domain.build_room(p, client,
                      bounds=((min(xs) - 6.0, max(xs) + 6.0), (-6.0, 6.0), (-0.05, 6.0)),
                      tex_dir=tex_dir, seed=0)
    rng = np.random.default_rng(0)
    for w in wins:
        domain.build_window(p, client, w["center"], w["ow"], w["oh"], w["color"],
                            rng=rng, pane=pane)


def generate(out_dir: str, n_frames: int, seed: int = 0, pane: bool = False,
             splits=(("train", 0.8), ("val", 0.1), ("test", 0.1))) -> Dict[str, int]:
    """Render n_frames, write images/labels/{train,val,test} + dataset yaml."""
    import cv2

    intr = domain.load_intrinsics()
    W, H = int(intr["width"]), int(intr["height"])
    K = np.array([[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1]])
    tex_dir = os.path.join(out_dir, "_tex")
    for s, _ in splits:
        os.makedirs(os.path.join(out_dir, "images", s), exist_ok=True)
        os.makedirs(os.path.join(out_dir, "labels", s), exist_ok=True)

    rng = random.Random(seed)

    # Shuffled, exact-count split. Assigning by frame index would put every
    # early-seed scene in train and every late one in test; the sampler drifts
    # (window count / colour draws are sequential), so an index split is a
    # distribution split too. Shuffle a fixed multiset instead, seeded so the
    # partition is reproducible.
    assign = []
    for s, frac in splits[:-1]:
        assign += [s] * int(round(frac * n_frames))
    assign += [splits[-1][0]] * (n_frames - len(assign))
    random.Random(seed + 777).shuffle(assign)

    counts = {s: 0 for s, _ in splits}
    n_lab = 0
    n_overlap_pairs = n_overlap_same_colour = n_frames_labelled = 0
    n_per_frame = {k: 0 for k in range(6)}
    client = p.connect(p.DIRECT)
    try:
        for i in range(n_frames):
            split = assign[i]
            wins = _sample_scene(rng)
            p.resetSimulation(physicsClientId=client)
            _build_scene(client, wins, tex_dir, pane)
            pos, quat = _camera(rng, wins)

            from eval_recon3d import quat_xyzw_to_rot
            fwd = quat_xyzw_to_rot(np.asarray(quat, float))[:, 2]
            img = pbs.render_frame(p, client, pos, np.asarray(pos) + fwd, intr)

            lines, boxes, cls = [], [], []
            for w in wins:
                uv = _project(pbs.window_corners_gt(w["center"], w["ow"], w["oh"]),
                              pos, quat, K)
                if uv is None:
                    continue
                ln = build_label_line(w["order_index"], uv, W, H)
                if ln:
                    lines.append(ln)
                    boxes.append([uv[:, 0].min(), uv[:, 1].min(),
                                  uv[:, 0].max(), uv[:, 1].max()])
                    cls.append(w["order_index"])
            if not lines:
                continue                       # skip empty frames
            n_per_frame[min(len(lines), 5)] += 1
            for bi in range(len(boxes)):
                for bj in range(bi + 1, len(boxes)):
                    if _iou(boxes[bi], boxes[bj]) > 0.10:
                        n_overlap_pairs += 1
                        if cls[bi] == cls[bj]:
                            n_overlap_same_colour += 1
            n_frames_labelled += 1

            stem = f"pyb_{i:06d}"
            cv2.imwrite(os.path.join(out_dir, "images", split, stem + ".png"),
                        cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            with open(os.path.join(out_dir, "labels", split, stem + ".txt"), "w") as f:
                f.write("\n".join(lines) + "\n")
            counts[split] += 1
            n_lab += len(lines)
            if (i + 1) % 100 == 0:
                print(f"  {i+1}/{n_frames}  labelled windows so far {n_lab}", flush=True)
    finally:
        p.disconnect(physicsClientId=client)

    yaml_path = os.path.join(out_dir, "window_pose_pyb.yaml")
    with open(yaml_path, "w") as f:
        f.write(
            f"path: {os.path.abspath(out_dir)}\n"
            "train: images/train\nval: images/val\ntest: images/test\n\n"
            "kpt_shape: [4, 3]\nflip_idx: [1, 0, 3, 2]\n\n"
            "names:\n  0: red\n  1: green\n  2: blue\n")
    print(f"\ndone: {counts}, {n_lab} labelled windows -> {out_dir}")
    print(f"겹침 통계: 프레임당 창문 " + ", ".join(f"{k}개:{v}" for k, v in sorted(n_per_frame.items()) if v)
          + f"  |  겹치는 쌍 {n_overlap_pairs} (같은 색 {n_overlap_same_colour})"
          + f"  |  라벨 프레임 {n_frames_labelled}")
    print(f"dataset yaml: {yaml_path}")
    return counts


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="PyBullet → YOLO-pose dataset (open-frame windows).")
    ap.add_argument("--out", required=True)
    ap.add_argument("--num-frames", type=int, default=600)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--pane", action="store_true",
                    help="fill the opening (the old opaque look); default is an open frame")
    a = ap.parse_args(argv)
    generate(a.out, a.num_frames, a.seed, pane=a.pane)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
