"""Procedural CPU renderer for the window dataset — Isaac-Sim FALLBACK.

Isaac Sim's RTX renderer is blocked on this cluster by a driver bug (see
sim/ISAAC_CLUSTER_NOTES.md). This renderer produces a *stopgap* dataset so 길남's
YOLO-pose training (gpu_jobs Job 1) can run now. It is NOT photoreal — it draws
perspective-correct coloured window quads over a textured, cluttered background —
but it is honest about geometry: it projects window corners with the SAME
`common.geometry` path that `build_label_lines` uses, so every rendered pixel
matches its label exactly.

Pipeline (mirrors the Isaac path so downstream code is identical):
    sample_scene(seed) -> render RGB (here) + scene_to_metadata(scene)
    -> <out>/frames/frame_000001.png + <out>/meta/frame_000001.json
    -> python3 sim/export_dataset.py --mode from-metadata --metadata-dir <out>/meta --out <ds>

Run with a python that has numpy + opencv (e.g. /scratch/pcn3tv/b200venv/bin/python).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from common import window_corners_world, world_to_camera_cv, project_points  # noqa: E402
from sim.scene_gen import sample_scene, window_rotation_from_normal, scene_to_metadata  # noqa: E402

try:
    import cv2
except ImportError as e:  # pragma: no cover
    raise SystemExit("procedural_render needs opencv-python (use /scratch/pcn3tv/b200venv/bin/python)") from e

# Window colours in RGB, chosen to land inside color_order.yaml's OpenCV-HSV bands
# (red H~0, green H~60, blue H~120; S,V high) so 길남's color_judge succeeds.
_RGB = {"red": (232, 26, 26), "green": (26, 200, 40), "blue": (26, 40, 224)}


def _textured_background(rng: np.random.Generator, w: int, h: int, lighting: dict) -> np.ndarray:
    """Muted, feature-rich background (VIO needs trackable features; never blank)."""
    # smooth low-frequency colour field (upsampled small noise)
    small = rng.integers(40, 150, size=(rng.integers(6, 14), rng.integers(6, 14), 3), dtype=np.uint8)
    bg = cv2.resize(small.astype(np.float32), (w, h), interpolation=cv2.INTER_CUBIC)
    bg += rng.normal(0, 8, bg.shape)  # fine grain
    # clutter: muted rectangles / lines / circles -> edges & corners for features
    for _ in range(int(rng.integers(12, 30))):
        c = tuple(int(v) for v in rng.integers(30, 170, size=3))
        x1, y1 = int(rng.integers(0, w)), int(rng.integers(0, h))
        kind = rng.integers(0, 3)
        if kind == 0:
            x2, y2 = int(rng.integers(0, w)), int(rng.integers(0, h))
            cv2.rectangle(bg, (x1, y1), (x2, y2), c, int(rng.integers(1, 4)))
        elif kind == 1:
            cv2.line(bg, (x1, y1), (int(rng.integers(0, w)), int(rng.integers(0, h))), c, int(rng.integers(1, 3)))
        else:
            cv2.circle(bg, (x1, y1), int(rng.integers(6, 60)), c, int(rng.integers(1, 3)))
    # directional lighting gradient + global brightness
    inten = float(lighting.get("intensity", 1.0)) if isinstance(lighting, dict) else 1.0
    ang = float(lighting.get("azimuth_rad", rng.uniform(0, 6.28))) if isinstance(lighting, dict) else rng.uniform(0, 6.28)
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    grad = (np.cos(ang) * xx / w + np.sin(ang) * yy / h)
    bg *= np.clip(0.55 + 0.35 * inten + 0.25 * grad, 0.35, 1.4)[..., None]
    return np.clip(bg, 0, 255).astype(np.uint8)


def render_scene(scene: dict) -> np.ndarray:
    """Render one scene to an RGB uint8 image consistent with build_label_lines."""
    intr = scene["intrinsics"]
    W, H = int(intr.width), int(intr.height)
    K = intr.K()
    T = np.asarray(scene["camera"]["T_world_cam_usd"], float)
    rng = np.random.default_rng(int(scene["seed"]) ^ 0x9E3779B9)
    img = _textured_background(rng, W, H, scene.get("lighting", {}))

    # draw windows far-to-near so nearer windows occlude correctly
    def depth(win):
        cc = world_to_camera_cv(np.asarray(win["center"], float), T)[0]
        return cc[2]
    for win in sorted(scene["windows"], key=depth, reverse=True):
        R = window_rotation_from_normal(win["normal"])
        corners_w = window_corners_world(win["center"], R, float(win["width"]), float(win["height"]))
        cam_cv = world_to_camera_cv(corners_w, T)
        uv, z, in_front = project_points(cam_cv, K)
        if not in_front.all():
            continue  # a corner behind camera: skip (also unlabelled)
        pts = uv.astype(np.int32)
        rgb = _RGB[win["color"]]
        shade = rng.uniform(0.75, 1.0)
        fill = tuple(int(c * shade) for c in rgb)
        cv2.fillConvexPoly(img, pts, fill, lineType=cv2.LINE_AA)
        # a darker frame border for realism + edge features
        cv2.polylines(img, [pts], True, tuple(int(c * 0.45) for c in rgb),
                      thickness=int(max(2, min(W, H) * 0.006)), lineType=cv2.LINE_AA)
    return img


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Procedural window-dataset renderer (Isaac fallback).")
    ap.add_argument("--num-frames", type=int, required=True)
    ap.add_argument("--out", required=True, help="output dir (creates frames/ and meta/).")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--start", type=int, default=0, help="starting frame index (for sharding).")
    args = ap.parse_args(argv)

    fr_dir = os.path.join(args.out, "frames")
    me_dir = os.path.join(args.out, "meta")
    os.makedirs(fr_dir, exist_ok=True)
    os.makedirs(me_dir, exist_ok=True)

    labelled = 0
    for i in range(args.start, args.start + args.num_frames):
        scene = sample_scene(args.seed + i)
        img = render_scene(scene)  # RGB
        stem = f"frame_{i:06d}"
        cv2.imwrite(os.path.join(fr_dir, stem + ".png"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        meta = scene_to_metadata(scene, image=f"../frames/{stem}.png", frame_id=i)
        with open(os.path.join(me_dir, stem + ".json"), "w") as f:
            json.dump(meta, f)
        labelled += len(scene["windows"])
        if (i - args.start + 1) % 250 == 0:
            print(f"  rendered {i - args.start + 1}/{args.num_frames}", flush=True)
    print(f"done: {args.num_frames} frames, {labelled} window instances -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
