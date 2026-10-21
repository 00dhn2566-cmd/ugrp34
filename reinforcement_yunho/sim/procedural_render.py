"""Procedural CPU renderer for the window dataset — Isaac-Sim FALLBACK.

Isaac Sim's RTX renderer is blocked on this cluster by a driver bug (see
sim/ISAAC_CLUSTER_NOTES.md). This renderer produces a *stopgap* dataset so 길남's
YOLO-pose training (gpu_jobs Job 1) can run now. It is NOT photoreal — it draws
perspective-correct coloured window FRAMES over a textured, cluttered background —
but it is honest about geometry: it projects window corners with the SAME
`common.geometry` path that `build_label_lines` uses, so every rendered pixel
matches its label exactly.

Window look = window_detection_spec_v0.2.md §2.5 (잠정 확정 2026-08-18):
    * a coloured frame ring drawn OUTSIDE the four labelled corners (frame colour
      = traversal-order colour, thickness randomised per window), and
    * a see-through opening: the background and any farther window stay visible
      inside it (optional slight tint, ``OPENING_TINT_MAX``).
    * the labelled corners are the OPENING corners (inner edge of the ring), never
      the frame's outer edge.
Filled colour panes are explicitly excluded by §2.5 (they break colour judging
and triangulation when windows overlap); the pre-10-21 version of this file drew
them, so any model trained on that output needs the retrain in checklist ⓑ.

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

#: Frame thickness (metres) = frac * min(opening width, height), sampled per window
#: (spec §2.5: thickness is a randomisation target). A window dict may pin it with
#: ``frame_thickness_m``.
FRAME_THICKNESS_FRAC = (0.08, 0.22)
#: Opening translucency, spec §2.5 "0~약간": alpha of the frame colour blended over
#: whatever is behind the opening. 0 = fully see-through.
OPENING_TINT_MAX = 0.10
#: Frame colour shading + darker edge lines (rl/domain.py mirrors these).
FILL_SHADE_RANGE = (0.75, 1.0)
BORDER_SHADE = 0.45
BORDER_PX_FRAC = 0.006


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


def _draw_window(img: np.ndarray, rng: np.random.Generator, win: dict,
                 T_world_cam_usd: np.ndarray, K: np.ndarray) -> bool:
    """Draw one spec §2.5 window: coloured frame ring OUTSIDE the labelled opening
    corners, opening left see-through. Returns False if the window was skipped
    (a corner behind the camera — such a window is also unlabelled)."""
    H, W = img.shape[:2]
    R = window_rotation_from_normal(win["normal"])
    w, h = float(win["width"]), float(win["height"])
    t = win.get("frame_thickness_m")
    t = float(t) if t is not None else float(rng.uniform(*FRAME_THICKNESS_FRAC) * min(w, h))
    inner_w = window_corners_world(win["center"], R, w, h)                  # == label corners
    outer_w = window_corners_world(win["center"], R, w + 2.0 * t, h + 2.0 * t)
    uv_in, _, in_front = project_points(world_to_camera_cv(inner_w, T_world_cam_usd), K)
    if not in_front.all():
        return False
    uv_out, _, out_front = project_points(world_to_camera_cv(outer_w, T_world_cam_usd), K)
    inner = uv_in.astype(np.int32)
    rgb = _RGB[win["color"]]
    shade = rng.uniform(*FILL_SHADE_RANGE)
    fill = tuple(int(c * shade) for c in rgb)
    dark = tuple(int(c * BORDER_SHADE) for c in rgb)
    edge_px = int(max(2, min(W, H) * BORDER_PX_FRAC))

    # frame ring mask = outer quad minus opening quad (the opening keeps whatever
    # was already drawn: background and farther windows)
    ring = np.zeros((H, W), np.uint8)
    if out_front.all():
        outer = uv_out.astype(np.int32)
        cv2.fillConvexPoly(ring, outer, 1)
    else:  # extreme close-up: outer corners behind the camera -> pixel-width ring
        outer = None
        cv2.polylines(ring, [inner], True, 1, thickness=4 * edge_px)
    cv2.fillConvexPoly(ring, inner, 0)
    ring_m = ring.astype(bool)

    # optional slight tint of the opening (spec §2.5 "0~약간"); background stays visible
    alpha = float(rng.uniform(0.0, OPENING_TINT_MAX))
    if alpha > 0.0:
        hole = np.zeros((H, W), np.uint8)
        cv2.fillConvexPoly(hole, inner, 1)
        hole_m = hole.astype(bool)
        img[hole_m] = np.clip((1.0 - alpha) * img[hole_m].astype(np.float32)
                              + alpha * np.asarray(fill, np.float32), 0, 255).astype(np.uint8)

    # paint the ring, then darker edge lines (inner + outer) confined to the ring
    img[ring_m] = fill
    edges = img.copy()
    cv2.polylines(edges, [inner], True, dark, thickness=edge_px, lineType=cv2.LINE_AA)
    if outer is not None:
        cv2.polylines(edges, [outer], True, dark, thickness=edge_px, lineType=cv2.LINE_AA)
    img[ring_m] = edges[ring_m]
    return True


def render_scene(scene: dict) -> np.ndarray:
    """Render one scene to an RGB uint8 image consistent with build_label_lines.
    Windows follow spec §2.5 (frame + see-through opening); see ``_draw_window``."""
    intr = scene["intrinsics"]
    W, H = int(intr.width), int(intr.height)
    K = intr.K()
    T = np.asarray(scene["camera"]["T_world_cam_usd"], float)
    rng = np.random.default_rng(int(scene["seed"]) ^ 0x9E3779B9)
    img = _textured_background(rng, W, H, scene.get("lighting", {}))

    # draw windows far-to-near so nearer frames occlude farther ones correctly
    def depth(win):
        cc = world_to_camera_cv(np.asarray(win["center"], float), T)[0]
        return cc[2]
    for win in sorted(scene["windows"], key=depth, reverse=True):
        _draw_window(img, rng, win, T, K)
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
