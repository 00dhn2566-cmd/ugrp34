"""Make the PyBullet scene look like the domain the detector was trained on.

WHY
---
``window_yolo11s_best.pt`` (gpu_jobs Job 1) was trained on frames produced by
``sim/procedural_render.py``. A PyBullet scene built from PyBullet defaults —
white sky, blue checkerboard ground, flat-shaded bars — is a different visual
domain, and the detector simply does not fire on it (measured: max conf 0.14 on
outline frames, 0.00 on plain filled plates). Retraining is expensive and is
blocked on team decisions A2/A3, so instead we move the *renderer* to the model.

HOW (single source of truth)
----------------------------
This module does NOT re-implement the training look. It imports the training
renderer's own background generator and colour table:

    sim.procedural_render._textured_background   -> the exact clutter background
    sim.procedural_render._RGB                   -> the exact window colours

and mirrors its two remaining draw rules (fill shade, darker border) as named
constants below. If 윤호 changes the renderer, this module follows automatically
— the same routing discipline ``sim/export_stream.py`` uses for 길남's builders.

The camera side is pinned to 길남's ``synth_intrinsics.yaml`` (spec §6), so the
projected pixel size of a window at a given range matches the training frames.

WHAT IT BUILDS
--------------
``build_room``    a closed, textured enclosure (kills the sky + checkerboard, and
                  gives VIO trackable features — 박태민 07/03 requirement)
``build_window``  4 thin collision bars (the drone can still fly through) plus a
                  filled colour pane and a darker border, matching the training
                  renderer's ``fillConvexPoly`` + ``polylines`` pair.

Everything here is visual-only except the window bars, so physics — and any
policy trained against it — is unchanged.
"""
from __future__ import annotations

import os
import sys
from typing import Dict, List, Sequence, Tuple

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

# --- the training renderer's own definitions (imported, never re-implemented) ---
from sim.procedural_render import _RGB as WINDOW_RGB          # noqa: E402
from sim.procedural_render import _textured_background        # noqa: E402

#: mirrors ``procedural_render.render_scene``: fill is shaded, border is darker.
FILL_SHADE_RANGE: Tuple[float, float] = (0.75, 1.0)
BORDER_SHADE: float = 0.45
#: border thickness there is ``0.006 * min(W, H)`` px; at 1280x720 that is ~4 px.
BORDER_PX_FRAC: float = 0.006

#: lighting dict shape that ``_textured_background`` actually reads. NOTE the
#: training set was generated while ``scene_gen`` emitted ``brightness``/``direction``
#: instead, so ``intensity`` fell back to its 1.0 default for every training frame
#: (the key mismatch logged in NEWS 2026-08-11). We reproduce that default here on
#: purpose — matching the data that exists, not the data that was intended.
TRAINING_LIGHTING: Dict[str, float] = {"intensity": 1.0}

_INTRINSICS_YAML = os.path.join(
    os.path.dirname(_REPO), "overall_gilnam", "vision", "synth_intrinsics.yaml")


# --------------------------------------------------------------------------- #
# camera
# --------------------------------------------------------------------------- #
def load_intrinsics(path: str = _INTRINSICS_YAML) -> Dict[str, float]:
    """길남's spec §6 intrinsics (fx=fy=600, cx=640, cy=360 @ 1280x720 today)."""
    import yaml
    with open(path) as fh:
        d = yaml.safe_load(fh) or {}
    return {k: float(d[k]) for k in ("width", "height", "fx", "fy", "cx", "cy")}


def fov_y_deg(intr: Dict[str, float] | None = None) -> float:
    """Vertical FOV for ``p.computeProjectionMatrixFOV`` from the pinhole model."""
    intr = intr or load_intrinsics()
    return float(np.degrees(2.0 * np.arctan(0.5 * intr["height"] / intr["fy"])))


def aspect(intr: Dict[str, float] | None = None) -> float:
    intr = intr or load_intrinsics()
    return float(intr["width"] / intr["height"])


# --------------------------------------------------------------------------- #
# texture
# --------------------------------------------------------------------------- #
def clutter_texture(out_path: str, seed: int = 0, size: int = 1024,
                    lighting: Dict[str, float] | None = None) -> str:
    """Write one training-style clutter texture to ``out_path``; return the path.

    Straight call into the training renderer's background generator, so the wall
    statistics (low-frequency colour field, 12-30 clutter primitives, grain,
    directional gradient) are identical to what the detector saw.
    """
    import cv2
    rng = np.random.default_rng(seed)
    img = _textured_background(rng, size, size, lighting or dict(TRAINING_LIGHTING))
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    cv2.imwrite(out_path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    return out_path


# --------------------------------------------------------------------------- #
# scene pieces
# --------------------------------------------------------------------------- #
def build_room(p, client: int, bounds: Sequence[Sequence[float]],
               tex_dir: str, seed: int = 0, n_textures: int = 3) -> List[int]:
    """Visual-only enclosure around ``bounds`` = ((x0,x1),(y0,y1),(z0,z1)).

    Six thin textured slabs. No collision shapes, so physics and any policy
    trained without them are unaffected. Removes both PyBullet giveaways at once:
    the white sky (walls + ceiling occlude it) and the blue checkerboard (the
    floor slab sits just above ``plane.urdf``).
    """
    (x0, x1), (y0, y1), (z0, z1) = bounds
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
    hx, hy, hz = (x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2
    t = 0.02

    texes = [p.loadTexture(clutter_texture(os.path.join(tex_dir, f"wall_{i}.png"),
                                           seed=seed + i))
             for i in range(max(1, n_textures))]

    slabs = [
        ([cx, cy, z0], [hx, hy, t]),      # floor
        ([cx, cy, z1], [hx, hy, t]),      # ceiling
        ([x0, cy, cz], [t, hy, hz]),      # -x
        ([x1, cy, cz], [t, hy, hz]),      # +x
        ([cx, y0, cz], [hx, t, hz]),      # -y
        ([cx, y1, cz], [hx, t, hz]),      # +y
    ]
    ids = []
    for i, (pos, he) in enumerate(slabs):
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he,
                                  rgbaColor=[1, 1, 1, 1], physicsClientId=client)
        bid = p.createMultiBody(0, -1, vis, pos, physicsClientId=client)
        p.changeVisualShape(bid, -1, textureUniqueId=texes[i % len(texes)],
                            physicsClientId=client)
        ids.append(bid)
    return ids


def build_clutter(p, client: int, bounds: Sequence[Sequence[float]], tex_dir: str,
                  n: int = 24, seed: int = 0, keep_clear=None,
                  clear_radius: float = 1.4) -> List[int]:
    """Scatter visual-only 3D props (boxes / cylinders / spheres) through the room.

    박태민 asked for this on 2026-07-03: a blank background makes VIO fail, it needs
    trackable features. The textured walls `build_room` puts up give 2D texture but
    every feature sits on a plane, so they carry almost no parallax. Real props at
    varying depth are what actually make the structure observable — and they also
    match the training renderer, whose background draws 12-30 clutter primitives
    (`sim/procedural_render._textured_background`).

    Props are visual-only (no collision shape) so physics and any trained policy are
    unaffected, and `keep_clear` centres — the windows and the corridor the drone
    flies — are kept empty so nothing blocks the opening.
    """
    (x0, x1), (y0, y1), (z0, z1) = bounds
    rng = np.random.default_rng(seed)
    texes = [p.loadTexture(clutter_texture(os.path.join(tex_dir, f"prop_{i}.png"),
                                           seed=seed + 100 + i, size=256))
             for i in range(3)]
    clear = [np.asarray(c, dtype=float) for c in (keep_clear or [])]

    ids = []
    for _ in range(int(n)):
        kind = rng.integers(0, 3)
        shade = float(rng.uniform(0.45, 0.95))
        rgba = [shade * float(rng.uniform(0.6, 1.0)),
                shade * float(rng.uniform(0.6, 1.0)),
                shade * float(rng.uniform(0.6, 1.0)), 1.0]
        # Props REST ON THE FLOOR: a crate floating at 2 m is not something a real
        # scene contains, and VIO would learn parallax from geometry that could not
        # exist. Each shape's half-height sets its centre so it sits on z0.
        if kind == 0:
            he = rng.uniform(0.08, 0.45, size=3)
            half_h = float(he[2])
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he.tolist(),
                                      rgbaColor=rgba, physicsClientId=client)
        elif kind == 1:
            length = float(rng.uniform(0.25, 1.2))
            half_h = length / 2.0
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=float(rng.uniform(0.06, 0.28)),
                                      length=length, rgbaColor=rgba, physicsClientId=client)
        else:
            r = float(rng.uniform(0.08, 0.32))
            half_h = r
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=rgba,
                                      physicsClientId=client)

        for _try in range(20):
            pos = np.array([rng.uniform(x0 + 0.5, x1 - 0.5),
                            rng.uniform(y0 + 0.4, y1 - 0.4),
                            z0 + half_h])                     # standing on the floor
            if all(np.linalg.norm(pos - c) > clear_radius for c in clear):
                break
        else:
            continue
        # yaw only — a box tumbled about x/y would not be resting on the ground
        quat = p.getQuaternionFromEuler([0.0, 0.0, float(rng.uniform(0, 2 * np.pi))])
        bid = p.createMultiBody(0, -1, vis, pos.tolist(), quat, physicsClientId=client)
        if rng.random() < 0.6:
            p.changeVisualShape(bid, -1, textureUniqueId=texes[int(rng.integers(len(texes)))],
                                physicsClientId=client)
        ids.append(bid)
    return ids


def build_wall(p, client: int, center, ow: float, oh: float, tex_dir: str,
               half_y: float = 4.0, half_z: float = 2.6, thickness: float = 0.06,
               seed: int = 0) -> Tuple[List[int], List[int]]:
    """A wall filling the plane at ``center[0]`` with a window-sized hole punched out.

    A window hanging in mid-air is not the mission — the mission is an opening in a
    wall — and it is also what makes the vision hard for the wrong reason: several
    frames line up in one image, the detector returns one box per class for the same
    opening (measured: 21 cross-class duplicates in a single sweep), and those
    observations then pollute each other's triangulation, biasing every reconstructed
    window toward the one behind it.

    A wall occludes what is behind it, so each image contains one opening. Built from
    4 slabs (above / below / left / right of the hole) rather than a boolean subtract,
    which PyBullet has no primitive for.

    Returns ``(collision_ids, visual_ids)``; the slabs carry collision so the drone
    really has to fly through the hole.
    """
    cx, cy, cz = [float(v) for v in center]
    t = thickness / 2.0
    tex = p.loadTexture(clutter_texture(os.path.join(tex_dir, f"wall_{seed}.png"), seed=seed))

    up_h = max(0.02, (cz + half_z) - (cz + oh / 2)) / 2.0
    dn_h = max(0.02, (cz - oh / 2) - (cz - half_z)) / 2.0
    side_w = max(0.02, (cy + half_y) - (cy + ow / 2)) / 2.0
    slabs = [
        ([cx, cy, cz + oh / 2 + up_h], [t, half_y, up_h]),                  # above
        ([cx, cy, cz - oh / 2 - dn_h], [t, half_y, dn_h]),                  # below
        ([cx, cy + ow / 2 + side_w, cz], [t, side_w, oh / 2]),              # left
        ([cx, cy - ow / 2 - side_w, cz], [t, side_w, oh / 2]),              # right
    ]
    ids = []
    for pos, he in slabs:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=he, physicsClientId=client)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=[1, 1, 1, 1],
                                  physicsClientId=client)
        bid = p.createMultiBody(0, col, vis, pos, physicsClientId=client)
        p.changeVisualShape(bid, -1, textureUniqueId=tex, physicsClientId=client)
        ids.append(bid)
    return ids, []


def hide_default_plane(p, client: int, plane_id: int) -> None:
    """Make ``plane.urdf``'s checkerboard invisible (physics untouched)."""
    p.changeVisualShape(plane_id, -1, rgbaColor=[0, 0, 0, 0], physicsClientId=client)


def build_window(p, client: int, center, ow: float, oh: float, color: str,
                 bar_t: float = 0.12, bar_d: float = 0.05,
                 rng: np.random.Generator | None = None,
                 pane: bool = True) -> Tuple[List[int], List[int]]:
    """One window. Returns ``(collision_ids, visual_only_ids)``.

    ``collision_ids`` are the 4 bars — the caller registers those for crash
    detection. ``visual_only_ids`` (filled pane, darker border) have no collision
    shape and MUST NOT be registered, or the drone would crash on its own image.
    """
    rng = rng or np.random.default_rng(0)
    cx, cy, cz = [float(v) for v in center]
    base = np.array(WINDOW_RGB[color], dtype=float) / 255.0
    fill = np.clip(base * rng.uniform(*FILL_SHADE_RANGE), 0, 1)
    # The frame bars ARE the window when there is no pane, so they carry the full
    # saturated colour; only the pane's inner ring uses the renderer's dark shade.
    bar_rgb = fill if not pane else np.clip(base * BORDER_SHADE, 0, 1)
    border = np.clip(base * BORDER_SHADE, 0, 1)

    collision, visual = [], []

    # --- 4 bars: the real opening the drone flies through ------------------- #
    bars = [
        ([cx, cy, cz + oh / 2 + bar_t / 2], [bar_d / 2, ow / 2 + bar_t, bar_t / 2]),
        ([cx, cy, cz - oh / 2 - bar_t / 2], [bar_d / 2, ow / 2 + bar_t, bar_t / 2]),
        ([cx, cy - ow / 2 - bar_t / 2, cz], [bar_d / 2, bar_t / 2, oh / 2]),
        ([cx, cy + ow / 2 + bar_t / 2, cz], [bar_d / 2, bar_t / 2, oh / 2]),
    ]
    for pos, he in bars:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=he, physicsClientId=client)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he,
                                  rgbaColor=[*bar_rgb, 1], physicsClientId=client)
        collision.append(p.createMultiBody(0, col, vis, pos, physicsClientId=client))

    if not pane:
        return collision, visual

    # --- filled pane: what the detector was trained to see ------------------ #
    v = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.004, ow / 2, oh / 2],
                            rgbaColor=[*fill, 1], physicsClientId=client)
    visual.append(p.createMultiBody(0, -1, v, [cx, cy, cz], physicsClientId=client))

    # --- darker border ring, mirroring the renderer's polylines pass -------- #
    bw = max(0.01, 0.02 * min(ow, oh))
    ring = [
        ([cx, cy, cz + oh / 2 - bw / 2], [0.005, ow / 2, bw / 2]),
        ([cx, cy, cz - oh / 2 + bw / 2], [0.005, ow / 2, bw / 2]),
        ([cx, cy - ow / 2 + bw / 2, cz], [0.005, bw / 2, oh / 2]),
        ([cx, cy + ow / 2 - bw / 2, cz], [0.005, bw / 2, oh / 2]),
    ]
    for pos, he in ring:
        v = p.createVisualShape(p.GEOM_BOX, halfExtents=he,
                                rgbaColor=[*border, 1], physicsClientId=client)
        visual.append(p.createMultiBody(0, -1, v, pos, physicsClientId=client))

    return collision, visual
