"""CPU-fallback renderer follows window_detection_spec_v0.2.md §2.5:
coloured frame OUTSIDE the labelled corners, opening see-through.

Needs opencv (skipped otherwise); numpy + pyyaml for the rest.
Run:  python -m pytest tests/test_procedural_render.py   (from reinforcement_yunho/)
"""
import os
import sys

import numpy as np
import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

cv2 = pytest.importorskip("cv2")


def _scene_one_window(frame_t_m: float):
    """Camera at the origin looking down +X; one 1x1 m red window 4 m ahead,
    facing the camera, frame thickness pinned so pixel geometry is known."""
    from sim.scene_gen import default_intrinsics, look_at_cv, cv_to_usd_transform

    intr = default_intrinsics()
    eye = np.array([0.0, 0.0, 0.0])
    R_cv = look_at_cv(eye, np.array([4.0, 0.0, 0.0]))
    win = {"color": "red", "order_index": 0, "center": np.array([4.0, 0.0, 0.0]),
           "normal": np.array([-1.0, 0.0, 0.0]), "width": 1.0, "height": 1.0,
           "frame_thickness_m": frame_t_m}
    scene = {"seed": 7, "intrinsics": intr,
             "camera": {"T_world_cam_usd": cv_to_usd_transform(R_cv, eye)},
             "windows": [win], "lighting": {"intensity": 1.0, "azimuth_rad": 0.0}}
    return scene, win, R_cv


def _project(scene, pts_world):
    from common import world_to_camera_cv, project_points
    K = scene["intrinsics"].K()
    uv, _, in_front = project_points(world_to_camera_cv(pts_world, scene["camera"]["T_world_cam_usd"]), K)
    assert in_front.all()
    return uv


def test_frame_outside_corners_opening_transparent():
    from sim.procedural_render import render_scene, _RGB
    from sim.scene_gen import window_rotation_from_normal
    from common import window_corners_world

    t = 0.2
    scene, win, _ = _scene_one_window(t)
    img = render_scene(scene)
    bg = render_scene({**scene, "windows": []})  # same seed -> identical background
    assert img.shape == bg.shape and img.dtype == np.uint8

    R = window_rotation_from_normal(win["normal"])
    inner = _project(scene, window_corners_world(win["center"], R, 1.0, 1.0))           # labelled corners
    outer = _project(scene, window_corners_world(win["center"], R, 1.0 + 2 * t, 1.0 + 2 * t))
    cx, cy = inner.mean(axis=0)
    cx, cy = int(round(cx)), int(round(cy))

    # 1) opening is see-through: its centre still shows the background (tint <= 10%)
    assert np.abs(img[cy, cx].astype(int) - bg[cy, cx].astype(int)).max() <= 0.12 * 255 + 1, \
        f"opening centre {img[cy, cx]} should show background {bg[cy, cx]}"

    # 2) frame ring OUTSIDE the labelled corners carries the window colour
    ring_x = int(round((inner[0, 0] + outer[0, 0]) / 2.0))   # midway between inner-TL and outer-TL, x
    px = img[cy, ring_x].astype(int)
    r, g, b = _RGB["red"]
    assert px[0] >= 0.7 * r and px[1] <= 60 and px[2] <= 60, f"ring pixel {px} is not the frame colour"
    assert not np.array_equal(img[cy, ring_x], bg[cy, ring_x])

    # 3) just INSIDE the labelled corner the opening is still background, i.e.
    #    the labelled corners are the opening corners, not the frame's outer edge
    in_x = int(round(inner[0, 0])) + 8
    in_y = int(round(inner[0, 1])) + 8
    assert np.abs(img[in_y, in_x].astype(int) - bg[in_y, in_x].astype(int)).max() <= 0.12 * 255 + 1

    # 4) outside the outer edge it is background again (frame has finite thickness)
    out_x = int(round(outer[0, 0])) - 8
    assert np.array_equal(img[cy, out_x], bg[cy, out_x])


def test_near_frame_occludes_far_window_but_opening_shows_it():
    """A nearer frame hides what is behind its ring; its opening does not."""
    from sim.procedural_render import render_scene, _RGB
    from sim.scene_gen import window_rotation_from_normal
    from common import window_corners_world

    scene, win, _ = _scene_one_window(0.2)
    far = dict(win, color="green", order_index=1, center=np.array([8.0, 0.0, 0.0]),
               width=1.0, height=1.0, frame_thickness_m=0.2)
    img = render_scene({**scene, "windows": [win, far]})
    bg = render_scene({**scene, "windows": []})
    R = window_rotation_from_normal(win["normal"])
    inner_near = _project(scene, window_corners_world(win["center"], R, 1.0, 1.0))
    cx, cy = (int(round(v)) for v in inner_near.mean(axis=0))
    # both openings are see-through at the shared centre: background survives
    # (two stacked tints of <= 10% each; renders draw different RNG tints, so compare to bg)
    assert np.abs(img[cy, cx].astype(int) - bg[cy, cx].astype(int)).max() <= 0.25 * 255
    gr = _RGB["green"]
    # the far frame's ring (0.5..0.7 m off-centre at 8 m) is visible THROUGH the near opening
    far_ring_x = cx + int(round(0.6 * scene["intrinsics"].fx / 8.0))
    px = img[cy, far_ring_x].astype(int)
    assert px[1] >= 0.7 * gr[1] and px[0] <= 60, f"far green ring should be visible through near opening: {px}"
    # ...and the near red ring (0.5..0.7 m at 4 m) hides the far window behind it
    near_ring_x = cx + int(round(0.6 * scene["intrinsics"].fx / 4.0))
    px = img[cy, near_ring_x].astype(int)
    rd = _RGB["red"]
    assert px[0] >= 0.7 * rd[0] and px[1] <= 60, f"near red ring should occlude: {px}"
