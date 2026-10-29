"""Vision in the loop (rl_planner/vision/detector.py): projection and detection-to-window association (no Isaac, no
ultralytics needed)."""
from __future__ import annotations

import os
import sys
from types import SimpleNamespace

import pytest
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from rl_planner.vision.detector import associate, project  # noqa: E402

PR = SimpleNamespace(fx=763.0, fy=763.0, cx=640.0, cy=360.0)


def test_project_puts_the_optical_axis_on_the_principal_point():
    uv, z = project(PR, torch.tensor([[0.0, 0.0, 2.0], [0.5, -0.25, 2.0]]), torch.zeros(3), torch.eye(3))
    assert uv.flatten().tolist() == pytest.approx([640.0, 360.0, 640.0 + 763.0 * 0.25, 360.0 - 763.0 * 0.125])
    assert z.tolist() == pytest.approx([2.0, 2.0])


def _square(x0, y0, s):
    return torch.tensor([[x0, y0], [x0 + s, y0], [x0 + s, y0 + s], [x0, y0 + s]], dtype=torch.float32)


def test_associate_matches_by_colour_and_corner_distance_and_counts_the_rest_false():
    uv_gt = torch.stack((_square(100, 100, 200), _square(700, 300, 100)))       # window 0 red, window 1 green
    color, valid, z = torch.tensor([0, 1]), torch.tensor([True, True]), torch.ones(2, 4)
    dets = [(0, 0.9, (_square(100, 100, 200) + 3.0).numpy(), None),            # red, 3 px off -> window 0
            (0, 0.6, (_square(100, 100, 200) - 2.0).numpy(), None),            # a second red on window 0: false
            (2, 0.8, _square(700, 300, 100).numpy(), None),                    # blue where green is: false
            (1, 0.7, _square(1000, 50, 80).numpy(), None)]                     # green far from window 1: false
    uv, det, conf, false = associate(dets, uv_gt, z, color, valid)
    assert det.tolist() == [True, False] and false == 3
    assert conf[0].item() == pytest.approx(0.9)
    assert torch.allclose(uv[0], uv_gt[0] + 3.0)


def test_associate_ignores_windows_behind_the_camera():
    uv_gt = _square(100, 100, 200)[None]
    uv, det, conf, false = associate([(0, 0.9, _square(100, 100, 200).numpy(), None)], uv_gt,
                                     torch.tensor([[1.0, 1.0, -1.0, 1.0]]), torch.tensor([0]), torch.tensor([True]))
    assert not bool(det[0]) and false == 1


def test_annotate_draws_the_detection_and_keeps_the_frame_size():
    import numpy as np

    from rl_planner.vision.overlay import PAL, annotate
    img = np.full((720, 1280, 3), 128, np.uint8)
    kp = _square(400, 300, 200).numpy()
    out = np.asarray(annotate(img, [(1, 0.9, kp, None)], [kp], "t 1.0 s"))
    assert out.shape == img.shape
    assert tuple(out[304, 404]) == PAL[1]                  # the corner marker in the class colour (off the + of truth)
    assert tuple(out[500, 300]) == (128, 128, 128)         # untouched away from the marks


@pytest.mark.parametrize("blur_px,tol", [(0, 0.3), (14, 1.0)])
def test_refine_corners_snaps_perturbed_corners_to_the_inner_edge_of_the_frame(blur_px, tol):
    import cv2
    import numpy as np

    from rl_planner.vision.detector import refine_corners
    k = 8                                                  # drawn at 8x, area-averaged: pixel coverage
    big = np.full((720 * k, 1280 * k, 3), 140, np.uint8)
    true = np.array([[402.3, 201.7], [801.6, 230.2], [790.4, 551.8], [395.1, 540.6]])    # a perspective quad
    ctr = true.mean(0)
    outer = ctr + (true - ctr) * 1.15
    for poly, col in ((outer, (40, 190, 60)), (true, (140, 140, 140))):              # green bars, then the opening
        cv2.fillPoly(big, [np.round((poly * k - 0.5) * 16).astype(np.int32)], col, cv2.LINE_8, shift=4)
    img = cv2.resize(big, (1280, 720), interpolation=cv2.INTER_AREA)
    if blur_px:                                            # motion blur along a diagonal, centred (mid-exposure)
        ker = np.eye(blur_px + 1, dtype=np.float32)
        img = cv2.filter2D(img, -1, ker / ker.sum())
    kp = true + np.array([[6.0, -4.0], [-5.0, 5.0], [7.0, 3.0], [-4.0, -6.0]])        # a detector a few px off
    out, ok = refine_corners(img, kp, 1)
    assert ok.all()
    assert np.abs(out - true).max() < tol                  # from 5-8 px off
    _, ok_red = refine_corners(img, kp, 0)                 # no red frame there: nothing to snap to
    assert not ok_red.any()


def test_associate_tracked_uses_the_estimates_then_course_order():
    from rl_planner.vision.detector import associate_tracked
    color = torch.tensor([0, 1, 2, 0, 1])                  # windows 0..4: red green blue red green; 0 already passed
    uv_est = torch.stack([_square(0, 0, 10)] * 5)
    uv_est[1] = _square(500, 300, 100)                     # window 1 has an estimate, predicted here
    valid = torch.tensor([True, True, False, False, False])
    dets = [(1, 0.6, _square(503, 302, 98).numpy(), None),     # green at window 1's prediction -> 1
            (1, 0.9, _square(100, 100, 40).numpy(), None),     # another green: no estimate there -> 4, next green ahead
            (0, 0.8, _square(700, 100, 60).numpy(), None),     # red, smaller: no red window left -> false
            (0, 0.7, _square(200, 200, 150).numpy(), None),    # red, larger -> 3, first red ahead (nearer = larger)
            (2, 0.5, _square(900, 500, 30).numpy(), None)]     # blue -> 2
    uv, det, conf, false = associate_tracked(dets, uv_est, torch.ones(5, 4), valid, color, target=1, n=5)
    assert det.tolist() == [False, True, True, True, True] and false == 1
    for w, j in ((1, 0), (2, 4), (3, 3), (4, 1)):
        assert torch.allclose(uv[w], torch.as_tensor(dets[j][2]))
