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
