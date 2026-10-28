"""Convex layer for the slung load (planners/convex_layer.py) on hand-built states."""
from __future__ import annotations

import math
import os
import sys

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from rl_planner.planners.convex_layer import SlungConvexLayer  # noqa: E402

CFG = {"sim": {"gravity": 9.81}, "drone": {"mass": 2.2726},
       "controller": {"kp_pos": [1.6, 1.6, 2.0], "kv": [3.0, 3.0, 4.0]},
       "action": {"step_max": [0.8, 0.6, 0.4]},
       "slung": {"cable_length": 0.4, "load_mass": 1.0, "load_radius": 0.12, "cable_stiffness": 5000.0,
                 "attach": [0.0, 0.0, -0.05], "convex_layer": {"horizon_s": 1.0, "block": 3, "max_swing_deg": 25.0, "margin_m": 0.05,
                                  "gate_band_m": 0.35, "iters": 400, "rho": 1.0}}}
L = 0.4 + 9.81 / 5000.0


def _layer(E=1, **kw):
    cfg = dict(CFG, slung=dict(CFG["slung"], convex_layer=dict(CFG["slung"]["convex_layer"], **kw)))
    return SlungConvexLayer(cfg, E, "cpu", 0.1)


def _state(E=1, z=1.5, vx=0.0):
    pos = torch.tensor([[0.0, 0.0, z]]).repeat(E, 1)
    vel = torch.tensor([[vx, 0.0, 0.0]]).repeat(E, 1)
    rel = torch.tensor([[0.0, 0.0, -(L + 0.05)]]).repeat(E, 1)
    return pos, vel, torch.zeros(E), rel, torch.zeros(E, 3)


def _swing(layer, a, pos, vel, rel, rvel):
    """Largest predicted horizontal load offset over the horizon for action a held (world steps)."""
    E = a.shape[0]
    M = torch.diag(layer.step_max).expand(E, 3, 3)
    s0 = torch.stack((pos[:, 0], vel[:, 0], pos[:, 0] + rel[:, 0], vel[:, 0] + rvel[:, 0],
                      pos[:, 1], vel[:, 1], pos[:, 1] + rel[:, 1], vel[:, 1] + rvel[:, 1], pos[:, 2], vel[:, 2]), -1)
    s = s0
    out = []
    At = layer.Phi[0]
    B = layer.G0[0]
    for _ in range(layer.N):
        s = s @ At.T + (M @ a[:, :3, None])[..., 0] @ B.T
        out.append((s[:, 2] - s[:, 0]).abs())
    return torch.stack(out, -1).amax(-1)


def test_gentle_action_passes_unchanged():
    layer = _layer()
    a = torch.tensor([[0.15, 0.0, 0.0, 0.3]])
    out, changed = layer.project(a, *_state()[:3], *_state()[3:], gates=[])
    assert torch.allclose(out, a, atol=2e-3) and not bool(changed[0])


def test_hard_start_is_softened_to_keep_the_swing_small():
    """Full forward from a hover swings the load ~21 deg (predicted); with a 12 deg limit the layer slows the start."""
    layer = _layer(max_swing_deg=12.0)
    a = torch.tensor([[1.0, 0.0, 0.0, 0.0]])
    pos, vel, yaw, rel, rvel = _state()
    assert float(_swing(layer, a, pos, vel, rel, rvel)) > L * math.sin(math.radians(12.0))     # would swing too far
    out, changed = layer.project(a, pos, vel, yaw, rel, rvel, gates=[])
    assert bool(changed[0]) and 0.0 < float(out[0, 0]) < 1.0
    assert float(out[0, 3]) == 0.0                                                             # yaw untouched


def test_drone_climbs_so_the_hanging_load_clears_the_bottom_frame():
    """Window 0.6 m ahead, its centre at the drone's height: the load 0.4 m below would hit the bottom bar
    (0.4 + 0.12 > 0.5), so the layer adds a climb; the drone itself still fits under the top bar."""
    layer = _layer()
    pos, vel, yaw, rel, rvel = _state(z=1.5, vx=0.8)
    gate = (torch.tensor([[0.6, 0.0, 1.5]]), torch.tensor([[-1.0, 0.0, 0.0]]), torch.tensor([[0.5, 0.5]]),
            torch.tensor([True]), False)
    a = torch.tensor([[0.5, 0.0, 0.0, 0.0]])
    out, changed = layer.project(a, pos, vel, yaw, rel, rvel, gates=[gate])
    assert bool(changed[0]) and float(out[0, 2]) > 0.1
    # without the window there is nothing to fix
    out2, changed2 = layer.project(a, pos, vel, yaw, rel, rvel, gates=[])
    assert not bool(changed2[0])
