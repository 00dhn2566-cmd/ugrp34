"""Slung load (core/slung.py): the cable pendulum the next mission flies with."""
from __future__ import annotations

import math
import os
import sys

import pytest
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from rl_planner.core.slung import SlungLoad  # noqa: E402

G, DT = 9.81, 0.01
CFG = {"sim": {"gravity": G},
       "slung": {"cable_length": 0.4, "load_mass": 1.0, "load_radius": 0.12, "cable_stiffness": 5000.0,
                 "cable_damping": 50.0, "load_drag": 0.0, "attach": [0.0, 0.0, -0.05]}}
Q0 = torch.tensor([[1.0, 0.0, 0.0, 0.0]])


def _fixed_drone(sl, steps, pos=None):
    """Drone held still at pos (as if bolted to a test stand): only the load moves."""
    pos = torch.tensor([[0.0, 0.0, 2.0]]) if pos is None else pos
    z = torch.zeros(1, 3)
    out = []
    for _ in range(steps):
        sl.cable_force(pos, z, Q0, z)
        sl.integrate(DT)
        out.append(sl.pos[0].clone())
    return torch.stack(out)


def test_load_hangs_still_below_the_hook_with_its_weight_on_the_cable():
    sl = SlungLoad(CFG, 1, "cpu")
    p = torch.tensor([[0.0, 0.0, 2.0]])
    sl.reset(torch.tensor([0]), p, Q0)
    traj = _fixed_drone(sl, 500)
    assert float((traj - traj[0]).abs().max()) < 1e-3
    assert float(sl.tension[0]) == pytest.approx(G, rel=1e-3)
    assert float(traj[-1, 2]) == pytest.approx(2.0 - 0.05 - 0.4, abs=3e-3)


def test_small_swing_has_the_pendulum_period():
    sl = SlungLoad(CFG, 1, "cpu")
    p = torch.tensor([[0.0, 0.0, 2.0]])
    sl.reset(torch.tensor([0]), p, Q0)
    th = math.radians(5.0)
    lvec = 0.4 + G / 5000.0
    sl.pos[0] = torch.tensor([lvec * math.sin(th), 0.0, 2.0 - 0.05 - lvec * math.cos(th)])
    x = _fixed_drone(sl, 600)[:, 0]
    s = torch.sign(x)
    cross = ((s[1:] * s[:-1]) < 0).nonzero().squeeze(-1).float() * DT
    period = float(2 * (cross[1:] - cross[:-1]).mean())
    assert period == pytest.approx(2 * math.pi * math.sqrt(lvec / G), rel=0.02)
    assert float(x[-100:].abs().max()) > 0.9 * lvec * math.sin(th)          # barely damped


def test_cable_goes_slack_when_the_load_is_thrown_up():
    sl = SlungLoad(CFG, 1, "cpu")
    p = torch.tensor([[0.0, 0.0, 2.0]])
    sl.reset(torch.tensor([0]), p, Q0)
    sl.vel[0, 2] = 2.0
    _fixed_drone(sl, 5)
    assert float(sl.tension[0]) == 0.0
    assert float(sl.vel[0, 2]) == pytest.approx(2.0 - G * 5 * DT, abs=1e-4)   # free flight from the first step


def test_cable_conserves_momentum_between_drone_and_load():
    """No gravity, drone pushed sideways: the drone and the load trade momentum through the cable only."""
    cfg = {"sim": {"gravity": 0.0}, "slung": dict(CFG["slung"], attach=[0.0, 0.0, 0.0])}
    sl = SlungLoad(cfg, 1, "cpu")
    mq = 1.2726
    p, v = torch.tensor([[0.0, 0.0, 2.0]]), torch.tensor([[1.5, 0.0, 0.5]])
    sl.reset(torch.tensor([0]), p, Q0)
    sl.pos[0, 2] = 2.0 - 0.45                                             # taut-ish, a little stretched
    p0 = mq * v[0] + sl.m * sl.vel[0]
    z = torch.zeros(1, 3)
    for _ in range(300):
        f, _ = sl.cable_force(p, v, Q0, z)
        sl.integrate(DT)
        v = v + f / mq * DT
        p = p + v * DT
    assert torch.allclose(mq * v[0] + sl.m * sl.vel[0], p0, atol=1e-4)
    assert float(sl.tension.max()) >= 0.0


def test_hook_below_the_centre_of_mass_turns_a_sideways_pull_into_a_roll_torque():
    sl = SlungLoad(CFG, 1, "cpu")
    p = torch.tensor([[0.0, 0.0, 2.0]])
    sl.reset(torch.tensor([0]), p, Q0)
    sl.pos[0] = torch.tensor([0.0, 0.3, 2.0 - 0.05 - 0.3])                # load out to the left, cable stretched
    f, tau = sl.cable_force(p, torch.zeros(1, 3), Q0, torch.zeros(1, 3))
    assert float(f[0, 1]) > 0 and float(f[0, 2]) < 0                       # pulled left and down
    # r = (0,0,-0.05), F = (0, Fy, Fz): torque_x = r_y F_z - r_z F_y = 0.05 Fy > 0 (rolls the top left)
    assert float(tau[0, 0]) == pytest.approx(0.05 * float(f[0, 1]), rel=1e-5)
