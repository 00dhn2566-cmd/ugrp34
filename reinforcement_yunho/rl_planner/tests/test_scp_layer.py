"""The sequential-convex layer for the long cable (planners/convex_layer.py SlungSCPLayer, RESULTS.md §15):
its prediction model against the simulator (taut swings and a thrown load on a slack cable) and the projection
being the identity when no constraint binds."""
import os

import torch

from rl_planner.core import math3d as m3
from rl_planner.core.config import load_config
from rl_planner.envs.torch_env import TorchWindowEnv
from rl_planner.planners.convex_layer import SlungSCPLayer

C = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs", "slung")
FAR = ["curriculum.override={lateral: [0.0, 0.0], first_gap: [30.0, 30.0], n_windows: [1, 1]}",
       "scene.bounds={x_min: -30.0, y_abs: 30.0, z_min: 0.15, z_max: 3.5}", "domain_randomization.enabled=false",
       "curriculum.fixed_level=0", "device=cpu"]


def _env(E, seed, start_z=1.6):
    cfg = load_config([os.path.join(C, "base.yaml"), os.path.join(C, "long.yaml")],
                      FAR + [f"drone.start_pos=[0.0, 0.0, {start_z}]"])
    env = TorchWindowEnv(cfg, num_envs=E, device="cpu", seed=seed)
    env.core.fixed_level = 0
    env._reset_all()
    return cfg, env


def _predict(lay, core, acts):
    st = core.state
    X = lay.state(st.pos, st.vel_w, core.slung.pos - st.pos, core.slung.vel - st.vel_w, st.quat, st.omega_b)
    yaw = core.yaw.clone()
    out = []
    for k in range(acts.shape[1]):
        w = X[:, :3] + m3.heading_to_world(acts[:, k] * lay.step_max, yaw)
        X = lay._step(X, torch.cat((w[:, :2], w[:, 2:].clamp(*lay.wp_z)), -1))
        out.append(X.clone())
    return torch.stack(out, 1)


def _fly(env, acts):
    core, alive, Q, Ld, slack = env.core, torch.ones(acts.shape[0], dtype=torch.bool), [], [], []
    for k in range(acts.shape[1]):
        _, _, d, _ = env.step(acts[:, k])
        alive &= ~d.bool()
        Q.append(core.state.pos.clone())
        Ld.append(core.slung.pos.clone())
        slack.append(core.slung.tension < 1e-6)
    return torch.stack(Q, 1), torch.stack(Ld, 1), torch.stack(slack, 1), alive


def test_scp_model_follows_the_simulator_swinging():
    torch.manual_seed(0)
    cfg, env = _env(64, 1)
    lay = SlungSCPLayer(cfg, 64, "cpu", env.core.step_dt)
    eps = torch.randn(64, 20 + lay.N, 3)
    for k in range(1, eps.shape[1]):
        eps[:, k] = 0.7 * eps[:, k - 1] + 0.5 * eps[:, k]
    acts = (0.9 * eps).clamp(-1, 1)
    acts[:, :, 1] *= 0.5
    for k in range(20):                                           # swing it up first
        env.step(acts[:, k])
    pred = _predict(lay, env.core, acts[:, 20:])
    Q, Ld, _, alive = _fly(env, acts[:, 20:])
    eQ = (pred[:, -1, :3] - Q[:, -1]).norm(dim=-1)[alive]
    eL = (pred[:, -1, 6:9] - Ld[:, -1]).norm(dim=-1)[alive]
    assert int(alive.sum()) > 32
    assert float(eQ.median()) < 0.10 and float(eL.median()) < 0.10, (float(eQ.median()), float(eL.median()))


def test_scp_model_follows_a_thrown_load_on_a_slack_cable():
    torch.manual_seed(1)
    cfg, env = _env(64, 2, start_z=2.0)
    lay = SlungSCPLayer(cfg, 64, "cpu", env.core.step_dt)
    for _ in range(5):
        env.step(torch.zeros(64, 3))
    core = env.core
    up = 1.5 + 2.5 * torch.rand(64)
    side = (torch.rand(64, 2) - 0.5) * 3.0
    core.slung.vel = core.state.vel_w + torch.cat((side, up[:, None]), -1)
    acts = (0.6 * torch.randn(64, lay.N, 3)).clamp(-1, 1)
    pred = _predict(lay, core, acts)
    Q, Ld, slack, alive = _fly(env, acts)
    assert float(slack.float().mean(1)[alive].median()) > 0.2       # the cable really went slack
    eL = (pred[:, -1, 6:9] - Ld[:, -1]).norm(dim=-1)[alive]
    eQ = (pred[:, -1, :3] - Q[:, -1]).norm(dim=-1)[alive]
    assert float(eL.median()) < 0.10 and float(eQ.median()) < 0.15, (float(eL.median()), float(eQ.median()))


def test_scp_projection_is_identity_when_nothing_binds():
    cfg = load_config([os.path.join(C, "base.yaml"), os.path.join(C, "long.yaml")], ["device=cpu"])
    E = 16
    lay = SlungSCPLayer(cfg, E, "cpu", 0.1)
    torch.manual_seed(2)
    pos = torch.tensor([0.0, 0.0, 2.0]).repeat(E, 1)
    load_rel = torch.tensor([0.0, 0.0, -1.05]).repeat(E, 1)
    a = torch.cat((0.3 * (torch.rand(E, 3) * 2 - 1), torch.zeros(E, 1)), -1)
    far = torch.tensor([30.0, 0.0, 1.2]).repeat(E, 1)
    nrm = torch.tensor([-1.0, 0.0, 0.0]).repeat(E, 1)
    half = torch.tensor([0.5, 0.5]).repeat(E, 1)
    on = torch.ones(E, dtype=torch.bool)
    out, changed = lay.project(a, pos, torch.zeros(E, 3), torch.zeros(E), load_rel, torch.zeros(E, 3),
                               [(far, nrm, half, on, False), (far, nrm, half, ~on, True)])
    assert float((out - a).abs().max()) < 2e-3 and not bool(changed.any())
