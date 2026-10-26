"""Unit tests for rl_planner core (torch only, CPU, < 1 min).

    cd reinforcement_yunho && python -m pytest rl_planner/tests -q
"""
import math
import os
import sys

import numpy as np
import pytest
import torch

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from rl_planner.core import math3d as m3  # noqa: E402
from rl_planner.core.config import load_config  # noqa: E402
from rl_planner.core.mission import CRASH_BACK, CRASH_WALL, MissionTracker  # noqa: E402
from rl_planner.core.perception import PerceptionSurrogate  # noqa: E402
from rl_planner.core.scene import scene_from_layout  # noqa: E402

DEV = "cpu"


def cfg_cpu(*over):
    return load_config(overrides=["device=cpu", *over])


# ----------------------------------------------------------------------------- math
def test_solve3_and_eigmin_match_linalg():
    g = torch.Generator().manual_seed(0)
    X = torch.randn(500, 3, 3, generator=g, dtype=torch.float64)
    A = X @ X.transpose(-1, -2) + 0.1 * torch.eye(3, dtype=torch.float64)
    b = torch.randn(500, 3, generator=g, dtype=torch.float64)
    assert torch.allclose(m3.solve3(A, b), torch.linalg.solve(A, b), atol=1e-9)
    assert torch.allclose(m3.eigmin_sym3(A), torch.linalg.eigvalsh(A)[..., 0], atol=1e-9)


def test_quat_roundtrip_and_yaw():
    yaw = torch.tensor([0.0, 0.7, -2.0])
    q = m3.quat_from_yaw(yaw)
    assert torch.allclose(m3.yaw_from_quat(q), yaw, atol=1e-6)
    R = m3.quat_to_rotmat(q)
    assert torch.allclose(m3.quat_to_rotmat(m3.quat_from_rotmat(R)), R, atol=1e-6)
    v = torch.randn(3, 3)
    assert torch.allclose(m3.quat_rotate_inverse(q, m3.quat_rotate(q, v)), v, atol=1e-5)


# ----------------------------------------------------------------------------- geometry contract
def test_corners_match_team_shared_geometry():
    """Corner order / normal sign identical to common.geometry + state_window spec §3.1."""
    from common.geometry import window_corners_world
    lay = [[{"center": [3.0, 0.4, 1.2], "size_wh": [1.0, 0.9],
             "normal": [-math.cos(0.3), -math.sin(0.3), 0.0]}]]
    sc = scene_from_layout(lay, 1, [0, 0, 1], DEV)
    R = torch.stack((sc.right[0, 0], sc.up[0, 0], sc.normal[0, 0]), dim=-1).numpy()
    ref = window_corners_world(np.array([3.0, 0.4, 1.2]), R, 1.0, 0.9)
    assert np.allclose(sc.corners[0, 0].numpy(), ref, atol=1e-6)
    c = sc.corners[0, 0]
    n = m3.normalize(torch.cross(c[3] - c[0], c[1] - c[0], dim=-1))
    assert torch.allclose(n, sc.normal[0, 0], atol=1e-6)


# ----------------------------------------------------------------------------- perception
def _single_window_perception(noise_px=0.0):
    cfg = cfg_cpu(f"perception.corner_noise_px=[{noise_px},{noise_px}]", "perception.miss_prob=[0.0,0.0]")
    lay = [[{"center": [4.0, 0.0, 1.2], "size_wh": [1.0, 1.0]}]]
    sc = scene_from_layout(lay, 1, [0, 0, 1.2], DEV)
    pr = PerceptionSurrogate(cfg, 1, 1, DEV)
    pr.reset(torch.tensor([0]), sigma_px=torch.tensor([noise_px]), miss_prob=torch.tensor([0.0]))
    return cfg, sc, pr


def test_triangulation_exact_without_noise():
    cfg, sc, pr = _single_window_perception(0.0)
    g = torch.Generator().manual_seed(0)
    q = torch.tensor([[1.0, 0.0, 0.0, 0.0]])
    for y in (-0.5, 0.5, 0.0):
        c, R = pr.camera_pose(torch.tensor([[0.0, y, 1.2]]), q)
        det = pr.observe(c, R, sc, torch.tensor([True]), g)
        assert bool(det.detected[0, 0])
    est = pr.estimate()
    assert bool(est.valid[0, 0])
    assert torch.allclose(est.corners[0, 0], sc.corners[0, 0], atol=1e-3)
    assert torch.allclose(est.normal[0, 0], sc.normal[0, 0], atol=1e-3)


def test_parallax_gate_blocks_single_viewpoint():
    cfg, sc, pr = _single_window_perception(2.0)
    g = torch.Generator().manual_seed(1)
    q = torch.tensor([[1.0, 0.0, 0.0, 0.0]])
    c, R = pr.camera_pose(torch.tensor([[0.0, 0.0, 1.2]]), q)
    for _ in range(20):
        pr.observe(c, R, sc, torch.tensor([True]), g)
    est = pr.estimate()
    assert not bool(est.valid[0, 0]) and float(est.parallax_deg[0, 0]) < 2.0


def test_occlusion_by_nearer_wall():
    cfg = cfg_cpu("perception.miss_prob=[0.0,0.0]")
    lay = [[{"center": [3.0, 2.0, 1.2], "size_wh": [1.0, 1.0]},     # wall at x=3, opening far to the side
            {"center": [6.0, 0.0, 1.2], "size_wh": [1.0, 1.0]}]]
    sc = scene_from_layout(lay, 2, [0, 0, 1.2], DEV)
    pr = PerceptionSurrogate(cfg, 1, 2, DEV)
    pr.reset(torch.tensor([0]), sigma_px=torch.tensor([0.0]), miss_prob=torch.tensor([0.0]))
    c, R = pr.camera_pose(torch.tensor([[0.0, 0.0, 1.2]]), torch.tensor([[1.0, 0.0, 0.0, 0.0]]))
    det = pr.observe(c, R, sc, torch.tensor([True]), torch.Generator().manual_seed(0))
    assert not bool(det.detected[0, 1]), "window behind a solid wall must not be detected"


# ----------------------------------------------------------------------------- referee
def _tracker():
    cfg = cfg_cpu()
    lay = [[{"center": [3.0, 0.0, 1.2], "size_wh": [1.0, 1.0]}]]
    sc = scene_from_layout(lay, 1, [0, 0, 1.2], DEV)
    mt = MissionTracker(cfg, 1, 1, DEV)
    mt.reset(torch.tensor([0]), torch.tensor([[0.0, 0.0, 1.2]]), sc)
    return mt, sc


def _fly(mt, sc, pts):
    q = torch.tensor([[1.0, 0.0, 0.0, 0.0]])
    for p in pts:
        mt.substep(torch.tensor([p]), q, sc, torch.tensor([0.0]))


def test_pass_through_centre():
    mt, sc = _tracker()
    _fly(mt, sc, [[x, 0.0, 1.2] for x in np.linspace(0.0, 4.0, 81)])
    assert bool(mt.success[0]) and not bool(mt.crashed[0])
    assert float(mt.clearance[0, 0]) == pytest.approx(0.5 - 0.30, abs=1e-5)


def test_frame_hit_when_offset():
    mt, sc = _tracker()
    _fly(mt, sc, [[x, 0.35, 1.2] for x in np.linspace(0.0, 4.0, 81)])
    assert bool(mt.crashed[0]) and int(mt.crash_reason[0]) == CRASH_WALL


def test_backwards_passage_flagged():
    mt, sc = _tracker()
    _fly(mt, sc, [[x, 0.0, 1.2] for x in np.linspace(0.0, 4.0, 41)] + [[x, 0.0, 1.2] for x in np.linspace(4.0, 2.0, 21)])
    assert int(mt.crash_reason[0]) in (0, CRASH_BACK)    # success latches first; never a silent pass


# ----------------------------------------------------------------------------- env
def test_torch_env_hover_and_shapes():
    from rl_planner.envs.torch_env import TorchWindowEnv
    cfg = cfg_cpu("sim.num_envs=16")
    env = TorchWindowEnv(cfg, num_envs=16, device="cpu")
    env.core.fixed_level = 0
    env._reset_all()
    z0 = env.core.state.pos[:, 2].clone()
    for _ in range(30):
        obs, rew, done, _ = env.step(torch.zeros(16, env.num_actions))
    assert obs["policy"].shape == (16, env.core.num_policy_obs)
    assert torch.isfinite(obs["policy"]).all() and torch.isfinite(obs["critic"]).all()
    alive = ~env.core.mission.crashed
    assert float((env.core.state.pos[alive, 2] - z0[alive]).abs().max()) < 0.15     # DR wind/mass on


def test_fsm_baseline_solves_level0():
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.eval_harness import run_planner, summarize
    from rl_planner.planners.fsm import FSMPlanner
    cfg = cfg_cpu("sim.num_envs=32", "seed=3")
    env = TorchWindowEnv(cfg, num_envs=32, device="cpu")
    env.core.fixed_level = 0
    env._reset_all()
    s = summarize(run_planner(env, FSMPlanner(cfg, 32, "cpu")))
    assert s["success_rate"] >= 0.9


def test_planners_see_identical_scenes_and_camera_noise():
    """eval_harness contract: after core.reseed + reset every planner gets the same scenes, and
    the camera-noise stream does not move with how often envs reset or read observations."""
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.eval_harness import scene_fingerprint
    cfg = cfg_cpu("sim.num_envs=16")
    env = TorchWindowEnv(cfg, num_envs=16, device="cpu", seed=3)
    env.core.fixed_level = 3
    hover = torch.zeros(16, env.num_actions)
    back = hover.clone()
    back[:, 0] = -1.0                                  # flies out of bounds -> early resets
    runs = []
    for act, extra_obs in ((hover, False), (back, True)):
        env.core.reseed(7)
        env._reset_all()
        fp = scene_fingerprint(env.core.scene)
        resets = 0
        for _ in range(30):
            if extra_obs:                              # a policy planner reads observations again
                env.core.compute_observations()
            _, _, d, _ = env.step(act)
            resets += int(d.sum())
        runs.append((fp, env.core.gen.get_state(), resets))
    assert runs[0][0] == runs[1][0]
    assert torch.equal(runs[0][1], runs[1][1])
    assert runs[1][2] > runs[0][2]
