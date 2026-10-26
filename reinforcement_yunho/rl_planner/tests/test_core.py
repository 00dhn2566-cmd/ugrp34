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


def test_safety_filter_holds_blind_drone_and_passes_nominal_missions():
    """planners/safety.py: a planner that backs away when it sees nothing (what the RL policy
    does with a dead camera) is held in place; nominal missions still succeed."""
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.eval_harness import run_planner, summarize
    from rl_planner.planners.fsm import FSMPlanner
    from rl_planner.planners.safety import SafePlanner

    class Backer:
        def reset(self, ids):
            pass

        def act(self, core):
            a = torch.zeros(core.E, core.action_dim)
            a[:, 0] = -1.0
            return a

    cfg = cfg_cpu("sim.num_envs=16", "perception.det_conf_min=1.01")
    for guarded in (False, True):
        env = TorchWindowEnv(cfg, num_envs=16, device="cpu", seed=5)
        env.core.fixed_level = 3
        env._reset_all()
        p0 = env.core.state.pos.clone()
        pl = SafePlanner(Backer(), cfg, 16, "cpu", env.core.step_dt) if guarded else Backer()
        pl.reset(torch.arange(16))
        crashed = torch.zeros(16, dtype=torch.bool)
        for _ in range(60):
            _, _, d, _ = env.step(pl.act(env.core))
            crashed |= d.bool() & env.core.episode_record["crashed"]   # auto-reset: read the record
        if guarded:
            assert not bool(crashed.any())
            assert float((env.core.state.pos - p0).norm(dim=-1).max()) < 0.3
        else:
            assert bool(crashed.all())

    cfg = cfg_cpu("sim.num_envs=32", "seed=3")
    env = TorchWindowEnv(cfg, num_envs=32, device="cpu")
    env.core.fixed_level = 0
    env._reset_all()
    s = summarize(run_planner(env, SafePlanner(FSMPlanner(cfg, 32, "cpu"), cfg, 32, "cpu", env.core.step_dt)))
    assert s["success_rate"] >= 0.9


def test_detection_latency_delivers_frames_late():
    """perception.latency_steps: the triangulator (and so estimates and observations) receives each
    camera frame that many decisions after it was taken, also the first frame of an episode."""
    from rl_planner.envs.torch_env import TorchWindowEnv
    hist = {}
    for lat in (0, 2):
        cfg = cfg_cpu("sim.num_envs=8", f"perception.latency_steps={lat}")
        env = TorchWindowEnv(cfg, num_envs=8, device="cpu", seed=4)
        env.core.fixed_level = 3
        env.core.reseed(11)
        env._reset_all()
        n_obs = [env.core.perception.n_obs.clone()]
        for _ in range(12):
            _, _, d, _ = env.step(torch.zeros(8, env.num_actions))    # hover: same flight in both runs
            assert not bool(d.any())
            n_obs.append(env.core.perception.n_obs.clone())
        hist[lat] = n_obs
    assert float(hist[0][12].sum()) > 0
    assert float(hist[2][0].sum()) == 0 and float(hist[2][1].sum()) == 0
    for k in range(2, 13):
        assert torch.equal(hist[2][k], hist[0][k - 2])


def test_detection_gap_keeps_counting_through_a_camera_outage():
    """detection_gap counts decisions on the wall clock; target_detection's age counts camera frames
    and freezes while no frames arrive, so only the gap tells a long outage from a missed frame."""
    from rl_planner.core.observations import REGISTRY
    from rl_planner.envs.torch_env import TorchWindowEnv
    cfg = cfg_cpu("sim.num_envs=16", "perception.blackout_s=[0.5,2.0]")
    env = TorchWindowEnv(cfg, num_envs=16, device="cpu", seed=2)
    core = env.core
    core.fixed_level = 3
    core.reseed(5)
    env._reset_all()
    hover = torch.zeros(16, env.num_actions)
    for _ in range(5):
        env.step(hover)                                    # t = 0.5 s: the outage starts at the next frame
    tgt = core.target_idx[:, None]
    gap0 = REGISTRY["detection_gap"].fn(core)[:, 1].clone()
    age0 = core.perception.det_age.gather(1, tgt).squeeze(1).clone()
    for k in range(1, 11):
        _, _, d, _ = env.step(hover)
        assert not bool(d.any())
        gap = REGISTRY["detection_gap"].fn(core)[:, 1]
        assert torch.allclose(gap, (gap0 + k * core.step_dt / 10.0).clamp_max(1.0), atol=1e-5)
        assert torch.equal(core.perception.det_age.gather(1, tgt).squeeze(1), age0)
    for _ in range(10):
        env.step(hover)                                    # frames are back after 2 s
    assert bool((REGISTRY["detection_gap"].fn(core)[:, 1] == 0).any())


def test_expand_obs_keeps_the_trained_policy():
    """scripts/expand_obs.py: appending an observation term leaves the first-layer output unchanged."""
    from rl_planner.core.observations import build_spec
    from rl_planner.scripts.expand_obs import expand_checkpoint, layouts
    old = cfg_cpu().to_dict()
    new = cfg_cpu("observation.policy=[lin_vel_heading,gravity_body,ang_vel_body,altitude,last_action,"
                  "target_estimate,detection_gap,target_detection,next_estimate,progress]").to_dict()
    g = torch.Generator().manual_seed(0)
    ck = {"iter": 7, "infos": None}
    for key, (lay_o, lay_n) in (("actor_state_dict", (layouts(old)[0], layouts(new)[0])),
                                ("critic_state_dict", (layouts(old)[1], layouts(new)[1]))):
        D = lay_o[-1][2]
        ck[key] = {"obs_normalizer._mean": torch.randn(1, D, generator=g),
                   "obs_normalizer._std": torch.rand(1, D, generator=g) + 0.5,
                   "obs_normalizer._var": torch.rand(1, D, generator=g) + 0.5,
                   "mlp.0.weight": torch.randn(32 if key[0] == "a" else 48, D, generator=g)}
    ck["optimizer_state_dict"] = {"state": {0: {"exp_avg": torch.randn(32, layouts(old)[0][-1][2], generator=g),
                                                "exp_avg_sq": torch.rand(32, layouts(old)[0][-1][2], generator=g)}},
                                  "param_groups": []}
    out = expand_checkpoint(ck, old, new)
    for key, (lay_o, lay_n) in (("actor_state_dict", (layouts(old)[0], layouts(new)[0])),
                                ("critic_state_dict", (layouts(old)[1], layouts(new)[1]))):
        x_old = torch.randn(4, lay_o[-1][2], generator=g)
        x_new = torch.randn(4, lay_n[-1][2], generator=g)              # new term: arbitrary values
        src = {k: (s, e) for k, s, e in lay_o}
        for k, s, e in lay_n:
            if k in src:
                x_new[:, s:e] = x_old[:, src[k][0]:src[k][1]]
        f = lambda sd, x: ((x - sd["obs_normalizer._mean"]) / (sd["obs_normalizer._std"] + 1e-2)) @ sd["mlp.0.weight"].T  # noqa: E731
        assert torch.allclose(f(ck[key], x_old), f(out[key], x_new), atol=1e-5)
    assert out["optimizer_state_dict"]["state"][0]["exp_avg"].shape == (32, layouts(new)[0][-1][2])
    assert len(build_spec(new, "policy")) == len(build_spec(old, "policy")) + 1


def test_eval_config_turns_training_only_augmentations_off():
    from rl_planner.core.config import config_for_policy
    trained = cfg_cpu("domain_randomization.blackout_prob=0.5", "curriculum.fixed_level=5",
                      "domain_randomization.latency_steps=[0,2]", "reward.crash=-30").to_dict()
    c = config_for_policy(trained)
    assert c["domain_randomization"]["blackout_prob"] == 0.0 and c["curriculum"]["fixed_level"] is None
    assert list(c["domain_randomization"]["latency_steps"]) == [0, 0]
    assert c["reward"]["crash"] == -30                    # everything else follows the training config
    c = config_for_policy(trained, overrides=["domain_randomization.blackout_prob=0.3"])
    assert c["domain_randomization"]["blackout_prob"] == 0.3
    trained["safety"]["lost_hold_s"] = 99.0              # runtime section: today's default, not the snapshot
    assert config_for_policy(trained)["safety"]["lost_hold_s"] == cfg_cpu()["safety"]["lost_hold_s"]
