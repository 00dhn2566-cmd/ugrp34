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
        if guarded:                         # holds where it brakes to a stop (hold_brake_acc), < 1 m back
            assert not bool(crashed.any())
            assert float((env.core.state.pos - p0).norm(dim=-1).max()) < 0.8
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


# ----------------------------------------------------------------------------- general-robustness knobs
def _env(*over, n=16, seed=3, level=3):
    from rl_planner.envs.torch_env import TorchWindowEnv
    cfg = cfg_cpu(f"sim.num_envs={n}", *over)
    env = TorchWindowEnv(cfg, num_envs=n, device="cpu", seed=seed)
    env.core.fixed_level = level
    env.core.reseed(seed)
    env._reset_all()
    return env


def test_robustness_knobs_off_draw_nothing_extra():
    """Every new knob defaults to off: no disturbance draws, no mounting error, no controller variation."""
    env = _env()
    s0 = env.core.dist_gen.get_state()
    for _ in range(5):
        env.step(torch.zeros(16, env.num_actions))
    assert torch.equal(env.core.dist_gen.get_state(), s0)
    assert env.core.R_err is None and env.core.controller.kp_s is None
    assert float(env.core.gust.abs().max()) == 0.0 and float(env.core.drift.abs().max()) == 0.0


def test_gusts_follow_the_ou_statistics():
    env = _env("domain_randomization.gust_accel=[0.5,0.5]", "domain_randomization.gust_tau_s=1.0", n=256)
    core = env.core
    xs = []
    for _ in range(60):
        core.pre_decision(torch.zeros(256, env.num_actions))       # gust update only (no physics needed)
        xs.append(core.gust[:, 0].clone())
    x = torch.stack(xs)
    assert abs(float(x.std()) - 0.5) < 0.05
    lag = 10                                                        # 1 s at 10 Hz -> correlation e^-1
    c = float((x[:-lag] * x[lag:]).mean() / x.var())
    assert abs(c - math.exp(-1.0)) < 0.1
    assert float(core.gust[:, 2].abs().max()) == 0.0                # horizontal only


def test_frame_drop_rate_and_drop_dr():
    env = _env("perception.frame_drop_prob=0.3", n=64)
    taken = []
    for _ in range(40):
        env.step(torch.zeros(64, env.num_actions))
        taken.append(env.core.det.frame_taken.float().mean())
    assert abs(float(torch.stack(taken).mean()) - 0.7) < 0.05
    env = _env("domain_randomization.frame_drop_prob=[0.0,0.4]", n=64)
    p = env.core.drop_p
    assert float(p.min()) >= 0.0 and float(p.max()) <= 0.4 and float(p.std()) > 0.05


def test_latency_jitter_delivers_each_frame_at_most_once_and_in_order():
    """latency 1 + jitter U{0..2}: frames arrive 1-3 decisions late, never twice, never older than one
    already delivered (latest-frame semantics)."""
    env = _env("perception.latency_steps=1", "perception.latency_jitter_steps=2", n=32)
    core = env.core
    last = torch.full((32,), -1, dtype=torch.long)
    delivered = torch.zeros(32)
    for k in range(30):
        env.step(torch.zeros(32, env.num_actions))
        cur = core._last_cap.clone()
        assert bool((cur >= last).all())
        delivered += (cur > last).float()
        last = cur
    assert 0.5 * 30 < float(delivered.mean()) < 30                 # some frames are superseded and dropped
    assert core.lat_max == 3


def test_camera_mount_error_biases_the_triangulation():
    cfg, sc, pr = _single_window_perception(0.0)
    q = torch.tensor([[1.0, 0.0, 0.0, 0.0]])
    R_err = m3.rot_axis_angle(torch.tensor([[0.0, 1.0, 0.0]]), torch.tensor([math.radians(2.0)]))
    errs = {}
    for name, R in (("nominal", None), ("mount_2deg", R_err)):
        pr.reset(torch.tensor([0]), sigma_px=torch.tensor([0.0]), miss_prob=torch.tensor([0.0]))
        for y in (-0.6, 0.6, 0.0):
            c, Rwc = pr.camera_pose(torch.tensor([[0.0, y, 1.2]]), q)
            pr.add_detections(*pr.capture(c, Rwc, sc, torch.tensor([True]), torch.Generator().manual_seed(0), R_err=R))
        est = pr.estimate()
        errs[name] = float((est.center[0, 0] - sc.center[0, 0]).norm())
    assert errs["nominal"] < 1e-3
    assert 0.05 < errs["mount_2deg"] < 0.3                           # ~ range x tan(2 deg) = 0.14 m


def test_velocity_integral_cancels_wind_drift():
    """With zero actions the waypoint follows the drone, so a P-only cascade drifts downwind at a_w / kv
    (0.2 m/s here); the velocity integral (domain_randomization.ctrl_ki) stops the drift after a_w / ki."""
    speed, drift = {}, {}
    for ki in (0.0, 1.5):
        env = _env("domain_randomization.wind_accel=[0.6,0.6]", "domain_randomization.mass_scale=[1.0,1.0]",
                   f"domain_randomization.ctrl_ki=[{ki},{ki}]", "domain_randomization.drag_scale=[1.0,1.0]",
                   "perception.det_conf_min=1.01", n=8, level=0)
        p0 = env.core.state.pos.clone()
        for _ in range(80):
            env.step(torch.zeros(8, env.num_actions))
        speed[ki] = float(env.core.state.vel_w[:, :2].norm(dim=-1).mean())
        drift[ki] = float((env.core.state.pos - p0)[:, :2].norm(dim=-1).mean())
    assert speed[0.0] > 0.15 and speed[1.5] < 0.03
    assert drift[1.5] < 0.6 / 1.5 + 0.05 < drift[0.0]


def test_gain_scale_and_privileged_dr_params():
    env = _env("domain_randomization.ctrl_kp_scale=[0.5,1.5]", "domain_randomization.ctrl_kv_scale=[0.7,0.7]",
               "domain_randomization.cam_rot_err_deg=[1.0,3.0]",
               "observation.critic=[target_gt,next_gt,estimate_error,dr_params]", n=32)
    kp, kv, ki = env.core.controller.variation()
    assert 0.5 <= float(kp.min()) < float(kp.max()) <= 1.5 and torch.allclose(kv, torch.full_like(kv, 0.7))
    assert float(ki.abs().max()) == 0.0
    deg = torch.rad2deg(env.core.cam_err_rad)
    assert float(deg.min()) >= 1.0 - 1e-4 and float(deg.max()) <= 3.0 + 1e-4
    obs, _, _, _ = env.step(torch.zeros(32, env.num_actions))
    assert obs["critic"].shape[1] == 16 + 12 and torch.isfinite(obs["critic"]).all()


def test_level_scale_turns_dr_down_on_easy_levels():
    env = _env("domain_randomization.wind_accel=[0.5,0.5]", "domain_randomization.gust_accel=[0.4,0.4]",
               "domain_randomization.level_scale=[0.0,0.0,0.5,1.0,1.0,1.0]", n=8, level=2)
    assert torch.allclose(env.core.wind_acc[:, :2].norm(dim=-1), torch.full((8,), 0.25), atol=1e-5)
    assert torch.allclose(env.core.gust_sigma, torch.full((8,), 0.2), atol=1e-6)
    env = _env("domain_randomization.wind_accel=[0.5,0.5]", "domain_randomization.level_scale=[0.0,0.0,0.5,1.0]",
               n=8, level=0)
    assert float(env.core.wind_acc.abs().max()) == 0.0


def test_curriculum_override_applies_on_every_level():
    from rl_planner.core.config import level_params
    cfg = cfg_cpu("curriculum.override={opening_w: [0.8, 0.85], yaw_deg: [-45, 45]}")
    for lv in range(6):
        p = level_params(cfg, lv)
        assert list(p["opening_w"]) == [0.8, 0.85] and list(p["yaw_deg"]) == [-45, 45]
    assert list(level_params(cfg, 0)["corner_noise_px"]) == [1.0, 2.0]          # other keys keep the level's


def test_eval_config_reverts_dr_section_and_stress_knobs():
    from rl_planner.core.config import config_for_policy
    trained = cfg_cpu("domain_randomization.wind_accel=[0.0,0.8]", "domain_randomization.gust_accel=[0.0,0.4]",
                      "perception.frame_drop_prob=0.2", "curriculum.override={opening_w: [0.8, 0.9]}").to_dict()
    c = config_for_policy(trained)
    d = cfg_cpu()
    assert c["domain_randomization"] == d["domain_randomization"]
    assert c["perception"]["frame_drop_prob"] == 0.0 and c["curriculum"]["override"] == {}
    c = config_for_policy(trained, overrides=["domain_randomization.wind_accel=[0.6,1.0]"])
    assert list(c["domain_randomization"]["wind_accel"]) == [0.6, 1.0]


def test_recurrent_policy_export_matches_training_model_and_resets_per_env(tmp_path):
    """scripts/train.py exports a GRU actor as (obs, state) -> (action, state); PolicyPlanner keeps one state
    per env and zeroes it on reset, so batched evaluation equals running each env alone."""
    import json

    from rsl_rl.models import RNNModel
    from tensordict import TensorDict

    from rl_planner.planners.policy import PolicyPlanner
    from rl_planner.scripts.train import export_recurrent_jit, recurrent_meta, runner_cfg
    cfg = cfg_cpu("train.policy.actor_type=gru", "train.policy.rnn_hidden_dim=16")
    rc = runner_cfg(cfg)
    a = {k: v for k, v in rc["actor"].items() if k != "class_name"}
    td = lambda x: TensorDict({"policy": x, "critic": torch.zeros(x.shape[0], 16)}, batch_size=[x.shape[0]])  # noqa: E731
    m = RNNModel(td(torch.zeros(4, 41)), rc["obs_groups"], "actor", 3, **a)
    export_recurrent_jit(m, str(tmp_path / "policy.pt"))
    (tmp_path / "policy_meta.json").write_text(json.dumps({"config": cfg.to_dict(), "recurrent": recurrent_meta(m)}))
    pp = PolicyPlanner(str(tmp_path))
    g = torch.Generator().manual_seed(0)
    xs = [torch.randn(4, 41, generator=g) for _ in range(5)]
    m.eval()
    with torch.no_grad():
        ref = [m(td(x)) for x in xs]
        out = [pp.act_obs(x) for x in xs]
    assert all(torch.allclose(r, o, atol=1e-6) for r, o in zip(ref, out))
    pp.reset(torch.tensor([1]))
    assert torch.allclose(pp.act_obs(xs[0])[1], PolicyPlanner(str(tmp_path)).act_obs(xs[0])[1], atol=1e-6)


def test_constant_vio_offset_leaves_relative_estimates_unchanged():
    """VIO drift moves the ray origins AND the planner's own position: an offset that stays constant while
    the rays are taken changes nothing the planner sees; only drift accumulated since then matters."""
    ests = []
    for drift in (None, torch.tensor([0.3, -0.2, 0.1])):
        env = _env(*(("perception.pose_drift_m=1.0e-12",) if drift is not None else ()), n=8, level=3)
        env.core.perception.reset(torch.arange(8))           # drop the reset frame (taken before the offset)
        if drift is not None:
            env.core.drift[:] = drift
        for k in range(20):
            a = torch.zeros(8, env.num_actions)
            a[:, 0], a[:, 1] = 0.3, 0.6 if (k // 4) % 2 else -0.6       # lateral zig-zag for parallax
            env.step(a)
        ests.append(env.core.est)
    v = ests[0].valid & ests[1].valid
    assert bool(v.any()) and torch.equal(ests[0].valid, ests[1].valid)
    assert torch.allclose(ests[0].center[v], ests[1].center[v], atol=1e-4)


def test_safety_slowdown_ramps_the_step_and_leaves_seen_envs_untouched():
    """safety.slow_after_s: blind (no detection) with an unknown target -> the waypoint step shrinks linearly
    from slow_after_s to the hold (lost_hold_s); envs that see a window keep the planner's action bit-for-bit."""
    from rl_planner.planners.safety import SafetyFilter
    cfg = cfg_cpu("safety.slow_after_s=1.0", "safety.lost_hold_s=3.0", "safety.slow_min_scale=0.25", "safety.hold_brake_acc=null")
    f = SafetyFilter(cfg, 2, "cpu", 0.1)
    act = torch.tensor([[0.8, 0.3, -0.2], [0.8, 0.3, -0.2]])
    pos = torch.tensor([[1.0, 0.0, 1.2], [1.0, 0.0, 1.2]])
    yaw, sm = torch.zeros(2), torch.tensor([0.8, 0.6, 0.4])
    unknown = torch.zeros(2, dtype=torch.bool)
    f.filter(act, torch.tensor([True, True]), unknown, pos, yaw, sm)      # first detection
    scales = []
    for k in range(1, 33):                                                # hold starts after 3 s (> 30 steps)
        out = f.filter(act, torch.tensor([False, True]), unknown, pos, yaw, sm)
        assert torch.equal(out[1], act[1])                                # env 1 keeps seeing: untouched
        if not bool(f.holding[0]):
            scales.append(float(out[0, 0] / act[0, 0]))
    assert scales[:10] == [1.0] * 10                                      # first 1 s blind: no change
    assert all(b <= a + 1e-6 for a, b in zip(scales[10:], scales[11:]))   # then a monotone ramp ...
    assert min(scales) >= 0.25 - 1e-6 and scales[-1] < 0.6               # ... down to the floor before the hold
    assert bool(f.holding[0])


def test_safety_hold_brake_latches_the_stopping_point():
    """safety.hold_brake_acc: the hold latches the horizontal braking stop point pos + v|v|/(2a) (PX4 Hold) instead
    of the position where the hold began (null), so a hold that starts at speed is not pulled back."""
    from rl_planner.planners.safety import SafetyFilter
    pos, vel = torch.tensor([[1.0, 0.0, 1.2]]), torch.tensor([[1.2, -0.5, 0.3]])
    act, yaw, sm = torch.tensor([[0.8, 0.0, 0.0]]), torch.zeros(1), torch.tensor([0.8, 0.6, 0.4])
    unknown = torch.zeros(1, dtype=torch.bool)
    for extra, want in (("safety.hold_brake_acc=null",), pos[0]), (("safety.hold_brake_acc=2.0",), pos[0] + torch.tensor([1.2, -0.5, 0.0]) * 1.3 / 4.0):
        f = SafetyFilter(cfg_cpu("safety.lost_hold_s=0.3", *extra), 1, "cpu", 0.1)
        f.filter(act, torch.tensor([True]), unknown, pos, yaw, sm, vel=vel)       # first detection
        for _ in range(4):
            f.filter(act, torch.tensor([False]), unknown, pos, yaw, sm, vel=vel)
        assert bool(f.holding[0])
        assert torch.allclose(f.hold_pos[0], want, atol=1e-6)


def test_safety_hold_retraces_the_path_flown_since_the_last_pass():
    """safety.retreat_after_s: a blind hold backs up along the breadcrumbs of the path flown since the last window
    pass at retreat_speed, never closer than crumb_clear_m to the pass point and at most retreat_max_m; a detection
    ends it. Off (null) the hold stays where it latched."""
    from rl_planner.planners.safety import SafetyFilter
    act, yaw, sm = torch.tensor([[0.8, 0.0, 0.0]]), torch.zeros(1), torch.tensor([0.8, 0.6, 0.4])
    unknown, tgt = torch.zeros(1, dtype=torch.bool), torch.tensor([1])
    base = ("safety.lost_hold_s=0.3", "safety.hold_brake_acc=null", "safety.slow_after_s=null")
    for extra, max_m in ((("safety.retreat_after_s=null",), None), (("safety.retreat_after_s=0.5", "safety.retreat_speed=0.5"), 3.0),
                         (("safety.retreat_after_s=0.5", "safety.retreat_speed=0.5", "safety.retreat_max_m=0.8"), 0.8)):
        f = SafetyFilter(cfg_cpu(*base, *extra), 1, "cpu", 0.1)
        # window 0 passed at x = 0; fly to x = 2 along a dog-leg (y = 0 -> 0.5), seeing windows on the way
        path = [torch.tensor([[0.1 * k, 0.0 if k < 10 else 0.05 * (k - 10), 1.2]]) for k in range(21)]
        for p in path:
            f.filter(act, torch.tensor([True]), unknown, p, yaw, sm, target_idx=tgt)
        here = path[-1]
        trace = []
        for _ in range(80):                                   # blind: hold after 0.3 s, retreat after 0.5 s more
            f.filter(act, torch.tensor([False]), unknown, here, yaw, sm, target_idx=tgt)
            if bool(f.holding[0]):
                trace.append(f.hold_pos[0].clone())
        assert bool(f.holding[0]) and torch.equal(trace[0], here[0])
        if max_m is None:
            assert torch.equal(trace[-1], here[0])
            continue
        steps = [float((b - a).norm()) for a, b in zip(trace, trace[1:])]
        assert max(steps) <= 0.05 + 1e-5                       # 0.5 m/s at 10 Hz
        travelled = sum(steps)
        assert travelled <= max_m + 0.05 + 1e-5
        on_path = min(float((trace[-1] - p[0]).norm()) for p in path)
        assert on_path < 0.06                                  # it stays on the flown path ...
        assert float((trace[-1] - path[0][0]).norm()) >= 0.6 - 1e-5   # ... and short of the pass point
        if max_m == 3.0:                                       # the whole path back to the clearance was used
            assert abs(float(trace[-1][0]) - 0.6) < 0.06 and abs(float(trace[-1][1])) < 1e-6
        f.filter(act, torch.tensor([True]), unknown, here, yaw, sm, target_idx=tgt)   # a detection ends the hold
        assert not bool(f.holding[0])


def test_rescale_action_keeps_the_commanded_displacement():
    """scripts/rescale_action.py: with a larger action.step_max the rescaled actor commands the same waypoint
    displacement (mean action x step_max) and the same exploration noise in metres, given the previous action in
    the new units; Adam moments of the rescaled parameters follow (grad x 1/s)."""
    from rl_planner.scripts.expand_obs import layouts
    from rl_planner.scripts.rescale_action import rescale_checkpoint
    old = cfg_cpu().to_dict()
    step_old = torch.tensor([float(v) for v in old["action"]["step_max"]])
    step_new = torch.tensor([1.25, 0.9, 0.5])
    g = torch.Generator().manual_seed(1)
    a_lay, c_lay = layouts(old)
    D, H, A = a_lay[-1][2], 16, 3
    actor = {"obs_normalizer._mean": torch.randn(1, D, generator=g), "obs_normalizer._std": torch.rand(1, D, generator=g) + 0.3,
             "obs_normalizer._var": torch.ones(1, D), "obs_normalizer.count": torch.tensor(10),
             "distribution.std_param": torch.rand(A, generator=g) + 0.2,
             "mlp.0.weight": torch.randn(H, D, generator=g), "mlp.0.bias": torch.randn(H, generator=g),
             "mlp.2.weight": torch.randn(A, H, generator=g), "mlp.2.bias": torch.randn(A, generator=g)}
    Dc = c_lay[-1][2]
    critic = {"obs_normalizer._mean": torch.randn(1, Dc, generator=g), "obs_normalizer._std": torch.rand(1, Dc, generator=g) + 0.3,
              "obs_normalizer._var": torch.ones(1, Dc), "mlp.0.weight": torch.randn(H, Dc, generator=g)}
    names = ["distribution.std_param", "mlp.0.weight", "mlp.0.bias", "mlp.2.weight", "mlp.2.bias"]
    opt = {"state": {i: {"exp_avg": torch.randn(actor[n].shape, generator=g), "exp_avg_sq": torch.rand(actor[n].shape, generator=g)}
                     for i, n in enumerate(names)}, "param_groups": [{"params": list(range(len(names)))}]}
    ck = {"actor_state_dict": actor, "critic_state_dict": critic, "optimizer_state_dict": opt, "iter": 3}
    out = rescale_checkpoint(ck, old, step_new.tolist())

    def act(sd, x):
        z = (x - sd["obs_normalizer._mean"]) / (sd["obs_normalizer._std"] + 1e-2)
        return torch.nn.functional.elu(z @ sd["mlp.0.weight"].T + sd["mlp.0.bias"]) @ sd["mlp.2.weight"].T + sd["mlp.2.bias"]

    la = next((lo, hi) for n, lo, hi in a_lay if n == "last_action")
    x_old = torch.randn(5, D, generator=g)
    x_old[:, la[0]:la[1]] = torch.rand(5, A, generator=g) * 2 - 1           # previous action, old units
    x_new = x_old.clone()
    x_new[:, la[0]:la[1]] *= step_old / step_new                            # the same displacement in new units
    assert torch.allclose(act(ck["actor_state_dict"], x_old) * step_old, act(out["actor_state_dict"], x_new) * step_new, atol=1e-5)
    assert torch.allclose(actor["distribution.std_param"] * step_old, out["actor_state_dict"]["distribution.std_param"] * step_new)
    s = step_old / step_new
    st = out["optimizer_state_dict"]["state"]
    assert torch.allclose(st[3]["exp_avg"], opt["state"][3]["exp_avg"] / s[:, None])
    assert torch.allclose(st[4]["exp_avg_sq"], opt["state"][4]["exp_avg_sq"] / s ** 2)
    assert torch.equal(st[1]["exp_avg"], opt["state"][1]["exp_avg"])      # hidden layers untouched


def test_motion_blur_streaks_corners_along_the_image_motion():
    """perception.exposure_s: a yawing camera smears the corners horizontally by about fx * yaw_rate * exposure;
    the detector picks a random point of the streak (between the corner now and at the start of the exposure)
    and is less confident. With the knob off there is no motion term (and no extra draw)."""
    from rl_planner.envs.torch_env import TorchWindowEnv
    cfg = cfg_cpu("sim.num_envs=8", "perception.noise_scale=0.0", "perception.exposure_s=0.01")
    env = TorchWindowEnv(cfg, num_envs=8, device="cpu", seed=4)
    core = env.core
    core.fixed_level = 0
    env._reset_all()
    core.state.omega_b[:] = torch.tensor([0.0, 0.0, 2.0])             # yaw rate 2 rad/s, hovering
    core.state.vel_w[:] = 0.0
    cam_c, R_WC = core.perception.camera_pose(core.state.pos, core.state.quat)
    take = torch.ones(8, dtype=torch.bool)
    out = {}
    for name, motion in (("sharp", None), ("blur", core._motion())):
        _, _, uv, det, conf, _ = core.perception.capture(cam_c, R_WC, core.scene, take, torch.Generator().manual_seed(0),
                                                         motion=motion)
        out[name] = (uv[:, 0], det[:, 0], conf[:, 0])
    (uv0, det0, c0), (uv1, det1, c1) = out["sharp"], out["blur"]
    seen = det0 & det1
    assert bool(seen.any())
    d = (uv1 - uv0)[seen]                                            # (n,4,2) shift along the streak
    full = 763.0 * 2.0 * 0.01                                        # fx * yaw rate * exposure ~ 15 px
    assert float(d[..., 0].abs().max()) <= 1.3 * full and float(d[..., 0].abs().mean()) > 0.15 * full
    assert float(d[..., 1].abs().mean()) < 0.3 * float(d[..., 0].abs().mean())   # mostly horizontal
    assert bool((c1[seen] < c0[seen]).all())
    off = TorchWindowEnv(cfg_cpu("sim.num_envs=8"), num_envs=8, device="cpu", seed=4).core
    assert off._motion() is None


# ----------------------------------------------------------------------------- look-around courses (RESULTS §12)
COURSE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs", "scan", "course.yaml")


def test_scatter_course_orders_windows_by_the_nearest_exit_rule():
    """scene.layout scatter: from the start and from every window the next one is the nearest window not passed
    yet, every other one at least nearest_margin farther; centres keep min_separation and stay in the arena."""
    from rl_planner.core.scene import LevelTable, SceneTensors, sample_scene
    cfg = load_config([COURSE], ["device=cpu", "scene.max_windows=20", "curriculum.override={n_windows: [20, 20]}"])
    E, W = 48, 20
    sc = SceneTensors.empty(E, W, "cpu")
    sample_scene(cfg, LevelTable(cfg, "cpu"), torch.full((E,), 4, dtype=torch.long), torch.Generator().manual_seed(5),
                 sc, torch.arange(E))
    assert bool((sc.n_windows == W).all())
    for e in range(E):
        c = sc.center[e]
        refs = torch.cat((sc.start[e:e + 1], c[:-1]))
        for j in range(W - 1):
            d = (c[j:] - refs[j]).norm(dim=-1)
            assert float(d[1:].min()) >= 1.25 * float(d[0]) - 1e-4
        dd = (c[:, None] - c[None]).norm(dim=-1) + torch.eye(W) * 99
        assert float(dd.min()) >= 2.4 - 1e-4
    b = cfg["scene"]["bounds"]
    assert float(sc.center[..., 0].min()) >= b["x_min"] and float(sc.center[..., 1].abs().max()) <= b["y_abs"]
    # the course really turns: next windows far off the direction the drone leaves a window in
    legs = sc.center[:, 1:] - sc.center[:, :-1]
    ex = -sc.normal[:, :-1]
    cosang = (ex[..., :2] * legs[..., :2]).sum(-1) / legs[..., :2].norm(dim=-1)
    assert float((cosang < math.cos(math.radians(45))).float().mean()) > 0.3


def test_line_layout_and_order_selection_are_untouched_by_the_new_keys():
    """Default config: per-env time limit = the old fixed one, the selector mirrors the course order."""
    from rl_planner.envs.torch_env import TorchWindowEnv
    cfg = cfg_cpu()
    env = TorchWindowEnv(cfg, num_envs=8, device="cpu")
    core = env.core
    assert bool((core.ep_limit == core.max_episode_length).all())
    for _ in range(5):
        env.step(torch.zeros(8, env.num_actions))
    assert torch.equal(core.selector.sel, core.target_idx) and bool(core.selector.has.all())


def test_nearest_selection_uses_only_seen_windows_measured_from_the_last_pass():
    from rl_planner.core.selector import TargetSelector
    cfg = cfg_cpu("mission.selection=nearest")
    lay = [[{"center": (3.0, 0.0, 1.0), "ow": 1.0, "oh": 1.0}, {"center": (6.0, 3.0, 1.0), "ow": 1.0, "oh": 1.0},
            {"center": (7.0, -1.0, 1.0), "ow": 1.0, "oh": 1.0}]]
    sc = scene_from_layout(lay, 3, (0.0, 0.0, 1.0), "cpu")
    pr = PerceptionSurrogate(cfg, 1, 3, "cpu")
    mission = MissionTracker(cfg, 1, 3, "cpu")
    mission.reset(torch.arange(1), torch.tensor([[0.0, 0.0, 1.0]]), sc)
    sel = TargetSelector(cfg, 1, 3, "cpu")
    sel.reset(torch.arange(1), torch.tensor([[0.0, 0.0, 1.0]]), torch.zeros(1), torch.zeros(1))
    est = pr.estimate()
    sel.select(sc, mission, est, pr)
    assert not bool(sel.has[0])                                   # nothing seen yet: no target
    # windows 1 and 2 seen (monocular guesses from their true range); 0 not yet
    for w in (1, 2):
        cam = torch.tensor([0.0, 0.0, 1.0])
        d = sc.center[0, w] - cam
        pr.last_cam[0, w] = cam
        pr.last_bearing[0, w] = d / d.norm()
        pr.last_side_max[0, w] = 763.0 * 1.0 / float(d.norm())
        pr.det_age[0, w] = 0.0
    sel.select(sc, mission, est, pr)
    assert bool(sel.has[0]) and int(sel.sel[0]) == 1 and int(sel.nxt[0]) == 2   # |w1| 6.7 m < |w2| 7.1 m
    # after passing window 1 from (6, 3): window 2 is the nearest not passed
    mission.passed[0, 1] = True
    sel.on_pass(torch.tensor([True]), torch.tensor([[6.0, 3.0, 1.0]]), torch.tensor([[1.0, 0.0, 0.0]]), torch.zeros(1),
                torch.ones(1), torch.zeros(1, 3), torch.tensor([False]))
    sel.select(sc, mission, est, pr)
    assert int(sel.sel[0]) == 2 and not bool(sel.has_nxt[0])


def test_local_window_subset_detects_the_same_windows():
    """perception.local_windows K: the K windows nearest the camera get the same detections as the full model."""
    cfg_full = cfg_cpu("scene.wall_half_extent=0.8")
    cfg_k = cfg_cpu("scene.wall_half_extent=0.8", "perception.local_windows=3")
    lay = [[{"center": (2.5 + 2.5 * i, 0.4 * (i % 3) - 0.4, 1.0), "ow": 1.0, "oh": 1.0} for i in range(6)]]
    sc = scene_from_layout(lay, 6, (0.0, 0.0, 1.0), "cpu")
    outs = []
    for cfg in (cfg_full, cfg_k):
        pr = PerceptionSurrogate(cfg, 1, 6, "cpu")
        pr.reset(torch.arange(1), sigma_px=torch.zeros(1), miss_prob=torch.zeros(1))
        cam, R = pr.camera_pose(torch.tensor([[0.0, 0.0, 1.0]]), m3.quat_from_yaw(torch.zeros(1)))
        outs.append(pr.capture(cam, R, sc, torch.ones(1, dtype=torch.bool), torch.Generator().manual_seed(0))[3])
    full, sub = outs
    near3 = torch.zeros(6, dtype=torch.bool)
    near3[:3] = True
    assert bool(full[0, :3].any())
    assert torch.equal(sub[0, near3], full[0, near3]) and not bool(sub[0, ~near3].any())


def test_expand_obs_adds_a_zero_yaw_action():
    """scripts/expand_obs.py --yaw: the 4th output starts at zero, the first three are unchanged."""
    from rl_planner.scripts.expand_obs import expand_checkpoint, grow_actions, layouts
    old = cfg_cpu().to_dict()
    new = cfg_cpu("action.yaw=true").to_dict()
    g = torch.Generator().manual_seed(1)
    D, H = layouts(old)[0][-1][2], 16
    sd = {"obs_normalizer._mean": torch.randn(1, D, generator=g), "obs_normalizer._std": torch.rand(1, D, generator=g) + 0.5,
          "obs_normalizer._var": torch.rand(1, D, generator=g) + 0.5, "distribution.std_param": torch.ones(3),
          "mlp.0.weight": torch.randn(H, D, generator=g), "mlp.0.bias": torch.randn(H, generator=g),
          "mlp.2.weight": torch.randn(3, H, generator=g), "mlp.2.bias": torch.randn(3, generator=g)}
    Dc = layouts(old)[1][-1][2]
    crit = {"obs_normalizer._mean": torch.zeros(1, Dc), "obs_normalizer._std": torch.ones(1, Dc),
            "obs_normalizer._var": torch.ones(1, Dc), "mlp.0.weight": torch.randn(24, Dc, generator=g)}
    ck = {"actor_state_dict": sd, "critic_state_dict": crit, "iter": 1, "infos": None}
    out = grow_actions(expand_checkpoint(ck, old, new), 4, 0.5)
    lo, ln = layouts(old)[0], layouts(new)[0]
    x_old = torch.randn(5, D, generator=g)
    x_new = torch.randn(5, ln[-1][2], generator=g)
    src = {k: (s, e) for k, s, e in lo}
    for k, s, e in ln:
        a, b = src[k]
        x_new[:, s:s + b - a] = x_old[:, a:b]

    def f(sd, x):
        h = torch.nn.functional.elu(((x - sd["obs_normalizer._mean"]) / (sd["obs_normalizer._std"] + 1e-2)) @ sd["mlp.0.weight"].T
                                    + sd["mlp.0.bias"])
        return h @ sd["mlp.2.weight"].T + sd["mlp.2.bias"]
    y_old, y_new = f(sd, x_old), f(out["actor_state_dict"], x_new)
    assert torch.allclose(y_old, y_new[:, :3], atol=1e-5) and float(y_new[:, 3].abs().max()) == 0.0
    assert out["actor_state_dict"]["distribution.std_param"].tolist() == [1.0, 1.0, 1.0, 0.5]


def test_yaw_action_turns_the_drone_and_the_look_around_bookkeeping_follows():
    from rl_planner.envs.torch_env import TorchWindowEnv
    cfg = load_config([COURSE], ["device=cpu", "action.yaw=true", "action.yaw_rate_max_deg=90.0",
                                 "domain_randomization.enabled=false"])
    env = TorchWindowEnv(cfg, num_envs=4, device="cpu", seed=2)
    core = env.core
    core.fixed_level = 0
    env._reset_all()
    a = torch.zeros(4, 4)
    a[:, 3] = 1.0                                                 # full yaw rate left for 1.5 s
    for _ in range(15):
        env.step(a)
    turned = math.degrees(float(core.selector.psi[0]))
    assert 100.0 < turned < 140.0                                 # 135 deg commanded, controller lag
    assert abs(math.degrees(float(core.selector.psi_hi[0])) - turned) < 1e-3 and float(core.selector.psi_lo[0]) <= 0.0
    assert bool((core.ep_limit == core.base_steps + torch.ceil(core.scene.n_windows * 80.0 - 1e-6).long()).all())


def test_scan_supervisor_sweeps_finds_a_window_to_the_side_and_faces_it():
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.scan import ScanSupervisor

    class Hover:
        def reset(self, ids):
            pass

        def act(self, core):
            return torch.zeros(core.E, 3)
    cfg = load_config([COURSE], ["device=cpu", "domain_randomization.enabled=false"])
    env = TorchWindowEnv(cfg, num_envs=1, device="cpu", seed=0)
    core = env.core
    lay = [[{"center": (0.5, 3.0, 1.0), "ow": 1.0, "oh": 1.0, "normal": (0.0, -1.0, 0.0)}]]   # 80 deg to the left
    core.scene = scene_from_layout(lay, core.W, (0.0, 0.0, 1.0), "cpu")
    core.perception.reset(torch.arange(1))
    core.after_reset(torch.arange(1))
    sup = ScanSupervisor(Hover(), cfg, 1, "cpu", core.step_dt)
    sup.reset(torch.arange(1))
    phases = []
    for _ in range(80):
        env.step(sup.act(core))
        phases.append(int(sup.phase[0]))
    assert bool(core.selector.has[0]) and int(core.selector.sel[0]) == 0
    assert 3 in phases                                            # swept, chose it, turned to it
    look = math.atan2(3.0 - float(core.state.pos[0, 1]), 0.5 - float(core.state.pos[0, 0]))
    assert abs(math.degrees(math.atan2(math.sin(look - float(core.yaw[0])), math.cos(look - float(core.yaw[0]))))) < 20.0
