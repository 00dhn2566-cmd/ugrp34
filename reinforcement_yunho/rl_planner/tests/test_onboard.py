"""Integration test: OnboardPlanner (real-message adapter) reproduces the training
observations and actions exactly, in closed loop on the torch backend.

Needs an exported policy: set RL_PLANNER_EXPORT=<run>/export (skipped otherwise). The slung-load test also takes
RL_PLANNER_SLUNG_EXPORT=<a policy trained with the load>/export.
"""
import os
import sys

import pytest
import torch

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

EXPORT = os.environ.get("RL_PLANNER_EXPORT")
SLUNG_EXPORT = os.environ.get("RL_PLANNER_SLUNG_EXPORT")
SLUNG_BASE = os.path.join(ROOT, "rl_planner", "configs", "slung", "base.yaml")


@pytest.mark.skipif(not EXPORT, reason="set RL_PLANNER_EXPORT to an export dir")
def test_onboard_matches_training_obs_and_actions():
    from rl_planner.core import math3d as m3
    from rl_planner.core.config import Cfg
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.onboard import OnboardPlanner
    from rl_planner.planners.policy import PolicyPlanner

    pol = PolicyPlanner(EXPORT)
    cfg = Cfg(pol.meta["config"])
    cfg["device"] = "cpu"
    cfg["domain_randomization"]["enabled"] = False
    for k in cfg["observation"]["noise"]:
        cfg["observation"]["noise"][k] = 0.0
    env = TorchWindowEnv(cfg, num_envs=1, device="cpu", seed=11)
    env.core.fixed_level = 3
    env._reset_all()
    core = env.core
    nearest = cfg.get("mission", {}).get("selection", "order") == "nearest"
    # nearest: the referee reports the passes (the planner's own crossing check uses the estimated plane, which a slow
    # drone can cross a few decisions before the true one — that would restart the look-around earlier than the env)
    ob = OnboardPlanner(EXPORT, n_windows=int(core.scene.n_windows[0]), device="cpu", auto_pass=not nearest)

    def vision_msg():
        pr = core.perception
        wins = []
        for k in range(core.W):
            if bool(pr.last_det[0, k]):
                wins.append({"order_index": k, "corners": pr.last_corners_px[0, k].tolist(),
                             "corner_vis": [1, 1, 1, 1], "det_conf": float(pr.last_conf[0, k])})
        return {"windows": wins}

    def drone_state():
        s = core.state
        q = s.quat[0]
        v_b = m3.quat_rotate_inverse(s.quat, s.vel_w)[0]
        return {"position": s.pos[0].tolist(), "orientation": [float(q[1]), float(q[2]), float(q[3]), float(q[0])],
                "lin_vel": v_b.tolist(), "ang_vel": s.omega_b[0].tolist()}

    max_diff = 0.0
    holds = 0
    term_diff = {}
    yaw_diff = 0.0
    for t in range(150):
        ds = drone_state()
        ob.add_frame(ds["position"], ds["orientation"], vision_msg())
        if nearest:                                           # report the referee's passes as they happen
            while int(ob.mission.target[0]) < int(core.mission.target[0]):
                ob.mark_passed()
        else:
            ob.mission.target[:] = core.mission.target        # same pass count as the referee
        out = ob.step(ds, t_s=float(core.episode_time[0]))
        o_env = core.compute_observations()["policy"][0]
        o_ob = ob.last_obs[0]
        for name, a0, a1 in ob.policy_spec:
            term_diff[name] = max(term_diff.get(name, 0.0), float((o_env[a0:a1] - o_ob[a0:a1]).abs().max()))
        a_env = pol.act(core).clamp(-1.0, 1.0)
        max_diff = max(max_diff, float((torch.tensor(out["policy_action"]) - a_env[0]).abs().max()))
        holds += int(out["hold"])
        ob.validate_waypoints_config(out["waypoints_config"])
        _, _, done, _ = env.step(torch.tensor([out["action"]]))
        if bool(done[0]):
            break
        yaw_diff = max(yaw_diff, abs(out["yaw"] - float(core.yaw_cmd[0])))   # heading set-point (yaw policies)
    print("per-term max |obs_env - obs_onboard|:", {k: round(v, 6) for k, v in term_diff.items()})
    assert max(term_diff.values()) < 1e-3, f"observation mismatch: {term_diff}"
    assert max_diff < 1e-3, f"onboard actions differ from training-env actions by {max_diff}"
    assert holds == 0, f"safety hold triggered {holds} times on a nominal mission"
    assert yaw_diff < 1e-3, f"onboard heading set-point differs from the env's by {yaw_diff} rad"


@pytest.mark.skipif(not EXPORT, reason="set RL_PLANNER_EXPORT to an export dir")
def test_onboard_counts_its_own_pass_only_through_the_opening():
    """auto_pass: crossing the estimated plane of the chosen window counts as a pass only inside the estimated
    opening (+ PASS_TOL) — flying past the side of the window (around its panel) is not a pass."""
    from rl_planner.core.perception import Estimates
    from rl_planner.planners.onboard import OnboardPlanner

    def run(y):
        ob = OnboardPlanner(EXPORT, n_windows=2, device="cpu")
        W = ob.W
        z = lambda *s: torch.zeros(1, W, *s)  # noqa: E731
        center, normal, size = z(3), z(3), torch.ones(1, W, 2)
        center[0, 0] = torch.tensor([3.0, 0.0, 1.0])
        normal[0, 0] = torch.tensor([-1.0, 0.0, 0.0])                 # approach side: x < 3
        valid = torch.zeros(1, W, dtype=torch.bool)
        valid[0, 0] = True
        ob.est = Estimates(valid=valid, corners=z(4, 3), center=center, normal=normal, size=size,
                           parallax_deg=z(), resid_rms=z(), sigma=z(), n_obs=z())
        ob.selector.sel[:] = 0
        ob.selector.has[:] = True
        for x in (2.6, 2.9, 3.2):
            ob.state.pos = torch.tensor([[x, y, 1.0]])
            ob._auto_pass()
        return int(ob.mission.target[0])

    assert run(0.3) == 1                                              # through the opening
    assert run(1.2) == 0                                              # beside it, across the plane


@pytest.mark.skipif(not (SLUNG_EXPORT or EXPORT), reason="set RL_PLANNER_SLUNG_EXPORT or RL_PLANNER_EXPORT")
def test_onboard_slung_layer_matches_env():
    """Slung load: the cable sensor in drone_state gives the env's observations (the slung term of a policy trained
    with the load), and the onboard convex layer behind the safety filter gives the action the env projects itself
    (slung.convex_layer.in_env) — closed loop, torch backend. A policy trained without the load (RL_PLANNER_EXPORT)
    flies the rig of configs/slung/base.yaml."""
    from rl_planner.core import math3d as m3
    from rl_planner.core.config import config_for_policy
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.onboard import OnboardPlanner
    from rl_planner.planners.policy import PolicyPlanner

    export = SLUNG_EXPORT or EXPORT
    pol = PolicyPlanner(export)
    cfg = config_for_policy(pol.meta["config"], extra=[] if SLUNG_EXPORT else [SLUNG_BASE])
    cfg["device"] = "cpu"
    cfg["domain_randomization"]["enabled"] = False
    for k in cfg["observation"]["noise"]:
        cfg["observation"]["noise"][k] = 0.0
    cfg["slung"]["convex_layer"]["in_env"] = True            # the env projects the action it gets: the reference
    env = TorchWindowEnv(cfg, num_envs=1, device="cpu", seed=11)
    env.core.fixed_level = 3
    env._reset_all()
    core = env.core
    nearest = cfg.get("mission", {}).get("selection", "order") == "nearest"
    ob = OnboardPlanner(export, n_windows=int(core.scene.n_windows[0]), device="cpu", auto_pass=not nearest,
                        slung=None if SLUNG_EXPORT else cfg["slung"])
    assert ob.layer is not None

    def vision_msg():
        pr = core.perception
        return {"windows": [{"order_index": k, "corners": pr.last_corners_px[0, k].tolist(), "corner_vis": [1, 1, 1, 1],
                             "det_conf": float(pr.last_conf[0, k])} for k in range(core.W) if bool(pr.last_det[0, k])]}

    def drone_state():
        s, sl = core.state, core.slung
        q = s.quat[0]
        hook, _ = sl.hook(s.pos, s.quat)
        body = lambda v: m3.quat_rotate_inverse(s.quat, v)[0].tolist()  # noqa: E731
        return {"position": s.pos[0].tolist(), "orientation": [float(q[1]), float(q[2]), float(q[3]), float(q[0])],
                "lin_vel": body(s.vel_w), "ang_vel": s.omega_b[0].tolist(),
                "load": {"offset": body(sl.pos - hook), "vel": body(sl.vel - s.vel_w)}}

    term_diff, raw_diff, layer_diff, changed, steps = {}, 0.0, 0.0, 0, 0
    for t in range(200):
        ds = drone_state()
        ob.add_frame(ds["position"], ds["orientation"], vision_msg())
        if nearest:
            while int(ob.mission.target[0]) < int(core.mission.target[0]):
                ob.mark_passed()
        else:
            ob.mission.target[:] = core.mission.target
        out = ob.step(ds, t_s=float(core.episode_time[0]))
        o_env = core.compute_observations()["policy"][0]
        for name, a0, a1 in ob.policy_spec:
            term_diff[name] = max(term_diff.get(name, 0.0), float((o_env[a0:a1] - ob.last_obs[0, a0:a1]).abs().max()))
        raw_diff = max(raw_diff, float((torch.tensor(out["policy_action"]) - pol.act(core).clamp(-1.0, 1.0)[0]).abs().max()))
        _, _, done, _ = env.step(torch.tensor([out["pre_layer_action"]]))
        if bool(done[0]):                                     # reset: the env's action is gone
            break
        layer_diff = max(layer_diff, float((core.actions[0] - torch.tensor(out["action"])).abs().max()))
        changed += int(out["layer_changed"])
        steps += 1
    print("per-term max |obs_env - obs_onboard|:", {k: round(v, 6) for k, v in term_diff.items()},
          f"raw {raw_diff:.2e} layer {layer_diff:.2e}, layer changed {changed}/{steps}")
    assert max(term_diff.values()) < 1e-3, f"observation mismatch: {term_diff}"
    assert raw_diff < 1e-3, f"onboard policy actions differ from the env's by {raw_diff}"
    assert changed > 0, "the layer never acted: nothing compared"
    assert layer_diff < 1e-3, f"onboard layer output differs from the env's projection by {layer_diff}"
