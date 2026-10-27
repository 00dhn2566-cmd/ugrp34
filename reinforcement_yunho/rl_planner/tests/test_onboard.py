"""Integration test: OnboardPlanner (real-message adapter) reproduces the training
observations and actions exactly, in closed loop on the torch backend.

Needs an exported policy: set RL_PLANNER_EXPORT=<run>/export (skipped otherwise).
"""
import os
import sys

import pytest
import torch

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

EXPORT = os.environ.get("RL_PLANNER_EXPORT")


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
    ob = OnboardPlanner(EXPORT, n_windows=int(core.scene.n_windows[0]), device="cpu")

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
    nearest = cfg.get("mission", {}).get("selection", "order") == "nearest"
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
