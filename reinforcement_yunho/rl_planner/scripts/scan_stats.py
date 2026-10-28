"""How a planner looks around on look-around courses (RESULTS.md §12): per window, was the next window in view right
after the pass, how long until it was first detected, how far the drone had turned and climbed / descended by then,
and how long the whole leg took.

    python rl_planner/scripts/scan_stats.py --policy <run>/export --config rl_planner/configs/scan/long50.yaml \
        --level 4 [--planner scan|fsmscan] [--safe] [--num-scenes 128]

Same seeded scenes as benchmark.py (validation seed 4242, CPU). A leg starts at a pass (or the start) and ends at the
next pass. "In view" = the next window was detected within 0.3 s of the pass. For the legs where it was not, the
look-around is summarised at the first detection: the largest heading change since the pass (|turn|) and the largest
altitude change (|dz|); the legs found only after more than 15 cm of altitude change count as found up/down.
Motion: mean |yaw rate|, share of flight time turning faster than 40 deg/s, mean |vertical speed| — how much the
drone swings its head and bobs to look around — overall and split into searching (the next window not seen yet in this
leg) and approaching (seen): an appropriate look-around turns while searching and flies steady once the window is found.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

COURSE = os.path.join(os.path.dirname(HERE), "configs", "scan", "course.yaml")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", default=None)
    ap.add_argument("--planner", default="rl", help="rl | scan | fsmscan")
    ap.add_argument("--config", action="append", default=[])
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--level", type=int, default=4)
    ap.add_argument("--num-scenes", type=int, default=128)
    ap.add_argument("--seed", type=int, default=4242)
    ap.add_argument("--safe", action="store_true")
    ap.add_argument("--threads", type=int, default=4)
    a = ap.parse_args(argv)
    import torch

    from rl_planner.core.config import config_for_policy, load_config
    from rl_planner.core.mission import CRASH_NAMES
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.policy import PolicyPlanner
    from rl_planner.planners.safety import SafePlanner
    torch.set_num_threads(a.threads)
    extra = [COURSE] + a.config
    if a.policy:
        with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
            cfg = config_for_policy(json.load(f)["config"], extra, ["device=cpu"] + a.set)
    else:
        cfg = load_config(extra, ["device=cpu"] + a.set)
    E = a.num_scenes
    env = TorchWindowEnv(cfg, num_envs=E, device="cpu", seed=a.seed)
    core = env.core
    core.fixed_level = a.level
    core.reseed(a.seed)
    env._reset_all()
    if a.planner == "fsmscan":
        from rl_planner.planners.fsm import FSMPlanner
        from rl_planner.planners.scan import ScanSupervisor
        planner = ScanSupervisor(FSMPlanner(cfg, E, "cpu", follow_selector=True), cfg, E, "cpu", core.step_dt)
    else:
        planner = PolicyPlanner(a.policy, device="cpu", cfg=cfg)
        if a.planner == "scan":
            from rl_planner.planners.scan import ScanSupervisor
            planner = ScanSupervisor(planner, cfg, E, "cpu", core.step_dt)
    if a.safe:
        planner = SafePlanner(planner, cfg, E, "cpu", core.step_dt)
    planner.reset(torch.arange(E))

    active = torch.ones(E, dtype=torch.bool)
    leg_t0 = torch.zeros(E)
    found_t = torch.full((E,), -1.0)
    found_turn = torch.zeros(E)
    found_dz = torch.zeros(E)
    legs = []                                   # (in view, search s, |turn| deg, |dz| m, leg s, found)
    end = {"success": 0, "crash": {}, "timeout": 0, "phase_crash": [0, 0]}
    motion = {k: [0.0, 0.0] for k in ("yaw_rate", "fast", "vz", "n")}   # [searching, approaching]
    prev_yaw = core.yaw.clone()
    post = core.post_decision

    def hooked():
        post()
        t = core.episode_time
        rate = torch.atan2(torch.sin(core.yaw - prev_yaw), torch.cos(core.yaw - prev_yaw)).abs() / core.step_dt
        prev_yaw.copy_(core.yaw)
        for i, m in enumerate((active & (found_t < 0), active & (found_t >= 0))):
            motion["yaw_rate"][i] += float(rate[m].sum())
            motion["fast"][i] += float((rate[m] > math.radians(40.0)).float().sum())
            motion["vz"][i] += float(core.state.vel_w[m, 2].abs().sum())
            motion["n"][i] += int(m.sum())
        sel = core.selector
        tgt = core.mission.target.clamp_max(core.W - 1)
        det_now = core.det.detected.gather(1, tgt[:, None]).squeeze(1)
        newly = active & (found_t < 0) & det_now
        found_t[newly] = (t - leg_t0)[newly]
        found_turn[newly] = torch.maximum(sel.psi_hi.abs(), sel.psi_lo.abs())[newly]
        found_dz[newly] = torch.maximum(sel.dz_hi.abs(), sel.dz_lo.abs())[newly]
        passed = core.mission.step_pass.any(-1) & active
        for e in passed.nonzero(as_tuple=False).squeeze(-1).tolist():
            ft = float(found_t[e])
            legs.append((0.0 <= ft <= 0.3, ft, math.degrees(float(found_turn[e])), float(found_dz[e]),
                         float(t[e] - leg_t0[e]), ft >= 0))
        leg_t0[passed] = t[passed]
        found_t[passed] = -1.0
        found_turn[passed] = 0.0
        found_dz[passed] = 0.0
    core.post_decision = hooked
    for _ in range(core.max_episode_length + 5):
        _, _, d, _ = env.step(planner.act(core))
        d = d.bool() & active
        if d.any():
            r = core.episode_record
            for e in d.nonzero(as_tuple=False).squeeze(-1).tolist():
                if bool(r["success"][e]):
                    end["success"] += 1
                elif bool(r["crashed"][e]):
                    nm = CRASH_NAMES[int(r["crash_reason"][e])]
                    end["crash"][nm] = end["crash"].get(nm, 0) + 1
                    end["phase_crash"][0 if float(found_t[e]) < 0 else 1] += 1
                else:
                    end["timeout"] += 1
            active &= ~d
        if not bool(active.any()):
            break
    import numpy as np
    L = np.array(legs) if legs else np.zeros((0, 6))
    print(f"{a.policy or a.planner} {a.planner}{'+safe' if a.safe else ''} L{a.level} {' '.join(a.config)}: "
          f"{E} scenes, success {end['success']}, crash {end['crash']} (before / after the next window was seen: "
          f"{end['phase_crash'][0]} / {end['phase_crash'][1]}), timeout {end['timeout']}")
    def mot(i):
        m = max(1, sum(motion["n"][j] for j in i))
        tot = lambda k: sum(motion[k][j] for j in i)  # noqa: E731
        return (f"|yaw rate| mean {math.degrees(tot('yaw_rate') / m):.0f} deg/s, turning > 40 deg/s "
                f"{tot('fast') / m * 100:.0f}% of the time, |vertical speed| mean {tot('vz') / m:.2f} m/s")
    share = motion["n"][0] / max(1, sum(motion["n"])) * 100
    print(f"  motion: {mot((0, 1))}")
    print(f"    searching ({share:.0f}% of the time): {mot((0,))}")
    print(f"    approaching: {mot((1,))}")
    if len(L):
        inview = L[:, 0] > 0
        scan = ~inview & (L[:, 5] > 0)
        print(f"  legs passed {len(L)}: next window in view right after the pass {inview.mean() * 100:.0f}%, "
              f"leg time median {np.median(L[:, 4]):.1f} s (in view {np.median(L[inview, 4]) if inview.any() else float('nan'):.1f} s, "
              f"looked around {np.median(L[~inview, 4]) if (~inview).any() else float('nan'):.1f} s)")
        if scan.any():
            s = L[scan]
            q = lambda v, p: float(np.percentile(v, p))  # noqa: E731
            print(f"  looked around ({scan.sum()} legs): found after {q(s[:, 1], 50):.1f} s (p90 {q(s[:, 1], 90):.1f}), "
                  f"turned {q(s[:, 2], 50):.0f} deg (p90 {q(s[:, 2], 90):.0f}, >90: {(s[:, 2] > 90).mean() * 100:.0f}%), "
                  f"altitude change {q(s[:, 3], 50):.2f} m (p90 {q(s[:, 3], 90):.2f}); found up/down (>0.15 m): "
                  f"{(s[:, 3] > 0.15).mean() * 100:.0f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
