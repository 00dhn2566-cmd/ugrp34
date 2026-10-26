"""Trained-policy planner: TorchScript actor (obs normaliser + MLP + mean action)
plus ``policy_meta.json`` that pins the observation layout, action scaling and
timing the policy was trained with (so a policy can never be run with the wrong
``step_max`` again — the 0.3 vs 0.6 incident in prototype_demo/utils/scene.py).
"""
from __future__ import annotations

import json
import os
from typing import Mapping

import torch

META_NAME = "policy_meta.json"
JIT_NAME = "policy.pt"


def build_meta(cfg: Mapping, core, extra: Mapping | None = None) -> dict:
    meta = {
        "format": "rl_planner/policy/v1",
        "observation": core.observation_layout(),
        "num_policy_obs": core.num_policy_obs,
        "action": {
            "dim": core.action_dim,
            "type": cfg["action"]["type"],
            "frame": "heading (yaw-only, gravity aligned); +x forward, +y left, +z up",
            "step_max_m": list(cfg["action"]["step_max"]),
            "yaw": bool(cfg["action"]["yaw"]),
            "meaning": "waypoint = position + heading_to_world(clip(a,-1,1) * step_max)",
        },
        "timing": {"decision_dt_s": core.step_dt, "physics_dt_s": core.dt, "decimation": core.decimation},
        "camera": dict(cfg["perception"]["camera"]),
        "config": json.loads(json.dumps(cfg)),
    }
    if extra:
        meta.update(extra)
    return meta


class PolicyPlanner:
    name = "rl"

    def __init__(self, export_dir: str, device="cpu", cfg: Mapping | None = None):
        self.device = torch.device(device)
        self.policy = torch.jit.load(os.path.join(export_dir, JIT_NAME), map_location=self.device)
        self.policy.eval()
        with open(os.path.join(export_dir, META_NAME), encoding="utf-8") as f:
            self.meta = json.load(f)
        if cfg is not None:
            self.check_compatible(cfg)

    def check_compatible(self, cfg: Mapping) -> None:
        trained = self.meta["config"]
        for key in ("action", "observation"):
            if json.dumps(trained[key], sort_keys=True) != json.dumps(cfg[key], sort_keys=True):
                raise ValueError(f"config.{key} differs from the one the policy was trained with — "
                                 f"run with the policy's config (policy_meta.json['config']).")
        for k in ("physics_dt", "decimation"):
            if trained["sim"][k] != cfg["sim"][k]:
                raise ValueError(f"sim.{k}: policy trained with {trained['sim'][k]}, env uses {cfg['sim'][k]}")

    def reset(self, env_ids) -> None:
        pass

    @torch.no_grad()
    def act(self, core) -> torch.Tensor:
        obs = core.compute_observations()["policy"].to(self.device)
        return self.policy(obs).to(core.device)

    @torch.no_grad()
    def act_obs(self, obs_policy: torch.Tensor) -> torch.Tensor:
        return self.policy(obs_policy.to(self.device))
