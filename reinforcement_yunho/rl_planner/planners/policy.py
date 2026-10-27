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


class RecurrentExport(torch.nn.Module):
    """Export wrapper for an rsl_rl RNNModel actor (GRU or LSTM): ``(obs (B,D), state (S,B,H)) -> (action, state)``.
    LSTM state = hidden and cell stacked along S."""
    __constants__ = ["lstm", "layers"]

    def __init__(self, model):
        super().__init__()
        import copy
        self.obs_normalizer = copy.deepcopy(model.obs_normalizer).cpu()
        self.rnn = copy.deepcopy(model.rnn.rnn).cpu()
        self.mlp = copy.deepcopy(model.mlp).cpu()
        self.out = model.distribution.as_deterministic_output_module().cpu()
        self.lstm = isinstance(self.rnn, torch.nn.LSTM)
        self.layers = int(self.rnn.num_layers)

    def forward(self, obs: torch.Tensor, state: torch.Tensor):
        x = self.obs_normalizer(obs).unsqueeze(0)
        if self.lstm:
            y, hc = self.rnn(x, (state[: self.layers].contiguous(), state[self.layers:].contiguous()))
            new = torch.cat((hc[0], hc[1]), 0)
        else:
            y, new = self.rnn(x, state.contiguous())
        return self.out(self.mlp(y.squeeze(0))), new


class PolicyPlanner:
    """MLP policies map obs -> action. Recurrent ones (policy_meta.json 'recurrent') map (obs, state) ->
    (action, state); the planner keeps one state per env, zeroed by ``reset(env_ids)`` at episode starts."""
    name = "rl"

    def __init__(self, export_dir: str, device="cpu", cfg: Mapping | None = None):
        self.device = torch.device(device)
        self.policy = torch.jit.load(os.path.join(export_dir, JIT_NAME), map_location=self.device)
        self.policy.eval()
        with open(os.path.join(export_dir, META_NAME), encoding="utf-8") as f:
            self.meta = json.load(f)
        rec = self.meta.get("recurrent")
        self.state_shape = tuple(rec["state_shape"]) if rec else None
        self.state = None                         # (S, num_envs, H), allocated on first use
        if cfg is not None:
            self.check_compatible(cfg)

    def check_compatible(self, cfg: Mapping) -> None:
        trained = self.meta["config"]
        for name, a, b in (("action", trained["action"], cfg["action"]),
                           ("observation.policy", trained["observation"]["policy"], cfg["observation"]["policy"])):
            if json.dumps(a, sort_keys=True) != json.dumps(b, sort_keys=True):
                raise ValueError(f"config.{name} differs from the one the policy was trained with — "
                                 f"run with the policy's config (policy_meta.json['config']).")
        for k in ("physics_dt", "decimation"):
            if trained["sim"][k] != cfg["sim"][k]:
                raise ValueError(f"sim.{k}: policy trained with {trained['sim'][k]}, env uses {cfg['sim'][k]}")

    def reset(self, env_ids) -> None:
        if self.state is not None:
            self.state[:, env_ids] = 0.0

    @torch.no_grad()
    def act(self, core) -> torch.Tensor:
        obs = core.compute_observations()["policy"].to(self.device)
        return self.act_obs(obs).to(core.device)

    @torch.no_grad()
    def act_obs(self, obs_policy: torch.Tensor) -> torch.Tensor:
        obs = obs_policy.to(self.device)
        if self.state_shape is None:
            return self.policy(obs)
        if self.state is None or self.state.shape[1] != obs.shape[0]:
            S, H = self.state_shape
            self.state = torch.zeros(S, obs.shape[0], H, device=self.device)
        act, self.state = self.policy(obs, self.state)
        return act
