"""Pure-torch backend: WindowMissionCore + TorchQuadrotor, exposed as an rsl_rl VecEnv.

Runs anywhere torch runs (no Isaac Sim): unit tests, laptops, CI, quick sweeps.
The physics is a 6-DOF rigid body with the same wrench the Isaac backend hands to
PhysX, so a policy trained here transfers to the Isaac backend unchanged (and
vice versa) — scripts/evaluate.py measures exactly that.
"""
from __future__ import annotations

from typing import Mapping

import torch

from ..core.dynamics import TorchQuadrotor
from ..core.env_core import WindowMissionCore

try:  # rsl_rl is optional for the env itself (needed only for training)
    from rsl_rl.env import VecEnv as _VecEnvBase
except Exception:  # pragma: no cover
    _VecEnvBase = object

try:
    from tensordict import TensorDict
except Exception:  # pragma: no cover
    TensorDict = None


class TorchWindowEnv(_VecEnvBase):
    def __init__(self, cfg: Mapping, num_envs: int | None = None, device: str | None = None,
                 seed: int | None = None):
        self.cfg = cfg
        self.num_envs = int(num_envs or cfg["sim"]["num_envs"])
        dev = device or cfg["device"]
        if dev.startswith("cuda") and not torch.cuda.is_available():
            dev = "cpu"
        self.device = torch.device(dev)
        self.core = WindowMissionCore(cfg, self.num_envs, self.device, seed=int(cfg["seed"] if seed is None else seed))
        self.body = TorchQuadrotor(self.num_envs, self.device, gravity=float(cfg["sim"]["gravity"]),
                                   substeps=2)
        self.core.state = self.body.state           # the core reads the integrator state directly
        self.num_actions = self.core.action_dim
        self.max_episode_length = self.core.max_episode_length
        self.extras: dict = {}
        self._reset_all()

    # rsl_rl writes a random initial length into this attribute (init_at_random_ep_len)
    @property
    def episode_length_buf(self) -> torch.Tensor:
        return self.core.episode_step

    @episode_length_buf.setter
    def episode_length_buf(self, value: torch.Tensor) -> None:
        self.core.episode_step = value.to(self.device).long()

    @property
    def num_obs(self) -> int:
        return self.core.num_policy_obs

    def _reset(self, env_ids: torch.Tensor) -> None:
        start, quat = self.core.reset_envs(env_ids)
        if start is None:
            return
        self.body.reset(env_ids, start, quat)
        self.core.episode_step[env_ids] = 0
        self.core.after_reset(env_ids)

    def _reset_all(self) -> None:
        self._reset(torch.arange(self.num_envs, device=self.device))

    def _obs(self):
        o = self.core.compute_observations()
        if TensorDict is None:
            return o
        return TensorDict(o, batch_size=[self.num_envs], device=self.device)

    def get_observations(self):
        return self._obs()

    @torch.no_grad()
    def step(self, actions: torch.Tensor):
        core = self.core
        core.pre_decision(actions.to(self.device))
        for _ in range(core.decimation):
            f, tau, f_ext = core.control()
            self.body.step(f, tau, f_ext, core.dt, core.body_mass, core.inertia)
            core.after_substep()
        core.episode_step += 1
        core.post_decision()
        terminated, time_out = core.compute_dones()
        reward = core.compute_rewards()
        dones = terminated | time_out
        self.extras = {"time_outs": time_out.clone()}
        ids = dones.nonzero(as_tuple=False).squeeze(-1)
        if ids.numel() > 0:
            self._reset(ids)
            self.extras["log"] = dict(core.log)      # "Metrics/..." keys are logged as-is by rsl_rl
        return self._obs(), reward, dones.long(), self.extras

    def close(self) -> None:
        pass
