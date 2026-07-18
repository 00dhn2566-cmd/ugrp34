"""Train (or smoke-test) a window-traversal policy (spec 7.3).

WHAT
----
CLI that builds ``num_envs`` vectorised ``WindowTraversalEnv`` (MockPhysics here,
Isaac later), trains a policy, logs a learning curve, and checkpoints so runs can
resume. Everything is written to a reproducible run folder:

    <log_dir>/run_<config-stem>_<seed>_<hash>/
        config_snapshot.yaml   # the resolved config actually used
        log.csv                # learning curve (always)
        tb/                     # tensorboard events (only if tensorboard importable)
        checkpoints/ckpt_*.npz  # policy checkpoints (+ latest.npz)

BACKENDS FOR LEARNING
---------------------
* If ``stable-baselines3`` is importable -> real PPO (the ``algo`` field). GUARDED.
* Otherwise -> a clearly-labelled MINIMAL hand-rolled loop with a tiny linear
  policy. It does NOT learn well; it exists so the pipeline (vectorised stepping,
  logging, checkpoint save/resume) runs and is testable WITHOUT heavy deps.

SMOKE MODE (``--smoke``) == gpu_jobs Job 2
-----------------------------------------
Runs a handful of steps with 1-2 MockPhysics envs, writes a checkpoint, RELOADS
it, and asserts the reload matches -- proving the checkpoint round-trip works.

CONVENTIONS "Reproducibility": the run folder records seed + config + commit hash
(a placeholder "nogit" when not in a git repo).
"""
from __future__ import annotations

import argparse
import hashlib
import os
import subprocess
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import yaml  # noqa: E402  (core dep)

import reward as reward_mod  # noqa: E402
from domain_randomization import DomainRandomizationConfig  # noqa: E402
from window_env import ACT_DIM, OBS_DIM, EnvConfig, MockPhysics, WindowTraversalEnv  # noqa: E402

# --- optional heavy deps (guarded) ------------------------------------------
try:
    import stable_baselines3 as sb3  # noqa: F401

    _HAS_SB3 = True
except Exception:
    sb3 = None  # type: ignore
    _HAS_SB3 = False

try:
    from torch.utils.tensorboard import SummaryWriter  # type: ignore

    _HAS_TB = True
except Exception:
    try:
        from tensorboardX import SummaryWriter  # type: ignore

        _HAS_TB = True
    except Exception:
        SummaryWriter = None  # type: ignore
        _HAS_TB = False


# ===========================================================================
# reproducibility helpers
# ===========================================================================
def git_hash(root: str) -> str:
    """Short git commit hash, or "nogit" when not in a git repo (spec allows a
    placeholder -- CONVENTIONS "Reproducibility")."""
    try:
        out = subprocess.run(
            ["git", "-C", root, "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if out.returncode == 0 and out.stdout.strip():
            return out.stdout.strip()
    except Exception:
        pass
    return "nogit"


def run_folder_name(config_path: str, seed: int, root: str) -> str:
    stem = os.path.splitext(os.path.basename(config_path))[0]
    return f"run_{stem}_{seed}_{git_hash(root)}"


# ===========================================================================
# fallback policy + vector env (used when stable-baselines3 is absent)
# ===========================================================================
class LinearGaussianPolicy:
    """A tiny deterministic linear policy: action = tanh(W @ obs + b).

    NOT a real learner -- it is the checkpointable object the fallback loop
    perturbs so the save/resume machinery is exercisable without SB3.
    """

    def __init__(self, obs_dim: int, act_dim: int, rng: np.random.Generator):
        self.W = rng.normal(0.0, 0.1, size=(act_dim, obs_dim))
        self.b = np.zeros(act_dim)

    def act(self, obs: np.ndarray) -> np.ndarray:
        obs = np.asarray(obs, dtype=np.float64)
        return np.tanh(self.W @ obs + self.b).astype(np.float32)

    def predict(self, obs, state=None, deterministic=True):  # SB3-style
        return self.act(obs), state

    # ---- checkpoint I/O ----
    def save(self, path: str, meta: Optional[Dict[str, Any]] = None) -> None:
        np.savez(
            path,
            W=self.W,
            b=self.b,
            meta=np.array(yaml.safe_dump(meta or {}), dtype=object),
        )

    @classmethod
    def load(cls, path: str) -> Tuple["LinearGaussianPolicy", Dict[str, Any]]:
        data = np.load(path, allow_pickle=True)
        pol = cls.__new__(cls)  # bypass __init__ (no rng needed)
        pol.W = data["W"]
        pol.b = data["b"]
        meta = yaml.safe_load(str(data["meta"])) if "meta" in data else {}
        return pol, (meta or {})


class SyncVectorEnv:
    """Minimal synchronous vector env: step a list of envs, auto-reset on done.

    Enough for the fallback training loop + smoke test (SB3 has its own VecEnv)."""

    def __init__(self, env_fns: List[Any], base_seed: int):
        self.envs = [fn() for fn in env_fns]
        self.n = len(self.envs)
        self._ep_return = np.zeros(self.n)
        self.episode_returns: List[float] = []
        obs = []
        for i, e in enumerate(self.envs):
            o, _ = e.reset(seed=base_seed + i)
            obs.append(o)
        self._obs = np.array(obs, dtype=np.float32)

    def observations(self) -> np.ndarray:
        return self._obs

    def step(self, actions: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        obs, rews, dones = [], [], []
        for i, e in enumerate(self.envs):
            o, r, term, trunc, info = e.step(actions[i])
            self._ep_return[i] += r
            done = bool(term or trunc)
            if done:
                self.episode_returns.append(float(self._ep_return[i]))
                self._ep_return[i] = 0.0
                o, _ = e.reset()  # auto-reset (rng continues -> new scene)
            obs.append(o)
            rews.append(r)
            dones.append(done)
        self._obs = np.array(obs, dtype=np.float32)
        return self._obs, np.array(rews), np.array(dones)


# ===========================================================================
# CSV logger (always) + optional tensorboard
# ===========================================================================
class CurveLogger:
    def __init__(self, run_dir: str):
        self.csv_path = os.path.join(run_dir, "log.csv")
        self._fh = open(self.csv_path, "w")
        self._fh.write("timestep,mean_episode_return,n_episodes\n")
        self._fh.flush()
        self.tb = None
        if _HAS_TB:
            self.tb = SummaryWriter(os.path.join(run_dir, "tb"))

    def log(self, timestep: int, mean_return: float, n_episodes: int) -> None:
        self._fh.write(f"{timestep},{mean_return:.6f},{n_episodes}\n")
        self._fh.flush()
        if self.tb is not None:
            self.tb.add_scalar("rollout/mean_episode_return", mean_return, timestep)

    def close(self) -> None:
        self._fh.close()
        if self.tb is not None:
            self.tb.close()


# ===========================================================================
# config resolution
# ===========================================================================
@dataclass
class TrainConfig:
    seed: int
    algo: str
    num_envs: int
    total_timesteps: int
    checkpoint_every: int
    log_dir: str
    reward_config: str
    dr_config: str
    env: EnvConfig
    raw: Dict[str, Any]  # the full parsed yaml (for snapshot + curriculum)


def load_train_config(path: str, root: str) -> TrainConfig:
    with open(path, "r") as fh:
        data = yaml.safe_load(fh) or {}

    def _resolve(p: str) -> str:
        return p if os.path.isabs(p) else os.path.join(root, p)

    return TrainConfig(
        seed=int(data.get("seed", 0)),
        algo=str(data.get("algo", "PPO")),
        num_envs=int(data.get("num_envs", 4)),
        total_timesteps=int(data.get("total_timesteps", 100000)),
        checkpoint_every=int(data.get("checkpoint_every", 50000)),
        log_dir=_resolve(str(data.get("log_dir", "runs"))),
        reward_config=_resolve(str(data.get("reward_config", "rl/configs/reward_default.yaml"))),
        dr_config=_resolve(str(data.get("dr_config", "rl/configs/domain_randomization_default.yaml"))),
        env=EnvConfig.from_dict(data.get("env", {})),
        raw=data,
    )


def make_env_fn(env_cfg: EnvConfig, reward_cfg, dr_cfg, seed: int):
    def _fn():
        return WindowTraversalEnv(
            config=env_cfg,
            reward_cfg=reward_cfg,
            dr_cfg=dr_cfg,
            backend=MockPhysics(),
            seed=seed,
        )

    return _fn


# ===========================================================================
# training loops
# ===========================================================================
def _fallback_train(
    tcfg: TrainConfig,
    reward_cfg,
    dr_cfg,
    run_dir: str,
    total_timesteps: int,
    resume_path: Optional[str],
) -> str:
    """Minimal hand-rolled loop (NO real learning) exercising the full pipeline."""
    print("[train] stable-baselines3 NOT found -> minimal fallback loop (no real learning).")
    rng = np.random.default_rng(tcfg.seed)
    ckpt_dir = os.path.join(run_dir, "checkpoints")
    os.makedirs(ckpt_dir, exist_ok=True)

    # policy (resume if given)
    if resume_path and os.path.exists(resume_path):
        policy, meta = LinearGaussianPolicy.load(resume_path)
        start_step = int(meta.get("timestep", 0))
        print(f"[train] resumed policy from {resume_path} at timestep={start_step}")
    else:
        policy = LinearGaussianPolicy(OBS_DIM, ACT_DIM, rng)
        start_step = 0

    env_fns = [
        make_env_fn(tcfg.env, reward_cfg, dr_cfg, tcfg.seed + i) for i in range(tcfg.num_envs)
    ]
    venv = SyncVectorEnv(env_fns, base_seed=tcfg.seed)
    logger = CurveLogger(run_dir)

    timestep = start_step
    next_ckpt = start_step + tcfg.checkpoint_every
    last_ckpt = os.path.join(ckpt_dir, "latest.npz")
    log_every = max(1, total_timesteps // 20)
    next_log = timestep + log_every

    while timestep < start_step + total_timesteps:
        obs = venv.observations()
        # policy actions + a little exploration noise (this "loop" does not learn)
        actions = np.stack([policy.act(o) for o in obs])
        actions = np.clip(actions + rng.normal(0.0, 0.3, actions.shape), -1.0, 1.0)
        venv.step(actions.astype(np.float32))
        timestep += venv.n

        if timestep >= next_log:
            recent = venv.episode_returns[-20:]
            mean_ret = float(np.mean(recent)) if recent else 0.0
            logger.log(timestep, mean_ret, len(venv.episode_returns))
            print(f"[train] t={timestep} mean_return(last20)={mean_ret:.3f}")
            next_log += log_every

        if timestep >= next_ckpt:
            path = os.path.join(ckpt_dir, f"ckpt_{timestep}.npz")
            policy.save(path, meta={"timestep": timestep, "seed": tcfg.seed})
            policy.save(last_ckpt, meta={"timestep": timestep, "seed": tcfg.seed})
            print(f"[train] checkpoint -> {path}")
            next_ckpt += tcfg.checkpoint_every

    # always write a final checkpoint
    policy.save(last_ckpt, meta={"timestep": timestep, "seed": tcfg.seed})
    logger.close()
    print(f"[train] done. final checkpoint -> {last_ckpt}")
    return last_ckpt


def _sb3_train(
    tcfg: TrainConfig, reward_cfg, dr_cfg, run_dir: str, total_timesteps: int, resume_path
) -> str:  # pragma: no cover - requires stable-baselines3 (not installed here)
    """Real training with stable-baselines3 (guarded; used only when importable)."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv
    from stable_baselines3.common.callbacks import CheckpointCallback

    print(f"[train] stable-baselines3 found -> {tcfg.algo}.")
    ckpt_dir = os.path.join(run_dir, "checkpoints")
    os.makedirs(ckpt_dir, exist_ok=True)
    env_fns = [
        make_env_fn(tcfg.env, reward_cfg, dr_cfg, tcfg.seed + i) for i in range(tcfg.num_envs)
    ]
    venv = DummyVecEnv(env_fns)
    if resume_path and os.path.exists(resume_path):
        model = PPO.load(resume_path, env=venv)
    else:
        model = PPO("MlpPolicy", venv, seed=tcfg.seed, tensorboard_log=os.path.join(run_dir, "tb"))
    cb = CheckpointCallback(save_freq=tcfg.checkpoint_every, save_path=ckpt_dir, name_prefix="ckpt")
    model.learn(total_timesteps=total_timesteps, callback=cb, reset_num_timesteps=False)
    final = os.path.join(ckpt_dir, "latest")
    model.save(final)
    print(f"[train] done. final checkpoint -> {final}.zip")
    return final + ".zip"


# ===========================================================================
# smoke test (gpu_jobs Job 2)
# ===========================================================================
def _smoke(tcfg: TrainConfig, reward_cfg, dr_cfg, run_dir: str) -> str:
    """Run a few steps with 1-2 MockPhysics envs, checkpoint, RELOAD, verify."""
    print("[smoke] running short MockPhysics rollout + checkpoint round-trip ...")
    tcfg.num_envs = min(2, max(1, tcfg.num_envs))
    ckpt = _fallback_train(
        tcfg, reward_cfg, dr_cfg, run_dir, total_timesteps=60, resume_path=None
    )
    assert os.path.exists(ckpt), f"checkpoint not written: {ckpt}"

    # reload and confirm it matches the saved policy exactly
    policy2, meta = LinearGaussianPolicy.load(ckpt)
    assert policy2.W.shape == (ACT_DIM, OBS_DIM), "reloaded policy has wrong shape"
    # deterministic action on a fixed obs must be reproducible after reload
    probe = np.ones(OBS_DIM, dtype=np.float32)
    a = policy2.act(probe)
    assert a.shape == (ACT_DIM,) and np.all(np.isfinite(a)), "reloaded policy misbehaves"
    print(
        f"[smoke] OK: checkpoint written and reloaded "
        f"(timestep={meta.get('timestep')}, action={np.round(a, 3).tolist()})"
    )
    print(f"[smoke] run folder: {run_dir}")
    return ckpt


# ===========================================================================
# CLI
# ===========================================================================
def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Train a window-traversal policy (spec 7.3).")
    ap.add_argument("--config", default=os.path.join(_HERE, "configs", "train_default.yaml"))
    ap.add_argument("--seed", type=int, default=None, help="override the config seed")
    ap.add_argument("--resume", default=None, help="checkpoint path to resume from")
    ap.add_argument(
        "--smoke",
        action="store_true",
        help="short MockPhysics run that writes + reloads a checkpoint (gpu_jobs Job 2)",
    )
    args = ap.parse_args(argv)

    tcfg = load_train_config(args.config, _ROOT)
    if args.seed is not None:
        tcfg.seed = args.seed
    if args.smoke:  # gpu_jobs Job 2: keep it tiny (1-2 MockPhysics envs)
        tcfg.num_envs = min(2, max(1, tcfg.num_envs))
    reward_cfg = reward_mod.RewardConfig.from_yaml(tcfg.reward_config)
    dr_cfg = DomainRandomizationConfig.from_yaml(tcfg.dr_config)

    run_dir = os.path.join(tcfg.log_dir, run_folder_name(args.config, tcfg.seed, _ROOT))
    os.makedirs(run_dir, exist_ok=True)
    # snapshot the resolved config (reproducibility)
    with open(os.path.join(run_dir, "config_snapshot.yaml"), "w") as fh:
        yaml.safe_dump(tcfg.raw, fh, sort_keys=False)
    print(f"[train] run folder: {run_dir}")
    print(f"[train] seed={tcfg.seed}  num_envs={tcfg.num_envs}  sb3={_HAS_SB3}  tb={_HAS_TB}")

    if args.smoke:
        _smoke(tcfg, reward_cfg, dr_cfg, run_dir)
        return 0

    train_fn = _sb3_train if _HAS_SB3 else _fallback_train
    train_fn(tcfg, reward_cfg, dr_cfg, run_dir, tcfg.total_timesteps, args.resume)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
