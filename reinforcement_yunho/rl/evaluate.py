"""Evaluate a policy against the simple-waypoint baseline (spec 7.4).

PROTOCOL (spec 7.4 / CONVENTIONS "RL boundaries")
-------------------------------------------------
1. Generate ``N`` SEEDED random scenes (window count / placement / colour / size).
2. Roll out BOTH the policy-under-test AND the ``WaypointBaseline`` on the SAME
   scene set (identical scene + identical episode seed -> identical dynamics), so
   the comparison is apples-to-apples.
3. Report per-policy: success rate, collision rate, mean pass time (seconds, over
   successful episodes only). Write a summary CSV (+ optional per-scene CSV).

The policy-under-test is selected by ``--policy``:
    baseline  -> the WaypointBaseline itself (useful as a sanity/smoke run)
    random    -> uniform random actions (lower bound)
    <path>    -> a LinearGaussianPolicy checkpoint (.npz) from rl.train's fallback
                 loop  (SB3 .zip models load via their own API when available)

Pure-Python/numpy + MockPhysics -> runnable + testable here.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import reward as reward_mod  # noqa: E402
from baseline import WaypointBaseline  # noqa: E402
from domain_randomization import DomainRandomizationConfig  # noqa: E402
from window_env import (  # noqa: E402
    ACT_DIM,
    EnvConfig,
    MockPhysics,
    Scene,
    WindowTraversalEnv,
    sample_scene,
)


# ===========================================================================
# policies
# ===========================================================================
class RandomPolicy:
    """Uniform random action in [-1,1]^3 (evaluation lower bound). Seeded."""

    def __init__(self, seed: int = 0):
        self._rng = np.random.default_rng(seed)

    def predict(self, obs, state=None, deterministic=True):
        return self._rng.uniform(-1.0, 1.0, ACT_DIM).astype(np.float32), state


def load_policy(spec: str) -> Any:
    """Resolve ``--policy`` into an object with ``predict(obs) -> (action, state)``."""
    if spec == "baseline":
        return WaypointBaseline()
    if spec == "random":
        return RandomPolicy()
    if spec.endswith(".npz"):
        from train import LinearGaussianPolicy  # local import (sibling module)

        pol, _ = LinearGaussianPolicy.load(spec)
        return pol
    if spec.endswith(".zip"):  # pragma: no cover - needs stable-baselines3
        from stable_baselines3 import PPO

        return PPO.load(spec)
    raise ValueError(f"unrecognised --policy {spec!r} (use baseline|random|<*.npz>|<*.zip>)")


# ===========================================================================
# rollout + metrics
# ===========================================================================
@dataclass
class EpisodeResult:
    success: bool
    collision: bool
    steps: int
    pass_time_s: float  # NaN if not a success


def rollout(env: WindowTraversalEnv, policy: Any, scene: Scene, seed: int) -> EpisodeResult:
    """One episode of ``policy`` on a FIXED ``scene`` (same seed -> same dynamics)."""
    obs, _ = env.reset(seed=seed, options={"scene": scene})
    success = collision = False
    steps = 0
    while True:
        action, _ = policy.predict(obs, deterministic=True)
        obs, _r, terminated, truncated, info = env.step(action)
        steps += 1
        if terminated or truncated:
            success = bool(info.get("success", False))
            collision = bool(info.get("collision", False))
            break
    pass_time = steps * env.cfg.dt if success else float("nan")
    return EpisodeResult(success, collision, steps, pass_time)


@dataclass
class PolicyMetrics:
    name: str
    n: int
    success_rate: float
    collision_rate: float
    mean_pass_time_s: float  # over successful episodes; NaN if none succeeded


def summarise(name: str, results: List[EpisodeResult]) -> PolicyMetrics:
    n = len(results)
    succ = [r for r in results if r.success]
    success_rate = len(succ) / n if n else 0.0
    collision_rate = sum(1 for r in results if r.collision) / n if n else 0.0
    mean_pt = float(np.mean([r.pass_time_s for r in succ])) if succ else float("nan")
    return PolicyMetrics(name, n, success_rate, collision_rate, mean_pt)


def evaluate(
    env_cfg: EnvConfig,
    reward_cfg,
    dr_cfg,
    policy_spec: str,
    n_scenes: int,
    seed: int,
) -> Tuple[List[PolicyMetrics], List[Dict[str, Any]]]:
    """Run the spec-7.4 comparison. Returns (summary metrics, per-scene rows)."""
    master = np.random.default_rng(seed)
    # Pre-build the shared scene set + a per-scene episode seed (used for BOTH
    # policies so dynamics/DR match exactly).
    scenes: List[Tuple[int, Scene]] = []
    for _ in range(n_scenes):
        scene_seed = int(master.integers(0, 2**31 - 1))
        scenes.append((scene_seed, sample_scene(np.random.default_rng(scene_seed), env_cfg)))

    policies: Dict[str, Any] = {
        "under_test": load_policy(policy_spec),
        "baseline": WaypointBaseline(),
    }
    # A single env instance is reused (reset pins each scene).
    env = WindowTraversalEnv(config=env_cfg, reward_cfg=reward_cfg, dr_cfg=dr_cfg, backend=MockPhysics())

    per_scene: List[Dict[str, Any]] = []
    results: Dict[str, List[EpisodeResult]] = {k: [] for k in policies}
    for idx, (scene_seed, scene) in enumerate(scenes):
        row: Dict[str, Any] = {
            "scene_idx": idx,
            "scene_seed": scene_seed,
            "n_windows": scene.n_windows,
        }
        for pname, pol in policies.items():
            res = rollout(env, pol, scene, seed=scene_seed)
            results[pname].append(res)
            row[f"{pname}_success"] = int(res.success)
            row[f"{pname}_collision"] = int(res.collision)
            row[f"{pname}_steps"] = res.steps
            row[f"{pname}_pass_time_s"] = res.pass_time_s
        per_scene.append(row)

    label = {"under_test": policy_spec, "baseline": "baseline"}
    metrics = [summarise(label[k], results[k]) for k in policies]
    return metrics, per_scene


# ===========================================================================
# CSV output
# ===========================================================================
def write_summary_csv(path: str, metrics: List[PolicyMetrics], n_scenes: int, seed: int) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["policy", "n_scenes", "success_rate", "collision_rate", "mean_pass_time_s", "eval_seed"])
        for m in metrics:
            w.writerow(
                [
                    m.name,
                    m.n,
                    f"{m.success_rate:.4f}",
                    f"{m.collision_rate:.4f}",
                    "nan" if np.isnan(m.mean_pass_time_s) else f"{m.mean_pass_time_s:.4f}",
                    seed,
                ]
            )


def write_per_scene_csv(path: str, per_scene: List[Dict[str, Any]]) -> None:
    if not per_scene:
        return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = list(per_scene[0].keys())
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for row in per_scene:
            w.writerow(row)


# ===========================================================================
# CLI
# ===========================================================================
def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Evaluate a policy vs the waypoint baseline (spec 7.4).")
    ap.add_argument("--config", default=os.path.join(_HERE, "configs", "train_default.yaml"))
    ap.add_argument("--policy", default="baseline", help="baseline | random | <checkpoint.npz|.zip>")
    ap.add_argument("--num-scenes", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None, help="summary CSV path (default: eval_<policy>_<seed>.csv)")
    ap.add_argument("--per-scene-out", default=None, help="optional per-scene CSV path")
    args = ap.parse_args(argv)

    env_cfg = EnvConfig.from_yaml(args.config)
    # reward/dr configs are referenced from the train yaml; fall back to defaults.
    import yaml

    with open(args.config) as fh:
        raw = yaml.safe_load(fh) or {}

    def _res(p, default):
        p = p or default
        return p if os.path.isabs(p) else os.path.join(_ROOT, p)

    reward_cfg = reward_mod.RewardConfig.from_yaml(
        _res(raw.get("reward_config"), "rl/configs/reward_default.yaml")
    )
    dr_cfg = DomainRandomizationConfig.from_yaml(
        _res(raw.get("dr_config"), "rl/configs/domain_randomization_default.yaml")
    )

    metrics, per_scene = evaluate(
        env_cfg, reward_cfg, dr_cfg, args.policy, args.num_scenes, args.seed
    )

    out = args.out or f"eval_{os.path.basename(args.policy).replace('/', '_')}_{args.seed}.csv"
    write_summary_csv(out, metrics, args.num_scenes, args.seed)
    if args.per_scene_out:
        write_per_scene_csv(args.per_scene_out, per_scene)

    # console summary
    print(f"[eval] {args.num_scenes} scenes, seed={args.seed}  (policy-under-test vs baseline)")
    print(f"{'policy':<24}{'success':>9}{'collision':>11}{'mean_pass_s':>13}")
    for m in metrics:
        pt = "nan" if np.isnan(m.mean_pass_time_s) else f"{m.mean_pass_time_s:.3f}"
        print(f"{m.name:<24}{m.success_rate:>9.3f}{m.collision_rate:>11.3f}{pt:>13}")
    print(f"[eval] summary CSV -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
