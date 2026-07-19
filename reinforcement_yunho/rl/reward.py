"""Reward function for the window-traversal RL task (spec 7.2).

WHAT
----
``compute_reward(state, action, info, cfg) -> (total, terms)`` turns one
environment transition into a scalar reward plus a per-term breakdown. The five
terms are fixed by the checklist (spec 7.2):

    window_pass  : reward for actually flying through a window opening
    collision    : penalty for hitting a wall / window frame
    progress     : dense shaping reward for closing distance to the target window
    attitude     : penalty for tilting (keeps the flight sane / camera stable)
    energy       : penalty for control effort (commanded acceleration)

WHY the split matters
---------------------
Returning the signed per-term contributions (they sum to ``total``) lets 윤호
log each channel to TensorBoard and *inspect for reward hacking* -- e.g. a policy
that farms ``progress`` by wobbling toward and away from the window, or one that
never passes a window because ``energy`` dominates. Reward DESIGN ownership is
currently UNASSIGNED on the checklist, so the weights in
``rl/configs/reward_default.yaml`` are deliberately STUB values to be tuned.

CONTRACT
--------
* Weights come ONLY from ``RewardConfig`` (loaded from yaml). Nothing is
  hard-coded here except the term *structure* (spec 7.2: "reward weights live in
  a separate yaml config, never hard-coded").
* All penalty weights are stored as POSITIVE magnitudes in the yaml; the sign is
  applied in this module so the yaml reads naturally ("collision penalty = 10").
* ``terms`` values are the SIGNED contributions, so ``sum(terms.values()) ==
  total`` exactly -- that invariant is asserted in the smoke test.

This module is pure-Python/numpy and fully runnable + testable here.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Tuple

import numpy as np

# The five term keys are a fixed contract (spec 7.2). Anything that logs or
# validates rewards can import this.
REWARD_TERMS: Tuple[str, ...] = (
    "window_pass",
    "collision",
    "progress",
    "attitude",
    "energy",
)


@dataclass
class RewardConfig:
    """Reward weights (spec 7.2). All values are STUBS to be tuned by whoever
    picks up reward-design ownership.

    Penalty weights (``w_collision``, ``w_attitude``, ``w_energy``) are positive
    magnitudes; the negative sign is applied in :func:`compute_reward`.
    """

    # --- positive rewards ---------------------------------------------------
    w_window_pass: float = 10.0   # per window opening cleanly traversed
    success_bonus: float = 20.0   # extra one-off when the LAST window is passed
    w_progress: float = 1.0       # per metre of distance closed to target this step
    # --- penalties (positive magnitudes; applied as negatives) --------------
    w_collision: float = 10.0     # one-off when a wall/frame is hit
    w_attitude: float = 0.10      # * tilt^2 (radians^2) each step
    w_energy: float = 0.01        # * commanded-accel magnitude each step

    # ---- (de)serialisation --------------------------------------------------
    @classmethod
    def from_dict(cls, d: Mapping[str, Any] | None) -> "RewardConfig":
        d = dict(d or {})
        known = {f: d[f] for f in cls.__dataclass_fields__ if f in d}
        return cls(**{k: float(v) for k, v in known.items()})

    @classmethod
    def from_yaml(cls, path: str) -> "RewardConfig":
        import yaml  # pyyaml is a core dep (requirements.txt)

        with open(path, "r") as fh:
            data = yaml.safe_load(fh) or {}
        # allow either a flat file or a nested {"reward": {...}} block
        if "reward" in data and isinstance(data["reward"], Mapping):
            data = data["reward"]
        return cls.from_dict(data)


def compute_reward(
    state: Mapping[str, Any],
    action: Any,
    info: Mapping[str, Any],
    cfg: RewardConfig,
) -> Tuple[float, Dict[str, float]]:
    """Compute reward for a single step.

    Parameters
    ----------
    state : mapping
        Current drone state relevant to shaping. Read keys:
        ``tilt`` (float, radians from vertical -- used for the attitude penalty).
    action : array-like
        The normalised action the policy emitted. Only used as a fallback energy
        proxy if ``info["control_effort"]`` is absent.
    info : mapping
        Per-step event flags produced by the env. Read keys:
        ``passed_window`` (bool), ``collision`` (bool), ``success`` (bool),
        ``progress`` (float, metres closed to target this step),
        ``control_effort`` (float, commanded-accel magnitude m/s^2).
    cfg : RewardConfig
        Weights (from yaml).

    Returns
    -------
    (total, terms) : (float, dict[str, float])
        ``terms`` holds the SIGNED contribution of each of the 5 channels and
        sums to ``total``.
    """
    passed = float(bool(info.get("passed_window", False)))
    collided = float(bool(info.get("collision", False)))
    success = bool(info.get("success", False))
    progress = float(info.get("progress", 0.0))

    tilt = float(state.get("tilt", 0.0))

    # Energy proxy: prefer the true commanded-acceleration magnitude the env
    # measured; fall back to the action norm so the function is usable stand-alone.
    if "control_effort" in info:
        effort = float(info["control_effort"])
    else:
        effort = float(np.linalg.norm(np.asarray(action, dtype=np.float64)))

    terms: Dict[str, float] = {
        # window_pass folds in the one-off success bonus so the term count stays 5.
        "window_pass": cfg.w_window_pass * passed
        + (cfg.success_bonus if success else 0.0),
        "collision": -cfg.w_collision * collided,
        # progress may be negative (moving away) -- that is the point of shaping.
        "progress": cfg.w_progress * progress,
        "attitude": -cfg.w_attitude * (tilt * tilt),
        "energy": -cfg.w_energy * effort,
    }

    total = float(sum(terms.values()))
    if not np.isfinite(total):
        # Never hand a NaN/inf reward to the learner; surface it loudly instead.
        raise FloatingPointError(f"non-finite reward from terms={terms}")
    # normalise term values to plain floats for clean logging
    terms = {k: float(v) for k, v in terms.items()}
    return total, terms
