"""Per-episode domain randomisation for the RL env (spec 7.1 / 7.6).

WHAT
----
A *seeded* sampler that draws, once per episode, the values that make the sim2real
gap smaller:

  * randomised DYNAMICS the low-level PID must cope with -- ``mass`` (kg),
    ``thrust_scale`` (actuation gain multiplier), ``drag_coef`` (quadratic drag);
  * randomised SENSOR-NOISE levels -- the std-devs used *only* when the env's
    observation-noise option is on (spec 7.6). These model estimator error on the
    relative window pose / body velocity / attitude that a detector+VIO stack would
    produce, so the policy never sees clean GT (spec 7.1 "no sim cheating").

WHY it lives here and not in the env
------------------------------------
Keeping the *ranges* in yaml + the *sampling* in one seeded place means an episode
is fully reproducible from ``(config, seed)`` and the same draw can be replayed for
a fair A/B (e.g. policy-vs-baseline eval on identical dynamics, spec 7.4).

SCOPE NOTE (important)
----------------------
Lighting / scene / texture randomisation (spec 4.1) is applied at SCENE-GENERATION
time in the Isaac-Sim + Replicator pipeline (vision/), NOT here. This module only
randomises what the RL dynamics + observation model need. The window *geometry* of a
scene (count/placement/colour/size) is sampled by ``rl.window_env.sample_scene`` --
also seeded -- so the two randomisation sources compose deterministically.

Pure-Python/numpy; runnable + testable here.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Tuple

import numpy as np


def _mid(rng_range: Tuple[float, float]) -> float:
    lo, hi = rng_range
    return 0.5 * (float(lo) + float(hi))


@dataclass
class DomainRandomizationSample:
    """One concrete per-episode draw."""

    mass: float
    thrust_scale: float
    drag_coef: float
    # sensor-noise std-devs (used only if the env's observation_noise is enabled)
    pos_noise_std: float   # metres, on relative window position
    vel_noise_std: float   # m/s, on body-frame velocity
    att_noise_std: float   # unitless, on the (unit) gravity/attitude vector


@dataclass
class DomainRandomizationConfig:
    """Ranges for :meth:`sample`. Defaults are gentle STUBS -- widen once the
    policy trains stably on the nominal dynamics (curriculum, spec 7.3)."""

    enabled: bool = True

    # --- dynamics (uniform ranges) -----------------------------------------
    mass_range: Tuple[float, float] = (0.9, 1.1)          # kg, ~+/-10%
    thrust_scale_range: Tuple[float, float] = (0.9, 1.1)  # actuation gain
    drag_coef_range: Tuple[float, float] = (0.15, 0.30)   # quadratic drag

    # --- sensor noise std-devs (uniform ranges) ----------------------------
    pos_noise_std_range: Tuple[float, float] = (0.0, 0.10)   # m
    vel_noise_std_range: Tuple[float, float] = (0.0, 0.10)   # m/s
    att_noise_std_range: Tuple[float, float] = (0.0, 0.02)   # unit-vec noise

    # ---- (de)serialisation --------------------------------------------------
    @classmethod
    def from_dict(cls, d: Mapping[str, Any] | None) -> "DomainRandomizationConfig":
        d = dict(d or {})
        kwargs: dict[str, Any] = {}
        for f in cls.__dataclass_fields__:
            if f not in d:
                continue
            if f == "enabled":
                kwargs[f] = bool(d[f])
            else:
                lo, hi = d[f]
                kwargs[f] = (float(lo), float(hi))
        return cls(**kwargs)

    @classmethod
    def from_yaml(cls, path: str) -> "DomainRandomizationConfig":
        import yaml

        with open(path, "r") as fh:
            data = yaml.safe_load(fh) or {}
        if "domain_randomization" in data and isinstance(
            data["domain_randomization"], Mapping
        ):
            data = data["domain_randomization"]
        return cls.from_dict(data)

    # ---- sampling -----------------------------------------------------------
    def nominal(self) -> DomainRandomizationSample:
        """The midpoint / noise-free draw used when DR is disabled."""
        return DomainRandomizationSample(
            mass=_mid(self.mass_range),
            thrust_scale=_mid(self.thrust_scale_range),
            drag_coef=_mid(self.drag_coef_range),
            pos_noise_std=0.0,
            vel_noise_std=0.0,
            att_noise_std=0.0,
        )

    def sample(self, rng: np.random.Generator) -> DomainRandomizationSample:
        """Draw one episode's dynamics + noise levels from ``rng``.

        When ``enabled`` is False this returns :meth:`nominal` and does NOT touch
        ``rng`` state -- so toggling DR off is exactly the deterministic nominal.
        """
        if not self.enabled:
            return self.nominal()
        return DomainRandomizationSample(
            mass=float(rng.uniform(*self.mass_range)),
            thrust_scale=float(rng.uniform(*self.thrust_scale_range)),
            drag_coef=float(rng.uniform(*self.drag_coef_range)),
            pos_noise_std=float(rng.uniform(*self.pos_noise_std_range)),
            vel_noise_std=float(rng.uniform(*self.vel_noise_std_range)),
            att_noise_std=float(rng.uniform(*self.att_noise_std_range)),
        )
