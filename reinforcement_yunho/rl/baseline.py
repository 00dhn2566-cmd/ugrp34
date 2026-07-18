"""``WaypointBaseline`` -- the non-learned reference policy for spec-7.4 eval.

WHAT / WHY
----------
The evaluation protocol (spec 7.4 / CONVENTIONS "RL boundaries") requires every
learned policy to be compared against a *simple-waypoint baseline* on the SAME
scenes. This is that baseline: it just steers straight at the current target
window centre. Because the env already reports the target window's relative
position in the heading frame (obs slice ``OBS_REL_WIN_POS``), the baseline is a
pure function of the observation -- it uses NO privileged/GT state, so it is a
fair reference (spec 7.1).

It shares the env's action space: a normalised heading-frame waypoint in
[-1,1]^3. The baseline outputs a unit-direction waypoint toward the window scaled
by ``gain`` (full commanded waypoint by default), which reliably makes progress
toward -- and through -- the opening.

API mirrors SB3 (``predict(obs) -> (action, state)``) so train/eval can treat a
learned model and this baseline interchangeably.
"""
from __future__ import annotations

import os
import sys
from typing import Any, Optional, Tuple

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from window_env import OBS_REL_WIN_POS  # noqa: E402  (shared obs layout)


class WaypointBaseline:
    """Steer straight at the current target window centre.

    Parameters
    ----------
    gain : float
        Scales the unit waypoint direction. 1.0 = command the maximum waypoint
        toward the window every step (aggressive but always makes progress).
    action_space : optional
        The env action space; if given, the action is clipped to it. Not required.
    """

    def __init__(self, gain: float = 1.0, action_space: Optional[Any] = None) -> None:
        self.gain = float(gain)
        self.action_space = action_space

    def act(self, obs: np.ndarray) -> np.ndarray:
        obs = np.asarray(obs, dtype=np.float64)
        rel = obs[OBS_REL_WIN_POS]                    # target dir in heading frame
        n = float(np.linalg.norm(rel))
        direction = rel / n if n > 1e-9 else np.zeros(3)
        action = np.clip(direction * self.gain, -1.0, 1.0)
        return action.astype(np.float32)

    def predict(
        self, obs: np.ndarray, state: Any = None, deterministic: bool = True
    ) -> Tuple[np.ndarray, Any]:
        """SB3-style predict so callers can use learned models and this baseline
        through one code path."""
        return self.act(obs), state
