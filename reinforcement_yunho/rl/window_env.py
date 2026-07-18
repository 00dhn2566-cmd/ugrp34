"""``WindowTraversalEnv`` -- the RL environment for flying a drone through a
sequence of coloured windows in traversal order (spec 7.1).

DESIGN (obeys CONVENTIONS.md "RL boundaries")
---------------------------------------------
* **Action = a waypoint setpoint**, NOT motor commands. The policy emits a
  normalised 3-vector in [-1,1]^3; the env scales it to a metric displacement in
  the drone's *heading frame* (yaw-only, gravity-aligned) and a low-level PID
  (modelled by the physics backend) tracks it. End-to-end motor control is out of
  scope this semester (CONVENTIONS "RL boundaries").
* **Observation = CLEAN state only, no sim cheating** (spec 7.1). We do NOT feed
  GT depth or GT world pose. Instead the observation is the kind of thing a
  detector + state-estimator would deliver:
      - relative pose of the *target* window (position + normal) in the heading
        frame -- i.e. bearing/range/orientation as a detector would report;
      - drone body-frame velocity;
      - attitude as the gravity direction in the body frame (what an IMU gives);
      - the previous action.
  An OPTION (default OFF) injects estimator noise (spec 7.6); levels come from
  domain randomisation.
* **Reward is delegated** to ``rl.reward.compute_reward`` with weights from yaml
  (spec 7.2). This file never hard-codes reward magnitudes.
* **Deterministic** given a seed: one ``np.random.Generator`` drives scene
  sampling, domain randomisation and observation noise.

BACKEND (pluggable)
-------------------
``PhysicsBackend`` is the interface. ``MockPhysics`` is a fully-runnable
point-mass + PID + quadratic-drag integrator so the whole env runs and is tested
here. ``IsaacSimBackend`` is an import-guarded stub for the real Isaac-Sim rollout.

COORDINATE FRAMES (per CONVENTIONS.md)
--------------------------------------
World is right-handed, +Z up, metres. Windows use the shared window model
(+X_local right, +Y_local up, +Z_local = outward normal on the approach side) and
the shared ``common.geometry.window_corners_world`` for their corners. The window
approach axis in a sampled scene is world +X.
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple

import numpy as np

# --- bootstrap so `common` (repo root) and sibling rl modules import from any cwd
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from common import window_corners_world  # noqa: E402  (shared geometry -- do NOT re-derive)

import reward as reward_mod  # noqa: E402
from domain_randomization import (  # noqa: E402
    DomainRandomizationConfig,
    DomainRandomizationSample,
)

# --- optional gymnasium (guarded) -------------------------------------------
# The file must import and STEP even without gymnasium installed, so we fall back
# to a minimal base class + Box space with the same reset/step contract.
try:
    import gymnasium as gym
    from gymnasium import spaces as gym_spaces

    _HAS_GYM = True
except Exception:  # pragma: no cover - exercised on machines without gymnasium
    gym = None  # type: ignore
    gym_spaces = None  # type: ignore
    _HAS_GYM = False


# ---------------------------------------------------------------------------
# Constants (world model)
# ---------------------------------------------------------------------------
G = 9.81                                   # m/s^2
UP = np.array([0.0, 0.0, 1.0])             # world up (CONVENTIONS: +Z up)

# Colour -> traversal order_index (CONVENTIONS "Colour / class mapping"). Windows
# are traversed in ascending order_index: red(0) -> green(1) -> blue(2).
ORDER_INDEX: Dict[str, int] = {"red": 0, "green": 1, "blue": 2}
COLOURS_IN_ORDER: Tuple[str, ...] = ("red", "green", "blue")

# ---------------------------------------------------------------------------
# Observation layout (flat float32 vector). Kept as module constants so
# baseline.py / evaluate.py agree on the indices. Frames noted per slice.
# ---------------------------------------------------------------------------
OBS_REL_WIN_POS = slice(0, 3)    # target window centre, heading frame, metres
OBS_WIN_NORMAL = slice(3, 6)     # target window normal, heading frame, unit
OBS_WIN_SIZE = slice(6, 8)       # (width, height), metres
OBS_VEL_BODY = slice(8, 11)      # drone velocity, body frame, m/s
OBS_GRAV_BODY = slice(11, 14)    # gravity direction, body frame, unit (== attitude)
OBS_PREV_ACTION = slice(14, 17)  # previous action, normalised [-1,1]
OBS_DIM = 17
ACT_DIM = 3


# ---------------------------------------------------------------------------
# small math helpers
# ---------------------------------------------------------------------------
def _normalize(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.zeros_like(v)
    return v / n


def _rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ===========================================================================
# Scene model
# ===========================================================================
@dataclass
class Window:
    """One target window, in world coordinates.

    ``R`` maps window-local axes into world axes (columns = [x_right, y_up,
    z_normal]); ``normal`` == ``R[:, 2]`` points toward the approach side (toward
    the drone). Corners come from the shared ``common.geometry`` helper so labels
    and RL agree on corner geometry.
    """

    center: np.ndarray            # (3,) world
    R: np.ndarray                 # (3,3) R_world_win
    width: float
    height: float
    color: str                    # "red" | "green" | "blue"
    order_index: int              # traversal order (== YOLO class)

    @property
    def normal(self) -> np.ndarray:
        return self.R[:, 2]

    def corners_world(self) -> np.ndarray:
        # reuse shared geometry (CONVENTIONS: do not re-derive)
        return window_corners_world(self.center, self.R, self.width, self.height)

    def local_of(self, p_world: np.ndarray) -> np.ndarray:
        """World point -> window-local coords (x_right, y_up, z_along_normal)."""
        return self.R.T @ (np.asarray(p_world, dtype=np.float64) - self.center)


@dataclass
class Scene:
    """A sampled scene: windows sorted by traversal order + drone start pose."""

    windows: List[Window]
    start_pos: np.ndarray  # (3,) world

    @property
    def n_windows(self) -> int:
        return len(self.windows)


# ===========================================================================
# Environment config
# ===========================================================================
@dataclass
class EnvConfig:
    """All env parameters (loadable from the ``env:`` block of a training yaml).

    Numbers are engineering defaults chosen so a MockPhysics rollout is stable at
    dt=0.05 s; tune with the curriculum (spec 7.3)."""

    # --- integration -------------------------------------------------------
    dt: float = 0.05               # s per step
    max_steps: int = 400           # episode step cap -> truncation

    # --- action scaling ----------------------------------------------------
    waypoint_scale: float = 2.0    # metres; action in [-1,1]^3 -> +/-2 m heading-frame

    # --- low-level PID (modelled by the backend) ---------------------------
    kp: float = 4.0
    kd: float = 3.0
    accel_max: float = 12.0        # m/s^2 controller acceleration limit

    # --- nominal dynamics (DR overrides these per-episode) -----------------
    mass: float = 1.0              # kg
    thrust_scale: float = 1.0
    drag_coef: float = 0.20

    # --- pass / collision geometry -----------------------------------------
    # A window is an opening in a wall of half-extent `wall_half_extent`. Crossing
    # the window plane inside the opening = pass; inside the wall = collision.
    wall_half_extent: float = 3.0  # m, wall size around the opening
    bounds_radius: float = 40.0    # m from scene centroid before out-of-bounds

    # --- scene sampling (window geometry randomisation) --------------------
    num_windows_min: int = 1
    num_windows_max: int = 3
    window_spacing: Tuple[float, float] = (4.0, 6.0)   # m between consecutive windows
    window_lateral: float = 1.0    # m, max |y| and height jitter about base_height
    base_height: float = 1.5       # m, nominal window/flight height
    window_yaw_deg: float = 20.0   # +/- yaw about world Z (spec 4.1 allows +/-60)
    window_width: Tuple[float, float] = (1.0, 1.6)     # m
    window_height: Tuple[float, float] = (1.0, 1.6)    # m
    start_back: float = 3.0        # m the drone starts behind x=0 (approach along +X)

    # --- toggles -----------------------------------------------------------
    domain_randomization: bool = False   # per-episode dynamics/noise DR
    observation_noise: bool = False      # spec 7.6 estimator noise; default OFF = clean

    # default sensor-noise std-devs used when observation_noise is ON but DR is OFF
    obs_pos_noise_std: float = 0.05
    obs_vel_noise_std: float = 0.05
    obs_att_noise_std: float = 0.01

    @classmethod
    def from_dict(cls, d: Mapping[str, Any] | None) -> "EnvConfig":
        d = dict(d or {})
        kwargs: dict[str, Any] = {}
        for f, fld in cls.__dataclass_fields__.items():
            if f not in d:
                continue
            val = d[f]
            if f in ("window_spacing", "window_width", "window_height"):
                lo, hi = val
                kwargs[f] = (float(lo), float(hi))
            elif f in ("num_windows_min", "num_windows_max", "max_steps"):
                kwargs[f] = int(val)
            elif f in ("domain_randomization", "observation_noise"):
                kwargs[f] = bool(val)
            else:
                kwargs[f] = float(val)
        return cls(**kwargs)

    @classmethod
    def from_yaml(cls, path: str) -> "EnvConfig":
        import yaml

        with open(path, "r") as fh:
            data = yaml.safe_load(fh) or {}
        if "env" in data and isinstance(data["env"], Mapping):
            data = data["env"]
        return cls.from_dict(data)


# ===========================================================================
# Physics backends
# ===========================================================================
@dataclass
class PhysicsParams:
    """Per-episode dynamics handed to a backend (nominal EnvConfig values, unless
    domain randomisation overrode mass/thrust/drag)."""

    mass: float
    thrust_scale: float
    drag_coef: float
    kp: float
    kd: float
    accel_max: float


@dataclass
class StepPhysics:
    """What a backend reports after a step (world frame + control effort)."""

    pos: np.ndarray       # (3,)
    vel: np.ndarray       # (3,)
    accel_thrust: np.ndarray  # (3,) actuation accel (thrust dir source for attitude)
    control_effort: float     # ||commanded accel|| (energy proxy)


class PhysicsBackend:
    """Interface a backend must implement. The env only ever calls these."""

    def reset(self, start_pos: np.ndarray, params: PhysicsParams) -> StepPhysics:
        raise NotImplementedError

    def step(self, waypoint_world: np.ndarray, dt: float) -> StepPhysics:
        raise NotImplementedError


class MockPhysics(PhysicsBackend):
    """Point-mass drone with a low-level PID position tracker + quadratic drag.

    The policy's waypoint is passed already resolved to a WORLD target position.
    The PID produces a commanded acceleration (this is the "low-level controller"
    that CONVENTIONS says follows the waypoint); ``thrust_scale`` and ``drag_coef``
    / ``mass`` (all domain-randomisable) shape the realised motion. Gravity is
    assumed thrust-compensated (a hovering quad), so the net commanded accel *is*
    the motion accel -- we still recover a tilt for the attitude signal from the
    thrust vector needed to produce it.

    Semi-implicit (symplectic) Euler is used for stability at dt=0.05 s.
    """

    def __init__(self) -> None:
        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._a = np.zeros(3)
        self._params: Optional[PhysicsParams] = None

    def reset(self, start_pos: np.ndarray, params: PhysicsParams) -> StepPhysics:
        self._p = np.asarray(start_pos, dtype=np.float64).copy()
        self._v = np.zeros(3)
        self._a = np.zeros(3)
        self._params = params
        return StepPhysics(self._p.copy(), self._v.copy(), self._a.copy(), 0.0)

    def step(self, waypoint_world: np.ndarray, dt: float) -> StepPhysics:
        prm = self._params
        assert prm is not None, "reset() before step()"
        p, v = self._p, self._v

        pos_err = np.asarray(waypoint_world, dtype=np.float64) - p
        a_cmd = prm.kp * pos_err - prm.kd * v          # PID (P on pos, D on vel)
        effort = float(np.linalg.norm(a_cmd))          # energy proxy (pre-clip)
        # controller acceleration limit
        if effort > prm.accel_max:
            a_cmd = a_cmd * (prm.accel_max / effort)
        a_thrust = prm.thrust_scale * a_cmd            # actuation gain (DR)
        # quadratic aerodynamic drag: F = -c|v|v  ->  a = F/m
        speed = float(np.linalg.norm(v))
        a_drag = -(prm.drag_coef * speed / prm.mass) * v
        a_net = a_thrust + a_drag

        v_new = v + a_net * dt
        p_new = p + v_new * dt                         # semi-implicit Euler
        self._p, self._v, self._a = p_new, v_new, a_thrust
        return StepPhysics(p_new.copy(), v_new.copy(), a_thrust.copy(), effort)


class IsaacSimBackend(PhysicsBackend):
    """STUB: real rollout inside Isaac Sim (drone dynamics + PID + collisions).

    Import-guarded so this module still imports on a machine without Isaac Sim.
    The REAL waypoint->PID->motor logic runs in Isaac Sim; the *interface* is
    identical to MockPhysics so the env code is unchanged. Fill in when the sim
    integration lands (owner: 윤호 + sim team).
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        try:
            import omni.replicator.core  # noqa: F401  (marker for the Isaac env)
        except Exception as exc:  # pragma: no cover - no Isaac Sim here
            raise ImportError(
                "IsaacSimBackend requires the Isaac Sim python env (omni.replicator "
                "is not importable here). Use MockPhysics for local runs/tests."
            ) from exc
        raise NotImplementedError(
            "IsaacSimBackend is a stub: wire drone spawn, PID waypoint tracking and "
            "collision queries to the same PhysicsBackend interface as MockPhysics."
        )


# ===========================================================================
# Scene sampling (seeded window-geometry randomisation)
# ===========================================================================
def _window_rotation(yaw: float) -> np.ndarray:
    """R_world_win for a window whose base normal faces -X (toward an approaching
    drone) rotated by ``yaw`` about world +Z. Columns = [x_right, y_up, z_normal].

    base (yaw=0): z_normal=-X (faces the drone), y_up=+Z, x_right = y_up x z_normal
    = (0,0,1) x (-1,0,0) = (0,-1,0). This is right-handed (x_right x y_up = z_normal).
    """
    base = np.column_stack(
        [
            np.array([0.0, -1.0, 0.0]),  # x_right
            np.array([0.0, 0.0, 1.0]),   # y_up
            np.array([-1.0, 0.0, 0.0]),  # z_normal (faces -X)
        ]
    )
    return _rot_z(yaw) @ base


def sample_scene(rng: np.random.Generator, cfg: EnvConfig) -> Scene:
    """Sample one random scene (count / placement / colour / size), seeded by
    ``rng``. Windows are laid out along the world +X approach axis and coloured in
    traversal order red->green->blue (CONVENTIONS colour map). Deterministic given
    the rng state."""
    n = int(rng.integers(cfg.num_windows_min, cfg.num_windows_max + 1))
    windows: List[Window] = []
    x = 0.0
    for i in range(n):
        x += float(rng.uniform(*cfg.window_spacing))
        y = float(rng.uniform(-cfg.window_lateral, cfg.window_lateral))
        z = cfg.base_height + float(rng.uniform(-cfg.window_lateral, cfg.window_lateral))
        yaw = math.radians(float(rng.uniform(-cfg.window_yaw_deg, cfg.window_yaw_deg)))
        w = float(rng.uniform(*cfg.window_width))
        h = float(rng.uniform(*cfg.window_height))
        color = COLOURS_IN_ORDER[i % len(COLOURS_IN_ORDER)]
        windows.append(
            Window(
                center=np.array([x, y, z]),
                R=_window_rotation(yaw),
                width=w,
                height=h,
                color=color,
                order_index=ORDER_INDEX[color],
            )
        )
    windows.sort(key=lambda win: win.order_index)  # enforce traversal order
    start_pos = np.array([-cfg.start_back, 0.0, cfg.base_height])
    return Scene(windows=windows, start_pos=start_pos)


# ===========================================================================
# The environment
# ===========================================================================
# Choose a base class: real gymnasium.Env if available, else a minimal shim that
# preserves the reset/step signatures so the env still imports and STEPS.
if _HAS_GYM:
    _EnvBase = gym.Env  # type: ignore
else:  # pragma: no cover - exercised on machines without gymnasium

    class _EnvBase:  # minimal local fallback with the gymnasium contract
        metadata: Dict[str, Any] = {"render_modes": []}

        def reset(self, *, seed=None, options=None):  # noqa: D401
            raise NotImplementedError

        def step(self, action):
            raise NotImplementedError

        def close(self):
            pass


class _BoxSpace:
    """Tiny stand-in for ``gymnasium.spaces.Box`` used when gymnasium is absent.
    Supports the bits the env / callers need: ``shape``, ``sample(rng)``, ``contains``."""

    def __init__(self, low, high, shape, dtype=np.float32):
        self.low = np.broadcast_to(np.asarray(low, dtype=dtype), shape).copy()
        self.high = np.broadcast_to(np.asarray(high, dtype=dtype), shape).copy()
        self.shape = tuple(shape)
        self.dtype = dtype

    def sample(self, rng: Optional[np.random.Generator] = None) -> np.ndarray:
        rng = rng or np.random.default_rng()
        return rng.uniform(self.low, self.high).astype(self.dtype)

    def contains(self, x) -> bool:
        x = np.asarray(x)
        return bool(x.shape == self.shape and np.all(x >= self.low) and np.all(x <= self.high))


def _make_box(low, high, shape) -> Any:
    if _HAS_GYM:
        return gym_spaces.Box(low=low, high=high, shape=shape, dtype=np.float32)
    return _BoxSpace(low=low, high=high, shape=shape, dtype=np.float32)


class WindowTraversalEnv(_EnvBase):
    """Gymnasium env: fly the drone through the coloured windows in order.

    reset/step follow the gymnasium 5-tuple contract. With MockPhysics the env is
    fully runnable + deterministic here; swap in IsaacSimBackend for the real sim.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        config: Optional[EnvConfig | Mapping[str, Any]] = None,
        reward_cfg: Optional[reward_mod.RewardConfig] = None,
        dr_cfg: Optional[DomainRandomizationConfig] = None,
        backend: Optional[PhysicsBackend] = None,
        seed: Optional[int] = None,
    ) -> None:
        super().__init__()
        if config is None:
            self.cfg = EnvConfig()
        elif isinstance(config, EnvConfig):
            self.cfg = config
        else:
            self.cfg = EnvConfig.from_dict(config)
        self.reward_cfg = reward_cfg or reward_mod.RewardConfig()
        self.dr_cfg = dr_cfg or DomainRandomizationConfig(enabled=self.cfg.domain_randomization)
        self.backend: PhysicsBackend = backend or MockPhysics()

        # spaces
        self.observation_space = _make_box(-np.inf, np.inf, (OBS_DIM,))
        self.action_space = _make_box(-1.0, 1.0, (ACT_DIM,))

        # rng + episode state (populated on reset)
        self._rng = np.random.default_rng(seed)
        self._scene: Optional[Scene] = None
        self._target_idx = 0
        self._steps = 0
        self._prev_action = np.zeros(ACT_DIM, dtype=np.float64)
        self._yaw = 0.0
        self._prev_dist = 0.0
        self._dr_sample: Optional[DomainRandomizationSample] = None
        self._last_phys: Optional[StepPhysics] = None
        self._R_world_body = np.eye(3)

    # ---- helpers ----------------------------------------------------------
    @property
    def scene(self) -> Optional[Scene]:
        return self._scene

    @property
    def target_window(self) -> Optional[Window]:
        if self._scene is None or self._target_idx >= self._scene.n_windows:
            return None
        return self._scene.windows[self._target_idx]

    def _scene_centroid(self) -> np.ndarray:
        assert self._scene is not None
        return np.mean([w.center for w in self._scene.windows], axis=0)

    def _yaw_to_target(self, p: np.ndarray) -> float:
        """Heading yaw so the drone faces the current target window (horizontal
        bearing). Defines the heading frame used for action + window-relative obs."""
        tgt = self.target_window
        if tgt is None:
            return self._yaw
        d = tgt.center - p
        if abs(d[0]) < 1e-9 and abs(d[1]) < 1e-9:
            return self._yaw
        return math.atan2(float(d[1]), float(d[0]))

    def _attitude(self, phys: StepPhysics, yaw: float) -> np.ndarray:
        """Body rotation R_world_body from the thrust vector (roll/pitch) and the
        heading yaw. body +Z is the thrust axis; body +X is yaw heading projected
        perpendicular to it."""
        thrust_vec = phys.accel_thrust + G * UP  # thrust must counter gravity + accelerate
        bz = _normalize(thrust_vec)
        if np.linalg.norm(bz) < 1e-9:
            bz = UP.copy()
        bx_des = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        bx = _normalize(bx_des - float(bx_des @ bz) * bz)
        if np.linalg.norm(bx) < 1e-9:  # degenerate (thrust ~ horizontal); pick any
            bx = _normalize(np.cross(bz, UP))
        by = np.cross(bz, bx)
        return np.column_stack([bx, by, bz])

    def _build_obs(self, phys: StepPhysics) -> np.ndarray:
        """Assemble the CLEAN observation, then optionally inject estimator noise
        (spec 7.6). No GT depth / GT world pose is ever exposed (spec 7.1)."""
        p, v = phys.pos, phys.vel
        yaw = self._yaw
        R_heading = _rot_z(yaw)
        R_body = self._R_world_body

        tgt = self.target_window
        if tgt is not None:
            rel_pos_h = R_heading.T @ (tgt.center - p)
            normal_h = R_heading.T @ tgt.normal
            size = np.array([tgt.width, tgt.height])
        else:  # all windows passed -- zeros (episode is ending anyway)
            rel_pos_h = np.zeros(3)
            normal_h = np.zeros(3)
            size = np.zeros(2)

        vel_body = R_body.T @ v
        grav_body = R_body.T @ (-UP)  # unit gravity in body frame == attitude (IMU-like)

        obs = np.empty(OBS_DIM, dtype=np.float64)
        obs[OBS_REL_WIN_POS] = rel_pos_h
        obs[OBS_WIN_NORMAL] = normal_h
        obs[OBS_WIN_SIZE] = size
        obs[OBS_VEL_BODY] = vel_body
        obs[OBS_GRAV_BODY] = grav_body
        obs[OBS_PREV_ACTION] = self._prev_action

        if self.cfg.observation_noise:
            obs = self._inject_noise(obs)
        return obs.astype(np.float32)

    def _inject_noise(self, obs: np.ndarray) -> np.ndarray:
        """Add estimator noise (spec 7.6). Std-devs come from the DR draw when DR
        is on, else from the EnvConfig defaults. Uses the env rng -> reproducible."""
        s = self._dr_sample
        if s is not None and self.cfg.domain_randomization:
            pos_std, vel_std, att_std = s.pos_noise_std, s.vel_noise_std, s.att_noise_std
        else:
            pos_std = self.cfg.obs_pos_noise_std
            vel_std = self.cfg.obs_vel_noise_std
            att_std = self.cfg.obs_att_noise_std
        obs = obs.copy()
        obs[OBS_REL_WIN_POS] += self._rng.normal(0.0, pos_std, 3)
        obs[OBS_VEL_BODY] += self._rng.normal(0.0, vel_std, 3)
        obs[OBS_GRAV_BODY] += self._rng.normal(0.0, att_std, 3)
        return obs

    def _dist_to_target(self, p: np.ndarray) -> float:
        tgt = self.target_window
        if tgt is None:
            return 0.0
        return float(np.linalg.norm(tgt.center - p))

    # ---- gymnasium API ----------------------------------------------------
    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Mapping[str, Any]] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        options = dict(options or {})
        # Scene: caller may pin one (eval reuses the SAME scene across policies).
        self._scene = options.get("scene") or sample_scene(self._rng, self.cfg)

        # Domain-randomisation hook (per-episode dynamics + noise levels).
        if "dr" in options and isinstance(options["dr"], DomainRandomizationSample):
            self._dr_sample = options["dr"]
        elif self.cfg.domain_randomization:
            self._dr_sample = self.dr_cfg.sample(self._rng)
        else:
            self._dr_sample = self.dr_cfg.nominal()

        params = PhysicsParams(
            mass=self._dr_sample.mass if self.cfg.domain_randomization else self.cfg.mass,
            thrust_scale=self._dr_sample.thrust_scale
            if self.cfg.domain_randomization
            else self.cfg.thrust_scale,
            drag_coef=self._dr_sample.drag_coef
            if self.cfg.domain_randomization
            else self.cfg.drag_coef,
            kp=self.cfg.kp,
            kd=self.cfg.kd,
            accel_max=self.cfg.accel_max,
        )

        self._target_idx = 0
        self._steps = 0
        self._prev_action = np.zeros(ACT_DIM, dtype=np.float64)

        phys = self.backend.reset(self._scene.start_pos, params)
        self._last_phys = phys
        self._yaw = self._yaw_to_target(phys.pos)
        self._R_world_body = self._attitude(phys, self._yaw)
        self._prev_dist = self._dist_to_target(phys.pos)

        obs = self._build_obs(phys)
        info = {
            "n_windows": self._scene.n_windows,
            "target_idx": self._target_idx,
            "dr": self._dr_sample,
        }
        return obs, info

    def step(
        self, action: Any
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        assert self._scene is not None and self._last_phys is not None, "call reset() first"
        action = np.clip(np.asarray(action, dtype=np.float64).reshape(ACT_DIM), -1.0, 1.0)

        # Resolve the normalised heading-frame waypoint into a WORLD target for the
        # low-level PID: displacement in the yaw-only heading frame, scaled to metres.
        p = self._last_phys.pos
        R_heading = _rot_z(self._yaw)
        waypoint_world = p + R_heading @ (action * self.cfg.waypoint_scale)

        p_prev = p.copy()
        phys = self.backend.step(waypoint_world, self.cfg.dt)
        p_new = phys.pos

        # --- pass / collision detection against the CURRENT target window ------
        passed_window = False
        collision = False
        success = False
        tgt = self.target_window
        if tgt is not None:
            n = tgt.normal
            d_prev = float(n @ (p_prev - tgt.center))
            d_cur = float(n @ (p_new - tgt.center))
            # approach side is +normal; a traversal crosses from d>0 to d<=0
            if d_prev > 0.0 >= d_cur and (d_prev - d_cur) > 1e-9:
                t = d_prev / (d_prev - d_cur)
                cross = p_prev + t * (p_new - p_prev)
                loc = tgt.local_of(cross)
                lx, ly = abs(float(loc[0])), abs(float(loc[1]))
                if lx <= tgt.width / 2.0 and ly <= tgt.height / 2.0:
                    passed_window = True
                elif lx <= self.cfg.wall_half_extent and ly <= self.cfg.wall_half_extent:
                    collision = True  # hit the wall around the opening
                # crossing far outside the wall: neither (flew around) -> handled by bounds

        # --- advance target / progress shaping --------------------------------
        progress = 0.0
        if passed_window:
            self._target_idx += 1
            if self._target_idx >= self._scene.n_windows:
                success = True
            self._prev_dist = self._dist_to_target(p_new)  # reset shaping baseline
        elif not collision:
            cur_dist = self._dist_to_target(p_new)
            progress = self._prev_dist - cur_dist
            self._prev_dist = cur_dist

        # --- out of bounds -----------------------------------------------------
        oob = bool(np.linalg.norm(p_new - self._scene_centroid()) > self.cfg.bounds_radius)

        # --- update attitude / heading for the NEXT obs -----------------------
        self._yaw = self._yaw_to_target(p_new)
        self._R_world_body = self._attitude(phys, self._yaw)
        tilt = float(math.acos(float(np.clip(self._R_world_body[:, 2] @ UP, -1.0, 1.0))))

        # --- reward (delegated; weights from yaml, spec 7.2) ------------------
        reward_state = {"tilt": tilt, "speed": float(np.linalg.norm(phys.vel))}
        reward_info = {
            "passed_window": passed_window,
            "collision": collision,
            "success": success,
            "progress": progress,
            "control_effort": phys.control_effort,
        }
        total, terms = reward_mod.compute_reward(
            reward_state, action, reward_info, self.reward_cfg
        )

        # --- termination / truncation -----------------------------------------
        self._steps += 1
        terminated = bool(success or collision or oob)
        truncated = bool(self._steps >= self.cfg.max_steps) and not terminated

        self._prev_action = action
        self._last_phys = phys
        obs = self._build_obs(phys)

        info: Dict[str, Any] = {
            "reward_terms": terms,
            "passed_window": passed_window,
            "collision": collision,
            "success": success,
            "out_of_bounds": oob,
            "progress": progress,
            "target_idx": self._target_idx,
            "n_passed": self._target_idx,
            "tilt": tilt,
            "control_effort": phys.control_effort,
            "pos": p_new.copy(),
            "steps": self._steps,
        }
        return obs, float(total), terminated, truncated, info

    def close(self) -> None:  # pragma: no cover - trivial
        pass
