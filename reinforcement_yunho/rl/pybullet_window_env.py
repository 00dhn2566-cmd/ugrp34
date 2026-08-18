"""Window-traversal RL environment on REAL quadrotor physics (PyBullet).

Isaac Sim's physics is blocked on this cluster by a driver bug (see
sim/ISAAC_CLUSTER_NOTES.md), and MockPhysics (rl/window_env.py) is only a point-mass
stand-in. This env uses `gym-pybullet-drones` — a full 6-DOF Crazyflie-2.x model
(mass/inertia/rotor thrust+torque from cf2x.urdf) with the battle-tested DSL PID as
the low-level controller — exactly the project architecture:

    RL policy  ->  waypoint (action)  ->  DSL-PID  ->  motor RPMs  ->  PyBullet physics

The drone must fly THROUGH coloured window openings IN ORDER. Window frames are real
collision geometry: clipping a frame bar = crash. This is the honest RL env until
Isaac Sim is unblocked (its PhysX would swap in behind the same waypoint interface).

Requires: gym-pybullet-drones, pybullet, gymnasium (see requirements.txt).
Train example (stable-baselines3): see rl/train_pybullet.py.
"""
from __future__ import annotations

import os

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))

try:
    import pybullet as p
    from gymnasium import spaces
    from gym_pybullet_drones.envs.BaseRLAviary import BaseRLAviary
    from gym_pybullet_drones.utils.enums import DroneModel, Physics, ActionType, ObservationType
    from rl import domain
    _HAS_GPD = True
except Exception as _e:  # pragma: no cover
    _HAS_GPD = False
    _IMPORT_ERR = _e
    BaseRLAviary = object  # so the class body parses without the dep


class WindowTraversalAviary(BaseRLAviary):
    """Fly a quadrotor through N coloured window openings, in order, on real physics.

    action  : ActionType.PID — a 3-vector in [-1,1]³ read as a body-relative nudge
              (scaled by STEP metres) added to the current position; DSL-PID flies there.
    obs     : KIN drone state (+ action buffer) ++ next-window relative position (3).
    reward  : progress toward the next window + pass bonus - collision - tilt/energy.
    """

    def __init__(self, n_windows: int = 3, gui: bool = False, record: bool = False,
                 ctrl_freq: int = 30, pyb_freq: int = 240, seed: int | None = None,
                 opening: float = 0.35, step: float = 0.6, pane: bool = True,
                 domain_match: bool = True, tex_dir: str | None = None,
                 clutter: int = 0, walls: bool = False,
                 spacing: float = 1.2, spacing_jitter: float = 0.15,
                 min_gap: bool = False):
        if not _HAS_GPD:
            raise ImportError(f"gym-pybullet-drones not available: {_IMPORT_ERR}")
        # Appearance switches (see rl/domain.py). None touches physics — every body
        # they add is visual-only — so a policy trained before them is unaffected.
        # Set all three off to get the original flat-shaded outline scene back.
        self.PANE = bool(pane)                    # fill the opening (training look)
        self.DOMAIN_MATCH = bool(domain_match)    # textured room instead of sky+checkerboard
        self.CLUTTER = int(clutter)               # n visual-only props (0 = none)
        # walls=True puts each window in a wall instead of hanging it in mid-air.
        # This DOES change physics (the slabs collide), which is the point: the
        # drone must fly through the hole, and each image shows one opening.
        self.WALLS = bool(walls)
        self.TEX_DIR = tex_dir or os.path.join(_HERE, "_textures")
        self.N_WINDOWS = int(n_windows)
        self.STEP = float(step)       # max waypoint nudge per env step (m)
        self.OPENING = (float(opening), float(opening))   # (w,h) opening; cf2x is ~0.09 m
        # Nominal x-gap between windows. 1.2 is the value the shipped PPO policy was
        # trained on — do not change the default or that policy's scene shifts under it.
        # The planner demo passes spacing>=2.0: with d_exit=1.0 and d_app=1.5 a gap
        # below 2.5 m makes consecutive gates overlap, i.e. the drone has to fly
        # backwards between windows.
        self.SPACING = float(spacing)
        self.SPACING_JITTER = float(spacing_jitter)
        # min_gap=False (기본, legacy): x_i = (i+1)*SPACING + U(-J, J) → 실제 간격이
        #   SPACING - 2J 까지 좁아진다. 학습된 정책이 본 분포라 기본값은 유지한다.
        # min_gap=True: 간격을 누적으로 뽑아 **최소 간격 = SPACING** 을 보장한다.
        self.MIN_GAP = bool(min_gap)
        self.EPISODE_LEN_SEC = 12
        self.WS = np.array([[-0.6, self.N_WINDOWS * self.SPACING + 1.2],  # x
                            [-1.2, 1.2],                                   # y
                            [0.2, 2.2]])                                   # z
        self._rng = np.random.default_rng(seed)
        self.window_layout = self._sample_layout()     # before super().__init__ (used by _addObstacles)
        self._next_idx = 0
        self._prev_dist = None
        self._window_bodies: list[list[int]] = []
        super().__init__(drone_model=DroneModel.CF2X, num_drones=1,
                         initial_xyzs=np.array([[0.0, 0.0, 1.0]]),
                         physics=Physics.PYB, pyb_freq=pyb_freq, ctrl_freq=ctrl_freq,
                         gui=gui, record=record,
                         obs=ObservationType.KIN, act=ActionType.PID)

    # ---- episode layout -----------------------------------------------------
    def _sample_layout(self):
        """N windows marching in +x with randomised y,z, opening size, and colour order."""
        rng = self._rng
        wins = []
        colours = ["red", "green", "blue"]
        prev_x = 0.0
        for i in range(self.N_WINDOWS):
            if self.MIN_GAP:
                wx = prev_x + self.SPACING + rng.uniform(0.0, 2 * self.SPACING_JITTER)
                prev_x = wx
            else:
                wx = (i + 1) * self.SPACING + rng.uniform(-self.SPACING_JITTER,
                                                          self.SPACING_JITTER)
            wy = rng.uniform(-0.6, 0.6)
            wz = rng.uniform(0.8, 1.6)
            ow = self.OPENING[0] + rng.uniform(-0.05, 0.1)
            oh = self.OPENING[1] + rng.uniform(-0.05, 0.1)
            wins.append({"center": np.array([wx, wy, wz]), "ow": ow, "oh": oh,
                         "color": colours[i % 3], "order_index": i})
        return wins

    def _rgba(self, color):
        return {"red": [1, .1, .1, 1], "green": [.1, .8, .15, 1], "blue": [.1, .2, 1, 1]}[color]

    def _addObstacles(self):
        """Build the scene.

        Geometry (the 4 collision bars per window) is unchanged; appearance is
        delegated to :mod:`rl.domain`, which mirrors the renderer the detector was
        trained on. Only the bars are registered in ``_window_bodies`` — every
        appearance body is visual-only, so physics and any policy trained before
        this change behave identically.
        """
        self._window_bodies = []
        rng = np.random.default_rng(0xD07A1 ^ int(self.N_WINDOWS))

        if self.DOMAIN_MATCH:
            domain.hide_default_plane(p, self.CLIENT, self.PLANE_ID)
            xs = [w["center"][0] for w in self.window_layout]
            bounds = ((min(xs) - 4.0, max(xs) + 4.0), (-4.0, 4.0), (-0.05, 5.0))
            domain.build_room(p, self.CLIENT, bounds=bounds,
                              tex_dir=self.TEX_DIR, seed=0)
            if self.CLUTTER:
                # Visual-only props: VIO parallax (박태민 07/03) + closer to the
                # training renderer's cluttered background. Windows and the flight
                # corridor are kept clear.
                keep = [w["center"] for w in self.window_layout] + \
                       [[x, 0.0, 1.0] for x in np.linspace(bounds[0][0], bounds[0][1], 8)]
                domain.build_clutter(p, self.CLIENT, bounds=bounds, tex_dir=self.TEX_DIR,
                                     n=self.CLUTTER, seed=7, keep_clear=keep)

        for w in self.window_layout:
            bars, _visual = domain.build_window(
                p, self.CLIENT, w["center"], w["ow"], w["oh"], w["color"],
                rng=rng, pane=self.PANE)
            if self.WALLS:
                slabs, _ = domain.build_wall(
                    p, self.CLIENT, w["center"], w["ow"], w["oh"],
                    tex_dir=self.TEX_DIR, seed=int(w["order_index"]))
                bars = bars + slabs        # hitting the wall is a crash too
            self._window_bodies.append(bars)

    # ---- gym API ------------------------------------------------------------
    def reset(self, seed=None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self.window_layout = self._sample_layout()
        self._next_idx = 0
        self._prev_dist = None
        obs, info = super().reset(seed=seed, options=options)
        if len(self.ctrl):
            self.ctrl[0].reset()
        return obs, info

    def _next_window(self):
        if self._next_idx >= self.N_WINDOWS:
            return self.window_layout[-1]
        return self.window_layout[self._next_idx]

    # action ∈[-1,1]³ -> body-relative waypoint nudge -> DSL-PID -> RPMs
    def _preprocessAction(self, action):
        state = self._getDroneStateVector(0)
        cur = state[0:3]
        target = np.clip(cur + np.clip(action[0], -1, 1) * self.STEP,
                         self.WS[:, 0], self.WS[:, 1])
        rpm, _, _ = self.ctrl[0].computeControlFromState(
            control_timestep=self.CTRL_TIMESTEP, state=state, target_pos=target)
        self.action_buffer.append(np.clip(action, -1, 1))
        return rpm.reshape(1, 4)

    def _observationSpace(self):
        base = super()._observationSpace()             # Box (1, 12 + buffer*3)
        lo = np.hstack([base.low, np.full((1, 3), -np.inf)])
        hi = np.hstack([base.high, np.full((1, 3), np.inf)])
        return spaces.Box(low=lo, high=hi, dtype=np.float32)

    def _computeObs(self):
        base = super()._computeObs()                   # (1, 12 + buffer*3)
        pos = self._getDroneStateVector(0)[0:3]
        rel = (self._next_window()["center"] - pos).astype(np.float32)
        return np.hstack([base, rel.reshape(1, 3)]).astype(np.float32)

    # ---- pass / crash bookkeeping ------------------------------------------
    def _crashed(self):
        did = self.DRONE_IDS[0]
        for ids in self._window_bodies:
            for bid in ids:
                if p.getContactPoints(did, bid, physicsClientId=self.CLIENT):
                    return True
        return False

    def _check_pass(self):
        """Advance _next_idx if the drone crossed the current window plane through the opening."""
        if self._next_idx >= self.N_WINDOWS:
            return False
        w = self.window_layout[self._next_idx]
        cx, cy, cz = w["center"]
        pos = self._getDroneStateVector(0)[0:3]
        # need previous x to detect a plane crossing
        px = getattr(self, "_prev_x", pos[0])
        crossed = (px < cx <= pos[0])
        through = (abs(pos[1] - cy) < w["ow"] / 2) and (abs(pos[2] - cz) < w["oh"] / 2)
        self._prev_x = pos[0]
        if crossed and through:
            self._next_idx += 1
            return True
        return False

    def _through_point(self, w):
        """A point 0.6 m PAST the opening (drone flies +x). Minimising distance to
        it pulls the drone THROUGH the hole, instead of the reward local-optimum of
        hovering just in front of the window centre."""
        return w["center"] + np.array([0.6, 0.0, 0.0])

    def _computeReward(self):
        pos = self._getDroneStateVector(0)[0:3]
        w = self._next_window()
        dist = float(np.linalg.norm(self._through_point(w) - pos))
        r = 0.0
        if self._prev_dist is not None:
            r += 5.0 * (self._prev_dist - dist)          # progress toward the THROUGH point
        self._prev_dist = dist
        # alignment: near the window plane, reward being centred in the opening
        if abs(pos[0] - w["center"][0]) < 0.4:
            ay = abs(pos[1] - w["center"][1]) / (w["ow"] / 2)
            az = abs(pos[2] - w["center"][2]) / (w["oh"] / 2)
            r += 0.6 * max(0.0, 1.0 - 0.5 * (ay + az))
        if self._check_pass():
            r += 20.0                                    # passed a window
            self._prev_dist = None                       # retarget the new window
        if self._next_idx >= self.N_WINDOWS:
            r += 40.0                                     # all windows cleared
        if self._crashed():
            r -= 15.0
        state = self._getDroneStateVector(0)
        r -= 0.02 * (state[7] ** 2 + state[8] ** 2)      # mild tilt penalty
        return r

    def _computeTerminated(self):
        return self._next_idx >= self.N_WINDOWS          # success

    def _computeTruncated(self):
        state = self._getDroneStateVector(0)
        pos = state[0:3]
        if self._crashed():
            return True
        if (pos[0] < self.WS[0, 0] or pos[0] > self.WS[0, 1] or
                abs(pos[1]) > self.WS[1, 1] + 0.3 or pos[2] < 0.1 or pos[2] > self.WS[2, 1] + 0.3):
            return True
        if abs(state[7]) > 0.7 or abs(state[8]) > 0.7:   # flipped
            return True
        return self.step_counter / self.PYB_FREQ > self.EPISODE_LEN_SEC

    def _computeInfo(self):
        return {"windows_passed": int(self._next_idx), "n_windows": self.N_WINDOWS}
