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

import numpy as np

try:
    import pybullet as p
    from gymnasium import spaces
    from gym_pybullet_drones.envs.BaseRLAviary import BaseRLAviary
    from gym_pybullet_drones.utils.enums import DroneModel, Physics, ActionType, ObservationType
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
                 opening: float = 0.35, step: float = 0.6):
        if not _HAS_GPD:
            raise ImportError(f"gym-pybullet-drones not available: {_IMPORT_ERR}")
        self.N_WINDOWS = int(n_windows)
        self.STEP = float(step)       # max waypoint nudge per env step (m)
        self.OPENING = (float(opening), float(opening))   # (w,h) opening; cf2x is ~0.09 m
        self.SPACING = 1.2        # nominal x-gap between windows
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
        for i in range(self.N_WINDOWS):
            wx = (i + 1) * self.SPACING + rng.uniform(-0.15, 0.15)
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
        """Build each window as 4 thin collision bars around an opening (real crash geometry)."""
        self._window_bodies = []
        t, d = 0.03, 0.05  # bar thickness, depth(x)
        for w in self.window_layout:
            cx, cy, cz = w["center"]; ow, oh = w["ow"], w["oh"]; rgba = self._rgba(w["color"])
            bars = [  # (center, halfextents)
                ([cx, cy, cz + oh / 2 + t / 2], [d / 2, ow / 2 + t, t / 2]),   # top
                ([cx, cy, cz - oh / 2 - t / 2], [d / 2, ow / 2 + t, t / 2]),   # bottom
                ([cx, cy - ow / 2 - t / 2, cz], [d / 2, t / 2, oh / 2]),        # left
                ([cx, cy + ow / 2 + t / 2, cz], [d / 2, t / 2, oh / 2]),        # right
            ]
            ids = []
            for pos, he in bars:
                col = p.createCollisionShape(p.GEOM_BOX, halfExtents=he, physicsClientId=self.CLIENT)
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=rgba, physicsClientId=self.CLIENT)
                bid = p.createMultiBody(0, col, vis, pos, physicsClientId=self.CLIENT)  # mass 0 = static
                ids.append(bid)
            self._window_bodies.append(ids)

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
