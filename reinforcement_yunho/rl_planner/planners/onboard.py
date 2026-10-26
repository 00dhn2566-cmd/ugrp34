"""Onboard / integration adapter: run an exported policy on the team's real messages.

This is the drop-in replacement for the mission state machine
(``backup/prototype_demo/traj_manager.py``) in the live pipeline:

    vision §5 detections ──┐                      ┌──> next waypoint (world)
    VIO pose (§6.1, XYZW) ─┼─> OnboardPlanner ────┤
    own pass count ────────┘   (triangulator +    └──> waypoints_config (성진 schema:
                                policy)                 {waypoints:[current, next], limits, dt})

* The triangulator is the SAME code the policy was trained with
  (PerceptionSurrogate.add_detections), so the estimates it sees online are built
  exactly like in training — no separate re-implementation that can drift.
* Observation layout, action scaling and timing come from policy_meta.json.
* A SafetyFilter (planners/safety.py) sits between policy and output: hold position on
  perception loss, clamp waypoints into the arena box. Set the real room with
  ``safety={"bounds": {"x_min": .., "y_abs": .., "z_min": .., "z_max": ..}}`` (VIO frame).
* Interface conventions (CONVENTIONS.md): drone state quaternion is XYZW (state_window
  §6.1 / OpenVINS); window corners in CORNER_ORDER (TL, TR, BR, BL); pixels are
  1280x720 §2 coordinates; camera = body +x (root spec §6.1 R_IC).

Minimal use (one window mission, 10 Hz):

    planner = OnboardPlanner("<run>/export", n_windows=3)
    for each camera frame:   planner.add_frame(p_WI, q_WI_xyzw, vision_msg)      # §5 dict
    every 0.1 s:             out = planner.step(drone_state)                     # §6.1 dict
                             send out["waypoints_config"] to the trajectory/PID stack
    after crossing a window: planner.mark_passed()   (or let it detect the crossing itself)
"""
from __future__ import annotations

import json
import os
import sys
from typing import Mapping, Optional

import torch

from ..core import math3d as m3
from ..core.config import Cfg, default_section
from ..core.dynamics import RigidBodyState
from ..core.observations import build_spec, compute_group, _gather
from ..core.perception import PerceptionSurrogate
from .policy import PolicyPlanner
from .safety import SafetyFilter

_RB = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # reinforcement_yunho/
_PLANNER_LIMITS = os.path.join(os.path.dirname(_RB), "overall_gilnam", "planning", "planner_limits.yaml")


class _Mission:
    def __init__(self, n_windows: int, device):
        self.target = torch.zeros(1, dtype=torch.long, device=device)
        self.n = n_windows

    def target_index(self, W):
        return self.target.clamp_max(W - 1)


class _Scene:
    def __init__(self, n_windows: int, device):
        self.n_windows = torch.tensor([n_windows], device=device)


class OnboardPlanner:
    def __init__(self, export_dir: str, n_windows: int, device: str = "cpu", limits: Optional[dict] = None,
                 safety: Optional[Mapping] = None):
        self.device = torch.device(device)
        self.policy = PolicyPlanner(export_dir, device=device)
        self.cfg = Cfg(self.policy.meta["config"])
        self.cfg["perception"]["frame_every"] = 1
        self.E, self.W = 1, int(self.cfg["scene"]["max_windows"])
        if n_windows > self.W:
            raise ValueError(f"policy was trained for at most {self.W} windows")
        self.perception = PerceptionSurrogate(self.cfg, 1, self.W, self.device)
        self.perception.reset(torch.tensor([0], device=self.device), sigma_px=torch.tensor([0.0], device=self.device),
                              miss_prob=torch.tensor([0.0], device=self.device))
        self.mission = _Mission(n_windows, self.device)
        self.scene = _Scene(n_windows, self.device)
        self.state = RigidBodyState(1, self.device)
        a = self.cfg["action"]
        self.action_dim = 4 if a["yaw"] else 3
        self.step_max = torch.tensor(a["step_max"], device=self.device)
        self.actions = torch.zeros(1, self.action_dim, device=self.device)
        self.prev_actions = torch.zeros_like(self.actions)
        self.step_dt = float(self.policy.meta["timing"]["decision_dt_s"])
        self.cfg["safety"] = {**default_section("safety"), **(safety or {})}   # runtime setting, not the training snapshot
        self.safety = SafetyFilter(self.cfg, 1, self.device, self.step_dt)
        self._seen = False                       # any window detected since the last step()
        self._framed = False                     # a camera frame arrived since the last step()
        self.episode_length_s = float(self.cfg["sim"]["episode_length_s"])
        self.t0: Optional[float] = None
        self.t_now = 0.0
        self.obs_noise_enabled = False
        self.obs_gen = torch.Generator(device=self.device)   # never drawn: obs noise is off on board
        self.policy_spec = build_spec(self.cfg, "policy")
        b = self.cfg["scene"]["bounds"]
        self.wp_z = (float(b["z_min"]) + 0.25, float(b["z_max"]) - 0.3)
        self.limits = limits or self._load_limits()
        self._take_frame = torch.zeros(1, dtype=torch.bool, device=self.device)
        self.det = None
        self.est = self.perception.estimate()
        self.yaw = torch.zeros(1, device=self.device)
        self.body_x_w = torch.tensor([[1.0, 0.0, 0.0]], device=self.device)
        self._prev_d: Optional[float] = None

    # --- attributes the observation terms read (same names as WindowMissionCore) ---
    @property
    def target_idx(self):
        return self.mission.target_index(self.W)

    @property
    def episode_time(self):
        return torch.tensor([self.t_now], device=self.device)

    @staticmethod
    def _load_limits() -> dict:
        try:
            import yaml
            with open(_PLANNER_LIMITS, encoding="utf-8") as f:
                return dict(yaml.safe_load(f)["limits"])
        except Exception:
            return {"v_max": 1.6, "a_max": 1.6, "j_max": 8.0, "snap_max": 40.0}

    # ------------------------------------------------------------------ inputs
    def add_frame(self, p_WI, q_WI_xyzw, vision_msg: Optional[Mapping]) -> None:
        """One camera frame. vision_msg = §5 dict ({"windows": [{order_index, corners, corner_vis,
        det_conf}, ...]}) or None when nothing was detected."""
        p = torch.tensor([p_WI], dtype=torch.float32, device=self.device)
        x, y, z, w = (float(v) for v in q_WI_xyzw)
        q = torch.tensor([[w, x, y, z]], device=self.device)
        cam_c, R_WC = self.perception.camera_pose(p, q)
        corners = torch.zeros(1, self.W, 4, 2, device=self.device)
        det = torch.zeros(1, self.W, dtype=torch.bool, device=self.device)
        conf = torch.zeros(1, self.W, device=self.device)
        thr = float(self.cfg["perception"]["det_conf_min"])
        for win in (vision_msg or {}).get("windows", []):
            k = int(win["order_index"])
            if k >= self.W or float(win.get("det_conf", 0.0)) < thr:
                continue
            if any(int(v) != 1 for v in win.get("corner_vis", [1, 1, 1, 1])):
                continue                      # training used fully visible windows only (policy A)
            corners[0, k] = torch.tensor(win["corners"], dtype=torch.float32, device=self.device)
            det[0, k] = True
            conf[0, k] = float(win["det_conf"])
        self.det = self.perception.add_detections(cam_c, R_WC, corners, det, conf,
                                                  torch.ones(1, dtype=torch.bool, device=self.device))
        self.est = self.perception.estimate()
        self._seen = self._seen or bool(det.any())
        self._framed = True

    def mark_passed(self) -> None:
        self.mission.target += 1
        self._prev_d = None

    # ------------------------------------------------------------------ decision
    def _set_state(self, drone_state: Mapping) -> None:
        s = self.state
        s.pos = torch.tensor([drone_state["position"]], dtype=torch.float32, device=self.device)
        x, y, z, w = (float(v) for v in drone_state["orientation"])          # XYZW (§6.1)
        s.quat = torch.tensor([[w, x, y, z]], device=self.device)
        v_b = torch.tensor([drone_state.get("lin_vel", [0.0, 0.0, 0.0])], dtype=torch.float32, device=self.device)
        s.vel_w = m3.quat_rotate(s.quat, v_b)                                 # §6.1 lin_vel is BODY
        s.omega_b = torch.tensor([drone_state.get("ang_vel", [0.0, 0.0, 0.0])], dtype=torch.float32, device=self.device)
        self.yaw = m3.yaw_from_quat(s.quat)
        self.body_x_w = m3.quat_to_rotmat(s.quat)[..., :, 0]

    def _auto_pass(self) -> None:
        """Advance the target when the drone crosses the ESTIMATED window plane inside the opening."""
        k = self.target_idx
        if not bool(_gather(self.est.valid, k)[0]):
            return
        c, n = _gather(self.est.center, k), _gather(self.est.normal, k)
        d = float(((self.state.pos - c) * n).sum())
        if self._prev_d is not None and self._prev_d > 0 >= d:
            self.mark_passed()
            return
        self._prev_d = d

    @torch.no_grad()
    def step(self, drone_state: Mapping, t_s: Optional[float] = None) -> dict:
        if t_s is not None:
            self.t0 = t_s if self.t0 is None else self.t0
            self.t_now = t_s - self.t0
        self._set_state(drone_state)
        self._auto_pass()
        if self.det is None:              # no frame yet: empty detection
            self.add_frame(drone_state["position"], drone_state["orientation"], None)
        elif not self._framed:            # no camera frame since the last decision: nothing detected now
            cam_c, R_WC = self.perception.camera_pose(self.state.pos, self.state.quat)
            no = torch.zeros(1, self.W, dtype=torch.bool, device=self.device)
            self.det = self.perception.add_detections(cam_c, R_WC, torch.zeros(1, self.W, 4, 2, device=self.device),
                                                      no, torch.zeros(1, self.W, device=self.device), no[:, 0])
        self._framed = False
        obs = compute_group(self, self.policy_spec)
        self.last_obs = obs                      # exactly what the policy saw (debug / logging)
        raw = self.policy.act_obs(obs).clamp(-1.0, 1.0)
        seen = torch.tensor([self._seen], device=self.device)
        self._seen = False
        act = self.safety.filter(raw, seen, _gather(self.est.valid, self.target_idx), self.state.pos, self.yaw,
                                 self.step_max, _gather(self.est.sigma, self.target_idx))
        self.prev_actions, self.actions = self.actions, act
        self.perception.tick()                   # same decision clock as simulation (detection_gap)
        wp = self.state.pos + m3.heading_to_world(act[:, :3] * self.step_max, self.yaw)
        wp[:, 2] = wp[:, 2].clamp(*self.wp_z)
        cur = [float(v) for v in self.state.pos[0]]
        nxt = [float(v) for v in wp[0]]
        wc = {"waypoints": [cur, nxt], "limits": self.limits, "dt": 0.01}
        return {"waypoint": nxt, "action": [float(v) for v in act[0]], "policy_action": [float(v) for v in raw[0]],
                "hold": bool(self.safety.holding[0]), "target_index": int(self.mission.target[0]),
                "done": int(self.mission.target[0]) >= self.mission.n, "waypoints_config": wc,
                "estimate_valid": bool(_gather(self.est.valid, self.target_idx)[0])}

    def validate_waypoints_config(self, wc: dict) -> None:
        """Validate against 성진's schema (reinforcement_yunho/interface)."""
        if _RB not in sys.path:
            sys.path.insert(0, _RB)
        from interface.schemas import WaypointsConfig
        WaypointsConfig.from_dict(wc).validate()


def load_meta(export_dir: str) -> dict:
    with open(os.path.join(export_dir, "policy_meta.json"), encoding="utf-8") as f:
        return json.load(f)
