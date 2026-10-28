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
    after crossing a window: planner.mark_passed()   (or let it detect the crossing itself: auto_pass=True, the
                             default, counts a pass when the drone crosses the estimated plane of the window it flies to)

Look-around policies (RESULTS.md §12; policy_meta action.yaw, mission.selection nearest) also steer the heading:
``out["yaw"]`` is the heading set-point [rad, world] (the waypoints_config schema has no yaw — the controller needs
it as a separate input), and the target is the nearest window seen from the last pass (core/selector.py), so the
vision message's ``order_index`` is only a track id there, not the course order.

Slung load (RESULTS.md §14): with a load on a cable — a policy trained with one (policy_meta ``slung.enabled``), or
``slung=<the rig's slung section, configs/slung/base.yaml>`` for a policy trained without — drone_state also carries
the cable sensor, ``drone_state["load"] = {"offset": [x, y, z], "vel": [vx, vy, vz]}``: the load's position from the
hook and its velocity minus the drone's, both in body axes [m, m/s]. From a 2-axis cable-angle sensor at the hook
(taut cable): offset = L dir_b, vel = d(offset)/dt + ang_vel x (attach + offset). The convex layer
(planners/convex_layer.py, the class the training env and ``+layer`` evaluation run) projects every action last
(``layer=False``: off), and ``done`` waits until the load is through the last window as well.
"""
from __future__ import annotations

import json
import os
import sys
from typing import Mapping, Optional

import torch

from ..core import math3d as m3
from ..core.config import Cfg, deep_merge, default_section
from ..core.dynamics import RigidBodyState
from ..core.observations import build_spec, compute_group, _gather
from ..core.perception import PerceptionSurrogate
from ..core.scene import window_frame
from ..core.selector import TargetSelector
from ..core.slung import SlungLoad
from .convex_layer import SlungLayerFilter
from .policy import PolicyPlanner
from .safety import SafetyFilter

_RB = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # reinforcement_yunho/
_PLANNER_LIMITS = os.path.join(os.path.dirname(_RB), "overall_gilnam", "planning", "planner_limits.yaml")


class _Mission:
    def __init__(self, n_windows: int, max_windows: int, device):
        self.target = torch.zeros(1, dtype=torch.long, device=device)      # windows passed so far
        self.passed = torch.zeros(1, max_windows, dtype=torch.bool, device=device)
        self.n, self.W = n_windows, max_windows

    def target_index(self, W=None):
        return self.target.clamp_max((W or self.W) - 1)


class _Scene:
    def __init__(self, n_windows: int, max_windows: int, device):
        self.n_windows = torch.tensor([n_windows], device=device)
        self.valid = torch.arange(max_windows, device=device)[None] < n_windows


class OnboardPlanner:
    PASS_TOL = 0.2                       # [m] slack on the estimated opening for the planner's own pass check
    def __init__(self, export_dir: str, n_windows: int, device: str = "cpu", limits: Optional[dict] = None,
                 safety: Optional[Mapping] = None, auto_pass: bool = True, slung: Optional[Mapping] = None,
                 layer: Optional[bool] = None):
        self.device = torch.device(device)
        self.auto_pass = bool(auto_pass)
        self.policy = PolicyPlanner(export_dir, device=device)
        self.cfg = Cfg(self.policy.meta["config"])
        self.cfg["perception"]["frame_every"] = 1
        if slung is not None:                    # the rig on board (cable, load, layer settings)
            self.cfg["slung"] = deep_merge({**(self.cfg.get("slung") or {}), "enabled": True}, slung)
        self.E, self.W = 1, int(self.cfg["scene"]["max_windows"])
        if n_windows > self.W:
            raise ValueError(f"policy was trained for at most {self.W} windows")
        self.perception = PerceptionSurrogate(self.cfg, 1, self.W, self.device)
        self.perception.reset(torch.tensor([0], device=self.device), sigma_px=torch.tensor([0.0], device=self.device),
                              miss_prob=torch.tensor([0.0], device=self.device))
        self.mission = _Mission(n_windows, self.W, self.device)
        self.scene = _Scene(n_windows, self.W, self.device)
        self.selector = TargetSelector(self.cfg, 1, self.W, self.device)
        self.yaw_rate = float(torch.deg2rad(torch.tensor(float(self.cfg["action"].get("yaw_rate_max_deg", 60.0)))))
        self.yaw_cmd: Optional[torch.Tensor] = None
        self.state = RigidBodyState(1, self.device)
        a = self.cfg["action"]
        self.action_dim = 4 if a["yaw"] else 3
        self.step_max = torch.tensor(a["step_max"], device=self.device)
        self.actions = torch.zeros(1, self.action_dim, device=self.device)
        self.prev_actions = torch.zeros_like(self.actions)
        self.step_dt = float(self.policy.meta["timing"]["decision_dt_s"])
        self.cfg["safety"] = {**default_section("safety"), **(safety or {})}   # runtime setting, not the training snapshot
        self.safety = SafetyFilter(self.cfg, 1, self.device, self.step_dt)
        self.slung = SlungLoad(self.cfg, 1, self.device) if (self.cfg.get("slung") or {}).get("enabled") else None
        use_layer = self.slung is not None and (layer is None or bool(layer))
        self.layer = SlungLayerFilter(self.cfg, 1, self.device, self.step_dt) if use_layer else None
        self._last_passed: Optional[torch.Tensor] = None
        self._seen = False                       # any window detected since the last step()
        self._framed = False                     # a camera frame arrived since the last step()
        self.episode_length_s = float(self.cfg["sim"]["episode_length_s"])
        self.episode_len_s = torch.tensor([self.episode_length_s + float(self.cfg["sim"].get("episode_s_per_window") or 0.0)
                                           * n_windows], device=self.device)
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
        self._prev_pos: Optional[torch.Tensor] = None
        self._prev_k = -1
        self._pass_pending: Optional[torch.Tensor] = None

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
        """The drone flew through the window it was flying to (the selector's choice). The look-around restarts from
        the drone state of the next step() (in simulation: the state right after the decision with the pass)."""
        k = self.selector.sel.clone()
        self._last_passed = k
        self.mission.passed[0, k] = True
        self.mission.target += 1
        self._prev_d = None
        self._pass_pending = k

    def _restart_look_around(self) -> None:
        k = self._pass_pending
        self._pass_pending = None
        t = torch.tensor([self.t_now], device=self.device)
        self.selector.on_pass(torch.ones(1, dtype=torch.bool, device=self.device), self.state.pos, self.state.vel_w,
                              self.yaw, t, _gather(self.est.center, k), _gather(self.est.valid, k), _gather(self.est.normal, k))

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
        if self.slung is not None:                                            # cable sensor, body axes
            if "load" not in drone_state:
                raise ValueError("slung load on board: drone_state needs 'load' {offset, vel} (cable sensor)")
            ld = drone_state["load"]
            off = torch.tensor([ld["offset"]], dtype=torch.float32, device=self.device)
            dv = torch.tensor([ld.get("vel", [0.0, 0.0, 0.0])], dtype=torch.float32, device=self.device)
            hook, _ = self.slung.hook(s.pos, s.quat)
            self.slung.pos = hook + m3.quat_rotate(s.quat, off)
            self.slung.vel = s.vel_w + m3.quat_rotate(s.quat, dv)

    def _load_clear(self) -> bool:
        """The load is through the window the drone passed last (its estimated plane)."""
        k = self._last_passed
        if k is None or not bool(_gather(self.est.valid, k)[0]):
            return True
        d = ((self.slung.pos - _gather(self.est.center, k)) * _gather(self.est.normal, k)).sum()
        return bool(d < -self.slung.radius)

    def _auto_pass(self) -> None:
        """Advance the target when the drone crosses the ESTIMATED window plane inside the opening (estimated size
        + PASS_TOL; a crossing beside the window — around its panel — is not a pass)."""
        k = self.selector.sel
        if int(k[0]) != self._prev_k:            # another window chosen: its plane has no history yet
            self._prev_k, self._prev_d = int(k[0]), None
        if not bool(_gather(self.est.valid, k)[0]) or not bool(self.selector.has[0]):
            return
        c, n = _gather(self.est.center, k), _gather(self.est.normal, k)
        pos = self.state.pos
        d = float(((pos - c) * n).sum())
        if self._prev_d is not None and self._prev_d > 0 >= d:
            f = self._prev_d / max(self._prev_d - d, 1e-9)
            x = self._prev_pos + f * (pos - self._prev_pos) - c          # crossing point, in the plane
            right, up = window_frame(n)
            half = _gather(self.est.size, k)[0] / 2 + self.PASS_TOL
            if abs(float((x * right).sum())) <= float(half[0]) and abs(float((x * up).sum())) <= float(half[1]):
                self.mark_passed()
                return
        self._prev_d = d
        self._prev_pos = pos.clone()

    @torch.no_grad()
    def step(self, drone_state: Mapping, t_s: Optional[float] = None) -> dict:
        if t_s is not None:
            self.t0 = t_s if self.t0 is None else self.t0
            self.t_now = t_s - self.t0
        self._set_state(drone_state)
        if self.yaw_cmd is None:                 # first decision: the look-around starts here
            self.yaw_cmd = self.yaw.clone()
            self.selector.reset(torch.zeros(1, dtype=torch.long, device=self.device), self.state.pos, self.yaw,
                                torch.tensor([self.t_now], device=self.device))
        if self._pass_pending is not None:       # mark_passed() since the last step
            self._restart_look_around()
        self.selector.select(self.scene, self.mission, self.est, self.perception)   # passes reported since the last step
        if self.auto_pass:
            self._auto_pass()
        if self._pass_pending is not None:       # crossing detected just now from this state
            self._restart_look_around()
            self.selector.select(self.scene, self.mission, self.est, self.perception)
        if self.det is None:              # no frame yet: empty detection
            self.add_frame(drone_state["position"], drone_state["orientation"], None)
        elif not self._framed:            # no camera frame since the last decision: nothing detected now
            cam_c, R_WC = self.perception.camera_pose(self.state.pos, self.state.quat)
            no = torch.zeros(1, self.W, dtype=torch.bool, device=self.device)
            self.det = self.perception.add_detections(cam_c, R_WC, torch.zeros(1, self.W, 4, 2, device=self.device),
                                                      no, torch.zeros(1, self.W, device=self.device), no[:, 0])
        self._framed = False
        self.selector.scan(self.yaw, self.state.pos[:, 2], torch.tensor([self.t_now], device=self.device))
        self.selector.select(self.scene, self.mission, self.est, self.perception)
        obs = compute_group(self, self.policy_spec)
        self.last_obs = obs                      # exactly what the policy saw (debug / logging)
        raw = self.policy.act_obs(obs).clamp(-1.0, 1.0)
        seen = torch.tensor([self._seen], device=self.device)
        self._seen = False
        sel = self.selector.sel
        act = self.safety.filter(raw, seen, _gather(self.est.valid, sel) & self.selector.has, self.state.pos, self.yaw,
                                 self.step_max, _gather(self.est.sigma, sel), self.state.vel_w, self.target_idx)
        pre_layer = act
        if self.layer is not None:               # convex layer last, as in the env loop and "+layer"
            act = self.layer.filter(act, self)
        if self.action_dim == 4:                 # heading set-point, integrated like the training env
            self.yaw_cmd = self.yaw_cmd + act[:, 3] * self.yaw_rate * self.step_dt
        self.prev_actions, self.actions = self.actions, act
        self.perception.tick()                   # same decision clock as simulation (detection_gap)
        wp = self.state.pos + m3.heading_to_world(act[:, :3] * self.step_max, self.yaw)
        wp[:, 2] = wp[:, 2].clamp(*self.wp_z)
        cur = [float(v) for v in self.state.pos[0]]
        nxt = [float(v) for v in wp[0]]
        wc = {"waypoints": [cur, nxt], "limits": self.limits, "dt": 0.01}
        done = int(self.mission.target[0]) >= self.mission.n
        if done and self.slung is not None:      # ... and the load through the last window as well
            done = self._load_clear()
        out = {"waypoint": nxt, "action": [float(v) for v in act[0]], "policy_action": [float(v) for v in raw[0]],
               "hold": bool(self.safety.holding[0]), "target_index": int(self.mission.target[0]),
               "done": done, "waypoints_config": wc,
               "estimate_valid": bool(_gather(self.est.valid, sel)[0] & self.selector.has[0]),
               "yaw": float(self.yaw_cmd[0]), "selected_window": int(sel[0]) if bool(self.selector.has[0]) else -1}
        if self.layer is not None:
            out["pre_layer_action"] = [float(v) for v in pre_layer[0]]
            out["layer_changed"] = bool(self.layer.changed[0])
        return out

    def validate_waypoints_config(self, wc: dict) -> None:
        """Validate against 성진's schema (reinforcement_yunho/interface)."""
        if _RB not in sys.path:
            sys.path.insert(0, _RB)
        from interface.schemas import WaypointsConfig
        WaypointsConfig.from_dict(wc).validate()


def load_meta(export_dir: str) -> dict:
    with open(os.path.join(export_dir, "policy_meta.json"), encoding="utf-8") as f:
        return json.load(f)
