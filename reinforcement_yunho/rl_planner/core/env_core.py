"""Backend-agnostic window-mission MDP.

Both the torch env (envs/torch_env.py) and the Isaac Lab env
(envs/isaaclab_env.py) delegate everything except rigid-body integration to
:class:`WindowMissionCore`. Per decision step the backend calls:

    core.pre_decision(actions)                 # waypoint from the policy action
    for k in range(decimation):
        thrust, torque, f_ext = core.control() # Lee controller + motor lag
        <backend integrates one physics step, then mirrors pos/vel/quat/omega
         into core.state>
        core.after_substep()                   # pass / collision referee
    core.post_decision()                       # camera frame, estimates, shaping
    terminated, time_out = core.compute_dones()
    reward = core.compute_rewards()
    <backend resets finished envs via core.reset_envs / core.after_reset>
    obs = core.compute_observations()

so the MDP is identical whichever physics runs underneath.
"""
from __future__ import annotations

import math
from typing import Dict, Mapping, Optional

import torch

from . import math3d as m3
from .config import num_levels
from .controller import MotorLag, make_controller
from .dynamics import RigidBodyState
from .mission import CRASH_NAMES, MissionTracker
from .observations import build_spec, compute_group, _gather
from .perception import PerceptionSurrogate
from .rewards import REGISTRY as REWARDS, active_terms
from .scene import LevelTable, SceneTensors, sample_scene
from .selector import TargetSelector
from .slung import SlungLoad


class WindowMissionCore:
    def __init__(self, cfg: Mapping, num_envs: int, device, seed: int = 0):
        self.cfg = cfg
        self.E = int(num_envs)
        self.W = int(cfg["scene"]["max_windows"])
        self.device = torch.device(device)
        sim = cfg["sim"]
        self.dt = float(sim["physics_dt"])
        self.decimation = int(sim["decimation"])
        self.step_dt = self.dt * self.decimation
        self.episode_length_s = float(sim["episode_length_s"])
        # per-env time limit = episode_length_s + episode_s_per_window x windows (long courses); rsl_rl sees the largest
        self.per_window_s = float(sim.get("episode_s_per_window") or 0.0)
        self.base_steps = int(math.ceil(self.episode_length_s / self.step_dt))
        self.max_episode_length = self.base_steps + int(math.ceil(self.per_window_s * self.W / self.step_dt))
        self.g = float(sim["gravity"])

        self.gen = torch.Generator(device=self.device)        # per-step camera noise
        self.reset_gen = torch.Generator(device=self.device)  # episode draws
        self.obs_gen = torch.Generator(device=self.device)    # observation noise
        self.dist_gen = torch.Generator(device=self.device)   # per-step disturbances (gusts, frame drops, jitter, drift)
        self.reseed(seed)

        self.table = LevelTable(cfg, self.device)
        self.n_levels = num_levels(cfg)
        self.scene = SceneTensors.empty(self.E, self.W, self.device)
        self.level = torch.zeros(self.E, dtype=torch.long, device=self.device)
        self.perception = PerceptionSurrogate(cfg, self.E, self.W, self.device)
        self.mission = MissionTracker(cfg, self.E, self.W, self.device)
        self.selector = TargetSelector(cfg, self.E, self.W, self.device)
        self.ep_limit = torch.full((self.E,), self.base_steps, dtype=torch.long, device=self.device)
        self.episode_len_s = torch.full((self.E,), self.episode_length_s, device=self.device)
        self.controller = make_controller(cfg, self.E, self.device)
        self.motor = MotorLag(self.E, self.device)
        self.state = RigidBodyState(self.E, self.device)
        # a load hanging from the drone on a cable (core/slung.py): integrated here, refereed with the drone
        self.slung = SlungLoad(cfg, self.E, self.device) if (cfg.get("slung") or {}).get("enabled") else None
        self.layer = None                       # convex layer applied to every action (slung.convex_layer.in_env)
        self._wind_now = None
        self._layer_hits, self._layer_n = 0.0, 0  # share of actions the layer changed, logged per reset

        a = cfg["action"]
        self.yaw_action = bool(a["yaw"])
        self.yaw_rate = math.radians(float(a.get("yaw_rate_max_deg", 60.0)))   # [rad/s] at action +-1
        self.action_dim = 4 if self.yaw_action else 3
        self.step_max = torch.tensor(a["step_max"], dtype=torch.float32, device=self.device)
        self.actions = torch.zeros(self.E, self.action_dim, device=self.device)
        self.prev_actions = torch.zeros_like(self.actions)
        self.raw_actions = torch.zeros_like(self.actions)     # the policy's action before the [-1, 1] clip
        self.wp = torch.zeros(self.E, 3, device=self.device)
        self.yaw_cmd = torch.full((self.E,), float(cfg["controller"]["yaw_target"]), device=self.device)
        b = cfg["scene"]["bounds"]
        self.wp_z = (float(b["z_min"]) + 0.25, float(b["z_max"]) - 0.3)

        d = cfg["drone"]
        self.mass0 = float(d["mass"])
        self.inertia0 = torch.tensor(d["inertia"], dtype=torch.float32, device=self.device)
        self.mass = torch.full((self.E,), self.mass0, device=self.device)
        self.inertia = self.inertia0.expand(self.E, 3).clone()       # plant (randomised)
        self.inertia_ctrl = self.inertia0.expand(self.E, 3).clone()  # what the controller assumes
        self.thrust_max = torch.full((self.E,), float(d["thrust_to_weight"]) * self.mass0 * self.g, device=self.device)
        self.drag = torch.full((self.E,), float(d["linear_drag"]), device=self.device)
        self.wind_acc = torch.zeros(self.E, 3, device=self.device)
        self.blackout = torch.full((self.E, 2), float("inf"), device=self.device)   # per-env [t0, t1) no frames

        # episode_step is OWNED by the backend env (rsl_rl may overwrite it with random
        # initial lengths); the core only reads it. episode_time derives from it.
        self.episode_step = torch.zeros(self.E, dtype=torch.long, device=self.device)
        self.obs_noise_enabled = True
        self.dr_enabled = bool(cfg["domain_randomization"]["enabled"])
        # detection latency [decision steps] = fixed stress knob + per-episode DR. Captured frames wait
        # in a ring buffer and reach the triangulator / observations that many decisions later.
        dr, pc = cfg["domain_randomization"], cfg["perception"]
        rng = lambda k, off: tuple(float(v) for v in (dr.get(k) or off)) if self.dr_enabled else tuple(off)  # noqa: E731
        lat_dr = rng("latency_steps", (0, 0))
        self.lat_fixed = int(pc.get("latency_steps") or 0)
        self.lat_dr = (int(lat_dr[0]), int(lat_dr[1]))
        self.latency = torch.full((self.E,), self.lat_fixed + self.lat_dr[0], dtype=torch.long, device=self.device)
        # latency jitter: each decision delivers the frame captured latency + U{0..jitter} decisions ago,
        # unless a newer one already arrived (latest-frame semantics: late frames are dropped)
        jit_dr = rng("latency_jitter_steps", (0, 0))
        self.jit_fixed = int(pc.get("latency_jitter_steps") or 0)
        self.jit_dr = (int(jit_dr[0]), int(jit_dr[1]))
        self.jit_max = self.jit_fixed + self.jit_dr[1]
        self.jitter = torch.full((self.E,), self.jit_fixed + self.jit_dr[0], dtype=torch.long, device=self.device)
        self.lat_max = self.lat_fixed + self.lat_dr[1] + self.jit_max
        self._lat_buf: Optional[tuple] = None              # (lat_max+1, E, ...) per add_detections argument
        self._lat_ptr = 0                                  # slot of the newest captured frame
        self._lat_cap = self._last_cap = None              # jitter: capture index per slot / last delivered
        self._cap_n = 0
        # general-robustness knobs (all off by default: no draws, bit-identical behaviour)
        self.gust_dr, self.gust_tau = rng("gust_accel", (0.0, 0.0)), float(dr.get("gust_tau_s", 1.0))
        self.gust_sigma = torch.zeros(self.E, device=self.device)
        self.gust = torch.zeros(self.E, 3, device=self.device)
        self.drop_fixed, self.drop_dr = float(pc.get("frame_drop_prob") or 0.0), rng("frame_drop_prob", (0.0, 0.0))
        self.drop_p = torch.full((self.E,), self.drop_fixed, device=self.device)
        self.drift_fixed, self.drift_dr = float(pc.get("pose_drift_m") or 0.0), rng("pose_drift_m", (0.0, 0.0))
        self.drift_sigma = torch.full((self.E,), self.drift_fixed, device=self.device)
        self.drift = torch.zeros(self.E, 3, device=self.device)
        self.cam_fixed, self.cam_dr = float(pc.get("cam_rot_err_deg") or 0.0), rng("cam_rot_err_deg", (0.0, 0.0))
        self.cam_err_rad = torch.zeros(self.E, device=self.device)
        self.R_err = None                                  # (E,3,3) camera mounting error, when enabled
        self.ctrl_dr = (rng("ctrl_kp_scale", (1.0, 1.0)), rng("ctrl_kv_scale", (1.0, 1.0)), rng("ctrl_ki", (0.0, 0.0)))
        self.exp_fixed, self.exp_dr = float(pc.get("exposure_s") or 0.0), rng("exposure_s", (0.0, 0.0))
        self.exposure = torch.full((self.E,), self.exp_fixed, device=self.device)   # camera exposure [s] (motion blur)
        self._gust_on = self.gust_dr[1] > 0
        self._drop_on = self.drop_fixed > 0 or self.drop_dr[1] > 0
        self._drift_on = self.drift_fixed > 0 or self.drift_dr[1] > 0
        self._cam_on = self.cam_fixed > 0 or self.cam_dr[1] > 0
        self._ctrl_on = self.ctrl_dr != ((1.0, 1.0), (1.0, 1.0), (0.0, 0.0))
        self._blur_on = self.exp_fixed > 0 or self.exp_dr[1] > 0
        ls = dr.get("level_scale") if self.dr_enabled else None
        self.level_scale = None if ls is None else torch.tensor([float(v) for v in ls], device=self.device)
        self.curriculum_enabled = bool(cfg["curriculum"]["enabled"])
        fl = cfg["curriculum"].get("fixed_level")       # yaml: train without curriculum (ablation)
        self.fixed_level: Optional[int] = None if fl is None else int(fl)   # evaluation pins a level

        self.policy_spec = build_spec(cfg, "policy")
        self.critic_spec = build_spec(cfg, "critic")
        self.num_policy_obs = self.policy_spec[-1][2]
        self.num_critic_obs = self.num_policy_obs + (self.critic_spec[-1][2] if self.critic_spec else 0)
        self.reward_terms = active_terms(cfg)
        self.episode_sums: Dict[str, torch.Tensor] = {k: torch.zeros(self.E, device=self.device)
                                                      for k in self.reward_terms}
        self.success_rewarded = torch.zeros(self.E, dtype=torch.bool, device=self.device)
        self.crash_rewarded = torch.zeros(self.E, dtype=torch.bool, device=self.device)
        self.prev_dist = torch.zeros(self.E, device=self.device)
        self.prev_phi = torch.zeros(self.E, device=self.device)
        self.progress_delta = torch.zeros(self.E, device=self.device)
        self.est_potential_delta = torch.zeros(self.E, device=self.device)
        self.err_cap = float(cfg["reward"]["estimate_error_cap"])
        self.frame_every = max(1, int(cfg["perception"]["frame_every"]))
        self.log: Dict[str, float] = {}
        # per-env outcome of the most recently finished episode (filled just before reset)
        z = lambda dt=torch.float32: torch.zeros(self.E, dtype=dt, device=self.device)  # noqa: E731
        self.episode_record = {
            "success": z(torch.bool), "crashed": z(torch.bool), "crash_reason": z(torch.long),
            "windows_passed": z(torch.long), "n_windows": z(torch.long), "time_s": z(),
            "clearance_mean": z(), "clearance_min": z(), "level": z(torch.long),
            "est_err_at_pass": z(), "return": z(),
        }
        self.episode_return = z()
        self._first_reset = torch.ones(self.E, dtype=torch.bool, device=self.device)
        self.det = None
        self.est = None
        self.est_err = torch.zeros(self.E, self.W, device=self.device)
        self.err_at_pass = torch.zeros(self.E, self.W, device=self.device)
        self.yaw = torch.zeros(self.E, device=self.device)
        self.body_x_w = torch.zeros(self.E, 3, device=self.device)
        if self.slung is not None and ((cfg["slung"].get("convex_layer") or {}).get("in_env")):
            from ..planners.convex_layer import SlungLayerFilter
            self.layer = SlungLayerFilter(cfg, self.E, self.device, self.step_dt)

    # ------------------------------------------------------------------ helpers
    @property
    def body_mass(self) -> torch.Tensor:
        """Mass the backend integrates: the drone alone when the load hangs on its cable (integrated here)."""
        return self.mass if self.slung is None else self.mass - self.slung.m

    @property
    def target_idx(self) -> torch.Tensor:
        return self.mission.target_index()

    def reseed(self, seed: int) -> None:
        """Seed the four RNG streams. ``gen`` draws the camera noise with a fixed shape every
        decision; ``reset_gen`` draws everything per episode (curriculum, scene, DR, noise
        levels, first frame); ``obs_gen`` draws observation noise; ``dist_gen`` draws the per-step
        disturbances (gusts, frame drops, latency jitter, VIO drift — fixed shapes, only when
        enabled). How often envs reset or a planner reads observations only advances its own
        stream, so after reseed + reset every planner sees the same scenes, camera noise and
        disturbances in each env's first episode."""
        s = int(seed)
        self.gen.manual_seed(s)
        self.reset_gen.manual_seed(s + 1)
        self.obs_gen.manual_seed(s + 2)
        self.dist_gen.manual_seed(s + 3)

    def _u(self, lohi, n) -> torch.Tensor:
        lo, hi = float(lohi[0]), float(lohi[1])
        return lo + (hi - lo) * torch.rand(n, generator=self.reset_gen, device=self.device)

    @property
    def episode_time(self) -> torch.Tensor:
        return self.episode_step.float() * self.step_dt

    def _refresh_cache(self) -> None:
        self.yaw = m3.yaw_from_quat(self.state.quat)
        self.body_x_w = m3.quat_to_rotmat(self.state.quat)[..., :, 0]

    # ------------------------------------------------------------------ reset
    def reset_envs(self, env_ids: torch.Tensor):
        """Curriculum update + new scene + DR for ``env_ids``.

        Returns (start_pos (n,3), start_quat (n,4)) for the backend to place the body.
        Call :meth:`after_reset` once the backend state is mirrored into ``self.state``.
        """
        if env_ids.numel() == 0:
            return None, None
        self._log_episodes(env_ids)
        self._update_levels(env_ids)
        sample_scene(self.cfg, self.table, self.level, self.reset_gen, self.scene, env_ids)
        if self.per_window_s > 0:
            nw = self.scene.n_windows[env_ids]
            self.ep_limit[env_ids] = self.base_steps + torch.ceil(nw.float() * (self.per_window_s / self.step_dt)).long()
            self.episode_len_s[env_ids] = self.episode_length_s + self.per_window_s * nw.float()

        n = env_ids.numel()
        dr = self.cfg["domain_randomization"]
        sc = None                                 # per-env DR strength (curriculum level_scale)
        if self.dr_enabled and self.level_scale is not None:
            sc = self.level_scale[self.level[env_ids].clamp_max(self.level_scale.numel() - 1)]
        if self.dr_enabled:
            ms = self._u(dr["mass_scale"], n)
            self.mass[env_ids] = self.mass0 * ms
            self.inertia[env_ids] = self.inertia0 * self._u(dr["inertia_scale"], n)[:, None]
            self.thrust_max[env_ids] = (float(self.cfg["drone"]["thrust_to_weight"]) * self.mass0 * self.g
                                        * self._u(dr["thrust_scale"], n))
            self.drag[env_ids] = float(self.cfg["drone"]["linear_drag"]) * self._u(dr["drag_scale"], n)
            tc = self._u(dr["motor_time_constant"], n)
            mag = self._u(dr["wind_accel"], n)
            if sc is not None:
                mag = mag * sc
            ang = self._u((0.0, 2 * math.pi), n)
            self.wind_acc[env_ids] = torch.stack((mag * torch.cos(ang), mag * torch.sin(ang), torch.zeros_like(mag)), -1)
            p_bo = float(dr.get("blackout_prob", 0.0))
            if p_bo > 0:                          # camera outage DR (no draws when off)
                has = torch.rand(n, generator=self.reset_gen, device=self.device) < (p_bo if sc is None else p_bo * sc)
                t0 = self._u(dr["blackout_start_s"], n)
                t1 = t0 + self._u(dr["blackout_len_s"], n)
                inf = torch.full_like(t0, float("inf"))
                self.blackout[env_ids] = torch.stack((torch.where(has, t0, inf), torch.where(has, t1, inf)), -1)
            lo, hi = self.lat_dr
            if hi > lo:                           # detection latency DR (no draws when off)
                k = (torch.rand(n, generator=self.reset_gen, device=self.device) * (hi - lo + 1)).long()
                k = lo + k.clamp_max(hi - lo)
                if sc is not None:
                    k = (k.float() * sc).round().long()
                self.latency[env_ids] = self.lat_fixed + k
        else:
            self.mass[env_ids] = self.mass0
            self.inertia[env_ids] = self.inertia0
            self.thrust_max[env_ids] = float(self.cfg["drone"]["thrust_to_weight"]) * self.mass0 * self.g
            self.drag[env_ids] = float(self.cfg["drone"]["linear_drag"])
            tc = torch.full((n,), float(self.cfg["drone"]["motor_time_constant"]), device=self.device)
            self.wind_acc[env_ids] = 0.0
        self._robust_draws(env_ids, n, sc)

        lv = self.level[env_ids]
        sig = self.table.uniform("corner_noise_px", lv, self.reset_gen) * float(self.cfg["perception"].get("noise_scale", 1.0))
        miss = self.table.uniform("miss_prob", lv, self.reset_gen)
        self.perception.reset(env_ids, sigma_px=sig, miss_prob=miss)
        self.controller.reset(env_ids)
        self.motor.reset(env_ids, self.mass[env_ids] * self.g, tc)

        start = self.scene.start[env_ids].clone()
        yaw0 = torch.full((n,), float(self.cfg["controller"]["yaw_target"]), device=self.device)
        return start, m3.quat_from_yaw(yaw0)

    def _robust_draws(self, env_ids: torch.Tensor, n: int, sc: Optional[torch.Tensor]) -> None:
        """Per-episode draws of the general-robustness knobs (gusts, frame drops, latency jitter, VIO drift,
        camera mounting error, controller mismatch). Each draws only when enabled, so with every knob
        off the episode stream is untouched. ``sc``: per-env DR strength (domain_randomization.level_scale)."""
        g = self.reset_gen
        scale = (lambda x: x) if sc is None else (lambda x: x * sc)  # noqa: E731
        self.gust[env_ids] = 0.0
        self.drift[env_ids] = 0.0
        if self._gust_on:                         # OU gusts, started from their stationary distribution
            sig = scale(self._u(self.gust_dr, n))
            self.gust_sigma[env_ids] = sig
            self.gust[env_ids, :2] = torch.randn(n, 2, generator=g, device=self.device) * sig[:, None]
        if self.drop_dr[1] > 0:
            self.drop_p[env_ids] = self.drop_fixed + scale(self._u(self.drop_dr, n))
        lo, hi = self.jit_dr
        if hi > 0:
            k = lo + (torch.rand(n, generator=g, device=self.device) * (hi - lo + 1)).long().clamp_max(hi - lo)
            self.jitter[env_ids] = self.jit_fixed + (k if sc is None else (k.float() * sc).round().long())
        if self.drift_dr[1] > 0:
            self.drift_sigma[env_ids] = self.drift_fixed + scale(self._u(self.drift_dr, n))
        if self._cam_on:                          # mounting error: random axis, magnitude fixed + DR
            axis = m3.normalize(torch.randn(n, 3, generator=g, device=self.device))
            mag = self.cam_fixed + (scale(self._u(self.cam_dr, n)) if self.cam_dr[1] > 0 else torch.zeros(n, device=self.device))
            rad = torch.deg2rad(mag)
            if self.R_err is None:
                self.R_err = torch.eye(3, device=self.device).repeat(self.E, 1, 1)
            self.R_err[env_ids] = m3.rot_axis_angle(axis, rad)
            self.cam_err_rad[env_ids] = rad
        if self.exp_dr[1] > 0:                    # camera exposure (motion blur)
            self.exposure[env_ids] = self.exp_fixed + scale(self._u(self.exp_dr, n))
        if self._ctrl_on:                         # low-level controller mismatch (gains, velocity integral)
            (kp_r, kv_r, ki_r) = self.ctrl_dr
            kp, kv, ki = self._u(kp_r, n), self._u(kv_r, n), self._u(ki_r, n)
            if sc is not None:
                kp, kv, ki = 1.0 + (kp - 1.0) * sc, 1.0 + (kv - 1.0) * sc, ki * sc
            self.controller.set_variation(env_ids, kp, kv, ki)

    def _motion(self):
        """(vel_w, omega_w, exposure) for motion blur, or None when the camera exposure knob is off."""
        if not self._blur_on:
            return None
        s = self.state
        return s.vel_w, m3.quat_rotate(s.quat, s.omega_b), self.exposure

    def after_reset(self, env_ids: torch.Tensor) -> None:
        if env_ids.numel() == 0:
            return
        s = self.state
        if self.slung is not None:
            self.slung.reset(env_ids, s.pos[env_ids], s.quat[env_ids])
            self.mission.reset(env_ids, s.pos[env_ids], self.scene, load_pos=self.slung.pos[env_ids])
            if self.layer is not None:
                self.layer.reset(env_ids)
        else:
            self.mission.reset(env_ids, s.pos[env_ids], self.scene)
        self.actions[env_ids] = 0.0
        self.prev_actions[env_ids] = 0.0
        self.raw_actions[env_ids] = 0.0
        self.wp[env_ids] = s.pos[env_ids]
        self.yaw_cmd[env_ids] = float(self.cfg["controller"]["yaw_target"])
        self.success_rewarded[env_ids] = False
        self.crash_rewarded[env_ids] = False
        for v in self.episode_sums.values():
            v[env_ids] = 0.0
        self.episode_return[env_ids] = 0.0
        self.err_at_pass[env_ids] = 0.0
        self._first_reset[env_ids] = False
        # first camera frame so the initial observation already holds a detection
        self._refresh_cache()
        take = torch.zeros(self.E, dtype=torch.bool, device=self.device)
        take[env_ids] = True
        cam_c, R_WC = self.perception.camera_pose(s.pos, s.quat)
        frame = self.perception.capture(cam_c, R_WC, self.scene, take, self.reset_gen,
                                        ray_offset=self.drift if self._drift_on else None, R_err=self.R_err,
                                        motion=self._motion())
        det = self.perception.add_detections(*self._deliver(frame, env_ids))
        self.det = det if self.det is None else self._merge_det(det, env_ids)
        self.est = self._estimates()
        self.est_err = PerceptionSurrogate.center_error(self.est, self.scene, self.err_cap)
        tgt = self.target_idx
        self.prev_dist[env_ids] = (_gather(self.scene.center, tgt) - s.pos).norm(dim=-1)[env_ids]
        self.prev_phi[env_ids] = -_gather(self.est_err, tgt).clamp_max(self.err_cap)[env_ids]
        self.selector.reset(env_ids, s.pos[env_ids], self.yaw[env_ids], torch.zeros(env_ids.numel(), device=self.device))
        self.selector.select(self.scene, self.mission, self.est, self.perception)

    def _estimates(self):
        """Triangulated estimates in the frame the planner works in. With VIO drift the rays were cast from
        drifted positions (VIO frame) and the planner's own position drifts the same way, so relative to the
        TRUE own position the estimates sit shifted by -drift(now): only drift accumulated since the rays were
        taken matters, as on the real drone."""
        est = self.perception.estimate()
        if not self._drift_on:
            return est
        import dataclasses
        d = self.drift[:, None, :] * est.valid[..., None].float()
        return dataclasses.replace(est, center=est.center - d, corners=est.corners - d[:, :, None, :])

    def _deliver(self, frame: tuple, reset_ids: Optional[torch.Tensor] = None) -> tuple:
        """Detection latency: keep ``frame`` (PerceptionSurrogate.add_detections arguments) as the newest
        capture and return, per env, the frame captured ``latency`` decisions ago. ``reset_ids``: the
        first frame of new episodes replaces those envs' newest slot, their older frames never arrive,
        and only they receive anything. No buffer (identical behaviour) when latency is 0."""
        if self.lat_max == 0:
            return frame
        L = self.lat_max + 1
        jit = self.jit_max > 0
        if self._lat_buf is None:
            self._lat_buf = tuple(t.unsqueeze(0).repeat(L, *([1] * t.dim())) for t in frame)
            self._lat_buf[3].zero_()                       # det: nothing in flight yet
            self._lat_buf[5].zero_()                       # take
            if jit:
                self._lat_cap = torch.full((L, self.E), -1, dtype=torch.long, device=self.device)
                self._last_cap = torch.full((self.E,), -1, dtype=torch.long, device=self.device)
        if reset_ids is None:
            self._lat_ptr = (self._lat_ptr + 1) % L
            for b, t in zip(self._lat_buf, frame):
                b[self._lat_ptr] = t
        else:
            self._lat_buf[3][:, reset_ids] = False
            self._lat_buf[5][:, reset_ids] = False
            for b, t in zip(self._lat_buf, frame):
                b[self._lat_ptr, reset_ids] = t[reset_ids]
        delay = self.latency
        if jit:
            self._cap_n += 1
            if reset_ids is None:
                self._lat_cap[self._lat_ptr] = self._cap_n
                # per-decision jitter (dist_gen, fixed shape); never drawn on resets, whose timing depends on the planner
                u = torch.rand(self.E, generator=self.dist_gen, device=self.device)
                delay = delay + (u * (self.jitter + 1).float()).long().clamp_max(self.jitter)
            else:
                self._lat_cap[self._lat_ptr, reset_ids] = self._cap_n
                self._last_cap[reset_ids] = -1
        slot = (self._lat_ptr - delay) % L
        ar = torch.arange(self.E, device=self.device)
        out = [b[slot, ar] for b in self._lat_buf]
        m = None
        if reset_ids is not None:                          # the others already got this slot last decision
            m = torch.zeros(self.E, dtype=torch.bool, device=self.device)
            m[reset_ids] = True
        if jit:                                            # deliver only frames newer than the last one delivered
            cap = self._lat_cap[slot, ar]
            fresh = cap > self._last_cap
            if m is not None:
                fresh = fresh & m
            self._last_cap = torch.where(fresh, cap, self._last_cap)
            m = fresh if m is None else m & fresh
        if m is not None:
            out[3] = out[3] & m[:, None]
            out[5] = out[5] & m
        return tuple(out)

    def _merge_det(self, det, env_ids):
        old = self.det
        m = torch.zeros(self.E, dtype=torch.bool, device=self.device)
        m[env_ids] = True
        for f in ("detected", "conf", "bearing_w", "side_px"):
            o, nw = getattr(old, f), getattr(det, f)
            mm = m.view(-1, *([1] * (o.dim() - 1)))
            setattr(old, f, torch.where(mm, nw, o))
        return old

    def _update_levels(self, env_ids: torch.Tensor) -> None:
        if self.fixed_level is not None:
            self.level[env_ids] = int(self.fixed_level)
            return
        if not self.curriculum_enabled or self.n_levels <= 1:
            self.level[env_ids] = 0
            return
        first = self._first_reset[env_ids]
        cur = self.cfg["curriculum"]
        init_max = int(cur["init_level_max"])
        init = (torch.rand(env_ids.numel(), generator=self.reset_gen, device=self.device) * (init_max + 1)).floor().long()
        lv = self.level[env_ids]
        up = self.mission.success[env_ids]
        if cur["promote_on"] == "windows":
            up = up | (self.mission.target[env_ids] >= self.scene.n_windows[env_ids].clamp_min(1))
        down = self.mission.crashed[env_ids] if cur["demote_on_crash"] else torch.zeros_like(up)
        lv = lv + up.long() - (down & ~up).long()
        lv = torch.where(first, init, lv).clamp(0, self.n_levels - 1)
        self.level[env_ids] = lv

    def _log_episodes(self, env_ids: torch.Tensor) -> None:
        done = env_ids[~self._first_reset[env_ids]]
        if done.numel() == 0:
            return
        m = self.mission
        rec = self.episode_record
        passed_d = m.passed[done]
        npass = passed_d.sum(-1)
        clr = torch.where(passed_d, m.clearance[done], torch.zeros_like(m.clearance[done]))
        rec["success"][done] = m.success[done]
        rec["crashed"][done] = m.crashed[done]
        rec["crash_reason"][done] = m.crash_reason[done]
        rec["windows_passed"][done] = m.target[done]
        rec["n_windows"][done] = self.scene.n_windows[done]
        rec["time_s"][done] = self.episode_time[done]
        rec["clearance_mean"][done] = clr.sum(-1) / npass.clamp_min(1)
        rec["clearance_min"][done] = torch.where(passed_d, m.clearance[done], torch.full_like(clr, 9.0)).amin(-1)
        rec["level"][done] = self.level[done]
        rec["est_err_at_pass"][done] = (self.err_at_pass[done] * passed_d).sum(-1) / npass.clamp_min(1)
        rec["return"][done] = self.episode_return[done]
        log = {}
        for k, v in self.episode_sums.items():
            log[f"Episode_Reward/{k}"] = float(v[done].mean())
        n_win = self.scene.n_windows[done].float().clamp_min(1)
        log["Metrics/success_rate"] = float(m.success[done].float().mean())
        log["Metrics/crash_rate"] = float(m.crashed[done].float().mean())
        log["Metrics/timeout_rate"] = float((~m.success[done] & ~m.crashed[done]).float().mean())
        log["Metrics/windows_passed_frac"] = float((m.target[done].float() / n_win).mean())
        log["Metrics/curriculum_level"] = float(self.level.float().mean())
        log["Metrics/episode_time_s"] = float(self.episode_time[done].mean())
        log["Metrics/n_windows"] = float(self.scene.n_windows[done].float().mean())
        passed = m.passed[done]
        if passed.any():
            log["Metrics/pass_clearance_m"] = float(m.clearance[done][passed].mean())
        if self.slung is not None:
            seen = m.load_seen[done]
            if seen.any():
                log["Metrics/load_clearance_m"] = float(m.load_clear[done][seen].mean())
        for code, name in enumerate(CRASH_NAMES[1:], start=1):
            log[f"Crash/{name}"] = float((m.crash_reason[done] == code).float().mean())
        if self.layer is not None and self._layer_n:
            log["Metrics/layer_changed"] = float(self._layer_hits) / self._layer_n
            self._layer_hits, self._layer_n = 0.0, 0
        self.log = log

    # ------------------------------------------------------------------ step
    def pre_decision(self, actions: torch.Tensor) -> None:
        self.prev_actions = self.actions.clone()
        self.raw_actions = actions
        self.actions = actions.clamp(-1.0, 1.0)
        if self.layer is not None:               # the convex layer in the loop: the env flies the projected action
            self.actions = self.layer.filter(self.actions, self)
            self._layer_hits = self._layer_hits + self.layer.changed.float().mean()
            self._layer_n += 1
        if self._gust_on:                        # Ornstein-Uhlenbeck gusts, stationary std = gust_sigma
            a = math.exp(-self.step_dt / self.gust_tau)
            w = torch.randn(self.E, 2, generator=self.dist_gen, device=self.device)
            self.gust[:, :2] = a * self.gust[:, :2] + (self.gust_sigma * math.sqrt(1.0 - a * a))[:, None] * w
        delta_h = self.actions[:, :3] * self.step_max
        wp = self.state.pos + m3.heading_to_world(delta_h, self.yaw)
        wp[:, 2] = wp[:, 2].clamp(*self.wp_z)
        self.wp = wp
        if self.yaw_action:
            self.yaw_cmd = self.yaw_cmd + self.actions[:, 3] * self.yaw_rate * self.step_dt
        self.mission.begin_step()
        self._sub = 0

    def control(self):
        """-> (thrust (E,), torque_b (E,3), external force world (E,3)) for one physics step."""
        s = self.state
        # The controller knows the true mass (hover-thrust calibration) but only the NOMINAL
        # inertia, so inertia randomisation creates a real model mismatch (README §7.1).
        f_cmd, tau_cmd = self.controller.compute(s.pos, s.vel_w, s.quat, s.omega_b, self.wp, self.yaw_cmd,
                                                 self.mass, self.inertia_ctrl, self.thrust_max)
        f, tau = self.motor.step(f_cmd, tau_cmd, self.dt)
        wind = self.wind_acc + self.gust if self._gust_on else self.wind_acc
        if self.slung is None:
            f_ext = self.mass[:, None] * wind - self.drag[:, None] * s.vel_w
            return f, tau, f_ext
        # the load pulls on the hook: a force on the drone and a torque about its centre of mass
        f_cable, tau_cable = self.slung.cable_force(s.pos, s.vel_w, s.quat, s.omega_b)
        self._wind_now = wind
        f_ext = self.body_mass[:, None] * wind - self.drag[:, None] * s.vel_w + f_cable
        return f, tau + tau_cable, f_ext

    def after_substep(self) -> None:
        self._sub += 1
        t = (self.episode_step.float() * self.step_dt) + self._sub * self.dt
        if self.slung is None:
            self.mission.substep(self.state.pos, self.state.quat, self.scene, t)
            return
        sl = self.slung
        sl.integrate(self.dt, self._wind_now)
        hook, _ = sl.hook(self.state.pos, self.state.quat)
        self.mission.substep(self.state.pos, self.state.quat, self.scene, t, load=(sl.pos, hook, sl.radius))

    def post_decision(self) -> None:
        """Call after the backend has advanced ``episode_step`` for this decision."""
        self._refresh_cache()
        s = self.state
        take = (self.episode_step % self.frame_every) == 0
        t = self.episode_time
        take = take & ~((t >= self.blackout[:, 0]) & (t < self.blackout[:, 1]))   # DR outage
        bo = self.cfg["perception"].get("blackout_s")
        if bo:                                   # stress knob: no camera frames in [t0, t1)
            take = take & ~((t >= float(bo[0])) & (t < float(bo[1])))
        if self._drop_on:                        # random frame loss (dist_gen; no draws when off)
            take = take & (torch.rand(self.E, generator=self.dist_gen, device=self.device) >= self.drop_p)
        if self._drift_on:                       # VIO drift: random walk of the position rays are cast from
            step = torch.randn(self.E, 3, generator=self.dist_gen, device=self.device)
            self.drift = self.drift + step * (self.drift_sigma * math.sqrt(self.step_dt))[:, None]
        cam_c, R_WC = self.perception.camera_pose(s.pos, s.quat)
        frame = self.perception.capture(cam_c, R_WC, self.scene, take, self.gen,
                                        ray_offset=self.drift if self._drift_on else None, R_err=self.R_err,
                                        motion=self._motion())
        self.perception.tick()
        self.det = self.perception.add_detections(*self._deliver(frame))
        self.est = self._estimates()
        self.est_err = PerceptionSurrogate.center_error(self.est, self.scene, self.err_cap)

        sp = self.mission.step_pass
        self.err_at_pass = torch.where(sp, self.est_err, self.err_at_pass)
        tgt = self.target_idx
        switched = sp.any(-1)
        if bool(switched.any()):                 # the look-around restarts from the window just passed
            w = sp.float().argmax(-1)
            self.selector.on_pass(switched, s.pos, s.vel_w, self.yaw, self.episode_time,
                                  _gather(self.est.center, w), _gather(self.est.valid, w), _gather(self.est.normal, w))
        self.selector.scan(self.yaw, s.pos[:, 2], self.episode_time)
        self.selector.select(self.scene, self.mission, self.est, self.perception)
        dist = (_gather(self.scene.center, tgt) - s.pos).norm(dim=-1)
        self.progress_delta = torch.where(switched, torch.zeros_like(dist), self.prev_dist - dist)
        self.prev_dist = dist
        phi = -_gather(self.est_err, tgt).clamp_max(self.err_cap)
        self.est_potential_delta = torch.where(switched, torch.zeros_like(phi), phi - self.prev_phi)
        self.prev_phi = phi

    def compute_dones(self):
        terminated = self.mission.crashed | self.mission.success
        time_out = self.episode_step >= self.ep_limit
        return terminated, time_out & ~terminated

    def compute_rewards(self) -> torch.Tensor:
        total = torch.zeros(self.E, device=self.device)
        for name, w in self.reward_terms.items():
            r = REWARDS[name](self) * w
            self.episode_sums[name] += r
            total += r
        self.success_rewarded |= self.mission.success
        self.crash_rewarded |= self.mission.crashed
        self.episode_return += total
        return total

    def compute_observations(self) -> Dict[str, torch.Tensor]:
        pol = compute_group(self, self.policy_spec)
        crit = compute_group(self, self.critic_spec) if self.critic_spec else pol[:, :0]
        return {"policy": pol, "critic": crit}

    # ------------------------------------------------------------------ meta
    def observation_layout(self) -> dict:
        return {"policy": [{"name": n, "start": a, "end": b} for n, a, b in self.policy_spec],
                "critic_extra": [{"name": n, "start": a, "end": b} for n, a, b in self.critic_spec]}
