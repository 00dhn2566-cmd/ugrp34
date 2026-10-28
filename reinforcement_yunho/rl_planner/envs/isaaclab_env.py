"""Isaac Lab backend: the window-mission MDP on PhysX rigid-body physics.

Standard Isaac Lab *direct* workflow (DirectRLEnv), the same pattern as the
official task suite: one cloned rigid body per env, the low-level controller's
wrench applied every physics step through the asset's wrench composer, and the
whole MDP (scene sampling, perception surrogate, referee, observations,
rewards, curriculum, DR) delegated to :class:`WindowMissionCore`, so it is
bit-for-bit the same task as the torch backend.

Import this module only after the Isaac Sim app is running
(``isaaclab.app.AppLauncher``) — see scripts/train.py.

Notes
-----
* Isaac Lab 3.0 quaternions are (x, y, z, w); the core uses (w, x, y, z). The
  conversion happens only in ``_sync_state`` / ``_reset_idx``.
* The drone body has no collider: contacts are refereed analytically by the core
  (MissionTracker), and inter-env collisions cannot happen. It is drawn as the real
  airframe (core/airframe.py: FX450 propellers, arms, body, package), visual only.
* Windows are not spawned for training (nothing physical touches them); the
  play script spawns visual frames when rendering a rollout.
"""
from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import MISSING

import gymnasium as gym
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg, schemas
from isaaclab.sim.spawners.utils import apply_schema_props
from isaaclab.sim.utils import bind_visual_material, clone, create_prim, get_current_stage
from isaaclab.utils import configclass
from isaaclab_physx.sim.schemas import PhysxRigidBodyCfg

from ..core import math3d as m3
from ..core.airframe import Airframe
from ..core.config import load_config
from ..core.env_core import WindowMissionCore

TASK_ID = "UGRP-WindowMission-Direct-v0"


def _xyzw_to_wxyz(q: torch.Tensor) -> torch.Tensor:
    return torch.cat((q[..., 3:4], q[..., 0:3]), dim=-1)


def _wxyz_to_xyzw(q: torch.Tensor) -> torch.Tensor:
    return torch.cat((q[..., 1:4], q[..., 0:1]), dim=-1)


@clone
def spawn_airframe(prim_path: str, cfg: "AirframeCfg", translation=None, orientation=None, **kwargs):
    """The real airframe as one rigid body with visual-only parts (Airframe.visual_parts). No colliders: the core
    referees contacts. Mass and inertia are written per env at reset; the centre of mass is the body origin."""
    from pxr import Gf, UsdPhysics
    stage = get_current_stage()
    prim = create_prim(prim_path, prim_type="Xform", translation=translation, orientation=orientation, stage=stage)
    for part in cfg.parts:
        path = f"{prim_path}/geometry/{part['name']}"
        q = _wxyz_to_xyzw(m3.quat_from_rotmat(torch.tensor(part["R"]))).tolist()
        if part["kind"] == "box":
            create_prim(path, "Cube", translation=part["center"], orientation=q, scale=part["size"],
                        attributes={"size": 1.0}, stage=stage)
        else:
            create_prim(path, "Cylinder", translation=part["center"], orientation=q, stage=stage,
                        attributes={"radius": part["radius"], "height": part["length"], "axis": "Z"})
        look_path = f"/World/Looks/airframe_{part['name']}"             # shared by every env
        if not stage.GetPrimAtPath(look_path).IsValid():
            look = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(part["rgb"]), opacity=float(part["opacity"]))
            look.func(look_path, look)
        bind_visual_material(path, look_path, stage=stage)
    apply_schema_props(cfg.mass_props, prim_path, schemas.apply_mass_properties, schemas.define_mass_properties, stage)
    UsdPhysics.MassAPI(prim).CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    apply_schema_props(cfg.rigid_props, prim_path, schemas.apply_rigid_body_properties,
                       schemas.define_rigid_body_properties, stage)
    return prim


@configclass
class AirframeCfg(sim_utils.RigidObjectSpawnerCfg):
    """Spawner for :func:`spawn_airframe`; ``parts`` = Airframe(cfg).visual_parts()."""

    func: Callable = spawn_airframe
    parts: list = MISSING


def airframe_spawn_cfg(mission_cfg) -> AirframeCfg:
    return AirframeCfg(
        parts=Airframe(mission_cfg, "cpu").visual_parts(),
        rigid_props=PhysxRigidBodyCfg(disable_gravity=False, linear_damping=0.0, angular_damping=0.0,
                                      enable_gyroscopic_forces=True, max_depenetration_velocity=1.0),
        mass_props=sim_utils.MassCfg(mass=float(mission_cfg["drone"]["mass"])))


@configclass
class WindowMissionSceneCfg(InteractiveSceneCfg):
    drone: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Drone",
        spawn=sim_utils.CuboidCfg(                          # replaced by the airframe in WindowMissionIsaacEnv
            size=(0.45, 0.45, 0.10),                       # FX450 footprint (visual only)
            rigid_props=PhysxRigidBodyCfg(disable_gravity=False, linear_damping=0.0, angular_damping=0.0,
                                          enable_gyroscopic_forces=True, max_depenetration_velocity=1.0),
            mass_props=sim_utils.MassCfg(mass=2.2726),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.15, 0.15)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 1.0)),
    )
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))


@configclass
class WindowMissionEnvCfg(DirectRLEnvCfg):
    """Isaac Lab cfg. Every task number lives in ``mission`` (rl_planner yaml dict)."""

    mission: dict = None                 # merged rl_planner config (dict); filled by make_env_cfg
    decimation = 10
    episode_length_s = 45.0
    action_space = 3
    observation_space = 41
    state_space = 0
    sim: SimulationCfg = SimulationCfg(dt=0.01, render_interval=10)
    scene: WindowMissionSceneCfg = WindowMissionSceneCfg(num_envs=1024, env_spacing=4.0, replicate_physics=True)


def make_env_cfg(mission_cfg, num_envs: int | None = None, device: str = "cuda:0") -> WindowMissionEnvCfg:
    """rl_planner config -> Isaac Lab env cfg (timing and sizes copied, not duplicated by hand)."""
    env_cfg = WindowMissionEnvCfg()
    env_cfg.mission = mission_cfg.to_dict() if hasattr(mission_cfg, "to_dict") else dict(mission_cfg)
    s = env_cfg.mission["sim"]
    env_cfg.decimation = int(s["decimation"])
    # the longest per-env limit (episode_s_per_window x max_windows on long courses); the core ends each env at its own
    env_cfg.episode_length_s = float(s["episode_length_s"]) + float(s.get("episode_s_per_window") or 0.0) * int(
        env_cfg.mission["scene"]["max_windows"])
    env_cfg.sim.dt = float(s["physics_dt"])
    env_cfg.sim.render_interval = env_cfg.decimation
    env_cfg.sim.device = device
    env_cfg.scene.num_envs = int(num_envs or s["num_envs"])
    env_cfg.seed = int(env_cfg.mission["seed"])
    probe = WindowMissionCore(env_cfg.mission, 1, "cpu")
    env_cfg.action_space = probe.action_dim
    env_cfg.observation_space = probe.num_policy_obs
    env_cfg.state_space = probe.num_critic_obs - probe.num_policy_obs
    return env_cfg


class WindowMissionIsaacEnv(DirectRLEnv):
    cfg: WindowMissionEnvCfg

    def __init__(self, cfg: WindowMissionEnvCfg, render_mode: str | None = None, **kwargs):
        if cfg.mission is None:
            cfg.mission = load_config().to_dict()
        cfg.scene.drone.spawn = airframe_spawn_cfg(cfg.mission)
        self.core = None
        super().__init__(cfg, render_mode, **kwargs)
        self.drone = self.scene["drone"]
        self.core = WindowMissionCore(cfg.mission, self.num_envs, self.device, seed=int(cfg.mission["seed"]))
        self._sub = 0
        self._body_ids = torch.tensor([0], device=self.device, dtype=torch.int32)
        self._all = torch.arange(self.num_envs, device=self.device)
        self._reset_idx(self._all)

    # ------------------------------------------------------------------ state mirror
    def _sync_state(self) -> None:
        d = self.drone.data
        st = self.core.state
        st.pos = d.root_link_pos_w.torch - self.scene.env_origins
        st.quat = _xyzw_to_wxyz(d.root_link_quat_w.torch)
        st.vel_w = d.root_link_lin_vel_w.torch.clone()
        st.omega_b = d.root_link_ang_vel_b.torch.clone()

    # ------------------------------------------------------------------ DirectRLEnv hooks
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        # DirectRLEnv never clears extras; drop last step's episode log so rsl_rl only
        # averages logs from steps where episodes actually ended.
        self.extras.pop("log", None)
        self._sync_state()
        self.core.episode_step = self.episode_length_buf
        self.core.pre_decision(actions)
        self._sub = 0

    def _apply_action(self) -> None:
        if self._sub > 0:                        # referee the physics step that just happened
            self._sync_state()
            self.core.after_substep()
        thrust, torque_b, f_ext_w = self.core.control()
        force_b = m3.quat_rotate_inverse(self.core.state.quat, f_ext_w)
        force_b[:, 2] += thrust
        self.drone.permanent_wrench_composer.set_forces_and_torques_index(
            forces=force_b.unsqueeze(1), torques=torque_b.unsqueeze(1), is_global=False)
        self._sub += 1

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self._sync_state()
        self.core.after_substep()                # last physics step of this decision
        self.core.episode_step = self.episode_length_buf
        self.core.post_decision()
        return self.core.compute_dones()

    def _get_rewards(self) -> torch.Tensor:
        return self.core.compute_rewards()

    def _get_observations(self) -> dict:
        o = self.core.compute_observations()
        return {"policy": o["policy"], "critic": o["critic"]}

    def _reset_idx(self, env_ids: Sequence[int] | torch.Tensor | None):
        if self.core is None:                    # DirectRLEnv.__init__ may reset before the core exists
            return
        if env_ids is None:
            env_ids = self._all
        env_ids = torch.as_tensor(env_ids, device=self.device).long()
        self.core.episode_step = self.episode_length_buf
        start, quat_wxyz = self.core.reset_envs(env_ids)   # logs the finished episodes first
        super()._reset_idx(env_ids)
        n = env_ids.numel()
        pose = torch.cat((start + self.scene.env_origins[env_ids], _wxyz_to_xyzw(quat_wxyz)), dim=-1)
        self.drone.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.drone.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=self.device),
                                                    env_ids=env_ids)
        self.drone.set_masses_index(masses=self.core.body_mass[env_ids].unsqueeze(1), env_ids=env_ids)
        I = torch.diag_embed(self.core.inertia[env_ids]).reshape(n, 1, 9)
        self.drone.set_inertias_index(inertias=I, env_ids=env_ids)
        st = self.core.state
        st.pos[env_ids] = start
        st.quat[env_ids] = quat_wxyz
        st.vel_w[env_ids] = 0.0
        st.omega_b[env_ids] = 0.0
        self.core.episode_step = self.episode_length_buf
        self.core.after_reset(env_ids)
        if self.core.log:
            self.extras.setdefault("log", {}).update(self.core.log)


gym.register(id=TASK_ID, entry_point=f"{__name__}:WindowMissionIsaacEnv", disable_env_checker=True,
             kwargs={"env_cfg_entry_point": f"{__name__}:WindowMissionEnvCfg"})
