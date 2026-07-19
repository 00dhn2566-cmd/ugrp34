# `rl/` — window-traversal RL (consumer: 윤호, spec 7)

Gym env + reward + training + evaluation for flying the drone through the coloured
windows in traversal order (`red 0 → green 1 → blue 2`, per `CONVENTIONS.md`).

Everything here runs on **numpy + pyyaml only** via a `MockPhysics` backend, so the
whole loop is testable on a plain machine. Heavy deps (`gymnasium`,
`stable-baselines3`, `tensorboard`, Isaac Sim) are import-guarded — files still
import and STEP without them.

## Files → spec map

| file | spec | what it is | runnable now? |
|------|------|-----------|---------------|
| `window_env.py` | 7.1 | `WindowTraversalEnv` (gymnasium.Env or local fallback base). Clean obs (no GT depth/pose), waypoint action, pluggable physics backend, DR hook, optional obs noise (7.6), reward delegated to `reward.py`. Also holds `Scene`/`Window`, `sample_scene`, `MockPhysics`, `IsaacSimBackend` stub. Owns the `OBS_*` obs-layout constants. | **yes** (MockPhysics) |
| `state_window_adapter.py` | 7.3 / 7.6 | `state_window_to_obs(drone_state, window_map, target_order_index)` — the **integration/inference seam**. Derives the *same* 17-dim obs from the real `state_window_interface` dicts (world→body transform, XYZW quats, target pick, normal-from-corners). Imports `window_env`'s `OBS_*` so it can't drift. GT-fed now, VIO-fed at integration. | **yes** |
| `reward.py` | 7.2 | `RewardConfig` + `compute_reward → (total, terms)`; 5 terms `window_pass/collision/progress/attitude/energy`, weights **only** from yaml. | **yes** |
| `domain_randomization.py` | 7.1 / 7.6 | `DomainRandomizationConfig` + seeded sampler → per-episode dynamics (mass/thrust/drag) + sensor-noise stds. | **yes** |
| `baseline.py` | 7.4 | `WaypointBaseline` — non-learned, steers straight at the target window centre; same action space as the env. | **yes** |
| `train.py` | 7.3 | CLI (`--config --seed --resume --smoke`); vectorised envs, learning-curve log, checkpoint save/resume, `run_<config>_<seed>_<hash>/` folder. | **yes** (fallback loop); SB3 path is a **guarded stub** |
| `evaluate.py` | 7.4 | CLI; N seeded scenes, rolls out policy **and** baseline on the **same** scenes → success rate / collision rate / mean pass time → CSV. | **yes** |
| `configs/reward_default.yaml` | 7.2 | reward weights (separate file, **stub** values) | — |
| `configs/train_default.yaml` | 7.3 | training hyperparams + `env:` block + curriculum | — |
| `configs/domain_randomization_default.yaml` | 7.1 | DR ranges | — |

## Runnable-now vs stub

**Runnable now (numpy+pyyaml):**
- `WindowTraversalEnv` with `MockPhysics` — full reset/step, deterministic per seed.
- `state_window_adapter.state_window_to_obs` — pure function; on a GT-consistent
  `drone_state`+`window_map` it reproduces the env obs to float precision.
- `WaypointBaseline` — completes scenes at the gentle default settings.
- `train.py --smoke` — writes + reloads a checkpoint (**gpu_jobs Job 2**).
- `train.py` fallback loop — vectorised stepping, CSV curve, checkpoint/resume.
- `evaluate.py` — the spec-7.4 comparison + CSV.

**Stub / guarded (needs a heavy dep or the real sim):**
- `IsaacSimBackend` — real drone dynamics + PID + collisions in Isaac Sim. Same
  `PhysicsBackend` interface as `MockPhysics`, so env code is unchanged. Raises a
  clear `ImportError`/`NotImplementedError` here.
- `train.py` SB3 path (`_sb3_train`) — real PPO; used automatically **iff**
  `stable-baselines3` imports. Otherwise the labelled fallback loop runs.
- TensorBoard logging — used iff a `SummaryWriter` imports; else CSV only.

## Quick start / smoke commands (run from repo root)

```bash
# env + baseline + reward-term sanity (import and step the env)
python3 -c "import sys; sys.path[:0]=['.','rl']; from window_env import WindowTraversalEnv, MockPhysics; \
e=WindowTraversalEnv(backend=MockPhysics(), seed=0); o,_=e.reset(seed=0); print(e.step(o[:3]*0)[1:4])"

# state_window_interface bridge: derive the 17-dim obs from GT-style dicts
python3 -c "import sys; sys.path[:0]=['.','rl']; from state_window_adapter import state_window_to_obs as f; \
ds={'position':[-3,0,1.5],'orientation':[0,0,0,1],'lin_vel':[0.5,0,0]}; \
wm={'windows':[{'order_index':0,'center':[4,0,1.5],'normal':[-1,0,0],'size_wh':[1.2,1.0], \
'corners_3d':[[4,0.6,2.0],[4,-0.6,2.0],[4,-0.6,1.0],[4,0.6,1.0]]}]}; \
o=f(ds,wm,0); print('obs', o.shape, 'finite', bool(__import__('numpy').all(__import__('numpy').isfinite(o))))"

# training smoke (Job 2): writes & reloads a checkpoint
python3 rl/train.py --smoke

# spec-7.4 eval: baseline vs baseline over 3 scenes -> CSV
python3 rl/evaluate.py --policy baseline --num-scenes 3 --seed 0 --out eval.csv

# evaluate a trained fallback checkpoint against the baseline
python3 rl/evaluate.py --policy runs/<run>/checkpoints/latest.npz --num-scenes 20
```

`train.py` writes to `<log_dir>/run_<config>_<seed>_<hash>/` (default `log_dir:
runs`; `hash` is `nogit` outside a git repo — `CONVENTIONS.md` "Reproducibility").

## Contract / conventions honoured

- **No sim cheating (7.1):** observation is the detector/estimator-style *relative*
  window pose + body velocity + attitude (gravity-in-body) + previous action — never
  GT depth or GT world pose. Obs layout constants (`OBS_*`) live in `window_env.py`
  and are imported by `baseline.py` / `evaluate.py` so all files agree.
- **Waypoint action (7.1 / CONVENTIONS):** policy emits a normalised heading-frame
  waypoint; the backend's PID follows it. Motor control is out of scope this semester.
- **Reward weights in yaml (7.2):** nothing reward-magnitude is hard-coded; only the
  5-term *structure* is fixed.
- **Colour → order_index:** `red 0 / green 1 / blue 2`; scenes are coloured in
  traversal order. Window corners come from `common.geometry.window_corners_world`.
- **Deterministic:** one `np.random.Generator` per env drives scene sampling, DR, and
  obs noise.

## `state_window_interface` bridge (`state_window_adapter.py`)

The RL **training** loop does **not** ride `state_window_interface`
(`overall_gilnam/docs/state_window_interface_spec_v0_1.md` §4): training uses the
vectorised in-sim obs from `window_env` directly. The spec is for the
**integration / inference** path, and the contract is only that the training obs
be **derivable-from** — *not equal-to* — `(drone_state, window_map)` with a
world→body transform done by the consumer (spec §1 원칙 2). `state_window_adapter`
**is** that consumer: `state_window_to_obs(...)` returns the *same* 17-dim vector
(it imports `window_env`'s `OBS_*` layout, so the two can never diverge), built
from the two real interface messages:

- **drone state** (spec §6.1, ~`nav_msgs/Odometry`): `position` world, `orientation`
  **XYZW**, `lin_vel` already **BODY** (spec §2.1-③, used as-is).
- **window 3D map** (spec §6.2, `WindowMap`): world `corners_3d`/`center`/`normal`/
  `size_wh` per `order_index`; the adapter picks the target by `order_index`,
  transforms it into the yaw-only heading frame, and derives the normal from the
  corners (`normalize(cross(c1-c0, c3-c0))`) when no explicit `normal` is shipped.

**GT now / VIO later** (spec §1 원칙 1, §9-조윤호): the schema has two producers.
Now, 윤호 feeds Isaac **GT** pose + **GT** window on this schema so the RL consumer
runs before VIO/궤적-담당 is final. At integration the same fields come from 태민's
`/ov_msckf/poseimu` (VIO pose, XYZW) + `/window_positions`
(`visual_imaging_taemin/window_recon_node.py`). Upstream detector/VIO quality then
caps policy performance (README §7.6) — so train through injected obs noise
(`EnvConfig.observation_noise`, §7.6) before trusting this bridge.

> **Quaternions here are XYZW, not WXYZ.** `drone_state["orientation"]` is
> `[qx,qy,qz,qw]` — matching 태민's `/ov_msckf/poseimu` and the state spec §2.1-②
> — whereas the control-side `interface/` schemas use `orientation_quat_wxyz`.
> There is **no single global quat order** (see `CONVENTIONS.md` "Quaternion order
> is per-interface"); the adapter reorders XYZW→WXYZ and reuses
> `common.geometry.quat_wxyz_to_R` rather than re-deriving. The normal ± direction
> and corner winding are still **OPEN** in the spec (§3.1): with the shared corner
> order the corner-derived normal comes out *antiparallel* to the env's
> toward-approach normal, so GT streams should ship an explicit `normal` until the
> convention is pinned.

## Handoff to 윤호 — what to fill in

1. **Reward weights are STUBS.** Reward *design* ownership is UNASSIGNED on the
   checklist. Tune `configs/reward_default.yaml` while watching the per-term
   breakdown the env logs (`info["reward_terms"]`) for reward hacking. The 5 keys
   are fixed; don't add/remove terms without updating `reward.REWARD_TERMS`.
2. **Plug in real training.** Install `stable-baselines3` → `train.py` auto-uses the
   PPO path (`_sb3_train`); wire the `algo`/hyperparams you want into
   `train_default.yaml`. The fallback loop does **not** learn — it only exercises the
   pipeline.
3. **`IsaacSimBackend`.** Implement `reset()`/`step()` against the same
   `PhysicsBackend` interface (spawn drone, PID-track the world waypoint, query
   collisions). The env, reward, eval, and baseline need **no** changes.
4. **Tune env/DR numbers.** `EnvConfig` defaults (dt, PID gains, `waypoint_scale`,
   `wall_half_extent`, scene ranges) and DR ranges are engineering defaults chosen
   for MockPhysics stability — revisit for the real dynamics + curriculum (7.3).
5. **Curriculum** is scaffolded in `train_default.yaml` (`curriculum.stages`) but the
   staged-override wiring in `train.py` is left for whoever runs the real training
   (the fields are parsed into `tcfg.raw`).
