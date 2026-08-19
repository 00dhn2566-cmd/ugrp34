# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

UGRP (undergraduate research) monorepo for "현수 하중 드론의 강건 통합 비행 제어 시스템 연구" (DGIST) — a drone that autonomously flies through arbitrarily-placed, color-labeled windows in an Isaac Sim environment. Full context/goals/roles/milestones live in the root [README.md](README.md).

The repo is **four independent student workstreams** glued together by one shared interface spec, not a single buildable application. There is no root build/lint/test command — each subfolder has its own toolchain (MATLAB/Simulink, Python/pytest, ROS2/colcon). Work inside the relevant subfolder.

```
카메라/IMU → [비전: 창문 탐지] → [VIO: 상태추정·3D 복원] → [경로계획: RL] → [저수준 제어: PID] → 드론
```

| Folder | Owner | Role | Stack |
|---|---|---|---|
| `overall_gilnam/` | 류길남 | Pipeline lead + vision (window detection, color ID) | Python, YOLO-pose |
| `visual_imaging_taemin/` | 박태민 | VIO / state estimation | OpenVINS (ROS2, C++), submodule |
| `control_seoungjin/` | 박성진 | Low-level PID control, trajectory generation | MATLAB/Simulink, Python |
| `reinforcement_yunho/` | 조윤호 | Isaac Sim environment, dataset generation, RL path planning | planning docs only so far (`docs/`) |
| `simul/` | 조윤호 | Isaac Sim itself (vendored upstream `isaac-sim/IsaacSim`) | submodule, not our code |

**The binding contract across all of this is [window_detection_spec_v0.2.md](window_detection_spec_v0.2.md)** at the repo root — read it before touching vision, VIO input, or dataset generation code. Key confirmed decisions from it:
- Coordinates are always in original 1280x720 pixel space; internal model resize (640) is never leaked downstream.
- Color→order mapping lives in config (`overall_gilnam/vision/color_order.yaml`), never hardcoded.
- The vision→VIO message format (§5) is fixed JSON (`timestamp`, `frame_id`, `windows[].{order_index, color, corners, corner_vis, center, det_conf, color_conf}`) — depth is intentionally excluded (VIO does 3D reconstruction via triangulation; sim depth would be a cheat that breaks on real hardware).
- Both the trained-model output and the GT-label stream must produce byte-compatible §5 messages so swapping one for the other requires no downstream changes (see `overall_gilnam/vision/gt_stream.py`).

Path planning was switched from optimization (ALM) to RL (README §7) — the RL policy only replaces the path-planning layer (outputs waypoints/reference trajectory), PID still does low-level control.

## `overall_gilnam/` — vision (window detection + color ID)

Data flow: `[training] YOLO-pose detection → color_judge.py` or `[pre-training] sim GT labels → gt_stream.py`, both converging on `vision_msg.py` which builds the §5 JSON sent to VIO.

- `vision/color_order.yaml` — color↔order + HSV thresholds (the single source of truth §3.1 config)
- `vision/window_pose.yaml` — YOLO-pose dataset definition (§4.3); `path:` must be filled in locally after the dataset arrives (dataset is not checked into git)
- `vision/color_judge.py` — HSV color judging: samples a border band around the corners (not bbox interior, which is the window's open interior/background)
- `vision/vision_msg.py` — the only place that builds §5 output messages; both model and GT stream funnel through it
- `vision/gt_stream.py` — converts §4.3 GT label txt → §5 messages for pipeline validation before the model is trained
- `vision/model_decisions.md` — 7 locked-in model architecture decisions (base model `yolo11s-pose`, `single_cls=True` with color handled entirely by HSV post-processing, `flip_idx: [1,0,3,2]` required to avoid corrupting corner order under augmentation, `imgsz=640` with full-res 1280x720 inference)

Reference training command (from `model_decisions.md`):
```bash
yolo pose train model=yolo11s-pose.pt data=window_pose.yaml imgsz=640 single_cls=True epochs=100
```

Tests (run from `overall_gilnam/vision/`):
```bash
python -m pip install -r requirements.txt
python -m pytest tests/ -q
```

## `control_seoungjin/` — PID control & trajectory generation

Two parallel toolchains: Python generates trajectories, MATLAB/Simulink simulates them.

- `path_time.py` — reusable module (arc-length reparameterization → curvature → velocity profile → time parameterization → PID feed) extracted from `path_time.ipynb`. Key functions: `reparameterize_by_arc_length`, `compute_curvature_and_kN`, `generate_velocity_profile`, `generate_pid_reference`, `plan_waypoints`/`plan_trajectory`.
- `controller/Quadcopter-Drone-Model-Simscape/` — MathWorks Simscape drone model (submodule), tuned for the FX450 frame (CAD swapped in for the generic geometry). **Do not run `git submodule update --init` on this path** — it would overwrite the FX450 CAD changes with upstream MathWorks content, which is instead distributed as a zip and unpacked manually over this folder.
- `sample/run_and_log.py` — full pipeline: read `config.yaml`/`config.json` (waypoints + limits + dt, see [INPUT_FORMAT.md](control_seoungjin/sample/INPUT_FORMAT.md)) → generate trajectory → run MATLAB batch sim → save CSV/JSON outputs to `sample/output/` (gitignored).
- **Current WIP** (see [HANDOFF_PATH_TO_CONTROLLER.md](control_seoungjin/docs/HANDOFF_PATH_TO_CONTROLLER.md), written 2026-07-13; handoff docs live under `control_seoungjin/docs/`): a second, parallel pipeline — JSON path → `path_time.py` time-parameterization → baked Simscape model → controller commands + tracking results — separate from `sample/run_and_log.py`. Glue script `controller/Quadcopter-Drone-Model-Simscape/run_traj_baked.m` exists (validated by the path_time session); the front half is now `traj_pipeline.py` (built by the path_time session, contract in `control_seoungjin/INTERFACE_SPEC.md`). Read the handoff docs before continuing this thread.

Run the full pipeline:
```bash
python control_seoungjin/sample/run_and_log.py --config control_seoungjin/sample/config.yaml
```

Individual steps (debugging):
```bash
cd control_seoungjin/sample
python waypoints_to_maneuver_input.py     # generates trajectory.mat
python verify_sample_trajectory.py        # verifies v_max/a_max/j_max, plots sample_trajectory.png
./run_sample_sim.sh                       # trajectory generation + MATLAB batch run
```

Requirements: MATLAB R2025b+ with Simulink, Simscape, Simscape Multibody, Simscape Electrical, and **Simscape Driveline** (required for the `Aerodynamic Propeller` block — missing it causes library load errors in the `Propeller 1~4` subsystems). Python needs numpy, scipy, matplotlib.

MATLAB executable is auto-detected (`MATLAB_EXE` env var → PATH → `C:\Program Files\MATLAB\`); override if multiple versions are installed. On this machine R2026a is the one with an actual executable (an R2025b folder exists but is not runnable):
```bash
MATLAB_EXE="/c/Program Files/MATLAB/R2026a/bin/matlab.exe" python control_seoungjin/sample/run_and_log.py
```

**The model is "baked" — treat `Models/quadcopter_package_delivery.slx` as tuned and load-bearing.** Anchor compensation (`Plate Anchor Comp`), thrust-bias rescale, altitude clamp, and x/y position-error saturation are all saved inside the file; loading it and running a bare hover is already stable (10s, attitude RMS 0.56°, verify with `diagnose/verify_hover.m` in the submodule). **Never `save_system` on it** for experiments — make in-memory edits and close without saving. Full incident/tuning history: `controller/Quadcopter-Drone-Model-Simscape/TUNING_STATUS.md`.

Gotchas (see [COMMANDS.md](control_seoungjin/COMMANDS.md) for more):
- `File Solid` CAD blocks store filename only, so `addpath(genpath('CAD'))` is required in addition to `Scripts_Data`/`Models`/`Libraries`, or unrelated CAD files fail to resolve.
- `waypoints` variable must be 3×N for the `Ground/Trajectory/Waypoints` block (`unique(waypoints','rows')`), but Python saves it as N×3 — transpose in MATLAB. `spline_data` is the opposite: used as N×3 directly.
- Trajectory variables (`timespot_spl`, `spline_data`, `spline_yaw`, `waypoints`) must be `assignin`'d into the **model workspace** (`get_param(mdl,'ModelWorkspace')`), not the base workspace, or the Lookup Table blocks won't see them.
- Attitude PID gains are negative on purpose (`kp_attitude=-100`, `kd_attitude=-150`) — the measured plant gain (u→pitch accel) is negative. This is not a bug; "fixing" the sign makes the sim diverge immediately.
- `quadcopter_drone_arm.stp` must never be rotated — its Transform ZXZ rotation + custom inertia are aligned to the file's current orientation. The two plate STEPs (`plate_top`/`plate_bottom`) *were* intentionally re-saved lying down to fix a roll-flip bug (see TUNING_STATUS.md); don't "fix" that back either.
- Git Bash POSIX paths need `cygpath -w` conversion before passing to native Windows MATLAB.
- PID gains live in `Scripts_Data/quadcopter_package_parameters.m` (`kp_position/kp_attitude/kp_yaw/kp_altitude/kp_motor`), tunable from the `Maneuver Controller` subsystem in `Models/quadcopter_package_delivery.slx`.
- `diagnose/` scripts under the controller submodule are one-off debugging tools (block/port/signal introspection), not part of the pipeline.
- **RAM**: dev machine has 16GB. One batch sim ≈ 2–4GB — never run two concurrently. Baking the model (double compile) needs 6–8GB+ and has crashed this machine before; close other apps first or check with the user.
- The submodule tracks its own branch (`fix/plate-orientation-cg`, pushed to the fork as `fix/plate-orientation-cg-workload` to avoid colliding with the shared public repo). Claude's push access is restricted here — hand the user the push command rather than pushing directly.

## `visual_imaging_taemin/` — VIO (OpenVINS)

`openvins_source/` is the `rpng/open_vins` submodule (ROS2/C++, MSCKF-based VIO), run inside WSL2 + ROS2 Jazzy — this does not run natively on Windows. See [commands/README.md](visual_imaging_taemin/commands/README.md) / `setup_1to5.sh` (install + build) and `run_6to10.sh` (run + evaluate) for the full command sequence: build with colcon → replay a EuRoC MAV rosbag → convert output to TUM format → evaluate ATE with `evo`.

`dataset/` expects EuRoC MAV sequences downloaded manually (not committed — too large); place under `dataset/<sequence_name>/`.

Two standalone ROS2 nodes implement window 3D reconstruction, both run under the same WSL2 + ROS2 Jazzy environment as OpenVINS:
- `window_sim_node.py` — stand-in detector: subscribes `/ov_msckf/poseimu`, projects hardcoded window definitions into the camera view, publishes §5.1 JSON on `/window_detections`. Gets swapped for the real vision detector once that's ready.
- `window_recon_node.py` — the real reconstruction node: subscribes `/window_detections` + `/ov_msckf/poseimu`, time-aligns them (20ms tolerance), accumulates per-corner sightlines into a least-squares estimator, and every 2s solves + publishes triangulated window center/size on `/window_positions`.

Both files currently hardcode placeholder camera intrinsics/extrinsics (marked `TODO: §6 확정되면 교체`) — swap to real values once the vision/VIO camera spec (README §4, VIO section) is finalized.

## `reinforcement_yunho/` & `simul/` — Isaac Sim + RL (planning stage)

No code yet — `reinforcement_yunho/docs/` holds the planning material:
- `To_do_checklist_yunho.md` — sim environment / dataset-generation task list.
- `gpu_jobs_yunho.md` — spec for batching the team's heavy compute onto 윤호's GPU cluster (20× 40GB). Key constraint: an A100 (no RT cores) can't run Isaac Sim's RTX renderer / Replicator, so **dataset rendering stays on a local RTX machine and the cluster is training-only** unless Job 0 confirms RTX-class GPUs. Job 1 is 길남's YOLO-pose training (pin `ultralytics==8.4.87`); Jobs 2–4 are RL smoke test → reward/HP sweep → eval batch (≥3 seeds per config).

No team laptop meets Isaac Sim's minimum spec (RTX 4080+), so Isaac Sim work happens on rented cloud GPUs via SSH — see [cloud_gpu_ssh_setup.md](cloud_gpu_ssh_setup.md) for the setup TODO and [cloud_services_quote.txt](cloud_services_quote.txt) for the cost comparison. Decision: **Paperspace Core** (persistent VM + browser remote desktop) for dev/scene-building/smoke tests, **RunPod** (per-second container billing) for large parallel RL sweeps.

`simul/` is the full [`isaac-sim/IsaacSim`](https://github.com/isaac-sim/IsaacSim) engine vendored as a submodule — huge upstream tree with its own `build.sh`/`build.bat` and its own `CLAUDE.md`/`AGENTS.md`. It is *not* our code; don't audit, reformat, or apply repo conventions to it.

## Cross-cutting notes

- All docs and comments in this repo are primarily in Korean; match that when editing existing docs.
- Large binary/generated artifacts (datasets, `sample/output/`, `.mat`/CSV results) are intentionally kept out of git — check `.gitignore` before assuming a missing file is an error.
- Three git submodules: `visual_imaging_taemin/openvins_source`, `control_seoungjin/controller/Quadcopter-Drone-Model-Simscape`, and `simul` (Isaac Sim). The Simscape one specifically must NOT be re-initialized from upstream (see above — the FX450 CAD lives on top of it, distributed as a zip).
