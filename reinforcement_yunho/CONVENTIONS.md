# Shared conventions (the contract every module obeys)

This project scaffolds the **simulation + dataset + RL** side of the window-
traversal drone (윤호's part). Files are grouped by consumer:

| dir          | consumer            | purpose                                            |
|--------------|---------------------|----------------------------------------------------|
| `common/`    | everyone            | intrinsics + projection math (numpy-only, tested)  |
| `vision/`    | 길남 (YOLO-pose)    | dataset generation, YOLO-pose labels (spec 4)      |
| `calib/`     | 하민 (OpenVINS/VIO) | Kalibr camera+IMU YAML templates (checklist 0)     |
| `interface/` | 성진 (control)      | motor-command / trajectory JSON schemas            |
| `rl/`        | 윤호 (RL)           | gym env, reward, training, evaluation (spec 7)     |

Everything importable/testable on a plain machine uses **numpy + pyyaml only**.
Heavy deps (Isaac Sim `omni.replicator`, `gymnasium`, `stable-baselines3`,
`ultralytics`) are import-guarded so the pure-Python logic still runs and tests
without them.

## Coordinate frames
- **World**: Isaac Sim default — right-handed, **+Z up**, metres.
- **USD camera**: looks down **-Z**, +X right, +Y up (what Isaac Sim reports as
  the camera prim pose `T_world_cam_usd`).
- **CV camera**: looks down **+Z**, +X right, +Y down — the frame we project in.
  Conversion: `R_USD_TO_CV = diag(1,-1,-1)` (in `common/geometry.py`).
- **Image**: +u right, +v down, origin top-left.

Do **not** re-derive projection. Use `common.geometry`:
`world_to_camera_cv(points_world, T_world_cam_usd)` then
`project_points(points_cam_cv, K)`. Intrinsics come from
`common.intrinsics.CameraIntrinsics`.

## Window model
- A window is a rectangular opening with width×height (metres), in its own local
  frame: +X_local right, +Y_local up, +Z_local = outward normal (drone-approach
  side).
- **Corner order is fixed and geometric** (NOT image-position based), so identity
  is stable at oblique angles (spec 4.1 allows ±60°):

  `0 top_left → 1 top_right → 2 bottom_right → 3 bottom_left`  (CW from front)

  Use `common.geometry.window_corners_world(...)` / `window_corners_local(...)`
  and `common.geometry.CORNER_ORDER`.

## Colour / class mapping (traversal order_index)
`red = 0, green = 1, blue = 2`. The YOLO-pose `class` field **is** this
order_index. Define it once as `ORDER_INDEX = {"red":0,"green":1,"blue":2}`.

## YOLO-pose label format (spec 4.3)
One line per fully-visible window, all values normalised to [0,1] by image size:

```
<class> <cx> <cy> <w> <h> <u1> <v1> <vis1> <u2> <v2> <vis2> <u3> <v3> <vis3> <u4> <v4> <vis4>
```

- `class` = order_index (red 0 / green 1 / blue 2)
- `cx cy w h` = axis-aligned bbox tight around the 4 projected corners
- keypoints in CORNER_ORDER; `vis` = 1 for all corners in **dataset 1** (policy A).
  Keep the `vis` field present from day one so the optional **dataset 2**
  (policy C: off-screen/occluded corners → estimated coord + `vis=0`) needs **no
  format change**.
- Dataset 1 labels only windows where **all 4 corners are inside the frame**.

## Dataset layout & split
```
<dataset_root>/
  images/{train,val,test}/*.png
  labels/{train,val,test}/*.txt      # same stem as the image
```
Split **80/10/10**. 길남 **owns** `overall_gilnam/vision/window_pose.yaml`
(`names: {0: window_red, 1: window_green, 2: window_blue}`, `kpt_shape [4,3]`,
`flip_idx [1,0,3,2]`, no `nc`) — 윤호 only sets its `path:` to `<dataset_root>`.
Do not ship a competing copy. Likewise `synth_intrinsics.yaml` uses 길남's exact
schema: `{width, height, fx, fy, cx, cy, distortion: []}` (field is `distortion`,
an empty list = no distortion — **not** `distortion_model`/`distortion_coeffs`).

## Trajectory / motor-command interface (with 성진)
- Control→sim boundary is the **per-motor angular-velocity setpoint (rad/s)**;
  motor/propeller physics is Isaac Sim's job.
- `interface/isaacsim_motor_commands.schema.json`:
  `{fps, frames:[{time, motor_cmd_w:[w1,w2,w3,w4]}]}` — **`time` is FLOAT SECONDS**
  (성진's real output), motor `fps` is variable (per-frame `time` is authoritative).
- `interface/isaacsim_trajectory.schema.json` (성진 output schema):
  `{fps, frames:[{time(float s), position:[x,y,z], yaw_rad, orientation_quat_wxyz:[w,x,y,z]}]}`
- 성진's controller **input** (where RL plugs in) is a separate
  `interface/waypoints_config.schema.json`: `{waypoints:[[x,y,z]…] N≥2 m world z-up,
  limits:{v_max,a_max,j_max,snap_max} scalar-or-[x,y,z], dt(s)}` — no yaw/time.
- Rotor index→geometry + CW/CCW spin sign for `motor_cmd_w[4]` is a **윤호 decision**
  (Isaac rotor config); publish it so 성진's order matches.

## Quaternion order is per-interface (NOT global — this bit me once)
| interface | order | note |
|---|---|---|
| 성진 control trajectory JSON | **WXYZ** | `orientation_quat_wxyz`, yaw-only |
| EuRoC-ASL GT `data.csv` (→ 태민 eval) | **WXYZ** | `q_w,q_x,q_y,q_z` |
| 길남 vision GT-pose stream, `state_window_interface` drone state | **XYZW** | `[qx,qy,qz,qw]` |
| 태민 live `/ov_msckf/poseimu` (ROS `geometry_msgs`) | **XYZW** | Hamilton |
`common.geometry.quat_wxyz_to_R` takes WXYZ — reorder XYZW inputs before calling it.

## RL boundaries (spec 7)
- Policy output = **waypoint / reference trajectory**; a low-level PID follows it.
  End-to-end motor control is **out of scope this semester**.
- **No sim cheating**: GT depth / GT pose must not be fed directly as observation
  (spec 7.1). Start from clean observation, add an *option* to inject estimator
  noise (spec 7.6).
- Reward weights live in a **separate yaml config** (spec 7.2), never hard-coded:
  window-pass reward / collision penalty / progress / attitude·energy penalty.
- Eval protocol (spec 7.4): random scenes → success rate, collision rate, mean
  pass time; always compared against a **simple-waypoint baseline** on the same
  scene set.

## Timestamps
- **Sensor / vision / VIO side = integer nanoseconds** on one shared cam+IMU clock:
  the §5 vision message, the flight-data bag (`/cam0/image_raw`, `/imu0`), the
  EuRoC-ASL GT `data.csv`, and `state_window_interface`. `vision_msg.py` *raises*
  on a float timestamp.
- **성진 control JSON side = float seconds** (`isaacsim_trajectory.json` /
  `isaacsim_motor_commands.json`). Do not conflate the two.

## "spec §7" means README §7
The RL notes my code cites as "spec 7.x" are **`README.md §7`** (RL adoption
considerations), not `window_detection_spec_v0.2.md`. `window_detection_spec` is
the dataset/§5 contract (its §-numbers are the dataset ones).

## Reproducibility
Every generated artifact records: seed, config file, and (when in git) commit
hash. RL run folders: `run_<config>_<seed>_<hash>/`.
