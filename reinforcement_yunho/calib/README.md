# `calib/` — OpenVINS master config + Kalibr camera/IMU templates (checklist 0)

**Consumer:** 태민 (OpenVINS / VIO + `visual_imaging_taemin/window_recon_node.py`).
**Producer of the numbers:** 윤호 (camera intrinsics), the drone CAD / Isaac Sim
scene (extrinsics), and the IMU datasheet / Isaac Sim IMU sensor (noise).

태민 launches OpenVINS with

```bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```

where the `euroc_mav` config directory is a **master `estimator_config.yaml`**
that pulls in two Kalibr-style chains it references at the bottom
(`relative_config_imu` → `kalibr_imu_chain.yaml`, `relative_config_imucam` →
`kalibr_imucam_chain.yaml`). This directory holds those three files plus a helper
that stamps the camera intrinsics in from 윤호's `synth_intrinsics.yaml`, so the
calibration always matches the camera the images were actually rendered with.

## Files

| file | what it is | runnable now? |
|------|------------|---------------|
| `estimator_config.yaml` | OpenVINS master config. **MONO by default** (`max_cameras: 1`, `use_stereo: false`, cam0 only), `gravity_mag: 9.81`, tracking/init params, and a commented **STEREO** alternative. References the two chains via `relative_config_imu`/`imucam`. Plain YAML (not `%YAML:1.0`) so it validates with pyyaml. | data file |
| `kalibr_imucam_chain.yaml` | `cam0` block: pinhole intrinsics, radtan distortion, resolution, `cam_overlaps`, `T_cam_imu` extrinsics (**direction pinned**, see below), `timeshift_cam_imu`, `rostopic`. | data file (placeholders) |
| `kalibr_imu_chain.yaml` | `imu0` block: accel/gyro noise densities + random walks, `update_rate`, `model`, `time_offset`, `T_i_b`, `rostopic`. | data file (placeholders) |
| `fill_from_intrinsics.py` | pyyaml helper: given a `synth_intrinsics.yaml` in **길남's schema** `{width,height,fx,fy,cx,cy,distortion}`, writes a **copy** of the camchain with `intrinsics`/`resolution`/`distortion` filled; leaves `T_cam_imu` + IMU noise as TODO. | **yes** (numpy + pyyaml) |
| `smoke_test.py` | validates both chains + `estimator_config.yaml` (keys/types), and round-trips the `gen_intrinsics → fill` seam. | **yes** |

Nothing here imports Isaac Sim, OpenVINS, or ROS — the YAMLs are consumed by
those downstream tools; this directory only produces and validates them.

> **File format note.** OpenVINS parses configs with OpenCV FileStorage, whose
> deployed files conventionally start with a `%YAML:1.0` first line. We keep this
> repo's copies as **plain YAML** so `smoke_test.py` can load them with pyyaml on
> a numpy-only box; prepend `%YAML:1.0` when handing the config dir to a live
> OpenVINS build if its loader requires it. pyyaml also drops comments on
> round-trip, so `.filled.yaml` output is a plain-data artifact.

## Run it

```bash
# from repo root
python3 calib/smoke_test.py

# regenerate 윤호's intrinsics, then fill the camchain from them
python3 scripts/gen_intrinsics.py --width 1280 --height 720 --hfov 90 \
    --out vision/synth_intrinsics.yaml
python3 calib/fill_from_intrinsics.py \
    --intrinsics vision/synth_intrinsics.yaml \
    --template   calib/kalibr_imucam_chain.yaml \
    --out        calib/kalibr_imucam_chain.filled.yaml
```

## MONO vs STEREO

Upstream `euroc_mav` ships **stereo** (`max_cameras: 2`). Our Isaac Sim renders a
**single** camera, so `estimator_config.yaml` defaults to **mono**. To go stereo:
flip `use_stereo`/`max_cameras` in `estimator_config.yaml`, set
`cam0.cam_overlaps: [1]`, and add a `cam1` block (`cam_overlaps: [0]`,
`rostopic: /cam1/image_raw`) to `kalibr_imucam_chain.yaml`.

## Extrinsic direction is PINNED (read before filling `T_cam_imu`)

윤호 delivers exactly **ONE** physical camera↔IMU extrinsic, but it appears in two
conventions:

- **camchain (`kalibr_imucam_chain.yaml`) stores `T_cam_imu`**, the Kalibr/
  OpenVINS direction: `p_cam = T_cam_imu · p_imu` (IMU-frame point → camera frame).
- **태민's `window_recon_node.py` wants the INVERSE** (`T_IC`, cam-in-IMU):
  `p_imu = T_IC · p_cam` (i.e. the camera pose expressed in the IMU frame; there
  `R_WC = R_WI·R_IC`, `c_W = p_WI + R_WI·p_IC`).

So `T_IC = inverse(T_cam_imu) = T_imu_cam`. **Do not paste the same 4×4 into both
places** — the camchain gets `T_cam_imu`, the recon node gets its inverse.

## Intrinsics must ALSO reach 태민's node constants

`fill_from_intrinsics.py` writes fx/fy/cx/cy into the **camchain** yaml, but 태민's
`window_recon_node.py` **and** `window_sim_node.py` hard-code
`FX, FY, CX, CY = 600, 600, 640, 360`. Those constants must be updated to the same
numbers as `synth_intrinsics.yaml` / the camchain — the yaml alone is not enough,
because the recon/sim nodes back-project pixels with those literals.

## Who supplies which number

| field | source | who / how |
|-------|--------|-----------|
| `cam0.intrinsics` `[fx,fy,cx,cy]` | **윤호** — `synth_intrinsics.yaml` (길남 schema) → `fill_from_intrinsics.py` | run the helper; do **not** hand-edit. **Also** copy into 태민's node `FX/FY/CX/CY`. |
| `cam0.resolution` `[1280,720]` | **윤호** — same intrinsics YAML (CONVENTIONS.md fixes 1280×720) | run `fill_from_intrinsics.py` |
| `cam0.distortion_*` | synthetic camera is distortion-free (spec 6) → `radtan [0,0,0,0]` | run helper; keep zero unless distortion deliberately rendered |
| `cam0.cam_overlaps` | `[]` mono / `[1]` stereo | manual (mono default) |
| `cam0.T_cam_imu` (4×4) | **drone CAD / Isaac Sim scene** — Kalibr direction (`p_cam = T_cam_imu·p_imu`); 태민 inverts for the recon node | manual (CAD / sim scene) |
| `cam0.timeshift_cam_imu`, `imu0.time_offset` | 0.0 — camera and IMU share one clock (CONVENTIONS.md) | leave 0.0 unless latency is modelled |
| `cam0.rostopic` / `imu0.rostopic` | the Isaac Sim / rosbag publisher topics (`/cam0/image_raw`, `/imu0`) | match publisher |
| `imu0.*_noise_density`, `imu0.*_random_walk` | **IMU datasheet + Allan variance**, OR the **Isaac Sim IMU sensor** noise config | manual; often inflated 2–10× for VIO robustness |
| `imu0.update_rate` | IMU publish rate (200 Hz), must match the sim publisher | keep consistent with publisher |
| `estimator_config` `max_cameras`/`use_stereo` | mono (this project renders one camera) | flip for stereo (see above) |

### Units (also commented in `kalibr_imu_chain.yaml`)
- `accelerometer_noise_density` — **m/s²/√Hz**
- `accelerometer_random_walk` — **m/s³/√Hz**
- `gyroscope_noise_density` — **rad/s/√Hz**
- `gyroscope_random_walk` — **rad/s²/√Hz**

## Checklist question — when are the Isaac Sim IMU noise parameters set, and why they must equal what OpenVINS is told

This is a **simulator**, so there is no physical IMU and no datasheet in the loop.
The IMU noise is *created* by the Isaac Sim IMU sensor at data-generation time and
*modelled* by OpenVINS at estimation time. Those are two separate places that hold
the **same four numbers**, and the pipeline is only self-consistent if they agree.

**Order of operations (single source of truth):**

1. **Decide the noise numbers once.** Pick the four densities/random-walks up
   front — either from the real IMU you intend to emulate (datasheet + Allan
   variance) or as a chosen simulated noise level. Record them (a small YAML /
   constants file that both sides read is ideal so they can never drift).
2. **Set them on the Isaac Sim IMU sensor at scene/sensor-config time**, *before*
   generating any rosbag/trajectory. Isaac Sim's IMU sensor injects Gaussian
   white noise + a random-walk bias using exactly these parameters; if the sensor
   is configured noise-free, the recorded IMU stream has none, and no OpenVINS
   tuning can recover what was never there.
3. **Copy the identical numbers into `kalibr_imu_chain.yaml`** (the `imu0` block)
   that OpenVINS reads. OpenVINS uses them to weight IMU pre-integration against
   the visual tracks.

**Why they must be equal:** the density in `imu.yaml` is OpenVINS's *assumption*
about how noisy the measurements are. If Isaac Sim injects more noise than
OpenVINS assumes, the filter is over-confident in the IMU → drift/divergence; if
it injects less, the filter is under-confident → it ignores good IMU information
and leans too hard on vision. Either way the covariance is miscalibrated. In sim
you have the luxury of knowing the ground-truth noise exactly, so make the two
match by construction (same source constants), then — as on real hardware —
optionally **inflate** the OpenVINS values by 2–10× for robustness, deliberately
and documented, rather than by accident.

**Practical note:** watch the **units/parametrisation**. A datasheet or an Isaac
Sim field may express noise per-sample, in °/s/√Hz, or as a discrete standard
deviation; Kalibr/OpenVINS want continuous-time SI densities in the units listed
above. Convert once, at step 1, so the number that lands in the sim sensor and
the number that lands in `imu.yaml` are literally the same quantity.

## Handoff to 태민

1. Run `gen_intrinsics.py` → `synth_intrinsics.yaml`, then `fill_from_intrinsics.py`
   → `kalibr_imucam_chain.filled.yaml` (intrinsics/resolution/distortion done).
2. Fill `cam0.T_cam_imu` from the drone CAD / Isaac Sim scene graph (Kalibr
   direction); **태민 inverts it** for `window_recon_node.py`'s `T_IC`. Also copy
   fx/fy/cx/cy into the node `FX/FY/CX/CY` constants.
3. Fill the four `imu0` noise values to match the Isaac Sim IMU sensor config;
   set `rostopic`s to the actual publishers; keep `time_offset`/`timeshift` at 0.0.
4. Drop `estimator_config.yaml` + both chains into the OpenVINS `euroc_mav` config
   dir (prepend `%YAML:1.0` if the build's loader needs it); mono by default.
