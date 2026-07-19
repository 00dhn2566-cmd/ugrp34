# Isaac Sim 4.5 window-dataset generation — end to end

Render the drone "window traversal" dataset headlessly in the Isaac Sim 4.5
apptainer container, then assemble the YOLO-pose dataset offline (no GPU) and hand
it to 길남. Three scripts are involved:

| step | script | where it runs |
|------|--------|---------------|
| 1. render frames + metadata | `sim/isaac_replicator.py` (via `sim/run_isaac_dataset.sh`) | **inside** the Isaac Sim 4.5 container, RTX GPU |
| 2. build labels + splits | `sim/export_dataset.py --mode from-metadata` | anywhere (numpy + pyyaml) |
| 3. train / eval | 길남's pipeline (`window_pose.yaml`) | 길남's env |

The two stages are decoupled by **per-frame metadata JSON** (`sim/metadata_schema.md`).
`isaac_replicator.py` emits exactly what `export_dataset.py` reads, so the render
and the labels always agree.

---

## 0. Prereqs

- An RTX GPU (target: RTX PRO 6000) and the NVIDIA driver on the host.
- `apptainer/1.5.0` (HPC module).
- NGC access to pull `nvcr.io/nvidia/isaac-sim:4.5.0` (needs an NGC API key for
  `docker://` pulls; run `apptainer remote login docker://nvcr.io` or set
  `SINGULARITY_DOCKER_USERNAME='$oauthtoken'` + `SINGULARITY_DOCKER_PASSWORD=<NGC key>`).

## 1. Pull the container once

```bash
source /etc/profile.d/modules.sh
module load apptainer/1.5.0
export APPTAINER_CACHEDIR=/scratch/$USER/.apptainer_cache
export APPTAINER_TMPDIR=/scratch/$USER/.apptainer_tmp
mkdir -p "$APPTAINER_CACHEDIR" "$APPTAINER_TMPDIR"

# ~15-20 GB; put the .sif on /scratch
apptainer pull /scratch/$USER/isaac-sim_4.5.0.sif docker://nvcr.io/nvidia/isaac-sim:4.5.0
```

## 2. Render the dataset (GPU, inside the container)

```bash
# args: NUM_FRAMES  OUT_DIR  SEED  [SIF]
sim/run_isaac_dataset.sh 500 /scratch/$USER/window_ds 0
# or point at a non-default .sif:
SIF=/scratch/$USER/isaac-sim_4.5.0.sif sim/run_isaac_dataset.sh 500 /scratch/$USER/window_ds 0
```

Output:

```
/scratch/$USER/window_ds/
├── frames/frame_000001.png …     # RGB, 1280x720, RTX-rendered
├── meta/frame_000001.json  …     # per sim/metadata_schema.md
└── _assets/bg_noise.png          # generated background texture (feature-rich)
```

What the wrapper sets up (see `sim/run_isaac_dataset.sh` for the exact flags):
- `apptainer exec --nv` (GPU) with `--bind /sfs:/sfs,/scratch:/scratch`.
- in-container python entrypoint **`/isaac-sim/python.sh`**.
- EULA / headless env: `ACCEPT_EULA=Y`, `OMNI_KIT_ACCEPT_EULA=YES`,
  `PRIVACY_CONSENT=Y`, empty `DISPLAY` (offscreen EGL).
- persistent Kit shader/asset caches on `/scratch` so re-runs skip recompilation.

### What `isaac_replicator.py` does per frame
1. `scene = sim.scene_gen.sample_scene(seed+i, intr)` — 1-5 windows, near/mid/far,
   ±60° tilt, randomised lighting, mandatory textured background (spec §4.1).
2. Sets the USD **camera prim** transform to `scene["camera"]["T_world_cam_usd"]`
   and its focal length / aperture so `fx=fy=600, cx=640, cy=360 @ 1280x720`
   (길남's `synth_intrinsics`). Builds each **window** as a thin coloured box whose
   material lands inside `color_order.yaml`'s HSV band, oriented with
   `window_rotation_from_normal(normal)` — the same rotation `export_dataset`
   rebuilds — so rendered corners == labelled corners. Randomises the lights.
3. `rep.orchestrator.step(rt_subframes=…)` → `rgb_annot.get_data()` → saves the PNG.
4. `scene_to_metadata(scene, image="../frames/<stem>.png")` → writes the JSON.

Tuning knobs (pass through `python.sh … isaac_replicator.py`):
`--num-frames --out --seed --rt-subframes --horizontal-aperture --clutter --start-index`.

## 3. Build the YOLO-pose dataset (no GPU)

`export_dataset.py` finds each image from the JSON's `image` field (we wrote it
relative to `meta/`, pointing at `frames/`), so there is **no `--images-dir` flag**:

```bash
python3 sim/export_dataset.py --mode from-metadata \
    --metadata-dir /scratch/$USER/window_ds/meta \
    --out /scratch/$USER/window_dataset_root
```

Produces `images/{train,val,test}`, `labels/{train,val,test}`, `meta.jsonl`
(길남's `eval_corners` schema) and `dataset_manifest.json` (80/10/10 seeded split).

## 4. Hand to 길남

Point 길남's `window_pose.yaml` `path:` at `/scratch/$USER/window_dataset_root`
(the dir with `images/` + `labels/`). `meta.jsonl` lets 길남 run `eval_corners`
(distance-binned corner error) on the real dataset unchanged.

---

## Verifying render↔label agreement (recommended smoke check)

After a small run (e.g. `--num-frames 5`), overlay a label on its image:

```bash
python3 sim/visualize_labels.py \
    --image  /scratch/$USER/window_ds/frames/frame_000001.png \
    --label  <dataset_root>/labels/<split>/frame_000001.txt   # after step 3
```

The drawn window corners should sit on the rendered coloured boxes. If they are
consistently offset, the camera pose or intrinsics need a look (see below).

---

## Things most likely to need a fix on the first real run

1. **Renderer string.** We use `"RaytracedLighting"` (NVIDIA rendering_modes docs).
   The brief said `"RayTracedLighting"`. If `SimulationApp` errors on the config,
   flip the spelling in `isaac_replicator.py` (`_RENDER_CONFIG["renderer"]`).
2. **Intrinsics units.** We set `focalLength`/`horizontalAperture` on a raw
   `UsdGeom.Camera` and rely on `fx = focalLength/horizontalAperture * width`
   (ratio-only, unit-invariant). The script prints `fx_eff` at startup and warns
   on mismatch. If labels are slightly scaled, cross-check with
   `isaacsim.sensors.camera.Camera(...).get_intrinsics_matrix()`.
3. **Camera pose round-trip.** The script prints the max error between the emitted
   `T_world_cam_usd` and `ComputeLocalToWorldTransform` on frame 0. It should be
   ~0. If not, the USD↔math transpose in `_gf_matrix_from_np` is the place to look.
4. **Colour under lighting.** Window materials are strongly-saturated primaries +
   matching-hue emissive, but aggressive lighting randomisation could still push a
   rendered pixel out of `color_order.yaml`'s band. Sample a few frames through
   길남's `color_judge` and, if needed, raise the emissive fraction in `WINDOW_RGB`
   handling or narrow `_sample_lighting`'s brightness range.
5. **PIL availability.** Frame saving + texture generation use `PIL` (bundled with
   Isaac Sim's `python.sh`, per NVIDIA's own SDG snippets). If missing, frames fall
   back to `.npy` and walls to a flat material — install Pillow into the Kit python.
6. **`rep.orchestrator.step` vs `simulation_app.update()`.** If annotator data
   comes back empty/black, add a couple of `simulation_app.update()` calls before
   the first `step`, or raise `--rt-subframes`. Some builds also need
   `rep.orchestrator.step()` called once before attaching annotators.
7. **Occlusion of labelled windows.** Labels are geometric (all-4-corners rule), so
   a clutter prop or wall between camera and a window would still label it while the
   pixels are occluded. Clutter is biased toward far/side walls to limit this; drop
   `--clutter 0` if you see occluded-but-labelled windows.
8. **argv handling.** `SimulationApp` shares `sys.argv` with Kit; we parse with
   `parse_known_args()` before launching. If Kit complains about unknown args, strip
   the custom flags from `sys.argv` before `SimulationApp(...)`.
