# `sim/` — Isaac Sim environment + dataset/flight-data generation (윤호)

Produces everything the other parts need out of the simulator: the YOLO-pose
**detection dataset** (→ 길남), the **§5 GT-pose stream** (→ 태민), and the
**EuRoC-ASL flight bag** (→ 태민 / OpenVINS). The scene sampling, corner
projection, dataset split, CSV and stream writers are **pure numpy + pyyaml** and
run/tested on any machine; the Isaac Sim (`omni.replicator`) pieces are
import-guarded stubs.

Consumer / spec map: `window_detection_spec_v0.2.md` §2–§5, `CONVENTIONS.md`,
`docs/To_do_checklist_yunho.md` (items 0 + 1 + 2).

## Files

| file | what it is | runs now? |
|------|-----------|-----------|
| `scene_gen.py` | domain-randomised scene sampler (spec §4.1): 1–5 windows, near/mid/far, ±60° yaw/pitch, random lighting, uniform colour, **mandatory textured background**. Returns build-ready windows + camera pose (CV **and** USD). Isaac Replicator graph builder = guarded stub. | **yes** (pure sampler) |
| `replicator_writer.py` | pure `build_label_lines` / `build_label_records` (§4.3, 17-token) + the `omni.replicator` `WindowCornerWriter`. | pure core **yes**; Writer needs Isaac |
| `export_dataset.py` | from per-frame metadata **or** an Isaac run → `images/labels/{train,val,test}` 80/10/10 (seeded) + **`meta.jsonl`** (길남's `eval_corners` schema) + `dataset_manifest.json`. | from-metadata **yes**; isaac = stub |
| `export_vio.py` | EuRoC-ASL bag (`mav0/cam0`, `imu0`, `state_groundtruth_estimate0`) — int-ns timestamps, IMU 200 Hz, cam ~20 Hz, GT quaternion **WXYZ**. | **yes** (CSV/yaml writers) |
| `export_stream.py` | §5 GT-pose JSONL stream for 태민, **routed through 길남's `gt_stream`** (never hand-rolled). | **yes** (needs `overall_gilnam/vision`) |
| `visualize_labels.py` | draw a YOLO-pose label over its image (corner-order/normalisation eyeball). mpl→pil→svg fallback. | **yes** |
| `metadata_schema.md` | the per-frame metadata JSON `export_dataset` consumes / Isaac dumps. | doc |
| `smoke_test.py` | end-to-end pure-logic smoke (scene → labels → dataset+meta → EuRoC → stream). | **yes** |

## Run it (pure logic, this machine)

```bash
cd /sfs/gpfs/tardis/home/pcn3tv/ugrp34/reinforcement_yunho
python3 sim/smoke_test.py            # scene→labels→dataset+meta.jsonl→EuRoC→§5 stream
# offline dataset from a dir of per-frame metadata JSON (sim/metadata_schema.md):
python3 sim/export_dataset.py --mode from-metadata --metadata-dir <dir> --out <dataset_root> --seed 0
```

## Needs Isaac Sim (run inside the Isaac python env)

- `scene_gen.build_replicator_graph` — realise §4.1 randomisation as a Replicator
  graph (windows / camera / lighting / **textured background**), driving the writer.
- `replicator_writer.WindowCornerWriter` / `export_dataset.py --mode isaac` — render
  RGB + dump per-frame GT; then split with `split_indices` and dump `meta.jsonl`
  exactly as the from-metadata path does.
- The actual `.png` frames for both the dataset and the EuRoC `cam0/data/` come
  from the Isaac render; the pure writers here only lay out + index them.

## Scene material / background requirements (hard constraints)

1. **Window materials must render inside the `color_order.yaml` HSV bands** so
   길남's `color_judge` succeeds: red `H∈[0,10]∪[170,179]`, green `H∈[50,70]`,
   blue `H∈[110,130]`, with **S ≥ 100, V ≥ 80** (spec §3.1). Use saturated primary
   materials, robust to the lighting randomisation. These bands may be nudged after
   길남 checks real renders (spec §7) — the config is the single source, code
   unchanged.
2. **Background must be TEXTURED, never blank** (박태민 07/03): VIO needs
   feature-rich texture to triangulate. Modelled as a required scene param
   (`scene["background"]["kind"] == "textured"`; `validate_scene` rejects blank).
3. **…but the background must stay OUT of the saturated primary bands** (keep
   `S < 100`, i.e. desaturated textures) so it does not trip `color_judge`. This is
   the one tension between 태민 (wants texture) and 길남 (wants clean colour) — a
   textured **but desaturated** background satisfies both.

## Coordinate / projection contract (must match 길남 + §5)

world Z-up X-forward (m) · camera OpenCV `+Z` optical / `+X` right / `+Y` down ·
pose `T_world_cam` · window corners TL→TR→BR→BL. `build_label_lines` projects via
`common.geometry.world_to_camera_cv`, which applies the **USD→CV flip**, so it
takes the *USD* camera-prim pose; `scene_gen.cv_to_usd_transform` bridges a CV
pose (`R_usd = R_cv @ diag(1,−1,−1)`). `smoke_test.py` cross-checks that the
resulting pixels equal 길남's documented formula `X_cam = R_wc^T (X − t)`
(`overall_gilnam/vision/synth_scene.py::project`).

Quaternion order is **per interface** (CONVENTIONS.md): §5 stream pose = **XYZW**
(`orientation:[qx,qy,qz,qw]`), EuRoC GT `data.csv` = **WXYZ** (`q_w,q_x,q_y,q_z`).
`scene_gen` exposes both (`camera["quat_xyzw"]`, `camera["quat_wxyz"]`).

## Handoffs

| to | artifact | how |
|----|----------|-----|
| **길남** | detection dataset | `export_dataset.py` → `images/labels/{train,val,test}` (80/10/10) + `meta.jsonl`. 길남 sets `overall_gilnam/vision/window_pose.yaml:path` to the dataset root (윤호 does **not** ship a competing `window_pose.yaml`) and runs `eval_corners.py` on it. |
| **태민** | §5 GT-pose stream | `export_stream.py` → JSONL `{"vision": <§5 msg>, "pose": {...xyzw...}}`, **built by 길남's `gt_stream.labels_to_message`** (`overall_gilnam/vision/gt_stream.py`), matching `sample_stream/sample_stream.jsonl`. |
| **태민** | flight bag (VIO/OpenVINS) | `export_vio.py` → EuRoC-ASL `mav0/` (cam0 ~20 Hz, imu0 200 Hz, GT WXYZ, int-ns shared clock). Pairs with `calib/` Kalibr YAMLs. |
| **길남 · 태민** | camera intrinsics (spec §6) | fill `overall_gilnam/vision/synth_intrinsics.yaml` numbers once the sim camera is fixed; `scene_gen.default_intrinsics()` reads that file, so pixels stay comparable. |

## Open items (see `docs/To_do_checklist_yunho.md`)

- `eval_corners` keys `distance_m` by `order_index`; a frame with two same-colour
  windows collapses to one distance entry. Confirm with 길남 whether repeated
  colours per frame are in scope for the eval set (scene_gen can emit up to 5
  windows over 3 colours).
- `tests/test_integration.py` still imports the pre-rename `vision.*` module names
  (`vision.replicator_corner_writer`, `vision.make_dataset`, `vision.visualize_labels`);
  it needs updating to `sim.*` (`replicator_writer`, `export_dataset`) — that file
  is outside `sim/`, so it was not touched here.
