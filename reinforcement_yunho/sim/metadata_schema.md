# Per-frame metadata JSON (`sim/export_dataset.py --mode from-metadata`)

The Isaac Sim side dumps **one JSON file per rendered frame** in this schema; the
offline exporter reads a directory of them and builds the YOLO-pose dataset
(labels + `meta.jsonl`) without needing Isaac Sim. `sim.scene_gen.scene_to_metadata`
produces exactly this shape, so the pure sampler and the Isaac dumper agree.

> One file per frame → `<metadata_dir>/frame_000001.json`, … . The JSON **filename
> stem** becomes the image/label stem in the dataset, so keep it unique & sortable.

## Schema

```json
{
  "image": "frame_000001.png",              // rel to this JSON's dir, or absolute
  "timestamp": 1720000000000000000,          // optional, int ns (shared cam/IMU clock)
  "frame_id": 1,                             // optional, int
  "T_world_cam_usd": [[r00,r01,r02,tx],      // 4x4 USD camera-prim pose in world.
                      [r10,r11,r12,ty],      //   Isaac reports this directly. The
                      [r20,r21,r22,tz],      //   exporter feeds it to build_label_*,
                      [0,   0,   0, 1]],     //   which applies the USD->CV flip.
  "windows": [
    {
      "color": "red",                        // "red"|"green"|"blue" -> class 0/1/2
      "order_index": 0,                      // optional (derivable from color)
      "center": [x, y, z],                   // world metres, window centre
      "width": 1.0, "height": 1.2,           // opening size, metres
      // orientation — supply EXACTLY ONE of:
      "normal": [nx, ny, nz],                //  outward normal (approach side); the
                                             //  exporter builds R_world_win from it
      "R_world_win": [[...3x3...]],          //  OR window-local -> world rotation
      "quat_wxyz": [w, x, y, z]              //  OR unit quaternion, WXYZ order
    }
  ]
}
```

## Conventions (CONVENTIONS.md — the whole dataset depends on these)

- **World**: right-handed, **+Z up**, X-forward, metres.
- **`T_world_cam_usd`**: the **USD camera-prim** pose (camera looks down **−Z**, +X
  right, +Y up). `common.geometry.world_to_camera_cv` applies `diag(1,−1,−1)` to
  reach the OpenCV frame we project in — so pass the *USD* pose here, not a CV one.
  (`sim.scene_gen.cv_to_usd_transform` bridges a CV pose into this if you sampled
  in CV: `R_usd = R_cv @ diag(1,−1,−1)`.)
- **Window frame**: local +X right, +Y up, +Z = outward normal; corners are emitted
  TL→TR→BR→BL (`common.geometry.CORNER_ORDER`). A bare `normal` is resolved into
  `R_world_win` by `sim.scene_gen.window_rotation_from_normal` (for a *vertical*
  window this matches 길남's `synth_scene._window_corners`).
- **Quaternion order** here is **WXYZ** (`quat_wxyz`), matching
  `common.geometry.quat_wxyz_to_R`. (The §5 stream pose uses XYZW — see
  CONVENTIONS.md's per-interface table; do not mix them.)

## What the exporter derives

- **Label lines** (`labels/{split}/<stem>.txt`): one 17-token YOLO-pose line per
  window whose 4 corners are all in-front **and** inside the frame (dataset-1 /
  policy A). Off-frame windows are dropped (no line, no meta row).
- **`meta.jsonl`** (길남's `eval_corners` schema): per image
  `{"image":"images/<split>/<stem>.png","windows":[{"order_index","distance_m"}]}`
  where `distance_m = ‖center − T_world_cam_usd[:3,3]‖` (world m, 3 dp), only for
  the labelled windows — so meta rows and label lines stay 1:1.
