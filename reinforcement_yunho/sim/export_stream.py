"""§5 GT-pose stream for 태민 (VIO) — routed through 길남's validated builders.

WHAT
----
Turns 윤호's per-frame (YOLO-pose label lines + camera pose) into the paired
"vision + pose" JSONL stream 태민 already consumes, matching
overall_gilnam/vision/sample_stream/sample_stream.jsonl exactly:

    {"vision": <§5 message>,
     "pose": {"timestamp": <int ns>, "frame": "world",
              "position": [x,y,z], "orientation": [qx,qy,qz,qw]}}

WHY WE DO NOT HAND-ROLL THE §5 DICT (real contract)
---------------------------------------------------
§5 emission is 길남's. The validated builders live in
    /sfs/gpfs/tardis/home/pcn3tv/ugrp34/overall_gilnam/vision/gt_stream.py
        labels_to_message(lines, timestamp_ns, frame_id, config)   (spec §4.4)
    /sfs/gpfs/tardis/home/pcn3tv/ugrp34/overall_gilnam/vision/vision_msg.py
        build_frame_message / build_window                          (spec §5.1)
We ROUTE our label lines through ``gt_stream.labels_to_message`` (which re-parses
each 17-token line via parse_label_line — a live format check) instead of
assembling the dict ourselves, so a spec §5 change touches only 길남's code and
GT stays det_conf = color_conf = 1.0.

POSE FRAME (window_detection_spec_v0.2.md §6.1)
------------------------------------------------
The inputs are the rendered CAMERA pose (scene_gen's camera["position"] /
camera["quat_xyzw"], CV T_world_cam). The emitted ``pose`` block is, by default
(``pose_frame="body"``), the **IMU/body pose** the spec fixes for the pose stream
(body x forward / y left / z up): T_WB = T_WC @ inv(T_IC) with the spec §6.1
standard T_IC (``common.T_IC_SPEC``; owner of the constant = 태민's
window_recon_node.py). ``pose_frame="camera"`` reproduces 길남's
sample_stream.jsonl contract (body ≡ camera, identity) — spec §6.1 says that
stream is for E2E rehearsal only and must NOT be fed to 태민's node directly.
Either way orientation is quaternion (x,y,z,w).

NOTE: gt_stream needs the colour config (color_order.yaml). 길남's loader
``color_judge.load_color_config`` is just ``yaml.safe_load`` but lives in a module
that ``import cv2`` at top — unavailable on this box — so we load the yaml directly
(same dict shape) to keep this importable/testable without cv2.
"""
from __future__ import annotations

import json
import os
import sys
from typing import Dict, List, Optional, Sequence

import numpy as np
import yaml

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
# 길남's vision dir (sibling package under ugrp34) — the §5 builders live here.
GILNAM_VISION = os.path.join(os.path.dirname(_ROOT), "overall_gilnam", "vision")
DEFAULT_COLOR_ORDER = os.path.join(GILNAM_VISION, "color_order.yaml")

QUAT_DECIMALS = 6
METER_DECIMALS = 4
PX_DECIMALS = 2


def _import_gt_stream():
    """Import 길남's gt_stream (+ its vision_msg dep) from GILNAM_VISION. Lazy +
    guarded so ``import sim.export_stream`` never hard-fails if that tree moves."""
    if GILNAM_VISION not in sys.path:
        sys.path.insert(0, GILNAM_VISION)
    try:
        import gt_stream  # noqa: E402  (pulls vision_msg — both cv2-free/pure)
        return gt_stream
    except Exception as e:  # pragma: no cover
        raise ImportError(
            f"could not import 길남's gt_stream from {GILNAM_VISION!r}: {e!r}. "
            "Ensure overall_gilnam/vision is present (do not hand-roll the §5 dict)."
        ) from e


def load_color_config(path: str = DEFAULT_COLOR_ORDER) -> dict:
    """color_order.yaml as a plain dict (== color_judge.load_color_config, but
    without importing that cv2-bound module). gt_stream only reads config['colors']."""
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _round(vs, nd):
    return [round(float(v), nd) for v in vs]


def camera_pose_to_body_pose(position, quat_xyzw, T_IC=None):
    """CV camera pose -> IMU/body pose (spec §6.1), both as (position, quat_xyzw).
    T_WB = T_WC @ inv(T_IC); T_IC defaults to ``common.T_IC_SPEC``."""
    from common import quat_wxyz_to_R, make_transform, T_world_body_from_cam_cv
    from sim.scene_gen import R_to_quat_xyzw

    q = list(quat_xyzw)
    R_wc = quat_wxyz_to_R([q[3], q[0], q[1], q[2]])
    T_wb = T_world_body_from_cam_cv(make_transform(R_wc, position), T_IC)
    return [float(x) for x in T_wb[:3, 3]], [float(x) for x in R_to_quat_xyzw(T_wb[:3, :3])]


def build_stream_record(
    label_lines: Sequence[str],
    timestamp_ns: int,
    frame_id: int,
    position,
    quat_xyzw,
    config: dict,
    *,
    round_pixels: bool = True,
    pose_frame: str = "body",
) -> Dict[str, object]:
    """One JSONL record = {"vision": <§5 msg>, "pose": {...}} (make_stream.py shape).

    label_lines : 17-token YOLO-pose lines for this frame (build_label_lines).
    timestamp_ns: int ns, shared cam/IMU clock (== pose timestamp).
    position    : CAMERA position in world [x,y,z] (as rendered).
    quat_xyzw   : CAMERA orientation quaternion (x,y,z,w) — CV T_world_cam.
    pose_frame  : "body" (default) -> emit the IMU/body pose (spec §6.1, via the
                  standard T_IC); "camera" -> emit the camera pose as-is (길남's
                  sample_stream body≡camera contract, E2E rehearsal only).
    """
    if not isinstance(timestamp_ns, int):
        raise ValueError(f"timestamp_ns must be int (ns), got {type(timestamp_ns).__name__}")
    if pose_frame == "body":
        position, quat_xyzw = camera_pose_to_body_pose(position, quat_xyzw)
    elif pose_frame != "camera":
        raise ValueError(f"pose_frame must be 'body' or 'camera', got {pose_frame!r}")
    gt_stream = _import_gt_stream()
    msg = gt_stream.labels_to_message(list(label_lines), timestamp_ns, frame_id, config)
    if round_pixels:
        for w in msg["windows"]:
            w["corners"] = [_round(pt, PX_DECIMALS) for pt in w["corners"]]
            w["center"] = _round(w["center"], PX_DECIMALS)
    return {
        "vision": msg,
        "pose": {
            "timestamp": int(timestamp_ns),
            "frame": "world",
            "position": _round(position, METER_DECIMALS),
            "orientation": _round(quat_xyzw, QUAT_DECIMALS),
        },
    }


def write_stream(
    out_path: str,
    frames: Sequence[Dict[str, object]],
    config: Optional[dict] = None,
    *,
    pose_frame: str = "body",
) -> Dict[str, object]:
    """Write a JSONL §5+pose stream. ``frames``: [{label_lines, timestamp_ns,
    frame_id, position, quat_xyzw}] with the CAMERA pose; the written pose block
    is the IMU/body pose unless ``pose_frame="camera"`` (see build_stream_record).
    Returns {path, frames}."""
    if config is None:
        config = load_color_config()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    n = 0
    with open(out_path, "w", encoding="utf-8") as f:
        for fr in frames:
            rec = build_stream_record(
                fr["label_lines"], fr["timestamp_ns"], fr["frame_id"],
                fr["position"], fr["quat_xyzw"], config, pose_frame=pose_frame,
            )
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            n += 1
    return {"path": out_path, "frames": n}
