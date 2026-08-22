"""PyBullet frames → YOLO → §5+pose stream (the missing image→corner link).

WHY
---
Every 3D-reconstruction and planning result in the repo so far was produced from
길남's *synthetic GT* corner stream with statistically re-created noise
(`vision/noisy_stream.py`). The real detector's output has never travelled that
path — `overall_gilnam/integration/e2e_rehearsal.py` starts from
`sample_stream.jsonl`, not from images.

This module closes that gap: it renders real frames out of a PyBullet scene whose
appearance is matched to the training domain (`rl/domain.py`), runs the actual
trained YOLO-pose weights over them, and emits records in EXACTLY the format
`noisy_stream.load_records` already reads:

    {"vision": <§5 message>, "pose": {timestamp, frame, position, orientation}}

so `eval_recon3d.reconstruct_windows`, `e2e_rehearsal.assemble_window_map` and
`planning.plan_waypoints` consume it unchanged. Nothing downstream is modified.

CONVENTIONS (README_stream.md — must match or triangulation silently degrades)
-----------------------------------------------------------------------------
world Z-up, camera OpenCV (+Z optical, +X right, +Y down), pose is T_world_cam
with quaternion xyzw, corners TL→TR→BR→BL seen from the approach side, and the
projection consumers rebuild is ``P = K [R_wc^T | −R_wc^T t_wc]``.

Routing discipline: the §5 message is built by 길남's `infer_stream.infer_frame`
(which routes through `vision_msg`), never hand-rolled here — same rule
`sim/export_stream.py` follows.
"""
from __future__ import annotations

import json
import os
import sys
from typing import Dict, List, Sequence, Tuple

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TEAM = os.path.dirname(_REPO)
for _p in (_REPO, os.path.join(_TEAM, "overall_gilnam", "vision"),
           os.path.join(_TEAM, "overall_gilnam", "planning"),
           os.path.join(_TEAM, "overall_gilnam", "integration")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from rl import domain  # noqa: E402


# --------------------------------------------------------------------------- #
# camera pose  (world <- OpenCV camera)
# --------------------------------------------------------------------------- #
def look_at_pose(eye, target, up=(0.0, 0.0, 1.0)) -> Tuple[np.ndarray, np.ndarray]:
    """Return ``(position, quat_xyzw)`` of T_world_cam for an OpenCV camera.

    OpenCV camera axes: +Z looks forward, +X right, +Y **down**. R_world_cam has
    those three axes as its columns, which is exactly what `projection_matrix`
    transposes back out.
    """
    eye = np.asarray(eye, dtype=float)
    z = np.asarray(target, dtype=float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, np.asarray(up, dtype=float))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)                       # down, completing a right-handed set
    R = np.column_stack([x, y, z])
    return eye, _rot_to_quat_xyzw(R)


def _rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → xyzw quaternion (inverse of eval_recon3d.quat_xyzw_to_rot)."""
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


# --------------------------------------------------------------------------- #
# rendering
# --------------------------------------------------------------------------- #
_EGL_PLUGIN = {}      # client -> plugin id (or -1 if unavailable)


def enable_gpu_render(p, client: int) -> bool:
    """Load PyBullet's EGL plugin so getCameraImage runs on the GPU.

    TinyRenderer is a CPU software rasteriser: 267 ms per 1280x720 frame, ~89% of
    the whole capture loop. EGL renders the same scene on the GPU. This path is
    plain OpenGL and has nothing to do with Isaac's RTX renderer, so the driver bug
    that blocks Isaac (ISAAC_CLUSTER_NOTES.md) does not apply here.

    Returns True if the GPU path is active; callers fall back to TinyRenderer.

    NOTE on this machine (WSL2): EGL loads, but binds Mesa's **llvmpipe** software
    rasteriser rather than the NVIDIA driver — GL_RENDERER reports llvmpipe. It is
    no faster than TinyRenderer (272 vs 249 ms at 720p) AND it renders blank frames
    here, so ``gpu`` defaults to False. Use ``scale`` for speed instead.
    """
    if client in _EGL_PLUGIN:
        return _EGL_PLUGIN[client] >= 0
    try:
        import pkgutil
        egl = pkgutil.get_loader("eglRenderer")
        pid = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin",
                           physicsClientId=client) if egl else \
            p.loadPlugin("eglRendererPlugin", physicsClientId=client)
    except Exception:
        pid = -1
    _EGL_PLUGIN[client] = pid
    return pid >= 0


def render_frame(p, client: int, eye, target, intr: Dict[str, float],
                 up=(0.0, 0.0, 1.0), scale: float = 1.0,
                 gpu: bool = False) -> np.ndarray:
    """One RGB frame at the team's §6 intrinsics (1280x720, fx=fy=600 today).

    ``scale`` < 1 rasterises smaller and upsamples back to the contract resolution.
    Rasterising cost goes with the pixel count, so scale 0.5 is ~4x cheaper. The
    RETURNED image is always width x height, because 태민's node has CX/CY = 640/360
    baked in and §5 fixes corner coordinates to the 1280x720 original — shrinking the
    output would silently move every coordinate. Detail is genuinely lost; that is a
    speed/accuracy dial, not a free win.
    """
    w, h = int(intr["width"]), int(intr["height"])
    rw, rh = max(64, int(w * scale)), max(64, int(h * scale))
    view = p.computeViewMatrix(cameraEyePosition=list(map(float, eye)),
                               cameraTargetPosition=list(map(float, target)),
                               cameraUpVector=list(up), physicsClientId=client)
    proj = p.computeProjectionMatrixFOV(domain.fov_y_deg(intr), w / h, 0.05, 60.0,
                                        physicsClientId=client)
    renderer = (p.ER_BULLET_HARDWARE_OPENGL if (gpu and enable_gpu_render(p, client))
                else p.ER_TINY_RENDERER)
    _, _, rgb, _, _ = p.getCameraImage(rw, rh, view, proj, shadow=1,
                                       renderer=renderer, physicsClientId=client)
    img = np.asarray(rgb, dtype=np.uint8).reshape(rh, rw, 4)[:, :, :3]
    if (rw, rh) != (w, h):
        import cv2
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
    return img


# --------------------------------------------------------------------------- #
# ground truth
# --------------------------------------------------------------------------- #
def window_corners_gt(center, ow: float, oh: float) -> np.ndarray:
    """GT corners in the detector's order: TL→TR→BR→BL as seen from the approach side.

    Approach side is −x (the drone flies +x), so with world up = +z the camera's
    image-right is world −y. Hence image-left is world +y.
    """
    cx, cy, cz = [float(v) for v in center]
    return np.array([
        [cx, cy + ow / 2, cz + oh / 2],   # TL
        [cx, cy - ow / 2, cz + oh / 2],   # TR
        [cx, cy - ow / 2, cz - oh / 2],   # BR
        [cx, cy + ow / 2, cz - oh / 2],   # BL
    ], dtype=float)


def scene_gt_from_layout(layout: Sequence[dict], intr: Dict[str, float],
                         seed: int = 0) -> dict:
    """PyBullet window layout → the scene_gt dict eval_recon3d / planning expect."""
    windows = []
    for w in layout:
        c = np.asarray(w["center"], dtype=float)
        ow, oh = float(w["ow"]), float(w["oh"])
        windows.append({
            "order_index": int(w["order_index"]),
            "color": str(w["color"]),
            "center": c.tolist(),
            "normal": [-1.0, 0.0, 0.0],          # faces the approaching drone (−x)
            "width": ow, "height": oh,
            "size_wh": [ow, oh],
            "corners_3d": window_corners_gt(c, ow, oh).tolist(),
        })
    return {
        "seed": int(seed),
        "intrinsics": dict(intr, distortion=[]),
        "conventions": ("world Z-up X-forward (m) / camera OpenCV +Z optical / "
                        "pose T_world_cam quat xyzw / corners TL->TR->BR->BL from "
                        "approach side / body==camera"),
        "windows": windows,
    }


# --------------------------------------------------------------------------- #
# capture
# --------------------------------------------------------------------------- #
def sweep_path(layout: Sequence[dict], n: int = 60, stand_off: float = 2.2,
               lateral: float = 1.6, rise: float = 0.5) -> List[Tuple[np.ndarray, np.ndarray]]:
    """One wide lateral sweep that tries to see the whole row at once.

    Kept for reference, but it is the WRONG capture for a corridor of windows:
    windows strung along +x project on top of each other from a single far
    viewpoint, and because the panes are opaque the nearest one's colour is what
    ``color_judge`` samples for all of them (measured: every window judged red,
    color_conf 0.53-0.64). Use :func:`per_window_sweep` instead.
    """
    centres = np.array([w["center"] for w in layout], dtype=float)
    look = centres.mean(axis=0)
    x0 = centres[:, 0].min() - stand_off
    out = []
    for k in range(n):
        s = k / max(1, n - 1)
        eye = np.array([x0, lateral * np.cos(np.pi * s), look[2] + rise * np.sin(np.pi * s)])
        out.append(look_at_pose(eye, look))
    return out


def per_window_sweep(layout: Sequence[dict], n_per_window: int = 24,
                     lateral: float = 0.75, rise: float = 0.35,
                     max_stand_off: float = 2.0, gap_frac: float = 0.7
                     ) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Approach each window in turn and sweep laterally in front of it.

    This mirrors the mission: the drone reconstructs the window it is heading
    for, passes it, and only then is the next one unoccluded. Two constraints
    drive the numbers:

    * ``reconstruct_windows`` only pairs frames whose camera positions differ by
      >= 0.5 m, so a straight fly-in gives no parallax — hence the lateral sweep.
    * the stand-off must stay INSIDE the gap to the previous window, or that
      window's opaque pane sits between the camera and the target.
    """
    order = sorted(layout, key=lambda w: float(w["center"][0]))
    xs = [float(w["center"][0]) for w in order]
    out = []
    for i, w in enumerate(order):
        c = np.asarray(w["center"], dtype=float)
        gap = xs[i] - xs[i - 1] if i > 0 else max_stand_off
        stand = min(max_stand_off, max(0.6, gap_frac * gap))
        for k in range(n_per_window):
            s = k / max(1, n_per_window - 1)
            eye = np.array([c[0] - stand,
                            c[1] + lateral * np.cos(np.pi * s),
                            c[2] + rise * np.sin(np.pi * s)])
            out.append(look_at_pose(eye, c))
    return out


#: colour name -> order_index, matching overall_gilnam/vision/color_order.yaml
_ORDER = {"red": 0, "green": 1, "blue": 2}


def infer_frame_multiclass(model, frame_rgb, timestamp_ns, frame_id, conf=0.25):
    """§5 message from a MULTI-CLASS pose model, taking colour from the network.

    길남's ``infer_stream.infer_frame`` re-derives colour with ``color_judge``'s HSV
    vote, because the shipped detector is ``single_cls`` and genuinely does not know
    colour. Our fine-tuned weights DO (nc=3, names {0:red,1:green,2:blue}), and on
    open-frame windows the HSV path actively destroys that: the corner polygon's
    interior is background seen through the opening, so the inlier ratio lands at
    0.23-0.41 against a 0.5 threshold and every detection is dropped as unknown —
    measured, and the reason green never reconstructed.

    So for a multi-class model we take the class straight from the head and keep
    everything else identical, still building the message through 길남's
    ``vision_msg.build_window`` so the §5 contract is enforced in one place.
    """
    from vision_msg import N_CORNERS, build_frame_message, build_window

    names = getattr(model, "names", {}) or {}
    # agnostic_nms: NMS across classes. Without it the same window can be returned
    # once per class — measured 21 cross-class duplicates (IoU>0.6) over one sweep,
    # which feed one window's corners into another window's accumulator and bias the
    # reconstruction toward it. With it: 0.
    res = model.predict(frame_rgb[:, :, ::-1], conf=conf, agnostic_nms=True,
                        verbose=False)[0]
    windows = []
    if res.boxes is not None and len(res.boxes):
        kpts = res.keypoints.xy.cpu().numpy()
        for box, corners in zip(res.boxes, kpts):
            colour = str(names.get(int(box.cls), "")).lower()
            if colour not in _ORDER:            # unknown class -> drop (같은 정책)
                continue
            pts = [[float(u), float(v)] for u, v in corners]
            if len(pts) != N_CORNERS:
                raise ValueError(f"model must emit {N_CORNERS} keypoints, got {len(pts)}")
            c = float(box.conf)
            # det_conf = box conf (geometry); color_conf = same head's confidence
            windows.append(build_window(_ORDER[colour], colour, pts,
                                        [1] * N_CORNERS, c, c))
    return build_frame_message(timestamp_ns, frame_id, windows)


def is_multiclass(model) -> bool:
    """True if the detector emits colour classes itself (fine-tuned nc=3)."""
    names = {str(v).lower() for v in (getattr(model, "names", {}) or {}).values()}
    return _ORDER.keys() <= names


def per_window_xy_sweep(layout: Sequence[dict], n_per_window: int = 32,
                        radius: float = 2.0, span_deg: float = 110.0,
                        gap_frac: float = 0.85) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Wide arc around each window in the HORIZONTAL plane, at constant z.

    Two reasons this beats the earlier sweeps, both measured:

    * **z held at the window's own height.** The camera looks level at the opening,
      so the window keeps the same upright orientation in every image and the
      detector's TL/TR/BR/BL keep meaning the same physical corners. 태민's node
      accumulates per corner INDEX across frames, so a corner-order flip mixes two
      physical points into one accumulator. The yaw-scan path, which views windows
      obliquely, produced 796 mm on the very window a level sweep reconstructs to
      17 mm with a similar observation count — angle, not observation count, is what
      hurts.
    * **baseline in x AND y.** The old sweep only moved along y at a fixed stand-off,
      so every ray came from one plane. An arc varies range and bearing together,
      which conditions the least-squares solve far better.

    The arc radius is clamped so the camera never backs past the previous window,
    whose frame would otherwise sit between it and the target.
    """
    order = sorted(layout, key=lambda w: float(w["center"][0]))
    xs = [float(w["center"][0]) for w in order]
    half = np.radians(span_deg) / 2.0
    out: List[Tuple[np.ndarray, np.ndarray]] = []
    for i, w in enumerate(order):
        c = np.asarray(w["center"], dtype=float)
        gap = xs[i] - xs[i - 1] if i > 0 else radius
        r = min(radius, max(0.8, gap_frac * gap))
        for k in range(n_per_window):
            s = k / max(1, n_per_window - 1)
            a = -half + s * (2 * half)          # bearing from the window normal (−x)
            eye = np.array([c[0] - r * np.cos(a),
                            c[1] + r * np.sin(a),
                            c[2]])              # ← z fixed at the window's height
            out.append(look_at_pose(eye, c))
    return out


def corridor_yaw_scan(layout: Sequence[dict], n: int = 120,
                      start_back: float = 2.0, end_past: float = 0.6,
                      weave: float = 0.35, rise: float = 0.15,
                      scan_rate_rad_s: float = 0.75, fps: float = 30.0
                      ) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Fly the corridor forward while yawing to keep the next opening in view.

    This is the mission's own capture, and the only one that works once the windows
    sit in walls: a lateral sweep in front of a wall mostly sees wall (measured:
    detections 119 -> 50, 3/3 -> 1/3 reconstructed).

    Yaw ALONE cannot triangulate — rotating about the optical centre gives every ray
    the same origin, so parallax is 0 and both ``reconstruct_windows`` (baseline
    >= 0.5 m) and 태민's node (``MIN_PARALLAX_DEG`` 2.0) reject it. The forward
    motion supplies the baseline; the yaw only keeps the target framed. That pairing
    is what 성진's ``scan`` yaw mode and 길남's ``scan.rate_rad_s`` (0.75 rad/s,
    bounded by the 2 Hz detection period) already describe.

    ``weave`` adds a small lateral serpentine so the baseline is not purely along the
    view axis, which is the degenerate direction for depth.
    """
    centres = np.array([w["center"] for w in layout], dtype=float)
    order = np.argsort(centres[:, 0])
    centres = centres[order]
    x0 = centres[0, 0] - start_back
    x1 = centres[-1, 0] + end_past
    z_nom = float(centres[:, 2].mean())
    max_dyaw = scan_rate_rad_s / fps          # per-frame yaw budget (rate cap)

    out: List[Tuple[np.ndarray, np.ndarray]] = []
    yaw = None
    for k in range(n):
        s = k / max(1, n - 1)
        eye = np.array([x0 + s * (x1 - x0),
                        weave * np.sin(2.0 * np.pi * 1.5 * s),
                        z_nom + rise * np.sin(2.0 * np.pi * s)])
        # aim at the nearest window still ahead of us; past the last one, hold it
        ahead = centres[centres[:, 0] > eye[0] + 0.15]
        target = ahead[0] if len(ahead) else centres[-1]
        desired = float(np.arctan2(target[1] - eye[1], target[0] - eye[0]))
        if yaw is None:
            yaw = desired
        else:                                  # respect the scan rate limit
            d = (desired - yaw + np.pi) % (2 * np.pi) - np.pi
            yaw += float(np.clip(d, -max_dyaw, max_dyaw))
        look = eye + np.array([np.cos(yaw), np.sin(yaw), 0.0]) * 3.0
        look[2] = target[2]
        out.append(look_at_pose(eye, look))
    return out


def capture(p, client: int, layout: Sequence[dict], model, color_config,
            intr: Dict[str, float], poses, conf: float = 0.25,
            fps: float = 30.0, save_dir: str | None = None) -> Tuple[List[dict], dict]:
    """Render → detect → §5+pose records. Returns ``(records, stats)``.

    Routes to the network's own colour head when the model has one, else to 길남's
    HSV path — so both the shipped single_cls weights and our fine-tuned 3-class
    weights work through the same call.
    """
    from infer_stream import infer_frame          # 길남's builder — not re-implemented

    multi = is_multiclass(model)
    records, n_det, n_frames_with_det = [], 0, 0
    for i, (pos, quat) in enumerate(poses):
        target = pos + _forward_of(quat)
        img = render_frame(p, client, pos, target, intr)
        t_ns = round(i * 1e9 / fps)
        msg = (infer_frame_multiclass(model, img, t_ns, i, conf) if multi
               else infer_frame(model, img, t_ns, i, color_config, conf))
        k = len(msg["windows"])
        n_det += k
        n_frames_with_det += int(k > 0)
        records.append({"vision": msg,
                        "pose": {"timestamp": msg["timestamp"], "frame": "world",
                                 "position": [float(v) for v in pos],
                                 "orientation": [float(v) for v in quat]}})
        if save_dir and i % 10 == 0:
            from PIL import Image
            os.makedirs(save_dir, exist_ok=True)
            Image.fromarray(img).save(os.path.join(save_dir, f"frame_{i:04d}.png"))
    stats = {"frames": len(records), "detections": n_det,
             "frames_with_detection": n_frames_with_det,
             "det_per_frame": round(n_det / max(1, len(records)), 2)}
    return records, stats


def _forward_of(quat_xyzw) -> np.ndarray:
    """Camera +Z (optical axis) in world coordinates."""
    from eval_recon3d import quat_xyzw_to_rot
    return quat_xyzw_to_rot(np.asarray(quat_xyzw, dtype=float))[:, 2]


def write_stream(records: List[dict], scene_gt: dict, out_dir: str) -> Tuple[str, str]:
    os.makedirs(out_dir, exist_ok=True)
    sp = os.path.join(out_dir, "pybullet_stream.jsonl")
    gp = os.path.join(out_dir, "scene_gt.json")
    with open(sp, "w", encoding="utf-8") as f:
        for r in records:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    with open(gp, "w", encoding="utf-8") as f:
        json.dump(scene_gt, f, ensure_ascii=False, indent=2)
    return sp, gp
