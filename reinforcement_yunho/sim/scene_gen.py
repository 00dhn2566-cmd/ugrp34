"""Domain-randomised scene sampler for the window dataset (spec §4.1).

WHAT
----
:func:`sample_scene` draws ONE frame's worth of randomised scene state -- window
placements + a camera pose + lighting + a (mandatory) textured background -- as a
pure numpy dict. Call it with many seeds to build a dataset. The windows come out
in exactly the dict shape ``replicator_writer.build_label_lines`` consumes, and
the camera pose is provided in BOTH the CV convention (the shared team contract)
and the USD convention (what build_label_lines wants), pre-bridged.

Domain randomisation (spec §4.1, all seeded/reproducible):
  * window count      : 1-5 per scene
  * distance          : near / mid / far bins, chosen uniformly (원거리 소형 학습)
  * yaw / pitch tilt  : +-60 deg off frontal (기울어진 창문 corner 학습)
  * lighting          : brightness + direction random (색 판정 강건성)
  * colour            : red/green/blue sampled uniformly (색별 균등 분포 emerges
                        at the dataset level)
  * background        : TEXTURED, non-blank -- MANDATORY (박태민 07/03 request; a
                        blank background starves VIO feature tracking). Modelled
                        as a required scene param; sample_scene never returns a
                        blank background and :func:`validate_scene` rejects one.

Ranges the spec leaves unspecified are DEFAULTS documented inline (cite: spec
§4.1 "위치 랜덤 / 거리 근·중·원 균등"). Tune in one place here.

COORD / PROJECTION CONTRACT (must match 길남's synth_scene + the §5 stream)
---------------------------------------------------------------------------
world  : Z-up, X-forward, metres.
camera : OpenCV (+Z optical forward, +X right, +Y down); pose = T_world_cam.
window : local +X right, +Y up, +Z = outward normal (approach side); corners
         TL->TR->BR->BL (common.geometry.CORNER_ORDER / window_corners_world).
The camera pose is sampled in the CV convention. build_label_lines projects via
common.geometry.world_to_camera_cv, which applies the USD->CV flip, so it needs
the USD-prim pose -- :func:`cv_to_usd_transform` bridges CV->USD
(R_usd = R_cv @ diag(1,-1,-1)). ``sim/smoke_test.py`` cross-checks that the
resulting pixels equal 길남's documented formula X_cam = R_wc^T (X - t)
(overall_gilnam/vision/synth_scene.py :func:`project`).

Isaac Sim (omni.replicator) graph building is an import-guarded stub at the
bottom; the sampler above is pure numpy and unit-tested offline.
"""
from __future__ import annotations

import os
import sys
from typing import Dict, List, Optional

import numpy as np

# --- shared 'common' bootstrap (CONVENTIONS.md) ------------------------------
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from common import (  # noqa: E402
    CameraIntrinsics,
    R_USD_TO_CV,
    make_transform,
    window_corners_world,
)
from sim.replicator_writer import ORDER_INDEX  # noqa: E402  (colour->class map)

WORLD_UP = np.array([0.0, 0.0, 1.0])  # +Z up (world frame, CONVENTIONS.md)

# --- spec-unspecified DEFAULTS (spec §4.1 leaves exact metres open) -----------
# Distance bins (metres, camera->window centre): near / mid / far, chosen with
# equal probability so the three regimes are uniform (spec §4.1 근·중·원 균등).
DISTANCE_BINS = {"near": (1.5, 3.0), "mid": (3.0, 6.0), "far": (6.0, 10.0)}
SIZE_WH_M = (0.6, 1.4)          # window opening w,h in metres (길남 uses 0.8-1.2)
TILT_DEG = 60.0                 # yaw & pitch each in +-TILT_DEG (spec §4.1 ±60°)
CAM_HEIGHT_M = 1.5              # nominal camera height above ground
CAM_JITTER_M = 0.3             # camera position jitter (each axis)
CAM_LOOK_DEG = 20.0            # camera look yaw/pitch jitter off world +X
IMG_MARGIN_FRAC = 0.12        # keep window CENTRE this far inside the frame edge

# Placeholder USD material/texture names for the mandatory textured background
# (박태민 07/03). Swap for real Isaac Sim material asset paths at scene-build time.
# Requirement (see sim/README.md): the background must be TEXTURED (feature-rich
# for VIO) yet stay OUT of the saturated primary HSV bands (color_order.yaml
# hsv_min_s=100) so 길남's color_judge is not fooled -- i.e. desaturated textures.
BACKGROUND_TEXTURES = (
    "brick_wall", "concrete", "plaster", "wood_planks",
    "carpet", "gravel", "fabric_weave", "corrugated_metal",
)


# ======================================================================
#  small numpy rotation / quaternion helpers (no new deps; cite 길남's algo)
# ======================================================================
def _normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("cannot normalise a ~zero vector")
    return v / n


def _Rx(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def _Ry(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def look_at_cv(eye, target, up=WORLD_UP) -> np.ndarray:
    """R_world_cam (CV): columns [x_right, y_down, z_forward]. Matches 길남's
    synth_scene._look_at_rotation: z=normalize(target-eye), x=normalize(cross(z,up)),
    y=cross(z,x) -> +Y points 'down' in world (OpenCV convention)."""
    z = _normalize(np.asarray(target, float) - np.asarray(eye, float))
    x = _normalize(np.cross(z, up))
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def R_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> unit quaternion (x,y,z,w). numpy-only (new-dep-free);
    same branchy algorithm as 길남's synth_scene._rotation_to_quat_xyzw."""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2.0
        q = np.empty(3)
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        w = (R[k, j] - R[j, k]) / s
        x, y, z = q
    return _normalize(np.array([x, y, z, w]))


def xyzw_to_wxyz(q) -> List[float]:
    """[qx,qy,qz,qw] -> [qw,qx,qy,qz] (EuRoC GT / 성진 control order; CONVENTIONS.md)."""
    q = list(q)
    return [q[3], q[0], q[1], q[2]]


def window_rotation_from_normal(normal, up=WORLD_UP) -> np.ndarray:
    """R_world_win with column 2 == outward normal (approach side).

    Columns [right, up, normal] with right = normalize(cross(up, normal)) and an
    orthonormalised up. Matches common.geometry's window-local frame (+X right,
    +Y up, +Z normal) AND, for VERTICAL windows (normal horizontal), 길남's
    synth_scene._window_corners (viewer_right = cross(-normal, UP), up = UP).
    Used by scene_gen and by export_dataset to resolve a bare "normal".
    """
    z = _normalize(np.asarray(normal, float))
    right = np.cross(up, z)
    if np.linalg.norm(right) < 1e-8:  # normal parallel to world up
        right = np.cross(np.array([1.0, 0.0, 0.0]), z)
    right = _normalize(right)
    up2 = np.cross(z, right)
    return np.column_stack([right, up2, z])


def cv_to_usd_transform(R_world_cam_cv: np.ndarray, position) -> np.ndarray:
    """CV camera pose -> the USD-prim pose T_world_cam_usd build_label_lines wants.

    common.geometry.world_to_camera_cv does p_cam_cv = diag(1,-1,-1) @ R_usd^T (X-t).
    For that to equal 길남's CV projection p_cam_cv = R_cv^T (X-t) we need
    R_usd = R_cv @ diag(1,-1,-1); translation is unchanged. (diag(1,-1,-1) =
    common.R_USD_TO_CV is its own inverse.) See module cross-check note.
    """
    R_usd = np.asarray(R_world_cam_cv, float) @ R_USD_TO_CV
    return make_transform(R_usd, position)


# ======================================================================
#  PURE SCENE SAMPLER  (numpy only; runs + tested anywhere)
# ======================================================================
def default_intrinsics() -> CameraIntrinsics:
    """The shared synthetic camera. Prefer 길남's synth_intrinsics.yaml (fx=fy=600,
    1280x720) so sampled pixels are directly comparable to 길남's toy stream; fall
    back to a 90deg-ish pinhole if the file is absent. 윤호 replaces the numbers
    once spec §6 is filled (intrinsics-only change, no code change)."""
    yaml_path = os.path.join(
        os.path.dirname(_ROOT), "overall_gilnam", "vision", "synth_intrinsics.yaml"
    )
    try:
        import yaml

        with open(yaml_path, encoding="utf-8") as f:
            d = yaml.safe_load(f)
        return CameraIntrinsics.from_yaml_dict(d)
    except Exception:
        return CameraIntrinsics.from_fov(1280, 720, 90.0)


def _sample_camera(rng: np.random.Generator, intr: CameraIntrinsics) -> Dict[str, object]:
    """Camera near origin, looking ~world +X with small yaw/pitch jitter."""
    eye = np.array(
        [
            rng.uniform(-CAM_JITTER_M, CAM_JITTER_M),
            rng.uniform(-CAM_JITTER_M, CAM_JITTER_M),
            CAM_HEIGHT_M + rng.uniform(-CAM_JITTER_M, CAM_JITTER_M),
        ]
    )
    yaw = np.radians(rng.uniform(-CAM_LOOK_DEG, CAM_LOOK_DEG))
    pitch = np.radians(rng.uniform(-CAM_LOOK_DEG, CAM_LOOK_DEG))
    # forward starts as world +X, tilt by yaw (about Z) and pitch (about Y)
    forward = _Ry(pitch) @ _Rx(0.0) @ np.array([np.cos(yaw), np.sin(yaw), 0.0])
    R_cv = look_at_cv(eye, eye + forward)
    return {
        "position": eye,
        "R_world_cam_cv": R_cv,
        "T_world_cam_cv": make_transform(R_cv, eye),
        "T_world_cam_usd": cv_to_usd_transform(R_cv, eye),  # for build_label_lines
        "quat_xyzw": R_to_quat_xyzw(R_cv),                  # §5 / stream order
        "quat_wxyz": np.array(xyzw_to_wxyz(R_to_quat_xyzw(R_cv))),  # EuRoC GT / control
    }


def _sample_window(rng, order_i, camera, intr) -> Dict[str, object]:
    """One window placed in front of the camera at a near/mid/far depth, its
    centre guaranteed inside the frame; normal faces the camera then tilted."""
    W, H = int(intr.width), int(intr.height)
    Kinv = np.linalg.inv(intr.K())

    bin_name = rng.choice(list(DISTANCE_BINS))          # near/mid/far uniform
    depth = rng.uniform(*DISTANCE_BINS[bin_name])
    mu, mv = IMG_MARGIN_FRAC * W, IMG_MARGIN_FRAC * H
    u = rng.uniform(mu, W - mu)
    v = rng.uniform(mv, H - mv)
    ray_cam = Kinv @ np.array([u, v, 1.0])
    center_cam = ray_cam / ray_cam[2] * depth           # CV camera frame
    R_cv = camera["R_world_cam_cv"]
    center_world = camera["position"] + R_cv @ center_cam

    # colour uniform over the 3 classes (spec §4.1 색별 균등 — balances over dataset)
    color = str(rng.choice(list(ORDER_INDEX)))
    w = float(rng.uniform(*SIZE_WH_M))
    h = float(rng.uniform(*SIZE_WH_M))

    # base normal points from the window back toward the camera (approach side),
    # then tilt yaw about local up + pitch about local right, each in +-60deg.
    base_n = _normalize(camera["position"] - center_world)
    R0 = window_rotation_from_normal(base_n)
    yaw = np.radians(rng.uniform(-TILT_DEG, TILT_DEG))
    pitch = np.radians(rng.uniform(-TILT_DEG, TILT_DEG))
    R_world_win = R0 @ _Ry(yaw) @ _Rx(pitch)
    normal = R_world_win[:, 2]

    return {
        "order_index": ORDER_INDEX[color],
        "color": color,
        "center": center_world,
        "normal": normal,
        "size_wh": [w, h],
        "width": w,
        "height": h,
        "yaw_deg": float(np.degrees(yaw)),
        "pitch_deg": float(np.degrees(pitch)),
        "distance_bin": bin_name,
        "distance_m": float(np.linalg.norm(center_world - camera["position"])),
        "R_world_win": R_world_win,
        "corners_3d": window_corners_world(center_world, R_world_win, w, h),
    }


def _sample_lighting(rng: np.random.Generator) -> Dict[str, object]:
    """Random brightness + direction (spec §4.1 조명 밝기·방향 랜덤)."""
    d = np.array([rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1.0, -0.2)])
    return {
        "brightness": float(rng.uniform(0.3, 1.6)),  # relative intensity multiplier
        "direction": _normalize(d).tolist(),          # points downward-ish
        "color_temperature_k": float(rng.uniform(3500.0, 7500.0)),
    }


def _sample_background(rng: np.random.Generator) -> Dict[str, object]:
    """MANDATORY textured background (박태민 07/03). Never blank."""
    return {
        "kind": "textured",                            # <- required; never "blank"
        "texture": str(rng.choice(BACKGROUND_TEXTURES)),
        "avoid_primary_hsv": True,                     # keep S < color bands (색 판정)
        "sat_max": 100,                                # == color_order.yaml hsv_min_s
        "brightness": float(rng.uniform(0.4, 1.2)),
    }


def sample_scene(
    seed: int,
    intr: Optional[CameraIntrinsics] = None,
    n_windows: Optional[int] = None,
    *,
    n_range=(1, 5),
) -> Dict[str, object]:
    """Sample one randomised scene (spec §4.1). Deterministic in ``seed``.

    Returns a dict with keys: seed, intrinsics (CameraIntrinsics), camera,
    windows (list; each is a build_label_lines-ready dict + extras), lighting,
    background. See metadata_schema.md for the serialisable per-frame form.
    """
    if intr is None:
        intr = default_intrinsics()
    rng = np.random.default_rng(seed)
    n = int(n_windows) if n_windows is not None else int(rng.integers(n_range[0], n_range[1] + 1))

    camera = _sample_camera(rng, intr)
    windows = [_sample_window(rng, i, camera, intr) for i in range(n)]
    scene = {
        "seed": seed,
        "intrinsics": intr,
        "camera": camera,
        "windows": windows,
        "lighting": _sample_lighting(rng),
        "background": _sample_background(rng),
    }
    validate_scene(scene)
    return scene


def validate_scene(scene: Dict[str, object]) -> None:
    """Guard the hard scene requirements (fail loud, not silently degrade)."""
    bg = scene.get("background")
    if not bg or bg.get("kind") == "blank":
        raise ValueError("scene background must be TEXTURED, never blank (박태민 07/03)")
    for w in scene["windows"]:
        if w["color"] not in ORDER_INDEX:
            raise ValueError(f"window colour {w['color']!r} not in {list(ORDER_INDEX)}")


def scene_to_metadata(scene: Dict[str, object], image: str,
                      timestamp_ns: Optional[int] = None,
                      frame_id: Optional[int] = None) -> Dict[str, object]:
    """Serialisable per-frame metadata JSON (see sim/metadata_schema.md).

    This is exactly what export_dataset's from-metadata mode consumes and what the
    Isaac side should dump per rendered frame.
    """
    cam = scene["camera"]
    meta: Dict[str, object] = {
        "image": image,
        "T_world_cam_usd": np.asarray(cam["T_world_cam_usd"], float).tolist(),
        "windows": [
            {
                "color": w["color"],
                "order_index": int(w["order_index"]),
                "center": list(map(float, w["center"])),
                "normal": list(map(float, w["normal"])),
                "width": float(w["width"]),
                "height": float(w["height"]),
            }
            for w in scene["windows"]
        ],
    }
    if timestamp_ns is not None:
        meta["timestamp"] = int(timestamp_ns)
    if frame_id is not None:
        meta["frame_id"] = int(frame_id)
    return meta


# ======================================================================
#  ISAAC SIM (omni.replicator) GRAPH BUILDER  (import-guarded stub)
# ======================================================================
def build_replicator_graph(intr: Optional[CameraIntrinsics] = None, **kwargs):
    """Set up the omni.replicator randomiser graph that realises spec §4.1 in
    Isaac Sim (windows, camera, lighting, textured background) and drives
    ``replicator_writer.WindowCornerWriter``. Import-guarded stub: import of this
    module still works without Isaac Sim; only calling this needs it.

    Sketch of the real graph (fill in scene-specific asset paths inside Isaac Sim):
        import omni.replicator.core as rep
        from sim.replicator_writer import register
        register()
        cam = rep.create.camera()
        with rep.trigger.on_frame():
            # window count / pose / colour / size per DISTANCE_BINS, TILT_DEG, ...
            # lighting brightness+direction per _sample_lighting
            # MANDATORY textured background material per _sample_background
            ...
        rp = rep.create.render_product(cam, (intr.width, intr.height))
        writer = rep.writers.get("WindowCornerWriter")
        writer.initialize(output_dir=..., intrinsics=intr, scene_provider=...)
        writer.attach([rp]); rep.orchestrator.run()
    """
    try:
        import omni.replicator.core as rep  # type: ignore  # noqa: F401
    except Exception as e:
        raise ImportError(
            "build_replicator_graph needs omni.replicator (run inside the Isaac "
            f"Sim python env). Import error: {e!r}. Use sample_scene() offline."
        ) from e
    raise NotImplementedError(
        "Replicator graph is a documented scaffold; complete it inside Isaac Sim."
    )
