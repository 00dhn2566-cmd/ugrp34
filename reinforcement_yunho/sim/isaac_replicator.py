#!/usr/bin/env python3
"""Headless Isaac Sim 4.5 dataset generator for the drone "window traversal" task.

WHAT
----
For each of N frames it (1) draws a domain-randomised scene from the *pure*
sampler ``sim.scene_gen.sample_scene`` (1-5 coloured windows, near/mid/far, +-60
deg tilt, randomised lighting, mandatory textured/feature-rich background), (2)
builds a USD stage that MATCHES that scene exactly -- coloured window prims whose
material lands inside ``overall_gilnam/vision/color_order.yaml``'s HSV bands, a
feature-rich enclosing room (박태민 07/03: VIO needs trackable features, never a
blank background), and a camera whose pixel intrinsics equal 길남's
``synth_intrinsics`` (fx=fy=600, cx=640, cy=360 @ 1280x720) -- (3) renders an RGB
frame with the RTX renderer, and (4) emits the per-frame metadata JSON that
``sim/export_dataset.py --mode from-metadata`` consumes UNCHANGED.

THE ONE INVARIANT THAT MAKES THIS WORK
--------------------------------------
The camera pose written to the metadata (``T_world_cam_usd``) and the pose set on
the Isaac ``Camera`` prim are the *same* 4x4, and each window prim is oriented
with the *same* rotation the offline exporter will reconstruct. So the rendered
pixels and the projected YOLO-pose labels agree. Concretely:
  * camera : we set the USD camera-prim transform to ``scene["camera"]["T_world_cam_usd"]``
    (produced by ``scene_gen.cv_to_usd_transform``) and emit that identical matrix.
  * windows: ``scene_to_metadata`` emits only the window ``normal``; the exporter
    (``export_dataset._resolve_window`` -> ``scene_gen.window_rotation_from_normal``)
    rebuilds ``R_world_win`` from that normal. We therefore build each window prim
    with ``window_rotation_from_normal(normal)`` -- NOT the sampler's internal
    ``R_world_win`` (which carries an extra in-plane roll that never reaches the
    metadata). This keeps rendered corners == labelled corners.

RUN (inside the Isaac Sim 4.5 apptainer; see sim/run_isaac_dataset.sh):
    /isaac-sim/python.sh sim/isaac_replicator.py --num-frames 100 --out /path/out --seed 0
Then, on any machine (no GPU):
    python3 sim/export_dataset.py --mode from-metadata \
        --metadata-dir /path/out/meta --images-dir /path/out/frames \
        --out /path/dataset_root

ISAAC SIM 4.5 API FACTS VERIFIED ON THE WEB (sources in comments below)
----------------------------------------------------------------------
* SimulationApp import + config keys (headless, renderer, width, height, ...):
  https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.simulation_app/docs/index.html
* Renderer string values ("RaytracedLighting" | "PathTracing" | "RealTimePathTracing"):
  https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/rendering_modes.html
* render_product + rgb annotator + rep.orchestrator.step(rt_subframes=...) + get_data(),
  and saving RGBA via PIL.Image.fromarray(...).save(...):
  https://docs.isaacsim.omniverse.nvidia.com/4.5.0/replicator_tutorials/tutorial_replicator_isaac_snippets.html
  https://docs.isaacsim.omniverse.nvidia.com/4.5.0/replicator_tutorials/tutorial_replicator_getting_started.html
* Camera intrinsics <-> focal_length / horizontal_aperture (fx = focal/aperture * width):
  https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_camera.html
* Reading a prim's world transform (UsdGeom.Xformable.ComputeLocalToWorldTransform):
  standard pxr USD API (used here to CONFIRM the camera pose we rendered with).

NOTE ON py_compile: this file keeps the SimulationApp + omni/pxr imports at module
scope, in the mandatory order (SimulationApp() BEFORE any omni/pxr import). That is
the idiomatic Isaac standalone layout; ``python3 -m py_compile`` only checks syntax
(it never executes the module), so it passes without Isaac Sim installed.
"""
from __future__ import annotations

# ---------------------------------------------------------------------------
# 1) PURE imports + CLI (numpy only; safe anywhere, needed before we launch Kit)
# ---------------------------------------------------------------------------
import argparse
import json
import os
import sys
import time

import numpy as np

# Put the repo root on sys.path so `sim.*` / `common.*` import regardless of cwd
# (same bootstrap the other sim/ modules use).
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# Pure numpy modules (no Isaac Sim). scene_gen imports replicator_writer, whose
# omni.replicator import is guarded, so this is safe on a plain machine too.
from common import make_transform  # noqa: E402
from sim.scene_gen import (  # noqa: E402
    default_intrinsics,
    sample_scene,
    scene_to_metadata,
    window_rotation_from_normal,
)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Headless Isaac Sim 4.5 window dataset generator")
    p.add_argument("--num-frames", type=int, default=100, help="number of frames to render")
    p.add_argument("--out", required=True, help="output dir (creates frames/ and meta/)")
    p.add_argument("--seed", type=int, default=0, help="base seed; frame i uses seed+i")
    p.add_argument("--rt-subframes", type=int, default=32,
                   help="RTX accumulation subframes per capture (denoise/converge)")
    p.add_argument("--horizontal-aperture", type=float, default=20.955,
                   help="camera sensor width (mm); only the focal/aperture RATIO sets fx")
    p.add_argument("--clutter", type=int, default=48,
                   help="number of feature clutter props in the background room")
    p.add_argument("--start-index", type=int, default=1,
                   help="first frame stem index (frame_%%06d), for resuming a run")
    return p


# Parsed up-front (pure). parse_known_args() so any Kit/Carb CLI args are ignored.
_ARGS, _ = build_parser().parse_known_args()


# ---------------------------------------------------------------------------
# 2) LAUNCH Isaac Sim 4.5 headless with the RTX renderer.
#    MUST happen before importing any omni.* / pxr module.
#    Source: isaacsim.simulation_app docs (config keys) + rendering_modes docs.
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp  # noqa: E402  (4.5 re-exports SimulationApp at top level)

# NOTE(renderer string): the task brief said "RayTracedLighting"; NVIDIA's
# rendering_modes docs spell the RTX Real-Time (legacy) mode "RaytracedLighting"
# (lowercase 't'). We use the documented spelling. If Kit rejects it on your build,
# try "RayTracedLighting" -- FLAGGED as a top thing to check on first run.
_RENDER_CONFIG = {
    "headless": True,
    "renderer": "RaytracedLighting",
    "width": 1280,
    "height": 720,
    # keep RT quality modest but feature-full; tune if renders are noisy:
    "anti_aliasing": 3,
    "samples_per_pixel_per_frame": 64,
    "denoiser": True,
    "multi_gpu": False,
}
simulation_app = SimulationApp(_RENDER_CONFIG)

# ---- now it is safe to import Kit / USD ------------------------------------
import carb  # noqa: E402
import omni.usd  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade  # noqa: E402

try:
    from PIL import Image  # bundled with Isaac Sim's python.sh (NVIDIA's own SDG snippets use it)
    _HAS_PIL = True
except Exception:  # pragma: no cover
    _HAS_PIL = False


# ---------------------------------------------------------------------------
# 3) Small USD helpers
# ---------------------------------------------------------------------------
def _gf_matrix_from_np(T: np.ndarray) -> Gf.Matrix4d:
    """4x4 math (column-vector, translation in last COLUMN) -> USD Gf.Matrix4d.

    USD stores transforms row-major for ROW-vectors (p_world = p_local * M), i.e.
    the transpose of the standard math matrix, with translation in the last ROW.
    So we transpose before handing the 16 values to Gf.Matrix4d (row-major ctor).
    """
    Tt = np.asarray(T, dtype=float).T
    return Gf.Matrix4d(*[float(x) for x in Tt.flatten()])


def _np_from_gf_matrix(m: Gf.Matrix4d) -> np.ndarray:
    """USD Gf.Matrix4d (row-vector) -> 4x4 math (column-vector) numpy array."""
    M = np.array([[m.GetRow(i)[j] for j in range(4)] for i in range(4)], dtype=float)
    return M.T


def _set_prim_transform(prim: Usd.Prim, T: np.ndarray) -> None:
    """Replace a prim's xform with a single transform op == math 4x4 ``T``."""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(_gf_matrix_from_np(T))


def _make_preview_material(stage, path: str, diffuse, emissive=(0.0, 0.0, 0.0),
                           roughness: float = 0.5, metallic: float = 0.0) -> UsdShade.Material:
    """A UsdPreviewSurface material (universally supported under RTX)."""
    mtl = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    surf = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mtl.CreateSurfaceOutput().ConnectToSource(surf)
    return mtl


def _make_textured_material(stage, path: str, tex_path: str, tile: float = 1.0) -> UsdShade.Material:
    """UsdPreviewSurface driven by a UsdUVTexture reading ``tex_path`` (tiled).

    Standard preview graph: PrimvarReader('st') -> UsdUVTexture -> diffuseColor.
    Meshes bound to this must carry a 'st' primvar (our wall quads do).
    """
    mtl = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    st_reader = UsdShade.Shader.Define(stage, path + "/stReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_out = st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    tex = UsdShade.Shader.Define(stage, path + "/diffuseTex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex_path)
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    tex_rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex_rgb)
    surf = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mtl.CreateSurfaceOutput().ConnectToSource(surf)
    # tiling is expressed by the 'st' primvar values on each wall (see _add_wall)
    return mtl


def _bind_material(prim: Usd.Prim, mtl: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mtl)


def _add_wall(stage, path: str, corners, tile: float, mtl: UsdShade.Material) -> None:
    """A double-sided quad Mesh from 4 world-space corners, with a tiled 'st'.

    ``corners`` is (4,3) world points (CCW); ``tile`` repeats the texture NxN.
    """
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*map(float, c)) for c in corners])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    st.Set([Gf.Vec2f(0.0, 0.0), Gf.Vec2f(tile, 0.0), Gf.Vec2f(tile, tile), Gf.Vec2f(0.0, tile)])
    _bind_material(mesh.GetPrim(), mtl)


def _generate_noise_texture(png_path: str, size: int = 512, seed: int = 0) -> bool:
    """Write a DESATURATED, feature-rich noise+grid texture PNG (numpy + PIL).

    Grayscale (S~0) keeps it OUT of color_order.yaml's saturated primary bands
    (hsv_min_s=100) so 길남's color judge is not fooled by the background, while
    the high-frequency detail gives VIO/feature trackers something to lock onto.
    """
    if not _HAS_PIL:
        return False
    rng = np.random.default_rng(seed)
    # multi-scale value noise -> lots of corners/edges
    base = np.zeros((size, size), dtype=np.float32)
    for step in (4, 8, 16, 32, 64):
        coarse = rng.random((size // step + 1, size // step + 1)).astype(np.float32)
        up = np.asarray(Image.fromarray((coarse * 255).astype(np.uint8)).resize((size, size)))
        base += up.astype(np.float32) / 255.0
    base /= 5.0
    # faint darker grid lines (structured features)
    grid = np.ones((size, size), dtype=np.float32)
    grid[::32, :] = 0.55
    grid[:, ::32] = 0.55
    val = np.clip(base * grid, 0.12, 0.88)
    img = (np.stack([val, val, val], axis=-1) * 255).astype(np.uint8)  # gray -> S=0
    Image.fromarray(img, "RGB").save(png_path)
    return True


# ---------------------------------------------------------------------------
# 4) Window colours -> RGB inside color_order.yaml's HSV bands
#    OpenCV HSV: red H in [0,10] or [170,179]; green [50,70]; blue [110,130];
#    S>=100, V>=80 (color_order.yaml). We render strongly-saturated primaries and
#    add a matching-hue emissive so the colour survives lighting randomisation.
# ---------------------------------------------------------------------------
WINDOW_RGB = {
    "red":   (0.90, 0.02, 0.02),   # OpenCV hue ~0   -> band [0,10]
    "green": (0.05, 0.80, 0.05),   # OpenCV hue ~60  -> band [50,70]
    "blue":  (0.05, 0.08, 0.90),   # OpenCV hue ~120 -> band [110,130]
}
WINDOW_THICKNESS_M = 0.06  # thin box -> visible frame edges (features) at oblique angles


def _build_window_materials(stage) -> dict:
    mats = {}
    for color, rgb in WINDOW_RGB.items():
        emissive = tuple(0.35 * c for c in rgb)  # keep hue even in shadow
        mats[color] = _make_preview_material(
            stage, f"/World/Looks/win_{color}", rgb, emissive, roughness=0.35
        )
    return mats


# ---------------------------------------------------------------------------
# 5) Scene assembly
# ---------------------------------------------------------------------------
# Enclosing room bounds (metres). Windows are sampled in front of a camera near
# the origin looking ~+X (near/mid/far up to 10 m, lateral within FOV), so a
# generous box contains everything; walls sit BEHIND the windows to limit
# occlusion of labelled windows.
ROOM = dict(xmin=-8.0, xmax=18.0, ymin=-16.0, ymax=16.0, zmin=-3.0, zmax=12.0)


def build_static_world(stage, out_dir: str, clutter_n: int, seed: int):
    """Build the parts that don't change per frame: room + textured walls + clutter
    + the (empty) lights + camera prim. Returns (camera_prim, sun, dome)."""
    UsdGeom.Xform.Define(stage, "/World")

    # ---- feature-rich TEXTURED room (박태민 07/03: never a blank background) ----
    tex_dir = os.path.join(out_dir, "_assets")
    os.makedirs(tex_dir, exist_ok=True)
    tex_png = os.path.join(tex_dir, "bg_noise.png")
    have_tex = _generate_noise_texture(tex_png, size=512, seed=seed)
    if have_tex:
        wall_mtl = _make_textured_material(stage, "/World/Looks/room_tex", tex_png)
    else:
        # PIL unavailable -> fall back to a plain desaturated wall; clutter below
        # still supplies trackable features so the background is never blank.
        carb.log_warn("PIL unavailable: using flat wall material; clutter still adds features.")
        wall_mtl = _make_preview_material(stage, "/World/Looks/room_flat", (0.55, 0.55, 0.55))

    x0, x1 = ROOM["xmin"], ROOM["xmax"]
    y0, y1 = ROOM["ymin"], ROOM["ymax"]
    z0, z1 = ROOM["zmin"], ROOM["zmax"]
    tile = 12.0  # repeat the texture ~this many times across a wall (dense features)
    walls = {
        "floor":   [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],
        "ceiling": [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],
        "back":    [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
        "front":   [(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)],
        "left":    [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
        "right":   [(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)],
    }
    for name, corners in walls.items():
        _add_wall(stage, f"/World/Room/{name}", corners, tile, wall_mtl)

    # ---- 3D feature clutter (guarantees parallax features even if a texture
    #      fails to load). Desaturated colours -> stays out of the colour bands. ----
    UsdGeom.Xform.Define(stage, "/World/Clutter")
    rng = np.random.default_rng(seed + 777)
    for i in range(int(clutter_n)):
        # place near a wall so props sit behind the window volume (less occlusion)
        wall = rng.integers(0, 6)
        if wall == 0:      # back
            pos = [x1 - rng.uniform(0.1, 1.5), rng.uniform(y0, y1), rng.uniform(z0, z1)]
        elif wall == 1:    # floor
            pos = [rng.uniform(x0 + 6, x1), rng.uniform(y0, y1), z0 + rng.uniform(0.1, 1.0)]
        elif wall == 2:    # ceiling
            pos = [rng.uniform(x0 + 6, x1), rng.uniform(y0, y1), z1 - rng.uniform(0.1, 1.0)]
        elif wall == 3:    # left
            pos = [rng.uniform(x0 + 6, x1), y0 + rng.uniform(0.1, 1.0), rng.uniform(z0, z1)]
        else:              # right
            pos = [rng.uniform(x0 + 6, x1), y1 - rng.uniform(0.1, 1.0), rng.uniform(z0, z1)]
        s = rng.uniform(0.2, 0.8)
        cube = UsdGeom.Cube.Define(stage, f"/World/Clutter/box_{i:03d}")
        cube.CreateSizeAttr(2.0)  # local extent [-1,1]; scale below -> edge length s
        _set_prim_transform(cube.GetPrim(), make_transform(np.eye(3) * (s / 2.0), pos))
        g = float(rng.uniform(0.2, 0.8))
        tint = rng.uniform(-0.05, 0.05, size=3)  # tiny tint -> still low saturation
        col = tuple(float(np.clip(g + t, 0.05, 0.95)) for t in tint)
        _bind_material(cube.GetPrim(), _make_preview_material(
            stage, f"/World/Looks/clutter_{i:03d}", col, roughness=0.8))

    # ---- lights (created once; intensity/direction/temperature updated per frame) ----
    UsdGeom.Xform.Define(stage, "/World/Lights")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateAngleAttr(1.0)
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")

    # ---- camera prim (transform + intrinsics set below / per frame) ----
    cam = UsdGeom.Camera.Define(stage, "/World/Camera")
    cam.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10000.0))
    return cam, sun, dome


def set_camera_intrinsics(cam: UsdGeom.Camera, intr, horizontal_aperture_mm: float) -> float:
    """Set focalLength/aperture so the pixel intrinsics equal ``intr``.

    USD/Isaac pinhole: fx = focalLength/horizontalAperture * width (see
    common.intrinsics.CameraIntrinsics.from_focal_aperture and the Isaac camera
    docs). Only the RATIO matters, so we pick the sensor width and solve for
    focalLength. Square pixels + centred principal point (cx=W/2, cy=H/2) hold
    automatically because we set verticalAperture = horizontalAperture*H/W and
    leave the aperture offsets at 0. Returns the effective fx for a sanity check.
    """
    W, H = int(intr.width), int(intr.height)
    ha = float(horizontal_aperture_mm)
    va = ha * (H / W)
    focal = intr.fx / W * ha           # -> fx = focal/ha*W = intr.fx exactly
    cam.CreateFocalLengthAttr(float(focal))
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(va)
    cam.CreateHorizontalApertureOffsetAttr(0.0)  # cx = W/2
    cam.CreateVerticalApertureOffsetAttr(0.0)    # cy = H/2
    fx_eff = focal / ha * W
    fy_eff = focal / va * H
    if abs(fx_eff - intr.fx) > 1e-6 or abs(fy_eff - intr.fy) > 1e-6:
        carb.log_warn(f"intrinsics mismatch: fx_eff={fx_eff} fy_eff={fy_eff} vs "
                      f"target fx={intr.fx} fy={intr.fy}")
    return fx_eff


def update_windows(stage, windows) -> None:
    """(Re)build /World/Windows for this frame's scene.

    CRITICAL: orient each window with window_rotation_from_normal(normal) -- the
    SAME rotation export_dataset reconstructs from the emitted normal -- so the
    rendered quad's corners coincide with the projected label corners.
    """
    if stage.GetPrimAtPath("/World/Windows"):
        stage.RemovePrim("/World/Windows")
    UsdGeom.Xform.Define(stage, "/World/Windows")
    for i, w in enumerate(windows):
        color = w["color"]
        normal = np.asarray(w["normal"], float)
        center = np.asarray(w["center"], float)
        width, height = float(w["width"]), float(w["height"])
        R = window_rotation_from_normal(normal)            # columns [right, up, normal]
        # thin box: local +-1 cube scaled to (w/2, h/2, thk/2), then R + translate.
        RS = R @ np.diag([width / 2.0, height / 2.0, WINDOW_THICKNESS_M / 2.0])
        cube = UsdGeom.Cube.Define(stage, f"/World/Windows/win_{i:02d}_{color}")
        cube.CreateSizeAttr(2.0)                            # local extent [-1,1]
        cube.CreateDoubleSidedAttr(True)                   # visible at +-60 deg
        _set_prim_transform(cube.GetPrim(), make_transform(RS, center))
        _bind_material(cube.GetPrim(), _WINDOW_MATERIALS[color])


def update_lights(sun: UsdLux.DistantLight, dome: UsdLux.DomeLight, lighting) -> None:
    """Randomised lighting per frame (spec 4.1: brightness/direction/colour temp)."""
    brightness = float(lighting["brightness"])
    direction = np.asarray(lighting["direction"], float)
    temp = float(lighting["color_temperature_k"])
    # DistantLight emits along its local -Z; orient so -Z aligns with `direction`.
    R = window_rotation_from_normal(-direction)  # columns[2] = -direction -> emits along +direction
    _set_prim_transform(sun.GetPrim(), make_transform(R, (ROOM["xmax"] * 0.5, 0.0, ROOM["zmax"])))
    for light, base in ((sun, 3000.0), (dome, 350.0)):
        p = light.GetPrim()
        p.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(base * brightness)
        p.CreateAttribute("inputs:colorTemperature", Sdf.ValueTypeNames.Float).Set(temp)
        p.CreateAttribute("inputs:enableColorTemperature", Sdf.ValueTypeNames.Bool).Set(True)


def verify_camera_pose(cam: UsdGeom.Camera, T_target: np.ndarray) -> float:
    """Confirm the USD camera-prim world transform == the pose we will emit.

    Uses UsdGeom.Xformable.ComputeLocalToWorldTransform (the canonical way to read
    a prim's world pose). Returns the max abs elementwise error.
    """
    m = UsdGeom.Xformable(cam.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    T_actual = _np_from_gf_matrix(m)
    return float(np.max(np.abs(T_actual - np.asarray(T_target, float))))


def save_rgb(rgb_data, png_path: str) -> None:
    """Save an rgb-annotator array (H,W,4 uint8) as an RGB PNG."""
    arr = np.asarray(rgb_data)
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    if not _HAS_PIL:
        # extremely defensive fallback; PIL ships with Isaac Sim's python in practice
        np.save(os.path.splitext(png_path)[0] + ".npy", arr)
        return
    Image.fromarray(arr).convert("RGB").save(png_path)


# ---------------------------------------------------------------------------
# 6) MAIN RENDER LOOP
# ---------------------------------------------------------------------------
_WINDOW_MATERIALS: dict = {}   # filled in run(); referenced by update_windows()
_FRAME_INTERVAL_NS = 33_333_333
_T0_NS = 1_720_000_000_000_000_000


def run(args) -> int:
    global _WINDOW_MATERIALS
    frames_dir = os.path.join(args.out, "frames")
    meta_dir = os.path.join(args.out, "meta")
    os.makedirs(frames_dir, exist_ok=True)
    os.makedirs(meta_dir, exist_ok=True)

    intr = default_intrinsics()  # 길남's synth_intrinsics (fx=fy=600, 1280x720)
    assert (int(intr.width), int(intr.height)) == (1280, 720), \
        f"expected 1280x720 intrinsics, got {intr.width}x{intr.height}"

    # Fresh stage, Z-up metres to match the world convention (geometry.py / scene_gen).
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # We drive rendering explicitly via rep.orchestrator.step(); disable capture-on-play.
    carb.settings.get_settings().set("/omni/replicator/captureOnPlay", False)

    cam, sun, dome = build_static_world(stage, args.out, args.clutter, args.seed)
    _WINDOW_MATERIALS = _build_window_materials(stage)
    fx_eff = set_camera_intrinsics(cam, intr, args.horizontal_aperture)

    # Render product + rgb annotator on OUR camera prim (1280x720). Source: Isaac
    # replicator snippets (render_product + AnnotatorRegistry 'rgb' + orchestrator.step).
    render_product = rep.create.render_product("/World/Camera", (int(intr.width), int(intr.height)))
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach(render_product)

    # a few warmup updates so the render pipeline/materials are resident
    for _ in range(5):
        simulation_app.update()

    print(f"[isaac_replicator] out={args.out} frames={args.num_frames} "
          f"seed={args.seed} fx_eff={fx_eff:.3f} renderer={_RENDER_CONFIG['renderer']}")

    n_written = 0
    for i in range(args.num_frames):
        seed_i = args.seed + i
        stem = f"frame_{args.start_index + i:06d}"
        scene = sample_scene(seed_i, intr)  # 1-5 windows, near/mid/far, +-60 deg, lighting, bg

        # --- set the Isaac camera to the SAME pose we will emit (the invariant) ---
        T_cam_usd = np.asarray(scene["camera"]["T_world_cam_usd"], float)
        _set_prim_transform(cam.GetPrim(), T_cam_usd)
        update_windows(stage, scene["windows"])
        update_lights(sun, dome, scene["lighting"])

        # confirm pose on the first frame (and loudly if it ever drifts)
        if i == 0:
            err = verify_camera_pose(cam, T_cam_usd)
            print(f"[isaac_replicator] camera pose round-trip max err = {err:.3e} "
                  f"(should be ~0; USD<->math transform check)")
            if err > 1e-4:
                carb.log_warn(f"camera pose mismatch {err}: rendered pixels may not match labels")

        # --- render + capture ---
        rep.orchestrator.step(rt_subframes=int(args.rt_subframes))
        rgb = rgb_annot.get_data()
        save_rgb(rgb, os.path.join(frames_dir, stem + ".png"))

        # --- metadata EXACTLY per sim/metadata_schema.md (scene_to_metadata builds it) ---
        # image is relative to the JSON's dir; frames/ is a sibling of meta/, so
        # "../frames/<stem>.png" resolves for export_dataset._load_frame_meta.
        meta = scene_to_metadata(
            scene,
            image=f"../frames/{stem}.png",
            timestamp_ns=_T0_NS + (args.start_index + i) * _FRAME_INTERVAL_NS,
            frame_id=args.start_index + i,
        )
        with open(os.path.join(meta_dir, stem + ".json"), "w", encoding="utf-8") as f:
            json.dump(meta, f)
        n_written += 1
        if (i + 1) % 20 == 0 or i == 0:
            print(f"[isaac_replicator] rendered {i + 1}/{args.num_frames}: {stem}")

    print(f"[isaac_replicator] done: wrote {n_written} frames -> {frames_dir} and {meta_dir}")
    # NOTE: export_dataset.py has no --images-dir flag; it resolves each frame's
    # image from the JSON "image" field (we wrote "../frames/<stem>.png", relative
    # to meta/, which lands on frames/). So the downstream command is just:
    print("[isaac_replicator] next: python3 sim/export_dataset.py --mode from-metadata "
          f"--metadata-dir {meta_dir} --out <dataset_root>")
    return 0


if __name__ == "__main__":
    _t = time.time()
    _rc = 1
    try:
        _rc = run(_ARGS)
    finally:
        print(f"[isaac_replicator] elapsed {time.time() - _t:.1f}s; closing SimulationApp")
        simulation_app.close()
    raise SystemExit(_rc)
