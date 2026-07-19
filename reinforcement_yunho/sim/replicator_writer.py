"""Custom Isaac Sim omni.replicator Writer that emits YOLO-pose window labels.

WHAT
----
Per rendered frame it projects each window's 4 world-space corners into the
image and writes:
  * an RGB ``.png``   (the rendered frame)
  * a YOLO-pose ``.txt`` label  (one line per fully-visible window)

WHY THE SPLIT
-------------
Everything that actually needs Isaac Sim (``omni.replicator``) is import-guarded
so this file imports fine on a plain machine. The *real* logic -- corner
projection + label building -- lives in the pure functions :func:`build_label_lines`
/ :func:`build_label_records`, which depend only on numpy + ``common.geometry`` and
are unit-tested by ``sim/smoke_test.py`` and consumed by ``sim/export_dataset.py``.
The Writer class is a thin shell that calls it.

Label format (CONVENTIONS.md, spec §4.3), all values normalised to [0,1]:
    <class> <cx> <cy> <w> <h> <u1> <v1> <vis1> ... <u4> <v4> <vis4>   (17 tokens)
  * class  = order_index  (red 0 / green 1 / blue 2)  -> ORDER_INDEX
  * cx cy w h = axis-aligned bbox tight around the 4 projected corners
  * keypoints in CORNER_ORDER (0 top_left, 1 top_right, 2 bottom_right,
    3 bottom_left); vis = 1 for every corner in dataset 1 (policy A).
The 17-token count is a hard contract: 길남's gt_stream.parse_label_line raises
on anything else (overall_gilnam/vision/gt_stream.py, spec §4.3).

Dataset-1 visibility rule (CONVENTIONS.md): a window is labelled only if ALL 4
corners are in front of the camera AND inside the image rectangle [0,W]x[0,H].
"""
from __future__ import annotations

import os
import sys
from typing import Callable, Dict, List, Sequence, Tuple

import numpy as np

# --- import the shared 'common' package regardless of cwd --------------------
# bootstrap (CONVENTIONS.md): _ROOT = repo root -> put on sys.path -> import common
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from common import (  # noqa: E402
    CameraIntrinsics,
    CORNER_ORDER,
    project_points,
    quat_wxyz_to_R,
    window_corners_world,
    world_to_camera_cv,
)

# Colour -> traversal order_index == YOLO class (CONVENTIONS.md colour/class map).
ORDER_INDEX: Dict[str, int] = {"red": 0, "green": 1, "blue": 2}

# A "window" passed to build_label_lines is a dict with these keys:
#   color        : "red" | "green" | "blue"
#   center       : (3,) world position of the window centre
#   width,height : opening size in metres
#   ONE of:
#     R_world_win : (3,3) rotation window-local -> world
#     quat_wxyz   : (4,)  unit quaternion (w,x,y,z), same as R_world_win
# (scene_gen produces R_world_win; export_dataset resolves a bare "normal" into
#  R_world_win via scene_gen.window_rotation_from_normal before calling here.)
Window = Dict[str, object]


# ======================================================================
#  PURE, TESTABLE LOGIC  (numpy + common only; no Isaac Sim)
# ======================================================================
def _window_rotation(window: Window) -> np.ndarray:
    """Resolve a window's 3x3 world<-local rotation from either representation."""
    R = window.get("R_world_win")
    if R is not None:
        return np.asarray(R, dtype=np.float64).reshape(3, 3)
    q = window.get("quat_wxyz")
    if q is not None:
        return quat_wxyz_to_R(q)  # WXYZ order (CONVENTIONS.md quaternion table)
    raise KeyError("window needs either 'R_world_win' (3x3) or 'quat_wxyz' (4,)")


def project_window_corners(
    window: Window,
    T_world_cam_usd: np.ndarray,
    intr: CameraIntrinsics,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project one window's 4 corners to pixels.

    Uses the shared pipeline (do NOT re-derive projection, CONVENTIONS.md):
    ``window_corners_world`` -> ``world_to_camera_cv`` -> ``project_points``.
    Note ``world_to_camera_cv`` applies the USD->CV flip, so ``T_world_cam_usd``
    must be the USD camera-prim pose (Isaac Sim convention). scene_gen builds this
    from a CV pose via ``cv_to_usd_transform`` (see its cross-check comment).

    Returns
    -------
    uv       : (4,2) pixel coords in CORNER_ORDER (nan where behind camera)
    depth    : (4,)  Z_cam metres (<=0 == behind)
    in_front : (4,)  bool, depth > 0
    """
    R_world_win = _window_rotation(window)
    corners_world = window_corners_world(
        window["center"], R_world_win, float(window["width"]), float(window["height"])
    )  # (4,3) in CORNER_ORDER
    corners_cam_cv = world_to_camera_cv(corners_world, T_world_cam_usd)
    uv, depth, in_front = project_points(corners_cam_cv, intr.K())
    return uv, depth, in_front


def _fully_visible(uv: np.ndarray, in_front: np.ndarray, width: int, height: int) -> bool:
    """Dataset-1 rule: all 4 corners in front AND inside [0,W]x[0,H]."""
    if not np.all(in_front):
        return False
    if np.any(np.isnan(uv)):
        return False
    u, v = uv[:, 0], uv[:, 1]
    return bool(
        np.all(u >= 0.0) and np.all(u <= width) and np.all(v >= 0.0) and np.all(v <= height)
    )


def build_label_records(
    windows: Sequence[Window],
    T_world_cam_usd: np.ndarray,
    intr: CameraIntrinsics,
    *,
    precision: int = 6,
) -> List[dict]:
    """Build one record per FULLY-VISIBLE window (the core of the Writer).

    Each record keeps the label line together with the source window so callers
    (export_dataset) can attach per-window side info such as ``distance_m`` for
    ``meta.jsonl`` without re-projecting. See the module docstring for the token
    layout. Dataset-1 rule applies (all 4 corners in front & inside the frame).

    Returns
    -------
    list[dict] : {"line": str, "window_index": int, "order_index": int,
                  "color": str, "uv": (4,2) float pixels}.
    """
    W, H = int(intr.width), int(intr.height)
    records: List[dict] = []
    for idx, win in enumerate(windows):
        color = win["color"]
        if color not in ORDER_INDEX:
            raise ValueError(
                f"unknown window colour {color!r}; expected one of {list(ORDER_INDEX)}"
            )
        cls = ORDER_INDEX[color]

        uv, _depth, in_front = project_window_corners(win, T_world_cam_usd, intr)
        if not _fully_visible(uv, in_front, W, H):
            continue  # dataset 1: skip windows with any corner off-frame/behind

        u, v = uv[:, 0], uv[:, 1]
        # axis-aligned bbox tight around the 4 projected corners
        u_min, u_max = float(u.min()), float(u.max())
        v_min, v_max = float(v.min()), float(v.max())
        cx = ((u_min + u_max) / 2.0) / W
        cy = ((v_min + v_max) / 2.0) / H
        bw = (u_max - u_min) / W
        bh = (v_max - v_min) / H

        tok: List[str] = [
            str(cls),
            f"{cx:.{precision}f}",
            f"{cy:.{precision}f}",
            f"{bw:.{precision}f}",
            f"{bh:.{precision}f}",
        ]
        # keypoints in CORNER_ORDER; vis=1 for all corners in dataset 1 (policy A)
        for i in range(len(CORNER_ORDER)):
            tok.append(f"{u[i] / W:.{precision}f}")
            tok.append(f"{v[i] / H:.{precision}f}")
            tok.append("1")
        records.append(
            {
                "line": " ".join(tok),
                "window_index": idx,
                "order_index": cls,
                "color": color,
                "uv": uv,
            }
        )
    return records


def build_label_lines(
    windows: Sequence[Window],
    T_world_cam_usd: np.ndarray,
    intr: CameraIntrinsics,
    *,
    precision: int = 6,
) -> List[str]:
    """YOLO-pose label lines for one frame (thin wrapper over build_label_records).

    One 17-token line per fully-visible window; empty list if nothing is visible.
    """
    return [r["line"] for r in build_label_records(windows, T_world_cam_usd, intr, precision=precision)]


# ======================================================================
#  ISAAC SIM WRITER  (import-guarded stub -- runs only inside Isaac Sim)
# ======================================================================
try:
    import omni.replicator.core as rep  # type: ignore
    from omni.replicator.core import (  # type: ignore
        AnnotatorRegistry,
        BackendDispatch,
        Writer,
        WriterRegistry,
    )

    _HAS_REPLICATOR = True
    _REPLICATOR_IMPORT_ERROR: Exception | None = None
except Exception as _e:  # pragma: no cover - exercised only inside Isaac Sim
    rep = None  # type: ignore
    _HAS_REPLICATOR = False
    _REPLICATOR_IMPORT_ERROR = _e


# SceneProvider() -> (T_world_cam_usd (4,4), windows list) for the current frame.
# The Replicator graph setup supplies this: the window poses come straight from
# the USD stage, and the camera pose from the camera prim's world transform.
# Deriving the pose this way (rather than from the camera_params annotator's
# view matrix) keeps the frame conventions unambiguous -- see CONVENTIONS.md.
SceneProvider = Callable[[], Tuple[np.ndarray, List[Window]]]


if _HAS_REPLICATOR:  # pragma: no cover - only importable inside Isaac Sim

    class WindowCornerWriter(Writer):
        """omni.replicator Writer: RGB png + YOLO-pose txt per frame.

        The heavy lifting is delegated to :func:`build_label_lines`; this class
        only wires Replicator's RGB annotator + the per-frame scene into it and
        writes the two files with matching stems (CONVENTIONS.md dataset layout).
        """

        def __init__(
            self,
            output_dir: str,
            intrinsics: CameraIntrinsics,
            scene_provider: SceneProvider,
            image_output_format: str = "png",
        ) -> None:
            self._frame_id = 0
            self.intrinsics = intrinsics
            self.scene_provider = scene_provider
            self.image_output_format = image_output_format
            self.backend = BackendDispatch({"paths": {"out_dir": output_dir}})
            os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
            os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)
            # Only RGB is needed for dataset 1 (labels come from geometry, not
            # from segmentation). Request more annotators here for dataset 2.
            self.annotators = [AnnotatorRegistry.get_annotator("rgb")]

        def write(self, data: dict) -> None:
            stem = f"frame_{self._frame_id:06d}"
            rgb = data["rgb"]  # (H,W,4) uint8

            T_world_cam_usd, windows = self.scene_provider()
            lines = build_label_lines(windows, T_world_cam_usd, self.intrinsics)

            self.backend.write_image(f"images/{stem}.{self.image_output_format}", rgb)
            self.backend.write_blob(
                f"labels/{stem}.txt", ("\n".join(lines) + ("\n" if lines else "")).encode()
            )
            self._frame_id += 1

    def register() -> None:
        """Register the writer so ``rep.writers.get('WindowCornerWriter')`` works."""
        WriterRegistry.register(WindowCornerWriter)

else:

    class WindowCornerWriter:  # type: ignore[no-redef]
        """Stub used when omni.replicator is unavailable (i.e. outside Isaac Sim).

        Import still succeeds so the pure logic (build_label_lines) is usable and
        testable; only instantiation fails, with a clear message.
        """

        def __init__(self, *args, **kwargs) -> None:
            raise ImportError(
                "WindowCornerWriter needs omni.replicator (Isaac Sim). Original "
                f"import error: {_REPLICATOR_IMPORT_ERROR!r}. Run inside the Isaac "
                "Sim python env; use build_label_lines() directly for offline tests."
            )

    def register() -> None:
        raise ImportError(
            "omni.replicator unavailable; run inside Isaac Sim to register the writer."
        )
