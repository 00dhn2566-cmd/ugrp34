"""3D geometry: window corners, frame conversions, pinhole projection.

Coordinate conventions (READ THIS — the whole dataset depends on it)
--------------------------------------------------------------------
World frame        : Isaac Sim default, right-handed, +Z up, metres.
USD camera frame   : how a USD/Isaac-Sim camera prim is oriented ->
                     looks down -Z, +X right, +Y up  (OpenGL-style).
CV camera frame    : the frame this project projects in ->
                     looks down +Z, +X right, +Y down (OpenCV-style).
Image frame        : +u right, +v down, origin at top-left pixel.

The rotation that takes a point expressed in the USD camera frame into the CV
camera frame flips Y and Z:

    R_USD_TO_CV = diag(1, -1, -1)

So if Isaac Sim gives you the camera pose as ``T_world_cam_usd`` (a 4x4 placing
the USD camera prim in the world), a world point is projected with:

    p_cam_usd = inv(T_world_cam_usd) @ p_world
    p_cam_cv  = R_USD_TO_CV @ p_cam_usd
    pixel     = project_points(p_cam_cv, K)

``world_to_camera_cv`` does the first two steps for you.

Window corner order (fixed, geometric — NOT image-position based)
-----------------------------------------------------------------
A window lives in its own local frame: +X_local = right, +Y_local = up,
+Z_local = outward normal (the face the drone approaches). Corners are returned
in this order, which reads clockwise when the window is viewed from the front:

    0 top_left   (-w/2, +h/2)
    1 top_right  (+w/2, +h/2)
    2 bottom_right(+w/2, -h/2)
    3 bottom_left (-w/2, -h/2)

This order is the same one used for YOLO-pose keypoints. Because it is defined
in the window's own frame, corner identity is stable even when the window is
seen at an oblique angle (up to +/-60 deg per spec 4.1).
"""
from __future__ import annotations

import numpy as np

# USD/OpenGL camera frame -> OpenCV camera frame (flip Y and Z).
R_USD_TO_CV = np.diag([1.0, -1.0, -1.0]).astype(np.float64)

# Fixed geometric corner order used everywhere (labels, keypoints, viz).
CORNER_ORDER = ("top_left", "top_right", "bottom_right", "bottom_left")


def quat_wxyz_to_R(q) -> np.ndarray:
    """Unit quaternion (w, x, y, z) -> 3x3 rotation matrix.

    (w, x, y, z) is the ordering used by Isaac Sim / USD `orientation_quat_wxyz`
    and by interface/isaacsim_trajectory.schema.json.
    """
    w, x, y, z = (float(v) for v in q)
    n = math_sqrt(w * w + x * x + y * y + z * z)
    if n == 0.0:
        raise ValueError("zero-norm quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def math_sqrt(v: float) -> float:
    # tiny local helper to avoid importing math just for one call
    return float(np.sqrt(v))


def make_transform(R: np.ndarray, t) -> np.ndarray:
    """Compose a 3x3 rotation and a length-3 translation into a 4x4 homogeneous
    transform."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Inverse of a rigid 4x4 transform (transpose R, re-project t)."""
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def _to_points_array(points) -> np.ndarray:
    p = np.asarray(points, dtype=np.float64)
    if p.ndim == 1:
        p = p.reshape(1, 3)
    if p.shape[-1] != 3:
        raise ValueError(f"expected (..,3) points, got shape {p.shape}")
    return p


def world_to_camera_cv(points_world, T_world_cam_usd: np.ndarray) -> np.ndarray:
    """Transform world points (N,3) into the OpenCV camera frame.

    ``T_world_cam_usd`` places the USD camera prim in the world (as Isaac Sim
    reports it). Returns (N,3) points in the CV camera frame (+Z forward).
    """
    p = _to_points_array(points_world)
    T_cam_world = invert_transform(T_world_cam_usd)
    ones = np.ones((p.shape[0], 1))
    p_h = np.hstack([p, ones])  # (N,4)
    p_cam_usd = (T_cam_world @ p_h.T).T[:, :3]  # (N,3) in USD cam frame
    p_cam_cv = (R_USD_TO_CV @ p_cam_usd.T).T  # (N,3) in CV cam frame
    return p_cam_cv


def project_points(points_cam_cv, K: np.ndarray):
    """Pinhole-project CV-camera-frame points (N,3) to pixels.

    Returns
    -------
    pixels : (N,2) float  -- (u, v); undefined (nan) where the point is not in
             front of the camera.
    depth  : (N,)  float  -- Z_cam (metres); <=0 means behind the camera.
    in_front : (N,) bool  -- depth > 0.
    """
    p = _to_points_array(points_cam_cv)
    K = np.asarray(K, dtype=np.float64)
    z = p[:, 2]
    in_front = z > 1e-9
    uv = np.full((p.shape[0], 2), np.nan, dtype=np.float64)
    zf = np.where(in_front, z, np.nan)
    x_n = p[:, 0] / zf
    y_n = p[:, 1] / zf
    uv[:, 0] = K[0, 0] * x_n + K[0, 2]
    uv[:, 1] = K[1, 1] * y_n + K[1, 2]
    return uv, z, in_front


def window_corners_local(width: float, height: float) -> np.ndarray:
    """4 window corners in the window's local frame, in CORNER_ORDER.

    Returns (4,3): columns are (x_right, y_up, z_normal=0).
    """
    hw, hh = width / 2.0, height / 2.0
    return np.array(
        [
            [-hw, +hh, 0.0],  # top_left
            [+hw, +hh, 0.0],  # top_right
            [+hw, -hh, 0.0],  # bottom_right
            [-hw, -hh, 0.0],  # bottom_left
        ],
        dtype=np.float64,
    )


def window_corners_world(
    center, R_world_win: np.ndarray, width: float, height: float
) -> np.ndarray:
    """Window corners expressed in world coordinates, in CORNER_ORDER.

    Parameters
    ----------
    center : (3,) world position of the window centre.
    R_world_win : (3,3) rotation taking window-local axes into world axes.
    width, height : window opening size in metres.
    """
    local = window_corners_local(width, height)  # (4,3)
    center = np.asarray(center, dtype=np.float64).reshape(3)
    return (R_world_win @ local.T).T + center
