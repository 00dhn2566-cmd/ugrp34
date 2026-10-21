"""Shared, dependency-light building blocks for the drone-window sim/RL work.

Only depends on numpy (+ pyyaml for the yaml helpers). Nothing in here imports
Isaac Sim, so it is importable and unit-testable on any machine.
"""
from .intrinsics import CameraIntrinsics
from .geometry import (
    R_USD_TO_CV,
    CORNER_ORDER,
    quat_wxyz_to_R,
    make_transform,
    invert_transform,
    R_IC_SPEC,
    T_IC_SPEC,
    T_world_body_from_cam_cv,
    world_to_camera_cv,
    project_points,
    window_corners_local,
    window_corners_world,
)

__all__ = [
    "CameraIntrinsics",
    "R_USD_TO_CV",
    "CORNER_ORDER",
    "quat_wxyz_to_R",
    "make_transform",
    "invert_transform",
    "R_IC_SPEC",
    "T_IC_SPEC",
    "T_world_body_from_cam_cv",
    "world_to_camera_cv",
    "project_points",
    "window_corners_local",
    "window_corners_world",
]
