"""``state_window_to_obs`` -- the VIO/GT -> RL integration seam.

WHAT / WHY
----------
The RL *training* loop does NOT ride the ``state_window_interface`` spec: training
uses the vectorised in-sim observation built by ``window_env.WindowTraversalEnv``
directly (state_window_interface_spec_v0.1 §4: "RL 훈련 루프는 본 규격을 타지
않는다"). This module is the *other* path -- **integration / inference** -- and its
whole job is to prove that the same 17-dim training observation is **DERIVABLE**
from the real interface dicts (spec §4: "훈련 관측이 본 규격에서 유도되도록 정합만
맞춘다"), doing the ``world->body`` transform the spec leaves to the consumer
(spec §1 원칙 2: "드론 기준 상대 위치 변환은 소비자(궤적) 몫").

``state_window_to_obs(drone_state, window_map, target_order_index)`` takes the two
real interface messages -- the drone state (~``nav_msgs/Odometry``, spec §2/§6.1)
and the window 3D map (spec §3/§6.2) -- and returns the **exact same** flat obs
vector layout as the env (imported ``OBS_*`` constants below are the single source
of truth, so this can never drift from ``window_env``).

GT now / VIO later (spec §1 원칙 1, §4, §9-조윤호)
-------------------------------------------------
Same schema, two producers:
  * **now (GT substitution):** 윤호 feeds Isaac GT pose + GT window on this schema
    so the RL consumer can be exercised before VIO/궤적-담당 is finalised.
  * **at integration (inference):** the drone state comes from 태민's VIO topic
    ``/ov_msckf/poseimu`` (``geometry_msgs``, quaternion **XYZW**) and the window
    map from 태민's ``/window_positions`` (``window_recon_node.py``). The upstream
    detector/VIO quality then bounds policy performance (README §7.6), which is why
    training should also pass through injected estimator noise before this bridge
    is trusted (README §7.6 / ``EnvConfig.observation_noise``).

QUATERNION ORDER (CONVENTIONS.md "Quaternion order is per-interface")
--------------------------------------------------------------------
``drone_state["orientation"]`` here is **XYZW** ``[qx,qy,qz,qw]`` -- this matches
태민's live ``/ov_msckf/poseimu`` (ROS Hamilton) and ``state_window_interface``
§2.1-② (ROS 관례), and is the SAME order as the vision GT-pose stream. It is NOT
the WXYZ order the control-side ``interface/`` schemas use
(``orientation_quat_wxyz``). There is **no single global quat order** -- we reorder
XYZW->WXYZ and reuse the shared ``common.geometry.quat_wxyz_to_R`` rather than
re-deriving the rotation matrix.

FRAMES (see also ``window_env`` COORDINATE FRAMES + CONVENTIONS.md)
------------------------------------------------------------------
World is right-handed, +Z up, metres (spec §6.1 ``frame:"world"``). The obs mixes
two frames, exactly as the env builds them:
  * window relative position + normal -> **heading frame** (yaw-only, gravity
    aligned), yaw = horizontal bearing from the drone to the target window
    (matches ``WindowTraversalEnv._yaw_to_target``);
  * body velocity + gravity/attitude -> **body frame** from the drone orientation.
    ``lin_vel`` is already BODY per spec §2.1-③ (``nav_msgs/Odometry`` twist is in
    the child/body frame), so it is used as-is; gravity ``-Z_world`` is rotated
    into the body frame for the IMU-like attitude signal.

NOTE (train/inference frame gap, README §7.6): the env's *training* body frame is
synthesised from the thrust vector + heading yaw (``_attitude``), whereas here the
body frame is the *measured* attitude quaternion. The velocity/gravity slices are
frame-consistent within each path; the residual is exactly the sim-to-real
attitude gap and is a domain-randomisation / obs-noise concern, not an adapter bug.
"""
from __future__ import annotations

import math
import os
import sys
from typing import Any, Mapping, Optional, Sequence

import numpy as np

# --- bootstrap so `common` (repo root) and sibling rl modules import from any cwd
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from common import quat_wxyz_to_R  # noqa: E402  (shared rotation math -- do NOT re-derive)

# Reuse the env's obs layout + heading-frame helpers as the SINGLE source of truth,
# so this integration path can never drift from the training observation.
from window_env import (  # noqa: E402
    ACT_DIM,
    G,  # noqa: F401  (kept for parity / future accel fields; UP is the one used)
    OBS_DIM,
    OBS_GRAV_BODY,
    OBS_PREV_ACTION,
    OBS_REL_WIN_POS,
    OBS_VEL_BODY,
    OBS_WIN_NORMAL,
    OBS_WIN_SIZE,
    UP,
    _normalize,
    _rot_z,
)


# ---------------------------------------------------------------------------
# small parsing helpers
# ---------------------------------------------------------------------------
def quat_xyzw_to_R(q: Sequence[float]) -> np.ndarray:
    """Unit quaternion (x, y, z, w) -> 3x3 rotation ``R_world_body``.

    ``state_window_interface`` §2.1-② / 태민 ``/ov_msckf/poseimu`` use XYZW; the
    shared helper takes WXYZ, so reorder before delegating (CONVENTIONS.md
    "Quaternion order is per-interface").
    """
    q = np.asarray(q, dtype=np.float64).reshape(4)
    qx, qy, qz, qw = q
    return quat_wxyz_to_R((qw, qx, qy, qz))


def _select_window(
    window_map: Mapping[str, Any], target_order_index: int
) -> Optional[Mapping[str, Any]]:
    """Pick the window whose ``order_index`` == the requested target (spec §3.1,
    v0.2 §3.1 traversal-order table). ``None`` if it is not in the map yet.

    The caller owns *which* order_index is "next" (mirrors the env advancing
    ``_target_idx`` on each pass); this only resolves the id -> window object out
    of the S-2 "known full map" (spec §3.2), so a still-tracking or already-passed
    window is simply the caller's choice of ``target_order_index``.
    """
    for win in window_map.get("windows", []):
        if int(win.get("order_index", -1)) == int(target_order_index):
            return win
    return None


def _window_center(win: Mapping[str, Any]) -> np.ndarray:
    """Window centre (spec §6.2 convenience field); fall back to the corner mean
    if a producer omitted it (S-3-style raw reconstruction)."""
    c = win.get("center")
    if c is not None:
        return np.asarray(c, dtype=np.float64).reshape(3)
    corners = np.asarray(win["corners_3d"], dtype=np.float64).reshape(4, 3)
    return corners.mean(axis=0)


def _window_size(win: Mapping[str, Any]) -> np.ndarray:
    """(width, height) in metres (spec §6.2 ``size_wh``); fall back to corner edge
    lengths in CORNER_ORDER (TL->TR = width, TR->BR = height), matching 태민's
    ``window_recon_node`` size computation."""
    s = win.get("size_wh")
    if s is not None:
        return np.asarray(s, dtype=np.float64).reshape(2)
    c = np.asarray(win["corners_3d"], dtype=np.float64).reshape(4, 3)
    width = float(np.linalg.norm(c[1] - c[0]))
    height = float(np.linalg.norm(c[2] - c[1]))
    return np.array([width, height])


def _window_normal(win: Mapping[str, Any]) -> np.ndarray:
    """Unit window normal in world.

    Prefer the explicit ``normal`` field (spec §6.2). If absent, derive it from
    the corners per state_window_interface_spec_v0.1 §3.1 (잠정 확정 2026-08-08):

        n = normalize(cross(c3 - c0, c1 - c0))      # (BL-TL) x (TR-TL)

    with CORNER_ORDER TL(0)->TR(1)->BR(2)->BL(3) wound clockwise as seen from the
    approach side. The spec pins the sign: the normal points TOWARD the approach
    side (the drone), which is exactly the env's ``Window.normal`` (``R[:, 2]``
    = +Z_local of ``common.geometry``). The opposite order
    ``cross(c1-c0, c3-c0)`` is antiparallel and is explicitly forbidden by §3.1.
    This fallback is live at integration: 태민's ``/window_positions``
    (``window_recon_node.py``) ships no ``normal`` field.
    """
    n = win.get("normal")
    if n is not None:
        v = np.asarray(n, dtype=np.float64).reshape(3)
        if float(np.linalg.norm(v)) > 1e-9:
            return _normalize(v)
    c = np.asarray(win["corners_3d"], dtype=np.float64).reshape(4, 3)
    # spec §3.1: (BL-TL) x (TR-TL) -> toward the approach side == Window.normal
    return _normalize(np.cross(c[3] - c[0], c[1] - c[0]))


def _heading_yaw(
    p: np.ndarray, center: np.ndarray, R_world_body: np.ndarray
) -> float:
    """Heading yaw = horizontal bearing from the drone to the target window,
    identical to ``WindowTraversalEnv._yaw_to_target``. Degenerate case (drone
    directly above/below the target): fall back to the drone's own heading (body
    +X projected to horizontal), else 0."""
    d = center - p
    if abs(float(d[0])) < 1e-9 and abs(float(d[1])) < 1e-9:
        bx = R_world_body[:, 0]
        if abs(float(bx[0])) < 1e-9 and abs(float(bx[1])) < 1e-9:
            return 0.0
        return math.atan2(float(bx[1]), float(bx[0]))
    return math.atan2(float(d[1]), float(d[0]))


# ---------------------------------------------------------------------------
# the integration seam
# ---------------------------------------------------------------------------
def state_window_to_obs(
    drone_state: Mapping[str, Any],
    window_map: Mapping[str, Any],
    target_order_index: int,
    prev_action: Optional[Sequence[float]] = None,
) -> np.ndarray:
    """Derive the env's 17-dim training observation from the real interface dicts.

    Parameters
    ----------
    drone_state : ``state_window_interface`` §6.1 (~``nav_msgs/Odometry``):
        ``position`` [x,y,z] m world, ``orientation`` [qx,qy,qz,qw] **XYZW**,
        ``lin_vel`` [vx,vy,vz] **BODY** (spec §2.1-③). ``ang_vel``/covariances are
        accepted but not used by this obs layout.
    window_map : ``state_window_interface`` §6.2 (``WindowMap``): ``windows`` list
        of ``{order_index, center, normal, size_wh, corners_3d, ...}`` in world.
    target_order_index : which window (by ``order_index``, v0.2 §3.1 table) the
        policy is currently aiming for. The caller advances this on each pass.
    prev_action : the policy's previous normalised action (obs slice
        ``OBS_PREV_ACTION``). This is policy state, NOT part of the interface, so
        the caller must thread it through; defaults to zeros (e.g. first step).

    Returns
    -------
    obs : ``np.ndarray`` shape ``(OBS_DIM,)`` float32 -- byte-for-byte the same
        layout ``WindowTraversalEnv._build_obs`` produces (imported ``OBS_*``).

    No GT depth / GT world pose is exposed as a raw obs field (README §7.1): every
    window quantity is a *relative* pose in the heading frame, exactly what a
    detector + VIO would report.
    """
    # --- drone state (spec §6.1) ------------------------------------------
    p = np.asarray(drone_state["position"], dtype=np.float64).reshape(3)
    R_world_body = quat_xyzw_to_R(drone_state["orientation"])  # XYZW -> R
    lin_vel_body = np.asarray(drone_state["lin_vel"], dtype=np.float64).reshape(3)
    # spec §2.1-③: Odometry twist is already BODY -> use directly, no transform.

    # --- target window (world), transformed into the heading frame --------
    win = _select_window(window_map, target_order_index)
    if win is not None:
        center = _window_center(win)
        normal = _window_normal(win)
        size = _window_size(win)
        yaw = _heading_yaw(p, center, R_world_body)
        R_heading = _rot_z(yaw)                       # yaw-only heading frame
        rel_pos_h = R_heading.T @ (center - p)        # world->heading (consumer's job)
        normal_h = R_heading.T @ normal
    else:
        # target not in the known map yet -> zeros (mirrors the env once all
        # windows are passed / target unavailable).
        rel_pos_h = np.zeros(3)
        normal_h = np.zeros(3)
        size = np.zeros(2)

    # --- body-frame attitude signals --------------------------------------
    grav_body = R_world_body.T @ (-UP)  # unit gravity in body frame (IMU-like attitude)

    if prev_action is None:
        prev = np.zeros(ACT_DIM, dtype=np.float64)
    else:
        prev = np.asarray(prev_action, dtype=np.float64).reshape(ACT_DIM)

    # --- assemble (same slices as window_env, imported so they can't drift) --
    obs = np.empty(OBS_DIM, dtype=np.float64)
    obs[OBS_REL_WIN_POS] = rel_pos_h
    obs[OBS_WIN_NORMAL] = normal_h
    obs[OBS_WIN_SIZE] = size
    obs[OBS_VEL_BODY] = lin_vel_body
    obs[OBS_GRAV_BODY] = grav_body
    obs[OBS_PREV_ACTION] = prev
    return obs.astype(np.float32)
