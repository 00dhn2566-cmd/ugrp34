"""Control <-> Isaac-Sim interface: motor-command, trajectory & waypoints JSON.

WHAT
----
This module pins three JSON/YAML contracts at the control boundary:

  * a *motor-command* stream  -- the real control<->sim boundary: four per-rotor
    angular-velocity setpoints (rad/s) per frame. Motor/propeller physics is
    Isaac Sim's job (CONVENTIONS.md "Trajectory / motor-command interface").
  * a *trajectory* stream     -- 성진's control output: a body pose per frame
    (position, yaw, wxyz orientation quaternion). Written by 성진 as
    isaacsim_trajectory.json; useful for a PID follower / bootstrapping.
  * a *waypoints config*      -- 성진's controller INPUT (control_seoungjin/
    sample/INPUT_FORMAT.md): {waypoints, limits, dt}. THIS is the RL->control
    seam -- the policy emits waypoints, they become this config, 성진 plans a
    minimum-time trajectory through them. No yaw/time/quat in the input.

For each contract this module provides typed dataclasses
(MotorCommandsFile/MotorFrame, TrajectoryFile/TrajectoryFrame, WaypointsConfig),
load_json / save_json round-trip helpers, validate(obj[, kind]) that prefers
`jsonschema` (validating against the sibling *.schema.json files) and falls back
to an *equivalent* pure-Python structural validator when jsonschema is not
installed -- both paths give the SAME accept/reject decision -- and
trajectory_frame_to_T(frame) -> 4x4 world<-body transform (composed with
common.geometry; projection/rotation math is NEVER re-derived here).

WHY the two validation paths must agree: the interface has to be checkable both
inside the Isaac Sim python env (where jsonschema is usually present) and on a
bare numpy-only box (this repo's default), without changing which files are
accepted.

Conventions (CONVENTIONS.md):
  * Quaternions are WXYZ everywhere; 성진's orientation_quat_wxyz is yaw-only.
  * Timestamps on THIS control JSON side (motor + trajectory) are FLOAT SECONDS
    (성진's real output, e.g. 0.01). The integer-NANOSECOND clock is the SEPARATE
    vision/VIO stream (§5 vision message, the flight-data bag, EuRoC-ASL GT) --
    do not conflate the two.
  * World frame is right-handed, +Z up, metres.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, field
from typing import Any, ClassVar, Dict, List, Optional, Union

# --- bootstrap so `from common import ...` works regardless of cwd -----------
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import numpy as np  # noqa: E402  (installed; used by the converter + smoke test)

from common.geometry import make_transform, quat_wxyz_to_R  # noqa: E402

# --- optional dependency: jsonschema (import-guarded, per requirements.txt) ---
try:
    import jsonschema  # type: ignore
    _HAS_JSONSCHEMA = True
except Exception:  # pragma: no cover - depends on the environment
    jsonschema = None  # type: ignore
    _HAS_JSONSCHEMA = False

# Schema files live next to this module.
_HERE = os.path.dirname(os.path.abspath(__file__))
MOTOR_SCHEMA_PATH = os.path.join(_HERE, "isaacsim_motor_commands.schema.json")
TRAJECTORY_SCHEMA_PATH = os.path.join(_HERE, "isaacsim_trajectory.schema.json")
WAYPOINTS_SCHEMA_PATH = os.path.join(_HERE, "waypoints_config.schema.json")

KIND_MOTOR = "motor"
KIND_TRAJECTORY = "trajectory"
KIND_WAYPOINTS = "waypoints"

# A single trajectory/limit value is a scalar or an [x, y, z] triple.
LimitT = Union[float, List[float]]


class SchemaValidationError(ValueError):
    """Raised when an object does not conform to its interface schema.

    Both the jsonschema-backed path and the pure-Python path raise this single
    type, so callers see identical reject behaviour regardless of environment.
    """


# =============================================================================
# Dataclasses (typed, JSON-safe)
# =============================================================================
@dataclass
class MotorFrame:
    """One frame of the motor-command stream (the control<->sim boundary)."""

    time: float               # FLOAT SECONDS on 성진's control clock, >= 0
    motor_cmd_w: List[float]  # [w1, w2, w3, w4] rotor angular velocity, rad/s

    def to_dict(self) -> dict:
        return {
            "time": float(self.time),
            "motor_cmd_w": [float(w) for w in self.motor_cmd_w],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "MotorFrame":
        return cls(
            time=float(d["time"]),
            motor_cmd_w=[float(w) for w in d["motor_cmd_w"]],
        )


@dataclass
class MotorCommandsFile:
    """A full motor-command file: {fps, frames:[MotorFrame, ...]}."""

    KIND: ClassVar[str] = KIND_MOTOR

    fps: float
    frames: List[MotorFrame] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {"fps": float(self.fps), "frames": [f.to_dict() for f in self.frames]}

    @classmethod
    def from_dict(cls, d: dict) -> "MotorCommandsFile":
        return cls(
            fps=float(d["fps"]),
            frames=[MotorFrame.from_dict(f) for f in d["frames"]],
        )

    def validate(self) -> None:
        """Validate this object against isaacsim_motor_commands.schema.json."""
        validate(self.to_dict(), kind=KIND_MOTOR)


@dataclass
class TrajectoryFrame:
    """One frame of the trajectory stream (성진's control output)."""

    time: float                          # FLOAT SECONDS, >= 0 (NOT ns)
    position: List[float]                # [x, y, z] world, metres (+Z up)
    yaw_rad: float                       # heading about world +Z, radians
    orientation_quat_wxyz: List[float]   # [w, x, y, z] unit quaternion, world<-body, yaw-only

    def to_dict(self) -> dict:
        return {
            "time": float(self.time),
            "position": [float(v) for v in self.position],
            "yaw_rad": float(self.yaw_rad),
            "orientation_quat_wxyz": [float(v) for v in self.orientation_quat_wxyz],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "TrajectoryFrame":
        return cls(
            time=float(d["time"]),
            position=[float(v) for v in d["position"]],
            yaw_rad=float(d["yaw_rad"]),
            orientation_quat_wxyz=[float(v) for v in d["orientation_quat_wxyz"]],
        )


@dataclass
class TrajectoryFile:
    """A full trajectory file: {fps, frames:[TrajectoryFrame, ...]}."""

    KIND: ClassVar[str] = KIND_TRAJECTORY

    fps: float
    frames: List[TrajectoryFrame] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {"fps": float(self.fps), "frames": [f.to_dict() for f in self.frames]}

    @classmethod
    def from_dict(cls, d: dict) -> "TrajectoryFile":
        return cls(
            fps=float(d["fps"]),
            frames=[TrajectoryFrame.from_dict(f) for f in d["frames"]],
        )

    def validate(self) -> None:
        """Validate this object against isaacsim_trajectory.schema.json."""
        validate(self.to_dict(), kind=KIND_TRAJECTORY)


@dataclass
class WaypointsConfig:
    """성진's controller INPUT: {waypoints, limits, dt} (INPUT_FORMAT.md).

    This is the RL->control seam. ``waypoints`` is an ordered list of [x, y, z]
    points (metres, world +Z-up, N >= 2, first = start). ``limits`` maps each of
    v_max/a_max/j_max/snap_max to a scalar OR an [x, y, z] triple; they are
    enforced PER-AXIS, not on the vector magnitude. ``dt`` is the output sample
    interval in seconds (default 0.01). No yaw/time/quat in the input.
    """

    KIND: ClassVar[str] = KIND_WAYPOINTS

    waypoints: List[List[float]]
    limits: Dict[str, LimitT]
    dt: float = 0.01

    def to_dict(self) -> dict:
        return {
            "waypoints": [[float(c) for c in wp] for wp in self.waypoints],
            "limits": {
                k: ([float(x) for x in v] if isinstance(v, (list, tuple)) else float(v))
                for k, v in self.limits.items()
            },
            "dt": float(self.dt),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "WaypointsConfig":
        return cls(
            waypoints=[[float(c) for c in wp] for wp in d["waypoints"]],
            limits={
                k: ([float(x) for x in v] if isinstance(v, (list, tuple)) else float(v))
                for k, v in d["limits"].items()
            },
            dt=float(d.get("dt", 0.01)),
        )

    def validate(self) -> None:
        """Validate this object against waypoints_config.schema.json."""
        validate(self.to_dict(), kind=KIND_WAYPOINTS)

    def limits_per_axis(self) -> Dict[str, List[float]]:
        """Return {name: [x, y, z]} -- scalars broadcast to all three axes.

        Documents 성진's PER-AXIS (not vector-magnitude) enforcement
        (INPUT_FORMAT.md); handy when the RL side reasons about x/y/z limits.
        """
        out: Dict[str, List[float]] = {}
        for name, v in self.limits.items():
            if isinstance(v, (list, tuple)):
                out[name] = [float(x) for x in v]
            else:
                out[name] = [float(v), float(v), float(v)]
        return out


# =============================================================================
# JSON I/O
# =============================================================================
def load_json(path: str) -> dict:
    """Load a JSON file into a plain dict (no validation)."""
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def save_json(obj: Any, path: str) -> None:
    """Write a dict OR a dataclass file (anything with .to_dict()) to JSON.

    json.dump preserves numeric type: integers stay integers, floats stay
    floats. Control-side timestamps are float seconds (e.g. 0.01) -- they are
    written as floats, never coerced to ns integers. A trailing newline is added
    for tidy diffs.
    """
    if hasattr(obj, "to_dict"):
        obj = obj.to_dict()
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=2, sort_keys=False)
        fh.write("\n")


# =============================================================================
# Validation
# =============================================================================
_SCHEMA_PATHS = {
    KIND_MOTOR: MOTOR_SCHEMA_PATH,
    KIND_TRAJECTORY: TRAJECTORY_SCHEMA_PATH,
    KIND_WAYPOINTS: WAYPOINTS_SCHEMA_PATH,
}
_schema_cache: dict = {}


def load_schema(kind: str) -> dict:
    """Load (and cache) the JSON Schema dict for a given kind."""
    if kind not in _SCHEMA_PATHS:
        raise ValueError(
            f"unknown schema kind {kind!r}; expected 'motor', 'trajectory' or 'waypoints'"
        )
    if kind not in _schema_cache:
        with open(_SCHEMA_PATHS[kind], "r", encoding="utf-8") as fh:
            _schema_cache[kind] = json.load(fh)
    return _schema_cache[kind]


def infer_kind(obj: Any) -> str:
    """Best-effort detect the kind of an object.

    'waypoints' when it carries a 'waypoints' key; otherwise 'motor' vs
    'trajectory' from the first frame. Raises ValueError when the object carries
    nothing to disambiguate on (e.g. an empty 'frames' list); pass ``kind=``
    explicitly in that case.
    """
    if isinstance(obj, dict):
        if "waypoints" in obj:
            return KIND_WAYPOINTS
        frames = obj.get("frames")
        if isinstance(frames, list) and frames and isinstance(frames[0], dict):
            first = frames[0]
            if "motor_cmd_w" in first:
                return KIND_MOTOR
            if "position" in first or "orientation_quat_wxyz" in first:
                return KIND_TRAJECTORY
    raise ValueError(
        "cannot infer schema kind from object; "
        "pass kind='motor', 'trajectory' or 'waypoints'"
    )


# ---- pure-Python structural validators (jsonschema-free fallback) -----------
def _is_number(v: Any) -> bool:
    # jsonschema treats booleans as NOT numbers; mirror that (bool is an int).
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _check_object(obj: Any, required: tuple, allowed: set, path: str) -> None:
    if not isinstance(obj, dict):
        raise SchemaValidationError(f"{path}: expected object, got {type(obj).__name__}")
    for key in required:
        if key not in obj:
            raise SchemaValidationError(f"{path}: missing required key '{key}'")
    extra = set(obj) - allowed
    if extra:
        raise SchemaValidationError(f"{path}: unexpected key(s) {sorted(extra)} (additionalProperties=false)")


def _check_number_array(val: Any, n: int, path: str) -> None:
    if not isinstance(val, list):
        raise SchemaValidationError(f"{path}: expected array of {n} numbers, got {type(val).__name__}")
    if len(val) != n:
        raise SchemaValidationError(f"{path}: expected exactly {n} items, got {len(val)}")
    for i, x in enumerate(val):
        if not _is_number(x):
            raise SchemaValidationError(f"{path}[{i}]: expected number, got {type(x).__name__}")


def _check_time_seconds(t: Any, path: str) -> None:
    # Control JSON side: FLOAT SECONDS, >= 0 (NOT integer nanoseconds).
    if not _is_number(t):
        raise SchemaValidationError(f"{path}.time: expected number (float seconds), got {t!r}")
    if t < 0:
        raise SchemaValidationError(f"{path}.time: must be >= 0, got {t}")


def _check_fps_and_frames(obj: Any) -> list:
    _check_object(obj, required=("fps", "frames"), allowed={"fps", "frames"}, path="<root>")
    if not _is_number(obj["fps"]) or obj["fps"] <= 0:
        raise SchemaValidationError(f"<root>.fps: expected number > 0, got {obj['fps']!r}")
    if not isinstance(obj["frames"], list):
        raise SchemaValidationError(f"<root>.frames: expected array, got {type(obj['frames']).__name__}")
    return obj["frames"]


def _validate_pure_motor(obj: Any) -> None:
    frames = _check_fps_and_frames(obj)
    for i, fr in enumerate(frames):
        p = f"frames[{i}]"
        _check_object(fr, required=("time", "motor_cmd_w"),
                      allowed={"time", "motor_cmd_w"}, path=p)
        _check_time_seconds(fr["time"], p)
        _check_number_array(fr["motor_cmd_w"], 4, f"{p}.motor_cmd_w")


def _validate_pure_trajectory(obj: Any) -> None:
    frames = _check_fps_and_frames(obj)
    req = ("time", "position", "yaw_rad", "orientation_quat_wxyz")
    for i, fr in enumerate(frames):
        p = f"frames[{i}]"
        _check_object(fr, required=req, allowed=set(req), path=p)
        _check_time_seconds(fr["time"], p)
        _check_number_array(fr["position"], 3, f"{p}.position")
        if not _is_number(fr["yaw_rad"]):
            raise SchemaValidationError(f"{p}.yaw_rad: expected number, got {fr['yaw_rad']!r}")
        _check_number_array(fr["orientation_quat_wxyz"], 4, f"{p}.orientation_quat_wxyz")


def _check_limit(val: Any, name: str) -> None:
    """A trajectory limit: a positive scalar OR a 3-element array of positives.

    Mirrors waypoints_config.schema.json's oneOf[number>0, [3 numbers >0]].
    """
    if _is_number(val):
        if val <= 0:
            raise SchemaValidationError(f"limits.{name}: scalar must be > 0, got {val!r}")
        return
    if isinstance(val, list):
        if len(val) != 3:
            raise SchemaValidationError(
                f"limits.{name}: per-axis limit must have exactly 3 items, got {len(val)}")
        for i, x in enumerate(val):
            if not _is_number(x):
                raise SchemaValidationError(f"limits.{name}[{i}]: expected number, got {type(x).__name__}")
            if x <= 0:
                raise SchemaValidationError(f"limits.{name}[{i}]: must be > 0, got {x!r}")
        return
    raise SchemaValidationError(
        f"limits.{name}: expected number or [x,y,z] array, got {type(val).__name__}")


def _validate_pure_waypoints(obj: Any) -> None:
    _check_object(obj, required=("waypoints", "limits"),
                  allowed={"waypoints", "limits", "dt"}, path="<root>")
    wps = obj["waypoints"]
    if not isinstance(wps, list):
        raise SchemaValidationError(f"<root>.waypoints: expected array, got {type(wps).__name__}")
    if len(wps) < 2:
        raise SchemaValidationError(f"<root>.waypoints: need >= 2 points, got {len(wps)}")
    for i, wp in enumerate(wps):
        _check_number_array(wp, 3, f"waypoints[{i}]")
    limits = obj["limits"]
    _check_object(limits, required=("v_max", "a_max", "j_max", "snap_max"),
                  allowed={"v_max", "a_max", "j_max", "snap_max"}, path="limits")
    for name in ("v_max", "a_max", "j_max", "snap_max"):
        _check_limit(limits[name], name)
    if "dt" in obj:
        if not _is_number(obj["dt"]) or obj["dt"] <= 0:
            raise SchemaValidationError(f"<root>.dt: expected number > 0, got {obj['dt']!r}")


_PURE_VALIDATORS = {
    KIND_MOTOR: _validate_pure_motor,
    KIND_TRAJECTORY: _validate_pure_trajectory,
    KIND_WAYPOINTS: _validate_pure_waypoints,
}


def validate(obj: Any, kind: Optional[str] = None) -> None:
    """Validate ``obj`` against its interface schema.

    Uses ``jsonschema`` against the sibling *.schema.json when available, else an
    equivalent pure-Python structural validator. Both raise SchemaValidationError
    on the same rejects. ``kind`` is 'motor'/'trajectory'/'waypoints'; inferred
    if omitted. Returns None on success.
    """
    kind = kind or infer_kind(obj)
    if kind not in _SCHEMA_PATHS:
        raise ValueError(
            f"unknown schema kind {kind!r}; expected 'motor', 'trajectory' or 'waypoints'"
        )
    if _HAS_JSONSCHEMA:
        try:
            jsonschema.validate(obj, load_schema(kind))
        except jsonschema.ValidationError as exc:  # type: ignore[attr-defined]
            raise SchemaValidationError(exc.message) from exc
    else:
        _PURE_VALIDATORS[kind](obj)


def is_valid(obj: Any, kind: Optional[str] = None) -> bool:
    """Boolean convenience wrapper around validate()."""
    try:
        validate(obj, kind=kind)
        return True
    except SchemaValidationError:
        return False


# =============================================================================
# Converter: trajectory frame -> 4x4 transform (uses common.geometry)
# =============================================================================
def trajectory_frame_to_T(frame: Any) -> np.ndarray:
    """Build the 4x4 world<-body homogeneous transform for one trajectory frame.

    ``frame`` is a TrajectoryFrame or a dict with 'position' (x,y,z) and
    'orientation_quat_wxyz' (w,x,y,z). Rotation comes from
    common.geometry.quat_wxyz_to_R and is composed with the translation by
    common.geometry.make_transform -- the geometry is never re-derived here.
    """
    if isinstance(frame, TrajectoryFrame):
        quat = frame.orientation_quat_wxyz
        pos = frame.position
    elif isinstance(frame, dict):
        quat = frame["orientation_quat_wxyz"]
        pos = frame["position"]
    else:
        raise TypeError(f"expected TrajectoryFrame or dict, got {type(frame).__name__}")
    R = quat_wxyz_to_R(quat)          # wxyz -> 3x3, shared source of truth
    return make_transform(R, pos)     # compose into a 4x4


# =============================================================================
# Smoke test (self-contained; runnable now with numpy-only + pyyaml env)
# =============================================================================
def _accepts_pure(obj: Any, kind: str) -> bool:
    try:
        _PURE_VALIDATORS[kind](obj)
        return True
    except SchemaValidationError:
        return False


def _accepts_jsonschema(obj: Any, kind: str) -> Optional[bool]:
    if not _HAS_JSONSCHEMA:
        return None
    try:
        jsonschema.validate(obj, load_schema(kind))
        return True
    except jsonschema.ValidationError:  # type: ignore[attr-defined]
        return False


def _expect(obj: Any, kind: str, want: bool, tag: str) -> None:
    """Assert the pure validator (and jsonschema, if present) agree with `want`."""
    got_pure = _accepts_pure(obj, kind)
    assert got_pure == want, f"[pure] {tag}: expected {want}, got {got_pure}"
    got_js = _accepts_jsonschema(obj, kind)
    if got_js is not None:
        assert got_js == want, f"[jsonschema] {tag}: expected {want}, got {got_js}"
        assert got_js == got_pure, f"{tag}: jsonschema/pure disagree ({got_js} vs {got_pure})"


def _smoke_test() -> int:
    import copy
    import tempfile

    backend = "jsonschema+pure" if _HAS_JSONSCHEMA else "pure-python only"
    print(f"[interface.schemas] smoke test  (validation backend: {backend})")

    # --- build a small VALID motor-commands dict (FLOAT SECONDS time) --------
    motor = MotorCommandsFile(
        fps=200.0,
        frames=[
            MotorFrame(time=0.0,   motor_cmd_w=[4.0, 4.0, 4.0, 4.0]),   # 성진: 4 rad/s
            MotorFrame(time=0.005, motor_cmd_w=[4.1, 3.9, 4.05, 3.95]),  # +5 ms as seconds
        ],
    ).to_dict()
    _expect(motor, KIND_MOTOR, True, "valid motor-commands")
    assert is_valid(motor) is True                       # kind inferred
    assert infer_kind(motor) == KIND_MOTOR
    print("  ok: valid motor-commands accepted (float-seconds time; both paths agree)")

    # --- build a small VALID trajectory dict (FLOAT SECONDS time) -----------
    traj = TrajectoryFile(
        fps=100.0,
        frames=[
            TrajectoryFrame(time=0.0,  position=[0.0, 0.0, 1.5],
                            yaw_rad=0.0, orientation_quat_wxyz=[1.0, 0.0, 0.0, 0.0]),
            TrajectoryFrame(time=0.01, position=[0.10, 0.0, 1.5],
                            yaw_rad=0.05, orientation_quat_wxyz=[0.99969, 0.0, 0.0, 0.025]),
        ],
    ).to_dict()
    _expect(traj, KIND_TRAJECTORY, True, "valid trajectory")
    assert is_valid(traj) is True
    assert infer_kind(traj) == KIND_TRAJECTORY
    print("  ok: valid trajectory accepted (float-seconds time; both paths agree)")

    # --- build a VALID waypoints config (RL->control seam) ------------------
    wpc = WaypointsConfig(
        waypoints=[[-2.0, -2.0, 0.15], [-2.0, -2.0, 6.0], [5.0, 0.0, 0.15]],
        limits={"v_max": 1.0, "a_max": [0.8, 0.8, 0.5], "j_max": 2.0, "snap_max": 10.0},
        dt=0.01,
    ).to_dict()
    _expect(wpc, KIND_WAYPOINTS, True, "valid waypoints-config (scalar + per-axis limits)")
    assert is_valid(wpc) is True                          # kind inferred from 'waypoints'
    assert infer_kind(wpc) == KIND_WAYPOINTS
    # dt is optional (schema default 0.01)
    wpc_no_dt = {"waypoints": wpc["waypoints"], "limits": wpc["limits"]}
    _expect(wpc_no_dt, KIND_WAYPOINTS, True, "valid waypoints-config (no dt)")
    print("  ok: valid waypoints-config accepted (scalar/per-axis limits, optional dt)")

    # --- INVALID: wrong array length ----------------------------------------
    bad_len = copy.deepcopy(motor)
    bad_len["frames"][0]["motor_cmd_w"] = [4.0, 4.0, 4.0]         # only 3 rotors
    _expect(bad_len, KIND_MOTOR, False, "motor: 3-element motor_cmd_w")

    bad_pos = copy.deepcopy(traj)
    bad_pos["frames"][0]["position"] = [0.0, 0.0, 1.5, 9.9]       # 4-element position
    _expect(bad_pos, KIND_TRAJECTORY, False, "trajectory: 4-element position")
    print("  ok: wrong array length rejected (both paths agree)")

    # --- INVALID: negative time / non-number time ---------------------------
    bad_time_m = copy.deepcopy(motor)
    bad_time_m["frames"][1]["time"] = -1.0
    _expect(bad_time_m, KIND_MOTOR, False, "motor: negative time")

    bad_time_t = copy.deepcopy(traj)
    bad_time_t["frames"][1]["time"] = "0.01"                      # string, not number
    _expect(bad_time_t, KIND_TRAJECTORY, False, "trajectory: string time")
    print("  ok: negative / non-number time rejected (both paths agree)")

    # --- INVALID: extra key, fps<=0 -----------------------------------------
    bad_extra = copy.deepcopy(traj)
    bad_extra["frames"][0]["extra"] = 1                           # additionalProperties
    _expect(bad_extra, KIND_TRAJECTORY, False, "trajectory: extra key")

    bad_fps = copy.deepcopy(motor)
    bad_fps["fps"] = 0                                            # must be > 0
    _expect(bad_fps, KIND_MOTOR, False, "motor: fps == 0")
    print("  ok: extra key / fps<=0 rejected (both paths agree)")

    # --- INVALID waypoints-config cases -------------------------------------
    bad_one_wp = copy.deepcopy(wpc)
    bad_one_wp["waypoints"] = [[0.0, 0.0, 0.0]]                   # N < 2
    _expect(bad_one_wp, KIND_WAYPOINTS, False, "waypoints: only 1 point")

    bad_wp_len = copy.deepcopy(wpc)
    bad_wp_len["waypoints"][0] = [0.0, 0.0]                       # 2-element point
    _expect(bad_wp_len, KIND_WAYPOINTS, False, "waypoints: 2-element point")

    bad_missing_limit = copy.deepcopy(wpc)
    del bad_missing_limit["limits"]["snap_max"]                   # missing required limit
    _expect(bad_missing_limit, KIND_WAYPOINTS, False, "waypoints: missing snap_max")

    bad_axis_limit = copy.deepcopy(wpc)
    bad_axis_limit["limits"]["a_max"] = [0.8, 0.8]                # per-axis needs 3
    _expect(bad_axis_limit, KIND_WAYPOINTS, False, "waypoints: 2-element per-axis limit")

    bad_neg_limit = copy.deepcopy(wpc)
    bad_neg_limit["limits"]["v_max"] = 0.0                        # must be > 0
    _expect(bad_neg_limit, KIND_WAYPOINTS, False, "waypoints: v_max == 0")

    bad_dt = copy.deepcopy(wpc)
    bad_dt["dt"] = -0.01                                          # must be > 0
    _expect(bad_dt, KIND_WAYPOINTS, False, "waypoints: dt <= 0")

    bad_wp_extra = copy.deepcopy(wpc)
    bad_wp_extra["waypoint"] = wpc["waypoints"]                   # typo'd extra key
    _expect(bad_wp_extra, KIND_WAYPOINTS, False, "waypoints: unexpected extra key")
    print("  ok: bad waypoints-configs rejected (N<2, bad point/limit, dt<=0, extra key)")

    # --- round-trip save_json / load_json -----------------------------------
    with tempfile.TemporaryDirectory() as td:
        mp = os.path.join(td, "motor.json")
        tp = os.path.join(td, "traj.json")
        cp = os.path.join(td, "waypoints.json")
        save_json(motor, mp)                             # dict input
        save_json(TrajectoryFile.from_dict(traj), tp)    # dataclass input
        save_json(WaypointsConfig.from_dict(wpc), cp)    # dataclass input
        motor_rt = load_json(mp)
        traj_rt = load_json(tp)
        wpc_rt = load_json(cp)
        assert motor_rt == motor, "motor round-trip changed the data"
        assert traj_rt == traj, "trajectory round-trip changed the data"
        assert wpc_rt == wpc, "waypoints round-trip changed the data"
        # control-side timestamps must survive as floats (float seconds).
        assert isinstance(motor_rt["frames"][1]["time"], float)
        assert is_valid(motor_rt, KIND_MOTOR) and is_valid(traj_rt, KIND_TRAJECTORY)
        assert is_valid(wpc_rt, KIND_WAYPOINTS)
    print("  ok: save_json/load_json round-trips (time stays float, still valid)")

    # --- WaypointsConfig.limits_per_axis broadcasts scalars -----------------
    per_axis = WaypointsConfig.from_dict(wpc).limits_per_axis()
    assert per_axis["v_max"] == [1.0, 1.0, 1.0], per_axis["v_max"]     # scalar broadcast
    assert per_axis["a_max"] == [0.8, 0.8, 0.5], per_axis["a_max"]     # per-axis kept
    print("  ok: WaypointsConfig.limits_per_axis broadcasts scalar->[x,y,z]")

    # --- converter: identity quaternion -> identity R, translation == position
    frame = TrajectoryFrame(time=0.0, position=[1.0, 2.0, 3.0],
                            yaw_rad=0.0, orientation_quat_wxyz=[1.0, 0.0, 0.0, 0.0])
    T = trajectory_frame_to_T(frame)
    assert T.shape == (4, 4)
    assert np.allclose(T[:3, :3], np.eye(3)), "identity quat should give identity rotation"
    assert np.allclose(T[:3, 3], [1.0, 2.0, 3.0]), "translation should equal position"
    assert np.allclose(T[3, :], [0, 0, 0, 1]), "bottom row must be homogeneous"
    # same result from a plain dict input
    T2 = trajectory_frame_to_T(frame.to_dict())
    assert np.allclose(T, T2)
    print("  ok: trajectory_frame_to_T([1,0,0,0]) -> identity R, t == position")

    print("[interface.schemas] ALL SMOKE CHECKS PASSED")
    return 0


# =============================================================================
# CLI
# =============================================================================
def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Validate / self-test the control<->sim interface schemas.")
    sub = p.add_subparsers(dest="cmd", required=True)

    vp = sub.add_parser("validate", help="validate a JSON file against a schema")
    vp.add_argument("path", help="path to a motor / trajectory / waypoints JSON file")
    vp.add_argument("--kind", choices=[KIND_MOTOR, KIND_TRAJECTORY, KIND_WAYPOINTS],
                    default=None,
                    help="schema kind (inferred from the object if omitted)")

    sub.add_parser("selftest", help="run the built-in smoke test")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.cmd == "selftest":
        return _smoke_test()
    if args.cmd == "validate":
        obj = load_json(args.path)
        kind = args.kind
        try:
            validate(obj, kind=kind)
        except SchemaValidationError as exc:
            print(f"INVALID: {exc}")
            return 1
        backend = "jsonschema" if _HAS_JSONSCHEMA else "pure-python"
        print(f"VALID ({kind or infer_kind(obj)}) [{backend}]")
        return 0
    return 2  # pragma: no cover


if __name__ == "__main__":
    raise SystemExit(main())
