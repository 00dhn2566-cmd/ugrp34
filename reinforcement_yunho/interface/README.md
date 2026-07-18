# `interface/` — control ⇄ Isaac-Sim contract (+ RL → control seam)

Consumer / counterparty: **성진 (control)**. Checklist section 1.

This directory pins the JSON/YAML handoffs between the RL policy, the controller,
and the simulator. There are **three** contracts:

1. **RL → control input** — `waypoints_config.schema.json`. The real entry point
   of 성진's controller (`control_seoungjin/sample/run_and_log.py`,
   `INPUT_FORMAT.md`). **This is where RL plugs in**: the policy emits waypoints,
   they become this config, 성진 plans a minimum-time trajectory through them.
2. **control → sim boundary** — `isaacsim_motor_commands.schema.json`. The real
   boundary: per-motor angular-velocity setpoints (rad/s). Motor/propeller
   physics is **Isaac Sim's job** (CONVENTIONS.md).
3. **control output pose stream** — `isaacsim_trajectory.schema.json`. 성진's
   current `isaacsim_trajectory.json` output; convenient for a PID follower.

## Files

| file | what | status |
|------|------|--------|
| `waypoints_config.schema.json` | JSON Schema (draft-07) for 성진's controller **input** `{waypoints:[[x,y,z]…] N≥2, limits:{v_max,a_max,j_max,snap_max} scalar-or-[x,y,z], dt}`. The RL→control seam. | **spec (runnable-now)** |
| `isaacsim_motor_commands.schema.json` | JSON Schema for the motor-command stream `{fps, frames:[{time(float s), motor_cmd_w[4]}]}`. | **provisional** — 성진 waits on 윤호 to finalise the Isaac Sim JSON schema |
| `isaacsim_trajectory.schema.json` | JSON Schema for 성진's pose output `{fps, frames:[{time(float s), position[3], yaw_rad, orientation_quat_wxyz[4]}]}`. | **provisional** — 성진 waits on 윤호 to finalise the Isaac Sim JSON schema |
| `schemas.py` | dataclasses (`WaypointsConfig`, `MotorCommandsFile`/`MotorFrame`, `TrajectoryFile`/`TrajectoryFrame`), `load_json`/`save_json`, `validate()`, `is_valid()`, `trajectory_frame_to_T()`. Pure `numpy`+stdlib. | **runnable-now** |
| `README.md` | this file. | — |

Nothing here needs Isaac Sim, a GPU, or any un-installed dependency. `jsonschema`
is **optional**: `validate()` uses it when importable and otherwise falls back to
an equivalent pure-Python structural validator — **both give the same
accept/reject**.

## Conventions (from CONVENTIONS.md — obey exactly)

- **Quaternions are WXYZ** everywhere: `orientation_quat_wxyz = [w, x, y, z]`,
  world←body, yaw-only. This is the order `common.geometry.quat_wxyz_to_R` expects.
- **Time on the 성진 control JSON side is FLOAT SECONDS** (`time`, `>= 0`,
  e.g. `0.01`) — 성진's real output. This is **NOT** the integer-nanosecond clock:
  that clock belongs to the **separate vision/VIO stream** (the §5 vision message,
  the flight-data bag, EuRoC-ASL GT). Do not conflate the two.
- `fps` is nominal. For the **motor** stream the command rate is **variable**, so
  the per-frame `time` is authoritative (not `fps`).
- World frame is right-handed, **+Z up**, metres. `position = [x, y, z]`.
- `motor_cmd_w` is exactly **four** rotor setpoints in **rad/s** (성진 emits
  exactly `4` rad/s per rotor in the reference output).
- **`limits` are enforced PER-AXIS**, not on the vector magnitude
  (`INPUT_FORMAT.md`): a diagonal move can exceed the scalar limit in magnitude.

## Worked example — waypoints config (RL → control)

```json
{
  "waypoints": [
    [-2.0, -2.0, 0.15],
    [-2.0, -2.0, 6.0],
    [5.0, 0.0, 0.15]
  ],
  "limits": { "v_max": 1.0, "a_max": 0.8, "j_max": 2.0, "snap_max": 10.0 },
  "dt": 0.01
}
```

`limits` values may also be `[x, y, z]` triples for per-axis limits. `dt` is
optional (default `0.01`). No `yaw`/`time`/`quat` in the input — yaw is derived
downstream from the velocity heading.

## Worked example — motor commands

```json
{
  "fps": 200.0,
  "frames": [
    { "time": 0.0,   "motor_cmd_w": [4.0, 4.0, 4.0, 4.0] },
    { "time": 0.005, "motor_cmd_w": [4.1, 3.9, 4.05, 3.95] }
  ]
}
```

`time` is **float seconds** (`0.005 s = 5 ms`); the motor rate is variable so the
per-frame `time` is authoritative.

## Worked example — trajectory

```json
{
  "fps": 100.0,
  "frames": [
    { "time": 0.00, "position": [0.0, 0.0, 1.5], "yaw_rad": 0.00,
      "orientation_quat_wxyz": [1.0, 0.0, 0.0, 0.0] },
    { "time": 0.01, "position": [0.1, 0.0, 1.5], "yaw_rad": 0.05,
      "orientation_quat_wxyz": [0.99969, 0.0, 0.0, 0.025] }
  ]
}
```

`orientation_quat_wxyz = [1,0,0,0]` is the identity rotation;
`trajectory_frame_to_T(frame)` then returns a 4×4 whose rotation is `I` and whose
translation equals `position`.

## Rotor index → geometry + spin (REQUIRED 윤호 decision)

`motor_cmd_w = [w1, w2, w3, w4]` is **intentionally agnostic** to which physical
rotor each entry drives and which way it spins — the schema only fixes "four
rad/s setpoints". **The mapping is a 윤호 decision** that lives in the **Isaac Sim
rotor config** (which arm each index is, and the CW/CCW spin sign per rotor), and
it must be **published so 성진's emission order matches Isaac Sim's application
order**. Until 윤호 pins it, motor commands are ambiguous. The decision must state,
for each index `0..3`:

- **which arm/position** it drives (e.g. a fixed convention such as
  `0 = front-right, 1 = back-left, 2 = front-left, 3 = back-right`), and
- **the spin direction** (CW / CCW), which sets the yaw-torque sign — diagonal
  rotors spin the same way, adjacent rotors opposite, for a standard quad-X.

Record it once (a small config/table that both the Isaac rotor setup and 성진's
controller read) so index→geometry and spin can never drift between the two sides.
This is the same class of "single source of truth" decision as the calib IMU
noise numbers.

## Using it

```python
from interface.schemas import (
    WaypointsConfig,
    MotorCommandsFile, MotorFrame,
    TrajectoryFile, TrajectoryFrame,
    validate, is_valid, load_json, save_json, trajectory_frame_to_T,
)

cfg = WaypointsConfig(
    waypoints=[[-2.0, -2.0, 0.15], [-2.0, -2.0, 6.0]],
    limits={"v_max": 1.0, "a_max": 0.8, "j_max": 2.0, "snap_max": 10.0},
)
cfg.validate()                       # raises SchemaValidationError if malformed
cfg.limits_per_axis()                # {"v_max":[1,1,1], ...} scalars broadcast

mc = MotorCommandsFile(fps=200.0, frames=[MotorFrame(0.0, [4, 4, 4, 4])])
mc.validate()
save_json(mc, "cmds.json")           # dataclass or dict both accepted

obj = load_json("traj.json")
validate(obj, kind="trajectory")     # or is_valid(obj) with kind inferred
T = trajectory_frame_to_T(obj["frames"][0])   # 4x4 world<-body
```

CLI:

```bash
python3 interface/schemas.py selftest                       # run the smoke test
python3 interface/schemas.py validate input.json            # kind inferred
python3 interface/schemas.py validate cmds.json --kind motor
python3 interface/schemas.py validate input.json --kind waypoints
```

## Handoff

- **RL → 성진**: the policy's waypoints become a `waypoints_config` (validate it,
  then hand to `run_and_log.py --config`). 성진 does **no** validation itself
  (`KeyError` on a missing key), so validate here first.
- **성진 → sim**: 성진 produces the motor-command / trajectory files (filling in
  `fps`, float-seconds `time`, `motor_cmd_w` rad/s, and/or pose). The Isaac-Sim
  side `validate()`s on ingest and applies `motor_cmd_w` as rotor setpoints
  (physics is the simulator's job), using `trajectory_frame_to_T()` for poses.
- **Open items for 윤호**: (1) finalise the Isaac Sim JSON schema (both isaacsim
  schemas are **provisional** until then); (2) publish the rotor index→geometry +
  CW/CCW spin mapping above.
