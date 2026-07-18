"""EuRoC-ASL flight-data bag writer for 태민 (VIO / OpenVINS).

WHAT
----
Writes an Isaac-Sim flight into the EuRoC-ASL (a.k.a. ASL / MAV) directory layout
that OpenVINS + most VIO evaluators read directly:

    <out>/mav0/
      cam0/data/<timestamp_ns>.png          # frames from the Isaac render
      cam0/data.csv                          # #timestamp [ns],filename
      cam0/sensor.yaml                       # pinhole model (from CameraIntrinsics)
      imu0/data.csv                          # #timestamp,w_x,w_y,w_z,a_x,a_y,a_z
      imu0/sensor.yaml                       # rate + noise placeholders
      state_groundtruth_estimate0/data.csv   # #timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z

CONTRACT (CONVENTIONS.md + the checklist "테스트용 시뮬 비행 데이터 1세트")
--------------------------------------------------------------------------
* Timestamps: INTEGER NANOSECONDS on ONE shared cam+IMU clock (never float).
* Camera:  ~20 Hz, 1280x720 (spec §2). Images come from the Isaac render; this
           writer only lays them out + indexes them in cam0/data.csv. The PNG
           filename IS the timestamp (EuRoC convention).
* IMU:     ~200 Hz. accel in m/s^2, gyro in rad/s (matches kalibr_imu_chain.yaml
           update_rate + unit comments in calib/).
* GT pose: EuRoC state_groundtruth_estimate0. position p [m]; quaternion order is
           **q_w,q_x,q_y,q_z (WXYZ)** — EuRoC GT convention (CONVENTIONS.md
           quaternion table). Frame = T_world_body; body ≡ camera here (same
           declaration as the §5 stream / scene_gt.json), so pass the camera pose.
           Optional velocity + gyro/accel bias columns fill out the canonical
           17-column EuRoC GT row when supplied (default: pose-only 8 columns).

The CSV writers/readers below are pure (stdlib + numpy) and unit-tested with
synthetic data; nothing here needs Isaac Sim.
"""
from __future__ import annotations

import csv
import os
import shutil
import sys
from typing import Dict, List, Optional, Sequence

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# EuRoC-ASL canonical CSV headers (leading '#', SI units in brackets).
CAM_HEADER = ["#timestamp [ns]", "filename"]
IMU_HEADER = [
    "#timestamp [ns]",
    "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]",
    "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]", "a_RS_S_z [m s^-2]",
]
GT_HEADER_POSE = [
    "#timestamp",
    "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]",
    "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []",
]
GT_HEADER_FULL = GT_HEADER_POSE + [
    "v_RS_R_x [m s^-1]", "v_RS_R_y [m s^-1]", "v_RS_R_z [m s^-1]",
    "b_w_RS_S_x [rad s^-1]", "b_w_RS_S_y [rad s^-1]", "b_w_RS_S_z [rad s^-1]",
    "b_a_RS_S_x [m s^-2]", "b_a_RS_S_y [m s^-2]", "b_a_RS_S_z [m s^-2]",
]


def _require_int_ns(ts) -> int:
    """Timestamps are integer nanoseconds (CONVENTIONS.md). Reject floats loudly —
    a float ns loses precision, exactly like vision_msg.build_frame_message does."""
    if isinstance(ts, bool) or not isinstance(ts, (int, np.integer)):
        raise ValueError(f"timestamp must be int nanoseconds, got {type(ts).__name__}: {ts!r}")
    return int(ts)


def _write_csv(path: str, header: Sequence[str], rows: Sequence[Sequence]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


# ======================================================================
#  WRITERS  (pure)
# ======================================================================
def write_cam0_csv(mav0_dir: str, cam_frames: Sequence[Dict[str, object]]) -> str:
    """cam0/data.csv (#timestamp,filename). ``cam_frames``: [{timestamp_ns, filename}].
    filename is just '<ts>.png' (EuRoC), pointing at cam0/data/<ts>.png."""
    rows = []
    for fr in cam_frames:
        ts = _require_int_ns(fr["timestamp_ns"])
        rows.append([ts, fr.get("filename", f"{ts}.png")])
    path = os.path.join(mav0_dir, "cam0", "data.csv")
    _write_csv(path, CAM_HEADER, rows)
    return path


def write_imu0_csv(mav0_dir: str, imu_samples: Sequence[Dict[str, object]]) -> str:
    """imu0/data.csv. ``imu_samples``: [{timestamp_ns, gyro[wx,wy,wz] rad/s,
    accel[ax,ay,az] m/s^2}] at ~200 Hz on the shared clock."""
    rows = []
    for s in imu_samples:
        ts = _require_int_ns(s["timestamp_ns"])
        gx, gy, gz = (float(x) for x in s["gyro"])
        ax, ay, az = (float(x) for x in s["accel"])
        rows.append([ts, gx, gy, gz, ax, ay, az])
    path = os.path.join(mav0_dir, "imu0", "data.csv")
    _write_csv(path, IMU_HEADER, rows)
    return path


def write_gt_csv(mav0_dir: str, gt_states: Sequence[Dict[str, object]]) -> str:
    """state_groundtruth_estimate0/data.csv. ``gt_states``: [{timestamp_ns,
    position[x,y,z] m, quat_wxyz[w,x,y,z]}]. Optional velocity[vx,vy,vz],
    bias_gyro[3], bias_accel[3] extend the row to the canonical 17-column EuRoC
    GT form; omit them for the pose-only 8-column form (the columns the checklist
    specifies: timestamp, p, q_w,q_x,q_y,q_z)."""
    full = any(("velocity" in s or "bias_gyro" in s or "bias_accel" in s) for s in gt_states)
    header = GT_HEADER_FULL if full else GT_HEADER_POSE
    rows = []
    for s in gt_states:
        ts = _require_int_ns(s["timestamp_ns"])
        px, py, pz = (float(x) for x in s["position"])
        q = list(s["quat_wxyz"])
        if len(q) != 4:
            raise ValueError(f"quat_wxyz must be length 4 (w,x,y,z), got {q!r}")
        qw, qx, qy, qz = (float(x) for x in q)
        row = [ts, px, py, pz, qw, qx, qy, qz]
        if full:
            v = s.get("velocity", [0.0, 0.0, 0.0])
            bg = s.get("bias_gyro", [0.0, 0.0, 0.0])
            ba = s.get("bias_accel", [0.0, 0.0, 0.0])
            row += [float(x) for x in list(v) + list(bg) + list(ba)]
        rows.append(row)
    path = os.path.join(mav0_dir, "state_groundtruth_estimate0", "data.csv")
    _write_csv(path, header, rows)
    return path


def write_cam0_sensor_yaml(mav0_dir: str, intr, rate_hz: float = 20.0,
                           T_cam_imu=None) -> str:
    """cam0/sensor.yaml — EuRoC pinhole model from CameraIntrinsics (spec §6/§2).
    ``intr`` is a common.intrinsics.CameraIntrinsics. T_cam_imu (4x4) defaults to
    identity (body ≡ camera, matching the §5 stream / scene_gt.json declaration)."""
    import yaml

    T = np.eye(4) if T_cam_imu is None else np.asarray(T_cam_imu, float)
    doc = {
        "sensor_type": "camera",
        "comment": "Isaac Sim synthetic pinhole (윤호). Fills spec §6 intrinsics.",
        "T_BS": {"cols": 4, "rows": 4, "data": [float(x) for x in T.reshape(-1)]},
        "rate_hz": float(rate_hz),
        "resolution": [int(intr.width), int(intr.height)],
        "camera_model": "pinhole",
        "intrinsics": [float(intr.fx), float(intr.fy), float(intr.cx), float(intr.cy)],
        "distortion_model": "radtan",
        "distortion_coefficients": [0.0, 0.0, 0.0, 0.0],  # distortion-free sim (spec §6)
    }
    path = os.path.join(mav0_dir, "cam0", "sensor.yaml")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    return path


def write_imu0_sensor_yaml(mav0_dir: str, rate_hz: float = 200.0) -> str:
    """imu0/sensor.yaml — rate + noise placeholders. Keep the four noise numbers
    identical to calib/kalibr_imu_chain.yaml AND to the Isaac Sim IMU sensor
    (calib/README.md: same source of truth or OpenVINS covariance is miscalibrated)."""
    import yaml

    doc = {
        "sensor_type": "imu",
        "comment": "Match noise to calib/kalibr_imu_chain.yaml + Isaac Sim IMU sensor.",
        "T_BS": {"cols": 4, "rows": 4, "data": [float(x) for x in np.eye(4).reshape(-1)]},
        "rate_hz": float(rate_hz),
        # placeholders (same units as calib/kalibr_imu_chain.yaml)
        "gyroscope_noise_density": 0.00016,
        "gyroscope_random_walk": 0.000022,
        "accelerometer_noise_density": 0.002,
        "accelerometer_random_walk": 0.0003,
    }
    path = os.path.join(mav0_dir, "imu0", "sensor.yaml")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    return path


def write_euroc(
    out_dir: str,
    cam_frames: Sequence[Dict[str, object]],
    imu_samples: Sequence[Dict[str, object]],
    gt_states: Sequence[Dict[str, object]],
    *,
    intr=None,
    image_src: Optional[Dict[int, str]] = None,
) -> Dict[str, object]:
    """Write a full EuRoC-ASL bag under <out_dir>/mav0/. Returns a summary dict.

    ``image_src`` (optional): {timestamp_ns: src_png_path} to copy the rendered
    frames into cam0/data/<ts>.png. If omitted, only cam0/data.csv is written and
    the images are expected to be dropped in by the Isaac render step (which names
    them <ts>.png — the timestamp is the filename)."""
    mav0 = os.path.join(out_dir, "mav0")
    os.makedirs(os.path.join(mav0, "cam0", "data"), exist_ok=True)

    if image_src:
        for ts, src in image_src.items():
            ts = _require_int_ns(ts)
            if os.path.exists(src):
                shutil.copy2(src, os.path.join(mav0, "cam0", "data", f"{ts}.png"))

    paths = {
        "cam0_csv": write_cam0_csv(mav0, cam_frames),
        "imu0_csv": write_imu0_csv(mav0, imu_samples),
        "gt_csv": write_gt_csv(mav0, gt_states),
        "imu0_sensor": write_imu0_sensor_yaml(mav0),
    }
    if intr is not None:
        paths["cam0_sensor"] = write_cam0_sensor_yaml(mav0, intr)
    return {
        "mav0": mav0,
        "n_cam": len(cam_frames),
        "n_imu": len(imu_samples),
        "n_gt": len(gt_states),
        "paths": paths,
    }


# ======================================================================
#  READERS  (for round-trip checks / downstream loading)
# ======================================================================
def _read_csv(path: str):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    return rows[0], rows[1:]


def read_cam0_csv(mav0_dir: str) -> List[Dict[str, object]]:
    _, rows = _read_csv(os.path.join(mav0_dir, "cam0", "data.csv"))
    return [{"timestamp_ns": int(r[0]), "filename": r[1]} for r in rows]


def read_imu0_csv(mav0_dir: str) -> List[Dict[str, object]]:
    _, rows = _read_csv(os.path.join(mav0_dir, "imu0", "data.csv"))
    return [
        {"timestamp_ns": int(r[0]),
         "gyro": [float(r[1]), float(r[2]), float(r[3])],
         "accel": [float(r[4]), float(r[5]), float(r[6])]}
        for r in rows
    ]


def read_gt_csv(mav0_dir: str) -> List[Dict[str, object]]:
    """Read state_groundtruth_estimate0/data.csv back (pose columns; quat WXYZ)."""
    _, rows = _read_csv(os.path.join(mav0_dir, "state_groundtruth_estimate0", "data.csv"))
    out = []
    for r in rows:
        out.append({
            "timestamp_ns": int(r[0]),
            "position": [float(r[1]), float(r[2]), float(r[3])],
            "quat_wxyz": [float(r[4]), float(r[5]), float(r[6]), float(r[7])],
        })
    return out
