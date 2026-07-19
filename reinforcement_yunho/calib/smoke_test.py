#!/usr/bin/env python3
"""Smoke test for calib/ -- runnable now with numpy + pyyaml only.

Checks:
  1. Both Kalibr YAMLs parse and have the required keys with correct
     types/lengths (cam0 pinhole intrinsics/resolution/distortion/cam_overlaps/
     extrinsics/timeshift/rostopic; imu0 four noise densities + update_rate/model/
     time_offset/rostopic/T_i_b).
  2. estimator_config.yaml (the OpenVINS master config) parses, is MONO by
     default (max_cameras 1, use_stereo false), has gravity_mag 9.81 + sane
     tracking/init params, and its relative_config_imu/imucam point at the two
     chain files that exist here.
  3. The RL/vision->calib intrinsics seam end to end: scripts/gen_intrinsics.py
     writes 길남's exact {width,height,fx,fy,cx,cy,distortion} schema, and
     fill_from_intrinsics reads THAT file and stamps cam0.intrinsics ==
     [fx,fy,cx,cy] / resolution == [W,H] into a camchain copy.

Does NOT depend on vision/synth_intrinsics.yaml existing (the intrinsics YAML is
synthesised in a temp dir).

    python3 calib/smoke_test.py
"""
from __future__ import annotations

import os
import sys
import tempfile

# --- bootstrap: shared 'common' package + this dir + scripts/ ----------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE, os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import yaml  # noqa: E402

from common import CameraIntrinsics  # noqa: E402
import fill_from_intrinsics as ffi  # noqa: E402
import gen_intrinsics as gi  # noqa: E402  (scripts/gen_intrinsics.py)

IMUCAM = os.path.join(_HERE, "kalibr_imucam_chain.yaml")
IMU = os.path.join(_HERE, "kalibr_imu_chain.yaml")
ESTIMATOR = os.path.join(_HERE, "estimator_config.yaml")

# 길남's exact synth_intrinsics.yaml field set (CONVENTIONS.md).
GILNAM_INTRINSICS_KEYS = {"width", "height", "fx", "fy", "cx", "cy", "distortion"}


def _is_4x4(m) -> bool:
    return (
        isinstance(m, list)
        and len(m) == 4
        and all(isinstance(r, list) and len(r) == 4 for r in m)
    )


def check_imucam() -> None:
    with open(IMUCAM) as f:
        doc = yaml.safe_load(f)
    assert "cam0" in doc, "missing cam0 block"
    cam = doc["cam0"]

    assert cam["camera_model"] == "pinhole", cam["camera_model"]

    intr = cam["intrinsics"]
    assert isinstance(intr, list) and len(intr) == 4, f"intrinsics {intr!r}"
    assert all(isinstance(v, (int, float)) for v in intr), intr

    assert cam["distortion_model"] == "radtan", cam["distortion_model"]
    dc = cam["distortion_coeffs"]
    assert isinstance(dc, list) and len(dc) == 4, f"distortion_coeffs {dc!r}"
    assert all(isinstance(v, (int, float)) for v in dc), dc

    res = cam["resolution"]
    assert isinstance(res, list) and len(res) == 2, f"resolution {res!r}"
    assert all(isinstance(v, int) for v in res), res
    assert res == [1280, 720], f"CONVENTIONS.md fixes 1280x720, got {res}"

    # cam_overlaps: a list (empty for the mono default; [1] when stereo).
    assert "cam_overlaps" in cam, "missing cam_overlaps"
    assert isinstance(cam["cam_overlaps"], list), f"cam_overlaps {cam['cam_overlaps']!r}"

    # T_cam_imu is the PINNED direction stored here (Kalibr: p_cam = T_cam_imu*p_imu);
    # 태민's recon inverts it. Must be 4x4.
    assert "T_cam_imu" in cam, "missing camera-imu extrinsic T_cam_imu"
    assert _is_4x4(cam["T_cam_imu"]), "T_cam_imu is not 4x4"

    assert isinstance(cam["timeshift_cam_imu"], (int, float)), "timeshift type"
    assert cam["rostopic"] == "/cam0/image_raw", cam["rostopic"]
    print(f"  [ok] cam0: pinhole/radtan, intrinsics len4, resolution {res}, "
          f"cam_overlaps {cam['cam_overlaps']}, T_cam_imu 4x4, rostopic {cam['rostopic']}")


def check_imu() -> None:
    with open(IMU) as f:
        doc = yaml.safe_load(f)
    assert "imu0" in doc, "missing imu0 block"
    imu = doc["imu0"]

    for key in (
        "accelerometer_noise_density",
        "accelerometer_random_walk",
        "gyroscope_noise_density",
        "gyroscope_random_walk",
    ):
        assert key in imu, f"missing {key}"
        assert isinstance(imu[key], (int, float)), f"{key} not numeric"
        assert imu[key] > 0.0, f"{key} must be positive, got {imu[key]}"

    assert float(imu["update_rate"]) == 200.0, imu["update_rate"]
    assert imu["model"] == "calibrated", imu["model"]
    assert "time_offset" in imu, "missing time_offset"
    assert isinstance(imu["time_offset"], (int, float)), imu["time_offset"]
    assert imu["rostopic"] == "/imu0", imu["rostopic"]
    assert _is_4x4(imu["T_i_b"]), "T_i_b is not 4x4"
    print("  [ok] imu0: 4 noise params >0, update_rate 200.0, model calibrated, "
          f"time_offset {imu['time_offset']}, rostopic /imu0, T_i_b 4x4")


def check_estimator() -> None:
    with open(ESTIMATOR) as f:
        doc = yaml.safe_load(f)
    assert isinstance(doc, dict), "estimator_config.yaml did not parse to a mapping"

    # MONO by default.
    assert doc["max_cameras"] == 1, f"expected mono max_cameras 1, got {doc['max_cameras']}"
    assert doc["use_stereo"] is False, f"expected use_stereo false, got {doc['use_stereo']}"

    # gravity + a couple of sane tracking/init params present and numeric.
    assert float(doc["gravity_mag"]) == 9.81, doc["gravity_mag"]
    for key in ("num_pts", "fast_threshold", "init_window_time", "max_clones"):
        assert key in doc, f"missing {key}"
        assert isinstance(doc[key], (int, float)), f"{key} not numeric"

    # references to the two chains resolve to files that exist next to it.
    for key, expect in (("relative_config_imu", IMU),
                        ("relative_config_imucam", IMUCAM)):
        rel = doc[key]
        assert isinstance(rel, str) and rel, f"{key} missing/empty"
        assert os.path.basename(expect) == rel, f"{key}={rel!r} != {os.path.basename(expect)}"
        assert os.path.exists(os.path.join(_HERE, rel)), f"{key} -> {rel} does not exist"
    print(f"  [ok] estimator_config: mono (max_cameras 1, use_stereo false), "
          f"gravity 9.81, chains -> {doc['relative_config_imu']} + {doc['relative_config_imucam']}")


def check_intrinsics_seam() -> None:
    # Build intrinsics WITHOUT touching vision/synth_intrinsics.yaml.
    ci = CameraIntrinsics.from_fov(width=1280, height=720, hfov_deg=90.0)
    expected = ci.kalibr_intrinsics()  # [fx, fy, cx, cy]

    with tempfile.TemporaryDirectory() as td:
        intr_path = os.path.join(td, "synth_intrinsics.yaml")
        # (a) gen_intrinsics writes 길남's EXACT schema.
        rc = gi.main(["--width", "1280", "--height", "720", "--hfov", "90",
                      "--out", intr_path])
        assert rc == 0, f"gen_intrinsics CLI returned {rc}"
        with open(intr_path) as f:
            intr_doc = yaml.safe_load(f)
        assert set(intr_doc) == GILNAM_INTRINSICS_KEYS, (
            f"synth_intrinsics keys {set(intr_doc)} != {GILNAM_INTRINSICS_KEYS}")
        assert intr_doc["distortion"] == [], f"distortion should be [] (none), got {intr_doc['distortion']}"

        # (b) fill_from_intrinsics reads THAT file and stamps the camchain.
        out_path = os.path.join(td, "camchain.filled.yaml")
        rc = ffi.main(
            ["--intrinsics", intr_path, "--template", IMUCAM, "--out", out_path]
        )
        assert rc == 0, f"fill CLI returned {rc}"
        with open(out_path) as f:
            filled = yaml.safe_load(f)

    got = filled["cam0"]["intrinsics"]
    assert got == [float(v) for v in expected], f"{got} != {expected}"
    assert filled["cam0"]["resolution"] == [1280, 720], filled["cam0"]["resolution"]
    # Extrinsics carried over untouched (still placeholder).
    assert _is_4x4(filled["cam0"]["T_cam_imu"]), "T_cam_imu lost in fill"
    print(f"  [ok] gen_intrinsics keys == 길남 set {sorted(GILNAM_INTRINSICS_KEYS)}, distortion []")
    print(f"  [ok] fill: cam0.intrinsics == [fx,fy,cx,cy] = {got}; "
          f"resolution [1280,720]; T_cam_imu preserved")


def main() -> int:
    print("calib smoke test")
    check_imucam()
    check_imu()
    check_estimator()
    check_intrinsics_seam()
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
