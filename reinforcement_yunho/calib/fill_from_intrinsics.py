#!/usr/bin/env python3
"""Fill the cam0 intrinsics/resolution/distortion of the Kalibr camchain from
a synthetic-camera intrinsics YAML.

WHAT
    Reads an intrinsics YAML in 길남's exact schema
    {width, height, fx, fy, cx, cy, distortion} (what scripts/gen_intrinsics.py
    writes and what overall_gilnam/vision/synth_intrinsics.yaml uses), and stamps
    its fx/fy/cx/cy, resolution and distortion into a COPY of
    calib/kalibr_imucam_chain.yaml. The extrinsics
    (T_cam_imu) and every IMU noise number are intentionally left untouched --
    they come from the drone CAD and the IMU datasheet / Isaac Sim IMU sensor,
    not from the camera intrinsics (see calib/README.md).

WHY
    The camchain OpenVINS consumes MUST describe the exact camera the images
    were rendered with. Hand-copying fx/fy/cx/cy invites drift; this helper
    makes CameraIntrinsics the single source of truth (spec 6 / CONVENTIONS.md).

NOTE ON COMMENTS
    pyyaml does not preserve comments on round-trip, so the generated output is
    a plain-data file. The rich field documentation lives in the template
    (calib/kalibr_imucam_chain.yaml); the output is a machine artifact whose
    T_cam_imu and IMU noise still need manual fill.

Runnable now (numpy + pyyaml only), no dependency on vision/ having run: pass the
intrinsics path explicitly.

    python3 calib/fill_from_intrinsics.py \
        --intrinsics vision/synth_intrinsics.yaml \
        --template   calib/kalibr_imucam_chain.yaml \
        --out        calib/kalibr_imucam_chain.filled.yaml
"""
from __future__ import annotations

import argparse
import copy
import os
import sys
from typing import Any, Dict

# --- bootstrap: import the shared 'common' package regardless of cwd ----------
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from common import CameraIntrinsics  # noqa: E402

import yaml  # noqa: E402  (pyyaml is a core dep per requirements.txt)


# --- pure logic (runnable + unit-tested here) --------------------------------
def fill_camchain_intrinsics(
    camchain: Dict[str, Any],
    intr: CameraIntrinsics,
    cam_key: str = "cam0",
) -> Dict[str, Any]:
    """Return a COPY of ``camchain`` with ``cam_key``'s camera fields set from
    ``intr``. Only the camera-owned fields are touched; T_cam_imu,
    timeshift_cam_imu and rostopic are preserved verbatim.

    Fields written:
        camera_model      -> "pinhole"
        intrinsics        -> [fx, fy, cx, cy]              (intr.kalibr_intrinsics())
        resolution        -> [width, height]
        distortion_model  -> intr.distortion_model         ("radtan")
        distortion_coeffs -> first 4 radtan coeffs [k1,k2,p1,p2]
                             (CameraIntrinsics stores 5 [k1,k2,p1,p2,k3];
                              Kalibr's 4-coeff radtan drops k3, which is 0 for
                              our distortion-free synthetic camera.)
    """
    out = copy.deepcopy(camchain)
    if cam_key not in out or not isinstance(out[cam_key], dict):
        raise KeyError(
            f"template has no '{cam_key}' camera block; got keys {list(out)}"
        )
    cam = out[cam_key]
    cam["camera_model"] = "pinhole"
    cam["intrinsics"] = [float(v) for v in intr.kalibr_intrinsics()]
    cam["resolution"] = [int(intr.width), int(intr.height)]
    cam["distortion_model"] = str(intr.distortion_model)
    cam["distortion_coeffs"] = [float(c) for c in list(intr.distortion)[:4]]
    return out


def load_intrinsics(path: str) -> CameraIntrinsics:
    """Load a synth_intrinsics.yaml into a CameraIntrinsics.

    Reads 길남's EXACT schema {width, height, fx, fy, cx, cy, distortion}
    (CONVENTIONS.md; overall_gilnam/vision/synth_intrinsics.yaml), where
    `distortion` is a list of radtan coeffs and `[]` means distortion-free. That
    key is NOT what CameraIntrinsics.from_yaml_dict expects (it looks for
    distortion_coeffs), so we map it here. Kept backward-compatible with the older
    to_yaml_dict schema (distortion_coeffs) for robustness.
    """
    with open(path, "r") as f:
        d = yaml.safe_load(f)
    if not isinstance(d, dict):
        raise ValueError(f"{path} did not parse to a mapping")
    # 길남 schema: `distortion` (empty list == none). Fallback: legacy
    # `distortion_coeffs`. Empty -> distortion-free (all-zero radtan).
    dist = d.get("distortion")
    if dist is None:
        dist = d.get("distortion_coeffs")
    dist = [float(c) for c in dist] if dist else [0.0, 0.0, 0.0, 0.0, 0.0]
    return CameraIntrinsics(
        width=int(d["width"]),
        height=int(d["height"]),
        fx=float(d["fx"]),
        fy=float(d["fy"]),
        cx=float(d["cx"]),
        cy=float(d["cy"]),
        distortion=dist,
    )


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def dump_yaml(obj: Dict[str, Any], path: str) -> None:
    with open(path, "w") as f:
        yaml.safe_dump(obj, f, default_flow_style=False, sort_keys=False)


# --- CLI ---------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    _here = os.path.dirname(os.path.abspath(__file__))
    p.add_argument(
        "--intrinsics",
        required=True,
        help="Path to synth_intrinsics.yaml (CameraIntrinsics.to_yaml_dict schema).",
    )
    p.add_argument(
        "--template",
        default=os.path.join(_here, "kalibr_imucam_chain.yaml"),
        help="Kalibr camchain template to copy (default: calib/kalibr_imucam_chain.yaml).",
    )
    p.add_argument(
        "--out",
        default=os.path.join(_here, "kalibr_imucam_chain.filled.yaml"),
        help="Output path for the filled camchain.",
    )
    p.add_argument(
        "--cam-key", default="cam0", help="Camera block key to fill (default: cam0)."
    )
    args = p.parse_args(argv)

    intr = load_intrinsics(args.intrinsics)
    template = load_yaml(args.template)
    filled = fill_camchain_intrinsics(template, intr, cam_key=args.cam_key)
    dump_yaml(filled, args.out)

    print(f"[fill_from_intrinsics] intrinsics <- {args.intrinsics}")
    print(f"[fill_from_intrinsics] template   <- {args.template}")
    print(f"[fill_from_intrinsics] wrote      -> {args.out}")
    print(f"  {args.cam_key}.intrinsics = {filled[args.cam_key]['intrinsics']}")
    print(f"  {args.cam_key}.resolution = {filled[args.cam_key]['resolution']}")
    print("  NOTE: T_cam_imu and all IMU noise values remain TODO placeholders.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
