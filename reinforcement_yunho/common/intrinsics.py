"""Camera intrinsics: FOV / focal-length <-> pinhole (fx, fy, cx, cy).

This is the single source of truth for the numbers that go into
`vision/synth_intrinsics.yaml` (spec 6) and into the Kalibr YAMLs for OpenVINS.

Pixel model (OpenCV / computer-vision convention):
    u = fx * (X_cam / Z_cam) + cx        # +u to the right
    v = fy * (Y_cam / Z_cam) + cy        # +v downward
    K = [[fx, 0, cx],
         [ 0, fy, cy],
         [ 0,  0,  1]]

Principal point convention: cx = W/2, cy = H/2 (image centre for a synthetic
pinhole camera). If you later match a real Kalibr calibration, override cx/cy
with the calibrated values.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

import numpy as np


@dataclass
class CameraIntrinsics:
    """Pinhole intrinsics for one camera. Distortion defaults to zero (spec 6:
    the synthetic camera is rendered distortion-free unless deliberately added)."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    # radtan (plumb-bob) [k1, k2, p1, p2, k3]; all zero == no distortion.
    distortion: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0])
    distortion_model: str = "radtan"

    # ---- constructors -------------------------------------------------------
    @classmethod
    def from_fov(
        cls,
        width: int,
        height: int,
        hfov_deg: float,
        vfov_deg: float | None = None,
        cx: float | None = None,
        cy: float | None = None,
    ) -> "CameraIntrinsics":
        """Build intrinsics from field(s) of view.

        If ``vfov_deg`` is None we assume square pixels (fy == fx) and derive the
        vertical FOV from the aspect ratio.
        """
        if hfov_deg <= 0 or hfov_deg >= 180:
            raise ValueError(f"hfov_deg must be in (0, 180), got {hfov_deg}")
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        if vfov_deg is None:
            fy = fx  # square pixels
        else:
            if vfov_deg <= 0 or vfov_deg >= 180:
                raise ValueError(f"vfov_deg must be in (0, 180), got {vfov_deg}")
            fy = (height / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
        return cls(
            width=width,
            height=height,
            fx=fx,
            fy=fy,
            cx=width / 2.0 if cx is None else cx,
            cy=height / 2.0 if cy is None else cy,
        )

    @classmethod
    def from_focal_aperture(
        cls,
        width: int,
        height: int,
        focal_length_mm: float,
        horizontal_aperture_mm: float,
        vertical_aperture_mm: float | None = None,
    ) -> "CameraIntrinsics":
        """Build intrinsics the way Isaac Sim / USD cameras are parameterised:
        a focal length and a sensor (aperture) size, both in millimetres.

            fx = focal_length_mm / horizontal_aperture_mm * width

        If ``vertical_aperture_mm`` is None it is inferred from the pixel aspect
        ratio so that pixels stay square.
        """
        if vertical_aperture_mm is None:
            vertical_aperture_mm = horizontal_aperture_mm * (height / width)
        fx = focal_length_mm / horizontal_aperture_mm * width
        fy = focal_length_mm / vertical_aperture_mm * height
        return cls(
            width=width,
            height=height,
            fx=fx,
            fy=fy,
            cx=width / 2.0,
            cy=height / 2.0,
        )

    # ---- derived quantities -------------------------------------------------
    def K(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    @property
    def hfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan((self.width / 2.0) / self.fx))

    @property
    def vfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan((self.height / 2.0) / self.fy))

    # ---- serialisation ------------------------------------------------------
    def to_yaml_dict(self) -> dict:
        """Flat dict that maps 1:1 onto vision/synth_intrinsics.yaml."""
        return {
            "width": int(self.width),
            "height": int(self.height),
            "fx": float(self.fx),
            "fy": float(self.fy),
            "cx": float(self.cx),
            "cy": float(self.cy),
            "distortion_model": self.distortion_model,
            "distortion_coeffs": [float(c) for c in self.distortion],
            # convenience, not read back (derived from fx/fy):
            "hfov_deg": round(self.hfov_deg, 6),
            "vfov_deg": round(self.vfov_deg, 6),
        }

    @classmethod
    def from_yaml_dict(cls, d: dict) -> "CameraIntrinsics":
        return cls(
            width=int(d["width"]),
            height=int(d["height"]),
            fx=float(d["fx"]),
            fy=float(d["fy"]),
            cx=float(d["cx"]),
            cy=float(d["cy"]),
            distortion=list(d.get("distortion_coeffs", [0.0] * 5)),
            distortion_model=d.get("distortion_model", "radtan"),
        )

    def kalibr_intrinsics(self) -> List[float]:
        """Kalibr 'intrinsics' vector: [fx, fy, cx, cy]."""
        return [float(self.fx), float(self.fy), float(self.cx), float(self.cy)]
