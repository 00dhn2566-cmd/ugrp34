"""Perception surrogate: 4-corner window detector + multi-view ray triangulation.

Why a surrogate and not rendered images
---------------------------------------
Training needs millions of decisions; rendering + YOLO inference per step would
dominate cost. What the planner actually consumes is the *output* of the vision
stack, so we model that output directly and keep it statistically honest:

* detection   — a window is detected only if all 4 corners project inside the
                image (dataset policy A), it faces the camera, it is not hidden
                behind another wall, and it is big enough; confidence falls with
                apparent size and incidence angle; random misses (DR).
* corners     — projected with the real intrinsics (prototype camera.yaml) plus
                pixel noise sigma (DR; measured detector error ~8.9 px).
* triangulate — per corner, conf-weighted least squares over all rays so far:
                    A += w (I - d d^T),  b += w (I - d d^T) c,   p = A^-1 b
                exactly 태민 window_recon_node / prototype recon_rays (weight="conf",
                robust="none"), including the >= 2 deg parallax gate.
* quality     — no ground truth is used: residual RMS of the rays and the
                geometric dilution of A give a 1-sigma position uncertainty, the
                same kind of runtime proxy traj_manager.py used ("spread + resid").

The policy only ever sees these estimates (README §7.1 "no sim cheating").
Ground-truth error is exposed separately for the critic, rewards and logging.

Known simplifications (documented, configurable later): identity association is
assumed correct (colour + order), frames do not partially occlude corners, and
there is no Huber re-weighting (outlier_prob defaults to 0).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

import torch

from . import math3d as m3
from .scene import SceneTensors

C = 4   # corners per window


@dataclass
class Detections:
    detected: torch.Tensor     # (E, W) bool — detected in this frame
    conf: torch.Tensor         # (E, W)
    bearing_w: torch.Tensor    # (E, W, 3) unit ray to the detected centre (world)
    side_px: torch.Tensor      # (E, W) shortest projected side [px]
    frame_taken: torch.Tensor  # (E,) bool


@dataclass
class Estimates:
    valid: torch.Tensor        # (E, W) bool — all 4 corners triangulated & parallax gate passed
    corners: torch.Tensor      # (E, W, 4, 3)
    center: torch.Tensor       # (E, W, 3)
    normal: torch.Tensor       # (E, W, 3) approach side (spec §3.1 sign)
    size: torch.Tensor         # (E, W, 2)
    parallax_deg: torch.Tensor  # (E, W) min over corners
    resid_rms: torch.Tensor    # (E, W) [m]
    sigma: torch.Tensor        # (E, W) [m] 1-sigma worst-axis position uncertainty
    n_obs: torch.Tensor        # (E, W) frames that contributed


class PerceptionSurrogate:
    def __init__(self, cfg: Mapping, num_envs: int, max_windows: int, device):
        p = cfg["perception"]
        cam = p["camera"]
        self.cfg = p
        self.wall_half = float(cfg["scene"]["wall_half_extent"])
        self.E, self.W, self.device = num_envs, max_windows, device
        self.fx, self.fy = float(cam["fx"]), float(cam["fy"])
        self.cx, self.cy = float(cam["cx"]), float(cam["cy"])
        self.width, self.height = float(cam["width"]), float(cam["height"])
        self.cam_offset_b = torch.tensor(cam["offset_body"], dtype=torch.float32, device=device)
        self.R_BC = torch.tensor(m3.R_BODY_CV, dtype=torch.float32, device=device)
        self.cos_max_inc = math.cos(math.radians(float(p["max_incidence_deg"])))
        self.min_par = float(p["min_parallax_deg"])
        E, W = num_envs, max_windows
        z = lambda *s: torch.zeros(*s, device=device)  # noqa: E731
        self.A = z(E, W, C, 3, 3)
        self.b = z(E, W, C, 3)
        self.s = z(E, W, C)
        self.wsum = z(E, W, C)
        self.n_rays = z(E, W, C)
        self.n_obs = z(E, W)
        # parallax bookkeeping: first ray + 2D basis, and 4 extreme rays per corner
        self.d0 = z(E, W, C, 3)
        self.e1 = z(E, W, C, 3)
        self.e2 = z(E, W, C, 3)
        self.ext = z(E, W, C, 4, 3)          # argmin/argmax along e1, e2
        self.ext_val = z(E, W, C, 4)
        # latest detection (kept between frames, with age)
        self.last_bearing = z(E, W, 3)
        self.last_side = z(E, W)
        self.last_conf = z(E, W)
        self.det_age = torch.full((E, W), 1.0e3, device=device)
        self.gap = torch.full((E, W), 1.0e3, device=device)   # decisions since detected (wall clock: see tick)
        self.last_corners_px = z(E, W, C, 2)   # latest corner pixels fed in (debug / integration tests)
        self.last_det = torch.zeros(E, W, dtype=torch.bool, device=device)
        # per-env noise (domain randomisation)
        self.sigma_px = torch.full((E,), 2.0, device=device)
        self.miss_prob = z(E)
        self._est_cache = None

    # ------------------------------------------------------------------ #
    def reset(self, env_ids: torch.Tensor, sigma_px=None, miss_prob=None) -> None:
        for t in (self.A, self.b, self.s, self.wsum, self.n_rays, self.n_obs, self.d0, self.e1,
                  self.e2, self.ext, self.ext_val, self.last_bearing, self.last_side, self.last_conf,
                  self.last_corners_px):
            t[env_ids] = 0.0
        self.last_det[env_ids] = False
        self.det_age[env_ids] = 1.0e3
        self.gap[env_ids] = 1.0e3
        if sigma_px is not None:
            self.sigma_px[env_ids] = sigma_px
        if miss_prob is not None:
            self.miss_prob[env_ids] = miss_prob
        self._est_cache = None

    def camera_pose(self, pos_w: torch.Tensor, quat_wxyz: torch.Tensor):
        """Body pose -> (camera centre (E,3), R_WC (E,3,3)) with R_WC = R_WB R_BC."""
        R_WB = m3.quat_to_rotmat(quat_wxyz)
        c = pos_w + (R_WB @ self.cam_offset_b.unsqueeze(-1)).squeeze(-1)
        return c, R_WB @ self.R_BC

    # ------------------------------------------------------------------ #
    @torch.no_grad()
    def observe(self, cam_c: torch.Tensor, R_WC: torch.Tensor, scene: SceneTensors,
                take: torch.Tensor, gen: torch.Generator) -> Detections:
        """One camera frame for envs where ``take`` is True. Updates the ray accumulators."""
        return self.add_detections(*self.capture(cam_c, R_WC, scene, take, gen))

    @torch.no_grad()
    def capture(self, cam_c: torch.Tensor, R_WC: torch.Tensor, scene: SceneTensors,
                take: torch.Tensor, gen: torch.Generator) -> tuple:
        """Synthetic detector output for one frame, not yet fed to the triangulator: the arguments
        of :meth:`add_detections` (ray camera centre, ray R_WC, corner pixels, det, conf, take), so
        the frame can also be delivered later (detection latency, core/env_core.py)."""
        p = self.cfg
        E, W = self.E, self.W
        P = scene.corners                                     # (E,W,4,3)
        rel = P - cam_c[:, None, None, :]
        pc = torch.einsum("eji,ewkj->ewki", R_WC, rel)        # R_WC^T rel -> CV camera coords
        zc = pc[..., 2]
        front = (zc > 0.2).all(-1)
        zs = zc.clamp_min(1e-3)
        # true camera may differ from the calibration the triangulator uses (stress test knob)
        cal = float(p.get("calib_fx_scale", 1.0))
        u = self.fx * cal * pc[..., 0] / zs + self.cx
        v = self.fy * cal * pc[..., 1] / zs + self.cy
        mg = float(p["image_margin_px"])
        inside = ((u >= mg) & (u <= self.width - mg) & (v >= mg) & (v <= self.height - mg)).all(-1)

        to_cam = cam_c[:, None, :] - scene.center             # (E,W,3)
        rng = to_cam.norm(dim=-1)
        cos_inc = (m3.normalize(to_cam) * scene.normal).sum(-1)
        facing = cos_inc > self.cos_max_inc
        in_range = rng < float(p["max_range"])

        vis = self._unoccluded(cam_c, P, scene)                # (E,W) all 4 corners clear

        uv = torch.stack((u, v), -1)                           # (E,W,4,2)
        sides = (uv - uv.roll(-1, dims=2)).norm(dim=-1)        # TL-TR, TR-BR, BR-BL, BL-TL
        side_px = sides.min(dim=-1).values
        conf = torch.sigmoid((side_px - float(p["conf_side_px50"])) / float(p["conf_side_px_scale"]))
        conf = conf * cos_inc.clamp_min(0.0).sqrt()

        miss = torch.rand(E, W, generator=gen, device=self.device) < self.miss_prob[:, None]
        det = (take[:, None] & scene.valid & front & inside & facing & in_range & vis
               & (side_px >= float(p["min_side_px"])) & (conf >= float(p["det_conf_min"])) & ~miss)

        # noisy corners -> world rays
        sig = self.sigma_px[:, None, None, None]
        noise = torch.randn(E, W, C, 2, generator=gen, device=self.device) * sig
        if float(p["outlier_prob"]) > 0:
            out = torch.rand(E, W, C, 1, generator=gen, device=self.device) < float(p["outlier_prob"])
            noise = noise + out * torch.randn(E, W, C, 2, generator=gen, device=self.device) * sig * 6.0
        noisy_uv = uv + noise
        # rays are cast from the ESTIMATED camera pose (VIO error knobs; no draws when 0)
        ray_c, ray_R = cam_c, R_WC
        pn, pd = float(p.get("pose_noise_m", 0.0)), float(p.get("pose_noise_deg", 0.0))
        if pn > 0:
            ray_c = cam_c + torch.randn(E, 3, generator=gen, device=self.device) * pn
        if pd > 0:
            yaw_err = torch.deg2rad(torch.randn(E, generator=gen, device=self.device) * pd)
            ray_R = m3.rot_z(yaw_err) @ R_WC
        return ray_c, ray_R, noisy_uv, det, conf, take

    @torch.no_grad()
    def add_detections(self, cam_c: torch.Tensor, R_WC: torch.Tensor, corners_px: torch.Tensor,
                       det: torch.Tensor, conf: torch.Tensor, take: torch.Tensor) -> Detections:
        """Feed detector output (synthetic or REAL) into the triangulator.

        corners_px (E,W,4,2) in CORNER_ORDER, det (E,W) bool, conf (E,W), take (E,) frame taken.
        This is the deployment seam: planners/onboard.py calls it with the vision
        module's §5 messages, so the policy sees estimates built exactly as in training.
        """
        un, vn = corners_px[..., 0], corners_px[..., 1]
        d_c = m3.normalize(torch.stack(((un - self.cx) / self.fx, (vn - self.cy) / self.fy,
                                        torch.ones_like(un)), -1))
        d_w = torch.einsum("eij,ewkj->ewki", R_WC, d_c)       # (E,W,4,3)

        self._accumulate(det, conf, cam_c, d_w)
        self.last_corners_px = corners_px.clone()
        self.last_det = det.clone()

        sides = (corners_px - corners_px.roll(-1, dims=2)).norm(dim=-1)
        side_px = sides.min(dim=-1).values
        bearing = m3.normalize(d_w.mean(dim=2))
        detf = det.float()
        self.last_bearing = torch.where(det[..., None], bearing, self.last_bearing)
        self.last_side = torch.where(det, side_px, self.last_side)
        self.last_conf = torch.where(det, conf, self.last_conf)
        self.det_age = torch.where(det, torch.zeros_like(self.det_age), self.det_age + take[:, None].float())
        self.gap = torch.where(det, torch.zeros_like(self.gap), self.gap)
        self._est_cache = None
        return Detections(det, conf * detf, bearing, side_px * detf, take)

    def tick(self) -> None:
        """One decision passed. ``gap`` counts decisions since each window was detected whether or not a
        camera frame arrived, so it keeps growing through an outage (``det_age`` counts frames and stops)."""
        self.gap = self.gap + 1.0

    def _unoccluded(self, cam_c, P, scene: SceneTensors) -> torch.Tensor:
        """(E,W): every corner ray of window j passes through the opening of any
        other wall i it crosses (walls are planes of half-extent ``wall_half``)."""
        c_i = scene.center[:, None, None, :, :]               # (E,1,1,Wi,3)
        n_i = scene.normal[:, None, None, :, :]
        cam = cam_c[:, None, None, None, :]
        pt = P[:, :, :, None, :]                               # (E,Wj,4,1,3)
        s_cam = ((cam - c_i) * n_i).sum(-1)                    # (E,1,1,Wi)
        s_pt = ((pt - c_i) * n_i).sum(-1)                      # (E,Wj,4,Wi)
        crosses = (s_cam * s_pt) < 0
        t = s_cam / (s_cam - s_pt).where(crosses, torch.ones_like(s_pt))
        X = cam + t[..., None] * (pt - cam)
        du = ((X - c_i) * scene.right[:, None, None, :, :]).sum(-1)
        dv = ((X - c_i) * scene.up[:, None, None, :, :]).sum(-1)
        hw = scene.size[:, None, None, :, 0] / 2
        hh = scene.size[:, None, None, :, 1] / 2
        in_open = (du.abs() <= hw) & (dv.abs() <= hh)
        on_wall = (du.abs() <= self.wall_half) & (dv.abs() <= self.wall_half)
        Wn = scene.center.shape[1]
        not_self = ~torch.eye(Wn, dtype=torch.bool, device=self.device)[None, :, None, :]
        blocked = crosses & ~in_open & on_wall & not_self & scene.valid[:, None, None, :]
        return ~blocked.any(-1).any(-1)

    def _accumulate(self, det, conf, cam_c, d_w) -> None:
        w = (conf * det.float())[..., None]                   # (E,W,1) broadcast over corners
        w = w.expand(-1, -1, C)
        eye = torch.eye(3, device=self.device)
        M = eye - d_w[..., :, None] * d_w[..., None, :]        # (E,W,4,3,3)
        c = cam_c[:, None, None, :].expand_as(d_w)
        Mc = (M @ c[..., None]).squeeze(-1)
        self.A += w[..., None, None] * M
        self.b += w[..., None] * Mc
        self.s += w * (c * Mc).sum(-1)
        self.wsum += w
        first = det[..., None].expand(-1, -1, C) & (self.n_rays == 0)
        self.n_rays += det[..., None].float()
        self.n_obs += det.float()

        # parallax: express the ray in a 2D tangent basis of the first ray
        if first.any():
            d0 = d_w
            helper = torch.zeros_like(d0)
            helper[..., 2] = 1.0
            alt = torch.zeros_like(d0)
            alt[..., 1] = 1.0
            helper = torch.where((d0[..., 2].abs() > 0.9)[..., None], alt, helper)
            e1 = m3.normalize(torch.cross(d0, helper, dim=-1))
            e2 = torch.cross(d0, e1, dim=-1)
            f = first[..., None]
            self.d0 = torch.where(f, d0, self.d0)
            self.e1 = torch.where(f, e1, self.e1)
            self.e2 = torch.where(f, e2, self.e2)
            self.ext = torch.where(f[..., None], d0[..., None, :].expand_as(self.ext), self.ext)
            self.ext_val = torch.where(first[..., None], torch.zeros_like(self.ext_val), self.ext_val)
        a1 = (d_w * self.e1).sum(-1)
        a2 = (d_w * self.e2).sum(-1)
        vals = torch.stack((-a1, a1, -a2, a2), -1)            # minimise a1 / maximise a1 / ...
        better = (vals > self.ext_val) & det[..., None, None].expand(-1, -1, C, 4)
        self.ext_val = torch.where(better, vals, self.ext_val)
        self.ext = torch.where(better[..., None], d_w[..., None, :].expand_as(self.ext), self.ext)

    # ------------------------------------------------------------------ #
    @torch.no_grad()
    def estimate(self) -> Estimates:
        if self._est_cache is not None:
            return self._est_cache
        ridge = float(self.cfg["ridge"])
        eye = torch.eye(3, device=self.device)
        A = self.A + ridge * eye
        p = m3.solve3(A, self.b)                                         # (E,W,4,3)

        cand = torch.cat((self.ext, self.d0[..., None, :]), dim=-2)     # (E,W,4,5,3)
        dots = (cand[..., :, None, :] * cand[..., None, :, :]).sum(-1)  # (E,W,4,5,5)
        par = torch.rad2deg(torch.acos(dots.amin(dim=(-1, -2)).clamp(-1.0, 1.0)))
        par = torch.where(self.n_rays >= 2, par, torch.zeros_like(par))  # (E,W,4)

        ok_corner = (self.n_rays >= 2) & (par >= self.min_par) & (self.wsum > 0)
        valid = ok_corner.all(-1)

        ws = self.wsum.clamp_min(1e-9)
        sq = (p[..., None, :] @ self.A @ p[..., :, None]).squeeze(-1).squeeze(-1) \
            - 2 * (p * self.b).sum(-1) + self.s
        resid = (sq.clamp_min(0.0) / ws).sqrt()                          # (E,W,4)
        # sigma: unit-weight-equivalent information matrix -> worst-axis std.
        # Each ray contributes 2 perpendicular residual axes; 3 parameters are fitted,
        # so mean r^2 = sigma^2 (2n - 3) / n  ->  sigma = resid * sqrt(n / (2n - 3)).
        n = self.n_rays.clamp_min(1.0)
        A_unit = A * (n / ws)[..., None, None]
        lam_min = m3.eigmin_sym3(A_unit).clamp_min(1e-9)
        dof = (2 * n - 3).clamp_min(1.0)
        sig_ray = resid * (n / dof).sqrt()
        sigma_c = sig_ray / lam_min.sqrt()
        big = torch.full_like(sigma_c, 1.0e3)
        sigma_c = torch.where(ok_corner, sigma_c, big)

        tl, tr, br, bl = p.unbind(dim=2)
        center = p.mean(dim=2)
        width = 0.5 * ((tr - tl).norm(dim=-1) + (br - bl).norm(dim=-1))
        height = 0.5 * ((bl - tl).norm(dim=-1) + (br - tr).norm(dim=-1))
        normal = m3.normalize(torch.cross(bl - tl, tr - tl, dim=-1))

        vf = valid[..., None]
        est = Estimates(
            valid=valid,
            corners=torch.where(vf[..., None], p, torch.zeros_like(p)),
            center=torch.where(vf, center, torch.zeros_like(center)),
            normal=torch.where(vf, normal, torch.zeros_like(normal)),
            size=torch.where(vf, torch.stack((width, height), -1), torch.zeros(*valid.shape, 2, device=self.device)),
            parallax_deg=par.amin(dim=-1),
            resid_rms=torch.where(valid, resid.mean(-1), torch.zeros_like(resid[..., 0])),
            sigma=sigma_c.amax(dim=-1),
            n_obs=self.n_obs.clone(),
        )
        self._est_cache = est
        return est

    @staticmethod
    def center_error(est: Estimates, scene: SceneTensors, cap: float = 5.0) -> torch.Tensor:
        """(E,W) GT centre error [m]; ``cap`` where no estimate exists. Privileged."""
        err = (est.center - scene.center).norm(dim=-1)
        return torch.where(est.valid, err.clamp_max(cap), torch.full_like(err, cap))
