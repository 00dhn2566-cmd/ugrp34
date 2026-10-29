"""YOLO-pose window detector in the loop: an onboard image -> per-window corner pixels, detected flag and confidence,
the layout core/perception.py add_detections takes — so the triangulator, the estimates and the policy's observations
are built exactly as in training, only the corners come from a detector instead of the surrogate's noisy projection.
"""
from __future__ import annotations

import numpy as np
import torch


class YoloCorners:
    """ultralytics YOLO-pose with 4 keypoints (TL, TR, BR, BL) and classes = frame colour (0 red, 1 green, 2 blue)."""

    def __init__(self, weights: str, device: str = "cuda:0", imgsz: int = 960, conf_min: float = 0.25):
        from ultralytics import YOLO
        self.model = YOLO(weights)
        self.device, self.imgsz, self.conf_min = device, imgsz, conf_min

    def detect(self, img_rgb: np.ndarray) -> list:
        """-> [(class, box conf, corners (4,2) px, corner conf (4,))]"""
        r = self.model.predict(np.ascontiguousarray(img_rgb[..., ::-1]), imgsz=self.imgsz, conf=self.conf_min,
                               device=self.device, verbose=False)[0]          # ultralytics takes BGR arrays
        if r.boxes is None or len(r.boxes) == 0:
            return []
        cls, cf = r.boxes.cls.int().tolist(), r.boxes.conf.tolist()
        xy = r.keypoints.xy.cpu().numpy()
        kc = r.keypoints.conf.cpu().numpy() if r.keypoints.conf is not None else np.ones(xy.shape[:2])
        return [(cls[j], cf[j], xy[j], kc[j]) for j in range(len(cls))]



def refine_corners(img: np.ndarray, kp: np.ndarray, cls: int, samples: int = 24, search_frac: float = 0.10,
                   search_min_px: float = 3.0, max_shift_frac: float = 0.12):
    """Detector corners (4,2) TL,TR,BR,BL -> sub-pixel corners from the inner edge of the coloured frame (the opening's
    boundary): along each side, the strongest rise of the frame colour going outward across the side within
    +-search_frac of its length, a line fitted to those edge points (trimmed least squares), adjacent lines intersected.
    A corner keeps the detector's position when either of its sides has too few edge points or it would move more than
    max_shift_frac of the shorter adjacent side. -> (corners (4,2), refined (4,) bool)"""
    import cv2
    f = img.astype(np.float32)
    o = [c for c in range(3) if c != cls % 3]
    sig = np.maximum(f[..., cls % 3] - 0.5 * (f[..., o[0]] + f[..., o[1]]), 0.0)   # frame colour (others and grey: 0)
    q = np.asarray(kp, dtype=np.float64)
    ctr = q.mean(0)
    lines, length = [], []
    for k in range(4):
        a, b = q[k], q[(k + 1) % 4]
        L = float(np.linalg.norm(b - a))
        length.append(L)
        if L < 8.0:
            lines.append(None)
            continue
        t = (b - a) / L
        n = np.array([-t[1], t[0]])
        n = n if np.dot(n, (a + b) / 2 - ctr) > 0 else -n                  # outward
        r = max(search_min_px, search_frac * L)
        ss = np.arange(-r, r + 0.25, 0.5)
        base = a + np.linspace(0.15, 0.85, samples)[:, None] * (b - a)
        pts = base[:, None, :] + ss[None, :, None] * n
        prof = cv2.remap(sig, (pts[..., 0] - 0.5).astype(np.float32), (pts[..., 1] - 0.5).astype(np.float32),
                         cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)   # pixel k spans [k, k+1) (labels, renders)
        d = (prof[:, 4:] - prof[:, :-4]) / 2.0                               # rise per px over 2 px, at ss[2:-2]
        j = d.argmax(1)
        jj = np.clip(j, 1, d.shape[1] - 2)
        y0, y1, y2 = (d[np.arange(samples), jj + m] for m in (-1, 0, 1))
        den = y0 - 2 * y1 + y2
        off = np.where(np.abs(den) > 1e-6, 0.5 * (y0 - y2) / np.where(np.abs(den) > 1e-6, den, 1.0), 0.0).clip(-0.5, 0.5)
        s_edge = ss[jj + 2] + 0.5 * off
        contrast = prof.max(1) - prof.min(1)
        ok = (j > 0) & (j < d.shape[1] - 1) & (contrast > 30.0) & (d.max(1) > 0.12 * contrast)
        e = base + s_edge[:, None] * n
        line = None
        for _ in range(4):                                                   # trimmed total least squares
            if ok.sum() < max(6, samples // 2):
                line = None
                break
            m0 = e[ok].mean(0)
            vt = np.linalg.svd(e[ok] - m0)[2]
            line = (m0, vt[0])
            res = np.abs((e - m0) @ vt[1])
            keep = ok & (res < max(1.0, 2.5 * float(np.median(res[ok]))))
            if (keep == ok).all():
                break
            ok = keep
        lines.append(line)
    out, refined = q.copy(), np.zeros(4, dtype=bool)
    for k in range(4):
        l1, l2 = lines[k - 1], lines[k]                                     # sides into and out of corner k
        if l1 is None or l2 is None:
            continue
        A = np.stack((l1[1], -l2[1]), 1)
        if abs(np.linalg.det(A)) < 1e-3:
            continue
        s = np.linalg.solve(A, l2[0] - l1[0])
        p = l1[0] + s[0] * l1[1]
        if np.linalg.norm(p - q[k]) <= max(2.0, max_shift_frac * min(length[k - 1], length[k])):
            out[k], refined[k] = p, True
    return out.astype(np.float32), refined


def project(pr, P: torch.Tensor, cam_c: torch.Tensor, R_WC: torch.Tensor):
    """World points (...,3) -> (pixels (...,2), depth (...)) with the surrogate's intrinsics (CV camera axes)."""
    pc = torch.einsum("ji,...j->...i", R_WC, P - cam_c)
    z = pc[..., 2]
    zs = z.clamp_min(1e-3)
    return torch.stack((pr.fx * pc[..., 0] / zs + pr.cx, pr.fy * pc[..., 1] / zs + pr.cy), -1), z


def visible(pr, scene, i: int, cam_c: torch.Tensor, R_WC: torch.Tensor):
    """(W,) windows of env i the surrogate would detect from this pose without noise or misses (the dataset's
    labelling rule, scripts/vision_dataset.py), and their true corner pixels (W,4,2)."""
    from rl_planner.core.scene import SceneTensors
    p = pr.cfg
    one = SceneTensors(**{k: getattr(scene, k)[i:i + 1] for k in SceneTensors.__dataclass_fields__})
    P = one.corners[0]
    uv, z = project(pr, P, cam_c, R_WC)
    mg = float(p["image_margin_px"])
    inside = ((uv[..., 0] >= mg) & (uv[..., 0] <= pr.width - mg) & (uv[..., 1] >= mg) & (uv[..., 1] <= pr.height - mg)).all(-1)
    to_cam = cam_c - one.center[0]
    facing = (to_cam / to_cam.norm(dim=-1, keepdim=True) * one.normal[0]).sum(-1) > pr.cos_max_inc
    in_range = to_cam.norm(dim=-1) < float(p["max_range"])
    vis = pr._unoccluded(cam_c[None], P[None], one)[0]
    side = (uv - uv.roll(-1, dims=1)).norm(dim=-1).min(-1).values
    ok = one.valid[0] & (z > 0.2).all(-1) & inside & facing & in_range & vis & (side >= float(p["min_side_px"]))
    return ok, uv


def associate(dets: list, uv_gt: torch.Tensor, z_gt: torch.Tensor, color: torch.Tensor, valid: torch.Tensor,
              gate_frac: float = 0.35, gate_min_px: float = 25.0):
    """Detections -> windows: same colour, the window in front of the camera, mean corner distance to its true
    projection under the gate (gate_frac of its projected side, at least gate_min_px). The identity is what the
    surrogate assumes known (core/perception.py: association by colour + order taken as correct); a detection that
    matches no window, or loses a window to a more confident one, counts as a false detection.
    -> (corners (W,4,2), det (W,), conf (W,), false detections)"""
    W = uv_gt.shape[0]
    uv, det, conf = torch.zeros(W, 4, 2), torch.zeros(W, dtype=torch.bool), torch.zeros(W)
    size = (uv_gt - uv_gt.roll(-1, dims=1)).norm(dim=-1).mean(-1)
    front = (z_gt > 0.2).all(-1)
    false = 0
    for cls, cf, kp, _ in sorted(dets, key=lambda d: -d[1]):
        k = torch.as_tensor(kp, dtype=torch.float32)
        d = (uv_gt - k[None]).norm(dim=-1).mean(-1)
        ok = valid & front & ((color % 3) == cls) & ~det & (d < (size * gate_frac).clamp_min(gate_min_px))
        if not bool(ok.any()):
            false += 1
            continue
        w = int(torch.where(ok, d, torch.full_like(d, float("inf"))).argmin())
        uv[w], det[w], conf[w] = k, True, float(cf)
    return uv, det, conf, false
