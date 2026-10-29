"""YOLO-pose dataset of the onboard camera in Isaac Sim (RESULTS.md §16, rl_planner/vision).

Random courses drawn from the curriculum levels (the training env's own scene sampler), camera poses around them
(closing in on a window, just through one and looking for the next, anywhere in the course volume), the slung load on
its cable at a random swing, rendered by rl_planner/vision/isaac_view.py with randomised light and wall / floor shades.
Labels need no annotation: the true corners projected with the same intrinsics. A window is labelled only where the
perception surrogate would detect it (core/perception.py): all four corners in frame with the image margin, facing the
camera within max_incidence_deg, within max_range, every corner ray through the openings of the walls in between, the
shortest side at least min_side_px. Split by course (validation = the last --val-frac of the courses).

Format (ultralytics pose): <class> <cx> <cy> <w> <h> then (u, v, 2) for TL, TR, BR, BL, normalised; class = frame colour
(0 red, 1 green, 2 blue = course order % 3); the box is the outer edge of the frame.

    python rl_planner/scripts/vision_dataset.py --out /data/datasets/synthetic/windows_v1 --scenes 420 --poses 10
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

NAMES = {0: "red", 1: "green", 2: "blue"}


def rot_ypr(yaw: float, pitch: float, roll: float):
    import numpy as np
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    Rx = np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def sample_pose(rng, sc, i: int):
    """-> (body position (3,), R_WB (3,3)) for env i of SceneTensors sc."""
    import numpy as np
    n = int(sc.n_windows[i])
    C = sc.center[i, :n].numpy()
    N, Rt, U = sc.normal[i, :n].numpy(), sc.right[i, :n].numpy(), sc.up[i, :n].numpy()
    mode = rng.choice(3, p=[0.6, 0.2, 0.2])
    if mode == 0:                                    # closing in on a window
        w = int(rng.integers(n))
        d = math.exp(rng.uniform(math.log(0.6), math.log(9.0)))
        pos = C[w] + N[w] * d + Rt[w] * rng.uniform(-1.2, 1.2) + U[w] * rng.uniform(-0.6, 0.6)
        look = C[w] + rng.normal(0.0, 0.3, 3)
    elif mode == 1 and n > 1:                        # just through one, looking for the next
        w = int(rng.integers(n - 1))
        pos = C[w] - N[w] * rng.uniform(0.05, 1.5) + Rt[w] * rng.uniform(-0.4, 0.4) + U[w] * rng.uniform(-0.3, 0.3)
        look = C[w + 1] + rng.normal(0.0, 0.5, 3)
    else:                                            # anywhere in the course volume
        lo, hi = C.min(0) - np.array([3.0, 2.0, 0.0]), C.max(0) + np.array([1.0, 2.0, 0.0])
        pos = np.array([rng.uniform(lo[0], hi[0]), rng.uniform(lo[1], hi[1]), rng.uniform(0.4, 2.8)])
        a = rng.uniform(-math.radians(80), math.radians(80))
        look = pos + np.array([math.cos(a), math.sin(a), rng.uniform(-0.3, 0.3)])
    pos[2] = float(np.clip(pos[2], 0.25, 3.3))
    f = look - pos
    yaw = math.atan2(f[1], f[0]) + rng.normal(0.0, math.radians(10))
    el = math.atan2(f[2], math.hypot(f[0], f[1]))
    pitch = float(np.clip(-el + rng.normal(0.0, math.radians(10)), -math.radians(40), math.radians(40)))
    roll = float(np.clip(rng.normal(0.0, math.radians(8)), -math.radians(30), math.radians(30)))
    return pos, rot_ypr(yaw, pitch, roll)


def labels(pr, sc, i: int, cam_c, R_WC, bar: float):
    """Label lines for env i seen from (cam_c, R_WC) (torch, CPU) — the surrogate's detectability, no noise."""
    import torch

    from rl_planner.core.scene import SceneTensors
    p = pr.cfg
    one = SceneTensors(**{k: getattr(sc, k)[i:i + 1] for k in SceneTensors.__dataclass_fields__})
    P = one.corners[0]                                                    # (W,4,3)
    pc = torch.einsum("ji,wkj->wki", R_WC, P - cam_c)
    z = pc[..., 2]
    zs = z.clamp_min(1e-3)
    u, v = pr.fx * pc[..., 0] / zs + pr.cx, pr.fy * pc[..., 1] / zs + pr.cy
    mg = float(p["image_margin_px"])
    inside = ((u >= mg) & (u <= pr.width - mg) & (v >= mg) & (v <= pr.height - mg)).all(-1)
    to_cam = cam_c - one.center[0]
    facing = (to_cam / to_cam.norm(dim=-1, keepdim=True) * one.normal[0]).sum(-1) > pr.cos_max_inc
    in_range = to_cam.norm(dim=-1) < float(p["max_range"])
    vis = pr._unoccluded(cam_c[None], P[None], one)[0]
    uv = torch.stack((u, v), -1)
    side = (uv - uv.roll(-1, dims=1)).norm(dim=-1).min(-1).values
    ok = one.valid[0] & (z > 0.2).all(-1) & inside & facing & in_range & vis & (side >= float(p["min_side_px"]))
    out = []
    for w in torch.nonzero(ok).flatten().tolist():
        r, up = one.right[0, w], one.up[0, w]
        outer = torch.stack([P[w, 0] - r * bar + up * bar, P[w, 1] + r * bar + up * bar,
                             P[w, 2] + r * bar - up * bar, P[w, 3] - r * bar - up * bar])
        po = (outer - cam_c) @ R_WC
        ou = (pr.fx * po[:, 0] / po[:, 2].clamp_min(1e-3) + pr.cx).clamp(0, pr.width)
        ov = (pr.fy * po[:, 1] / po[:, 2].clamp_min(1e-3) + pr.cy).clamp(0, pr.height)
        x0, x1, y0, y1 = float(ou.min()), float(ou.max()), float(ov.min()), float(ov.max())
        kp = " ".join(f"{float(u[w, k]) / pr.width:.6f} {float(v[w, k]) / pr.height:.6f} 2" for k in range(4))
        out.append(f"{int(one.color[0, w]) % 3} {(x0 + x1) / 2 / pr.width:.6f} {(y0 + y1) / 2 / pr.height:.6f} "
                   f"{(x1 - x0) / pr.width:.6f} {(y1 - y0) / pr.height:.6f} {kp}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--config", action="append", default=None,
                    help="default: train/general.yaml (level 6), slung/base.yaml, slung/long.yaml (the 1 m load)")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--levels", default="0,1,2,3,4,5,6")
    ap.add_argument("--scenes", type=int, default=420, help="courses in all")
    ap.add_argument("--poses", type=int, default=10, help="images per course")
    ap.add_argument("--val-frac", type=float, default=0.1)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(argv)

    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, enable_cameras=True).app
    import numpy as np
    import torch
    from PIL import Image

    from rl_planner.core.config import load_config
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.vision.isaac_view import BAR, OnboardView

    C = os.path.join(ROOT, "rl_planner", "configs")
    configs = a.config or [os.path.join(C, "train", "general.yaml"), os.path.join(C, "slung", "base.yaml"),
                           os.path.join(C, "slung", "long.yaml")]
    cfg = load_config(configs, a.set + ["device=cpu"])
    levels = [int(x) for x in a.levels.split(",")]
    per = 32
    env = TorchWindowEnv(cfg, num_envs=per, device="cpu", seed=a.seed)
    core, pr = env.core, env.core.perception
    view = OnboardView(app, cfg, randomize=True, seed=a.seed, updates=1)
    sl = core.slung
    rng = np.random.default_rng(a.seed)
    for split in ("train", "val"):
        os.makedirs(os.path.join(a.out, "images", split), exist_ok=True)
        os.makedirs(os.path.join(a.out, "labels", split), exist_ok=True)
    n_val = int(round(a.scenes * a.val_frac))
    t0, n_img, n_lab, s = time.time(), 0, 0, 0
    while s < a.scenes:
        core.fixed_level = levels[(s // per) % len(levels)]
        env._reset_all()
        sc = core.scene
        for i in range(per):
            if s >= a.scenes:
                break
            split = "val" if s >= a.scenes - n_val else "train"
            view.set_course(sc, i, load_radius=None if sl is None else sl.radius)
            for k in range(a.poses):
                pos, R_WB = sample_pose(rng, sc, i)
                R_WC = R_WB @ np.asarray(pr.R_BC.cpu())
                cam_c = pos + R_WB @ np.asarray(pr.cam_offset_b.cpu())
                if sl is not None:
                    hook = pos + R_WB @ np.asarray(sl.attach.cpu() if hasattr(sl.attach, "cpu") else sl.attach)
                    th = abs(rng.normal(0.0, math.radians(30))) if rng.random() > 0.15 else rng.uniform(math.radians(60),
                                                                                                    math.radians(110))
                    ph = math.atan2(R_WB[1, 0], R_WB[0, 0]) + (rng.normal(0.0, 0.4) if rng.random() < 0.5 else rng.uniform(-math.pi, math.pi))
                    view.set_load(hook + sl.L * np.array([math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), -math.cos(th)]), hook)
                img = view.render(cam_c, R_WC)
                lab = labels(pr, sc, i, torch.tensor(cam_c, dtype=torch.float32), torch.tensor(R_WC, dtype=torch.float32), BAR)
                name = f"s{a.seed}_{s:05d}_{k:02d}"
                Image.fromarray(img).save(os.path.join(a.out, "images", split, name + ".jpg"), quality=93)
                with open(os.path.join(a.out, "labels", split, name + ".txt"), "w", encoding="utf-8") as f:
                    f.write("\n".join(lab) + ("\n" if lab else ""))
                n_img += 1
                n_lab += len(lab)
            s += 1
            if s % 20 == 0:
                print(f"[vision_dataset] {s}/{a.scenes} courses, {n_img} images, {n_lab} labels [{time.time() - t0:.0f}s]",
                      flush=True)
    with open(os.path.join(a.out, "data.yaml"), "w", encoding="utf-8") as f:
        f.write(f"path: {os.path.abspath(a.out)}\ntrain: images/train\nval: images/val\nkpt_shape: [4, 3]\n"
                f"flip_idx: [1, 0, 3, 2]\nnames:\n" + "".join(f"  {k}: {v}\n" for k, v in NAMES.items()))
    print(f"[vision_dataset] done: {n_img} images, {n_lab} labels [{time.time() - t0:.0f}s]")
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
