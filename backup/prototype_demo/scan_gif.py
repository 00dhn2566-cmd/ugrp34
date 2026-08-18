"""스캔 모션 GIF — 고정 시점(화각 프러스텀 표시) + 드론 시점(검출).

    bash scripts/run_scan_gif.sh
    python scan_gif.py --seconds 20 --window 0

왼쪽: 한 자리에 고정된 카메라. 드론 위치·궤적과 **카메라 화각 프러스텀**을 3D 로
      그려 넣는다 (월드 좌표를 직접 투영). 창문이 화각 안에 들어와 있는지가 보인다.
오른쪽: 그 순간 드론 카메라가 실제로 본 그림 + 검출 오버레이.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from utils import paths  # noqa: E402

paths.bootstrap()

from make_gif import project  # noqa: E402
from module import contract  # noqa: E402
from traj_manager import Config, scan_waypoints  # noqa: E402
from utils import device, scene  # noqa: E402

RGB = {"red": (235, 60, 60), "green": (40, 200, 60), "blue": (60, 130, 235)}
TRAIL = (255, 210, 60)
FRUSTUM = (255, 255, 255)


def cam_axes(yaw: float):
    """수평 카메라의 (forward, right, up). 이 리포 규약: image-right = world −y."""
    f = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return f, r, u


def frustum_pts(pos, yaw, half_h_deg, half_v_deg, dist):
    """화각 4모서리 방향의 끝점 (4,3). 순서 TL,TR,BR,BL (화면 기준)."""
    f, r, u = cam_axes(yaw)
    th, tv = np.tan(np.radians(half_h_deg)), np.tan(np.radians(half_v_deg))
    return np.array([pos + dist * (f - th * r + tv * u),
                     pos + dist * (f + th * r + tv * u),
                     pos + dist * (f + th * r - tv * u),
                     pos + dist * (f - th * r - tv * u)])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--weights", default=None)
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--n-windows", type=int, default=3)
    ap.add_argument("--spacing", type=float, default=2.6)
    ap.add_argument("--clutter", type=int, default=10)
    ap.add_argument("--window", type=int, default=0, help="어느 창문을 스캔할지")
    ap.add_argument("--scan-radius", type=float, default=2.0)
    ap.add_argument("--scan-span", type=float, default=50.0)
    ap.add_argument("--n-per-motion", type=int, default=24)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--frustum-len", type=float, default=3.0)
    ap.add_argument("--cam-dist", type=float, default=5.5)
    ap.add_argument("--cam-yaw", type=float, default=42.0)
    ap.add_argument("--cam-pitch", type=float, default=-22.0)
    ap.add_argument("--bright", type=float, default=1.7)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--gif-width", type=int, default=900)
    ap.add_argument("--colors", type=int, default=112)
    ap.add_argument("--device", default="auto", choices=("auto", "cuda", "cpu"))
    ap.add_argument("--out", default=os.path.join(paths.FIG_DIR, "v2_11_scan.gif"))
    a = ap.parse_args(argv)

    import pybullet as p
    from PIL import Image, ImageDraw, ImageEnhance
    from sim import pybullet_stream as pbs

    intr = contract.intrinsics()
    half_h = float(np.degrees(np.arctan(intr["width"] / 2 / intr["fx"])))
    half_v = float(np.degrees(np.arctan(intr["height"] / 2 / intr["fy"])))
    print(f"카메라 반화각  가로 {half_h:.1f}°  세로 {half_v:.1f}°")

    det = device.load_detector(a.weights, prefer=a.device)
    env, layout = scene.make(seed=a.seed, n_windows=a.n_windows,
                             clutter=a.clutter, spacing=a.spacing)
    cid = env.CLIENT
    scene.print_layout(layout)
    p.resetBasePositionAndOrientation(env.DRONE_IDS[0], [0, 0, -50], [0, 0, 0, 1],
                                      physicsClientId=cid)

    cfg = Config(scan_radius=a.scan_radius, scan_span_deg=a.scan_span)
    w = layout[a.window]
    c = np.asarray(w["center"], float)
    A = c - np.array([cfg.scan_radius, 0.0, 0.0])
    A[2] = 1.0
    print(f"창문{a.window} center {np.round(c,2)}   앵커 {np.round(A,2)}")

    poses = []
    for kind in cfg.scan_kinds:
        pts, yaws = scan_waypoints(kind, A, c, cfg, n=a.n_per_motion)
        poses += [(kind, q, float(y)) for q, y in zip(pts, yaws)]

    W, H = 900, 506
    mid = np.mean([q["center"] for q in layout], axis=0)
    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=mid.tolist(), distance=a.cam_dist, yaw=a.cam_yaw,
        pitch=a.cam_pitch, roll=0, upAxisIndex=2, physicsClientId=cid)
    proj = p.computeProjectionMatrixFOV(60, W / H, 0.05, 40, physicsClientId=cid)

    n = len(poses)
    dur = max(30, int(round(a.seconds * 1000 / n)))
    print(f"{n} 프레임 x {dur} ms = {n*dur/1000:.1f} s")

    frames, trail, n_det = [], [], 0
    for i, (kind, q, yaw) in enumerate(poses):
        trail.append(q)
        # ---- 왼쪽: 고정 시점 + 프러스텀 ----------------------------------
        _, _, rgb, _, _ = p.getCameraImage(W, H, view, proj, shadow=1,
                                           renderer=p.ER_TINY_RENDERER,
                                           physicsClientId=cid)
        im = Image.fromarray(np.asarray(rgb, np.uint8).reshape(H, W, 4)[:, :, :3])
        if a.bright != 1.0:
            im = ImageEnhance.Brightness(im).enhance(a.bright)
        dr = ImageDraw.Draw(im)
        tr = [z for z in project(np.array(trail), view, proj, W, H) if z]
        if len(tr) > 1:
            dr.line(tr, fill=TRAIL, width=2)
        # 프러스텀: 카메라에서 4모서리로 뻗는 선 + 끝 사각형
        fp = frustum_pts(q, yaw, half_h, half_v, a.frustum_len)
        s0 = project(np.array([q]), view, proj, W, H)[0]
        se = project(fp, view, proj, W, H)
        if s0 and all(se):
            for e in se:
                dr.line([s0, e], fill=FRUSTUM, width=1)
            dr.line(list(se) + [se[0]], fill=FRUSTUM, width=2)
            dr.ellipse([s0[0]-7, s0[1]-7, s0[0]+7, s0[1]+7],
                       outline=(255, 255, 255), width=3)
        dr.text((8, 8), f"fixed camera   scan={kind}   frame {i+1}/{n}", fill=(255,)*3)
        dr.text((8, 24), f"drone ({q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f})  "
                         f"yaw {np.degrees(yaw):+.0f} deg", fill=(255,)*3)
        dr.text((8, 40), f"FOV  h +/-{half_h:.0f} deg   v +/-{half_v:.0f} deg",
                fill=FRUSTUM)

        # ---- 오른쪽: 드론 시점 + 검출 ------------------------------------
        f, _, _ = cam_axes(yaw)
        img = pbs.render_frame(p, cid, q, q + f, intr, scale=a.scale)
        im2 = Image.fromarray(img)
        dr2 = ImageDraw.Draw(im2)
        res = det.predict(img[:, :, ::-1], conf=0.25, agnostic_nms=True)[0]
        got = []
        if res.boxes is not None and len(res.boxes):
            for box, kp in zip(res.boxes, res.keypoints.xy.cpu().numpy()):
                name = det.names[int(box.cls)]
                col = RGB.get(name, (255,)*3)
                pl = [tuple(map(float, t)) for t in kp]
                dr2.line(pl + [pl[0]], fill=col, width=3)
                for u_, v_ in pl:
                    dr2.ellipse([u_-3, v_-3, u_+3, v_+3], fill=(255,)*3, outline=(0,)*3)
                cf = float(box.conf)
                got.append(f"{name} {cf:.2f}")
                dr2.text((min(x for x, _ in pl), min(y for _, y in pl) - 12),
                         f"{name} {cf:.2f}", fill=col)
                n_det += 1
        dr2.text((8, 8), "drone camera + detections", fill=(255,)*3)
        dr2.text((8, 24), ("  ".join(got) if got else "NO DETECTION"),
                 fill=(120, 255, 120) if got else (255, 90, 90))

        h = max(im.height, im2.height)
        sc = [x.resize((round(x.width * h / x.height), h), Image.BILINEAR)
              for x in (im, im2)]
        comp = Image.new("RGB", (sum(x.width for x in sc), h), (18, 18, 18))
        ox = 0
        for x in sc:
            comp.paste(x, (ox, 0)); ox += x.width
        frames.append(comp)
        if (i + 1) % 24 == 0:
            print(f"  {i+1}/{n}  누적 검출 {n_det}")

    env.close()
    if a.gif_width and frames[0].width > a.gif_width:
        hh = round(frames[0].height * a.gif_width / frames[0].width)
        frames = [x.resize((a.gif_width, hh), Image.LANCZOS) for x in frames]
    pal = frames[0].quantize(colors=a.colors, method=Image.MEDIANCUT)
    frames = [x.quantize(palette=pal, dither=Image.FLOYDSTEINBERG) for x in frames]
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    frames[0].save(a.out, save_all=True, append_images=frames[1:],
                   duration=dur, loop=0, optimize=True)
    print(f"\nwrote {a.out}  ({n} frames, {n*dur/1000:.1f} s, "
          f"{os.path.getsize(a.out)/2**20:.1f} MB, 검출 {n_det})")
    print("SCAN_GIF_DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
