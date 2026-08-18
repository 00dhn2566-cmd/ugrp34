"""비행 GIF — **고정 카메라에서 드론이 움직이는 것**을 보여준다.

    bash scripts/run_gif.sh                     # 25초, 고정 3인칭 시점
    python make_gif.py --view both              # 좌: 고정 시점 / 우: 드론 시점 + 검출
    python make_gif.py --view pov               # 드론 시점만

--seconds 는 **GIF 총 재생 시간**이다. 프레임 수에 맞춰 프레임 간격을 역산한다.

주의: 관측 경로(poses)는 원래 "가상 카메라를 그 자리에 놓고 렌더"하는 용도라
드론 본체는 원점에 그대로 있다. 3인칭으로 보여주려면 프레임마다 드론 바디를
그 pose 로 옮겨줘야 한다 (resetBasePositionAndOrientation). 물리 스텝을 안 돌리므로
옮긴 자리에 그대로 있는다.

CF2X 는 27 g 크레이지플라이(팔 길이 4 cm)라 복도 스케일에서 점처럼 보인다. 그래서
지나온 궤적과 현재 위치를 **월드 좌표를 직접 투영해서** 2D 로 덧그린다 (pybullet
디버그 라인은 getCameraImage 에 안 찍힌다).
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

from module import contract  # noqa: E402
from utils import device, scene  # noqa: E402

RGB = {"red": (235, 60, 60), "green": (40, 200, 60), "blue": (60, 130, 235)}
TRAIL = (255, 210, 60)


def project(pts_w, view, proj, W, H):
    """월드 점 -> 화면 픽셀. 카메라 뒤면 None. view/proj 는 pybullet 의 열우선 16개."""
    V = np.asarray(view, float).reshape(4, 4).T
    P = np.asarray(proj, float).reshape(4, 4).T
    out = []
    for q in np.atleast_2d(pts_w):
        clip = P @ V @ np.array([q[0], q[1], q[2], 1.0])
        if clip[3] <= 1e-6:
            out.append(None)
            continue
        ndc = clip[:3] / clip[3]
        out.append(((ndc[0] * 0.5 + 0.5) * W, (1.0 - (ndc[1] * 0.5 + 0.5)) * H))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=None)
    ap.add_argument("--out", default=os.path.join(paths.FIG_DIR, "v2_flight.gif"))
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--seconds", type=float, default=25.0, help="GIF 총 재생 시간 [s]")
    ap.add_argument("--frames-per-window", type=int, default=48)
    ap.add_argument("--mode", default="xy", choices=scene.PATH_MODES)
    ap.add_argument("--span", type=float, default=110.0)
    ap.add_argument("--clutter", type=int, default=18)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--view", default="fixed", choices=("fixed", "pov", "both"))
    # 고정 카메라 (씬 중심을 바라보는 한 자리에 계속 머문다)
    # 이 값들은 눈으로 확인해서 고른 것이다. 더 멀리(6.0+) 빼면 카메라가 방 벽
    # 바깥으로 나가서 화면이 벽으로 덮인다.
    ap.add_argument("--cam-dist", type=float, default=5.0)
    ap.add_argument("--cam-yaw", type=float, default=42.0)
    ap.add_argument("--cam-pitch", type=float, default=-22.0)
    ap.add_argument("--cam-fov", type=float, default=60.0)
    ap.add_argument("--bright", type=float, default=1.7,
                    help="고정 시점 밝기 보정 (학습 도메인 조명이 어두워서 눈으로 보기 힘듦). "
                         "보여주기용이고 검출기가 보는 드론 시점에는 안 먹인다.")
    ap.add_argument("--width", type=int, default=800, help="고정 시점 렌더 가로")
    ap.add_argument("--scale", type=float, default=0.5, help="드론 시점 렌더 축소")
    ap.add_argument("--gif-width", type=int, default=720, help="GIF 가로 픽셀 (용량)")
    ap.add_argument("--colors", type=int, default=96, help="GIF 팔레트 색 수 (용량)")
    ap.add_argument("--device", default="auto", choices=("auto", "cuda", "cpu"))
    a = ap.parse_args(argv)

    import pybullet as p
    from PIL import Image, ImageDraw
    from eval_recon3d import quat_xyzw_to_rot
    from sim import pybullet_stream as pbs

    need_det = a.view in ("pov", "both")
    intr = contract.intrinsics()
    det = device.load_detector(a.weights, prefer=a.device) if need_det else None
    env, layout = scene.make(seed=a.seed, clutter=a.clutter)
    scene.print_layout(layout)
    cid = env.CLIENT
    drone = env.DRONE_IDS[0]

    poses, path_name = scene.path(layout, mode=a.mode,
                                  n_per_window=a.frames_per_window, span_deg=a.span)
    n = len(poses)
    duration_ms = max(30, int(round(a.seconds * 1000 / n)))
    print(f"경로: {path_name}")
    print(f"시점: {a.view}  (고정 카메라 dist={a.cam_dist} yaw={a.cam_yaw} "
          f"pitch={a.cam_pitch})")
    print(f"{n} 프레임 x {duration_ms} ms = {n*duration_ms/1000:.1f} s GIF")

    # ---- 고정 카메라: 루프 밖에서 한 번만 만든다 (그래서 '고정') ----------------
    mid = np.mean([w["center"] for w in layout], axis=0)
    W, H = a.width, round(a.width * 9 / 16)
    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=mid.tolist(), distance=a.cam_dist, yaw=a.cam_yaw,
        pitch=a.cam_pitch, roll=0, upAxisIndex=2, physicsClientId=cid)
    proj = p.computeProjectionMatrixFOV(a.cam_fov, W / H, 0.05, 40, physicsClientId=cid)

    frames, n_det, trail = [], 0, []
    for i, (pos, q) in enumerate(poses):
        # 드론 본체를 이 pose 로 옮긴다 — 물리 스텝은 안 돈다
        p.resetBasePositionAndOrientation(drone, pos.tolist(),
                                          [float(v) for v in q], physicsClientId=cid)
        trail.append(np.asarray(pos, float))

        panels = []
        if a.view in ("fixed", "both"):
            _, _, rgb, _, _ = p.getCameraImage(W, H, view, proj, shadow=1,
                                               renderer=p.ER_TINY_RENDERER,
                                               physicsClientId=cid)
            im = Image.fromarray(np.asarray(rgb, np.uint8).reshape(H, W, 4)[:, :, :3])
            if a.bright and a.bright != 1.0:
                from PIL import ImageEnhance
                im = ImageEnhance.Brightness(im).enhance(a.bright)
            dr = ImageDraw.Draw(im)
            # 지나온 궤적 + 현재 위치를 직접 투영해서 덧그린다
            pts = [q_ for q_ in project(np.array(trail), view, proj, W, H) if q_]
            if len(pts) > 1:
                dr.line(pts, fill=TRAIL, width=2)
            if pts:
                u, v = pts[-1]
                dr.ellipse([u-9, v-9, u+9, v+9], outline=(255, 255, 255), width=3)
                dr.ellipse([u-4, v-4, u+4, v+4], fill=TRAIL)
            dr.text((8, 8), f"fixed camera   frame {i+1}/{n}   "
                            f"drone ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
                    fill=(255, 255, 255))
            panels.append(im)

        if need_det:
            R = quat_xyzw_to_rot(q)
            img = pbs.render_frame(p, cid, pos, pos + R[:, 2], intr, scale=a.scale)
            im2 = Image.fromarray(img)
            dr2 = ImageDraw.Draw(im2)
            res = det.predict(img[:, :, ::-1], conf=a.conf, agnostic_nms=True)[0]
            if res.boxes is not None and len(res.boxes):
                for box, kp in zip(res.boxes, res.keypoints.xy.cpu().numpy()):
                    name = det.names[int(box.cls)]
                    c = RGB.get(name, (255, 255, 255))
                    pl = [tuple(map(float, t)) for t in kp]
                    dr2.line(pl + [pl[0]], fill=c, width=3)
                    for u, v in pl:
                        dr2.ellipse([u-3, v-3, u+3, v+3], fill=(255, 255, 255),
                                    outline=(0, 0, 0))
                    dr2.text((min(u for u, _ in pl), min(v for _, v in pl) - 12),
                             f"{name} {float(box.conf):.2f}", fill=c)
                    n_det += 1
            dr2.text((8, 8), "drone view + detections", fill=(255, 255, 255))
            if a.view == "pov":
                panels = [im2]
            else:
                panels.append(im2)

        if len(panels) == 1:
            frames.append(panels[0])
        else:
            h = max(x.height for x in panels)
            scaled = [x.resize((round(x.width * h / x.height), h), Image.BILINEAR)
                      for x in panels]
            comp = Image.new("RGB", (sum(x.width for x in scaled), h), (20, 20, 20))
            ox = 0
            for x in scaled:
                comp.paste(x, (ox, 0)); ox += x.width
            frames.append(comp)

        if (i + 1) % 24 == 0:
            print(f"  {i+1}/{n}" + (f"  누적 검출 {n_det}" if need_det else ""))

    env.close()

    # 용량: 가로 축소 + 공용 팔레트 양자화. 프레임마다 팔레트를 따로 만들면
    # GIF 가 프레임당 팔레트를 싣느라 오히려 커진다.
    if a.gif_width and frames[0].width > a.gif_width:
        h = round(frames[0].height * a.gif_width / frames[0].width)
        frames = [f.resize((a.gif_width, h), Image.LANCZOS) for f in frames]
    pal = frames[0].quantize(colors=a.colors, method=Image.MEDIANCUT)
    frames = [f.quantize(palette=pal, dither=Image.FLOYDSTEINBERG) for f in frames]

    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    frames[0].save(a.out, save_all=True, append_images=frames[1:],
                   duration=duration_ms, loop=0, optimize=True)
    print(f"\nwrote {a.out}  ({n} frames, {n*duration_ms/1000:.1f} s, "
          f"{os.path.getsize(a.out)/2**20:.1f} MB"
          + (f", 검출 {n_det}" if need_det else "") + ")")
    print("GIF_DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
