"""Render one evaluation episode in Isaac Sim (RTX) to a GIF: chase camera + fixed side camera.

    python rl_planner/scripts/play_isaac.py --policy <run>/export --level 3 --seed 5 --out flight.gif
    python rl_planner/scripts/play_isaac.py --planner fsm --level 3 --seed 5 --out fsm.gif

Windows are spawned as visual USD prims (coloured frame bars + translucent grey wall)
from the episode's sampled scene; physics/referee are exactly the training env's.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

COLORS = [(0.9, 0.1, 0.1), (0.1, 0.75, 0.2), (0.1, 0.3, 0.95)]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", default=None)
    ap.add_argument("--planner", choices=("rl", "fsm"), default="rl")
    ap.add_argument("--level", type=int, default=3)
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--out", default="flight.gif")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--every", type=int, default=1, help="render every N decisions")
    ap.add_argument("--config", action="append", default=[], help="extra yaml merged over the config (e.g. tuned fsm)")
    a = ap.parse_args(argv)

    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, enable_cameras=True).app

    import numpy as np
    import omni.replicator.core as rep
    import omni.usd
    import torch
    from PIL import Image, ImageDraw
    from pxr import Gf, UsdGeom, UsdShade, Sdf

    from rl_planner.core.config import Cfg, load_config
    from rl_planner.envs.isaaclab_env import WindowMissionIsaacEnv, make_env_cfg
    from rl_planner.planners.fsm import FSMPlanner, STATE_NAMES

    if a.policy:
        with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
            cfg = Cfg(json.load(f)["config"])
    else:
        cfg = load_config()
    if a.config:
        import yaml
        from rl_planner.core.config import deep_merge
        merged = dict(cfg)
        for path in a.config:
            with open(path, encoding="utf-8") as f:
                merged = deep_merge(merged, yaml.safe_load(f) or {})
        cfg = Cfg(merged)
    cfg["seed"] = a.seed
    cfg["sim"]["num_envs"] = 1
    env = WindowMissionIsaacEnv(make_env_cfg(cfg, num_envs=1, device="cuda:0"))
    core = env.core
    core.fixed_level = a.level
    env._reset_idx(None)
    if a.planner == "rl":
        from rl_planner.planners.policy import PolicyPlanner
        planner = PolicyPlanner(a.policy, device=env.device, cfg=cfg)
    else:
        planner = FSMPlanner(cfg, 1, env.device)
    planner.reset(torch.arange(1, device=env.device))

    stage = omni.usd.get_context().get_stage()
    origin = env.scene.env_origins[0].cpu().numpy()

    def material(path, rgb, opacity=1.0):
        mat = UsdShade.Material.Define(stage, path)
        sh = UsdShade.Shader.Define(stage, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    def box(path, center, R, scale, mat):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.ClearXformOpOrder()
        M = Gf.Matrix4d(1.0)
        M.SetRotateOnly(Gf.Matrix3d(*[float(v) for v in R.T.reshape(-1)]))
        M.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in center]))
        xf.AddTransformOp().Set(Gf.Matrix4d().SetScale(Gf.Vec3d(*[float(s) for s in scale])) * M)
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)

    mats = [material(f"/World/Looks/win{i}", c) for i, c in enumerate(COLORS)]
    wall_mat = material("/World/Looks/wall", (0.55, 0.55, 0.58), 0.35)
    floor_mat = material("/World/Looks/floor", (0.35, 0.33, 0.3))
    box("/World/Viz/floor", origin + np.array([8.0, 0.0, -0.05]), np.eye(3), (30.0, 12.0, 0.1), floor_mat)
    sc = core.scene
    bar = 0.06
    for w in range(int(sc.n_windows[0])):
        c = sc.center[0, w].cpu().numpy() + origin
        r, u, n = (sc.right[0, w].cpu().numpy(), sc.up[0, w].cpu().numpy(), sc.normal[0, w].cpu().numpy())
        R = np.stack((r, u, n), 1)                       # columns: local x,y,z in world
        hw, hh = (float(v) / 2 for v in sc.size[0, w])
        m = mats[w % 3]
        box(f"/World/Viz/w{w}_top", c + u * (hh + bar / 2), R, (2 * hw + 2 * bar, bar, bar), m)
        box(f"/World/Viz/w{w}_bot", c - u * (hh + bar / 2), R, (2 * hw + 2 * bar, bar, bar), m)
        box(f"/World/Viz/w{w}_l", c - r * (hw + bar / 2), R, (bar, 2 * hh, bar), m)
        box(f"/World/Viz/w{w}_r", c + r * (hw + bar / 2), R, (bar, 2 * hh, bar), m)
        # wall = 4 panels around the opening, floor to z_top, +-wall_half sideways (visual only)
        wall_half, z_top, th = 1.6, 2.8, 0.02
        z_lo, z_hi = c[2] - hh - bar, c[2] + hh + bar
        side = wall_half - hw - bar
        base = np.array([c[0], c[1], 0.0])
        for sgn, tag in ((-1, "wl"), (1, "wr")):
            ctr = base + r * sgn * (hw + bar + side / 2) + np.array([0, 0, z_top / 2])
            box(f"/World/Viz/w{w}_{tag}", ctr, R, (side, z_top, th), wall_mat)
        box(f"/World/Viz/w{w}_wt", base + np.array([0, 0, (z_hi + z_top) / 2]), R, (2 * (hw + bar), z_top - z_hi, th), wall_mat)
        box(f"/World/Viz/w{w}_wb", base + np.array([0, 0, z_lo / 2]), R, (2 * (hw + bar), max(z_lo, 0.02), th), wall_mat)

    # The cloned drone's pose lives in Fabric, so draw the referee's collision sphere
    # (radius = drone.collision_radius) as our own USD marker, moved every frame.
    drone_mat = material("/World/Looks/drone", (1.0, 0.55, 0.1), 0.85)
    marker = UsdGeom.Sphere.Define(stage, "/World/Viz/drone")
    marker.CreateRadiusAttr(float(cfg["drone"]["collision_radius"]))
    UsdShade.MaterialBindingAPI(marker.GetPrim()).Bind(drone_mat)
    marker_op = UsdGeom.Xformable(marker).AddTranslateOp()

    cams = {}
    for name in ("chase", "side"):
        cam = UsdGeom.Camera.Define(stage, f"/World/Viz/cam_{name}")
        cam.CreateFocalLengthAttr(14.0 if name == "chase" else 18.0)
        cams[name] = cam
        rp = rep.create.render_product(cam.GetPath().pathString, (a.width, a.height))
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])
        cams[name] = (cam, ann)

    def look_at(cam, eye, target):
        eye, target = np.asarray(eye, float), np.asarray(target, float)
        f = target - eye
        f /= np.linalg.norm(f)
        right = np.cross(f, [0, 0, 1.0])
        right /= np.linalg.norm(right) + 1e-9
        up = np.cross(right, f)
        R = np.stack((right, up, -f), 1)                # USD camera looks along -Z
        M = Gf.Matrix4d(1.0)
        M.SetRotateOnly(Gf.Matrix3d(*[float(v) for v in R.T.reshape(-1)]))
        M.SetTranslateOnly(Gf.Vec3d(*eye))
        xf = UsdGeom.Xformable(cam)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(M)

    n_win = int(sc.n_windows[0])
    mid = sc.center[0, :n_win].mean(0).cpu().numpy() + origin
    frames = []
    for t in range(core.max_episode_length):
        p = core.state.pos[0].cpu().numpy() + origin
        marker_op.Set(Gf.Vec3d(*[float(v) for v in p]))
        look_at(cams["chase"][0], p + np.array([-2.2, -0.9, 0.9]), p + np.array([1.5, 0.0, -0.1]))
        look_at(cams["side"][0], mid + np.array([-6.0, -7.5, 4.0]), mid + np.array([1.0, 0.0, 0.5]))
        if t % a.every == 0:
            app.update()                      # sim.render() is a no-op here (is_rendering False); app.update renders without stepping physics
            imgs = []
            for name in ("chase", "side"):
                d = cams[name][1].get_data()
                if d is not None and getattr(d, "size", 0):
                    imgs.append(Image.fromarray(np.asarray(d)[..., :3].astype(np.uint8)))
            if len(imgs) == 2:
                canvas = Image.new("RGB", (a.width * 2, a.height))
                canvas.paste(imgs[0], (0, 0))
                canvas.paste(imgs[1], (a.width, 0))
                dr = ImageDraw.Draw(canvas)
                try:
                    from PIL import ImageFont
                    font = ImageFont.load_default(size=18)
                except Exception:  # noqa: BLE001
                    font = None
                label = f"{a.planner.upper()}  t={core.episode_time[0]:.1f}s  passed {int(core.mission.target[0])}/{n_win}"
                if a.planner == "fsm":
                    label += f"  state={STATE_NAMES[int(planner.state[0])]}"
                dr.rectangle((0, 0, 2 * a.width, 30), fill=(20, 20, 20))
                dr.text((8, 5), label, fill=(255, 255, 255), font=font)
                frames.append(canvas)
        act = planner.act(core)
        _, _, term, trunc, _ = env.step(act)
        if bool(term[0] | trunc[0]):
            break
    rec = {k: (v[0].item() if hasattr(v[0], "item") else v[0]) for k, v in core.episode_record.items()}
    print("[play] episode:", json.dumps(rec))
    if frames:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        frames[0].save(a.out, save_all=True, append_images=frames[1:], duration=int(100 * a.every), loop=0)
        print(f"[play] wrote {a.out} ({len(frames)} frames)")
    env.close()
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
