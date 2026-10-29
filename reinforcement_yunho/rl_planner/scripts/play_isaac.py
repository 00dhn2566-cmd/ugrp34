"""Render one evaluation episode in Isaac Sim (RTX) to a GIF: chase camera + fixed side camera.

    python rl_planner/scripts/play_isaac.py --policy <run>/export --level 3 --seed 5 --out flight.gif
    python rl_planner/scripts/play_isaac.py --planner fsm --level 3 --seed 5 --out fsm.gif
    python rl_planner/scripts/play_isaac.py --policy <export> --config configs/slung/base.yaml --config configs/slung/long.yaml \
        --level 1 --reseed 0 --safe --layer --substeps 4 --frame-ms 80 --view2 profile --out why.gif   # 25 fps, 0.5x

Windows are spawned as visual USD prims (coloured frame bars + translucent grey wall)
from the episode's sampled scene; physics/referee are exactly the training env's. The drone is
drawn as the real airframe (core/airframe.py: FX450, red front arms, the camera at the front)
whatever the referee's shape (drone.shape: sphere | airframe), and with configs/slung/base.yaml the load
on its cable (a box the size of the referee's ball; the header shows the swing).
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
    ap.add_argument("--substeps", type=int, default=0,
                    help="render every N physics steps instead (inside the decision, up to the step that crashes)")
    ap.add_argument("--frame-ms", type=int, default=None, help="GIF frame duration (default: real time)")
    ap.add_argument("--reseed", type=int, default=None,
                    help="core.reseed(N) before the episode: the scene and noise of seed N as the torch env draws them")
    ap.add_argument("--config", action="append", default=[], help="extra yaml merged over the config (e.g. tuned fsm)")
    ap.add_argument("--set", action="append", default=[], help="dotted override, e.g. safety.slow_after_s=2.0")
    ap.add_argument("--safe", action="store_true", help="run the planner behind the safety filter (planners/safety.py)")
    ap.add_argument("--layer", action="store_true", help="and behind the slung-load convex layer (planners/convex_layer.py)")
    ap.add_argument("--title", default=None, help="label prefix in the frame header (default: planner name)")
    ap.add_argument("--view2", choices=("auto", "side", "top", "profile"), default="auto",
                    help="right-hand camera: fixed side view of the course, top-down following the drone (auto: top "
                         "for scene.layout scatter), or profile: square to the course at window height, following the drone")
    ap.add_argument("--max-seconds", type=float, default=None, help="stop rendering after this much flight time")
    a = ap.parse_args(argv)

    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, enable_cameras=True).app

    import numpy as np
    import omni.replicator.core as rep
    import omni.usd
    import torch
    from PIL import Image, ImageDraw
    from pxr import Gf, UsdGeom, UsdShade, Sdf

    from rl_planner.core import math3d as m3
    from rl_planner.core.airframe import Airframe
    from rl_planner.core.config import config_for_policy, load_config
    from rl_planner.core.mission import CRASH_NAMES
    from rl_planner.envs.isaaclab_env import WindowMissionIsaacEnv, make_env_cfg
    from rl_planner.planners.fsm import FSMPlanner, STATE_NAMES
    from rl_planner.planners.safety import SafePlanner

    if a.policy:                        # same evaluation config as evaluate.py / benchmark.py (training-only DR off)
        with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
            cfg = config_for_policy(json.load(f)["config"], a.config, a.set)
    else:
        cfg = load_config(a.config, a.set)
    cfg["seed"] = a.seed
    cfg["sim"]["num_envs"] = 1
    env = WindowMissionIsaacEnv(make_env_cfg(cfg, num_envs=1, device="cuda:0"))
    core = env.core
    core.fixed_level = a.level
    if a.reseed is not None:
        core.reseed(a.reseed)
    env._reset_idx(None)
    if a.planner == "rl":
        from rl_planner.planners.policy import PolicyPlanner
        planner = PolicyPlanner(a.policy, device=env.device, cfg=cfg)
    else:
        planner = FSMPlanner(cfg, 1, env.device)
    fsm = planner
    if a.safe:
        planner = SafePlanner(planner, cfg, 1, env.device, core.step_dt)
    safe_planner = planner
    if a.layer:
        from rl_planner.planners.convex_layer import SlungLayerPlanner
        planner = SlungLayerPlanner(planner, cfg, 1, env.device, core.step_dt)
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
    sc = core.scene
    n_win = int(sc.n_windows[0])
    pts = torch.cat((sc.center[0, :n_win], core.state.pos[:1])).cpu().numpy()
    lo, hi = pts[:, :2].min(0) - 4.0, pts[:, :2].max(0) + 4.0
    box("/World/Viz/floor", origin + np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, -0.05]), np.eye(3),
        (max(30.0, hi[0] - lo[0]), max(12.0, hi[1] - lo[1]), 0.1), floor_mat)
    panel = float(cfg["scene"]["wall_half_extent"])    # finite wall panel around each window (look-around courses)
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
        th = 0.02
        if panel < 100.0:                                # the referee's square panel of half-extent wall_half_extent
            side = panel - hw - bar
            for sgn, tag in ((-1, "wl"), (1, "wr")):
                box(f"/World/Viz/w{w}_{tag}", c + r * sgn * (hw + bar + side / 2), R, (side, 2 * panel, th), wall_mat)
            vs = panel - hh - bar
            for sgn, tag in ((-1, "wb"), (1, "wt")):
                box(f"/World/Viz/w{w}_{tag}", c + u * sgn * (hh + bar + vs / 2), R, (2 * (hw + bar), vs, th), wall_mat)
            continue
        # wall = 4 panels around the opening, floor to z_top, +-wall_half sideways (visual only)
        wall_half, z_top = 1.6, 2.8
        z_lo, z_hi = c[2] - hh - bar, c[2] + hh + bar
        side = wall_half - hw - bar
        base = np.array([c[0], c[1], 0.0])
        for sgn, tag in ((-1, "wl"), (1, "wr")):
            ctr = base + r * sgn * (hw + bar + side / 2) + np.array([0, 0, z_top / 2])
            box(f"/World/Viz/w{w}_{tag}", ctr, R, (side, z_top, th), wall_mat)
        box(f"/World/Viz/w{w}_wt", base + np.array([0, 0, (z_hi + z_top) / 2]), R, (2 * (hw + bar), z_top - z_hi, th), wall_mat)
        box(f"/World/Viz/w{w}_wb", base + np.array([0, 0, z_lo / 2]), R, (2 * (hw + bar), max(z_lo, 0.02), th), wall_mat)

    def cylinder(path, center, R, radius, length, mat):
        cyl = UsdGeom.Cylinder.Define(stage, path)
        cyl.CreateRadiusAttr(float(radius))
        cyl.CreateHeightAttr(float(length))
        cyl.CreateAxisAttr("Z")
        M = Gf.Matrix4d(1.0)
        M.SetRotateOnly(Gf.Matrix3d(*[float(v) for v in R.T.reshape(-1)]))
        M.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in center]))
        UsdGeom.Xformable(cyl).AddTransformOp().Set(M)
        UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(mat)

    # The cloned drone's pose lives in Fabric, so draw the airframe (core/airframe.py: the FX450's propellers,
    # arms, body, package and camera) as our own USD prims under one Xform, posed every frame (position + attitude).
    env_drone = stage.GetPrimAtPath("/World/envs/env_0/Drone")
    if env_drone.IsValid():
        UsdGeom.Imageable(env_drone).MakeInvisible()
    drone = UsdGeom.Xform.Define(stage, "/World/Viz/drone")
    drone_op = drone.AddTransformOp()
    for part in Airframe(cfg, "cpu").visual_parts():
        mat = material(f"/World/Looks/drone_{part['name']}", part["rgb"], part["opacity"])
        path, c, R = f"/World/Viz/drone/{part['name']}", np.array(part["center"]), np.array(part["R"])
        if part["kind"] == "box":
            box(path, c, R, part["size"], mat)
        else:
            cylinder(path, c, R, part["radius"], part["length"], mat)

    def frame_z(v):
        """Rotation whose z axis is v (unit)."""
        x = np.cross([0.0, 1.0, 0.0] if abs(v[1]) < 0.9 else [1.0, 0.0, 0.0], v)
        x /= np.linalg.norm(x)
        return np.stack((x, np.cross(v, x), v), 1)

    slung = core.slung                                    # the load on its cable (core/slung.py), when there is one
    if slung is not None:
        side = 2 * slung.radius / np.sqrt(3.0)            # the box whose half-diagonal is the referee's ball
        load_x = UsdGeom.Xform.Define(stage, "/World/Viz/load")
        load_op = load_x.AddTransformOp()
        box("/World/Viz/load/box", np.zeros(3), np.eye(3), (side, side, side),
            material("/World/Looks/load", (0.78, 0.58, 0.32)))
        cable = UsdGeom.Cylinder.Define(stage, "/World/Viz/cable")
        cable.CreateRadiusAttr(0.004)
        cable.CreateHeightAttr(1.0)
        cable.CreateAxisAttr("Z")
        UsdShade.MaterialBindingAPI(cable.GetPrim()).Bind(material("/World/Looks/cable", (0.05, 0.05, 0.05)))
        cable_op = UsdGeom.Xformable(cable).AddTransformOp()

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

    mid = sc.center[0, :n_win].mean(0).cpu().numpy() + origin
    scatter = cfg["scene"].get("layout", "line") == "scatter"
    top = a.view2 == "top" or (a.view2 == "auto" and scatter)
    z_win = float(sc.center[0, :n_win, 2].mean()) + origin[2]   # profile: square to the course at window height
    try:
        from PIL import ImageFont
        font = ImageFont.load_default(size=18)
    except Exception:  # noqa: BLE001
        font = None
    frames, trace = [], []

    def snap():
        """Pose the airframe, cable and load from the env state, render both cameras, add the frame."""
        p = core.state.pos[0].cpu().numpy() + origin
        Rb = m3.quat_to_rotmat(core.state.quat[:1])[0].cpu().numpy()
        M = Gf.Matrix4d(1.0)
        M.SetRotateOnly(Gf.Matrix3d(*[float(v) for v in Rb.T.reshape(-1)]))
        M.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in p]))
        drone_op.Set(M)
        swing = None
        if slung is not None:
            hk = slung.hook(core.state.pos[:1], core.state.quat[:1])[0][0].cpu().numpy() + origin
            lp = slung.pos[0].cpu().numpy() + origin
            seg = lp - hk
            ln = max(float(np.linalg.norm(seg)), 1e-6)
            Rc = frame_z(seg / ln)
            Mc = Gf.Matrix4d(1.0)
            Mc.SetRotateOnly(Gf.Matrix3d(*[float(v) for v in Rc.T.reshape(-1)]))
            Mc.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in (hk + lp) / 2]))
            cable_op.Set(Gf.Matrix4d().SetScale(Gf.Vec3d(1.0, 1.0, ln)) * Mc)
            Ml = Gf.Matrix4d(1.0)
            Ml.SetRotateOnly(Gf.Matrix3d(*[float(v) for v in Rc.T.reshape(-1)]))
            Ml.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in lp]))
            load_op.Set(Ml)
            swing = float(np.degrees(np.arctan2(np.linalg.norm(seg[:2]), -seg[2])))
        if scatter:                                       # behind the drone along its heading (it turns)
            yw = float(core.yaw[0])
            f, l = np.array([np.cos(yw), np.sin(yw), 0.0]), np.array([-np.sin(yw), np.cos(yw), 0.0])
            look_at(cams["chase"][0], p - 1.9 * f + 0.55 * l + np.array([0, 0, 0.8]), p + 1.5 * f + np.array([0, 0, -0.1]))
        else:
            look_at(cams["chase"][0], p + np.array([-2.2, -0.9, 0.9]), p + np.array([1.5, 0.0, -0.1]))
        if a.view2 == "profile":                          # 4.5 m to the side, following the drone along the course
            look_at(cams["side"][0], np.array([p[0] + 0.6, mid[1] - 4.5, z_win + 0.1]), np.array([p[0] + 0.6, mid[1], z_win]))
        elif top:                                         # top-down, following the drone
            look_at(cams["side"][0], p + np.array([-0.01, -0.01, 9.0]), p + np.array([0.0, 0.0, 0.0]))
        else:
            look_at(cams["side"][0], mid + np.array([-6.0, -7.5, 4.0]), mid + np.array([1.0, 0.0, 0.5]))
        app.update()                      # sim.render() is a no-op here (is_rendering False); app.update renders without stepping physics
        imgs = []
        for name in ("chase", "side"):
            d = cams[name][1].get_data()
            if d is not None and getattr(d, "size", 0):
                imgs.append(Image.fromarray(np.asarray(d)[..., :3].astype(np.uint8)))
        crashed = bool(core.mission.crashed[0])
        trace.append(dict(t=float(core.episode_time[0]), pos=(p - origin).tolist(), quat=core.state.quat[0].tolist(),
                          load=None if slung is None else slung.pos[0].tolist(), swing_deg=swing,
                          target=int(core.mission.target[0]), crashed=crashed))
        if len(imgs) != 2:
            return
        canvas = Image.new("RGB", (a.width * 2, a.height))
        canvas.paste(imgs[0], (0, 0))
        canvas.paste(imgs[1], (a.width, 0))
        dr = ImageDraw.Draw(canvas)
        label = f"{a.title or a.planner.upper()}  t={core.episode_time[0]:.2f}s  passed {int(core.mission.target[0])}/{n_win}"
        if swing is not None:
            label += f"  swing {swing:.0f} deg"
        if a.planner == "fsm":
            label += f"  state={STATE_NAMES[int(fsm.state[0])]}"
        if a.safe and bool(safe_planner.safety.holding[0]):
            label += "  [safety hold]"
        dr.rectangle((0, 0, 2 * a.width, 30), fill=(20, 20, 20))
        dr.text((8, 5), label, fill=(255, 255, 255), font=font)
        if crashed:
            msg = f"CRASH: {CRASH_NAMES[int(core.mission.crash_reason[0])]} (window {int(core.mission.target[0]) + 1})"
            dr.rectangle((0, a.height - 36, 2 * a.width, a.height), fill=(170, 20, 20))
            dr.text((8, a.height - 31), msg, fill=(255, 255, 255), font=font)
        frames.append(canvas)

    sub = {"n": 0, "on": True}
    if a.substeps:                                        # render inside the decision, every N physics steps
        orig_sub = core.after_substep

        def after_substep():
            orig_sub()
            sub["n"] += 1
            if sub["on"] and sub["n"] % a.substeps == 0:
                snap()
                if bool(core.mission.crashed[0] | core.mission.success[0]):
                    sub["on"] = False                     # the env resets after this decision
        core.after_substep = after_substep
    else:
        snap()
    for t in range(core.max_episode_length):
        if a.max_seconds is not None and float(core.episode_time[0]) > a.max_seconds:
            break
        act = planner.act(core)
        _, _, term, trunc, _ = env.step(act)
        if bool(term[0] | trunc[0]):
            break
        if not a.substeps and (t + 1) % a.every == 0:
            snap()
    rec = {k: (v[0].item() if hasattr(v[0], "item") else v[0]) for k, v in core.episode_record.items()}
    print("[play] episode:", json.dumps(rec))
    if frames:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        ms = a.frame_ms or (int(1000 * core.step_dt * a.substeps / int(cfg["sim"]["decimation"])) if a.substeps
                            else int(100 * a.every))
        hold = [frames[-1]] * max(1, 1500 // ms)
        frames[0].save(a.out, save_all=True, append_images=frames[1:] + hold, duration=ms, loop=0)
        with open(os.path.splitext(a.out)[0] + ".json", "w", encoding="utf-8") as f:
            json.dump(dict(record=rec, frames=trace), f)
        print(f"[play] wrote {a.out} ({len(frames)} frames, {ms} ms each)")
    env.close()
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
