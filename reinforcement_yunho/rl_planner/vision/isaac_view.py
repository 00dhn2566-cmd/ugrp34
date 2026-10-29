"""The onboard camera in Isaac Sim (RTX): one env's course as USD prims and a pinhole camera with the real intrinsics.

The course: coloured frames (red / green / blue = course order % 3, bar 6 cm) around each opening, set in OPAQUE walls
large enough to fill the view — the referee's walls are infinite planes and the perception surrogate's occlusion test
assumes nothing behind a wall is seen except through its opening — plus a floor, lights and, with a slung load, the
load on its cable. The camera (configs perception.camera: 1280x720, fx = fy = 763, centred principal point) is posed
at the camera centre and R_WC the env computes (core/perception.py camera_pose; CV axes: x right, y down, z forward).

Needs a running Kit app with cameras enabled: ``isaaclab.app.AppLauncher(headless=True, enable_cameras=True)``.
"""
from __future__ import annotations

import numpy as np

COLORS = [(0.9, 0.1, 0.1), (0.1, 0.75, 0.2), (0.1, 0.3, 0.95)]      # scene.color 0/1/2 (play_isaac.py)
BAR = 0.06                       # frame bar width / depth [m]
WALL_HALF, WALL_TOP, WALL_T = 12.0, 6.0, 0.02
APERTURE_H = 20.955              # [mm] USD default horizontal aperture; focal length follows from fx


def _mat4(R: np.ndarray, t, scale=None):
    from pxr import Gf
    M = Gf.Matrix4d(1.0)
    M.SetRotateOnly(Gf.Matrix3d(*[float(v) for v in np.asarray(R).T.reshape(-1)]))
    M.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in t]))
    if scale is not None:
        M = Gf.Matrix4d().SetScale(Gf.Vec3d(*[float(s) for s in scale])) * M
    return M


class OnboardView:
    def __init__(self, app, cfg, randomize: bool = False, seed: int = 0, updates: int = 2):
        import omni.replicator.core as rep
        import omni.usd
        from pxr import UsdGeom, UsdLux
        self.app, self.updates = app, updates
        self.rng = np.random.default_rng(seed)
        self.randomize = randomize
        cam = cfg["perception"]["camera"]
        self.W, self.H = int(cam["width"]), int(cam["height"])
        self.fx, self.fy, self.cx, self.cy = (float(cam[k]) for k in ("fx", "fy", "cx", "cy"))
        if abs(self.cx - self.W / 2) > 0.5 or abs(self.cy - self.H / 2) > 0.5 or abs(self.fx - self.fy) > 1e-3:
            raise ValueError("OnboardView renders a centred, square-pixel pinhole camera")
        self.stage = omni.usd.get_context().get_stage()
        st = self.stage
        self.dome = UsdLux.DomeLight.Define(st, "/World/Light/dome")
        self.sun = UsdLux.DistantLight.Define(st, "/World/Light/sun")
        self.sun_xf = UsdGeom.Xformable(self.sun).AddTransformOp()
        self.cam = UsdGeom.Camera.Define(st, "/World/OnboardCam")
        self.cam.CreateHorizontalApertureAttr(APERTURE_H)
        self.cam.CreateVerticalApertureAttr(APERTURE_H * self.H / self.W)
        self.cam.CreateFocalLengthAttr(self.fx * APERTURE_H / self.W)
        self.cam.CreateClippingRangeAttr().Set((0.02, 200.0))
        self.cam_xf = UsdGeom.Xformable(self.cam).AddTransformOp()
        rp = rep.create.render_product(self.cam.GetPath().pathString, (self.W, self.H))
        self.ann = rep.AnnotatorRegistry.get_annotator("rgb")
        self.ann.attach([rp])
        self._load = None
        self._lights()

    # ------------------------------------------------------------------ materials / prims
    def _material(self, path, rgb, rough=0.6):
        from pxr import Gf, Sdf, UsdShade
        mat = UsdShade.Material.Define(self.stage, path)
        sh = UsdShade.Shader.Define(self.stage, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(c) for c in rgb]))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    def _box(self, path, center, R, size, mat):
        from pxr import UsdGeom, UsdShade
        cube = UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        UsdGeom.Xformable(cube).AddTransformOp().Set(_mat4(R, center, size))
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)

    def _lights(self):
        j = self.rng.uniform if self.randomize else (lambda lo, hi: (lo + hi) / 2)
        self.dome.CreateIntensityAttr(float(j(150.0, 450.0)))
        self.sun.CreateIntensityAttr(float(j(400.0, 1400.0)))
        az, el = np.radians(j(0.0, 360.0)), np.radians(j(30.0, 70.0))
        d = -np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])   # light travel direction
        z = -d                                                                        # USD distant light shines along -Z
        x = np.cross([0.0, 0.0, 1.0], z)
        x = x / (np.linalg.norm(x) + 1e-9) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0.0, 0.0])
        self.sun_xf.Set(_mat4(np.stack((x, np.cross(z, x), z), 1), (0.0, 0.0, 0.0)))

    # ------------------------------------------------------------------ course
    def set_course(self, scene, i: int, load_radius: float | None = None) -> None:
        """Rebuild the course of env ``i`` (SceneTensors on any device)."""
        st = self.stage
        if st.GetPrimAtPath("/World/Course").IsValid():
            st.RemovePrim("/World/Course")
        if self.randomize:
            self._lights()
        j = self.rng.uniform if self.randomize else (lambda lo, hi: (lo + hi) / 2)
        g = j(0.45, 0.8)
        wall = self._material("/World/Course/Looks/wall", (g * j(0.92, 1.0), g * j(0.92, 1.0), g * j(0.92, 1.0)), 0.8)
        f = j(0.22, 0.45)
        floor = self._material("/World/Course/Looks/floor", (f * j(0.9, 1.1), f * j(0.85, 1.0), f * j(0.75, 0.95)), 0.9)
        wins = [self._material(f"/World/Course/Looks/win{k}", c, 0.5) for k, c in enumerate(COLORS)]
        n = int(scene.n_windows[i])
        ctr = scene.center[i, :n].cpu().numpy()
        lo, hi = ctr[:, :2].min(0) - 15.0, ctr[:, :2].max(0) + 15.0
        self._box("/World/Course/floor", ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, -0.05), np.eye(3),
                  (hi[0] - lo[0], hi[1] - lo[1], 0.1), floor)
        for w in range(n):
            c = scene.center[i, w].cpu().numpy()
            r, u, nn = (scene.right[i, w].cpu().numpy(), scene.up[i, w].cpu().numpy(), scene.normal[i, w].cpu().numpy())
            R = np.stack((r, u, nn), 1)                    # local x, y, z = right, up, normal
            hw, hh = (float(v) / 2 for v in scene.size[i, w])
            m = wins[int(scene.color[i, w]) % 3]
            p = f"/World/Course/w{w}"
            self._box(p + "_top", c + u * (hh + BAR / 2), R, (2 * hw + 2 * BAR, BAR, BAR), m)
            self._box(p + "_bot", c - u * (hh + BAR / 2), R, (2 * hw + 2 * BAR, BAR, BAR), m)
            self._box(p + "_l", c - r * (hw + BAR / 2), R, (BAR, 2 * hh, BAR), m)
            self._box(p + "_r", c + r * (hw + BAR / 2), R, (BAR, 2 * hh, BAR), m)
            side = WALL_HALF - hw - BAR                    # wall around the frame, floor to WALL_TOP
            for sgn, tag in ((-1, "wl"), (1, "wr")):
                ctr_w = c + r * sgn * (hw + BAR + side / 2) + u * (WALL_TOP / 2 - c[2])
                self._box(p + tag, ctr_w, R, (side, WALL_TOP, WALL_T), wall)
            top = WALL_TOP - (c[2] + hh + BAR)
            self._box(p + "wt", c + u * (hh + BAR + top / 2), R, (2 * (hw + BAR), top, WALL_T), wall)
            bot = max(c[2] - hh - BAR, 0.01)
            self._box(p + "wb", c - u * (hh + BAR + bot / 2), R, (2 * (hw + BAR), bot, WALL_T), wall)
        self._load = None
        if load_radius is not None:
            from pxr import UsdGeom, UsdShade
            sph = UsdGeom.Sphere.Define(st, "/World/Course/load")
            sph.CreateRadiusAttr(float(load_radius))
            UsdShade.MaterialBindingAPI(sph.GetPrim()).Bind(self._material("/World/Course/Looks/load", (0.78, 0.58, 0.32)))
            cab = UsdGeom.Cylinder.Define(st, "/World/Course/cable")
            cab.CreateRadiusAttr(0.004)
            cab.CreateHeightAttr(1.0)
            cab.CreateAxisAttr("Z")
            UsdShade.MaterialBindingAPI(cab.GetPrim()).Bind(self._material("/World/Course/Looks/cable", (0.05, 0.05, 0.05)))
            self._load = (UsdGeom.Xformable(sph).AddTransformOp(), UsdGeom.Xformable(cab).AddTransformOp())

    def set_load(self, load_pos, hook_pos) -> None:
        if self._load is None:
            return
        lp, hk = np.asarray(load_pos, float), np.asarray(hook_pos, float)
        seg = lp - hk
        ln = max(float(np.linalg.norm(seg)), 1e-6)
        z = seg / ln
        x = np.cross([0.0, 1.0, 0.0] if abs(z[1]) < 0.9 else [1.0, 0.0, 0.0], z)
        x /= np.linalg.norm(x)
        R = np.stack((x, np.cross(z, x), z), 1)
        self._load[0].Set(_mat4(np.eye(3), lp))
        self._load[1].Set(_mat4(R, (hk + lp) / 2, (1.0, 1.0, ln)))

    # ------------------------------------------------------------------ render
    def render(self, cam_c, R_WC) -> np.ndarray:
        """(H,W,3) uint8 from camera centre (3,) and R_WC (3,3) (CV axes; USD's camera looks along -Z, y up)."""
        R = np.asarray(R_WC, float) @ np.diag([1.0, -1.0, -1.0])
        self.cam_xf.Set(_mat4(R, np.asarray(cam_c, float)))
        for _ in range(self.updates):
            self.app.update()
        d = self.ann.get_data()
        return np.asarray(d)[..., :3].astype(np.uint8)
