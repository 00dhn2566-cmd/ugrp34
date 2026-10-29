"""Vision in the loop (RESULTS.md §16): the same episodes flown once per mode — corners from the perception surrogate
(surrogate), from Isaac-rendered onboard images through a YOLO-pose detector (yolo), and the same with the corners
refined on the inner edge of the frame and a detection kept only when all four are found there (yolo+edge,
vision/detector.py refine_corners: a window cut by the image border has a side with no edge) — into the same triangulator
(core/perception.py add_detections). Dynamics, referee, planner and the rest of the env are the torch env's, as in
benchmark.py; only where the window corners come from changes. Per mode: success and crash rates, the window estimate
error at each pass (also split into the window's normal — along the flight — and its plane, from the estimate at the
last decision before the pass), and per camera frame the detector's recall of the windows the surrogate would see, its
false detections and its corner error against the true projection; seen_<mode>.npy keeps every window seen per frame
(episode, t, window, range, true shortest side px, surrogate-visible, detected, mean corner error px).

    python rl_planner/scripts/vision_loop.py --policy <export> --planner rl+safe+layer \
        --config rl_planner/configs/slung/base.yaml --weights best.pt --level 2 --episodes 32 --out DIR
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

KEYS = ("success", "crashed", "crash_reason", "windows_passed", "n_windows", "time_s", "est_err_at_pass")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", required=True)
    ap.add_argument("--planner", default="rl+safe", help="rl | rl+safe | rl+safe+layer")
    ap.add_argument("--config", action="append", default=[])
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--weights", required=True, help="YOLO-pose weights (4 corners, classes red/green/blue)")
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--level", type=int, default=2)
    ap.add_argument("--episodes", type=int, default=32)
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--modes", default="surrogate,yolo", help="comma list of surrogate | yolo | yolo+edge")
    ap.add_argument("--gif-episodes", type=int, default=1, help="onboard GIF of the first N episodes of each image mode")
    ap.add_argument("--assoc", choices=("gt", "track"), default="gt",
                    help="detections -> windows: gt = the true projection (the surrogate's assumption); track = as on the "
                         "drone (vision/detector.py associate_tracked: the current estimates, else course order)")
    ap.add_argument("--blur-samples", type=int, default=6,
                    help="with a camera exposure (perception.exposure_s, e.g. general/blur10.yaml): the yolo image is the "
                         "mean of this many renders along the exposure (the surrogate's blur model streaks the corners)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, enable_cameras=True).app
    import numpy as np
    import torch
    from PIL import Image

    from rl_planner.core import math3d as m3
    from rl_planner.core.config import config_for_policy
    from rl_planner.core.mission import CRASH_NAMES
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.policy import PolicyPlanner
    from rl_planner.vision.detector import YoloCorners, associate, associate_tracked, project, refine_corners, visible
    from rl_planner.vision.isaac_view import OnboardView
    from rl_planner.vision.overlay import annotate

    with open(os.path.join(a.policy, "policy_meta.json"), encoding="utf-8") as f:
        cfg = config_for_policy(json.load(f)["config"], a.config, a.set + ["device=cuda"])
    dev = "cuda:0"
    env = TorchWindowEnv(cfg, num_envs=1, device=dev, seed=a.seed)
    core = env.core
    pr = core.perception
    base, _, guard = a.planner.partition("+")
    planner = PolicyPlanner(a.policy, device=dev, cfg=cfg)
    if "safe" in guard.split("+"):
        from rl_planner.planners.safety import SafePlanner
        planner = SafePlanner(planner, cfg, 1, dev, core.step_dt)
    if "layer" in guard.split("+"):
        from rl_planner.planners.convex_layer import SlungLayerPlanner
        planner = SlungLayerPlanner(planner, cfg, 1, dev, core.step_dt)
    view = OnboardView(app, cfg, randomize=False, updates=1)
    yolo = YoloCorners(a.weights, device=dev, imgsz=a.imgsz)
    p = cfg["perception"]
    conf_min, side_min, mg = float(p["det_conf_min"]), float(p["min_side_px"]), float(p["image_margin_px"])
    sl = core.slung
    orig_capture = pr.capture
    state = {"fp": None, "stats": None, "frames": None, "refine": False, "ep": 0, "trace": None}

    def new_stats():
        return dict(frames=0, visible=0, found=0, extra=0, false=0, corner_err=[], rows=[], assoc_err=0)

    def account(st, vis, det, uv, uv_gt, n_false, cam_c, centers):
        st["frames"] += 1
        st["visible"] += int(vis.sum())
        st["found"] += int((det & vis).sum())
        st["extra"] += int((det & ~vis).sum())
        st["false"] += n_false
        if bool(det.any()):
            st["corner_err"] += (uv[det] - uv_gt[det]).norm(dim=-1).flatten().tolist()
        rng = (centers - cam_c).norm(dim=-1)
        side = (uv_gt - uv_gt.roll(-1, dims=1)).norm(dim=-1).min(-1).values
        err = (uv - uv_gt).norm(dim=-1).mean(-1)
        t = float(core.episode_time[0])
        for w in torch.nonzero(vis | det).flatten().tolist():   # per window seen: ep, t, w, range, side, vis, det, err
            st["rows"].append((state["ep"], t, w, float(rng[w]), float(side[w]), bool(vis[w]), bool(det[w]),
                               float(err[w]) if bool(det[w]) else float("nan")))

    def surrogate_capture(cam_c, R_WC, scene, take, gen, ray_offset=None, R_err=None, motion=None):
        out = orig_capture(cam_c, R_WC, scene, take, gen, ray_offset=ray_offset, R_err=R_err, motion=motion)
        if bool(take[0]):
            R_img = R_WC if R_err is None else R_WC @ R_err
            vis, uv_gt = (t.cpu() for t in visible(pr, scene, 0, cam_c[0], R_img[0]))
            account(state["stats"], vis, out[3][0].cpu(), out[2][0].cpu(), uv_gt, 0, cam_c[0].cpu(), scene.center[0].cpu())
        return out

    def yolo_capture(cam_c, R_WC, scene, take, gen, ray_offset=None, R_err=None, motion=None):
        n = int(scene.n_windows[0])
        fp = (n, round(float(scene.center[0, :n].sum()), 5))
        new_course = fp != state["fp"]
        if new_course:                            # a new course (the env resets before the first frame)
            view.set_course(scene, 0, load_radius=None if sl is None else sl.radius)
            state["fp"] = fp
        W = scene.center.shape[1]
        uv = torch.zeros(1, W, 4, 2, device=cam_c.device)
        det = torch.zeros(1, W, dtype=torch.bool, device=cam_c.device)
        conf = torch.zeros(1, W, device=cam_c.device)
        if bool(take[0]):
            R_img = R_WC if R_err is None else R_WC @ R_err
            if sl is not None:
                hook = sl.hook(core.state.pos, core.state.quat)[0][0]
                view.set_load(sl.pos[0].cpu().numpy(), hook.cpu().numpy())
            if motion is None or a.blur_samples < 2 or float(motion[2][0]) <= 0:
                img = view.render(cam_c[0].cpu().numpy(), R_img[0].cpu().numpy())
            else:                                 # the exposure: poses from its start (as in the surrogate) to now
                vel, omega, t_exp = (x[0] for x in motion)
                acc = None
                for back in torch.linspace(1.0, 0.0, a.blur_samples).tolist():
                    dt = back * float(t_exp)
                    R_s = m3.rot_axis_angle(m3.normalize(omega), -omega.norm() * dt) @ R_img[0]
                    f = view.render((cam_c[0] - vel * dt).cpu().numpy(), R_s.cpu().numpy()).astype(np.float32)
                    acc = f if acc is None else acc + f
                img = (acc / a.blur_samples).round().astype(np.uint8)
            dets = yolo.detect(img)
            if state["refine"]:                   # the detector proposes, the frame's edges verify: all four corners
                dets = [(c, cf, q, kc) for c, cf, kp, kc in dets for q, ok in [refine_corners(img, kp, c)] if ok.all()]
            uv_gt, z_gt = project(pr, scene.corners[0].cpu(), cam_c[0].cpu(), R_img[0].cpu())
            u1, d1, c1, n_false = associate(dets, uv_gt, z_gt, scene.color[0].cpu(), scene.valid[0].cpu())
            if a.assoc == "track":                # the estimates so far, projected from the estimated camera pose
                est = pr.estimate()
                ray_c0 = (cam_c if ray_offset is None else cam_c + ray_offset)[0].cpu()
                uv_e, z_e = project(pr, est.corners[0].cpu(), ray_c0, R_WC[0].cpu())
                ug, dg = u1, d1
                u1, d1, c1, n_false = associate_tracked(dets, uv_e, z_e, est.valid[0].cpu(), scene.color[0].cpu(),
                                                        int(core.mission.target[0]), n)
                state["stats"]["assoc_err"] += int((d1 & (~dg | ((u1 - ug).norm(dim=-1).amax(-1) > 1e-3))).sum())
            side = (u1 - u1.roll(-1, dims=1)).norm(dim=-1).min(-1).values
            inside = ((u1[..., 0] >= mg) & (u1[..., 0] <= pr.width - mg) & (u1[..., 1] >= mg)
                      & (u1[..., 1] <= pr.height - mg)).all(-1)     # the surrogate's rule, on the detected corners
            d1 = d1 & (c1 >= conf_min) & (side >= side_min) & inside
            vis = visible(pr, scene, 0, cam_c[0], R_img[0])[0].cpu()
            account(state["stats"], vis, d1, u1, uv_gt, n_false, cam_c[0].cpu(), scene.center[0].cpu())
            uv[0], det[0], conf[0] = u1.to(cam_c.device), d1.to(cam_c.device), c1.to(cam_c.device)
            if state["frames"] is not None and not (new_course and state["frames"]):   # not the next course's reset
                tgt, t = int(core.mission.target[0]), float(core.episode_time[0])
                k = min(tgt, n - 1)
                ok = t > 0 and core.est is not None and bool(core.est.valid[0, k])
                err = f"estimate error {float(core.est_err[0, k]) * 100:4.1f} cm" if ok else "no estimate yet"
                truth = [uv_gt[w] for w in torch.nonzero(vis).flatten().tolist()]
                im = annotate(img, dets, truth, f"t {t:4.1f} s    passed {tgt}/{n}    {err}",
                              title="onboard camera  ·  YOLO-pose" + (" + edge refinement" if state["refine"] else " corners"))
                state["frames"].append(im.resize((960, 540), Image.LANCZOS))
                est = core.est
                state["trace"].append(dict(                  # for a composed view (drone, load, estimates) per frame
                    t=t, target=tgt, pos=core.state.pos[0].tolist(), quat=core.state.quat[0].tolist(),
                    load=None if sl is None else sl.pos[0].tolist(),
                    est_valid=[bool(x) for x in est.valid[0, :n]] if est is not None else [False] * n,
                    est_center=est.center[0, :n].tolist() if est is not None else [],
                    est_size=est.size[0, :n].tolist() if est is not None else [],
                    dets=[(int(c), float(cf), [list(map(float, q)) for q in kp]) for c, cf, kp, _ in dets]))
        ray_c = cam_c if ray_offset is None else cam_c + ray_offset
        return ray_c, R_WC, uv, det, conf, take

    os.makedirs(a.out, exist_ok=True)
    results = {}
    for mode in a.modes.split(","):
        pr.capture = surrogate_capture if mode == "surrogate" else yolo_capture
        state["refine"] = mode == "yolo+edge"
        state["stats"] = new_stats()
        eps = []
        t0 = time.time()
        for e in range(a.episodes):
            state["ep"] = e
            gif = mode != "surrogate" and e < a.gif_episodes
            state["frames"] = [] if gif else None
            state["trace"] = [] if gif else None
            core.reseed(a.seed + e)
            core.fixed_level = a.level
            env._reset_all()
            planner.reset(torch.arange(1, device=dev))
            nw = int(core.scene.n_windows[0])        # this course (the env resets to the next one when done)
            win = {k: getattr(core.scene, k)[0, :nw].tolist() for k in ("center", "normal", "right", "up", "size", "color")}
            axes = torch.tensor([win["normal"], win["right"], win["up"]]).transpose(0, 1)        # (nw, 3 axes, 3)
            true_c = torch.tensor(win["center"])
            pass_err, snap = [], None                # target estimate at the last decision before each pass
            for _ in range(int(core.max_episode_length) + 1):
                k = int(core.mission.target[0])
                if k < nw and core.est is not None and bool(core.est.valid[0, k]):
                    snap = (k, core.est.center[0, k].cpu())
                _, _, done, _ = env.step(planner.act(core))
                passed = int(core.episode_record["windows_passed"][0]) if bool(done[0]) else int(core.mission.target[0])
                if passed > k and snap is not None and snap[0] == k:
                    pass_err.append((axes[k] @ (snap[1] - true_c[k])).tolist())          # (normal, right, up) [m]
                if bool(done[0]):
                    break
            rec = {k: core.episode_record[k][0].item() for k in KEYS}
            rec["pass_err_nru"] = pass_err
            rec["crash_reason"] = CRASH_NAMES[int(rec["crash_reason"])] if rec["crashed"] else ""
            eps.append(rec)
            if gif and state["frames"]:
                path = os.path.join(a.out, f"onboard_{mode.replace('+', '_')}_ep{e}.gif")
                fr = [f.quantize(colors=200, dither=0) for f in state["frames"]]
                fr[0].save(path, save_all=True, append_images=fr[1:], duration=100, loop=0)
                with open(path[:-4] + ".json", "w", encoding="utf-8") as f:
                    json.dump(dict(record=rec, windows=win, frames=state["trace"][:len(fr)]), f)
                print(f"[vision_loop] wrote {path} ({len(fr)} frames)", flush=True)
            print(f"[vision_loop] {mode} ep {e}: {'success' if rec['success'] else rec['crash_reason'] or 'timeout'} "
                  f"{rec['windows_passed']}/{rec['n_windows']} [{time.time() - t0:.0f}s]", flush=True)
        st = state["stats"]
        ce = np.array(st["corner_err"]) if st["corner_err"] else np.array([np.nan])
        n = len(eps)
        summ = dict(
            episodes=n, success_rate=sum(r["success"] for r in eps) / n, crash_rate=sum(r["crashed"] for r in eps) / n,
            windows_passed_frac=float(np.mean([r["windows_passed"] / max(r["n_windows"], 1) for r in eps])),
            est_err_at_pass_m=float(np.mean([r["est_err_at_pass"] for r in eps if r["windows_passed"] > 0] or [np.nan])),
            est_err_normal_m=float(np.mean([abs(e[0]) for r in eps for e in r["pass_err_nru"]] or [np.nan])),
            est_err_inplane_m=float(np.mean([math.hypot(e[1], e[2]) for r in eps for e in r["pass_err_nru"]] or [np.nan])),
            crash_reasons={c: sum(r["crash_reason"] == c for r in eps) for c in set(r["crash_reason"] for r in eps) if c},
            frames=st["frames"], recall=st["found"] / max(st["visible"], 1),
            extra_per_frame=st["extra"] / max(st["frames"], 1), false_per_frame=st["false"] / max(st["frames"], 1),
            assoc_err_per_frame=st["assoc_err"] / max(st["frames"], 1),
            corner_err_px_median=float(np.nanmedian(ce)), corner_err_px_p90=float(np.nanpercentile(ce, 90)),
            corner_err_px_rms=float(np.sqrt(np.nanmean(ce ** 2))), seconds=time.time() - t0)
        results[mode] = dict(summary=summ, episodes=eps)
        np.save(os.path.join(a.out, f"seen_{mode.replace('+', '_')}.npy"), np.array(st["rows"], dtype=np.float64))
        print(f"[vision_loop] {mode}: " + json.dumps({k: v for k, v in summ.items()}), flush=True)
    meta = dict(policy=a.policy, planner=a.planner, config=a.config, set=a.set, weights=a.weights, imgsz=a.imgsz,
                blur_samples=a.blur_samples, assoc=a.assoc, level=a.level, episodes=a.episodes, seed=a.seed)
    with open(os.path.join(a.out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(dict(meta=meta, results=results), f, indent=1)
    print(f"[vision_loop] wrote {a.out}/summary.json")
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
