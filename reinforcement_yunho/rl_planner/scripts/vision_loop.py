"""Vision in the loop (RESULTS.md §16): the same episodes flown twice — corners from the perception surrogate, and
corners from Isaac-rendered onboard images through a YOLO-pose detector (rl_planner/vision) — into the same
triangulator (core/perception.py add_detections). Dynamics, referee, planner and the rest of the env are the torch
env's, as in benchmark.py; only where the window corners come from changes. Per mode: success and crash rates, the
window estimate error at each pass, and per camera frame the detector's recall of the windows the surrogate would
see, its false detections and its corner error against the true projection.

    python rl_planner/scripts/vision_loop.py --policy <export> --planner rl+safe+layer \
        --config rl_planner/configs/slung/base.yaml --weights best.pt --level 2 --episodes 32 --out DIR
"""
from __future__ import annotations

import argparse
import json
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
    ap.add_argument("--modes", default="surrogate,yolo")
    ap.add_argument("--gif-episodes", type=int, default=1, help="onboard GIF of the first N yolo episodes")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, enable_cameras=True).app
    import numpy as np
    import torch
    from PIL import Image, ImageDraw

    from rl_planner.core.config import config_for_policy
    from rl_planner.core.mission import CRASH_NAMES
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.planners.policy import PolicyPlanner
    from rl_planner.vision.detector import YoloCorners, associate, project, visible
    from rl_planner.vision.isaac_view import OnboardView

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
    conf_min, side_min = float(p["det_conf_min"]), float(p["min_side_px"])
    sl = core.slung
    orig_capture = pr.capture
    state = {"fp": None, "stats": None, "frames": None}
    PAL = [(255, 70, 70), (70, 230, 90), (90, 150, 255)]

    def new_stats():
        return dict(frames=0, visible=0, found=0, extra=0, false=0, corner_err=[])

    def account(st, vis, det, uv, uv_gt, n_false):
        st["frames"] += 1
        st["visible"] += int(vis.sum())
        st["found"] += int((det & vis).sum())
        st["extra"] += int((det & ~vis).sum())
        st["false"] += n_false
        if bool(det.any()):
            st["corner_err"] += (uv[det] - uv_gt[det]).norm(dim=-1).flatten().tolist()

    def surrogate_capture(cam_c, R_WC, scene, take, gen, ray_offset=None, R_err=None, motion=None):
        out = orig_capture(cam_c, R_WC, scene, take, gen, ray_offset=ray_offset, R_err=R_err, motion=motion)
        if bool(take[0]):
            R_img = R_WC if R_err is None else R_WC @ R_err
            vis, uv_gt = (t.cpu() for t in visible(pr, scene, 0, cam_c[0], R_img[0]))
            account(state["stats"], vis, out[3][0].cpu(), out[2][0].cpu(), uv_gt, 0)
        return out

    def yolo_capture(cam_c, R_WC, scene, take, gen, ray_offset=None, R_err=None, motion=None):
        n = int(scene.n_windows[0])
        fp = (n, round(float(scene.center[0, :n].sum()), 5))
        if fp != state["fp"]:                     # a new course (the env resets before the first frame)
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
            img = view.render(cam_c[0].cpu().numpy(), R_img[0].cpu().numpy())
            dets = yolo.detect(img)
            uv_gt, z_gt = project(pr, scene.corners[0].cpu(), cam_c[0].cpu(), R_img[0].cpu())
            u1, d1, c1, n_false = associate(dets, uv_gt, z_gt, scene.color[0].cpu(), scene.valid[0].cpu())
            side = (u1 - u1.roll(-1, dims=1)).norm(dim=-1).min(-1).values
            d1 = d1 & (c1 >= conf_min) & (side >= side_min)
            vis = visible(pr, scene, 0, cam_c[0], R_img[0])[0].cpu()
            account(state["stats"], vis, d1, u1, uv_gt, n_false)
            uv[0], det[0], conf[0] = u1.to(cam_c.device), d1.to(cam_c.device), c1.to(cam_c.device)
            if state["frames"] is not None:
                im = Image.fromarray(img)
                dr = ImageDraw.Draw(im)
                for cls, cf, kp, _ in dets:
                    pts = [tuple(map(float, q)) for q in kp]
                    dr.line(pts + [pts[0]], fill=PAL[cls % 3], width=4)
                    for q in pts:
                        dr.ellipse((q[0] - 7, q[1] - 7, q[0] + 7, q[1] + 7), outline=(255, 255, 0), width=3)
                    dr.text((pts[0][0], pts[0][1] - 24), f"{cf:.2f}", fill=(255, 255, 0))
                for w in torch.nonzero(vis).flatten().tolist():
                    for q in uv_gt[w].tolist():
                        dr.ellipse((q[0] - 3, q[1] - 3, q[0] + 3, q[1] + 3), fill=(255, 255, 255))
                tgt = int(core.mission.target[0])
                err = float(core.est_err[0, min(tgt, W - 1)]) if core.est_err is not None else float("nan")
                dr.rectangle((0, 0, 1280, 44), fill=(10, 14, 24))
                dr.text((14, 12), f"onboard camera -> YOLO-pose -> triangulation    t = {float(core.episode_time[0]):4.1f} s    "
                                  f"windows passed {tgt}/{n}    target estimate error {err * 100:4.1f} cm", fill=(235, 240, 250))
                state["frames"].append(im.resize((960, 540)))
        ray_c = cam_c if ray_offset is None else cam_c + ray_offset
        return ray_c, R_WC, uv, det, conf, take

    os.makedirs(a.out, exist_ok=True)
    results = {}
    for mode in a.modes.split(","):
        pr.capture = surrogate_capture if mode == "surrogate" else yolo_capture
        state["stats"] = new_stats()
        eps = []
        t0 = time.time()
        for e in range(a.episodes):
            gif = mode == "yolo" and e < a.gif_episodes
            state["frames"] = [] if gif else None
            core.reseed(a.seed + e)
            core.fixed_level = a.level
            env._reset_all()
            planner.reset(torch.arange(1, device=dev))
            for _ in range(int(core.max_episode_length) + 1):
                _, _, done, _ = env.step(planner.act(core))
                if bool(done[0]):
                    break
            rec = {k: core.episode_record[k][0].item() for k in KEYS}
            rec["crash_reason"] = CRASH_NAMES[int(rec["crash_reason"])] if rec["crashed"] else ""
            eps.append(rec)
            if gif and state["frames"]:
                path = os.path.join(a.out, f"onboard_ep{e}.gif")
                fr = [f.quantize(colors=200, dither=0) for f in state["frames"]]
                fr[0].save(path, save_all=True, append_images=fr[1:], duration=100, loop=0)
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
            crash_reasons={c: sum(r["crash_reason"] == c for r in eps) for c in set(r["crash_reason"] for r in eps) if c},
            frames=st["frames"], recall=st["found"] / max(st["visible"], 1),
            extra_per_frame=st["extra"] / max(st["frames"], 1), false_per_frame=st["false"] / max(st["frames"], 1),
            corner_err_px_median=float(np.nanmedian(ce)), corner_err_px_p90=float(np.nanpercentile(ce, 90)),
            corner_err_px_rms=float(np.sqrt(np.nanmean(ce ** 2))), seconds=time.time() - t0)
        results[mode] = dict(summary=summ, episodes=eps)
        print(f"[vision_loop] {mode}: " + json.dumps({k: v for k, v in summ.items()}), flush=True)
    meta = dict(policy=a.policy, planner=a.planner, config=a.config, set=a.set, weights=a.weights, imgsz=a.imgsz,
                level=a.level, episodes=a.episodes, seed=a.seed)
    with open(os.path.join(a.out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(dict(meta=meta, results=results), f, indent=1)
    print(f"[vision_loop] wrote {a.out}/summary.json")
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
