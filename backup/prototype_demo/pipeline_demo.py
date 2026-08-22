"""End-to-end demo: camera image -> YOLO -> triangulation -> waypoints.

This is the seam that used to be missing. Every 3D-reconstruction and margin
number in the repo so far came from 길남's synthetic GT corner stream with
statistically re-created noise; the real detector's output had never travelled
that path. Here a PyBullet scene is rendered, the trained YOLO-pose weights run
on the actual frames, and the resulting §5 stream is handed to the team's own
`reconstruct_windows` -> `assemble_window_map` -> `plan_waypoints` unchanged.

    python pipeline_demo.py                       # fine-tuned weights, open frames
    python pipeline_demo.py --pane                # original weights' domain (opaque)

Outputs a report to stdout and figures to --fig-dir.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TEAM = os.path.dirname(HERE)
REPO = os.path.join(TEAM, "reinforcement_yunho")
for _p in (REPO,
           os.path.join(TEAM, "overall_gilnam", "vision"),
           os.path.join(TEAM, "overall_gilnam", "planning"),
           os.path.join(TEAM, "overall_gilnam", "integration")):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--weights", default=os.path.join(HERE, "model", "pyb_openframe_best.pt"),
                    help="detector weights (.pt)")
    ap.add_argument("--camera-config", default=os.path.join(HERE, "config", "camera.yaml"))
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--n-windows", type=int, default=3)
    ap.add_argument("--opening", type=float, default=1.0, help="window opening [m] (spec 0.8-1.2)")
    ap.add_argument("--frames-per-window", type=int, default=24)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--pane", action="store_true",
                    help="fill the opening (matches the ORIGINAL weights' domain)")
    ap.add_argument("--fig-dir", default=os.path.join(HERE, "out"))
    a = ap.parse_args(argv)

    if not os.path.exists(a.weights):
        print(f"[error] weights not found: {a.weights}\n"
              f"        put a .pt in prototype/model/ (see prototype/model/README.md)",
              file=sys.stderr)
        return 2

    import pybullet as p
    import yaml
    from rl.pybullet_window_env import WindowTraversalAviary
    from rl import domain
    from sim import pybullet_stream as pbs
    from color_judge import load_color_config
    from infer_stream import load_model
    from eval_recon3d import reconstruct_windows
    from e2e_rehearsal import assemble_window_map
    from window_waypoint_planner import (
        crossing_warnings, gate_points, load_planner_config, plan_waypoints)

    # --- camera: this repo's own config, not the placeholder in vision/ ---------
    with open(a.camera_config) as f:
        cam = yaml.safe_load(f)
    ci = cam["camera"]["intrinsics"]
    intr = {"width": float(cam["camera"]["resolution"]["width"]),
            "height": float(cam["camera"]["resolution"]["height"]),
            "fx": float(ci["fx"]), "fy": float(ci["fy"]),
            "cx": float(ci["cx"]), "cy": float(ci["cy"])}
    cfg = load_planner_config(os.path.join(TEAM, "overall_gilnam", "planning",
                                           "planner_limits.yaml"))
    print(f"camera   {intr['width']:.0f}x{intr['height']:.0f}  fx={intr['fx']:.0f}  "
          f"hfov={cam['camera']['fov']['horizontal_deg']:.1f} deg  "
          f"({cam['meta']['status']})")
    print(f"planner  d_app={cfg['d_app']} d_exit={cfg['d_exit']} "
          f"clearance={cfg['clearance_margin']}")
    print(f"weights  {a.weights}")
    print(f"windows  {'FILLED pane' if a.pane else 'OPEN frame'}\n")

    env = WindowTraversalAviary(n_windows=a.n_windows, seed=a.seed, step=0.3,
                                opening=a.opening, pane=a.pane, domain_match=True)
    env.reset(seed=a.seed)
    layout = env.window_layout
    scene_gt = pbs.scene_gt_from_layout(layout, intr, seed=a.seed)
    print("scene (ground truth):")
    for w in layout:
        c = w["center"]
        print(f"  #{w['order_index']} {w['color']:6s} "
              f"center=({c[0]:5.2f},{c[1]:5.2f},{c[2]:5.2f})  "
              f"opening {w['ow']:.2f} x {w['oh']:.2f} m")

    # --- 1. render + detect ----------------------------------------------------
    model = load_model(a.weights)
    color_config = load_color_config(os.path.join(TEAM, "overall_gilnam", "vision",
                                                  "color_order.yaml"))
    poses = pbs.per_window_sweep(layout, n_per_window=a.frames_per_window)
    records, stats = pbs.capture(p, env.CLIENT, layout, model, color_config, intr,
                                 poses, conf=a.conf,
                                 save_dir=os.path.join(a.fig_dir, "frames"))
    print(f"\n[1] detect       {stats}")

    # --- 2. triangulate (team code, unmodified) --------------------------------
    recon = reconstruct_windows(records, scene_gt)
    print("\n[2] triangulate  (corner error vs ground truth)")
    n_ok = 0
    for gt in scene_gt["windows"]:
        oi = gt["order_index"]
        est = recon[oi]["corners_3d_est"]
        if est is None:
            print(f"  #{oi} {gt['color']:6s}  FAILED  (n_pairs={recon[oi]['n_pairs']})")
            continue
        n_ok += 1
        err = np.linalg.norm(np.asarray(est) - np.asarray(gt["corners_3d"]), axis=1)
        cen = np.linalg.norm(np.asarray(est).mean(0) - np.asarray(gt["center"]))
        print(f"  #{oi} {gt['color']:6s}  n_pairs={recon[oi]['n_pairs']:5d}  "
              f"corner mean {err.mean()*1000:7.1f} mm  max {err.max()*1000:7.1f} mm  "
              f"center {cen*1000:7.1f} mm")
    if n_ok == 0:
        print("\nno window reconstructed — stopping")
        env.close()
        return 1

    # --- 3/4. window map + waypoints (team code, unmodified) -------------------
    wmap, failed = assemble_window_map(recon)
    print(f"\n[3] window map   {len(wmap['windows'])} window(s) usable, failed={failed}")

    warns = []
    wc = plan_waypoints({"position": [0.0, 0.0, 1.0]}, wmap, cfg, warn=warns.append)
    warns += crossing_warnings(wc.waypoints, scene_gt["windows"], cfg["clearance_margin"])
    print(f"\n[4] waypoints    {len(wc.waypoints)} points (= 1 + 2N)")
    for i, w in enumerate(wc.waypoints):
        print(f"  {i}: [{w[0]:7.3f}, {w[1]:7.3f}, {w[2]:7.3f}]")
    print(f"    limits {wc.limits}   dt {wc.dt}")

    print("\n[5] gate-point error  (planned from vision vs planned from GT)")
    for gt in scene_gt["windows"]:
        oi = gt["order_index"]
        if recon[oi]["corners_3d_est"] is None:
            continue
        a_gt, e_gt = gate_points(
            {"center": gt["center"], "corners_3d": gt["corners_3d"],
             "size_wh": gt["size_wh"], "order_index": oi},
            cfg["d_app"], cfg["d_exit"], cfg["clearance_margin"])
        est = next(w for w in wmap["windows"] if w["order_index"] == oi)
        a_es, e_es = gate_points(est, cfg["d_app"], cfg["d_exit"], cfg["clearance_margin"])
        print(f"  #{oi} {gt['color']:6s}  approach {np.linalg.norm(a_es-a_gt)*1000:7.1f} mm"
              f"   exit {np.linalg.norm(e_es-e_gt)*1000:7.1f} mm")

    print(f"\n[6] warnings     {len(warns)}")
    for w_ in warns:
        print("    " + str(w_))

    ok = (n_ok == len(scene_gt["windows"])) and not warns
    print("\n" + ("PASS — all windows reconstructed, no planner warning"
                  if ok else
                  f"PARTIAL — {n_ok}/{len(scene_gt['windows'])} reconstructed, "
                  f"{len(warns)} warning(s)"))
    env.close()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
