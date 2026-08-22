"""현재 파이프라인 상태를 그림으로 만든다.

    python render_status.py --out /home/yoonho/fig/6_render --tag v2

산출 (라벨은 전부 영문 — claude.md 규칙 3):
  <tag>_01_scene.png     뚫린 테두리 창문 + 텍스처 방 + 3D 잡물
  <tag>_02_detect.png    YOLO 검출, 클래스(색)는 헤드에서 직접 (HSV 후처리 없음)
  <tag>_03_sweep.gif     관측 스윕
  <tag>_04_recon.png     GT vs 태민 원본 vs overrides — 3D + 탑뷰
  <tag>_05_error.png     창문별 center 오차 (원본 vs overrides) + 인라이어 비율
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

from module import contract, taemin_bridge  # noqa: E402
from overrides import detections as ovd  # noqa: E402
from overrides import recon_rays  # noqa: E402
from utils import device, metrics, scene, viz  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=None)
    ap.add_argument("--out", default=paths.FIG_DIR)
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--frames-per-window", type=int, default=32)
    ap.add_argument("--clutter", type=int, default=18)
    ap.add_argument("--mode", default="xy", choices=scene.PATH_MODES)
    ap.add_argument("--span", type=float, default=110.0)
    ap.add_argument("--det-conf-min", type=float, default=0.5)
    ap.add_argument("--device", default="auto", choices=("auto", "cuda", "cpu"))
    ap.add_argument("--tag", default="status")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)

    import pybullet as p
    viz.use_agg()
    import matplotlib.pyplot as plt
    from PIL import Image
    from eval_recon3d import quat_xyzw_to_rot
    from sim import pybullet_stream as pbs

    intr = contract.intrinsics()
    det = device.load_detector(a.weights, prefer=a.device)
    env, layout = scene.make(seed=a.seed, clutter=a.clutter)
    scene.print_layout(layout)
    mid = np.mean([w["center"] for w in layout], axis=0)

    # --- 01 scene ------------------------------------------------------------
    def shot(dist, yaw, pitch, W=1000, H=700, target=None):
        v = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=(target if target is not None else mid).tolist(),
            distance=dist, yaw=yaw, pitch=pitch, roll=0, upAxisIndex=2,
            physicsClientId=env.CLIENT)
        pr = p.computeProjectionMatrixFOV(60, W / H, 0.05, 40, physicsClientId=env.CLIENT)
        _, _, rgb, _, _ = p.getCameraImage(W, H, v, pr, shadow=1,
                                           renderer=p.ER_TINY_RENDERER,
                                           physicsClientId=env.CLIENT)
        return np.asarray(rgb, np.uint8).reshape(H, W, 4)[:, :, :3]

    fig, axes = plt.subplots(1, 2, figsize=(15, 5.2))
    for ax, (d, yw, pt, t) in zip(axes, [(5.0, 42, -22, "isometric"),
                                         (4.6, 90, -8, "side")]):
        ax.imshow(shot(d, yw, pt)); ax.axis("off"); ax.set_title(t, fontsize=11)
    fig.suptitle("PyBullet scene — open-frame windows, textured room, 3D clutter for VIO parallax",
                 fontsize=13)
    viz.save(fig, a.out, f"{a.tag}_01_scene.png", dpi=110)

    # --- observe -------------------------------------------------------------
    poses, path_name = scene.path(layout, mode=a.mode,
                                  n_per_window=a.frames_per_window, span_deg=a.span)
    print(f"경로: {path_name}  ({len(poses)} 프레임)")
    samples, stats = taemin_bridge.observe(env, det, None, poses, conf=0.25)
    pre = ovd.count(samples)
    samples_clean = ovd.clean_samples(samples)
    post = ovd.count(samples_clean)
    print(f"관측 {stats['frames']} 프레임, {stats['detections']} 검출, "
          f"색 판정 = {stats['colour_from']}")
    print(f"overrides/detections: 중복 표 {pre['duplicate_votes']}개 제거 "
          f"({pre['detections']} -> {post['detections']})")

    res_orig = taemin_bridge.run_offline(samples, verbose=False,
                                         det_conf_min=a.det_conf_min)
    res_ovr = recon_rays.reconstruct(samples_clean, det_conf_min=a.det_conf_min)
    rows_orig = metrics.score(res_orig, layout)
    rows_ovr = metrics.score(res_ovr, layout)

    # --- 02 detections -------------------------------------------------------
    picks = [3, a.frames_per_window + 3, 2 * a.frames_per_window + 3]
    picks = [i for i in picks if i < len(poses)]
    fig, axes = plt.subplots(1, len(picks), figsize=(6.4 * len(picks), 4.2))
    for ax, fi in zip(np.atleast_1d(axes), picks):
        pos, q = poses[fi]
        R = quat_xyzw_to_rot(q)
        img = pbs.render_frame(p, env.CLIENT, pos, pos + R[:, 2], intr)
        ax.imshow(img)
        r = det.predict(img[:, :, ::-1], conf=0.25, agnostic_nms=True)[0]
        for box, kp in zip(r.boxes, r.keypoints.xy.cpu().numpy()):
            name = det.names[int(box.cls)]
            ax.plot(np.r_[kp[:, 0], kp[0, 0]], np.r_[kp[:, 1], kp[0, 1]],
                    lw=2.4, color=viz.COL.get(name, "w"))
            ax.scatter(kp[:, 0], kp[:, 1], s=28, c="w", edgecolors="k", zorder=5)
            ax.text(kp[:, 0].mean(), kp[:, 1].min() - 10,
                    f"{name} {float(box.conf):.2f}",
                    color=viz.COL.get(name, "w"), fontsize=10, ha="center", weight="bold")
        ax.set_xlim(0, intr["width"]); ax.set_ylim(intr["height"], 0); ax.axis("off")
        ax.set_title(f"frame {fi}", fontsize=10)
    fig.suptitle("Detections — class (colour) comes straight from the fine-tuned head, "
                 "no HSV post-step", fontsize=13)
    viz.save(fig, a.out, f"{a.tag}_02_detect.png", dpi=105)

    # --- 03 sweep gif --------------------------------------------------------
    frames = [Image.fromarray(shot(2.6, 42, -18, 640, 460,
                                   target=np.asarray(poses[fi][0])))
              for fi in range(0, len(poses), 3)]
    frames[0].save(os.path.join(a.out, f"{a.tag}_03_sweep.gif"), save_all=True,
                   append_images=frames[1:], duration=90, loop=0)
    print(f"wrote {a.tag}_03_sweep.gif ({len(frames)} frames)")
    del frames

    # --- 04 GT vs original vs override --------------------------------------
    by_o = {r["order_index"]: r for r in res_orig}
    by_v = {r["order_index"]: r for r in res_ovr}
    cam = np.array([s["p_WI"] for s in samples])

    fig = plt.figure(figsize=(15, 5.6))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.plot(cam[:, 0], cam[:, 1], cam[:, 2], lw=1.0, color="#888", label="camera path")
    seen = set()
    for w in layout:
        c = np.asarray(w["center"], float)
        gt = scene.corners_world(w)
        viz.draw_window_3d(ax, gt, w["color"],
                           label=viz.legend_once(ax, seen, "gt", "ground truth"))
        r = by_o.get(w["order_index"])
        if r:
            e = np.array(r["corners_w"])
            ax.plot(np.r_[e[:, 0], e[0, 0]], np.r_[e[:, 1], e[0, 1]],
                    np.r_[e[:, 2], e[0, 2]], lw=1.6, ls=":", color=viz.COL[w["color"]],
                    label=viz.legend_once(ax, seen, "o", "recon — original node"))
        r = by_v.get(w["order_index"])
        if r:
            e = np.array(r["corners_w"])
            ax.plot(np.r_[e[:, 0], e[0, 0]], np.r_[e[:, 1], e[0, 1]],
                    np.r_[e[:, 2], e[0, 2]], lw=2.2, ls="--", color=viz.COL[w["color"]],
                    label=viz.legend_once(ax, seen, "v", "recon — overrides"))
            ax.scatter(*np.array(r["center_w"]), s=60, marker="x",
                       color=viz.COL[w["color"]])
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.view_init(elev=16, azim=-64); ax.set_box_aspect((3, 2, 1.3))
    ax.legend(fontsize=8, loc="upper left")
    ax.set_title("solid = ground truth,  dotted = original,  dashed = overrides", fontsize=10)

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(cam[:, 0], cam[:, 1], lw=1.0, color="#888")
    for w in layout:
        viz.draw_window_top(ax2, scene.corners_world(w), w["color"])
        r = by_o.get(w["order_index"])
        if r:
            e = np.array(r["corners_w"])
            ax2.plot([e[:, 0].mean()] * 2, [e[:, 1].min(), e[:, 1].max()],
                     lw=1.6, ls=":", color=viz.COL[w["color"]])
        r = by_v.get(w["order_index"])
        if r:
            viz.draw_window_top(ax2, np.array(r["corners_w"]), w["color"], est=True)
    ax2.set_xlabel("x [m]"); ax2.set_ylabel("y [m]"); ax2.grid(alpha=.3)
    ax2.set_title("top view — depth (x) is the weak axis", fontsize=10)
    fig.suptitle("Image -> YOLO -> triangulation -> 3D windows   "
                 "(original node vs our overrides, same observations)", fontsize=13)
    viz.save(fig, a.out, f"{a.tag}_04_recon.png", dpi=115)

    # --- 05 error comparison -------------------------------------------------
    names = [r["color"] for r in rows_orig]
    eo = [r["center_mm"] if r["ok"] else np.nan for r in rows_orig]
    ev = [r["center_mm"] if r["ok"] else np.nan for r in rows_ovr]
    x = np.arange(len(names))

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(13.5, 4.6))
    axA.bar(x - 0.2, eo, 0.38, label="original node", color="#bbb", edgecolor="#777")
    axA.bar(x + 0.2, ev, 0.38, label="overrides",
            color=[viz.COL[n] for n in names], edgecolor="#333")
    for i, (vo, vv) in enumerate(zip(eo, ev)):
        if np.isfinite(vo):
            axA.text(i - 0.2, vo + 8, f"{vo:.0f}", ha="center", fontsize=9)
        if np.isfinite(vv):
            axA.text(i + 0.2, vv + 8, f"{vv:.0f}", ha="center", fontsize=9, weight="bold")
    axA.axhline(viz.CLEARANCE_MM, ls="--", color="#666", lw=1.3)
    axA.text(len(names) - 0.5, viz.CLEARANCE_MM * 1.03, "planner clearance 350 mm",
             ha="right", fontsize=9, color="#666")
    axA.set_xticks(x); axA.set_xticklabels(names)
    axA.set_ylabel("centre error [mm]"); axA.grid(alpha=.3, axis="y")
    axA.legend(fontsize=9)
    axA.set_title("reconstruction error — same observations, different maths", fontsize=11)

    inl = [by_v[r["order_index"]]["inlier_frac"] * 100 if r["order_index"] in by_v else 0
           for r in rows_ovr]
    axB.bar(x, inl, 0.5, color=[viz.COL[n] for n in names], edgecolor="#333")
    for i, v in enumerate(inl):
        axB.text(i, v + 1.2, f"{v:.0f}%", ha="center", fontsize=10)
    axB.set_xticks(x); axB.set_xticklabels(names)
    axB.set_ylim(0, 105); axB.set_ylabel("inlier rays kept [%]")
    axB.grid(alpha=.3, axis="y")
    axB.set_title("Huber IRLS — share of rays that survived", fontsize=11)

    n_ok_o = sum(r["ok"] for r in rows_orig)
    n_ok_v = sum(r["ok"] for r in rows_ovr)
    fig.suptitle(f"{n_ok_o}/{len(layout)} original, {n_ok_v}/{len(layout)} overrides   "
                 f"({stats['detections']} detections over {stats['frames']} frames, "
                 f"{scene.path_label_en(a.mode, a.span)})", fontsize=12)
    viz.save(fig, a.out, f"{a.tag}_05_error.png", dpi=115)

    print("\n원본 노드:")
    metrics.print_rows(rows_orig)
    print("overrides:")
    metrics.print_rows(rows_ovr)
    print(f"\ncenter 오차 중앙값  원본 {np.nanmedian(eo):.0f} mm  ->  "
          f"overrides {np.nanmedian(ev):.0f} mm")

    env.close()
    print("\nRENDER_STATUS_DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
