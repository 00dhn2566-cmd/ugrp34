"""Pure-logic smoke test for the sim/ package (numpy + pyyaml only).

Exercises the real cross-team contracts end-to-end without Isaac Sim / GPU:
  1. scene_gen.sample_scene         -> spec §4.1 randomisation sane; textured bg
  2. build_label_lines              -> 17-token YOLO-pose lines, class/coords sane
  3. convention bridge cross-check  -> pixels == 길남's synth_scene formula
  4. export_dataset (from-metadata) -> 8/1/1 split + meta.jsonl (길남 eval_corners)
  5. export_vio                     -> EuRoC-ASL mav0 CSVs round-trip (int ns, WXYZ)
  6. export_stream                  -> §5 message via 길남's gt_stream (not hand-rolled)

Run:  cd reinforcement_yunho && python3 sim/smoke_test.py
"""
import json
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from common import (  # noqa: E402
    project_points,
    window_corners_world,
    world_to_camera_cv,
)
from sim import scene_gen  # noqa: E402
from sim.replicator_writer import build_label_lines, build_label_records  # noqa: E402
from sim import export_dataset, export_vio, export_stream  # noqa: E402

T0_NS = 1_720_000_000_000_000_000


def test_scene_sampler():
    scene = scene_gen.sample_scene(seed=7)
    n = len(scene["windows"])
    assert 1 <= n <= 5, f"spec §4.1: 1-5 windows, got {n}"
    for w in scene["windows"]:
        assert w["color"] in ("red", "green", "blue")
        assert w["order_index"] == {"red": 0, "green": 1, "blue": 2}[w["color"]]
        lo = min(r[0] for r in scene_gen.DISTANCE_BINS.values())
        hi = max(r[1] for r in scene_gen.DISTANCE_BINS.values())
        # distance_m is the Euclidean camera->centre range; it is >= the along-axis
        # depth bin (off-axis windows sit on the hypotenuse), so bound it loosely.
        assert 0.5 * lo <= w["distance_m"] <= 1.7 * hi, w["distance_m"]
        assert w["distance_bin"] in scene_gen.DISTANCE_BINS
    assert scene["background"]["kind"] == "textured", "textured bg mandatory (박태민 07/03)"
    # blank background must be rejected
    bad = dict(scene); bad["background"] = {"kind": "blank"}
    try:
        scene_gen.validate_scene(bad)
        raise AssertionError("blank background should raise")
    except ValueError:
        pass
    print(f"  [1 scene_gen]  {n} windows, textured bg, distances in range OK")


def _first_scene_with_labels():
    """Find a seed whose scene yields >=1 fully-visible window."""
    intr = scene_gen.default_intrinsics()
    for seed in range(50):
        scene = scene_gen.sample_scene(seed=seed, intr=intr)
        recs = build_label_records(scene["windows"], scene["camera"]["T_world_cam_usd"], intr)
        if recs:
            return scene, intr, recs
    raise AssertionError("no visible window in 50 seeds (sampler/projection broken)")


def test_label_lines():
    scene, intr, _ = _first_scene_with_labels()
    lines = build_label_lines(scene["windows"], scene["camera"]["T_world_cam_usd"], intr)
    assert lines, "expected >=1 label line"
    for ln in lines:
        tok = ln.split()
        assert len(tok) == 17, f"17-token line required, got {len(tok)}: {ln!r}"
        assert int(tok[0]) in (0, 1, 2), "class = order_index red0/green1/blue2"
        vals = [float(t) for t in tok[1:]]
        assert all(0.0 <= x <= 1.0 for x in vals), "normalised coords in [0,1]"
        assert all(tok[7 + 3 * i] == "1" for i in range(4)), "dataset-1 vis all 1"
    print(f"  [2 labels]     {len(lines)} line(s), 17 tokens, class/coords sane OK")


def test_projection_cross_check():
    """common (via USD bridge) pixels == 길남's synth_scene.project formula
    X_cam = R_wc^T (X - t), u = fx*Xc/Zc + cx  (README_stream.md contract)."""
    intr = scene_gen.default_intrinsics()
    K = intr.K()
    eye = np.array([0.0, 0.0, 1.5])
    R_cv = scene_gen.look_at_cv(eye, eye + np.array([1.0, 0.0, 0.0]))  # look world +X
    center = np.array([5.0, 0.3, 1.6])
    normal = center * 0 + (eye - center) / np.linalg.norm(eye - center)  # toward camera (vertical)
    R_win = scene_gen.window_rotation_from_normal(normal)
    corners = window_corners_world(center, R_win, 1.0, 1.2)

    T_usd = scene_gen.cv_to_usd_transform(R_cv, eye)
    uv_common, _, in_front = project_points(world_to_camera_cv(corners, T_usd), K)
    assert in_front.all()

    Xc = (R_cv.T @ (corners - eye).T).T  # 길남's CV projection, inline
    uv_gilnam = np.column_stack([K[0, 0] * Xc[:, 0] / Xc[:, 2] + K[0, 2],
                                 K[1, 1] * Xc[:, 1] / Xc[:, 2] + K[1, 2]])
    assert np.allclose(uv_common, uv_gilnam, atol=1e-6), (uv_common - uv_gilnam)
    print("  [3 coords]     USD-bridge pixels == 길남 synth_scene formula OK")


def test_export_dataset_and_meta():
    intr = scene_gen.default_intrinsics()
    with tempfile.TemporaryDirectory() as d:
        meta_dir = os.path.join(d, "metadata")
        os.makedirs(meta_dir)
        n_frames = 10
        for i in range(n_frames):
            scene = scene_gen.sample_scene(seed=100 + i, intr=intr)
            m = scene_gen.scene_to_metadata(scene, image=f"frame_{i:04d}.png",
                                            timestamp_ns=T0_NS + i, frame_id=i)
            Path(meta_dir, f"frame_{i:04d}.json").write_text(json.dumps(m), encoding="utf-8")

        out = os.path.join(d, "dataset")
        manifest = export_dataset.assemble_from_metadata(
            meta_dir, out, intr, seed=42, copy_images=False
        )
        counts = manifest["counts"]
        assert (counts["train"], counts["val"], counts["test"]) == (8, 1, 1), counts

        # meta.jsonl parses + matches the schema 길남's eval_corners._load_meta reads
        recs = [json.loads(l) for l in Path(out, "meta.jsonl").read_text().splitlines()]
        assert len(recs) == n_frames
        for r in recs:
            assert r["image"].startswith("images/") and r["image"].endswith(".png")
            for w in r["windows"]:
                assert set(w) == {"order_index", "distance_m"}
        sys.path.insert(0, export_stream.GILNAM_VISION)  # cv2-free: gt_stream only
        import eval_corners
        loaded = eval_corners._load_meta(Path(out))
        assert len(loaded) == n_frames, "eval_corners._load_meta must read every image"
    print(f"  [4 dataset]    8/1/1 split + meta.jsonl read by eval_corners OK")


def test_export_vio():
    intr = scene_gen.default_intrinsics()
    scene = scene_gen.sample_scene(seed=3, intr=intr)
    q_wxyz = list(map(float, scene["camera"]["quat_wxyz"]))
    pos = list(map(float, scene["camera"]["position"]))

    cam = [{"timestamp_ns": T0_NS + k * 50_000_000, "filename": f"{T0_NS + k*50_000_000}.png"}
           for k in range(5)]                                   # 20 Hz
    imu = [{"timestamp_ns": T0_NS + k * 5_000_000,
            "gyro": [0.01, -0.02, 0.0], "accel": [0.0, 0.0, 9.81]}
           for k in range(50)]                                  # 200 Hz
    gt = [{"timestamp_ns": T0_NS + k * 50_000_000, "position": pos, "quat_wxyz": q_wxyz}
          for k in range(5)]

    with tempfile.TemporaryDirectory() as d:
        summary = export_vio.write_euroc(d, cam, imu, gt, intr=intr)
        mav0 = summary["mav0"]
        rc, ri, rg = (export_vio.read_cam0_csv(mav0), export_vio.read_imu0_csv(mav0),
                      export_vio.read_gt_csv(mav0))
        assert len(rc) == 5 and len(ri) == 50 and len(rg) == 5
        assert all(isinstance(s["timestamp_ns"], int) for s in ri), "IMU ts int ns"
        assert len(rg[0]["quat_wxyz"]) == 4 and np.allclose(rg[0]["quat_wxyz"], q_wxyz)
        # cam rate ~20 Hz, imu ~200 Hz from the stamps
        assert ri[1]["timestamp_ns"] - ri[0]["timestamp_ns"] == 5_000_000
        # float timestamps must be rejected (int-ns contract)
        try:
            export_vio.write_gt_csv(mav0, [{"timestamp_ns": 1.0, "position": pos, "quat_wxyz": q_wxyz}])
            raise AssertionError("float timestamp should raise")
        except ValueError:
            pass
    print("  [5 euroc]      mav0 cam/imu/gt CSV round-trip, int-ns + WXYZ OK")


def test_export_stream():
    scene, intr, _ = _first_scene_with_labels()
    lines = build_label_lines(scene["windows"], scene["camera"]["T_world_cam_usd"], intr)
    config = export_stream.load_color_config()
    rec = export_stream.build_stream_record(
        lines, T0_NS, 0,
        list(map(float, scene["camera"]["position"])),
        list(map(float, scene["camera"]["quat_xyzw"])),
        config,
    )
    assert rec["vision"]["timestamp"] == rec["pose"]["timestamp"] == T0_NS
    assert len(rec["pose"]["orientation"]) == 4  # xyzw
    assert len(rec["vision"]["windows"]) == len(lines)
    for w in rec["vision"]["windows"]:
        assert w["det_conf"] == 1.0 and w["color_conf"] == 1.0  # GT stream
        assert len(w["corners"]) == 4 and w["order_index"] in (0, 1, 2)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "stream.jsonl")
        export_stream.write_stream(p, [{
            "label_lines": lines, "timestamp_ns": T0_NS, "frame_id": 0,
            "position": list(map(float, scene["camera"]["position"])),
            "quat_xyzw": list(map(float, scene["camera"]["quat_xyzw"])),
        }], config)
        back = json.loads(Path(p).read_text().splitlines()[0])
        assert back["vision"]["windows"][0]["corners"]
    print("  [6 stream]     §5 msg via 길남 gt_stream (routed, not hand-rolled) OK")


if __name__ == "__main__":
    print("sim/ smoke test (pure logic, numpy+pyyaml only):")
    test_scene_sampler()
    test_label_lines()
    test_projection_cross_check()
    test_export_dataset_and_meta()
    test_export_vio()
    test_export_stream()
    print("ALL SIM SMOKE TESTS PASSED")
