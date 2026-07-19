"""Cross-module integration test (numpy + pyyaml only).

Exercises every module's pure/runnable logic together to prove they all resolve
`common/` and agree on the shared conventions (CONVENTIONS.md). This is what
should pass on any machine before touching Isaac Sim / a GPU.

Run:  python3 tests/test_integration.py        (from the repo root)
"""
import os
import sys
import tempfile

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def _camera_looking_down_world_x():
    """USD camera at origin whose forward (-Z_cam) points along world +X."""
    from common import make_transform

    Rwc = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]], float)  # -Z_cam -> +X world
    return make_transform(Rwc, [0, 0, 0])


def test_common_projection():
    from common import CameraIntrinsics, window_corners_world, world_to_camera_cv, project_points

    intr = CameraIntrinsics.from_fov(1280, 720, 90.0)
    assert abs(intr.fx - 640.0) < 1e-6 and intr.cx == 640 and intr.cy == 360
    Rww = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]], float)  # window faces -X (toward cam)
    corners = window_corners_world([4, 0, 0], Rww, 2.0, 1.5)
    uv, depth, in_front = project_points(
        world_to_camera_cv(corners, _camera_looking_down_world_x()), intr.K()
    )
    assert in_front.all() and np.allclose(depth, 4.0)
    u, v = uv[:, 0], uv[:, 1]
    assert u[0] < u[1] and u[3] < u[2] and v[0] < v[3] and v[1] < v[2]  # TL,TR,BR,BL order
    print("  [common]    projection + corner order OK")


def test_vision_labels_and_split():
    from sim.replicator_writer import build_label_lines, ORDER_INDEX
    from sim.export_dataset import split_indices
    from sim.visualize_labels import parse_label_line

    Rww = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]], float)
    windows = [
        {"color": "green", "center": [4, 0, 0], "R_world_win": Rww, "width": 2.0, "height": 1.5},
        {"color": "red", "center": [-4, 0, 0], "R_world_win": Rww, "width": 2.0, "height": 1.5},  # behind
    ]
    intr = __import__("common").CameraIntrinsics.from_fov(1280, 720, 90.0)
    lines = build_label_lines(windows, _camera_looking_down_world_x(), intr)
    assert len(lines) == 1, f"only the front window is fully visible, got {len(lines)}"
    toks = lines[0].split()
    assert len(toks) == 17, f"17-token YOLO-pose line, got {len(toks)}"
    assert int(toks[0]) == ORDER_INDEX["green"] == 1
    vals = [float(t) for t in toks[1:]]
    assert all(0.0 <= x <= 1.0 for x in vals), "all normalised coords in [0,1]"
    parsed = parse_label_line(lines[0])
    assert parsed is not None
    sp = split_indices(10, seed=42)
    assert (len(sp["train"]), len(sp["val"]), len(sp["test"])) == (8, 1, 1)
    assert sorted(sp["train"] + sp["val"] + sp["test"]) == list(range(10))
    print("  [vision]    label geometry + 80/10/10 split OK")


def test_interface_schemas():
    from interface.schemas import validate, is_valid, trajectory_frame_to_T, save_json, load_json

    motor = {"fps": 200.0, "frames": [{"time": 0, "motor_cmd_w": [1.0, 2.0, 3.0, 4.0]}]}
    traj = {
        "fps": 100.0,
        "frames": [{"time": 0, "position": [1.0, 2.0, 3.0], "yaw_rad": 0.0,
                    "orientation_quat_wxyz": [1.0, 0.0, 0.0, 0.0]}],
    }
    validate(motor, "motor")
    validate(traj, "trajectory")
    assert not is_valid({"fps": 200.0, "frames": [{"time": 0, "motor_cmd_w": [1, 2, 3]}]}, "motor")  # wrong len
    assert not is_valid({"fps": 100.0, "frames": [{"time": -1, "position": [0, 0, 0],
                        "yaw_rad": 0.0, "orientation_quat_wxyz": [1, 0, 0, 0]}]}, "trajectory")  # neg time
    T = trajectory_frame_to_T(traj["frames"][0])
    assert np.allclose(T[:3, :3], np.eye(3)) and np.allclose(T[:3, 3], [1, 2, 3])
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "t.json")
        save_json(traj, p)
        assert load_json(p)["fps"] == 100.0
    print("  [interface] schema validate + wxyz->T + round-trip OK")


def test_rl_env_and_baseline():
    from rl.window_env import WindowTraversalEnv, MockPhysics, OBS_DIM, OBS_REL_WIN_POS
    from rl.baseline import WaypointBaseline

    env = WindowTraversalEnv(backend=MockPhysics(), seed=0)
    obs, info = env.reset(seed=0)
    assert obs.shape == (OBS_DIM,)
    r = None
    for _ in range(50):  # random policy: env must keep stepping with finite rewards
        obs, r, term, trunc, info = env.step(env.action_space.sample() if hasattr(env, "action_space") else np.random.uniform(-1, 1, 3))
        assert obs.shape == (OBS_DIM,) and np.isfinite(r)
        if term or trunc:
            obs, info = env.reset()
    assert isinstance(info.get("reward_terms", {}), dict) and set(
        ["window_pass", "collision", "progress", "attitude", "energy"]
    ).issubset(info.get("reward_terms", {}).keys())

    # WaypointBaseline must make progress toward the window (distance decreases).
    env2 = WindowTraversalEnv(backend=MockPhysics(), seed=1)
    obs, _ = env2.reset(seed=1)
    d0 = float(np.linalg.norm(obs[OBS_REL_WIN_POS]))
    pol = WaypointBaseline(gain=1.0)
    for _ in range(60):
        obs, _, term, trunc, _ = env2.step(pol.act(obs))
        if term or trunc:
            break
    d1 = float(np.linalg.norm(obs[OBS_REL_WIN_POS]))
    assert d1 < d0, f"baseline should approach the window: {d0:.2f} -> {d1:.2f}"
    print(f"  [rl]        env step + reward terms + baseline progress OK ({d0:.2f}->{d1:.2f} m)")


if __name__ == "__main__":
    print("integration test (all modules, numpy+pyyaml only):")
    test_common_projection()
    test_vision_labels_and_split()
    test_interface_schemas()
    test_rl_env_and_baseline()
    print("ALL INTEGRATION TESTS PASSED")
