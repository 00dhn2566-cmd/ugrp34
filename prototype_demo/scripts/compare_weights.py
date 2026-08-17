"""가중치 세대(v1 vs v2)를 같은 씬·같은 경로에서 여러 시드로 비교한다.

    python scripts/compare_weights.py --seeds 1 3 5 7 11

한 프로세스 안에서 두 모델을 번갈아 돌린다 (PyBullet/CUDA 기동 비용 1회).
복원은 태민 노드 원본 경로 — 검출기 차이만 보려는 것이므로 복원단은 고정한다.
복원 알고리즘 쪽 비교는 scripts/compare_recon.py.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils import paths  # noqa: E402

paths.bootstrap()

from module import taemin_bridge  # noqa: E402
from utils import device, metrics, scene  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 3, 5, 7, 11])
    ap.add_argument("--v1", default=paths.BASELINE_WEIGHTS)
    ap.add_argument("--v2", default=paths.DEFAULT_WEIGHTS)
    ap.add_argument("--n-windows", type=int, default=3)
    ap.add_argument("--frames-per-window", type=int, default=32)
    ap.add_argument("--mode", default="xy", choices=scene.PATH_MODES)
    ap.add_argument("--span", type=float, default=110.0)
    ap.add_argument("--clutter", type=int, default=18)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--det-conf-min", type=float, default=0.5)
    ap.add_argument("--device", default="auto", choices=("auto", "cuda", "cpu"))
    a = ap.parse_args(argv)

    models = {"v1": device.load_detector(a.v1, prefer=a.device),
              "v2": device.load_detector(a.v2, prefer=a.device)}
    rows = {k: [] for k in models}
    dets = {k: 0 for k in models}

    for seed in a.seeds:
        print(f"\n=== seed {seed} ===")
        for tag, det in models.items():
            env, layout = scene.make(seed=seed, n_windows=a.n_windows,
                                     clutter=a.clutter)
            poses, _ = scene.path(layout, mode=a.mode,
                                  n_per_window=a.frames_per_window, span_deg=a.span)
            samples, stats = taemin_bridge.observe(env, det, None, poses, conf=a.conf)
            env.close()
            results = taemin_bridge.run_offline(samples, verbose=False,
                                                det_conf_min=a.det_conf_min)
            sc = metrics.score(results, layout)
            rows[tag].append(sc)
            dets[tag] += stats["detections"]
            got = [r for r in sc if r["ok"]]
            desc = "  ".join(f"{r['color'][0]}={r['center_mm']:.0f}" if r["ok"]
                             else f"{r['color'][0]}=X" for r in sc)
            print(f"  {tag}  복원 {len(got)}/{len(sc)}  검출 {stats['detections']:4d}  "
                  f"center[mm] {desc}")

    print("\n" + "=" * 84)
    metrics.print_summary({t: metrics.aggregate(rows[t]) for t in models})
    print("=" * 84)
    for t in models:
        print(f"  {t}: 총 검출 {dets[t]}  ({os.path.basename(models[t].path)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
