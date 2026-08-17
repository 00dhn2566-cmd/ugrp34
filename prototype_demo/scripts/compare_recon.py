"""태민 원본 복원 vs 우리 오버라이드를 **같은 관측 위에서** 비교한다.

    python scripts/compare_recon.py --seeds 1 3 5 7 11

렌더+검출은 시드당 한 번만 하고, 그 샘플을 복원 방식들에 나눠 먹인다. 그래야
차이가 검출 랜덤성이 아니라 복원 알고리즘에서 온 것임이 보장된다.

비교 대상
  taemin      그의 window_recon_node.py 원본 (스텁 ROS 위에서 그대로 구동)
  ours-same   우리 재구현, 설정을 원본과 동일하게 — 구현이 맞는지 검증용
  ours-dedup  + 프레임당 색별 top-1 (중복 표 제거)
  ours-conf   + conf 가중
  ours-full   + Huber IRLS 아웃라이어 제거
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils import paths  # noqa: E402

paths.bootstrap()

from module import taemin_bridge  # noqa: E402
from overrides import detections, recon_rays  # noqa: E402
from utils import device, metrics, scene  # noqa: E402

VARIANTS = ("taemin", "ours-same", "ours-dedup", "ours-conf", "ours-full")


def run_variant(name, samples, det_conf_min):
    if name == "taemin":
        return taemin_bridge.run_offline(samples, verbose=False,
                                         det_conf_min=det_conf_min)
    if name == "ours-same":
        return recon_rays.reconstruct_like_taemin(samples, det_conf_min=det_conf_min)
    if name == "ours-dedup":
        return recon_rays.reconstruct_like_taemin(
            detections.clean_samples(samples), det_conf_min=det_conf_min)
    if name == "ours-conf":
        return recon_rays.reconstruct(
            detections.clean_samples(samples), det_conf_min=det_conf_min,
            weight="conf", robust="none")
    if name == "ours-full":
        return recon_rays.reconstruct(
            detections.clean_samples(samples), det_conf_min=det_conf_min,
            weight="conf", robust="huber")
    raise ValueError(name)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 3, 5, 7, 11])
    ap.add_argument("--weights", default=None)
    ap.add_argument("--n-windows", type=int, default=3)
    ap.add_argument("--frames-per-window", type=int, default=32)
    ap.add_argument("--mode", default="xy", choices=scene.PATH_MODES)
    ap.add_argument("--span", type=float, default=110.0)
    ap.add_argument("--clutter", type=int, default=18)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--det-conf-min", type=float, default=0.5)
    ap.add_argument("--device", default="auto")
    a = ap.parse_args(argv)

    det = device.load_detector(a.weights, prefer=a.device)
    rows = {v: [] for v in VARIANTS}

    for seed in a.seeds:
        env, layout = scene.make(seed=seed, n_windows=a.n_windows, clutter=a.clutter)
        poses, pname = scene.path(layout, mode=a.mode,
                                  n_per_window=a.frames_per_window, span_deg=a.span)
        samples, stats = taemin_bridge.observe(env, det, None, poses, conf=a.conf)
        env.close()

        pre = detections.count(samples)
        post = detections.count(detections.clean_samples(samples))
        print(f"\n=== seed {seed} === {pname}, {stats['frames']} 프레임")
        print(f"  검출 {pre['detections']} (중복 표 {pre['duplicate_votes']}) "
              f"-> 정리 후 {post['detections']}")

        for v in VARIANTS:
            r = run_variant(v, samples, a.det_conf_min)
            sc = metrics.score(r, layout)
            rows[v].append(sc)
            got = [x for x in sc if x["ok"]]
            desc = "  ".join(f"{x['color'][0]}={x['center_mm']:.0f}" if x["ok"]
                             else f"{x['color'][0]}=X" for x in sc)
            print(f"  {v:11s} 복원 {len(got)}/{len(sc)}  center[mm] {desc}")

    print("\n" + "=" * 84)
    metrics.print_summary({v: metrics.aggregate(rows[v]) for v in VARIANTS})
    print("=" * 84)
    print("ours-same 가 taemin 과 거의 같아야 재구현이 맞는 것 "
          "(pose 보간 경로가 달라 완전 일치는 아님)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
