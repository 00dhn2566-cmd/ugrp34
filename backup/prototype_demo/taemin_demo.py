"""PyBullet 데모 -> 태민 window_recon_node.py (수정 0) -> 창문 3D.

    python taemin_demo.py                 # offline: ROS2 없이 그의 코드 그대로 구동
    python taemin_demo.py --ros           # ROS2 토픽 발행 (그의 노드는 별도 터미널)
    python taemin_demo.py --override      # overrides/recon_rays.py 로 복원 (비교용)

offline 모드가 검증하는 것: 그의 삼각측량·시차각 판정·리포트 코드가 우리 PyBullet
관측으로 올바른 창문 3D 를 내는가.  검증하지 않는 것: DDS 통신/타이머/QoS.
"""
from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from utils import paths  # noqa: E402

paths.bootstrap()

from module import contract, taemin_bridge  # noqa: E402
from utils import device, metrics, scene  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--weights", default=None, help="model/ 안 파일명 또는 절대경로")
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--n-windows", type=int, default=3)
    ap.add_argument("--opening", type=float, default=1.0)
    ap.add_argument("--frames-per-window", type=int, default=24)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--clutter", type=int, default=18)
    ap.add_argument("--walls", action="store_true", help="창문을 벽에 뚫린 구멍으로")
    ap.add_argument("--mode", default="sweep", choices=scene.PATH_MODES,
                    help="관측 경로 (xy=원호스윕, sweep=횡스윕, scan=전진+yaw)")
    ap.add_argument("--xy", action="store_true", help="--mode xy 단축")
    ap.add_argument("--scan", action="store_true", help="--mode scan 단축")
    ap.add_argument("--span", type=float, default=110.0, help="xy 스윕 각도 폭 [deg]")
    ap.add_argument("--radius", type=float, default=2.0, help="xy 스윕 반경 [m]")
    ap.add_argument("--scale", type=float, default=1.0, help="렌더 축소 배율 (속도)")
    ap.add_argument("--device", default="auto", choices=("auto", "cuda", "cpu"))
    ap.add_argument("--ros", action="store_true", help="ROS2 토픽으로 발행 (노드는 별도 실행)")
    ap.add_argument("--pose-rate", type=float, default=200.0)
    ap.add_argument("--det-conf-min", type=float, default=None,
                    help="태민 노드 DET_CONF_MIN 런타임 오버라이드 (그의 파일은 불변)")
    ap.add_argument("--override", action="store_true",
                    help="overrides/recon_rays.py 로 복원 (conf 가중 + Huber IRLS)")
    a = ap.parse_args(argv)

    mode = "xy" if a.xy else "scan" if a.scan else a.mode

    try:
        weights = paths.weights(a.weights)
    except FileNotFoundError as e:
        print(f"[error] {e}", file=sys.stderr)
        return 2

    intr = contract.intrinsics()
    c = contract.read_constants()
    print("태민 노드 계약 (그의 소스에서 직접 읽음)")
    print(f"  intrinsics   fx={intr['fx']:.0f} fy={intr['fy']:.0f} "
          f"cx={intr['cx']:.0f} cy={intr['cy']:.0f}   <- 우리가 여기에 맞춰 렌더")
    print(f"  det_conf >=  {c['DET_CONF_MIN']}")
    print(f"  시차각   >=  {c['MIN_PARALLAX_DEG']} deg")
    print(f"  pose 매칭    {c['POSE_TOL_NS']/1e6:.0f} ms")
    print(f"  T_IC         카메라-IMU 외부파라미터 적용 (IMU pose 로 변환해서 전달)\n")

    det = device.load_detector(weights, prefer=a.device)
    env, layout = scene.make(seed=a.seed, n_windows=a.n_windows, clutter=a.clutter,
                             walls=a.walls, opening=a.opening)
    scene.print_layout(layout)

    poses, path_name = scene.path(layout, mode=mode, n_per_window=a.frames_per_window,
                                  span_deg=a.span, radius=a.radius)
    print(f"경로: {path_name}  ({len(poses)} 프레임, render scale {a.scale})")
    samples, stats = taemin_bridge.observe(env, det, None, poses,
                                           conf=a.conf, scale=a.scale)
    print(f"\n관측  frames={stats['frames']}  detections={stats['detections']}")
    env.close()

    if a.ros:
        print("\nROS2 발행 시작 — 다른 터미널에서 그의 노드를 띄워두세요:")
        print(f"  python {os.path.join(paths.TAEMIN, 'window_recon_node.py')}")
        taemin_bridge.run_ros_publisher(samples, pose_rate_hz=a.pose_rate)
        return 0

    if a.override:
        from overrides import detections, recon_rays
        pre, post = detections.count(samples), None
        samples_c = detections.clean_samples(samples)
        post = detections.count(samples_c)
        print(f"\noverrides/detections: 중복 표 {pre['duplicate_votes']}개 제거 "
              f"({pre['detections']} -> {post['detections']} 검출)")
        print("overrides/recon_rays 구동 (conf 가중 + Huber IRLS):")
        results = recon_rays.reconstruct(
            samples_c, det_conf_min=(a.det_conf_min if a.det_conf_min is not None
                                     else c["DET_CONF_MIN"]))
        src = "overrides/recon_rays.py"
    else:
        print("\n태민 노드 구동 (offline, 그의 코드 그대로):")
        results = taemin_bridge.run_offline(samples, pose_rate_hz=a.pose_rate,
                                            det_conf_min=a.det_conf_min)
        src = f"그의 노드 출력 ({contract.TOPIC_POSITIONS})"

    if not results:
        print("\n복원된 창문 없음")
        return 1

    print(f"\n{src} vs ground truth:")
    rows = metrics.score(results, layout)
    ok = metrics.print_rows(rows)
    if a.override:
        for r in results:
            if "inlier_frac" in r:
                print(f"    #{r['order_index']} 인라이어 {r['inlier_frac']*100:.0f}% "
                      f"({r['n_rejected']}개 거절)")
    return 0 if ok == len(layout) else 1


if __name__ == "__main__":
    raise SystemExit(main())
