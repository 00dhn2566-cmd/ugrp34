"""학습 모델 추론 래퍼: YOLO-pose 검출 → color_judge → §5 메시지 (체크리스트 §1).

README 데이터 흐름의 [학습 후] 경로를 잇는다. ultralytics는 CLI/실시간 추론에서만
지연 import — 조립 로직(detections_to_windows)은 numpy+cv2만으로 테스트된다
(§5 규격 준수는 vision_msg가 보장).

정책 (model_decisions #4·#7):
- det_conf = 박스 conf (기하 신뢰도) / 색·순서·color_conf = HSV 후처리 (single_cls 모델)
- unknown 색(HSV 판정 실패) 검출은 드롭 — 통과 순서가 없는 창문은 §5 하류에 무의미
- 같은 색 중복 검출은 드롭하지 않음 — 선별은 하류(VIO 3D 복원) 몫
- corner_vis: 정책 A(전 corner 가시) 모델이므로 전부 1. 정책 C 전환 시 kpt conf 임계로 교체
추론 입력은 원본 1280x720 프레임 그대로 (#6 — corner 좌표는 원본 픽셀로 돌아온다).
"""

import argparse
import json
import time
from pathlib import Path

from color_judge import judge_color, load_color_config
from vision_msg import N_CORNERS, build_frame_message, build_window, to_json

DEFAULT_CONF = 0.25  # ultralytics 기본 박스 conf 임계 — CLI --conf로 조정


def detections_to_windows(det_confs, kpts_xy, frame_rgb, color_config):
    """검출 1프레임분 → §5 windows[] 리스트.

    det_confs: 검출별 박스 conf (n,)
    kpts_xy: 검출별 corner 픽셀 좌표 (n, 4, 2) — 좌상→우상→우하→좌하 (§4.3 학습 라벨 순서)
    frame_rgb: 원본 720p RGB 프레임 (색 판정은 증강·리사이즈가 닿지 않은 원본에서)
    """
    windows = []
    for conf, corners in zip(det_confs, kpts_xy):
        corners = [[float(u), float(v)] for u, v in corners]
        if len(corners) != N_CORNERS:
            raise ValueError(f"model must emit {N_CORNERS} keypoints per window, got {len(corners)}")
        color, order, color_conf = judge_color(frame_rgb, corners, color_config)
        if color is None:  # unknown 색 → 드롭 (정책)
            continue
        windows.append(build_window(order, color, corners, [1] * N_CORNERS, float(conf), color_conf))
    return windows


def load_model(weights_path):
    from ultralytics import YOLO  # 지연 import — 테스트·경량 사용 시 불필요

    return YOLO(str(weights_path))


def infer_frame(model, frame_rgb, timestamp_ns, frame_id, color_config, conf=DEFAULT_CONF):
    """프레임 1장 추론 → §5 메시지 dict. (ultralytics의 numpy 입력은 BGR 가정 → 채널 반전)"""
    result = model.predict(frame_rgb[:, :, ::-1], conf=conf, verbose=False)[0]
    if result.boxes is None or len(result.boxes) == 0:
        windows = []
    else:
        windows = detections_to_windows(
            result.boxes.conf.tolist(), result.keypoints.xy.tolist(), frame_rgb, color_config
        )
    return build_frame_message(timestamp_ns, frame_id, windows)


def main():
    import cv2

    ap = argparse.ArgumentParser(description="이미지 폴더 → §5 JSONL 스트림 (+추론 속도 벤치마크)")
    ap.add_argument("--model", required=True, help="학습 가중치 .pt (예: 윤호 반환물 best.pt)")
    ap.add_argument("--images", required=True, help="720p 프레임 이미지 폴더 (png/jpg, 이름순 재생)")
    ap.add_argument("--out", required=True, help="출력 §5 JSONL 경로")
    ap.add_argument("--fps", type=float, default=30.0, help="타임스탬프 부여용 가정 fps (기본 30)")
    ap.add_argument("--conf", type=float, default=DEFAULT_CONF)
    ap.add_argument("--color-config", default=str(Path(__file__).parent / "color_order.yaml"))
    args = ap.parse_args()

    frames = sorted(p for p in Path(args.images).iterdir() if p.suffix.lower() in (".png", ".jpg", ".jpeg"))
    if not frames:
        raise SystemExit(f"no images in {args.images}")
    model = load_model(args.model)
    color_config = load_color_config(args.color_config)

    infer_ms = []
    with open(args.out, "w", encoding="utf-8") as f:
        for i, path in enumerate(frames):
            frame_rgb = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
            t0 = time.perf_counter()
            msg = infer_frame(model, frame_rgb, round(i * 1e9 / args.fps), i, color_config, args.conf)
            infer_ms.append((time.perf_counter() - t0) * 1000)
            f.write(to_json(msg) + "\n")

    stats = sorted(infer_ms[1:] or infer_ms)  # 첫 프레임은 워밍업 — 통계에서 제외
    mean_ms = sum(stats) / len(stats)
    print(json.dumps({
        "frames": len(frames),
        "mean_ms": round(mean_ms, 1),
        "p95_ms": round(stats[int(len(stats) * 0.95) - 1], 1),
        "effective_fps": round(1000 / mean_ms, 1),  # scan.rate_rad_s 산정 입력(탐지 주기)
        "out": args.out,
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
