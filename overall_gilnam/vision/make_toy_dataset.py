"""합성 씬 → 학습 리허설용 토이 데이터셋 생성 CLI.

    python make_toy_dataset.py --seeds 11,22,33,44 --frames-per-scene 30 --out window_dataset/toy/

목적: 윤호의 실데이터 도착 전에 데이터→학습→평가 루프를 통째로 리허설한다.
씬·투영·라벨 직렬화는 전부 synth_scene의 기존 함수 재사용 (재구현 금지).

정책 A 정합: 4 corner가 모두 vis=1인 창문만 "그리고" "라벨링"한다 —
부분 가시 창문은 이미지에도 넣지 않아 이미지·라벨이 완전히 일치한다.

렌더 규칙: 배경·클러터는 S < hsv_min_s(color_order.yaml)로 제한해 색 판정
오답 소스를 원천 차단. 창문 테두리 색은 같은 config의 order별 첫 h_range
중앙 H (config가 곧 렌더 기준 — spec §3.1 단일 기준 원칙).
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from color_judge import load_color_config
from synth_scene import (
    IMG_H,
    IMG_W,
    load_intrinsics,
    make_scene,
    make_trajectory,
    project,
    to_label_lines,
)

VISION_DIR = Path(__file__).resolve().parent
SPLITS = ("train", "val", "test")  # 80/10/10 (§4.3)
BORDER_FRAC = 0.08          # 테두리 두께 = 투영 짧은 변의 8%
BORDER_MIN, BORDER_MAX = 4, 20  # px 클램프
LOW_SAT_MARGIN = 20         # 배경·클러터 S 상한 = hsv_min_s - margin


def _order_border_hsv(config):
    """order_index → 렌더 테두리 HSV (첫 h_range 중앙, S=V=255)."""
    out = {}
    for spec in config["colors"].values():
        lo, hi = spec["h_ranges"][0]
        out[spec["order_index"]] = (int((lo + hi) // 2), 255, 255)
    return out


def _render_frame(visible, scene, pose, config, rng):
    """완전 가시 창문만 그린 720p RGB 프레임 (visible: project() 결과 필터본)."""
    max_s = config["hsv_min_s"] - LOW_SAT_MARGIN
    hsv = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
    hsv[:] = (rng.integers(0, 180), rng.integers(0, max_s), rng.integers(60, 200))
    for _ in range(rng.integers(2, 5)):  # 저채도 클러터 사각형 2~4개
        x, y = rng.integers(0, IMG_W - 40), rng.integers(0, IMG_H - 40)
        w, h = rng.integers(40, 400), rng.integers(40, 300)
        color = (int(rng.integers(0, 180)), int(rng.integers(0, max_s)), int(rng.integers(40, 230)))
        cv2.rectangle(hsv, (int(x), int(y)), (int(min(x + w, IMG_W)), int(min(y + h, IMG_H))), color, -1)

    border = _order_border_hsv(config)
    centers = {w["order_index"]: w["center"] for w in scene["windows"]}
    # 먼 창문부터 그려 가까운 창문이 위에 오게 (자연스러운 가림)
    for p in sorted(visible, key=lambda p: -np.linalg.norm(centers[p["order_index"]] - pose["position"])):
        pts = np.asarray(p["corners_px"], dtype=np.int32)
        sides = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
        thickness = int(np.clip(min(sides) * BORDER_FRAC, BORDER_MIN, BORDER_MAX))
        cv2.polylines(hsv, [pts], isClosed=True, color=border[p["order_index"]], thickness=thickness)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


def _write_toy_yaml(out_dir, n_images):
    """window_pose.yaml의 실제 구조(kpt_shape·flip_idx·names)를 복사하고
    path만 생성 시점 절대경로로 기입 — 실 config를 그대로 리허설.
    절대경로 포함이라 데이터와 함께 gitignore됨 (window_dataset/ 규칙)."""
    src = yaml.safe_load((VISION_DIR / "window_pose.yaml").read_text(encoding="utf-8"))
    toy = {
        "path": str(Path(out_dir).resolve()),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "kpt_shape": src["kpt_shape"],
        "flip_idx": src["flip_idx"],
        "names": src["names"],
    }
    header = (f"# window_pose_toy.yaml — 학습 리허설용 (토이 {n_images}장, make_toy_dataset.py 생성)\n"
              f"# kpt_shape/flip_idx/names는 window_pose.yaml에서 복사 — path만 로컬 절대경로\n")
    (Path(out_dir) / "window_pose_toy.yaml").write_text(
        header + yaml.safe_dump(toy, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )


def generate_toy_dataset(seeds, frames_per_scene, out_dir):
    """토이 데이터셋 생성. 반환: {split: 이미지 수}."""
    out = Path(out_dir)
    intr = load_intrinsics(VISION_DIR / "synth_intrinsics.yaml")
    config = load_color_config(VISION_DIR / "color_order.yaml")

    frames = []  # (name, rgb, label_lines, meta_windows)
    for seed in seeds:
        scene = make_scene(seed)
        traj = make_trajectory(scene)
        centers = {w["order_index"]: w["center"] for w in scene["windows"]}
        idxs = sorted(set(np.linspace(0, len(traj) - 1, frames_per_scene).round().astype(int)))
        rng = np.random.default_rng(seed)  # 렌더 랜덤도 시드 고정 (재현성)
        for i in idxs:
            pose = traj[i]
            visible = [p for p in project(scene, pose, intr["K"])
                       if all(v == 1 for v in p["vis"])]  # 정책 A: 완전 가시만
            rgb = _render_frame(visible, scene, pose, config, rng)
            meta_windows = [
                {"order_index": p["order_index"],
                 "distance_m": round(float(np.linalg.norm(centers[p["order_index"]] - pose["position"])), 3)}
                for p in visible
            ]
            frames.append((f"s{seed}_f{i:04d}", rgb, to_label_lines(visible), meta_windows))

    # 80/10/10 분할 (시드 고정 셔플)
    order = np.random.default_rng(0).permutation(len(frames))
    n = len(frames)
    n_val = n_test = max(1, round(n * 0.1))
    split_of = {}
    for rank, fi in enumerate(order):
        split_of[fi] = "val" if rank < n_val else "test" if rank < n_val + n_test else "train"

    for split in SPLITS:
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)
    counts = dict.fromkeys(SPLITS, 0)
    import json

    with open(out / "meta.jsonl", "w", encoding="utf-8") as meta_f:
        for fi, (name, rgb, lines, meta_windows) in enumerate(frames):
            split = split_of[fi]
            counts[split] += 1
            cv2.imwrite(str(out / "images" / split / f"{name}.png"),
                        cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))  # imwrite는 BGR
            (out / "labels" / split / f"{name}.txt").write_text(
                "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8"
            )
            meta_f.write(json.dumps(
                {"image": f"images/{split}/{name}.png", "windows": meta_windows},
                ensure_ascii=False) + "\n")

    _write_toy_yaml(out, n)
    return counts


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="학습 리허설용 토이 데이터셋 생성")
    parser.add_argument("--seeds", type=str, default="11,22,33,44")
    parser.add_argument("--frames-per-scene", type=int, default=30)
    parser.add_argument("--out", type=str, default="window_dataset/toy")
    args = parser.parse_args()
    counts = generate_toy_dataset(
        seeds=[int(s) for s in args.seeds.split(",")],
        frames_per_scene=args.frames_per_scene,
        out_dir=args.out,
    )
    print(f"생성 완료: {counts} → {args.out}")
