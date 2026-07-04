# 토이 데이터셋 생성기 + corner 평가 코어 테스트 (ultralytics 불필요)
import json
from pathlib import Path

import cv2
import pytest

from color_judge import judge_color, load_color_config
from eval_corners import evaluate
from gt_stream import parse_label_line
from make_toy_dataset import generate_toy_dataset

VISION_DIR = Path(__file__).resolve().parents[1]
CONFIG = load_color_config(VISION_DIR / "color_order.yaml")


@pytest.fixture(scope="session")
def toy_dir(tmp_path_factory):
    out = tmp_path_factory.mktemp("toy")
    generate_toy_dataset(seeds=[11, 22], frames_per_scene=8, out_dir=out)
    return out


def _all_label_files(toy_dir):
    files = sorted((toy_dir / "labels").rglob("*.txt"))
    assert files, "라벨이 생성되어야 함"
    return files


def test_image_label_config_round_trip(toy_dir):
    # 이미지↔라벨↔config 3자 정합: 라벨 corner로 렌더 이미지를 색 판정하면
    # 라벨 class와 같은 order가 나와야 한다. 테두리 밴드가 가장 넉넉한
    # (bbox 최대) 창문을 골라 판정한다.
    best = None  # (bbox_area, label_file, line)
    for lf in _all_label_files(toy_dir):
        for line in lf.read_text().splitlines():
            f = line.split()
            area = float(f[3]) * float(f[4])
            if best is None or area > best[0]:
                best = (area, lf, line)
    _, label_file, line = best
    img_path = Path(str(label_file.with_suffix(".png")).replace("labels", "images", 1))
    rgb = cv2.cvtColor(cv2.imread(str(img_path)), cv2.COLOR_BGR2RGB)

    gt = parse_label_line(line)
    color, order, conf = judge_color(rgb, gt["corners"], CONFIG)
    assert order == gt["order_index"]
    assert CONFIG["colors"][color]["order_index"] == gt["order_index"]


def test_labels_parse_and_fully_visible(toy_dir):
    # 정책 A: 완전 가시 창문만 라벨링했으므로 정규화 좌표는 전부 [0,1]
    for lf in _all_label_files(toy_dir):
        for line in lf.read_text().splitlines():
            gt = parse_label_line(line)  # §4.3 파서 통과 (재구현 금지 — 기존 파서)
            assert gt["order_index"] in {0, 1, 2}
            assert gt["corner_vis"] == [1, 1, 1, 1]
            fields = [float(x) for x in line.split()[1:]]
            assert all(0.0 <= v <= 1.0 for v in fields)


GT = [[100.0, 100.0], [200.0, 100.0], [200.0, 200.0], [100.0, 200.0]]


def _rec(pred, dist):
    return {"gt_corners": GT, "pred_corners": pred, "distance_m": dist}


def test_evaluate_zero_error():
    result = evaluate([_rec(GT, 4.0)])
    assert result["overall"]["mean_px"] == 0.0


def test_evaluate_known_shift():
    shifted = [[u + 3.0, v] for u, v in GT]
    result = evaluate([_rec(shifted, 4.0)])
    assert abs(result["overall"]["mean_px"] - 3.0) < 1e-6


def test_evaluate_distance_binning():
    result = evaluate([_rec(GT, 2.0), _rec(GT, 4.5), _rec(GT, 8.0)], bins=(3.0, 6.0))
    assert result["bins"]["<3m"]["n"] == 1
    assert result["bins"]["3-6m"]["n"] == 1
    assert result["bins"][">6m"]["n"] == 1
