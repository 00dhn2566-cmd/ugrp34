# HSV 색 판정(color_order.yaml 기반) 테스트 — 체크리스트 §2, spec §3.1
from pathlib import Path

import cv2
import numpy as np

from color_judge import judge_color, load_color_config

CONFIG_PATH = Path(__file__).resolve().parents[1] / "color_order.yaml"

# 720p 프레임 안의 창문 corner (좌상→우상→우하→좌하)
CORNERS = [[400, 200], [800, 200], [800, 500], [400, 500]]


def make_frame(border_hsv, interior_hsv=(90, 20, 200), thickness=14):
    """창문 테두리(개구부 프레임)만 border 색, 내부는 배경색인 720p RGB 프레임."""
    hsv = np.zeros((720, 1280, 3), dtype=np.uint8)
    hsv[:] = (30, 30, 120)  # 벽 배경: 저채도
    pts = np.array(CORNERS, dtype=np.int32)
    cv2.fillPoly(hsv, [pts], interior_hsv)  # 개구부 내부로 비쳐 보이는 배경
    cv2.polylines(hsv, [pts], isClosed=True, color=border_hsv, thickness=thickness)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


def test_load_config_matches_spec_table():
    cfg = load_color_config(CONFIG_PATH)
    orders = {name: c["order_index"] for name, c in cfg["colors"].items()}
    assert orders == {"red": 0, "green": 1, "blue": 2}


def test_red_window():
    color, order, conf = judge_color(make_frame((5, 255, 255)), CORNERS, load_color_config(CONFIG_PATH))
    assert (color, order) == ("red", 0)
    assert conf > 0.9


def test_red_hue_wraparound():
    # 빨강은 H 원형 경계(170~179) 구간도 인정해야 한다
    color, order, _ = judge_color(make_frame((175, 255, 255)), CORNERS, load_color_config(CONFIG_PATH))
    assert (color, order) == ("red", 0)


def test_green_window():
    color, order, _ = judge_color(make_frame((60, 255, 255)), CORNERS, load_color_config(CONFIG_PATH))
    assert (color, order) == ("green", 1)


def test_blue_window():
    color, order, _ = judge_color(make_frame((120, 255, 255)), CORNERS, load_color_config(CONFIG_PATH))
    assert (color, order) == ("blue", 2)


def test_low_saturation_returns_unknown():
    # 회색 테두리: 어느 색 구간도 통과 못함 → unknown
    color, order, conf = judge_color(make_frame((60, 30, 200)), CORNERS, load_color_config(CONFIG_PATH))
    assert color is None
    assert order is None
    assert conf < 0.5


def test_band_sampling_ignores_interior_background():
    # 내부(개구부로 비치는 배경)가 파란색이어도 테두리가 빨강이면 red여야 한다.
    # bbox/폴리곤 전체 샘플링이면 면적이 큰 내부 파랑이 이긴다 → 테두리 밴드 샘플링 검증.
    frame = make_frame(border_hsv=(5, 255, 255), interior_hsv=(120, 255, 255))
    color, order, _ = judge_color(frame, CORNERS, load_color_config(CONFIG_PATH))
    assert (color, order) == ("red", 0)
