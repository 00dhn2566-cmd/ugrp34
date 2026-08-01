# 추론 래퍼 조립 로직 테스트 — 체크리스트 §1 (ultralytics 불필요: 검출 결과를 배열로 모사)
from pathlib import Path

import cv2
import numpy as np

from color_judge import load_color_config
from infer_stream import detections_to_windows
from vision_msg import build_frame_message

CONFIG = load_color_config(Path(__file__).resolve().parents[1] / "color_order.yaml")

GREEN_CORNERS = [[400, 200], [800, 200], [800, 500], [400, 500]]
RED_CORNERS = [[950, 150], [1150, 150], [1150, 400], [950, 400]]


def make_frame(windows, thickness=14):
    """[(corners, border_hsv)] 목록으로 720p RGB 프레임 생성 (테두리만 색, 내부는 배경)."""
    hsv = np.zeros((720, 1280, 3), dtype=np.uint8)
    hsv[:] = (30, 30, 120)
    for corners, border in windows:
        pts = np.array(corners, dtype=np.int32)
        cv2.fillPoly(hsv, [pts], (90, 20, 200))
        cv2.polylines(hsv, [pts], isClosed=True, color=border, thickness=thickness)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


def test_known_color_detection_becomes_window():
    frame = make_frame([(GREEN_CORNERS, (60, 255, 255))])
    windows = detections_to_windows([np.float32(0.91)], [GREEN_CORNERS], frame, CONFIG)
    assert len(windows) == 1
    w = windows[0]
    assert (w["color"], w["order_index"]) == ("green", 1)
    assert abs(w["det_conf"] - 0.91) < 1e-6  # 박스 conf가 det_conf로 (float32→float 변환 오차 허용)
    assert w["corner_vis"] == [1, 1, 1, 1]  # 정책 A: 전 corner 가시
    assert w["color_conf"] > 0.9


def test_unknown_color_detection_is_dropped():
    # 회색 테두리(저채도) → HSV 판정 실패 → 드롭 정책
    frame = make_frame([(GREEN_CORNERS, (60, 30, 200))])
    assert detections_to_windows([0.9], [GREEN_CORNERS], frame, CONFIG) == []


def test_multiple_windows_pass_through():
    frame = make_frame([(GREEN_CORNERS, (60, 255, 255)), (RED_CORNERS, (5, 255, 255))])
    windows = detections_to_windows([0.8, 0.7], [GREEN_CORNERS, RED_CORNERS], frame, CONFIG)
    assert {w["order_index"] for w in windows} == {0, 1}


def test_windows_feed_frame_message():
    frame = make_frame([(RED_CORNERS, (5, 255, 255))])
    windows = detections_to_windows([0.85], [RED_CORNERS], frame, CONFIG)
    msg = build_frame_message(123_000_000, 7, windows)
    assert msg["frame_id"] == 7 and len(msg["windows"]) == 1
    assert msg["windows"][0]["color"] == "red"
