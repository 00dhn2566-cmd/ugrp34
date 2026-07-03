# GT 라벨(YOLO-pose txt, §4.3) → §5 메시지 어댑터 테스트 — spec §4.4
from pathlib import Path

import pytest

from color_judge import load_color_config
from gt_stream import labels_to_message, parse_label_line

CONFIG = load_color_config(Path(__file__).resolve().parents[1] / "color_order.yaml")

# class=1(green), bbox 중심(0.5,0.5) 크기(0.25,0.5),
# corners 정규화: (0.375,0.25) (0.625,0.25) (0.625,0.75) (0.375,0.75), 전부 vis=1
LINE = "1 0.5 0.5 0.25 0.5 0.375 0.25 1 0.625 0.25 1 0.625 0.75 1 0.375 0.75 1"


def test_parse_label_line_denormalizes_to_720p():
    w = parse_label_line(LINE)
    assert w["order_index"] == 1
    assert w["corners"] == [[480.0, 180.0], [800.0, 180.0], [800.0, 540.0], [480.0, 540.0]]
    assert w["corner_vis"] == [1, 1, 1, 1]


def test_parse_label_line_rejects_wrong_field_count():
    with pytest.raises(ValueError):
        parse_label_line("1 0.5 0.5 0.25 0.5 0.375 0.25 1")


def test_labels_to_message_is_spec5_compliant():
    msg = labels_to_message([LINE], timestamp_ns=123456789, frame_id=3, config=CONFIG)
    assert msg["timestamp"] == 123456789
    assert msg["frame_id"] == 3
    w = msg["windows"][0]
    assert w["order_index"] == 1
    assert w["color"] == "green"  # order_index → color 역매핑 (config 기준)
    assert w["det_conf"] == 1.0 and w["color_conf"] == 1.0  # GT는 신뢰도 1.0
    assert w["center"] == [640.0, 360.0]


def test_labels_to_message_multiple_and_blank_lines():
    line_red = "0 0.5 0.5 0.25 0.5 0.375 0.25 1 0.625 0.25 1 0.625 0.75 0 0.375 0.75 1"
    msg = labels_to_message([LINE, "", line_red, "  "], timestamp_ns=1, frame_id=0, config=CONFIG)
    assert len(msg["windows"]) == 2
    assert msg["windows"][1]["color"] == "red"
    assert msg["windows"][1]["corner_vis"] == [1, 1, 0, 1]


def test_labels_to_message_empty_frame():
    msg = labels_to_message([], timestamp_ns=1, frame_id=0, config=CONFIG)
    assert msg["windows"] == []
