# §5 메시지 규격(window_detection_spec_v0.2) 준수 테스트
import json

import pytest

from vision_msg import build_frame_message, build_window, to_json

CORNERS = [[100.0, 200.0], [300.0, 200.0], [300.0, 400.0], [100.0, 400.0]]


def test_build_window_has_all_spec_fields():
    w = build_window(
        order_index=0,
        color="red",
        corners=CORNERS,
        corner_vis=[1, 1, 1, 1],
        det_conf=0.97,
        color_conf=0.88,
    )
    assert w["order_index"] == 0
    assert w["color"] == "red"
    assert w["corners"] == CORNERS
    assert w["corner_vis"] == [1, 1, 1, 1]
    assert w["det_conf"] == 0.97
    assert w["color_conf"] == 0.88
    # center = corner 4점 평균 (model_decisions #7)
    assert w["center"] == [200.0, 300.0]


def test_build_window_rejects_bad_corner_count():
    with pytest.raises(ValueError):
        build_window(0, "red", CORNERS[:3], [1, 1, 1], 0.9, 0.9)


def test_build_window_rejects_out_of_range_conf():
    with pytest.raises(ValueError):
        build_window(0, "red", CORNERS, [1, 1, 1, 1], 1.5, 0.9)


def test_build_window_rejects_bad_visibility():
    with pytest.raises(ValueError):
        build_window(0, "red", CORNERS, [1, 2, 1, 1], 0.9, 0.9)


def test_frame_message_structure_and_empty_windows():
    msg = build_frame_message(timestamp_ns=1234567890123456789, frame_id=42, windows=[])
    assert msg["timestamp"] == 1234567890123456789
    assert msg["frame_id"] == 42
    assert msg["windows"] == []


def test_frame_message_rejects_non_int_timestamp():
    with pytest.raises(ValueError):
        build_frame_message(timestamp_ns=1.5e18, frame_id=0, windows=[])


def test_to_json_round_trip_keeps_ns_timestamp_exact():
    w = build_window(2, "blue", CORNERS, [1, 1, 0, 1], 0.5, 0.6)
    msg = build_frame_message(1234567890123456789, 7, [w])
    parsed = json.loads(to_json(msg))
    assert parsed["timestamp"] == 1234567890123456789  # int 정밀도 유지 (float화 금지)
    assert parsed["windows"][0]["corner_vis"] == [1, 1, 0, 1]
