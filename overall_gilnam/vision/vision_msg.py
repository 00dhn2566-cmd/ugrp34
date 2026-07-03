"""§5 VIO 전달 메시지 빌더 (window_detection_spec_v0.2).

비전 출력(모델·GT 공통)이 이 모듈을 거쳐 나가면 §5 규격 준수가 보장된다.
좌표는 항상 원본 720p 픽셀 기준(§2) — 이 모듈은 값을 변환하지 않고 검증만 한다.
"""

import json

N_CORNERS = 4  # 좌상→우상→우하→좌하 (§4.3)


def build_window(order_index, color, corners, corner_vis, det_conf, color_conf):
    """§5.1 windows[] 원소 1개를 만든다. center는 corner 4점 평균."""
    if len(corners) != N_CORNERS or any(len(pt) != 2 for pt in corners):
        raise ValueError(f"corners must be {N_CORNERS} [u,v] points, got {corners!r}")
    if len(corner_vis) != N_CORNERS or any(v not in (0, 1) for v in corner_vis):
        raise ValueError(f"corner_vis must be {N_CORNERS} flags of 0/1, got {corner_vis!r}")
    for name, conf in (("det_conf", det_conf), ("color_conf", color_conf)):
        if not 0.0 <= conf <= 1.0:
            raise ValueError(f"{name} must be in [0,1], got {conf}")
    center = [
        sum(pt[0] for pt in corners) / N_CORNERS,
        sum(pt[1] for pt in corners) / N_CORNERS,
    ]
    return {
        "order_index": int(order_index),
        "color": color,
        "corners": [[float(u), float(v)] for u, v in corners],
        "corner_vis": list(corner_vis),
        "center": center,
        "det_conf": float(det_conf),
        "color_conf": float(color_conf),
    }


def build_frame_message(timestamp_ns, frame_id, windows):
    """§5.1 프레임 단위 메시지. timestamp는 ns 단위 int (float 금지 — 정밀도 손실)."""
    if not isinstance(timestamp_ns, int):
        raise ValueError(f"timestamp_ns must be int (ns), got {type(timestamp_ns).__name__}")
    return {
        "timestamp": timestamp_ns,
        "frame_id": int(frame_id),
        "windows": list(windows),
    }


def to_json(msg):
    return json.dumps(msg, ensure_ascii=False)
