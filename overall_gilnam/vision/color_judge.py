"""HSV 규칙 기반 색 판정 → 통과 순서 식별 (spec §3, 체크리스트 §2).

판정 기준값은 전부 color_order.yaml에서 읽는다 — 하드코딩 금지.
샘플 영역은 corner 사각형의 안쪽 테두리 밴드: 창문은 개구부라 내부에는
배경이 비쳐 보이므로 프레임(테두리) 픽셀만 유효하다 (config sampling 참조).
"""

import cv2
import numpy as np
import yaml


def load_color_config(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _edge_band_mask(shape_hw, corners, band_px):
    """corner 폴리곤 경계에서 안쪽으로 band_px 폭의 밴드 마스크 (uint8 0/255)."""
    mask = np.zeros(shape_hw, dtype=np.uint8)
    pts = np.array(corners, dtype=np.int32)
    cv2.fillPoly(mask, [pts], 255)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * band_px + 1, 2 * band_px + 1))
    inner = cv2.erode(mask, kernel)
    return cv2.subtract(mask, inner)


def judge_color(rgb_frame, corners, config):
    """창문 1개의 색을 판정한다.

    rgb_frame: 원본 720p RGB 프레임 (§2 컬러 포맷)
    corners: [[u,v]]*4, 720p 픽셀 (좌상→우상→우하→좌하)
    returns: (color|None, order_index|None, color_conf)
             color_conf = 최다 득표 색의 inlier 비율 (§5 color_conf로 그대로 출력)
    """
    sampling = config["sampling"]
    band = _edge_band_mask(rgb_frame.shape[:2], corners, sampling["band_px"])
    pixels = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)[band > 0]
    if len(pixels) == 0:
        return None, None, 0.0

    h, s, v = pixels[:, 0].astype(int), pixels[:, 1].astype(int), pixels[:, 2].astype(int)
    sv_ok = (s >= config["hsv_min_s"]) & (v >= config["hsv_min_v"])

    best = (None, None, 0.0)
    for name, spec in config["colors"].items():
        h_ok = np.zeros(len(pixels), dtype=bool)
        for lo, hi in spec["h_ranges"]:
            h_ok |= (h >= lo) & (h <= hi)
        ratio = float(np.mean(h_ok & sv_ok))
        if ratio > best[2]:
            best = (name, spec["order_index"], ratio)

    if best[2] < sampling["min_inlier_ratio"]:
        return None, None, best[2]
    return best
