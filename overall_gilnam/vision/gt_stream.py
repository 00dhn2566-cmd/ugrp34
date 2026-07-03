"""GT 스트림 어댑터: YOLO-pose 라벨(§4.3) → §5 메시지 (spec §4.4).

모델 학습 완료 전 파이프라인 검증용 — 시뮬 GT corner를 §5 규격 그대로
흘려보내므로, 나중에 모델로 교체해도 하류(VIO) 수정이 없다.
GT는 정답이므로 det_conf = color_conf = 1.0.
"""

from vision_msg import build_frame_message, build_window

IMG_W, IMG_H = 1280, 720  # §2 원본 해상도 — 좌표는 항상 이 기준
N_FIELDS = 5 + 4 * 3  # class + bbox(4) + corner(u,v,vis)*4


def parse_label_line(line, img_w=IMG_W, img_h=IMG_H):
    """§4.3 라벨 1행(정규화)을 720p 픽셀 좌표로 되돌린다."""
    fields = line.split()
    if len(fields) != N_FIELDS:
        raise ValueError(f"label line must have {N_FIELDS} fields, got {len(fields)}: {line!r}")
    order_index = int(fields[0])
    kpts = [float(x) for x in fields[5:]]
    corners = [[kpts[i * 3] * img_w, kpts[i * 3 + 1] * img_h] for i in range(4)]
    corner_vis = [int(kpts[i * 3 + 2]) for i in range(4)]
    return {"order_index": order_index, "corners": corners, "corner_vis": corner_vis}


def labels_to_message(lines, timestamp_ns, frame_id, config):
    """라벨 행 목록(빈 행 무시) → §5 프레임 메시지."""
    order_to_color = {c["order_index"]: name for name, c in config["colors"].items()}
    windows = []
    for line in lines:
        if not line.strip():
            continue
        gt = parse_label_line(line)
        windows.append(
            build_window(
                order_index=gt["order_index"],
                color=order_to_color[gt["order_index"]],
                corners=gt["corners"],
                corner_vis=gt["corner_vis"],
                det_conf=1.0,
                color_conf=1.0,
            )
        )
    return build_frame_message(timestamp_ns, frame_id, windows)
