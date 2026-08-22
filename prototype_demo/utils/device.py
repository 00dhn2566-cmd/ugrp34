"""디바이스 선택 + 검출기 로드.

    from utils import device
    det = device.load_detector("pyb_overlap_v2_best.pt", prefer="auto")

왜 래퍼를 두나
--------------
ultralytics 는 ``predict(device=...)`` 를 호출마다 받는데, 팀 코드
(``infer_stream.infer_frame``, ``pybullet_stream.infer_frame_multiclass``) 는
device 를 안 넘긴다. 그 파일들을 안 고치려면 모델 쪽에서 기본 device 를 물고
있어야 한다. ``Detector`` 는 ``.names`` / ``.predict`` 만 노출하므로 팀 코드에서
YOLO 객체와 구분 없이 쓰인다 (``is_multiclass`` 는 ``.names`` 만 본다).

실측 (MX450 2GB, 1280x720):
    cuda   58 ms/frame,  VRAM 110 MiB   <- 학습이 1.1GB 물고 있어도 같이 돎
    cpu   ~230 ms/frame
"""
from __future__ import annotations

import os
from typing import Any


def pick(prefer: str = "auto", verbose: bool = True) -> str:
    """'auto' | 'cuda' | 'cpu' → 실제 device 문자열. CUDA 없으면 조용히 cpu."""
    if prefer == "cpu":
        return "cpu"
    try:
        import torch
    except ImportError:
        return "cpu"
    if not torch.cuda.is_available():
        if prefer == "cuda" and verbose:
            print("[device] CUDA 요청했지만 사용 불가 — cpu 로 감")
        return "cpu"
    if verbose:
        free, total = torch.cuda.mem_get_info()
        print(f"[device] cuda:0 {torch.cuda.get_device_name(0)}  "
              f"여유 {free/2**20:.0f}/{total/2**20:.0f} MiB")
    return "cuda:0"


class Detector:
    """YOLO 래퍼 — device 를 물고 있고 나머지는 그대로 위임."""

    def __init__(self, weights_path: str, device: str = "cpu"):
        from ultralytics import YOLO
        self.path = str(weights_path)
        self.device = device
        self.model = YOLO(self.path)
        self.names = self.model.names

    def predict(self, *args, **kw):
        kw.setdefault("device", self.device)
        kw.setdefault("verbose", False)
        return self.model.predict(*args, **kw)

    def __getattr__(self, item) -> Any:     # .task, .overrides 등 나머지 위임
        return getattr(self.model, item)

    def __repr__(self) -> str:
        return f"Detector({os.path.basename(self.path)}, {self.device}, {self.names})"


def load_detector(weights: str | None = None, prefer: str = "auto",
                  verbose: bool = True) -> Detector:
    from . import paths
    w = paths.weights(weights)
    d = Detector(w, pick(prefer, verbose))
    if verbose:
        print(f"[weights] {os.path.basename(w)}  classes={d.names}")
    return d
