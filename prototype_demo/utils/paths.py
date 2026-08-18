"""경로 부트스트랩 — 데모 스크립트 4개에 복붙돼 있던 sys.path 삽입을 한 곳으로.

    from utils import paths; paths.bootstrap()

이 리포는 패키지가 아니라 폴더 4개가 각자 flat 하게 놓여 있어서 (overall_gilnam/vision,
overall_gilnam/planning, reinforcement_yunho, visual_imaging_taemin) import 하려면
sys.path 를 손대는 수밖에 없다. 팀 파일을 안 고치는 게 원칙이라 이 방식을 유지한다.
"""
from __future__ import annotations

import os
import sys

UTILS = os.path.dirname(os.path.abspath(__file__))
PROTO = os.path.dirname(UTILS)
TEAM = os.path.dirname(PROTO)

REPO = os.path.join(TEAM, "reinforcement_yunho")          # 내 PyBullet 코드
VISION = os.path.join(TEAM, "overall_gilnam", "vision")    # 길남 검출·복원
PLANNING = os.path.join(TEAM, "overall_gilnam", "planning")
INTEGRATION = os.path.join(TEAM, "overall_gilnam", "integration")
TAEMIN = os.path.join(TEAM, "visual_imaging_taemin")       # 태민 VIO 복원 노드
CONTROL = os.path.join(TEAM, "control_seoungjin")          # 성진 제어

MODEL_DIR = os.path.join(PROTO, "model")
CONFIG_DIR = os.path.join(PROTO, "config")
OUT_DIR = os.path.join(PROTO, "out")
FIG_DIR = "/home/yoonho/fig/6_render"

PLANNER_LIMITS = os.path.join(PLANNING, "planner_limits.yaml")
COLOR_ORDER = os.path.join(VISION, "color_order.yaml")
CAMERA_YAML = os.path.join(CONFIG_DIR, "camera.yaml")

_ALL = (PROTO, REPO, VISION, PLANNING, INTEGRATION)


def bootstrap(extra=()) -> None:
    """팀 폴더들을 sys.path 앞에 넣는다. 여러 번 불러도 안전."""
    for p in tuple(extra) + _ALL:
        if p not in sys.path:
            sys.path.insert(0, p)


def weights(name: str | None = None) -> str:
    """가중치 경로 해결. 이름만 주면 model/ 에서 찾고, 없으면 뭐가 있는지 알려준다.

    name 이 None 이면 기본 가중치(현재 v2)를 쓴다.
    """
    if name is None:
        name = DEFAULT_WEIGHTS
    cand = name if os.path.isabs(name) else os.path.join(MODEL_DIR, name)
    if os.path.exists(cand):
        return cand
    have = sorted(f for f in os.listdir(MODEL_DIR) if f.endswith(".pt")) \
        if os.path.isdir(MODEL_DIR) else []
    raise FileNotFoundError(
        f"가중치 없음: {cand}\n"
        f"  model/ 안에 있는 것: {have or '(없음)'}\n"
        f"  받는 법은 {os.path.join(MODEL_DIR, 'README.md')} 참고")


# 현재 기본값. 파인튜닝 세대가 바뀌면 여기만 고친다.
DEFAULT_WEIGHTS = "pyb_overlap_v2_best.pt"
BASELINE_WEIGHTS = "pyb_openframe_best.pt"    # v1 (겹침 커리큘럼 이전)
ORIGINAL_WEIGHTS = "window_yolo11s_best.pt"   # 팀 원본 single_cls
DEFAULT_POLICY = "ppo_window_3win.zip"
