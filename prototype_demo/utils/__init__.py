"""prototype_demo 공통 유틸.

데모 스크립트 4개(pipeline_demo / taemin_demo / render_status / rl_demo)에 복붙돼
있던 것들을 모았다. 팀 파일(overall_gilnam, visual_imaging_taemin,
control_seoungjin)은 여전히 **읽기 전용**이다 — 그쪽을 우리 입맛대로 바꾼 버전은
``overrides/`` 에 따로 있다.

    from utils import paths
    paths.bootstrap()                       # sys.path 먼저
    from utils import device, scene, metrics, viz

모듈
  paths    경로 상수 + sys.path 부트스트랩 + 가중치 해결
  device   GPU/CPU 선택, device 를 물고 있는 검출기 래퍼
  scene    env 생성(step=0.3 고정), 관측 경로 3종, GT 코너
  metrics  center/size 오차, 시드 집계, 색 혼동행렬
  viz      팔레트 + 그림 헬퍼 (라벨은 전부 영문 — claude.md 규칙 3)
"""
from . import paths          # noqa: F401  — 나머지는 bootstrap 이후에 import 할 것

__all__ = ["paths", "device", "scene", "metrics", "viz"]
