"""팀 코드를 우리 입맛대로 고친 버전들. **원본은 안 건드린다.**

원칙
  * 팀 파일(overall_gilnam/, visual_imaging_taemin/, control_seoungjin/)은 읽기 전용.
  * 여기 있는 건 그 옆에 나란히 놓고 숫자를 재기 위한 우리 버전이다.
  * 반환 형식은 원본과 동일하게 유지한다 — 하류가 구분 없이 먹어야 갈아끼울 수 있다.
  * 각 파일 docstring 에 **뭘 바꿨고 왜 바꿨는지** 와 **일부러 안 바꾼 것**을 적는다.

모듈
  detections   §5 검출 스트림 후처리 (중복 표 제거, conf/크기 필터).
               팀 코드에 넘기기 *전에* 거르는 층이라 태민·길남 양쪽에 동시에 먹는다.
  recon_rays   태민 window_recon_node.py 수치 경로 — conf 가중 + Huber IRLS 추가.

숫자 비교는 ``python scripts/compare_recon.py``.
"""
from . import detections, recon_rays  # noqa: F401

__all__ = ["detections", "recon_rays"]
