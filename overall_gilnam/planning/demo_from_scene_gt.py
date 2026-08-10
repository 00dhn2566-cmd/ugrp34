"""scene_gt.json(비전 합성 씬 GT) → §6.2 창문 맵 → 웨이포인트 계획 데모.

비전 산출물(창문 3D GT)과 경로계획을 엔드투엔드로 잇는 최소 예시.
실행: planning/에서  python demo_from_scene_gt.py  (출력: waypoints_config JSON을 stdout)
"""
import json
from pathlib import Path

from window_waypoint_planner import PLANNING_DIR, load_planner_config, plan_waypoints

VISION_STREAM = PLANNING_DIR.parents[0] / "vision" / "sample_stream"


def main():
    scene_gt = json.loads((VISION_STREAM / "scene_gt.json").read_text(encoding="utf-8"))
    first = json.loads((VISION_STREAM / "sample_stream.jsonl").read_text(encoding="utf-8").splitlines()[0])
    drone_state = {"position": first["pose"]["position"]}      # §6.1 중 position만 사용
    window_map = {"windows": scene_gt["windows"]}              # §6.2 부분집합 (passed 부재 → 미통과)
    wc = plan_waypoints(drone_state, window_map, load_planner_config(PLANNING_DIR / "planner_limits.yaml"))
    print(json.dumps(wc.to_dict(), ensure_ascii=False, indent=2))
    print(f"# waypoints={len(wc.waypoints)} (시작 1 + 창문 {len(window_map['windows'])} x 2)")


if __name__ == "__main__":
    main()
