# 창문 통과 웨이포인트 계획기 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** (드론 상태, 창문 3D 맵) → 창문 법선 정렬 접근·이탈 웨이포인트 열 → 성진 컨트롤러 입력(`waypoints_config`)을 만드는 고전 경로계획기.

**Architecture:** `overall_gilnam/planning/` 신설. 순수 기하 코어(numpy) → `interface.schemas.WaypointsConfig` 조립·검증(윤호 모듈 import만, 수정 없음) → CLI/데모. 궤적 스무딩은 하류(성진 `plan_waypoints`) 몫이므로 여기서는 웨이포인트 선정만 한다.

**Tech Stack:** Python, numpy, pyyaml, pytest. 새 의존성 금지.

**Spec:** `overall_gilnam/docs/superpowers/specs/2026-08-08-window-waypoint-planner-design.md`

## Global Constraints

- Python 실행체: `C:\Users\user\anaconda3\python.exe` (PATH의 `python`은 Windows 스토어 스텁 — 사용 금지).
- 테스트 실행 위치: `C:\Users\user\Desktop\ugrp34\overall_gilnam\planning` — `C:\Users\user\anaconda3\python.exe -m pytest tests/ -q` (conftest.py가 경로 주입, vision/tests 패턴).
- 기존 파일 무변경. 윤호 `reinforcement_yunho/interface/schemas.py`는 import만 (PEP 420 네임스페이스: repo 루트의 `reinforcement_yunho`를 sys.path에 넣고 `from interface.schemas import ...`).
- 출력 조립은 반드시 `WaypointsConfig` 경유 + `.validate()` 통과 확인 (직접 dict 조립 금지).
- 기본 파라미터 (spec): d_app=1.5, d_exit=1.0, clearance_margin=0.35, limits {v_max: 2.0, a_max: 2.0, j_max: 10.0, snap_max: 50.0}, dt=0.01.
- 입력 스키마는 `state_window_interface_spec_v0_1` §6.1/§6.2 **미확정 후보안** 기준 — 모듈 docstring에 명시.
- 주석·docstring 한국어, 모듈 docstring에 목적·정책 요약 (vision/ 파일 스타일).
- 커밋 메시지는 각 태스크에 명시된 문구 그대로, 말미에 다음 트레일러:

```
Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

- 알려진 환경: `tests/test_toy_and_eval.py`(vision) 실패는 기존·환경 기인 — planning 테스트와 무관.

---

### Task 1: 기하 코어 — 게이트 점 생성·여유 검사·순서 필터

**Files:**
- Create: `overall_gilnam/planning/window_waypoint_planner.py`
- Create: `overall_gilnam/planning/tests/conftest.py`
- Test: `overall_gilnam/planning/tests/test_planner.py`

**Interfaces:**
- Produces: `UP` (np.array [0,0,1]), `gate_points(window: dict, d_app: float, d_exit: float, clearance_margin: float) -> tuple[np.ndarray, np.ndarray]` (접근점, 이탈점 — 각 (3,)); `ordered_open_windows(window_map: dict) -> list[dict]` (passed 제외, order_index 오름차순). `gate_points`는 여유 부족·normal 부재 시 `ValueError`(창문 식별 포함).
- Consumes: 없음 (numpy만).

- [ ] **Step 1: conftest + 실패하는 테스트 작성**

`overall_gilnam/planning/tests/conftest.py`:

```python
# tests가 planning/ 모듈을 패키지 설치 없이 import할 수 있게 경로 추가 (vision/tests 패턴)
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
```

`overall_gilnam/planning/tests/test_planner.py`:

```python
"""window_waypoint_planner 테스트 — 기하(접근측)·순서·여유 검사가 핵심."""
import numpy as np
import pytest

from window_waypoint_planner import gate_points, ordered_open_windows

# 접근측 normal = -X (synth_scene 관례와 동일한 예시 창문)
WIN = {
    "order_index": 0, "color": "red",
    "center": [5.0, 0.0, 1.5], "normal": [-1.0, 0.0, 0.0],
    "size_wh": [1.0, 1.2],
}


def test_gate_points_on_correct_sides():
    approach, exit_ = gate_points(WIN, d_app=1.5, d_exit=1.0, clearance_margin=0.35)
    center = np.array(WIN["center"])
    n = np.array(WIN["normal"])
    assert np.dot(approach - center, n) > 0        # 접근점은 접근측
    assert np.dot(exit_ - center, n) < 0           # 이탈점은 반대측
    np.testing.assert_allclose(np.linalg.norm(approach - center), 1.5)
    np.testing.assert_allclose(np.linalg.norm(exit_ - center), 1.0)


def test_gate_points_rejects_narrow_window():
    narrow = dict(WIN, size_wh=[0.6, 1.2])         # min/2 = 0.3 < margin 0.35
    with pytest.raises(ValueError, match="order_index=0"):
        gate_points(narrow, 1.5, 1.0, clearance_margin=0.35)


def test_gate_points_rejects_missing_normal():
    no_normal = {k: v for k, v in WIN.items() if k != "normal"}
    with pytest.raises(ValueError, match="normal"):
        gate_points(no_normal, 1.5, 1.0, 0.35)


def test_ordered_open_windows_filters_and_sorts():
    wmap = {"windows": [
        dict(WIN, order_index=2),
        dict(WIN, order_index=0, passed=True),     # 통과 완료 → 제외
        dict(WIN, order_index=1),                  # passed 부재 → false 취급
    ]}
    out = ordered_open_windows(wmap)
    assert [w["order_index"] for w in out] == [1, 2]
```

- [ ] **Step 2: 실패 확인**

Run (`overall_gilnam/planning/`에서): `C:\Users\user\anaconda3\python.exe -m pytest tests/ -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'window_waypoint_planner'`

- [ ] **Step 3: 최소 구현** — `window_waypoint_planner.py`

```python
"""창문 통과 웨이포인트 계획기 (고전, 비학습) — 설계: docs/superpowers/specs/2026-08-08-window-waypoint-planner-design.md.

(드론 상태, 창문 3D 맵) → 창문 법선 정렬 접근·이탈점 열 → 성진 waypoints_config.
궤적 스무딩은 하류(성진 plan_waypoints) 몫 — 여기서는 웨이포인트 선정만.

입력 스키마는 state_window_interface_spec_v0_1 §6.1/§6.2 **미확정 후보안** 기준:
- 드론 상태에서 position만 사용.
- 창문 맵에서 order_index/center/normal/size_wh/passed 사용.
- normal은 접근측을 향하는 단위벡터(§3.1 관례). 부재 시 에러 — corner 유도
  법선은 ± 방향 관례 미확정이라 접근측 판정 불가 (rl/README.md 동일 지적).
- passed 부재 시 false 취급 (소유권 미결, spec §7).
"""

import numpy as np

UP = np.array([0.0, 0.0, 1.0])


def gate_points(window, d_app, d_exit, clearance_margin):
    """창문 1개 → (접근점, 이탈점). 접근점 = center + d_app·n̂, 이탈점 = center − d_exit·n̂.

    두 점을 잇는 직선이 center를 지나므로 center 웨이포인트는 별도로 두지 않는다
    (웨이포인트 최소화 → 하류 최소시간 계획이 더 부드러움).
    """
    ident = f"order_index={window.get('order_index')}({window.get('color', '?')})"
    if "normal" not in window:
        raise ValueError(f"창문 {ident}: normal 부재 — 접근측 판정 불가 (spec §3.1 관례 미확정)")
    w, h = window["size_wh"]
    if min(w, h) / 2.0 - clearance_margin < 0:
        raise ValueError(
            f"창문 {ident}: 통과 여유 부족 — min(w,h)/2={min(w, h) / 2.0:.3f}m < margin={clearance_margin}m"
        )
    center = np.asarray(window["center"], dtype=float)
    n = np.asarray(window["normal"], dtype=float)
    n = n / np.linalg.norm(n)
    return center + d_app * n, center - d_exit * n


def ordered_open_windows(window_map):
    """passed=false 창문만 order_index 오름차순으로."""
    return sorted(
        (w for w in window_map["windows"] if not w.get("passed", False)),
        key=lambda w: w["order_index"],
    )
```

- [ ] **Step 4: 통과 확인**

Run: `C:\Users\user\anaconda3\python.exe -m pytest tests/ -v`
Expected: 4 passed

- [ ] **Step 5: 커밋**

```bash
git add overall_gilnam/planning/window_waypoint_planner.py overall_gilnam/planning/tests/
git commit -m "planning: 웨이포인트 계획기 기하 코어 — 게이트 점·여유 검사·순서 필터"
```

---

### Task 2: WaypointsConfig 조립 + limits 설정 + CLI

**Files:**
- Modify: `overall_gilnam/planning/window_waypoint_planner.py` (Task 1에서 생성)
- Create: `overall_gilnam/planning/planner_limits.yaml`
- Test: `overall_gilnam/planning/tests/test_planner.py` (추가)

**Interfaces:**
- Consumes: Task 1의 `gate_points`·`ordered_open_windows`; 윤호 `interface.schemas`의 `WaypointsConfig(waypoints, limits, dt)`·`.validate()`·`save_json(obj, path)` (repo 루트 기준 `reinforcement_yunho`를 sys.path에 추가 후 import).
- Produces: `load_planner_config(path) -> dict` (키: d_app/d_exit/clearance_margin/limits/dt), `plan_waypoints(drone_state: dict, window_map: dict, cfg: dict) -> WaypointsConfig`. CLI: `python window_waypoint_planner.py --state s.json --window-map m.json --out wp.json [--config planner_limits.yaml]`.

- [ ] **Step 1: 실패하는 테스트 추가** — `tests/test_planner.py`에 이어서

```python
import json
from pathlib import Path

from window_waypoint_planner import PLANNING_DIR, load_planner_config, plan_waypoints

STATE = {"position": [0.0, 0.0, 1.5]}
WMAP3 = {"windows": [
    {"order_index": i, "color": c,
     "center": [4.0 + 5.0 * i, 0.3 * i, 1.5], "normal": [-1.0, 0.0, 0.0],
     "size_wh": [1.0, 1.0]}
    for i, c in enumerate(["red", "green", "blue"])
]}


def _cfg():
    return load_planner_config(PLANNING_DIR / "planner_limits.yaml")


def test_plan_waypoints_sequence_and_schema():
    wc = plan_waypoints(STATE, WMAP3, _cfg())
    wc.validate()                                   # 성진 스키마 통과 (윤호 validate 경유)
    assert len(wc.waypoints) == 1 + 2 * 3           # 시작 + 창문 3개 × (접근·이탈)
    assert wc.waypoints[0] == [0.0, 0.0, 1.5]       # 첫 점 = 드론 현재 위치
    # 접근(-X쪽) < center < 이탈: 창문 0의 x 좌표로 확인
    assert wc.waypoints[1][0] == pytest.approx(4.0 - 1.5)
    assert wc.waypoints[2][0] == pytest.approx(4.0 + 1.0)


def test_plan_waypoints_requires_at_least_one_window():
    with pytest.raises(ValueError, match="열린 창문"):
        plan_waypoints(STATE, {"windows": []}, _cfg())


def test_config_defaults_loaded():
    cfg = _cfg()
    assert cfg["d_app"] == 1.5 and cfg["d_exit"] == 1.0
    assert cfg["clearance_margin"] == 0.35
    assert cfg["limits"]["v_max"] == 2.0 and cfg["dt"] == 0.01


def test_cli_roundtrip(tmp_path):
    import subprocess, sys
    state_p, wmap_p, out_p = tmp_path / "s.json", tmp_path / "m.json", tmp_path / "wp.json"
    state_p.write_text(json.dumps(STATE), encoding="utf-8")
    wmap_p.write_text(json.dumps(WMAP3), encoding="utf-8")
    r = subprocess.run(
        [sys.executable, str(PLANNING_DIR / "window_waypoint_planner.py"),
         "--state", str(state_p), "--window-map", str(wmap_p), "--out", str(out_p)],
        capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    saved = json.loads(out_p.read_text(encoding="utf-8"))
    assert len(saved["waypoints"]) == 7 and "limits" in saved
```

- [ ] **Step 2: 실패 확인**

Run: `C:\Users\user\anaconda3\python.exe -m pytest tests/ -v`
Expected: 새 테스트 4개 FAIL — `ImportError: cannot import name 'PLANNING_DIR'`

- [ ] **Step 3: 구현** — `window_waypoint_planner.py`에 추가 + `planner_limits.yaml` 생성

`planner_limits.yaml`:

```yaml
# 계획 파라미터 + 성진 waypoints_config limits 기본값
# limits는 협의 전 임시값 (INPUT_FORMAT.md / waypoints_config.schema.json 참조) — 협의 후 이 파일만 갱신
d_app: 1.5              # 접근점 거리 [m] — 합성 씬 전방 간격 4~6m 대비
d_exit: 1.0             # 이탈점 거리 [m]
clearance_margin: 0.35  # 기체 여유 [m] ≈ 휠베이스 450mm/2 + 프로펠러 — margin 회의 확정 시 갱신
limits:
  v_max: 2.0            # [m/s]
  a_max: 2.0            # [m/s^2]
  j_max: 10.0           # [m/s^3]
  snap_max: 50.0        # [m/s^4]
dt: 0.01                # [s]
```

`window_waypoint_planner.py`에 추가:

```python
import argparse
import json
import sys
from pathlib import Path

import yaml

PLANNING_DIR = Path(__file__).resolve().parent
_REPO_ROOT = PLANNING_DIR.parents[1]
# 윤호 interface 모듈 import 경로 (수정 없음 — WaypointsConfig 조립·검증 경유용)
sys.path.insert(0, str(_REPO_ROOT / "reinforcement_yunho"))

from interface.schemas import WaypointsConfig, save_json  # noqa: E402


def load_planner_config(path):
    """planner_limits.yaml → dict (d_app/d_exit/clearance_margin/limits/dt)."""
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def plan_waypoints(drone_state, window_map, cfg):
    """(드론 상태, 창문 맵) → WaypointsConfig. 웨이포인트 = [현 위치] + [접근ᵢ, 이탈ᵢ]…"""
    windows = ordered_open_windows(window_map)
    if not windows:
        raise ValueError("열린 창문이 없음 — 계획할 대상 없음")
    points = [[float(c) for c in drone_state["position"]]]
    for w in windows:
        approach, exit_ = gate_points(w, cfg["d_app"], cfg["d_exit"], cfg["clearance_margin"])
        points.append([float(c) for c in approach])
        points.append([float(c) for c in exit_])
    wc = WaypointsConfig(waypoints=points, limits=dict(cfg["limits"]), dt=float(cfg["dt"]))
    wc.validate()  # 성진 스키마 검증 — 실패 시 여기서 즉시 드러남
    return wc


def main():
    ap = argparse.ArgumentParser(description="(드론 상태, 창문 맵) JSON → 성진 waypoints_config JSON")
    ap.add_argument("--state", required=True, help="§6.1 드론 상태 JSON (position 사용)")
    ap.add_argument("--window-map", required=True, help="§6.2 창문 맵 JSON")
    ap.add_argument("--out", required=True, help="출력 waypoints_config JSON 경로")
    ap.add_argument("--config", default=str(PLANNING_DIR / "planner_limits.yaml"))
    args = ap.parse_args()

    with open(args.state, encoding="utf-8") as f:
        drone_state = json.load(f)
    with open(args.window_map, encoding="utf-8") as f:
        window_map = json.load(f)
    wc = plan_waypoints(drone_state, window_map, load_planner_config(args.config))
    save_json(wc, args.out)
    print(f"waypoints={len(wc.waypoints)} -> {args.out}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 전체 테스트 통과 확인**

Run: `C:\Users\user\anaconda3\python.exe -m pytest tests/ -q`
Expected: 8 passed

- [ ] **Step 5: 커밋**

```bash
git add overall_gilnam/planning/window_waypoint_planner.py overall_gilnam/planning/planner_limits.yaml overall_gilnam/planning/tests/test_planner.py
git commit -m "planning: WaypointsConfig 조립·limits 설정·CLI — 성진 스키마 검증 경유"
```

---

### Task 3: 벽 평면 교차 경고 + scene_gt 데모 + 문서 커밋

**Files:**
- Modify: `overall_gilnam/planning/window_waypoint_planner.py`
- Create: `overall_gilnam/planning/demo_from_scene_gt.py`
- Test: `overall_gilnam/planning/tests/test_planner.py` (추가)
- 커밋 포함: `overall_gilnam/docs/superpowers/specs/2026-08-08-window-waypoint-planner-design.md`, `overall_gilnam/docs/superpowers/plans/2026-08-08-window-waypoint-planner.md`

**Interfaces:**
- Consumes: Task 2의 `plan_waypoints`(내부에서 경고 수집 추가), `vision/sample_stream/scene_gt.json`(창문 3개: order_index/color/center/normal/size_wh), `vision/sample_stream/sample_stream.jsonl`(첫 pose position).
- Produces: `crossing_warnings(waypoints: list, windows: list, clearance_margin: float) -> list[str]` — 웨이포인트 연속 구간이 어느 창문 벽 평면을 개구부(여유 적용) 밖에서 교차하면 경고 문자열. `plan_waypoints`는 `(WaypointsConfig)` 반환 유지하되 경고는 `plan_waypoints(..., warn=print)` 콜백으로 전달 (기본 print — CLI에서 stderr로).

- [ ] **Step 1: 실패하는 테스트 추가**

```python
from window_waypoint_planner import crossing_warnings


def test_crossing_warning_outside_opening():
    # 구간이 창문 평면을 개구부 밖(y=2.0, 반폭 0.5-마진 바깥)에서 교차 → 경고 1건
    win = {"order_index": 0, "color": "red", "center": [5.0, 0.0, 1.5],
           "normal": [-1.0, 0.0, 0.0], "size_wh": [1.0, 1.0]}
    seg = [[0.0, 2.0, 1.5], [10.0, 2.0, 1.5]]      # x=5 평면을 y=2.0에서 관통
    warns = crossing_warnings(seg, [win], clearance_margin=0.35)
    assert len(warns) == 1 and "order_index=0" in warns[0]


def test_no_warning_through_opening():
    win = {"order_index": 0, "color": "red", "center": [5.0, 0.0, 1.5],
           "normal": [-1.0, 0.0, 0.0], "size_wh": [1.0, 1.0]}
    seg = [[0.0, 0.0, 1.5], [10.0, 0.0, 1.5]]      # 정중앙 통과
    assert crossing_warnings(seg, [win], 0.35) == []
```

- [ ] **Step 2: 실패 확인**

Run: `C:\Users\user\anaconda3\python.exe -m pytest tests/ -v`
Expected: 새 테스트 2개 FAIL — `ImportError: cannot import name 'crossing_warnings'`

- [ ] **Step 3: 구현**

`window_waypoint_planner.py`에 추가 (gate_points 아래):

```python
def crossing_warnings(waypoints, windows, clearance_margin):
    """연속 웨이포인트 구간이 창문 벽 평면을 개구부 밖에서 교차하면 경고 문자열 리스트.

    벽의 실제 범위는 스펙에 없어 거부 판단이 불가 — v1은 경고만 (설계 §알고리즘 5).
    개구부 내부 판정: 평면 교차점을 창문 폭축(cross(UP, n̂))·높이축(UP)에 투영,
    |u| ≤ w/2−margin ∧ |v| ≤ h/2−margin.
    """
    warns = []
    for w in windows:
        center = np.asarray(w["center"], dtype=float)
        n = np.asarray(w["normal"], dtype=float)
        n = n / np.linalg.norm(n)
        width_axis = np.cross(UP, n)
        width_axis = width_axis / np.linalg.norm(width_axis)
        half_w = w["size_wh"][0] / 2.0 - clearance_margin
        half_h = w["size_wh"][1] / 2.0 - clearance_margin
        for a, b in zip(waypoints, waypoints[1:]):
            a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
            da, db = np.dot(a - center, n), np.dot(b - center, n)
            if da * db >= 0:  # 평면을 안 가로지름 (한 점이 평면 위인 경우 포함)
                continue
            p = a + (b - a) * (da / (da - db))  # 평면 교차점
            u, v = np.dot(p - center, width_axis), np.dot(p - center, UP)
            if abs(u) > half_w or abs(v) > half_h:
                warns.append(
                    f"경고: 구간 {np.round(a, 2).tolist()}→{np.round(b, 2).tolist()}가 "
                    f"창문 order_index={w['order_index']}({w.get('color', '?')}) 평면을 "
                    f"개구부 밖(u={u:.2f}, v={v:.2f})에서 교차"
                )
    return warns
```

`plan_waypoints` 끝부분(`wc.validate()` 앞)에 경고 호출 추가 — 시그니처를 `plan_waypoints(drone_state, window_map, cfg, warn=print)`로 바꾸고:

```python
    for msg in crossing_warnings(points, windows, cfg["clearance_margin"]):
        warn(msg)
```

CLI `main()`에서는 `plan_waypoints(..., warn=lambda m: print(m, file=sys.stderr))`로 호출.

`demo_from_scene_gt.py`:

```python
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
```

- [ ] **Step 4: 전체 테스트 + 데모 확인**

Run: `C:\Users\user\anaconda3\python.exe -m pytest tests/ -q` → Expected: 10 passed
Run: `C:\Users\user\anaconda3\python.exe demo_from_scene_gt.py` → Expected: waypoints 7개 JSON + 경고 0건 (합성 씬은 순차 배치)

- [ ] **Step 5: 커밋 (설계·계획 문서 포함)**

```bash
git add overall_gilnam/planning/ overall_gilnam/docs/superpowers/specs/2026-08-08-window-waypoint-planner-design.md overall_gilnam/docs/superpowers/plans/2026-08-08-window-waypoint-planner.md
git commit -m "planning: 벽 평면 교차 경고 + scene_gt 데모 — 비전 GT와 엔드투엔드 연결"
```

---

## Self-Review 기록

- 스펙 커버리지: 구성 4파일(T1~3), 알고리즘 1~4(T1·2)·5 경고(T3), 파라미터 yaml(T2), 입출력 계약(T2 CLI·validate), 테스트 기준 1~6(T1: 1·2·3 / T2: 4·6 / T3: 5·데모) — 전 항목 대응. 테스트 기준 6(결정성)은 난수 미사용 구조로 충족 — 별도 테스트 불필요 판단.
- 자리표시자: 없음.
- 타입 일관성: `gate_points`/`ordered_open_windows`(T1 정의 = T2 사용), `plan_waypoints` 시그니처는 T3에서 `warn=print` 인자 추가로 변경됨을 T3에 명시 — T2 테스트는 위치 인자만 써서 호환.
