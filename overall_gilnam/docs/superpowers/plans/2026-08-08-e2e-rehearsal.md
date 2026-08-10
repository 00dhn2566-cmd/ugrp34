# 파이프라인 E2E 리허설 + normal 부호 확정 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** corner 유도 normal 부호를 spec에 잠정 확정하고, 비전 GT→노이즈→삼각측량 복원→창문 맵→웨이포인트 계획 전 구간을 잇는 E2E 리허설로 노이즈 수준별 최종 계획 품질(통과점 여유 잠식)을 측정한다.

**Architecture:** 계획기에 `normal_from_corners`(부호 확정 공식) + 폴백 추가, `eval_recon3d`에 복원 원본 공개 API 추가(기존 동작 불변), `overall_gilnam/integration/`에 이음새(창문 맵 조립)와 스케일 스윕 CLI 신설. 전부 기존 모듈 재사용 — 신규 로직은 이음새·지표 계산뿐.

**Tech Stack:** Python, numpy, pyyaml, cv2(간접), pytest. 새 의존성 금지.

**Spec:** `overall_gilnam/docs/superpowers/specs/2026-08-08-e2e-rehearsal-design.md`

## Global Constraints

- Python 실행체: `C:\Users\user\anaconda3\python.exe` (PATH `python`은 스토어 스텁 — 사용 금지).
- 테스트는 각 폴더에서: `overall_gilnam/planning`·`overall_gilnam/vision`·`overall_gilnam/integration` 각각 `-m pytest tests/ -q`. vision의 `test_toy_and_eval.py::test_image_label_config_round_trip` 실패는 기존·환경 기인(OpenCV 비ASCII 임시 경로) — 무시, 그 외 전부 green 유지.
- 부호 확정 공식 (spec 결정, 변경 금지): `n̂ = normalize(cross(c3 − c0, c1 − c0))` — (BL−TL)×(TR−TL), 접근측 향함.
- 실측 상수: 노이즈 평균 8.87px/p95 36.6px, 드롭 31/802, 시드 1234, 스케일 {0, 0.5, 1.0, 1.5, 2.0}. **scale 0은 노이즈·드롭 없이 원본 스트림 그대로** (게이트 용도).
- 다른 팀원 코드(reinforcement_yunho·control_seoungjin·visual_imaging_taemin) 무변경.
- 주석·문서 한국어, 기존 모듈 docstring 스타일. 커밋 메시지는 태스크 명시 문구 그대로 + 트레일러:

```
Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

---

### Task 1: normal 부호 확정 — 계획기 헬퍼·폴백 + spec 기입

**Files:**
- Modify: `overall_gilnam/planning/window_waypoint_planner.py`
- Modify: `overall_gilnam/docs/state_window_interface_spec_v0_1.md`
- Test: `overall_gilnam/planning/tests/test_planner.py` (추가)

**Interfaces:**
- Produces: `normal_from_corners(corners_3d) -> np.ndarray(3,)` (접근측 단위 법선, (4,3) 아님·퇴화 시 ValueError); `gate_points`는 `normal` 부재 시 `corners_3d`로 폴백, 둘 다 없으면 ValueError; `crossing_warnings`도 동일 폴백 (판정 불가 창문은 건너뜀 — 경고 전용 기능).
- Consumes: Task 1 이전의 기존 계획기 코드.

- [ ] **Step 1: 실패하는 테스트 추가** — `tests/test_planner.py`에 이어서

```python
from window_waypoint_planner import normal_from_corners


def _corners_for(center, n, w, h):
    # synth 관례(README_stream): viewer_right = cross(-n, UP), TL→TR→BR→BL (접근측 기준)
    center = np.asarray(center, dtype=float)
    n = np.asarray(n, dtype=float)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(-n, up)
    right = right / np.linalg.norm(right)
    return [
        (center + h / 2 * up - w / 2 * right).tolist(),
        (center + h / 2 * up + w / 2 * right).tolist(),
        (center - h / 2 * up + w / 2 * right).tolist(),
        (center - h / 2 * up - w / 2 * right).tolist(),
    ]


def test_normal_from_corners_points_to_approach_side():
    # 부호 확정 공식 검증 — yaw 있는 normal 포함
    for n in ([-1.0, 0.0, 0.0], [-0.9, -0.436, 0.0]):
        corners = _corners_for([5.0, 0.0, 1.5], n, 1.0, 1.2)
        derived = normal_from_corners(corners)
        n_unit = np.asarray(n) / np.linalg.norm(n)
        assert float(np.dot(derived, n_unit)) > 0.999


def test_normal_from_corners_rejects_degenerate():
    with pytest.raises(ValueError):
        normal_from_corners([[0, 0, 0]] * 4)          # 퇴화 (모든 점 동일)
    with pytest.raises(ValueError):
        normal_from_corners([[0, 0, 0], [1, 1, 1]])   # (4,3) 아님


def test_gate_points_fallback_to_corners():
    # normal 없이 corners_3d만 있어도 명시 normal과 동일한 게이트 점
    corners = _corners_for(WIN["center"], WIN["normal"], *WIN["size_wh"])
    win_c = {"order_index": 0, "color": "red", "center": WIN["center"],
             "size_wh": WIN["size_wh"], "corners_3d": corners}
    a1, e1 = gate_points(WIN, 1.5, 1.0, 0.35)
    a2, e2 = gate_points(win_c, 1.5, 1.0, 0.35)
    np.testing.assert_allclose(a1, a2, atol=1e-9)
    np.testing.assert_allclose(e1, e2, atol=1e-9)


def test_gate_points_requires_normal_or_corners():
    bare = {"order_index": 0, "color": "red", "center": [5.0, 0.0, 1.5], "size_wh": [1.0, 1.2]}
    with pytest.raises(ValueError, match="corners_3d"):
        gate_points(bare, 1.5, 1.0, 0.35)
```

- [ ] **Step 2: 실패 확인**

Run (`overall_gilnam/planning/`에서): `C:\Users\user\anaconda3\python.exe -m pytest tests/ -v`
Expected: 새 테스트 4개 FAIL — `ImportError: cannot import name 'normal_from_corners'`. 기존 11개는 PASS 유지.

- [ ] **Step 3: 구현** — `window_waypoint_planner.py`

`UP` 정의 아래에 추가:

```python
def normal_from_corners(corners_3d):
    """corner 4점(접근측에서 본 TL→TR→BR→BL) → 접근측을 향하는 단위 법선.

    winding 계약(v0.2 §4.3)의 따름정리 (spec §3.1 잠정 확정 2026-08-08):
    n̂ = normalize(cross(BL−TL, TR−TL)). 인자 순서 주의 — cross(TR−TL, BL−TL)은
    반대 방향 (reinforcement_yunho/rl/README.md의 antiparallel 지적 참조).
    """
    c = np.asarray(corners_3d, dtype=float)
    if c.shape != (4, 3):
        raise ValueError(f"corners_3d는 (4,3)이어야 함 — got {c.shape}")
    n = np.cross(c[3] - c[0], c[1] - c[0])
    n_len = float(np.linalg.norm(n))
    if n_len < 1e-9:
        raise ValueError("퇴화 corner — 법선 유도 불가")
    return n / n_len
```

`gate_points`의 normal 처리 블록(현재: `"normal" not in window` 검사 → 여유 검사 → center/n 계산)을 다음으로 교체 — 여유 검사(`<= 0`)와 ident·거리 계산은 그대로 두고 **법선 결정만** 3분기로:

```python
    if "normal" in window:
        n = np.asarray(window["normal"], dtype=float)
        n_len = float(np.linalg.norm(n))
        if n_len < 1e-9:
            raise ValueError(f"창문 {ident}: normal이 영벡터 — 접근측 판정 불가")
        n = n / n_len
    elif "corners_3d" in window:
        n = normal_from_corners(window["corners_3d"])  # 부호 확정 공식 폴백
    else:
        raise ValueError(f"창문 {ident}: normal·corners_3d 모두 부재 — 접근측 판정 불가")
```

`crossing_warnings`의 창문별 법선 결정도 동일 폴백으로 교체 (경고 전용이므로 판정 불가는 건너뜀 — 기존 width_axis 가드 유지):

```python
        if "normal" in w:
            n = np.asarray(w["normal"], dtype=float)
        elif "corners_3d" in w:
            try:
                n = normal_from_corners(w["corners_3d"])
            except ValueError:
                continue  # 판정 불가 — 경고 전용 기능이므로 건너뜀
        else:
            continue
        n_len = float(np.linalg.norm(n))
        if n_len < 1e-9:
            continue
        n = n / n_len
```

`state_window_interface_spec_v0_1.md` 갱신 2곳:

§3.1의 미결 관례 블록 —

기존:
```
- B-3 채택 시 추가로 확정할 관례 2개:
  - **normal의 ± 방향** (예: 접근측을 향하도록)
  - **corner winding 재정의** — v0.2의 "좌상→우상→우하→좌하"를 "**접근측에서 본 기준**"으로 명시 (3D에서는 보는 쪽에 따라 시계방향이 뒤집히므로)
```

교체:
```
- B-3 채택 시 추가로 확정할 관례 2개 → **잠정 확정 (2026-08-08, 비동기 — 이의 없으면 v1.0 반영)**:
  - **corner winding**: v0.2의 "좌상→우상→우하→좌하"는 "**접근측에서 본 기준**"으로 명시 (3D에서는 보는 쪽에 따라 뒤집히므로). 합성 스트림 계약(`vision/sample_stream/README_stream.md`)과 동일.
  - **normal의 ± 방향**: **접근측을 향한다.** winding이 접근측 기준 시계방향이므로 corner에서 유도 가능 — `n̂ = normalize(cross(c3−c0, c1−c0))` ((BL−TL)×(TR−TL)). 별도 관례가 아니라 winding 계약의 따름정리. `cross(c1−c0, c3−c0)`는 반대 방향이니 주의 (`reinforcement_yunho/rl/README.md`의 antiparallel 지적 — 본 확정으로 해소). 소비측 구현: `overall_gilnam/planning/window_waypoint_planner.py`의 `normal_from_corners`.
```

§7 체크리스트 —

기존: `- [ ] B-3 채택 시: normal ± 방향, corner winding(접근측 기준) 확정`
교체: `- [x] B-3 채택 시: normal ± 방향, corner winding(접근측 기준) — **잠정 확정 (2026-08-08 비동기, §3.1)**. 회의에서 추인만`

- [ ] **Step 4: 통과 확인**

Run: `C:\Users\user\anaconda3\python.exe -m pytest tests/ -q`
Expected: 15 passed (기존 11 + 신규 4)

- [ ] **Step 5: 커밋**

```bash
git add overall_gilnam/planning/window_waypoint_planner.py overall_gilnam/planning/tests/test_planner.py overall_gilnam/docs/state_window_interface_spec_v0_1.md
git commit -m "planning+spec: corner 유도 normal 부호 잠정 확정 — 접근측 = cross(BL−TL, TR−TL), 계획기 폴백"
```

---

### Task 2: eval_recon3d 복원 원본 공개 API

**Files:**
- Modify: `overall_gilnam/vision/eval_recon3d.py`
- Test: `overall_gilnam/vision/tests/test_eval_recon3d.py` (추가)

**Interfaces:**
- Produces: `reconstruct_windows(records, scene_gt, min_baseline_m=0.5, max_pairs=2000) -> dict[int, dict]` — `{order_index: {"color": str, "corners_3d_est": np.ndarray(4,3)|None, "n_pairs": int}}`. `evaluate_records`는 이 함수를 내부 재사용하되 **반환 형식·수치 불변**.
- Consumes: 기존 `_intrinsics_K`·`_observations`·`_triangulate`.

- [ ] **Step 1: 실패하는 테스트 추가** — `tests/test_eval_recon3d.py`에 이어서

```python
from eval_recon3d import reconstruct_windows


def test_reconstruct_windows_matches_gt_corners():
    # 무노이즈 복원 원본이 GT corner와 mm 수준 일치 + evaluate_records와 정합
    records, scene_gt = _load_sample()
    recon = reconstruct_windows(records, scene_gt)
    assert sorted(recon.keys()) == [0, 1, 2]
    for gt in scene_gt["windows"]:
        r = recon[gt["order_index"]]
        assert r["n_pairs"] > 0 and r["color"] == gt["color"]
        err_mm = np.linalg.norm(
            np.asarray(r["corners_3d_est"]) - np.asarray(gt["corners_3d"]), axis=1) * 1000.0
        assert float(err_mm.max()) < 1.0
```

(파일 상단에 `import numpy as np`가 없으면 추가.)

- [ ] **Step 2: 실패 확인**

Run (`overall_gilnam/vision/`에서): `C:\Users\user\anaconda3\python.exe -m pytest tests/test_eval_recon3d.py -v`
Expected: 새 테스트 FAIL — `ImportError: cannot import name 'reconstruct_windows'`

- [ ] **Step 3: 구현** — `eval_recon3d.py`

`evaluate_records` 위에 추가:

```python
def reconstruct_windows(records, scene_gt, min_baseline_m=0.5, max_pairs=2000):
    """창문별 삼각측량 복원 원본 (E2E 리허설 등 소비용).

    반환: {order_index: {"color", "corners_3d_est"(4,3) | None(복원 불가), "n_pairs"}}.
    evaluate_records는 이 결과에서 오차만 계산한다 — 수치 경로는 동일.
    """
    K = _intrinsics_K(scene_gt)
    out = {}
    for gt in scene_gt["windows"]:
        est, n_pairs = _triangulate(
            _observations(records, K, gt["order_index"]), min_baseline_m, max_pairs)
        out[gt["order_index"]] = {"color": gt["color"], "corners_3d_est": est, "n_pairs": n_pairs}
    return out
```

`evaluate_records`의 창문 루프 도입부를 리팩터 — 기존:

```python
    for gt in scene_gt["windows"]:
        est, n_pairs = _triangulate(_observations(records, K, gt["order_index"]),
                                    min_baseline_m, max_pairs)
```

를 함수 첫 줄에서 `recon = reconstruct_windows(records, scene_gt, min_baseline_m, max_pairs)`를 계산한 뒤:

```python
    for gt in scene_gt["windows"]:
        r = recon[gt["order_index"]]
        est, n_pairs = r["corners_3d_est"], r["n_pairs"]
```

로 교체 (K 지역 계산은 reconstruct_windows 안으로 이동 — evaluate_records에 K가 더 이상 필요 없으면 제거). 스텁 처리·오차 계산 블록은 그대로.

- [ ] **Step 4: 전체 테스트 통과 확인**

Run: `C:\Users\user\anaconda3\python.exe -m pytest tests/ -q`
Expected: 41 passed, 1 failed (기존 환경 실패만)

- [ ] **Step 5: 커밋**

```bash
git add overall_gilnam/vision/eval_recon3d.py overall_gilnam/vision/tests/test_eval_recon3d.py
git commit -m "vision: 삼각측량 복원 원본 공개 API reconstruct_windows (evaluate_records 내부 재사용)"
```

---

### Task 3: E2E 이음새 — 창문 맵 조립 + scale 0 게이트

**Files:**
- Create: `overall_gilnam/integration/e2e_rehearsal.py`
- Create: `overall_gilnam/integration/tests/conftest.py`
- Test: `overall_gilnam/integration/tests/test_e2e.py`

**Interfaces:**
- Consumes: `noisy_stream.load_records/make_noisy_records/P_TAIL/DEFAULT_DROP`, `eval_recon3d.reconstruct_windows`(Task 2), `window_waypoint_planner`의 `PLANNING_DIR/UP/gate_points/load_planner_config/normal_from_corners/plan_waypoints/crossing_warnings`(Task 1).
- Produces: `assemble_window_map(recon) -> (window_map: dict, failed: list[int])`; `run_scale(records, scene_gt, cfg, scale, seed=1234) -> dict` (창문별 게이트 오차·통과점 지표·경고·n_pairs — scale 0은 원본 스트림 그대로); 모듈 상수 `SCALES = [0.0, 0.5, 1.0, 1.5, 2.0]`.

- [ ] **Step 1: conftest + 실패하는 테스트 작성**

`integration/tests/conftest.py`:

```python
# tests가 integration/ 모듈을 패키지 설치 없이 import할 수 있게 경로 추가 (vision/tests 패턴)
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
```

`integration/tests/test_e2e.py`:

```python
"""E2E 리허설 테스트 — scale 0 전 구간 게이트(≤1mm)가 핵심."""
import numpy as np
import pytest

from e2e_rehearsal import SAMPLE, assemble_window_map, load_inputs, run_scale


def test_scale0_full_pipeline_gate():
    # 무노이즈: 복원 창문 맵 계획이 GT 창문 계획과 게이트점 1mm 이내 일치
    records, scene_gt, cfg = load_inputs()
    result = run_scale(records, scene_gt, cfg, scale=0.0)
    assert result["failed"] == [] and result["n_warnings"] == 0
    for w in result["windows"]:
        assert w["approach_err_mm"] < 1.0 and w["exit_err_mm"] < 1.0
        assert w["margin_left_mm"] > 0


def test_assembled_map_matches_gt():
    # 이음새 검증: 복원 맵의 center·size·normal(부호 포함)이 GT와 정합
    records, scene_gt, cfg = load_inputs()
    from eval_recon3d import reconstruct_windows
    wmap, failed = assemble_window_map(reconstruct_windows(records, scene_gt))
    assert failed == [] and len(wmap["windows"]) == 3
    for w, gt in zip(wmap["windows"], scene_gt["windows"]):
        assert w["order_index"] == gt["order_index"]
        assert np.linalg.norm(np.asarray(w["center"]) - np.asarray(gt["center"])) * 1000 < 1.0
        assert float(np.dot(w["normal"], np.asarray(gt["normal"]))) > 0.999  # 부호 확정 공식
        np.testing.assert_allclose(w["size_wh"], gt["size_wh"], atol=2e-3)


def test_failed_window_excluded_and_reported():
    recon = {
        0: {"color": "red", "corners_3d_est": None, "n_pairs": 0},
        1: {"color": "green", "n_pairs": 5, "corners_3d_est": np.array(
            [[4.0, 0.6, 2.0], [4.0, -0.6, 2.0], [4.0, -0.6, 1.0], [4.0, 0.6, 1.0]])},
    }
    wmap, failed = assemble_window_map(recon)
    assert failed == [0]
    assert [w["order_index"] for w in wmap["windows"]] == [1]
```

- [ ] **Step 2: 실패 확인**

Run (`overall_gilnam/integration/`에서): `C:\Users\user\anaconda3\python.exe -m pytest tests/ -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'e2e_rehearsal'`

- [ ] **Step 3: 구현** — `e2e_rehearsal.py`

```python
"""파이프라인 E2E 리허설: 비전 GT → 노이즈 → 삼각측량 복원 → 창문 맵 → 웨이포인트 계획.

설계: overall_gilnam/docs/superpowers/specs/2026-08-08-e2e-rehearsal-design.md.
체크리스트 4번 "전체 파이프라인 통합 검증 주도"의 실행 — 태민(융합)·성진(궤적) 대역은
각각 eval_recon3d 재현 삼각측량·waypoints_config 스키마 검증으로 대신한다.

지표: GT 창문 계획 대비 게이트점 오차(mm), 계획 경로의 GT 창문 평면 통과점이
개구부 중심에서 벗어난 거리와 잔여 여유(mm — margin 논의 직결), 안전 경고 수.
scale 0은 노이즈·드롭 없는 원본 스트림 그대로 (전 구간 정합 게이트).
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

INTEGRATION_DIR = Path(__file__).resolve().parent
GILNAM = INTEGRATION_DIR.parent
for _sub in ("vision", "planning"):
    sys.path.insert(0, str(GILNAM / _sub))

from noisy_stream import DEFAULT_DROP, DEFAULT_MEAN_PX, DEFAULT_P95_PX, P_TAIL, load_records, make_noisy_records  # noqa: E402
from eval_recon3d import reconstruct_windows  # noqa: E402
from window_waypoint_planner import (  # noqa: E402
    PLANNING_DIR, UP, crossing_warnings, gate_points, load_planner_config,
    normal_from_corners, plan_waypoints,
)

SAMPLE = GILNAM / "vision" / "sample_stream"
SCALES = [0.0, 0.5, 1.0, 1.5, 2.0]


def load_inputs():
    """샘플 스트림·scene_gt·계획기 설정 로드."""
    records = load_records(SAMPLE / "sample_stream.jsonl")
    scene_gt = json.loads((SAMPLE / "scene_gt.json").read_text(encoding="utf-8"))
    cfg = load_planner_config(PLANNING_DIR / "planner_limits.yaml")
    return records, scene_gt, cfg


def assemble_window_map(recon):
    """reconstruct_windows 결과 → §6.2 창문 맵. 복원 불가 창문은 제외하고 failed로 보고."""
    windows, failed = [], []
    for order_index in sorted(recon):
        r = recon[order_index]
        est = r["corners_3d_est"]
        if est is None:
            failed.append(order_index)
            continue
        est = np.asarray(est, dtype=float)
        tl, tr, br, bl = est
        w = (np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2.0
        h = (np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2.0
        windows.append({
            "order_index": order_index,
            "color": r["color"],
            "corners_3d": est.tolist(),
            "center": est.mean(axis=0).tolist(),
            "normal": normal_from_corners(est).tolist(),  # 부호 확정 공식 (spec §3.1)
            "size_wh": [float(w), float(h)],
        })
    return {"windows": windows}, failed


def _pass_point(a, b, center, n):
    """구간 a→b가 평면(center, n)을 지나는 점 (crossing_warnings와 동일 보간)."""
    da, db = float(np.dot(a - center, n)), float(np.dot(b - center, n))
    return a + (b - a) * (da / (da - db))


def run_scale(records, scene_gt, cfg, scale, seed=1234):
    """스케일 1개 실행 → 창문별 지표 dict. scale 0은 원본 그대로 (게이트)."""
    if scale == 0.0:
        stream = records
    else:
        stream = make_noisy_records(records, scale, seed, DEFAULT_MEAN_PX, DEFAULT_P95_PX,
                                    P_TAIL, DEFAULT_DROP)
    recon = reconstruct_windows(stream, scene_gt)
    wmap, failed = assemble_window_map(recon)

    start = records[0]["pose"]["position"]
    warnings = []
    wc = plan_waypoints({"position": start}, wmap, cfg, warn=warnings.append)
    warnings += crossing_warnings(wc.waypoints, scene_gt["windows"], cfg["clearance_margin"])

    gt_by_order = {w["order_index"]: w for w in scene_gt["windows"]}
    rows = []
    for i, w in enumerate(wmap["windows"]):
        gt = gt_by_order[w["order_index"]]
        approach_gt, exit_gt = gate_points(gt, cfg["d_app"], cfg["d_exit"], cfg["clearance_margin"])
        a = np.asarray(wc.waypoints[1 + 2 * i], dtype=float)
        b = np.asarray(wc.waypoints[2 + 2 * i], dtype=float)
        center = np.asarray(gt["center"], dtype=float)
        n = np.asarray(gt["normal"], dtype=float)
        n = n / np.linalg.norm(n)
        p = _pass_point(a, b, center, n)
        width_axis = np.cross(UP, n)
        width_axis = width_axis / np.linalg.norm(width_axis)
        u = abs(float(np.dot(p - center, width_axis)))
        v = abs(float(np.dot(p - center, UP)))
        margin_left = min(gt["size_wh"][0] / 2.0 - u, gt["size_wh"][1] / 2.0 - v)
        rows.append({
            "order_index": w["order_index"],
            "color": w["color"],
            "n_pairs": recon[w["order_index"]]["n_pairs"],
            "approach_err_mm": float(np.linalg.norm(a - approach_gt)) * 1000.0,
            "exit_err_mm": float(np.linalg.norm(b - exit_gt)) * 1000.0,
            "pass_u_mm": u * 1000.0,
            "pass_v_mm": v * 1000.0,
            "margin_left_mm": margin_left * 1000.0,
        })
    return {"scale": scale, "windows": rows, "failed": failed, "n_warnings": len(warnings),
            "warnings": warnings}
```

- [ ] **Step 4: 통과 확인 (전 구간 게이트)**

Run: `C:\Users\user\anaconda3\python.exe -m pytest tests/ -v`
Expected: 3 passed. `test_scale0_full_pipeline_gate` FAIL 시 스윕 진행 금지 — 이음새(부호·corner 순서)부터 재점검.

- [ ] **Step 5: 커밋**

```bash
git add overall_gilnam/integration/
git commit -m "integration: E2E 이음새(복원→창문 맵→계획) + scale 0 전 구간 1mm 게이트"
```

---

### Task 4: 스케일 스윕 CLI + 결과 보고 문서

**Files:**
- Modify: `overall_gilnam/integration/e2e_rehearsal.py` (CLI 추가)
- Create: `overall_gilnam/docs/e2e_rehearsal_report.md`
- 커밋 포함: `overall_gilnam/docs/superpowers/specs/2026-08-08-e2e-rehearsal-design.md`, `overall_gilnam/docs/superpowers/plans/2026-08-08-e2e-rehearsal.md`

**Interfaces:**
- Consumes: Task 3의 `run_scale`·`SCALES`·`load_inputs`.
- Produces: CLI `python e2e_rehearsal.py [--scales 0,0.5,1,1.5,2] [--seed 1234] [--json out.json]` → markdown 표 stdout. 보고 문서.

- [ ] **Step 1: CLI 구현** — `e2e_rehearsal.py`에 추가

```python
def main():
    ap = argparse.ArgumentParser(description="E2E 리허설: 스케일별 복원→계획 품질 표 (markdown)")
    ap.add_argument("--scales", default=",".join(str(s) for s in SCALES))
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--json", help="기계 판독 결과 저장 경로 (선택)")
    args = ap.parse_args()

    records, scene_gt, cfg = load_inputs()
    results = []
    print("| scale | 창문 | n_pairs | 게이트 오차 접근/이탈 (mm) | 통과점 u/v (mm) | 잔여 여유 (mm) |")
    print("|---|---|---|---|---|---|")
    for scale in [float(s) for s in args.scales.split(",")]:
        res = run_scale(records, scene_gt, cfg, scale, args.seed)
        results.append(res)
        for w in res["windows"]:
            print(f"| x{scale:g} | {w['order_index']} ({w['color']}) | {w['n_pairs']} "
                  f"| {w['approach_err_mm']:.1f} / {w['exit_err_mm']:.1f} "
                  f"| {w['pass_u_mm']:.1f} / {w['pass_v_mm']:.1f} | {w['margin_left_mm']:.1f} |")
        worst = min((w["margin_left_mm"] for w in res["windows"]), default=float("nan"))
        note = f"경고 {res['n_warnings']}건" + (f", 복원 불가 {res['failed']}" if res["failed"] else "")
        print(f"| **x{scale:g} 요약** | 창문 {len(res['windows'])}개 | — | — | — "
              f"| **최소 {worst:.1f}** ({note}) |")
    for res in results:
        for msg in res["warnings"]:
            print(f"  ! x{res['scale']:g}: {msg}", file=sys.stderr)
    if args.json:
        Path(args.json).write_text(json.dumps(results, ensure_ascii=False, indent=2),
                                   encoding="utf-8")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 스윕 실행**

Run (`overall_gilnam/integration/`에서): `PYTHONIOENCODING=utf-8` 환경으로
`C:\Users\user\anaconda3\python.exe e2e_rehearsal.py --json <스크래치>/e2e_results.json`
Expected: scale 0 행이 sub-mm, 스케일 증가에 따라 게이트 오차·중심 이탈 증가, 잔여 여유 감소. 표 전문 보존 (Step 3 입력).

- [ ] **Step 3: 보고 문서 작성** — `overall_gilnam/docs/e2e_rehearsal_report.md`

구성 (수치는 전부 Step 2 실측 — 자리표시자 금지):

```markdown
# 파이프라인 E2E 리허설 보고 — 비전→복원→계획 전 구간

> 2026-08-08 · 류길남 · 체크리스트 4번 "통합 검증 주도" 실행 (회의 부재 중 비동기 공유용)
> 코드: integration/e2e_rehearsal.py (방법·한계는 specs/2026-08-08-e2e-rehearsal-design.md)

## 요약 (결론 먼저)
[scale 0 게이트 통과 여부 + 실측 수준(x1)에서 통과점 잔여 여유 최소값 + margin 논의와의 연결 1~2문장
 + corner 유도 normal 부호 잠정 확정 안내 (spec §3.1, 윤호 rl/README OPEN 해소)]

## 스케일 스윕 표
[Step 2 표 전문]

## 해석
[게이트점 오차 vs 통과점 여유 잠식 관계, eval_target_derivation.md의 margin 역산과 대조 —
 3D 복원 오차(중심 58mm@x1)가 통과점 이탈로 어떻게 전파되는지]

## 한계·가정
[설계 문서의 3항 그대로]

## 팀 통보 사항
[① normal 부호 잠정 확정 (윤호: rl/README antiparallel 건 해소, state_window spec §3.1)
 ② 계획기·리허설이 interface.schemas 소비 중 ③ 이의 있으면 회의에서 재론]
```

- [ ] **Step 4: 전체 테스트 최종 확인**

Run: planning(15)·vision(41+기존 환경 실패 1)·integration(3) 각 폴더에서 `-m pytest tests/ -q`
Expected: 신규 포함 전부 green (환경 실패 1건 제외)

- [ ] **Step 5: 커밋**

```bash
git add overall_gilnam/integration/e2e_rehearsal.py overall_gilnam/docs/e2e_rehearsal_report.md \
        overall_gilnam/docs/superpowers/specs/2026-08-08-e2e-rehearsal-design.md \
        overall_gilnam/docs/superpowers/plans/2026-08-08-e2e-rehearsal.md
git commit -m "integration: E2E 스케일 스윕 CLI + 리허설 보고 (통과점 여유 잠식 실측)"
```

---

## Self-Review 기록

- 스펙 커버리지: 부호 확정·spec 기입(T1), reconstruct_windows(T2), 이음새·게이트(T3), 스윕·보고·통보(T4). 성공 기준 1·2=T1 테스트, 3=T3 게이트, 4=T4, 5=각 태스크 Step 4 — 전 항목 대응.
- 자리표시자: 보고 문서 골격(T4)은 실측값 기입 지시로 대체 (실행 전 수치 부재) — "자리표시자 금지" 명시로 방어.
- 타입 일관성: `normal_from_corners`(T1 정의 = T3 사용), `reconstruct_windows` 반환 키 `corners_3d_est`(T2 정의 = T3 사용), `run_scale` 반환 키(T3 정의 = T4 사용) 대조 완료. 발견된 오타 방어 분기 1건은 계획 본문에서 직접 수정함.
