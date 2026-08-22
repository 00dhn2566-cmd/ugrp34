# 실모델 노이즈 주입 → 3D 복원 오차 역산 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** GT 스트림에 실측 프로파일(평균 8.87px / p95 36.6px / 미검출 3.9%) 노이즈를 배율별로 주입해 삼각측량 3D 복원 오차 곡선을 뽑고, 통과 여유(margin)별 허용 픽셀 오차를 역산해 eval 목표치를 제안한다.

**Architecture:** 새 CLI 2개(`noisy_stream.py` 주입기, `eval_recon3d.py` 복원 평가기)가 jsonl 파일로만 통신한다. 노이즈는 2성분 가우시안 혼합(코어+꼬리)을 실측 평균·p95에 수치 캘리브레이션. 복원은 README_stream.md 계약 레시피(cv2.triangulatePoints, 다중 프레임쌍 중앙값 집계) 재현이며, 무노이즈 정합 게이트(≤1mm)를 통과해야 스윕 결과를 신뢰한다.

**Tech Stack:** Python, numpy, opencv-python(cv2), pytest. 모두 `overall_gilnam/vision/requirements.txt`에 이미 있는 의존성 — 새 의존성 추가 금지.

**Spec:** `overall_gilnam/docs/superpowers/specs/2026-08-08-recon3d-noise-target-design.md`

## Global Constraints

- 작업 디렉터리: `overall_gilnam/vision/` (테스트는 이 폴더에서 `python -m pytest tests/ -q` — conftest.py가 경로 주입).
- 기존 파일 무변경 (체크리스트·결과 문서 제외). §5 조립은 반드시 기존 `vision_msg.build_window`/`build_frame_message` 경유.
- 실측 상수(기본값): 평균 8.87px, p95 36.6px, 미검출 31/802, 꼬리 비율 p=0.1, 배율 {0.25, 0.5, 1, 1.5, 2, 3}, 시드 1234.
- 좌표 관례: pose = T_world_cam (position t_wc, quat xyzw), X_cam = R_wcᵀ(X_world − t_wc), P = K·[R_wcᵀ | −R_wcᵀ·t_wc]. corner 순서 TL→TR→BR→BL.
- 스윕 중간 산출물(jsonl 6개)은 git에 넣지 않는다. 커밋 대상: 코드+테스트, 태민 패키지(`vision/noisy_stream_x1/`), 결과 문서, 체크리스트.
- 주석·문서는 한국어, 기존 파일들의 스타일(모듈 docstring에 목적·정책 요약)을 따른다.

---

### Task 1: 노이즈 혼합 모델 캘리브레이션 (`noisy_stream.py` 코어)

**Files:**
- Create: `overall_gilnam/vision/noisy_stream.py`
- Test: `overall_gilnam/vision/tests/test_noisy_stream.py`

**Interfaces:**
- Produces: `calibrate_mixture(mean_px: float, p95_px: float, p_tail: float = P_TAIL) -> tuple[float, float]` — (sigma_core, sigma_tail). 상수 `P_TAIL = 0.1`, `DEFAULT_MEAN_PX = 8.87`, `DEFAULT_P95_PX = 36.6`, `DEFAULT_DROP = 31.0 / 802.0`.
- Consumes: 없음 (표준 라이브러리 + numpy).

- [ ] **Step 1: 실패하는 테스트 작성** — `tests/test_noisy_stream.py`

```python
"""noisy_stream 테스트 — 캘리브레이션 정합·주입 스키마·재현성."""
import numpy as np

from noisy_stream import P_TAIL, calibrate_mixture

MEAN_PX, P95_PX = 8.87, 36.6


def _sample_radial(sigma_core, sigma_tail, p_tail, n, seed=0):
    """혼합 모델에서 2D 반경 오차 표본 추출 (테스트 전용 몬테카를로)."""
    rng = np.random.default_rng(seed)
    sigmas = np.where(rng.random(n) < p_tail, sigma_tail, sigma_core)
    xy = rng.normal(0.0, 1.0, (n, 2)) * sigmas[:, None]
    return np.linalg.norm(xy, axis=1)


def test_calibration_matches_measured_stats():
    sc, st = calibrate_mixture(MEAN_PX, P95_PX)
    assert 0.0 < sc < st  # 코어보다 꼬리가 넓어야 함
    r = _sample_radial(sc, st, P_TAIL, 200_000)
    assert abs(r.mean() - MEAN_PX) / MEAN_PX < 0.03
    assert abs(np.percentile(r, 95) - P95_PX) / P95_PX < 0.05
```

- [ ] **Step 2: 실패 확인**

Run (`overall_gilnam/vision/`에서): `python -m pytest tests/test_noisy_stream.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'noisy_stream'`

- [ ] **Step 3: 최소 구현** — `noisy_stream.py`

```python
"""GT 스트림에 실측 프로파일 노이즈 주입 (설계: overall_gilnam/docs/superpowers/specs/2026-08-08-*.md).

실측(2026-08-02 본 판정): corner 반경 오차 평균 8.87px / p95 36.6px / 미검출 31/802.
p95/평균 ≈ 4.1로 가우시안(≈1.95)보다 꼬리가 무거워, 2성분 가우시안 혼합
(per-axis, 확률 1-p 코어 σc / p 꼬리 σt, p=0.1 고정)을 쓴다.
(σc, σt)는 혼합 Rayleigh의 평균·p95가 실측치와 일치하도록 이분법으로 캘리브레이션.
"""

import math

P_TAIL = 0.1
DEFAULT_MEAN_PX = 8.87
DEFAULT_P95_PX = 36.6
DEFAULT_DROP = 31.0 / 802.0
RADIAL_MEAN_COEF = math.sqrt(math.pi / 2.0)  # E[r] = σ·√(π/2) (2D 가우시안 반경 = Rayleigh)


def _mixture_p95(sigma_core, sigma_tail, p_tail):
    """혼합 Rayleigh CDF의 95% 분위 (이분법)."""

    def cdf(r):
        core = 1.0 - math.exp(-r * r / (2.0 * sigma_core * sigma_core)) if sigma_core > 0 else 1.0
        tail = 1.0 - math.exp(-r * r / (2.0 * sigma_tail * sigma_tail))
        return (1.0 - p_tail) * core + p_tail * tail

    lo, hi = 0.0, 20.0 * max(sigma_core, sigma_tail)
    for _ in range(200):
        mid = (lo + hi) / 2.0
        if cdf(mid) < 0.95:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def calibrate_mixture(mean_px, p95_px, p_tail=P_TAIL):
    """(σc, σt) 캘리브레이션. σt를 이분 탐색, σc는 평균 제약에서 종속 결정.

    탐색 구간: σt = σ_all(단일 가우시안, p95 하한)부터 σc=0이 되는 상한까지.
    이 구간에서 p95는 σt에 단조 증가 — 실측 p95가 구간 밖이면 ValueError.
    """
    sigma_all = mean_px / RADIAL_MEAN_COEF
    lo, hi = sigma_all, mean_px / (RADIAL_MEAN_COEF * p_tail)
    if not _mixture_p95(sigma_all, sigma_all, p_tail) < p95_px < _mixture_p95(0.0, hi, p_tail):
        raise ValueError(f"p95={p95_px}는 혼합 모델 표현 범위 밖 (mean={mean_px}, p={p_tail})")
    for _ in range(100):
        sigma_tail = (lo + hi) / 2.0
        sigma_core = (mean_px / RADIAL_MEAN_COEF - p_tail * sigma_tail) / (1.0 - p_tail)
        if _mixture_p95(sigma_core, sigma_tail, p_tail) < p95_px:
            lo = sigma_tail
        else:
            hi = sigma_tail
    sigma_tail = (lo + hi) / 2.0
    sigma_core = (mean_px / RADIAL_MEAN_COEF - p_tail * sigma_tail) / (1.0 - p_tail)
    return sigma_core, sigma_tail
```

- [ ] **Step 4: 통과 확인**

Run: `python -m pytest tests/test_noisy_stream.py -v`
Expected: PASS (1 passed)

- [ ] **Step 5: 커밋**

```bash
git add overall_gilnam/vision/noisy_stream.py overall_gilnam/vision/tests/test_noisy_stream.py
git commit -m "vision: 노이즈 혼합 모델 캘리브레이션 (실측 평균·p95 정합, 이분법)"
```

---

### Task 2: 스트림 노이즈 주입 + CLI (`noisy_stream.py` 완성)

**Files:**
- Modify: `overall_gilnam/vision/noisy_stream.py` (Task 1에서 생성)
- Test: `overall_gilnam/vision/tests/test_noisy_stream.py` (추가)

**Interfaces:**
- Consumes: Task 1의 `calibrate_mixture`, 기존 `vision_msg.build_window(order_index, color, corners, corner_vis, det_conf, color_conf)`·`build_frame_message(timestamp_ns, frame_id, windows)`·`to_json(msg)`. 입력 스트림 레코드 형식(sample_stream.jsonl 한 줄): `{"vision": <§5 메시지>, "pose": {"timestamp", "frame", "position", "orientation"}}`.
- Produces: `make_noisy_records(records, scale, seed, mean_px, p95_px, p_tail, drop_prob) -> list` (레코드 리스트, pose 불변), `load_records(path) -> list`, `write_records(records, path)`. CLI: `python noisy_stream.py --stream sample_stream/sample_stream.jsonl --out <dir> --scales 0.25,0.5,1,1.5,2,3 --seed 1234` → `<dir>/noisy_x{scale}.jsonl`.

- [ ] **Step 1: 실패하는 테스트 추가** — `tests/test_noisy_stream.py`에 이어서

```python
from vision_msg import N_CORNERS
from noisy_stream import make_noisy_records

SAMPLE_RECORDS = [
    {
        "vision": {
            "timestamp": 1_720_000_000_000_000_000 + i * 33_333_333,
            "frame_id": i,
            "windows": [
                {
                    "order_index": 0, "color": "red",
                    "corners": [[560.0, 298.0], [721.0, 296.0], [721.0, 423.0], [560.0, 421.0]],
                    "corner_vis": [1, 1, 1, 1], "center": [640.5, 359.5],
                    "det_conf": 1.0, "color_conf": 1.0,
                }
            ],
        },
        "pose": {"timestamp": 1_720_000_000_000_000_000 + i * 33_333_333, "frame": "world",
                 "position": [0.1 * i, 0.0, 1.5], "orientation": [0.0, 0.0, 0.0, 1.0]},
    }
    for i in range(200)
]


def test_schema_preserved_and_pose_untouched():
    out = make_noisy_records(SAMPLE_RECORDS, scale=1.0, seed=1234,
                             mean_px=8.87, p95_px=36.6, p_tail=P_TAIL, drop_prob=0.0)
    assert len(out) == len(SAMPLE_RECORDS)
    for rec, src in zip(out, SAMPLE_RECORDS):
        assert rec["pose"] == src["pose"]
        msg = rec["vision"]
        assert isinstance(msg["timestamp"], int) and msg["timestamp"] == src["vision"]["timestamp"]
        for w in msg["windows"]:
            assert len(w["corners"]) == N_CORNERS
            assert w["corner_vis"] == [1, 1, 1, 1]
            assert w["det_conf"] == 1.0  # 기하 외 필드 불변 (설계)


def test_seed_reproducible_and_scale_zero_identity():
    a = make_noisy_records(SAMPLE_RECORDS, 1.0, 1234, 8.87, 36.6, P_TAIL, DEFAULT_DROP)
    b = make_noisy_records(SAMPLE_RECORDS, 1.0, 1234, 8.87, 36.6, P_TAIL, DEFAULT_DROP)
    assert a == b
    zero = make_noisy_records(SAMPLE_RECORDS, 0.0, 1234, 8.87, 36.6, P_TAIL, 0.0)
    src_corners = SAMPLE_RECORDS[0]["vision"]["windows"][0]["corners"]
    assert zero[0]["vision"]["windows"][0]["corners"] == src_corners


def test_drop_probability_applied():
    dropped = make_noisy_records(SAMPLE_RECORDS, 1.0, 1234, 8.87, 36.6, P_TAIL, drop_prob=0.5)
    n_kept = sum(len(r["vision"]["windows"]) for r in dropped)
    assert 60 <= n_kept <= 140  # 이항(200, 0.5)의 넉넉한 구간
```

(파일 상단 import에 `DEFAULT_DROP` 추가: `from noisy_stream import P_TAIL, DEFAULT_DROP, calibrate_mixture, make_noisy_records`)

- [ ] **Step 2: 실패 확인**

Run: `python -m pytest tests/test_noisy_stream.py -v`
Expected: 새 테스트 3개 FAIL — `ImportError: cannot import name 'make_noisy_records'`

- [ ] **Step 3: 구현** — `noisy_stream.py`에 추가

```python
import argparse
import json
from pathlib import Path

import numpy as np

from vision_msg import build_frame_message, build_window, to_json

PX_DECIMALS = 2  # make_stream.py와 동일한 기록 자릿수


def _noisy_windows(windows, rng, sigma_core, sigma_tail, p_tail, drop_prob):
    """§5 windows[] → 노이즈 주입본. 기하(corners·center)만 변경, 드롭은 창문 단위."""
    out = []
    for w in windows:
        if rng.random() < drop_prob:
            continue
        corners = np.asarray(w["corners"], dtype=float)
        sigmas = np.where(rng.random(len(corners)) < p_tail, sigma_tail, sigma_core)
        corners = corners + rng.normal(0.0, 1.0, corners.shape) * sigmas[:, None]
        nw = build_window(
            w["order_index"], w["color"],
            [[round(float(u), PX_DECIMALS), round(float(v), PX_DECIMALS)] for u, v in corners],
            w["corner_vis"], w["det_conf"], w["color_conf"],
        )
        nw["center"] = [round(c, PX_DECIMALS) for c in nw["center"]]
        out.append(nw)
    return out


def make_noisy_records(records, scale, seed, mean_px, p95_px, p_tail, drop_prob):
    """스트림 레코드 리스트 → 배율 scale 노이즈 주입본 (pose 불변, 결정적)."""
    sigma_core, sigma_tail = calibrate_mixture(mean_px, p95_px, p_tail)
    rng = np.random.default_rng([seed, int(round(scale * 100))])  # 배율별 독립 시드
    out = []
    for rec in records:
        msg = rec["vision"]
        windows = _noisy_windows(msg["windows"], rng,
                                 sigma_core * scale, sigma_tail * scale, p_tail, drop_prob)
        out.append({"vision": build_frame_message(msg["timestamp"], msg["frame_id"], windows),
                    "pose": rec["pose"]})
    return out


def load_records(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def write_records(records, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for rec in records:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def main():
    ap = argparse.ArgumentParser(description="GT 스트림 → 실측 프로파일 노이즈 주입 jsonl (배율별)")
    ap.add_argument("--stream", required=True, help="입력 §5+pose jsonl (make_stream.py 산출물)")
    ap.add_argument("--out", required=True, help="출력 디렉터리")
    ap.add_argument("--scales", default="0.25,0.5,1,1.5,2,3")
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--mean", type=float, default=DEFAULT_MEAN_PX)
    ap.add_argument("--p95", type=float, default=DEFAULT_P95_PX)
    ap.add_argument("--drop", type=float, default=DEFAULT_DROP)
    args = ap.parse_args()

    records = load_records(args.stream)
    sc, st = calibrate_mixture(args.mean, args.p95)
    print(f"calibrated: sigma_core={sc:.3f}px sigma_tail={st:.3f}px (p_tail={P_TAIL})")
    for scale in [float(s) for s in args.scales.split(",")]:
        noisy = make_noisy_records(records, scale, args.seed, args.mean, args.p95, P_TAIL, args.drop)
        path = Path(args.out) / f"noisy_x{scale:g}.jsonl"
        write_records(noisy, path)
        n_win = sum(len(r["vision"]["windows"]) for r in noisy)
        print(f"x{scale:g}: {len(noisy)} frames, {n_win} windows → {path}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 전체 테스트 통과 확인**

Run: `python -m pytest tests/ -q`
Expected: 기존 + 신규 전부 PASS

- [ ] **Step 5: CLI 스모크**

Run (`overall_gilnam/vision/`에서): `python noisy_stream.py --stream sample_stream/sample_stream.jsonl --out "$TMP_SWEEP" --scales 1` (`$TMP_SWEEP`은 스크래치 디렉터리)
Expected: calibrated 로그 + `noisy_x1.jsonl` 생성, 창문 수가 원본(약 900)보다 3~5% 적음

- [ ] **Step 6: 커밋**

```bash
git add overall_gilnam/vision/noisy_stream.py overall_gilnam/vision/tests/test_noisy_stream.py
git commit -m "vision: 스트림 노이즈 주입 CLI (배율 스윕·미검출 드롭·시드 재현)"
```

---

### Task 3: 삼각측량 복원 평가기 코어 + 무노이즈 정합 게이트 (`eval_recon3d.py`)

**Files:**
- Create: `overall_gilnam/vision/eval_recon3d.py`
- Test: `overall_gilnam/vision/tests/test_eval_recon3d.py`

**Interfaces:**
- Consumes: 스트림 레코드 형식(Task 2와 동일), `sample_stream/scene_gt.json` (intrinsics: fx/fy/cx/cy, windows[]: order_index/color/center/size_wh/corners_3d). `noisy_stream.load_records`.
- Produces: `evaluate_records(records, scene_gt, min_baseline_m=0.5, max_pairs=2000) -> list[dict]` — 창문별 `{"order_index", "color", "n_pairs", "corner_err_mm": [4], "corner_err_mean_mm", "corner_err_max_mm", "center_err_mm", "size_err_mm": [w, h]}`. 보조: `quat_xyzw_to_rot(q) -> np.ndarray(3,3)`, `projection_matrix(K, R_wc, t_wc) -> np.ndarray(3,4)`.

- [ ] **Step 1: 실패하는 테스트 작성** — `tests/test_eval_recon3d.py`

```python
"""eval_recon3d 테스트 — 무노이즈 정합 게이트(≤1mm)가 핵심."""
import json
from pathlib import Path

from eval_recon3d import evaluate_records
from noisy_stream import load_records

VISION_DIR = Path(__file__).resolve().parents[1]


def _load_sample():
    records = load_records(VISION_DIR / "sample_stream" / "sample_stream.jsonl")
    scene_gt = json.loads((VISION_DIR / "sample_stream" / "scene_gt.json").read_text(encoding="utf-8"))
    return records, scene_gt


def test_noiseless_reconstruction_within_1mm():
    """태민 7/4 결과표(0.01~0.07mm)와 자릿수 정합 — 구현 검증 게이트."""
    records, scene_gt = _load_sample()
    results = evaluate_records(records, scene_gt)
    assert len(results) == 3  # 창문 3개 전부 복원돼야 함
    for r in results:
        assert r["n_pairs"] > 0
        assert r["corner_err_max_mm"] < 1.0
        assert r["center_err_mm"] < 1.0
        assert all(abs(e) < 2.0 for e in r["size_err_mm"])
```

- [ ] **Step 2: 실패 확인**

Run: `python -m pytest tests/test_eval_recon3d.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'eval_recon3d'`

- [ ] **Step 3: 구현** — `eval_recon3d.py`

```python
"""§5+pose 스트림 → 삼각측량 3D 복원 → scene_gt 대조 오차표.

README_stream.md 계약 레시피의 재현 구현 (태민 원본 코드는 리포에 없음):
- P = K·[R_wcᵀ | −R_wcᵀ·t_wc], cv2.triangulatePoints (2-프레임).
- 프레임쌍: 창문 4 corner 모두 vis=1 & 카메라 위치 차 ≥ 0.5m인 모든 쌍
  (max_pairs 초과 시 균등 서브샘플 — 결과에 n_pairs 기록).
- 집계: 쌍별 결과의 corner별 성분 중앙값 (꼬리 노이즈 강건 — 태민 집계 방식과
  다를 수 있음, 결과 문서에 가정으로 명시).
무노이즈 스트림에서 corner ≤ 1mm 정합 게이트를 통과해야 스윕 결과를 신뢰한다.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from noisy_stream import load_records


def quat_xyzw_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def projection_matrix(K, R_wc, t_wc):
    return K @ np.hstack([R_wc.T, (-R_wc.T @ t_wc).reshape(3, 1)])


def _intrinsics_K(scene_gt):
    i = scene_gt["intrinsics"]
    return np.array([[i["fx"], 0.0, i["cx"]], [0.0, i["fy"], i["cy"]], [0.0, 0.0, 1.0]])


def _observations(records, K, order_index):
    """해당 창문이 4 corner 모두 vis=1인 프레임 → (position, P, corners(4,2)) 리스트."""
    obs = []
    for rec in records:
        for w in rec["vision"]["windows"]:
            if w["order_index"] == order_index and all(v == 1 for v in w["corner_vis"]):
                t = np.asarray(rec["pose"]["position"], dtype=float)
                R = quat_xyzw_to_rot(rec["pose"]["orientation"])
                obs.append((t, projection_matrix(K, R, t), np.asarray(w["corners"], dtype=float)))
    return obs


def _triangulate(obs, min_baseline_m, max_pairs):
    """모든 유효 쌍 삼각측량 → corner별 성분 중앙값 (4,3). 반환: (estimate, n_pairs)."""
    pairs = [
        (i, j)
        for i in range(len(obs))
        for j in range(i + 1, len(obs))
        if np.linalg.norm(obs[i][0] - obs[j][0]) >= min_baseline_m
    ]
    if not pairs:
        return None, 0
    if len(pairs) > max_pairs:
        idx = np.linspace(0, len(pairs) - 1, max_pairs).astype(int)
        pairs = [pairs[k] for k in idx]
    estimates = []
    for i, j in pairs:
        X = cv2.triangulatePoints(obs[i][1], obs[j][1], obs[i][2].T, obs[j][2].T)
        estimates.append((X[:3] / X[3]).T)  # (4,3)
    return np.median(np.stack(estimates), axis=0), len(pairs)


def evaluate_records(records, scene_gt, min_baseline_m=0.5, max_pairs=2000):
    K = _intrinsics_K(scene_gt)
    results = []
    for gt in scene_gt["windows"]:
        est, n_pairs = _triangulate(_observations(records, K, gt["order_index"]),
                                    min_baseline_m, max_pairs)
        if est is None:
            continue
        gt_corners = np.asarray(gt["corners_3d"], dtype=float)
        corner_err = np.linalg.norm(est - gt_corners, axis=1) * 1000.0  # mm
        center_err = float(np.linalg.norm(est.mean(axis=0) - np.asarray(gt["center"]))) * 1000.0
        tl, tr, br, bl = est
        w_est = (np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2.0
        h_est = (np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2.0
        results.append({
            "order_index": gt["order_index"],
            "color": gt["color"],
            "n_pairs": n_pairs,
            "corner_err_mm": [round(float(e), 3) for e in corner_err],
            "corner_err_mean_mm": round(float(corner_err.mean()), 3),
            "corner_err_max_mm": round(float(corner_err.max()), 3),
            "center_err_mm": round(center_err, 3),
            "size_err_mm": [round((w_est - gt["size_wh"][0]) * 1000.0, 3),
                            round((h_est - gt["size_wh"][1]) * 1000.0, 3)],
        })
    return results
```

- [ ] **Step 4: 통과 확인 (정합 게이트)**

Run: `python -m pytest tests/test_eval_recon3d.py -v`
Expected: PASS. FAIL이면 스윕 진행 금지 — 투영 관례(전치·부호)부터 재점검.

- [ ] **Step 5: 커밋**

```bash
git add overall_gilnam/vision/eval_recon3d.py overall_gilnam/vision/tests/test_eval_recon3d.py
git commit -m "vision: 삼각측량 복원 평가기 — 무노이즈 1mm 정합 게이트 통과"
```

---

### Task 4: 스윕 CLI + 노이즈 증가 sanity 테스트 (`eval_recon3d.py` 완성)

**Files:**
- Modify: `overall_gilnam/vision/eval_recon3d.py`
- Test: `overall_gilnam/vision/tests/test_eval_recon3d.py` (추가)

**Interfaces:**
- Consumes: Task 3의 `evaluate_records`, Task 2의 `make_noisy_records`·`load_records`.
- Produces: CLI `python eval_recon3d.py --scene-gt <path> --streams <jsonl...> [--json <out>]` → stdout에 markdown 표(스트림×창문 행 + 스트림별 집계 행), `--json`에 기계 판독 결과. `summarize(label, results) -> dict` — `{"label", "corner_err_mean_mm", "corner_err_max_mm", "center_err_mean_mm", "size_err_mean_abs_mm"}` (창문 평균/최대).

- [ ] **Step 1: 실패하는 테스트 추가** — `tests/test_eval_recon3d.py`에 이어서

```python
from noisy_stream import P_TAIL, make_noisy_records
from eval_recon3d import summarize


def test_error_grows_with_noise_scale():
    records, scene_gt = _load_sample()
    err = {}
    for scale in (0.0, 0.5, 2.0):
        noisy = make_noisy_records(records, scale, 1234, 8.87, 36.6, P_TAIL, drop_prob=0.0)
        s = summarize(f"x{scale}", evaluate_records(noisy, scene_gt))
        err[scale] = s["center_err_mean_mm"]
    assert err[0.0] < err[0.5] < err[2.0]
```

- [ ] **Step 2: 실패 확인**

Run: `python -m pytest tests/test_eval_recon3d.py -v`
Expected: 새 테스트 FAIL — `ImportError: cannot import name 'summarize'`

- [ ] **Step 3: 구현** — `eval_recon3d.py`에 추가

```python
def summarize(label, results):
    """창문별 결과 → 스트림 1개 집계 행."""
    return {
        "label": label,
        "corner_err_mean_mm": round(float(np.mean([r["corner_err_mean_mm"] for r in results])), 3),
        "corner_err_max_mm": round(max(r["corner_err_max_mm"] for r in results), 3),
        "center_err_mean_mm": round(float(np.mean([r["center_err_mm"] for r in results])), 3),
        "size_err_mean_abs_mm": round(
            float(np.mean([abs(e) for r in results for e in r["size_err_mm"]])), 3),
    }


def main():
    ap = argparse.ArgumentParser(description="스트림(들) → 삼각측량 3D 복원 오차표 (markdown)")
    ap.add_argument("--scene-gt", required=True)
    ap.add_argument("--streams", required=True, nargs="+")
    ap.add_argument("--json", help="기계 판독 결과 저장 경로 (선택)")
    args = ap.parse_args()

    scene_gt = json.loads(Path(args.scene_gt).read_text(encoding="utf-8"))
    all_out = []
    print("| 스트림 | 창문 | n_pairs | corner 평균/최대 (mm) | 중심 (mm) | 크기 w/h (mm) |")
    print("|---|---|---|---|---|---|")
    for path in args.streams:
        label = Path(path).stem
        results = evaluate_records(load_records(path), scene_gt)
        for r in results:
            print(f"| {label} | {r['order_index']} ({r['color']}) | {r['n_pairs']} "
                  f"| {r['corner_err_mean_mm']} / {r['corner_err_max_mm']} "
                  f"| {r['center_err_mm']} | {r['size_err_mm'][0]} / {r['size_err_mm'][1]} |")
        summary = summarize(label, results)
        all_out.append({"summary": summary, "windows": results})
        print(f"| **{label} 집계** | 창문 {len(results)}개 | — "
              f"| **{summary['corner_err_mean_mm']} / {summary['corner_err_max_mm']}** "
              f"| **{summary['center_err_mean_mm']}** | 평균절대 {summary['size_err_mean_abs_mm']} |")
    if args.json:
        Path(args.json).write_text(json.dumps(all_out, ensure_ascii=False, indent=2),
                                   encoding="utf-8")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 전체 테스트 통과 확인**

Run: `python -m pytest tests/ -q`
Expected: 전부 PASS

- [ ] **Step 5: 커밋**

```bash
git add overall_gilnam/vision/eval_recon3d.py overall_gilnam/vision/tests/test_eval_recon3d.py
git commit -m "vision: 복원 오차 스윕 CLI + 노이즈 단조 증가 sanity 테스트"
```

---

### Task 5: 스윕 실행 + 태민 패키지 생성

**Files:**
- Create: `overall_gilnam/vision/noisy_stream_x1/` (jsonl + scene_gt.json 사본 + README.md — 커밋 대상)
- 스크래치: 배율별 jsonl 6개 + sweep 결과 json (git 제외 — 스크래치 디렉터리 사용)

**Interfaces:**
- Consumes: Task 2·4의 CLI. `sample_stream/sample_stream.jsonl`(302프레임)·`scene_gt.json`.
- Produces: 스윕 결과 markdown 표 + json (Task 6의 문서 입력), 태민 전달 패키지.

- [ ] **Step 1: 스윕 스트림 생성**

Run (`overall_gilnam/vision/`에서, `$SWEEP`은 스크래치 디렉터리):

```bash
python noisy_stream.py --stream sample_stream/sample_stream.jsonl --out "$SWEEP" --scales 0.25,0.5,1,1.5,2,3 --seed 1234
```

Expected: calibrated σ 로그 + jsonl 6개. σc·σt 값을 기록해 둔다 (문서에 기재).

- [ ] **Step 2: 스윕 평가 실행**

```bash
python eval_recon3d.py --scene-gt sample_stream/scene_gt.json \
  --streams sample_stream/sample_stream.jsonl "$SWEEP"/noisy_x0.25.jsonl "$SWEEP"/noisy_x0.5.jsonl \
            "$SWEEP"/noisy_x1.jsonl "$SWEEP"/noisy_x1.5.jsonl "$SWEEP"/noisy_x2.jsonl "$SWEEP"/noisy_x3.jsonl \
  --json "$SWEEP"/sweep_results.json
```

Expected: 무노이즈 행이 mm 미만, 배율 증가에 따라 오차 증가하는 markdown 표. 표 전문을 보존 (Task 6 입력).

- [ ] **Step 3: 태민 패키지 생성**

```bash
mkdir -p noisy_stream_x1
python noisy_stream.py --stream sample_stream/sample_stream.jsonl --out noisy_stream_x1 --scales 1 --seed 1234
cp sample_stream/scene_gt.json noisy_stream_x1/
```

`noisy_stream_x1/README.md` 작성 — 내용:

```markdown
# noisy_stream_x1 — 실측 노이즈 주입 §5 + GT pose 스트림 (태민 재검증용)

> 원본: sample_stream/ (seed 42, 302프레임). 노이즈: 1차 학습 모델 실측
> (corner 반경 오차 평균 8.87px / p95 36.6px, 2026-08-02 본 판정)을 2성분
> 가우시안 혼합으로 정합, 미검출 3.9%는 창문 단위 드롭.
> 재생성: `python noisy_stream.py --stream sample_stream/sample_stream.jsonl --out noisy_stream_x1 --scales 1 --seed 1234`

- 형식·좌표 관례는 sample_stream/README_stream.md와 동일 (§5 + pose, scene_gt.json 사본 포함).
- det_conf/color_conf는 GT값(1.0) 유지 — 기하 노이즈만 주입.
- 요청: 7/4과 동일한 방법으로 3D 복원 → 오차 공유 (길남 재현 구현과 교차 검증,
  결과 비교표: overall_gilnam/docs/eval_target_derivation.md).
```

- [ ] **Step 4: 커밋**

```bash
git add overall_gilnam/vision/noisy_stream_x1/
git commit -m "vision: 태민 재검증용 실측 노이즈 스트림 패키지 (x1.0)"
```

---

### Task 6: 결과 문서 (역산·목표 제안) + 체크리스트 갱신 + 문서 커밋

**Files:**
- Create: `overall_gilnam/docs/eval_target_derivation.md`
- Modify: `overall_gilnam/docs/To_do_checklist_gilnam.md` (0번 ⓐ·3번 재검증 항목)
- 커밋 포함: `overall_gilnam/docs/superpowers/specs/2026-08-08-recon3d-noise-target-design.md`, `overall_gilnam/docs/superpowers/plans/2026-08-08-recon3d-noise-target.md`, 미커밋 체크리스트 정리분

**Interfaces:**
- Consumes: Task 5의 스윕 표·json, 캘리브레이션 σ 값.
- Produces: 회의 안건용 목표 제안 문서.

- [ ] **Step 1: 역산 계산**

`sweep_results.json`에서 배율별 유효 잠식 `침범_mm = center_err_mean_mm + size_err_mean_abs_mm / 2`을 계산한다. margin {50, 100, 150}mm 각각에 대해 `침범 ≤ margin`을 만족하는 최대 배율을 찾고, 대응 픽셀 오차 = 8.87 × 배율 (배율 사이는 선형 보간). 계산 결과가 스윕 범위를 벗어나면(전 배율 통과 등) 그대로 기록 — 과외삽 금지.

- [ ] **Step 2: `eval_target_derivation.md` 작성**

구성 (설계 문서의 산출물 정의 그대로):

```markdown
# eval 목표치 역산 — 실측 노이즈 주입 3D 복원 실험

> 2026-08-08 · 류길남 · 체크리스트 0번 ⓐ 결과 (회의 안건)
> 방법·코드: vision/noisy_stream.py, vision/eval_recon3d.py
> (설계: overall_gilnam/docs/superpowers/specs/2026-08-08-recon3d-noise-target-design.md)

## 요약 (결론 먼저)
[margin별 허용 픽셀 오차 표 + 목표치 제안 1~2문장 + 현 모델(8.87px) 합격 여부]

## 방법
[노이즈 모델·캘리브레이션 σc/σt 값, 삼각측량 재현·중앙값 집계, 무노이즈 정합 게이트 결과]

## 픽셀 → 3D 오차 곡선 (스윕 결과)
[Task 5의 markdown 표 전문]

## margin 역산
[Step 1 계산 표: margin 50/100/150mm × 허용 배율 × 허용 픽셀 오차]

## 한계·가정
[iid 가정(3D 오차 과소평가 방향), 미검출 배율 무관 고정, 집계 방식 가정 — 설계 문서와 동일 3항]

## 태민 교차검증 요청
[noisy_stream_x1/ 패키지 안내 + 비교표 자리(태민 결과 수령 후 기입)]
```

수치는 전부 Task 5 실측 결과로 채운다 — 자리표시자 금지.

- [ ] **Step 3: 체크리스트 갱신**

`To_do_checklist_gilnam.md`:
- 0번 "eval_corners 본 판정 결과 → 목표치 확정 필요" 항목에 진행 기록 추가: ⓐ 실험 완료 (`docs/eval_target_derivation.md`), margin 확정만 회의 잔여.
- 3번 "실모델 출력으로 재검증" → `- [x]` 부분 완료 처리: 합성 노이즈 재현 구현으로 완료, 태민 교차 확인 잔여 (`noisy_stream_x1/` 전달).
- 하단 갱신 이력에 1줄 추가: `*2026-08-08 갱신 2: 노이즈 주입 3D 복원 역산 실험 완료 — eval_target_derivation.md, 태민 패키지 전달 대기.*`

- [ ] **Step 4: 전체 테스트 최종 확인**

Run (`overall_gilnam/vision/`에서): `python -m pytest tests/ -q`
Expected: 전부 PASS

- [ ] **Step 5: 문서 일괄 커밋** (사용자가 미뤄둔 체크리스트 정리분 포함)

```bash
git add overall_gilnam/docs/eval_target_derivation.md overall_gilnam/docs/To_do_checklist_gilnam.md \
        overall_gilnam/docs/superpowers/
git commit -m "docs: eval 목표치 역산 결과 (margin별 허용 픽셀 오차) + 체크리스트 갱신"
```

---

## Self-Review 기록

- 스펙 커버리지: 구성요소 2파일(T1~4), 노이즈 모델(T1·2), 복원 평가·정합 게이트(T3), 스윕(T4·5), 산출물 3종(T5·6), 성공 기준 ①=T3 Step4 ②=T1 테스트 ③=T5·6 ④=T4·6 — 전 항목 태스크 대응 확인.
- 자리표시자: 결과 문서 골격(T6)은 실측값 기입 지시로 대체 — 실행 전에는 수치가 존재하지 않으므로 계획상 허용, "자리표시자 금지" 명시로 방어.
- 타입 일관성: `make_noisy_records` 시그니처(T2 정의 = T4 테스트 사용), `evaluate_records`/`summarize` 반환 키(T3 정의 = T4·6 사용) 대조 완료.
