# margin 역산 재계산 (v2) 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 태민 실방식(LS 시선 교점) 재현을 추가해 두 집계로 margin 역산을 재산출하고, worst-window 판정·실현 가능 margin 상한을 반영해 eval_target_derivation.md의 결론을 정정한다.

**Architecture:** `eval_recon3d.py`에 `reconstruct_windows_rays` + `method` 분기 추가(기존 경로 불변), 재실행은 스크래치 스크립트(CLI 확장 없음), 문서 4곳 일관 개정.

**Tech Stack:** Python, numpy, pytest. 새 의존성 금지.

**Spec:** `overall_gilnam/docs/superpowers/specs/2026-08-08-margin-inversion-v2-design.md`

## Global Constraints

- Python: `C:\Users\user\anaconda3\python.exe` (PATH `python`은 스토어 스텁 — 금지). 스크립트 실행 시 `PYTHONIOENCODING=utf-8`.
- vision 테스트는 `overall_gilnam/vision`에서 `-m pytest tests/ -q` — 기대: 기존 41 + 신규 2 = 43 passed + 1 기존 환경 실패(test_toy_and_eval, OpenCV 비ASCII — 무시).
- 태민 코드(`visual_imaging_taemin/window_recon_node.py`)는 **읽기 전용 재현 대상** — 절대 수정 금지. 서브모듈 절대 add 금지 (`git add`는 명시 경로만).
- 실측 상수: 시드 1234, 배율 {0.25,0.5,1,1.5,2,3}, 스팟 시드 {1234,7,99,2026} × 배율 {0.5,0.75,1.0,1.25,1.5}, margin {50,100,150}mm, 허용 픽셀 = 8.87 × 배율(선형 보간, 과외삽 금지), r_body=0.35m, e_track ∈ {0, 50}mm.
- 판정 v2: 창문별 침범_i = center_err_i_mm + mean(|Δw_i|,|Δh_i|)/2, 판정값 = **max_i 침범_i**.
- 감사 수치(4.75/6.93/8.22px 등)는 참조만 — 전부 독립 재산출, 불일치 시 그대로 기록.
- 주석·문서 한국어. 커밋 메시지는 태스크 명시 문구 + 트레일러:

```
Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
```

---

### Task 1: 태민 방식 재현 (`reconstruct_windows_rays`) + method 분기

**Files:**
- Modify: `overall_gilnam/vision/eval_recon3d.py`
- Test: `overall_gilnam/vision/tests/test_eval_recon3d.py` (추가)

**Interfaces:**
- Produces: `reconstruct_windows_rays(records, scene_gt, det_conf_min=0.7, min_parallax_deg=2.0) -> dict[int, dict]` (반환 형식은 `reconstruct_windows`와 동일 — `n_pairs` 키에 관측 수); `evaluate_records(records, scene_gt, min_baseline_m=0.5, max_pairs=2000, method="pairs_median")` — `"rays_ls"` 분기 + 방식별 크기 계산 `_size_wh_est(est, method)`.
- Consumes: 기존 `quat_xyzw_to_rot`, `reconstruct_windows`. 재현 원본: `visual_imaging_taemin/window_recon_node.py`의 CornerAccumulator·det_cb·report 수치 경로.

- [ ] **Step 1: 실패하는 테스트 추가** — `tests/test_eval_recon3d.py`에 이어서

```python
def test_rays_reconstruction_matches_taemin_table():
    # 태민 방식(LS 시선 교점) 재현 게이트 — 무노이즈에서 태민 7/4 표(0.01~0.07mm)와 자릿수 정합
    records, scene_gt = _load_sample()
    results = evaluate_records(records, scene_gt, method="rays_ls")
    assert len(results) == 3
    for r in results:
        assert r["n_pairs"] > 0
        assert r["corner_err_max_mm"] < 1.0


def test_pairs_median_unchanged_by_method_param():
    # 기존 경로 회귀 없음: method 기본값과 명시가 동일 결과
    records, scene_gt = _load_sample()
    assert evaluate_records(records, scene_gt) == evaluate_records(records, scene_gt, method="pairs_median")
```

- [ ] **Step 2: 실패 확인**

Run (`overall_gilnam/vision/`에서): `C:\Users\user\anaconda3\python.exe -m pytest tests/test_eval_recon3d.py -v`
Expected: 신규 2개 FAIL — `TypeError: evaluate_records() got an unexpected keyword argument 'method'`

- [ ] **Step 3: 구현** — `eval_recon3d.py`

`reconstruct_windows` 아래에 추가:

```python
def reconstruct_windows_rays(records, scene_gt, det_conf_min=0.7, min_parallax_deg=2.0):
    """태민 window_recon_node.py의 수치 경로 재현 (corner별 전 관측 시선 LS 교점).

    재현 범위: det_conf ≥ det_conf_min 창문의 corner_vis=1 관측만,
    시선 d = normalize(((u−cx)/fx, (v−cy)/fy, 1)) → world 변환(body≡camera 항등 —
    오프라인 재현이므로 태민 노드의 EuRoC T_IC는 적용하지 않음, 방식 차이 측정이 목적),
    corner별 A += I − ddᵀ, b += (I−ddᵀ)c 누적 후 3×3 해.
    채택 조건도 태민 코드 그대로: corner 4개 전부 해 존재 + corner별 시차각
    (방향행렬 D의 최소 내적의 arccos)의 창문 최소가 min_parallax_deg 이상.
    반환 형식은 reconstruct_windows와 동일 — 단 n_pairs 키에는 '관측 수 합'을 기록
    (쌍 수가 아님, 소비측 주의). 실패 창문은 corners_3d_est=None, n_pairs=0.
    """
    intr = scene_gt["intrinsics"]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    colors = {w["order_index"]: w["color"] for w in scene_gt["windows"]}
    acc = {}  # (order_index, ci) -> [A(3,3), b(3,), dirs(list)]
    for rec in records:
        R = quat_xyzw_to_rot(rec["pose"]["orientation"])
        c = np.asarray(rec["pose"]["position"], dtype=float)
        for win in rec["vision"]["windows"]:
            if win["det_conf"] < det_conf_min:
                continue
            oi = win["order_index"]
            for ci in range(4):
                if win["corner_vis"][ci] != 1:
                    continue
                u, v = win["corners"][ci]
                d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
                d = R @ (d / np.linalg.norm(d))
                M = np.eye(3) - np.outer(d, d)
                a = acc.setdefault((oi, ci), [np.zeros((3, 3)), np.zeros(3), []])
                a[0] += M
                a[1] += M @ c
                a[2].append(d)
    out = {}
    for gt in scene_gt["windows"]:
        oi = gt["order_index"]
        pts, min_ang, n_obs = [], float("inf"), 0
        for ci in range(4):
            a = acc.get((oi, ci))
            if a is None or len(a[2]) < 2:
                pts = None
                break
            p = np.linalg.solve(a[0], a[1])
            D = np.asarray(a[2])
            ang = float(np.degrees(np.arccos(np.clip((D @ D.T).min(), -1.0, 1.0))))
            pts.append(p)
            min_ang = min(min_ang, ang)
            n_obs += len(a[2])
        if pts is None or min_ang < min_parallax_deg:
            out[oi] = {"color": colors.get(oi), "corners_3d_est": None, "n_pairs": 0}
        else:
            out[oi] = {"color": colors.get(oi), "corners_3d_est": np.asarray(pts), "n_pairs": n_obs}
    return out


def _size_wh_est(est, method):
    """복원 corner → (w, h). 방식별 관례: 태민(rays_ls)은 단일 변, 기본은 양변 평균."""
    tl, tr, br, bl = est
    if method == "rays_ls":  # window_recon_node.report와 동일: w=|TR−TL|, h=|BR−TR|
        return float(np.linalg.norm(tr - tl)), float(np.linalg.norm(br - tr))
    w = (np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2.0
    h = (np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2.0
    return float(w), float(h)
```

`evaluate_records` 변경: 시그니처를 `def evaluate_records(records, scene_gt, min_baseline_m=0.5, max_pairs=2000, method="pairs_median"):`로. 함수 첫 줄의 recon 계산을:

```python
    if method == "rays_ls":
        recon = reconstruct_windows_rays(records, scene_gt)
    else:
        recon = reconstruct_windows(records, scene_gt, min_baseline_m, max_pairs)
```

기존 크기 계산 2줄(`w_est = ...`, `h_est = ...`)을 `w_est, h_est = _size_wh_est(est, method)`로 교체. 나머지 불변.

- [ ] **Step 4: 전체 테스트 통과 확인**

Run: `C:\Users\user\anaconda3\python.exe -m pytest tests/ -q`
Expected: 43 passed, 1 failed (기존 환경 실패만). 신규 게이트 FAIL 시 진행 금지 — 태민 코드와 축·부호 재대조.

- [ ] **Step 5: 커밋**

```bash
git add overall_gilnam/vision/eval_recon3d.py overall_gilnam/vision/tests/test_eval_recon3d.py
git commit -m "vision: 태민 실방식(LS 시선 교점) 재현 reconstruct_windows_rays + evaluate method 분기"
```

---

### Task 2: 재산출 실행 — 두 집계 스윕·시드 체크·worst-window 역산·실현 가능성

**Files:**
- 스크래치 실행 스크립트 (git 밖): `<스크래치>/margin_v2.py` → 결과 `<스크래치>/margin_v2_results.json` + 표 stdout
- 커밋 없음 (Task 3의 문서 입력)

**Interfaces:**
- Consumes: Task 1의 `evaluate_records(method=...)`, `noisy_stream.make_noisy_records/load_records/P_TAIL/DEFAULT_DROP`.
- Produces: ① 배율×집계별 창문 침범·max-침범 표 ② margin {50,100,150} × 집계별 허용 배율·허용 픽셀 (worst-window, 보간) ③ 시드 4개 × 집계별 허용 픽셀 @50/100 ④ 창문별 실현 가능 margin 상한 (r_body 0.35, e_track {0,50}mm) — 전부 JSON+stdout.

- [ ] **Step 1: 스크립트 작성** — `<스크래치>/margin_v2.py`

```python
# margin 역산 v2 재산출 — 두 집계(pairs_median/rays_ls) × worst-window × 실현 가능성
import json
import sys
from pathlib import Path

VISION = Path(r"C:\Users\user\Desktop\ugrp34\overall_gilnam\vision")
sys.path.insert(0, str(VISION))

from noisy_stream import P_TAIL, DEFAULT_DROP, load_records, make_noisy_records
from eval_recon3d import evaluate_records

MEAN_PX, P95_PX = 8.87, 36.6
SCALES = [0.25, 0.5, 1.0, 1.5, 2.0, 3.0]
SPOT_SCALES = [0.5, 0.75, 1.0, 1.25, 1.5]
SEEDS = [1234, 7, 99, 2026]
MARGINS = [50.0, 100.0, 150.0]
METHODS = ["pairs_median", "rays_ls"]
R_BODY_MM, E_TRACK_MM = 350.0, [0.0, 50.0]

records = load_records(VISION / "sample_stream" / "sample_stream.jsonl")
scene_gt = json.loads((VISION / "sample_stream" / "scene_gt.json").read_text(encoding="utf-8"))


def invasion_rows(results):
    """창문별 침범_i = center_err + mean(|Δw|,|Δh|)/2. 반환: (rows, max침범). 복원 실패는 inf."""
    rows, worst = [], 0.0
    for r in results:
        if r["n_pairs"] == 0:
            rows.append({"order_index": r["order_index"], "invasion_mm": None})
            worst = float("inf")
            continue
        inv = r["center_err_mm"] + (abs(r["size_err_mm"][0]) + abs(r["size_err_mm"][1])) / 2.0 / 2.0
        rows.append({"order_index": r["order_index"], "invasion_mm": round(inv, 1)})
        worst = max(worst, inv)
    return rows, worst


def allowable_px(points, margin):
    for (s0, v0), (s1, v1) in zip(points, points[1:]):
        if v0 <= margin < v1:
            return round(MEAN_PX * (s0 + (s1 - s0) * (margin - v0) / (v1 - v0)), 2)
    return None  # 과외삽 금지 — 범위 밖


def eval_scale(scale, seed, method):
    stream = records if scale == 0 else make_noisy_records(
        records, scale, seed, MEAN_PX, P95_PX, P_TAIL, DEFAULT_DROP)
    return evaluate_records(stream, scene_gt, method=method)


out = {"sweep": {}, "inversion": {}, "spot": {}, "feasibility": []}
for method in METHODS:
    pts = []
    out["sweep"][method] = []
    for scale in SCALES:
        rows, worst = invasion_rows(eval_scale(scale, 1234, method))
        out["sweep"][method].append({"scale": scale, "windows": rows, "worst_mm": round(worst, 1)})
        pts.append((scale, worst))
        print(f"[{method}] x{scale:g}: worst 침범 {worst:.1f}mm  {rows}", flush=True)
    out["inversion"][method] = {f"margin{int(m)}": allowable_px(pts, m) for m in MARGINS}
    print(f"[{method}] 역산: {out['inversion'][method]}")

for method in METHODS:
    out["spot"][method] = []
    for seed in SEEDS:
        pts = [(s, invasion_rows(eval_scale(s, seed, method))[1]) for s in SPOT_SCALES]
        row = {"seed": seed,
               "margin50_px": allowable_px(pts, 50.0), "margin100_px": allowable_px(pts, 100.0)}
        out["spot"][method].append(row)
        print(f"[{method}] seed {seed}: @50={row['margin50_px']} @100={row['margin100_px']}")

for w in scene_gt["windows"]:
    half_min = min(w["size_wh"]) / 2.0 * 1000.0
    caps = {f"etrack{int(e)}": round(half_min - R_BODY_MM - e, 1) for e in E_TRACK_MM}
    out["feasibility"].append({"order_index": w["order_index"], "color": w["color"],
                               "half_min_mm": round(half_min, 1), **caps})
    print(f"실현가능성 W{w['order_index']}({w['color']}): 반치수 {half_min:.1f}mm → 상한 {caps}")

Path(sys.argv[1]).write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
print(f"saved -> {sys.argv[1]}")
```

- [ ] **Step 2: 실행 및 sanity 확인**

Run (`overall_gilnam/vision/`에서): `PYTHONIOENCODING=utf-8` + `C:\Users\user\anaconda3\python.exe <스크래치>/margin_v2.py <스크래치>/margin_v2_results.json`
Sanity: pairs_median worst-침범이 v1 평균-침범보다 크거나 같음 (worst ≥ mean) / rays_ls 허용 픽셀이 pairs_median보다 작거나 같음 (평균 성격 ≥ 민감) / 실현 가능성 상한이 창문 반치수−350mm−e와 일치. 위배 시 STOP — 계산 재점검.

- [ ] **Step 3: 감사 수치와 대조 기록**

감사 참조치(rays 기준 4.75/6.93/8.22px @50/100/150, 슬랙 68.9~95.5mm)와 재산출치의 일치/불일치를 스크래치 노트에 기록 — Task 3 문서의 "감사 대조" 각주 입력.

---

### Task 3: 문서 4곳 개정 + 체크리스트 + 커밋

**Files:**
- Modify: `overall_gilnam/docs/eval_target_derivation.md` (전면 개정), `overall_gilnam/vision/eval_recon3d.py` (docstring 정정), `overall_gilnam/docs/e2e_rehearsal_report.md` (인용 1문장), `overall_gilnam/docs/To_do_checklist_gilnam.md` (0번 갱신)
- 커밋 포함: spec·plan (`overall_gilnam/docs/superpowers/{specs,plans}/2026-08-08-margin-inversion-v2*.md`)

**Interfaces:**
- Consumes: Task 2의 결과 JSON·표 전문·감사 대조 기록.

- [ ] **Step 1: eval_target_derivation.md 전면 개정** — 구성:

```markdown
# eval 목표치 역산 v2 — 두 집계 · worst-window · 실현 가능성

> [v1 이력 + v2 개정 사유 3줄: 집계 가정 오류(태민 코드 존재 — grep 착오 정정 고지)·worst 희석·실현 가능성]

## 요약 (결론 먼저)
[두 집계 × margin 판정표 (worst-window 기준) + 현 모델 8.87px 판정 + 실현 가능 margin 상한 결론
 + 권고 (ⓑ 재학습 여부 — 재산출 결과가 가리키는 대로)]

## 방법 v2 (v1과의 차이)
[태민 방식 재현(수치 경로·재현 한계), 침범 공식 창문별 적용 + max 판정, 크기 계산 방식별 관례]

## 재산출 표
[Task 2 스윕 표(두 집계)·역산표·시드 스팟 체크·실현 가능성 표 전문]

## 감사 대조
[감사 참조치와 재산출치 일치/불일치 명시]

## 한계·가정
[v1 3항 승계 + v2 추가: rays 재현은 ROS 배관 생략, det_conf 필터는 GT 스트림에서 무효]

## 태민 교차검증 요청 (v1 승계)
[기존 표 유지 + rays_ls 재현치 열 추가 가능하면 추가]
```

수치는 전부 Task 2 산출물 — 자리표시자 금지. v1 파일을 이 내용으로 교체 (이력은 git).

- [ ] **Step 2: 나머지 3곳 정정**

- `eval_recon3d.py` 모듈 docstring: "태민 원본 코드는 리포에 없음" 문장 → "태민 원본은 `visual_imaging_taemin/window_recon_node.py`(LS 시선 교점) — `reconstruct_windows_rays`가 그 수치 경로의 오프라인 재현, 기본 `reconstruct_windows`는 쌍별+중앙값(비교용 강건 집계)"로 교체.
- `e2e_rehearsal_report.md`: "8.87px는 50mm에서 FAIL, 100mm에서 PASS" 인용 문장을 v2 결론으로 교체 (재산출 결과 기준, `eval_target_derivation.md` v2 참조 표기).
- `To_do_checklist_gilnam.md` 0번 eval 항목: v2 재산정 완료 기록 추가 (두 집계·worst-window·실현 가능성 반영, 결론 요약 1줄) + 하단 갱신 이력 1줄.

- [ ] **Step 3: 전체 테스트 최종 확인**

Run (`overall_gilnam/vision/`에서): `-m pytest tests/ -q` → 43 passed + 1 기존 환경 실패.

- [ ] **Step 4: 커밋**

```bash
git add overall_gilnam/docs/eval_target_derivation.md overall_gilnam/vision/eval_recon3d.py \
        overall_gilnam/docs/e2e_rehearsal_report.md overall_gilnam/docs/To_do_checklist_gilnam.md \
        overall_gilnam/docs/superpowers/specs/2026-08-08-margin-inversion-v2-design.md \
        overall_gilnam/docs/superpowers/plans/2026-08-08-margin-inversion-v2.md
git commit -m "docs: margin 역산 v2 — 태민 실방식 병행·worst-window 판정·실현 가능성 상한 (v1 결론 정정)"
```

---

## Self-Review 기록

- 스펙 커버리지: rays 재현+method 분기(T1), 재산출 4종(T2), 문서 4곳+체크리스트(T3), 게이트 1=T1 테스트, 성공 기준 2=T2, 3=T3 — 전 항목 대응.
- 자리표시자: T3 문서 골격은 실측 기입 지시로 대체 (실행 전 수치 부재).
- 타입 일관성: `evaluate_records(method=...)` (T1 정의 = T2 사용), `reconstruct_windows_rays` 반환 키 = `reconstruct_windows`와 동일 (T1 명시), 침범 공식 (스펙 = T2 코드 `(|Δw|+|Δh|)/2/2` = mean(|Δw|,|Δh|)/2) 대조 완료.
