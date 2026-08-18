"""태민 window_recon_node.py 의 **수치 경로만** 우리 입맛대로 다시 쓴 것.

    from overrides import recon_rays
    results = recon_rays.reconstruct(samples)      # taemin_bridge.run_offline 과 같은 반환

그의 파일은 안 건드린다. 이건 나란히 놓고 재기 위한 우리 버전이고, 숫자가 좋으면
근거를 들고 그에게 제안하면 된다.

원본이 하는 일 (window_recon_node.py:39-58, 125-160)
----------------------------------------------------
코너별로 시선을 누적해서 최소자승 교점을 푼다.

    M = I − ddᵀ ;  A += M ;  b += M·c ;  p = A⁻¹b

이건  min_p Σ_k ‖(I − d_k d_kᵀ)(p − c_k)‖²  — 모든 시선까지의 수직거리 제곱합
최소화다. O(1) 증분이라 실시간에는 이상적인 형태.

바꾼 것 4가지와 이유
--------------------
1. **conf 가중**  ``A += w·M``.  원본은 det_conf 를 0.7 문턱에만 쓰고 통과 후엔
   버린다. conf 0.71 과 0.99 가 같은 표였다.

2. **Huber IRLS (아웃라이어 제거)**  원본은 순수 L2 라 breakdown point 가 0 이다.
   나쁜 시선 하나가 제곱으로 해를 끌어당긴다. 실제로 green 창문에 blue 창문 시선이
   섞이면서 복원 중심이 x=2.37 → 3.22 로 밀렸다 (857 mm). 잔차 기반으로 재가중해
   그 시선들을 죽인다.

3. **크기 산출에 코너 4개 다 씀**  원본은 w=|TR−TL|, h=|BR−TR| 로 한 변씩만 본다.
   마주보는 변을 평균하면 코너 하나가 틀렸을 때 오차가 절반으로 준다.
   ``size="taemin"`` 으로 원본 방식도 선택 가능 (숫자 비교할 때).

4. **거절 통계 반환**  ``n_rejected`` / ``inlier_frac`` 을 같이 낸다. 뭐가 얼마나
   버려졌는지 안 보이면 튜닝을 못 한다.

바꾸지 않은 것 (일부러)
-----------------------
* 게이트는 그대로: 코너 4개 전부 해가 있어야 하고, 최소 시차각 ≥ 2°.
* T_IC 적용 방식 그대로: R_WC = R_WI·R_IC, c_W = p_WI + R_WI·p_IC.
* 반환 dict 키 그대로 — 하류(metrics, 그림, planner)가 구분 없이 먹는다.

**포기한 것**: 원본의 O(1) 증분 누적. IRLS 는 재가중하려면 시선을 다 들고 있어야
해서 메모리가 O(N) 이고 report 때 O(N·iter) 를 쓴다. 창문당 코너 4개 × 수백 관측
규모라 오프라인·저속 리포트에선 무의미한 비용이지만, 그가 30 Hz 로 돌리는 노드에
그대로 넣을 물건은 아니다. 그에게 제안한다면 "리포트 주기에만 IRLS 한 번" 형태.
"""
from __future__ import annotations

from typing import Dict, List, Sequence

import numpy as np

DET_CONF_MIN = 0.7        # 원본 기본값과 동일
MIN_PARALLAX_DEG = 2.0    # 원본 기본값과 동일
HUBER_ITERS = 6
HUBER_DELTA_MIN_M = 0.06  # 잔차 스케일 하한 [m]. 검출 코너가 몇 픽셀만 흔들려도
                          # 먼 창문에선 수 cm 잔차가 나온다 — 2 cm 는 너무 빡빡해서
                          # 정상 광선까지 아웃라이어로 잘렸다


def _rot(q) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def _solve(C: np.ndarray, D: np.ndarray, w: np.ndarray):
    """가중 최소자승 시선 교점. C:(n,3) 카메라중심, D:(n,3) 단위시선, w:(n,)."""
    # M_k = I − d_k d_kᵀ  를 하나씩 안 만들고 한 번에 누적
    A = np.einsum("k,ij->ij", w, np.eye(3)) - np.einsum("k,ki,kj->ij", w, D, D)
    b = (w[:, None] * C).sum(0) - np.einsum("k,ki,kj,kj->i", w, D, D, C)
    try:
        return np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return None


def _residuals(p: np.ndarray, C: np.ndarray, D: np.ndarray) -> np.ndarray:
    """점 p 에서 각 시선까지의 수직거리 [m]."""
    v = p - C                                   # (n,3)
    par = (v * D).sum(1)[:, None] * D           # 시선 방향 성분
    return np.linalg.norm(v - par, axis=1)


def _robust_scale(r: np.ndarray) -> float:
    """MAD 기반 잔차 스케일. 정규분포면 sigma 와 같아지는 상수 1.4826."""
    med = np.median(r)
    return float(1.4826 * np.median(np.abs(r - med)))


def _fit_corner(C: np.ndarray, D: np.ndarray, conf: np.ndarray,
                weight: str, robust: str):
    """코너 하나 → (점, 인라이어 마스크). 실패 시 (None, None)."""
    if len(C) < 2:
        return None, None
    w0 = conf.copy() if weight == "conf" else np.ones(len(C))
    p = _solve(C, D, w0)
    if p is None:
        return None, None
    if robust != "huber":
        return p, np.ones(len(C), bool)

    w = w0
    for _ in range(HUBER_ITERS):
        r = _residuals(p, C, D)
        delta = max(HUBER_DELTA_MIN_M, 2.0 * _robust_scale(r))
        hw = np.where(r <= delta, 1.0, delta / np.maximum(r, 1e-9))
        w = w0 * hw
        if w.sum() <= 1e-9:
            break
        p_new = _solve(C, D, w)
        if p_new is None:
            break
        if np.linalg.norm(p_new - p) < 1e-6:
            p = p_new
            break
        p = p_new
    r = _residuals(p, C, D)
    delta = max(HUBER_DELTA_MIN_M, 2.0 * _robust_scale(r))
    return p, r <= delta


def _parallax_deg(D: np.ndarray) -> float:
    """방향 집합의 최대 벌어짐 각 — 원본과 같은 정의 (최소 내적의 arccos)."""
    if len(D) < 2:
        return 0.0
    return float(np.degrees(np.arccos(np.clip((D @ D.T).min(), -1.0, 1.0))))


def reconstruct(samples: Sequence[dict],
                det_conf_min: float = DET_CONF_MIN,
                min_parallax_deg: float = MIN_PARALLAX_DEG,
                weight: str = "conf",
                robust: str = "huber",
                size: str = "mean",
                intr: Dict | None = None,
                T_IC: np.ndarray | None = None) -> List[dict]:
    """taemin_bridge.observe 샘플 → 창문 3D. run_offline 과 반환 형식 동일.

    weight  "conf" (기본) | "none"    — 원본은 "none"
    robust  "huber" (기본) | "none"   — 원본은 "none"
    size    "mean" (기본, 마주보는 변 평균) | "taemin" (한 변씩)

    원본과 완전히 같은 수치를 내려면 weight="none", robust="none", size="taemin".
    """
    from module import contract
    if intr is None:
        intr = contract.intrinsics()
    if T_IC is None:
        T_IC = contract.T_imu_cam()
    R_IC, p_IC = T_IC[:3, :3], T_IC[:3, 3]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]

    # (order_index, corner) -> [중심들, 시선들, conf들]
    acc: Dict[tuple, List[list]] = {}
    colors: Dict[int, str] = {}

    for s in samples:
        det = s.get("detection")
        if not det:
            continue
        R_WI = _rot(s["q_WI_xyzw"])
        p_WI = np.asarray(s["p_WI"], float)
        R_WC = R_WI @ R_IC
        c_W = p_WI + R_WI @ p_IC
        for win in det["windows"]:
            if win["det_conf"] < det_conf_min:
                continue
            oi = win["order_index"]
            colors[oi] = win.get("color", "?")
            for ci in range(4):
                if win["corner_vis"][ci] != 1:
                    continue
                u, v = win["corners"][ci]
                d_C = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
                d_C /= np.linalg.norm(d_C)
                a = acc.setdefault((oi, ci), [[], [], []])
                a[0].append(c_W)
                a[1].append(R_WC @ d_C)
                a[2].append(float(win["det_conf"]))

    results = []
    for oi in sorted({k[0] for k in acc}):
        pts, min_ang, n_obs, n_in = [], float("inf"), 0, 0
        resid = []
        for ci in range(4):
            a = acc.get((oi, ci))
            if a is None:
                pts = None
                break
            C = np.asarray(a[0], float)
            D = np.asarray(a[1], float)
            conf = np.asarray(a[2], float)
            p, inl = _fit_corner(C, D, conf, weight, robust)
            if p is None:
                pts = None
                break
            pts.append(p)
            n_obs += len(C)
            n_in += int(inl.sum())
            rr = _residuals(p, C, D)
            resid.append(float(rr[inl].mean() if inl.any() else rr.mean()))
            # 시차각은 **전체 광선**으로 잰다. 인라이어로만 재면 자기 발등을 찍는다:
            # 같은 원호를 반복해 돌면 잔차 MAD 가 줄고 -> delta 가 좁아지고 ->
            # 잔차가 큰 광선이 먼저 잘리는데, 그게 바로 각도가 제일 벌어진 광선이다
            # (검출 노이즈가 각도에 비례해 증폭되므로). 결국 관측을 늘릴수록 시차각이
            # 떨어져 2° 게이트에서 탈락한다. 실측: 1라운드 성공 -> 이후 전부 복원 실패.
            # 인라이어는 '위치 추정에 쓸 광선'을 고르는 것이지 '관측이 기하학적으로
            # 충분한가'를 재는 지표가 아니다.
            min_ang = min(min_ang, _parallax_deg(D))
        if pts is None or min_ang < min_parallax_deg:
            continue

        P = np.asarray(pts)
        tl, tr, br, bl = P
        if size == "taemin":
            width = float(np.linalg.norm(tr - tl))
            height = float(np.linalg.norm(br - tr))
        else:
            width = float((np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2)
            height = float((np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2)

        results.append({
            "order_index": oi, "color": colors.get(oi),
            "center_w": [round(float(x), 3) for x in P.mean(0)],
            "corners_w": [[round(float(x), 3) for x in q] for q in P],
            "width": round(width, 3), "height": round(height, 3),
            "n_obs": n_obs, "min_parallax_deg": round(min_ang, 1),
            # --- 원본에 없는 진단 필드 (하류는 무시해도 됨) ---
            "n_rejected": n_obs - n_in,
            # 광선들이 한 점에서 얼마나 안 만나는지 [mm] — GT 없이 쓰는 품질 지표
            "resid_mm": round(float(np.mean(resid)) * 1000, 2) if resid else None,
            "inlier_frac": round(n_in / n_obs, 3) if n_obs else 0.0,
            "method": f"weight={weight},robust={robust},size={size}"})
    return results


def reconstruct_like_taemin(samples: Sequence[dict], **kw) -> List[dict]:
    """원본과 동일 설정. 이 파일의 구현이 그의 코드와 같은 답을 내는지 검증용."""
    kw.setdefault("weight", "none")
    kw.setdefault("robust", "none")
    kw.setdefault("size", "taemin")
    return reconstruct(samples, **kw)
