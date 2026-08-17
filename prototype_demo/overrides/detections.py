"""§5 검출 스트림 후처리 — **우리 쪽에서만** 손대는 층.

팀 파일은 한 줄도 안 고친다. 태민 노드도 길남 복원기도 §5.1 메시지를 받아가는데,
그 메시지를 넘기기 *전에* 우리가 한 번 거르는 것뿐이다. 그래서 이 층은 두 소비자
모두에게 동시에 먹힌다.

왜 필요한가 (복원 오차 원인 분석에서 나온 것)
----------------------------------------------
태민 노드는 관측을 **전부 동등 가중 1표**로 누적한다 (window_recon_node.py:46-49,
``A += I − ddᵀ`` 에 계수가 없다). conf 는 0.7 문턱 통과/탈락에만 쓰이고 그 뒤로는
버려진다. 그래서:

  * 한 프레임에서 같은 창문에 박스가 3개 잡히면 그 프레임이 **3표**를 행사한다.
    (v2_02_detect.png frame 35 에서 blue 창문 하나에 blue 박스 3개 확인)
  * conf 0.71 짜리와 0.99 짜리가 같은 무게다.

여기서 표를 정리해서 넘긴다. 복원 알고리즘 자체를 고치는 건 recon_rays.py.
"""
from __future__ import annotations

import copy
from typing import Dict, List, Sequence

import numpy as np

_COLORS = ("red", "green", "blue")


def top1_per_colour(msg: Dict) -> Dict:
    """한 프레임 안에서 색마다 conf 최고 1개만 남긴다.

    창문 하나당 세상에 하나뿐인 색이라는 씬 가정(§4 통과 순서 = 색)에 기댄 필터다.
    같은 색 창문이 둘 이상인 씬에서는 쓰면 안 된다.
    """
    best: Dict[str, Dict] = {}
    for w in msg["windows"]:
        c = w.get("color")
        if c not in best or w["det_conf"] > best[c]["det_conf"]:
            best[c] = w
    out = dict(msg)
    out["windows"] = [best[c] for c in _COLORS if c in best]
    return out


def min_conf(msg: Dict, thr: float) -> Dict:
    """det_conf 문턱. 태민 노드도 0.7 로 거르지만, 그 전에 우리가 더 세게 걸고 싶을 때."""
    out = dict(msg)
    out["windows"] = [w for w in msg["windows"] if w["det_conf"] >= thr]
    return out


def drop_tiny(msg: Dict, min_px: float = 24.0) -> Dict:
    """코너 사각형이 너무 작은 검출 제거 — 먼 창문의 몇 픽셀짜리 박스는 시선 각도
    오차가 커서 삼각측량에 독이 된다."""
    keep = []
    for w in msg["windows"]:
        xs = [p[0] for p in w["corners"]]
        ys = [p[1] for p in w["corners"]]
        if (max(xs) - min(xs)) >= min_px and (max(ys) - min(ys)) >= min_px:
            keep.append(w)
    out = dict(msg)
    out["windows"] = keep
    return out


def clean(msg: Dict, dedupe: bool = True, conf_min: float | None = None,
          min_px: float | None = 24.0) -> Dict:
    """위 필터들을 순서대로. 순서 중요: 작은 것/저conf 먼저 버리고 나서 top-1."""
    if conf_min is not None:
        msg = min_conf(msg, conf_min)
    if min_px:
        msg = drop_tiny(msg, min_px)
    if dedupe:
        msg = top1_per_colour(msg)
    return msg


def clean_samples(samples: Sequence[dict], **kw) -> List[dict]:
    """taemin_bridge.observe 가 낸 샘플 리스트 전체에 적용 (원본 불변, 사본 반환)."""
    out = []
    for s in samples:
        s2 = copy.copy(s)
        det = s.get("detection")
        if det is not None:
            m = clean(det, **kw)
            s2["detection"] = m if m["windows"] else None
        out.append(s2)
    return out


def clean_records(records: Sequence[dict], **kw) -> List[dict]:
    """길남 경로용 — pybullet_stream.capture 가 낸 records 형식에 적용."""
    out = []
    for r in records:
        r2 = copy.copy(r)
        r2["vision"] = clean(r["vision"], **kw)
        out.append(r2)
    return out


def assign_order_by_passing(samples: Sequence[dict], layout: Sequence[dict],
                            behind_m: float = 0.35) -> List[dict]:
    """창문이 4개 이상일 때 ``order_index`` 를 **통과 순서**로 다시 매긴다.

    왜 필요한가
    -----------
    §5 메시지의 ``order_index`` 는 색에서 나온다 (red=0, green=1, blue=2). 창문이
    3개까지는 색이 곧 고유 식별자라 문제가 없지만, 10개짜리 씬은 색이 r,g,b 로
    순환해서 **10개가 3바구니로 뭉갠다.** 태민 노드도 길남 복원기도 order_index 로
    관측을 묶으므로, 그대로 두면 서로 다른 창문 4개의 광선이 한 점에 수렴한다.

    어떻게 푸는가
    -------------
    드론은 창문을 앞에서부터 하나씩 지난다. 어느 시점에 보이는 창문들 중에서는
    색이 겹치지 않는다 (연속 3개가 r,g,b). 그래서 "현재 위치보다 앞에 있는,
    같은 색 창문 중 가장 가까운 것" 으로 붙이면 유일하게 정해진다.

    **한계 (데모용 단순화)**: 이 함수는 GT 창문 x 좌표를 쓴다. 실제 시스템은 GT 가
    없으므로 복원 중인 지도와 프레임 간 트래킹으로 연결해야 한다. 여기서는 색
    중복이 복원을 망가뜨리는 것만 막는 게 목적이다.
    """
    xs = [(w["order_index"], float(np.asarray(w["center"], float)[0]), w["color"])
          for w in layout]
    out = []
    for s in samples:
        s2 = copy.copy(s)
        det = s.get("detection")
        if det is None:
            out.append(s2)
            continue
        dx = float(s["p_WI"][0])
        msg = dict(det)
        wins = []
        for w in det["windows"]:
            cand = [(x, oi) for oi, x, col in xs
                    if col == w.get("color") and x > dx - behind_m]
            if not cand:
                continue                       # 이미 다 지나친 색 — 버린다
            w2 = dict(w)
            w2["order_index"] = min(cand)[1]
            wins.append(w2)
        msg["windows"] = wins
        s2["detection"] = msg if wins else None
        out.append(s2)
    return out


def count(samples: Sequence[dict]) -> Dict[str, int]:
    """필터 전후 비교용 집계."""
    n_det = n_frames = 0
    dup = 0
    for s in samples:
        det = s.get("detection")
        if not det:
            continue
        n_frames += 1
        ws = det["windows"]
        n_det += len(ws)
        seen: Dict[str, int] = {}
        for w in ws:
            seen[w["color"]] = seen.get(w["color"], 0) + 1
        dup += sum(v - 1 for v in seen.values() if v > 1)
    return {"frames_with_det": n_frames, "detections": n_det, "duplicate_votes": dup}
