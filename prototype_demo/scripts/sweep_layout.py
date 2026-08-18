"""모터 인덱스 → 위치 배정을 **전수조사** 한다 (24 순열 × 자세게인 부호 × 크기).

    python scripts/sweep_layout.py --workers 4

왜 이걸 보나
------------
자세 게인 크기를 800배 범위로 훑고 부호도 뒤집어 봤지만 mode=move 가 전부 발산했다.
그런데 고도는 z최저 0.92 m 로 끝까지 버텼다 — 고도·위치 루프는 멀쩡하고 **자세만**
뒤집힌다는 뜻이다. 게인 크기 문제가 아니라 자세 피드백 경로의 **구조**가 어긋난
것이고, 그 구조에서 우리가 임의로 정한 건 딱 하나, ``qc.py`` 의

    motor_xy = [[+s,-s], [-s,-s], [-s,+s], [+s,+s]]

이 배정이다. 그의 믹서표(mixPitch +--+ / mixRoll --++)를 보고 추측한 값이고 근거가
없다. 부호 4조합은 이미 봤으니 이제 **순열 24가지**를 전부 본다.

각 워커는 독립 프로세스다 — ctypes 세션이 전역이라 한 프로세스 안에서는 순차 실행.
"""
from __future__ import annotations

import argparse
import itertools
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils import paths  # noqa: E402

paths.bootstrap()

S = 0.15909903
CORNERS = np.array([[+S, -S], [-S, -S], [-S, +S], [+S, +S]])
ALT = dict(kpAlt=60.0, kdAlt=24.0, kiAlt=0.5, limAlt=30.0)
BASE_ATT = (-85.0, -10.0, -127.5)


def one(job):
    """(perm, sign, scale, limAtt) → 결과 dict. 워커에서 실행."""
    perm, sg, sc, lim, mode, seconds = job
    from tune_gains import fly
    layout = CORNERS[list(perm)]
    g = dict(ALT)
    g.update(kpAtt=BASE_ATT[0] * sc * sg, kiAtt=BASE_ATT[1] * sc * sg,
             kdAtt=BASE_ATT[2] * sc * sg, limAtt=lim)
    r = fly(g, mode, seconds=seconds, layout=layout)
    r.update(perm=list(perm), sign=sg, scale=sc, limAtt=lim)
    return r


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--mode", default="move")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)

    jobs = [(perm, sg, sc, lim, a.mode, a.seconds)
            for perm in itertools.permutations(range(4))
            for sg in (+1, -1)
            for sc, lim in ((0.3, 20.0), (0.05, 20.0))]
    print(f"전수조사: 순열 24 × 부호 2 × 게인 2 = {len(jobs)} 회  "
          f"(mode={a.mode}, {a.seconds}s, workers={a.workers})\n")

    with Pool(a.workers) as pool:
        res = []
        for i, r in enumerate(pool.imap_unordered(one, jobs), 1):
            res.append(r)
            if i % 12 == 0:
                alive = sum(1 for x in res if not x["diverged"])
                print(f"  {i}/{len(jobs)}  생존 {alive}")

    ok = sorted((r for r in res if not r["diverged"]), key=lambda r: r["err_rms"])
    print(f"\n생존 {len(ok)}/{len(res)}")
    if ok:
        print(f"\n{'perm':>14}{'부호':>5}{'scale':>7}{'limAtt':>8}"
              f"{'RMS':>11}{'최종':>11}{'기울기':>9}")
        for r in ok[:12]:
            print(f"{str(r['perm']):>14}{r['sign']:>+5d}{r['scale']:>7}"
                  f"{r['limAtt']:>8.0f}{r['err_rms']*1000:>9.1f}mm"
                  f"{r['err_final']*1000:>9.1f}mm{r['tilt_max']:>8.1f}deg")
    else:
        # 살아남은 게 없으면 '덜 죽은' 순으로 — 방향은 알려준다
        near = sorted(res, key=lambda r: (r["tilt_max"], r["err_final"]))
        print("\n생존 0. 기울기가 가장 작았던 조합 (방향 참고용):")
        print(f"{'perm':>14}{'부호':>5}{'scale':>7}{'최종':>11}{'기울기':>9}")
        for r in near[:12]:
            print(f"{str(r['perm']):>14}{r['sign']:>+5d}{r['scale']:>7}"
                  f"{r['err_final']*1000:>10.1f}mm{r['tilt_max']:>8.1f}°")

    if a.out:
        with open(a.out, "w") as f:
            json.dump(res, f, indent=1)
        print(f"\nwrote {a.out}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
