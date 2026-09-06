"""성능 시나리오 세트 지표 (2026-09-06).

`diagnose/run_scenarios.sh` 가 남긴 시계열을 **짝지어** 읽는다.

왜 짝인가 — 지금까지 쓰던 복귀 지표는 `abs(y) < 2 cm 가 끝까지 유지`였고, 여기에
두 가지 결함이 있다:

1. **y 축만 본다.** pitch 외란은 진행방향(x)으로 미는데 y 는 거의 안 움직이므로
   "복귀 0 초"로 찍힌다. 실제로는 35 cm 밀렸는데도. 축을 바꾸면 지표가 죽는다.
2. **2 cm 절대 밴드가 질량을 안 본다.** 0 kg 은 같은 토크에 더 크게 밀리므로
   밴드에 드는 데 더 걸린다. 1 kg 대비 5 배 차이의 상당 부분이 제어 성능이 아니라
   자의 문제였다.

그래서 여기서는 **같은 조건의 무외란 런을 기준선으로 빼고** 잰다:

    d(t) = || p_외란(t) - p_무외란(t) ||          (3축, cm)

이러면 (a) 축과 무관하고, (b) 이동 추종오차가 양쪽에서 상쇄되어 외란 효과만 남고,
(c) 절대 밴드와 상대 밴드(peak 의 10 %)를 함께 낼 수 있다. 상대 밴드가 질량 간
비교용이고, 절대 밴드는 상위(경로계획)가 회피 여유를 잡을 때 쓰는 값이다.

주의: 두 런의 t 격자는 같다(같은 tsDt 로 리샘플). 다르면 보간한다.
"""
from __future__ import annotations

import csv
import math
import os
import sys

# 콘솔이 cp949 라 한글/기호가 깨진다 (MATLAB_USAGE.md 도 같은 함정을 적어 뒀다).
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "controller", "Quadcopter-Drone-Model-Simscape",
                   "diagnose", "results", "worstcase")

# (이름, 짐질량, 기준선 태그+라벨, 외란 태그+라벨, 외란 설명)
PAIRS = [
    ("0 kg / roll",  0.0, "0kg_nd_K", "0kg_K",  "토크 roll 0.3 N*m x 0.3 s"),
    ("1 kg / roll",  1.0, "nd_K",     "K",      "토크 roll 0.3 N*m x 0.3 s"),
    ("2 kg / roll",  2.0, "2kg_nd_K", "2kg_K",  "토크 roll 0.3 N*m x 0.3 s"),
    ("1 kg / pitch", 1.0, "nd_K",     "KP",     "토크 pitch 0.3 N*m x 0.3 s"),
    ("1 kg / yaw",   1.0, "nd_K",     "KY",     "토크 yaw 0.3 N*m x 0.3 s"),
]

ABS_BAND_CM = 2.0      # 기존 지표와 이어 붙이려고 유지 (sweep_delay_spec 과 동일)
REL_BAND = 0.10        # peak 의 10 % — 질량 무관 상대 지표


def load_ts(key):
    path = os.path.join(RES, "ts_%s.csv" % key)
    if not os.path.exists(path):
        return None
    cols = {}
    with open(path, newline="", encoding="utf-8") as f:
        rd = csv.DictReader(f)
        names = rd.fieldnames or []
        for n in names:
            cols[n] = []
        for row in rd:
            for n in names:
                cols[n].append(float(row[n]))
    return cols


def interp(tq, t, v):
    """t 는 단조 증가. 격자가 같으면 그대로 돌려준다."""
    if len(t) == len(tq) and abs(t[0] - tq[0]) < 1e-9 and abs(t[-1] - tq[-1]) < 1e-9:
        return v
    out, j = [], 0
    for x in tq:
        while j + 2 < len(t) and t[j + 1] < x:
            j += 1
        t0, t1 = t[j], t[j + 1]
        w = 0.0 if t1 == t0 else (x - t0) / (t1 - t0)
        out.append(v[j] + w * (v[j + 1] - v[j]))
    return out


def sustained_below(t, d, thr):
    """d 가 thr 아래로 내려가 **끝까지 유지**되는 첫 시각. 없으면 None.

    뒤에서부터 훑는다 — '유지' 조건은 뒤에서 보면 한 번의 통과로 풀린다.
    """
    idx = None
    for i in range(len(d) - 1, -1, -1):
        if d[i] > thr:
            break
        idx = i
    return None if idx is None else t[idx]


def energy_wh(C):
    """시계열의 P_est_W 를 적분한다.

    summary.csv 를 안 쓰는 이유: 그 작성기의 서식 지시자가 21개인데 인자가 24개라
    fx_N 이후 열이 전부 밀려 있었다 (09-06 발견, verify_worstcase.m 에서 고침).
    고치기 전에 나온 CSV 는 여전히 깨져 있으므로 여기서는 시계열을 직접 적분한다.
    """
    t, P = C["t"], C["P_est_W"]
    acc = 0.0
    for i in range(len(t) - 1):
        acc += 0.5 * (P[i] + P[i + 1]) * (t[i + 1] - t[i])
    return acc / 3600.0


def noise_floor(t, d):
    """정착 뒤 두 런의 상시 차이. 상대 밴드가 이 아래로 내려가면 무의미하다.

    같은 게인·같은 궤적이어도 두 런은 한계사이클 위상이 달라 영원히 정확히 겹치지
    않는다. 그래서 'peak 의 10 %' 를 그대로 쓰면 편차가 작은 케이스(1 kg)에서
    밴드가 그 상시 차이 밑으로 내려가 '복귀 못 함'으로 잘못 찍힌다.
    """
    tail = [d[i] for i in range(len(t)) if t[i] > t[-1] - 3.0]
    if not tail:
        return 0.0
    tail.sort()
    return 2.0 * tail[len(tail) // 2]


def summary_rows():
    """summary.csv 들에서 추종/기존 지표를 끌어온다 (에너지는 위 함수를 쓴다)."""
    out = {}
    for fn in os.listdir(RES):
        if not fn.endswith("summary.csv"):
            continue
        tag = fn[:-len("summary.csv")]
        with open(os.path.join(RES, fn), newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if row.get("label"):
                    out[tag + row["label"]] = row
    return out


def main():
    if not os.path.isdir(RES):
        sys.exit("결과 폴더가 없다: %s" % RES)
    S = summary_rows()

    print()
    print("=== 성능 시나리오 세트 - 외란 효과 (무외란 짝 대비) ===")
    print("d(t) = 외란 런과 무외란 런의 3축 위치 차이. 축·질량과 무관하게 비교된다.")
    print()
    hdr = ("%-14s %9s %7s %10s %8s %10s %8s %9s %8s" %
           ("시나리오", "최대편차", "발생", "복귀(2cm)", "밴드", "복귀(밴드)",
            "잔류", "자세peak", "에너지"))
    print(hdr)
    print("%-14s %9s %7s %10s %8s %10s %8s %9s %8s" %
          ("", "[cm]", "[s]", "[s]", "[cm]", "[s]", "[cm]", "[deg]", "[Wh]"))
    print("-" * len(hdr))

    missing = []
    for name, mass, base_key, dist_key, desc in PAIRS:
        B, D = load_ts(base_key), load_ts(dist_key)
        if B is None or D is None:
            missing.append((name, base_key if B is None else dist_key))
            continue
        t = D["t"]
        bx = interp(t, B["t"], B["x"])
        by = interp(t, B["t"], B["y"])
        bz = interp(t, B["t"], B["z"])
        br = interp(t, B["t"], B["roll"])
        bp = interp(t, B["t"], B["pitch"])
        bw = interp(t, B["t"], B["yaw"])

        d = [100.0 * math.sqrt((D["x"][i] - bx[i]) ** 2 +
                               (D["y"][i] - by[i]) ** 2 +
                               (D["z"][i] - bz[i]) ** 2) for i in range(len(t))]
        att = [max(abs(D["roll"][i] - br[i]), abs(D["pitch"][i] - bp[i]),
                   abs(D["yaw"][i] - bw[i])) for i in range(len(t))]

        ipk = max(range(len(d)), key=lambda i: d[i])
        dmax, tpk = d[ipk], t[ipk]
        # 외란 이후 구간에서만 복귀를 본다 (그 전은 두 런이 같으므로 0 이다)
        i0 = ipk
        band = max(REL_BAND * dmax, noise_floor(t, d))
        t_abs = sustained_below(t[i0:], d[i0:], ABS_BAND_CM)
        t_rel = sustained_below(t[i0:], d[i0:], band)
        # 복귀 시각은 외란 끝(= 피크 직전 상승 시작)이 아니라 **피크**를 기준으로 잰다.
        # 피크는 로그에서 바로 읽히는 점이라 정의가 흔들리지 않는다.
        rec_abs = None if t_abs is None else t_abs - tpk
        rec_rel = None if t_rel is None else t_rel - tpk
        resid = d[-1]
        attpk = math.degrees(max(att))   # 로그는 rad 다
        wh = energy_wh(D)

        def f(v, fmt="%.2f"):
            return "-" if v is None else fmt % v

        # 짝 비교가 성립하려면 두 런이 외란 밖에서는 겹쳐야 한다. 상시 차이(밴드)가
        # 최대편차에 육박하면 겹치지 않은 것이고, 그때 이 행의 '복귀'는 외란이 아니라
        # 기준선 자체의 발산을 재고 있다. 숫자를 내되 무효라고 못박는다.
        # 편차가 2 cm 밴드 안에서 놀면 '복귀'라는 말이 성립하지 않는다 (yaw 가 그렇다 -
        # 위치는 거의 안 밀고 기수만 돌아간다. 그 축의 진짜 지표는 자세peak 열이다).
        if dmax < ABS_BAND_CM:
            note = "   <-- 위치 영향 미미 (자세peak 를 볼 것)"
        elif band > 0.5 * dmax:
            note = "   <-- 짝 비교 무효 (기준선이 안 겹침)"
        else:
            note = ""
        print("%-14s %9.2f %7.2f %10s %8.2f %10s %8.2f %9.2f %8.4f%s" %
              (name, dmax, tpk, f(rec_abs), band, f(rec_rel), resid, attpk, wh, note))

    print()
    if missing:
        print("빠진 런:")
        for n, k in missing:
            print("  %-14s  ts_%s.csv 없음" % (n, k))
        print()

    print("=== 무외란 기준선 자체의 건전성 (외란 없이 3 m 직선 이동만) ===")
    print("짝 비교가 무효인 행이 있으면 원인이 여기 있다. 외란 응답을 논하기 전에")
    print("기준선이 임무를 제대로 수행하는지부터 본다.")
    print()
    print("%-10s %10s %10s %10s %10s %10s" %
          ("기준선", "yaw진폭", "roll최대", "pitch최대", "고도최저", "종단오차"))
    print("%-10s %10s %10s %10s %10s %10s" %
          ("", "[deg]", "[deg]", "[deg]", "[m]", "[cm]"))
    seen = set()
    for name, mass, base_key, dist_key, desc in PAIRS:
        if base_key in seen:
            continue
        seen.add(base_key)
        B = load_ts(base_key)
        if B is None:
            continue
        yaw_pp = math.degrees(max(B["yaw"]) - min(B["yaw"]))
        rmax = math.degrees(max(abs(v) for v in B["roll"]))
        pmax = math.degrees(max(abs(v) for v in B["pitch"]))
        zmin = min(B["z"])
        endcm = 100.0 * abs(B["x"][-1] - B["xref"][-1])
        print("%-10s %10.2f %10.2f %10.2f %10.3f %10.2f" %
              ("%g kg" % mass, yaw_pp, rmax, pmax, zmin, endcm))
    print()

    # 기존 y-only 지표와 나란히 (지표가 얼마나 갈리는지 보여주는 것이 요점)
    print("=== 참고: 기존 y-only 2 cm 지표 (같은 런) ===")
    print("%-14s %10s %10s %10s" % ("시나리오", "dev_y[cm]", "복귀[s]", "추종[cm]"))
    for name, mass, base_key, dist_key, desc in PAIRS:
        row = S.get(dist_key)
        if not row:
            continue
        print("%-14s %10s %10s %10s" %
              (name, row.get("dev_y_cm", "-"),
               row.get("recover_s") or "미복귀", row.get("track_cm", "-")))
    print()


if __name__ == "__main__":
    main()
