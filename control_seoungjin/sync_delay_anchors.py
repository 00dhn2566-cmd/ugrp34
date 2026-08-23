"""MATLAB 지연 스윕 결과 -> `capability._LAT_POS_ANCHORS` + C++ `SpecLatencyRule` 동기.

2026-08-23. 같은 실측표를 파이썬과 C++ 두 곳에 적어 두는 이상, 한쪽만 갱신되는 사고가
반드시 난다. 손으로 옮기지 말고 이걸 돌린다. (`tests/test_cpp_spec_parity.py` 가
앵커 **개수** 불일치를 잡지만, 값까지 지켜 주려면 애초에 한 곳에서 생성해야 한다.)

입력: controller/.../diagnose/results/sweep_delay_spec_progress.txt
      (MATLAB `sweep_delay_spec.m` 이 한 줄씩 흘려 쓴 파일)

사용:
    python sync_delay_anchors.py            # 무엇이 바뀌는지 보여주기만
    python sync_delay_anchors.py --write    # 실제로 두 파일을 고침
"""
from __future__ import annotations

import argparse
import io
import os
import re

ROOT = os.path.dirname(os.path.abspath(__file__))
PROG = os.path.join(ROOT, "controller", "Quadcopter-Drone-Model-Simscape",
                    "diagnose", "results", "sweep_delay_spec_progress.txt")
PY = os.path.join(ROOT, "capability.py")
HPP = os.path.join(ROOT, "controller_cpp", "include", "qc_controller.hpp")

ROW = re.compile(r"\s*(\d+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+"
                 r"([\d.]+)\s+(\S+)\s+(OK|FAIL)")


def read_anchors(path=PROG):
    """진행 파일 -> {지연[s]: 채택 배율}. 통과가 없는 지연은 0.0 (운용 불가)."""
    if not os.path.exists(path):
        raise SystemExit(f"스윕 결과 없음: {path}\n먼저 sweep_delay_spec.m 을 돌릴 것")
    best, seen = {}, []
    with io.open(path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            m = ROW.match(ln)
            if not m:
                continue
            tau_ms, s, ok = int(m.group(1)), float(m.group(2)), m.group(8) == "OK"
            if tau_ms not in seen:
                seen.append(tau_ms)
            if ok:
                best[tau_ms] = max(best.get(tau_ms, 0.0), s)
    # 시도했지만 하나도 통과 못한 지연은 0.0 = 운용 불가. 빠뜨리면 보간이 그 구멍을
    # 낙관적으로 이어 버린다 (안 재본 게 아니라 '못 하는' 것이므로 반드시 남긴다).
    return {ms: best.get(ms, 0.0) for ms in sorted(seen)}


def patch_python(anchors, write):
    src = io.open(PY, encoding="utf-8").read()
    body = "\n".join(f"    {ms/1000:.3f}: {s:.2f}," for ms, s in anchors.items())
    new_block = "_LAT_POS_ANCHORS = {\n" + body + "\n}"
    pat = re.compile(r"_LAT_POS_ANCHORS = \{.*?\n\}", re.S)
    if not pat.search(src):
        raise SystemExit("capability.py 에서 _LAT_POS_ANCHORS 블록을 못 찾음")
    out = pat.sub(new_block, src)
    out = out.replace("#   ⚠ 아래 표는 스윕이 끝나는 대로 갱신한다. [TODO-B상]\n",
                      "#   판정: 종단오차 <= 5 cm AND 외란 복귀 <= 3 s. 0.00 = 그 지연에서는\n"
                      "#   어떤 배율로도 통과 못함(운용 불가). sync_delay_anchors.py 가 생성.\n")
    changed = out != src
    if write and changed:
        io.open(PY, "w", encoding="utf-8", newline="\n").write(out)
    return changed, new_block


def patch_cpp(anchors, write):
    src = io.open(HPP, encoding="utf-8").read()
    taus = list(anchors.keys())
    n = len(taus)
    cap_n = 8
    if n > cap_n:
        raise SystemExit(f"앵커 {n}개 > C++ kMaxAnchors {cap_n} — 헤더 상한을 먼저 늘릴 것")
    tv = [f"{ms/1000:.3f}" for ms in taus] + ["0.0"] * (cap_n - n)
    sv = [f"{anchors[ms]:.2f}" for ms in taus] + ["0.0"] * (cap_n - n)
    new_t = "    double posTau[kMaxAnchors]   = {" + ", ".join(tv) + "};"
    new_s = "    double posScale[kMaxAnchors] = {" + ", ".join(sv) + "};"
    out = re.sub(r"    double posTau\[kMaxAnchors\]\s*=\s*\{[^}]*\};", new_t, src)
    out = re.sub(r"    double posScale\[kMaxAnchors\]\s*=\s*\{[^}]*\};", new_s, out)
    out = re.sub(r"    int    posN = \d+;[^\n]*\n",
                 f"    int    posN = {n};             "
                 "// sync_delay_anchors.py 가 생성 — 손으로 고치지 말 것\n", out)
    changed = out != src
    if write and changed:
        io.open(HPP, "w", encoding="utf-8", newline="\n").write(out)
    return changed, new_t + "\n" + new_s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="실제로 파일을 고침")
    ap.add_argument("--progress", default=PROG)
    a = ap.parse_args()

    anchors = read_anchors(a.progress)
    print("실측 앵커 (지연[ms] -> 허용 배율):")
    for ms, s in anchors.items():
        note = "  <- 운용 불가" if s == 0.0 else ""
        print(f"  {ms:>4} ms : {s:.2f}   v {1.6*s:.3f} m/s{note}")

    cpy, blk_py = patch_python(anchors, a.write)
    ccc, blk_c = patch_cpp(anchors, a.write)
    print(f"\ncapability.py           : {'갱신' if cpy else '변화 없음'}")
    print(f"controller_cpp/...hpp   : {'갱신' if ccc else '변화 없음'}")
    if not a.write and (cpy or ccc):
        print("\n(--write 를 붙여야 실제로 고쳐진다)")
        print(blk_py)
        print(blk_c)
    if a.write and ccc:
        print("\n※ C++ 를 고쳤으니 다시 빌드할 것: controller_cpp/build_spec.ps1")


if __name__ == "__main__":
    main()
