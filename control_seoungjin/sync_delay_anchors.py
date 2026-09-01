"""MATLAB 지연 스윕 결과 -> 파이썬 `capability` + C++ `SpecLatencyRule` 실측표 동기.

2026-08-23. 같은 실측표를 두 언어에 적어 두는 이상, 한쪽만 갱신되는 사고가 반드시
난다. 손으로 옮기지 말고 이걸 돌린다. (`tests/test_cpp_spec_parity.py` 가 앵커 개수
불일치를 잡지만, 값까지 지키려면 애초에 한 곳에서 생성해야 한다.)

표는 **두 벌**이다 (사용자 정정: "디폴트는 외란 없다"):
  기본 (`--gust` 없음) : 외란 없는 조건. 상위에 늘 내보내는 값.
                         MATLAB `SPEC_PULSE_NM=0` 으로 잰 진행 파일.
  돌풍 (`--gust`)      : 이동 중 0.3 N*m 펄스를 맞고도 복귀가 사는 배율.
                         외란이 실제로 감지될 때만, rho 크기로 기본표와 보간해 쓴다.

사용:
    python sync_delay_anchors.py --progress <파일>            # 미리보기
    python sync_delay_anchors.py --progress <파일> --write     # 기본표 갱신
    python sync_delay_anchors.py --progress <파일> --gust --write
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

CAP_N = 8          # C++ SpecLatencyRule::kMaxAnchors


def read_anchors(path=PROG):
    """진행 파일 -> {지연[ms]: 채택 배율}. 통과가 없는 지연은 0.0 (운용 불가).

    0.0 을 빼먹으면 보간이 그 구멍을 낙관적으로 이어 버린다 — '안 재봤다'가 아니라
    '어떤 배율로도 못 한다'는 뜻이므로 반드시 표에 남긴다.
    """
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
    if not seen:
        raise SystemExit(f"진행 파일에서 결과 행을 못 찾음: {path}")
    return {ms: best.get(ms, 0.0) for ms in sorted(seen)}


def read_mass(path=PROG):
    """진행 파일 헤더에서 짐 질량과 그 질량의 속도 앵커를 읽는다.

    표가 어느 질량의 것인지는 파일 안에만 있다. 이걸 안 보면 0 kg 표를 1 kg 자리에
    쓰는 사고가 난다 (배율은 무차원이라 숫자만 봐서는 구분이 안 된다).
    """
    m = None
    with io.open(path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            g = re.search(r"==== 시작: 짐 ([\d.]+) kg", ln)
            if g:
                m = float(g.group(1))       # 여러 실행이 이어 붙었으면 마지막 것
    if m is None:
        raise SystemExit(f"진행 파일에서 '==== 시작: 짐 N kg' 헤더를 못 찾음: {path}")
    return m, (1.2 if m == 0 else 1.6)      # capability._ANCHORS 의 v 앵커


_TARGETS = {
    (1.0, False): ("_LAT_POS_ANCHORS",      ("posTau",  "posScale",  "posN")),
    (1.0, True):  ("_LAT_POS_ANCHORS_GUST", ("gustTau", "gustScale", "gustN")),
    (0.0, False): ("_LAT_POS_ANCHORS_0KG",  ("posTau0", "posScale0", "posN0")),
}


def target_for(pkg, gust):
    """질량 x 조건 -> 갱신할 파이썬/C++ 슬롯.

    0 kg 돌풍 자리는 일부러 비워 뒀다 — 1 kg 과 **다른 복귀 게이트**로 재기 때문에
    (그 질량의 tau=0 복귀의 2배) 같은 표에 넣으면 서로 다른 기준이 섞인다.
    중간 질량은 실측이 아니라 보간이 담당한다 (capability._lat_table_for_pkg).
    """
    key = (float(pkg), bool(gust))
    if key not in _TARGETS:
        raise SystemExit(
            "거부: 짐 %g kg / %s 을 담을 자리가 없다. "
            "갱신 가능: 1 kg 무외란/돌풍, 0 kg 무외란."
            % (pkg, "돌풍" if gust else "무외란"))
    return _TARGETS[key]


def patch_python(anchors, write, gust=False, pkg=1.0):
    name, _ = target_for(pkg, gust)
    src = io.open(PY, encoding="utf-8").read()
    body = "\n".join(f"    {ms / 1000:.3f}: {s:.2f}," for ms, s in anchors.items())
    new_block = name + " = {\n" + body + "\n}"
    pat = re.compile(re.escape(name) + r" = \{.*?\n\}", re.S)
    if not pat.search(src):
        raise SystemExit(f"capability.py 에서 {name} 블록을 못 찾음")
    out = pat.sub(lambda _m: new_block, src, count=1)
    # 무외란 표가 채워지면 자리표시 주석을 지운다
    out = out.replace("}   # [TODO] 무외란 스윕 결과로 교체 중 (SPEC_PULSE_NM=0)", "}")
    changed = out != src
    if write and changed:
        io.open(PY, "w", encoding="utf-8", newline="\n").write(out)
    return changed, new_block


def patch_cpp(anchors, write, gust=False, pkg=1.0):
    _, (tname, sname, nname) = target_for(pkg, gust)
    src = io.open(HPP, encoding="utf-8").read()
    taus = list(anchors.keys())
    n = len(taus)
    if n > CAP_N:
        raise SystemExit(f"앵커 {n}개 > C++ kMaxAnchors {CAP_N} — 헤더 상한을 먼저 늘릴 것")
    tv = [f"{ms / 1000:.3f}" for ms in taus] + ["0.0"] * (CAP_N - n)
    sv = [f"{anchors[ms]:.2f}" for ms in taus] + ["0.0"] * (CAP_N - n)
    # 열 맞춤 (헤더 가독성) — 가장 긴 이름 gustScale 기준
    pad = lambda nm: " " * (len("gustScale") - len(nm))  # noqa: E731
    new_t = f"    double {tname}[kMaxAnchors]{pad(tname)}   = {{" + ", ".join(tv) + "};"
    new_s = f"    double {sname}[kMaxAnchors]{pad(sname)}   = {{" + ", ".join(sv) + "};"
    out = re.sub(r"    double " + tname + r"\[kMaxAnchors\][^=]*=\s*\{[^}]*\};",
                 lambda _m: new_t, src, count=1)
    out = re.sub(r"    double " + sname + r"\[kMaxAnchors\][^=]*=\s*\{[^}]*\};",
                 lambda _m: new_s, out, count=1)
    line_n = (f"    int    {nname} = {n};" + " " * max(1, 13 - len(nname))
              + "// sync_delay_anchors.py 가 생성 — 손으로 고치지 말 것\n")
    out = re.sub(r"    int    " + nname + r" = \d+;[^\n]*\n",
                 lambda _m: line_n, out, count=1)
    changed = out != src
    if write and changed:
        io.open(HPP, "w", encoding="utf-8", newline="\n").write(out)
    return changed, new_t + "\n" + new_s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="실제로 파일을 고침")
    ap.add_argument("--gust", action="store_true",
                    help="돌풍 표를 갱신 (기본은 무외란 기본표)")
    ap.add_argument("--progress", default=PROG)
    a = ap.parse_args()

    anchors = read_anchors(a.progress)
    pkg, v_ref = read_mass(a.progress)
    label = "돌풍" if a.gust else "기본(무외란)"
    print(f"실측 앵커 [{label}, 짐 {pkg:g} kg] (지연[ms] -> 허용 배율):")
    for ms, s in anchors.items():
        note = "  <- 운용 불가" if s == 0.0 else ""
        print(f"  {ms:>4} ms : {s:.2f}   v {v_ref * s:.3f} m/s{note}")

    # 질량별로 **다른 자리**에 쓴다. 예전에는 어느 질량이든 1 kg 자리에 썼고,
    # 그러면 0 kg 표(80 ms = 0.75)가 1 kg 표(0.37)를 덮어 상위가 1 kg 임무를
    # 두 배 빠르게 짜도 된다고 읽는다. 담을 자리가 없는 조합은 target_for 가 막는다.
    name, _ = target_for(pkg, a.gust)
    print(f"  -> 갱신 대상: {name}")

    cpy, blk_py = patch_python(anchors, a.write, a.gust, pkg)
    ccc, blk_c = patch_cpp(anchors, a.write, a.gust, pkg)
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
