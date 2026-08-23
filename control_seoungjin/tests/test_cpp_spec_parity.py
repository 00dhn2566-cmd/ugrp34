"""C++ 지연/스펙 경로가 파이썬과 **같은 수**를 내는지 대조.

왜 이게 필요한가 — 제어 본체는 구운 Simulink 모델을 정답 삼아 골든 트레이스로
대조한다. 오늘 들어온 지연 경로는 Simulink 쪽에 대응물이 없고(계획측 판단이라),
대신 파이썬 구현이 정답이다. 두 구현이 조용히 갈라지면 상위가 받는 스펙이
기체마다 달라진다 — 그러면 같은 임무가 어떤 기체에서는 서고 어떤 기체에서는 안 선다.

빌드가 없으면 건너뛴다 (CI/다른 머신에서 g++ 이 없을 수 있음):
    controller_cpp/build_spec.ps1  ->  controller_cpp/qc_spec_trace.exe
"""
import os
import re
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import capability as cap                      # noqa: E402
from latency_tracker import LatencyTracker    # noqa: E402
from spec_governor import att_delay_verdict, scale_from_latency_pos  # noqa: E402

EXE = os.path.join(ROOT, "controller_cpp", "qc_spec_trace.exe")
pytestmark = pytest.mark.skipif(
    not os.path.exists(EXE),
    reason="qc_spec_trace.exe 없음 — controller_cpp/build_spec.ps1 로 빌드")

TOL = 1e-9


def run_cpp(att_s):
    out = subprocess.run([EXE, str(att_s)], capture_output=True, text=True, check=True)
    lines = out.stdout.strip().splitlines()
    head = lines[0].split(",")
    return [dict(zip(head, ln.split(","))) for ln in lines[1:]]


def py_reference(att_s):
    """C++ main_spec_trace.cpp 와 **같은 수열**을 파이썬으로 돌린다."""
    tr = LatencyTracker()
    rows = []
    s_att, _ = att_delay_verdict(att_s)
    for k in range(120):
        sample = 0.075 if 30 <= k < 70 else 0.012
        pred = tr.update(sample)
        pred = tr.predicted_s
        # C++ main_spec_trace 는 sDisturb=1.0 (외란 없음) 으로 부른다 -> rhoEff = 0
        s_pos = scale_from_latency_pos(pred, gust=False)
        s = cap.combine_scales(1.0, s_pos, s_att) if s_att > 0.0 else 0.0
        rows.append(dict(
            k=k, detected=tr.detected, ema_fast=tr.ema_fast, ema_slow=tr.ema_slow,
            predicted=pred, scale=s,
            v=1.6 * 0.75 * s, a=1.6 * 0.75 * s ** 2,
            j=8.0 * 0.75 * s ** 3, snap=64.0 * 0.75 * s ** 4,
            mission=(s_att > 0.0)))
    return rows


@pytest.mark.parametrize("att_s,label", [
    (0.003, "청정"),
    (0.014, "여유 감쇄"),
    (0.020, "운용 불가"),
])
def test_latency_and_spec_parity(att_s, label):
    cpp = run_cpp(att_s)
    py = py_reference(att_s)
    assert len(cpp) == len(py)
    for c, p in zip(cpp, py):
        assert int(c["k"]) == p["k"]
        assert (c["detected"] == "1") == p["detected"], f"{label} k={p['k']} 감지 불일치"
        assert (c["mission"] == "1") == p["mission"], f"{label} k={p['k']} 임무 판정 불일치"
        for key in ("ema_fast", "ema_slow", "predicted", "scale", "v", "a", "j", "snap"):
            d = abs(float(c[key]) - p[key])
            assert d < TOL, f"{label} k={p['k']} {key}: C++ {c[key]} vs py {p[key]} (차 {d:.3e})"


def test_cpp_rule_constants_match_python():
    """C++ 쪽 문턱값이 파이썬 상수와 같은지 — 트레이스가 못 잡는 경계까지 본다.

    C++ 헤더의 리터럴을 직접 읽는다. 두 곳에 같은 수를 적어 두는 이상, 한쪽만
    고쳐지는 사고를 시험으로 막는 수밖에 없다.
    """
    hpp = os.path.join(ROOT, "controller_cpp", "include", "qc_controller.hpp")
    with open(hpp, encoding="utf-8") as f:
        src = f.read()
    seg = src[src.index("struct SpecLatencyRule"):src.index("struct SpecReport")]
    def literal(name):
        m = re.search(rf"\b{name}\s*=\s*([0-9.eE+-]+)\s*;", seg)
        assert m, f"{name} 리터럴을 C++ 헤더에서 못 찾음 (선언 형태가 바뀌었나)"
        return float(m.group(1))

    for name, val in (("attCleanS", cap.LAT_ATT_CLEAN_S),
                      ("attMaxS", cap.LAT_ATT_MAX_S),
                      ("attMargin", cap.LAT_ATT_MARGIN_SCALE)):
        assert literal(name) == pytest.approx(val), f"{name}: C++ {literal(name)} vs py {val}"

    # 위치 실측표 앵커 개수도 같아야 한다 (표를 갱신하면 양쪽 다 고쳤는지 확인)
    for name, tbl, label in (("posN", cap._LAT_POS_ANCHORS, "기본"),
                             ("gustN", cap._LAT_POS_ANCHORS_GUST, "돌풍")):
        n_cpp = int(literal(name))
        assert n_cpp == len(tbl), (
            f"{label} 앵커 개수 불일치: C++ {n_cpp} vs py {len(tbl)} — "
            "실측표를 한쪽만 갱신했다")
    assert literal("gustRhoRef") == pytest.approx(cap.GUST_RHO_REF)


def run_cpp_rec():
    out = subprocess.run([EXE, "--rec"], capture_output=True, text=True, check=True)
    lines = out.stdout.strip().splitlines()
    head = lines[0].split(",")
    return [dict(zip(head, ln.split(","))) for ln in lines[1:]]


def test_recovery_watcher_parity():
    """회복 감시도 두 언어가 같은 수를 내야 한다.

    이건 스펙 표(정적 데이터)와 달리 **상태를 가진 루프**라, 한쪽이 어긋나면
    기체마다 다른 시점에 스펙을 깎게 된다. 같은 입력 수열로 9000 스텝을 대조한다.
    """
    from recovery_watcher import RecoveryWatcher
    cpp = run_cpp_rec()
    w = RecoveryWatcher()
    assert len(cpp) == 9000
    t = 0.0
    for k, c in enumerate(cpp):
        err = 0.09 if 20.0 <= t < 50.0 else 0.01
        w.observe(err, True, 0.01)
        s = w.decide(2.0)
        assert int(c["k"]) == k
        for key, got in (("t_above", w.t_above), ("t_clean", w.t_clean),
                         ("scale", s), ("ratio", w.last_ratio)):
            d = abs(float(c[key]) - got)
            assert d < 1e-9, f"k={k} {key}: C++ {c[key]} vs py {got} (차 {d:.3e})"
        assert int(c["cuts"]) == w.cuts, f"k={k} cuts 불일치"
        t += 0.01


def test_recovery_constants_match():
    """C++ 헤더 리터럴 == 파이썬 기본값 (한쪽만 고치는 사고 방지)."""
    from recovery_watcher import LEAD_MARGIN, S_FLOOR, RecoveryWatcher
    hpp = os.path.join(ROOT, "controller_cpp", "include", "qc_controller.hpp")
    with open(hpp, encoding="utf-8") as f:
        src = f.read()
    seg = src[src.index("struct RecoveryWatcher"):src.index("void reset() {", src.index("struct RecoveryWatcher"))]

    def lit(name):
        m = re.search(rf"\b{name}\s*=\s*([0-9.eE+-]+)\s*;", seg)
        assert m, f"{name} 리터럴을 C++ 에서 못 찾음"
        return float(m.group(1))

    w = RecoveryWatcher()
    assert lit("kLeadMargin") == pytest.approx(LEAD_MARGIN)
    assert lit("kRecFloor") == pytest.approx(S_FLOOR)
    for cname, pval in (("trackBandM", w.track_band_m), ("settleS", w.settle_s),
                        ("cutGain", w.cut_gain), ("maxCut", w.max_cut),
                        ("minPeriodS", w.min_period_s), ("cleanHoldS", w.clean_hold_s),
                        ("restoreTauS", w.restore_tau_s)):
        assert lit(cname) == pytest.approx(pval), f"{cname}: C++ {lit(cname)} vs py {pval}"


MLERP_M = os.path.join(ROOT, "controller", "Quadcopter-Drone-Model-Simscape",
                       "Scripts_Data", "qc_mass_lerp_apply.m")


def test_mass_lerp_matches_matlab_anchors():
    """C++ `qc_mass_lerp` 의 앵커 == MATLAB `qc_mass_lerp_apply.m` 의 앵커.

    같은 1차식을 두 곳에 적어 두는 이상 한쪽만 고쳐지는 사고가 난다. MATLAB 쪽이
    검증을 돌리는 원본이므로 그쪽을 진실로 보고 C++ 를 대조한다.
    """
    if not os.path.exists(MLERP_M):
        pytest.skip("qc_mass_lerp_apply.m 없음")
    with open(MLERP_M, encoding="utf-8") as f:
        msrc = f.read()

    def m_anchor(name):
        m = re.search(rf"c\.{name}\s*=\s*L\(\s*([0-9.eE+-]+)\s*,\s*([0-9.eE+-]+)\s*\)", msrc)
        assert m, f"MATLAB 에서 {name} 앵커를 못 찾음"
        return float(m.group(1)), float(m.group(2))

    out = subprocess.run([EXE, "--mass"], capture_output=True, text=True, check=True)
    lines = out.stdout.strip().splitlines()
    head = lines[0].split(",")
    rows = [dict(zip(head, ln.split(","))) for ln in lines[1:]]
    at0 = next(r for r in rows if float(r["m"]) == 0.0)
    at1 = next(r for r in rows if float(r["m"]) == 1.0)

    pairs = [("sA", "sA"), ("sZ", "sZ"), ("r_att", "rAtt"), ("limit_att", "limAtt"),
             ("kp_pos", "kpPos"), ("filtPz", "filtPz"),
             ("biasChassis", "bias"), ("nl_gmax", "nlGmax")]
    for mname, cname in pairs:
        a0, a1 = m_anchor(mname)
        assert float(at0[cname]) == pytest.approx(a0, abs=1e-9), f"{mname} 0 kg 불일치"
        assert float(at1[cname]) == pytest.approx(a1, abs=1e-9), f"{mname} 1 kg 불일치"

    # 파생 관계는 보간이 아니라 재계산이어야 한다 (불변식 보존)
    for r in rows:
        assert float(r["kdPos"]) == pytest.approx(0.4 * float(r["kpPos"]), abs=1e-12)


def test_mass_lerp_is_identity_at_1kg():
    """1 kg 에서 신·구 법칙이 같아야 한다 — 아니면 1 kg 골든 트레이스가 깨진다."""
    out = subprocess.run([EXE, "--mass"], capture_output=True, text=True, check=True)
    lines = out.stdout.strip().splitlines()
    head = lines[0].split(",")
    at1 = next(dict(zip(head, ln.split(","))) for ln in lines[1:]
               if float(ln.split(",")[0]) == 1.0)
    assert float(at1["sA"]) == pytest.approx(float(at1["sA_old"]), abs=1e-12)
    assert float(at1["sZ"]) == pytest.approx(float(at1["sZ_old"]), abs=1e-12)
