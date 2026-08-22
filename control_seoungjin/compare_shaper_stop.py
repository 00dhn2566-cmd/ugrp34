"""성형기 정지 거동 비교 — 뱅뱅 제동(구판) vs 거리 연동 속도 상한(신판).

2026-08-22. 사용자 지시: "멈추는 것도 점점 smooth 하게, 가속·감속 구간이 겹치면
최대 스펙까지 안 올라가면 되지" → `traj_smoother(..., smooth_stop=True)`.

측정 항목
  overshoot_cm : 목표(입력의 최종 극값) 반대편으로 넘어간 최대량  ← 핵심 비교 지표
  vPk/aPk/jPk  : 성형 결과의 실제 피크 (한계 준수 + '최대 스펙까지 안 올라감' 확인)
  maxDev_cm    : 입력 대비 최대 변형 (정상 궤적 무개입 확인용)
  settle_s     : 목표 ±1 cm 안으로 들어와 유지되는 시각

사용:  python compare_shaper_stop.py
"""
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0] if "\\" in __file__ else ".")
from traj_shaping import traj_smoother  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

VMAX, AMAX, JMAX = 2.0, 2.0, 10.0
DT = 0.01


def make_case(name, T, fn):
    t = np.arange(0.0, T + DT / 2, DT)
    return name, t, np.asarray(fn(t), float)


def step(t, amp, t0):
    return np.where(t >= t0, amp, 0.0)


def ramp(t, amp, t0, dur):
    u = np.clip((t - t0) / dur, 0.0, 1.0)
    return amp * u


def raised_cos(t, amp, t0, dur):
    u = np.clip((t - t0) / dur, 0.0, 1.0)
    return amp * 0.5 * (1 - np.cos(np.pi * u))


CASES = [
    # 이름, 총시간, 기준 생성기
    make_case("step_3m",      12.0, lambda t: step(t, 3.0, 1.0)),          # 계단 3 m (최악)
    make_case("step_0.5m",     8.0, lambda t: step(t, 0.5, 1.0)),          # 짧은 계단 = 가감속 구간 겹침
    make_case("step_0.15m",    6.0, lambda t: step(t, 0.15, 1.0)),         # 서브미터 미세 이동
    make_case("ramp_fast_3m", 12.0, lambda t: ramp(t, 3.0, 1.0, 1.5)),     # 급램프 (v 2 m/s 요구)
    make_case("rc_3m_8s",     16.0, lambda t: raised_cos(t, 3.0, 1.0, 8.0)),  # 정상 궤적 (무개입이어야)
    make_case("updown",       16.0, lambda t: step(t, 2.0, 1.0) - step(t, 2.0, 6.0)),  # 갔다 되돌아옴
]


def metrics(t, p_in, p_out):
    tgt = p_in[-1]
    d = p_out - tgt
    # 오버슈트 = 입력의 전역 극값을 양쪽 어느 쪽으로든 넘어간 최대량 (updown 같은
    # 왕복 궤적에서도 유효하도록 축·부호 무관하게 정의)
    overshoot = max(float(np.max(p_out) - np.max(p_in)),
                    float(np.min(p_in) - np.min(p_out)), 0.0)
    dv = np.diff(p_out) / np.diff(t)
    da = np.diff(dv) / np.diff(t[:-1])
    dj = np.diff(da) / np.diff(t[:-2])
    ok = np.abs(d) <= 0.01
    settle = np.nan
    for i in range(len(t)):
        if ok[i:].all():
            settle = t[i]
            break
    return dict(overshoot_cm=100 * overshoot,
                vPk=float(np.max(np.abs(dv))),
                aPk=float(np.max(np.abs(da))) if len(da) else 0.0,
                jPk=float(np.max(np.abs(dj))) if len(dj) else 0.0,
                maxDev_cm=100 * float(np.max(np.abs(p_out - p_in))),
                settle_s=settle)


def main():
    print(f"한계: v {VMAX} / a {AMAX} / j {JMAX},  dt {DT}\n")
    hdr = (f"{'케이스':<14}{'판':<5}{'오버슈트':>10}{'vPk':>8}{'aPk':>8}{'jPk':>8}"
           f"{'변형':>9}{'정착[s]':>9}")
    print(hdr)
    print("-" * len(hdr.encode("utf-8").decode("utf-8")) if False else "-" * 74)
    worst = []
    for name, t, p in CASES:
        row = {}
        for tag, flag in (("구판", False), ("신판", True)):
            out, _ = traj_smoother(t, p, VMAX, AMAX, JMAX, smooth_stop=flag)
            m = metrics(t, p, out)
            row[tag] = m
            print(f"{name if tag == '구판' else '':<14}{tag:<5}"
                  f"{m['overshoot_cm']:>9.2f}cm{m['vPk']:>8.3f}{m['aPk']:>8.3f}"
                  f"{m['jPk']:>8.2f}{m['maxDev_cm']:>8.1f}cm{m['settle_s']:>9.2f}")
        o0, o1 = row["구판"]["overshoot_cm"], row["신판"]["overshoot_cm"]
        worst.append((name, o0, o1))
        print()
    print("=" * 74)
    print(f"{'케이스':<14}{'구판 오버슈트':>16}{'신판':>12}{'변화':>12}")
    for name, o0, o1 in worst:
        chg = "—" if o0 < 1e-9 else f"{100*(o1/o0 - 1):+.1f}%"
        print(f"{name:<14}{o0:>15.2f}cm{o1:>11.2f}cm{chg:>12}")
    tot0 = sum(o0 for _, o0, _ in worst)
    tot1 = sum(o1 for _, _, o1 in worst)
    print(f"{'합계':<14}{tot0:>15.2f}cm{tot1:>11.2f}cm"
          f"{('—' if tot0 < 1e-9 else f'{100*(tot1/tot0-1):+.1f}%'):>12}")


if __name__ == "__main__":
    main()
