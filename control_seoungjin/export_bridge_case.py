"""다리 궤적을 MATLAB 검증용으로 내보낸다.

2026-08-23. 다리(`traj_bridge`)는 지금까지 **파이썬 안에서만** 확인됐다 — 물리 한계를
안 넘고 기하가 안 바뀐다는 것까지는 봤지만, 실제 기체가 그걸 따라가는지는 안 봤다.
플랜트를 통과시켜야 비로소 검증이다 (이 저장소의 규율: Simulink 가 정답 플랜트).

내보내는 것 (`output/bridge_case.mat`):
    t_base, x_base   비교군 — 감쇄 없이 계속 가는 기준 (전속)
    t_br,   x_br     실험군 — t_derate 에서 감쇄를 시작한 다리
    meta             감쇄 시각·목표 배율·새 한계 등

MATLAB 쪽: diagnose/verify_bridge_sim.m

사용:
    python export_bridge_case.py [--tau-ms 40] [--s 0.55]
"""
from __future__ import annotations

import argparse
import os

import numpy as np
from scipy.io import savemat

from traj_bridge import BASE_LIMITS, _smoothstep7, plan_bridge

DT = 0.01
DX = 3.0
Z0 = 1.0
T0 = 3.0            # 이동 시작 (MATLAB 쪽 스윕과 같은 규약)


def build(v0=1.6, dx=DX, s_target=0.55, t_derate_frac=0.5,
          replan_budget_s=0.25, tail=14.0):
    """전속 기준 + 그 위에 놓은 다리. 둘 다 같은 시간 격자."""
    tm = 2.1875 * dx / v0                       # 7차 스무드스텝의 피크 속도 = 2.1875·dx/T
    t = np.arange(0.0, T0 + tm + tail + DT, DT)
    u = np.clip((t - T0) / tm, 0.0, 1.0)
    base = np.column_stack([dx * _smoothstep7(u),
                            np.zeros_like(t), Z0 * np.ones_like(t)])

    t_derate = T0 + tm * t_derate_frac          # 순항 한복판에서 감쇄 시작
    lim = {k: BASE_LIMITS[k] * s_target ** p
           for k, p in (("v", 1), ("a", 2), ("j", 3), ("snap", 4))}
    br = plan_bridge(t, base, t_now=t_derate, limits_new=lim,
                     replan_budget_s=replan_budget_s)

    # 다리 이후는 **정지 래치**로 끝난다. MATLAB 시뮬은 같은 길이를 돌려야 비교가 되므로
    # 다리 앞(감쇄 전)과 뒤(정지 후)를 붙여 base 와 같은 시간축으로 만든다.
    i0 = int(round(t_derate / DT))
    x_br = np.vstack([base[:i0], br.pos])
    if len(x_br) < len(t):
        x_br = np.vstack([x_br, np.repeat(x_br[-1:], len(t) - len(x_br), axis=0)])
    x_br = x_br[:len(t)]

    meta = dict(
        dx=dx, z0=Z0, t0=T0, tmove=tm, v0=v0,
        t_derate=t_derate, s_target=br.s_target, t_ramp=br.t_ramp,
        t_handoff=br.t_handoff, failsafe_from=br.failsafe_from_s,
        lim_v=lim["v"], lim_a=lim["a"], lim_j=lim["j"], lim_snap=lim["snap"],
        compliant_after=br.compliant_after_s,
        phys_use=max(br.phys_use.values()),
    )
    return t, base, x_br, br, meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tau-ms", type=float, default=40.0,
                    help="MATLAB 에서 걸 위치 경로 지연 [ms] (메타에만 실림)")
    ap.add_argument("--s", type=float, default=0.55,
                    help="목표 스펙 배율 — 40 ms 실측 s_max 기본값")
    ap.add_argument("--out", default=os.path.join("output", "bridge_case.mat"))
    a = ap.parse_args()

    t, base, x_br, br, meta = build(s_target=a.s)
    meta["tau_pos_ms"] = a.tau_ms
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    savemat(a.out, dict(t=t.reshape(-1, 1), x_base=base, x_bridge=x_br,
                        s_of_t=br.s_of_t.reshape(-1, 1), meta=meta))

    print(f"내보냄: {a.out}")
    print(f"  이동 {meta['dx']} m, 전속 v {meta['v0']} m/s (이동시간 {meta['tmove']:.2f} s)")
    print(f"  감쇄 시작 t={meta['t_derate']:.2f} s -> 목표 배율 {meta['s_target']:.2f} "
          f"(v {meta['lim_v']:.3f} m/s), 램프 {meta['t_ramp']:.2f} s")
    print(f"  새 한계 진입 {meta['compliant_after']:.2f} s, 물리 한계 사용 {meta['phys_use']:.2f}")
    for n in br.notes:
        print(f"  · {n}")


if __name__ == "__main__":
    main()
