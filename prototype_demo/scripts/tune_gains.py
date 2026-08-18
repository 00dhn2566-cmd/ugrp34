"""성진 제어기 게인 튜닝 — 고도 → 자세 → 위치 순서로 좁혀 간다.

    python scripts/tune_gains.py --stage alt
    python scripts/tune_gains.py --stage att
    python scripts/tune_gains.py --stage pos
    python scripts/tune_gains.py --stage all      # 순서대로 전부

T/W 는 2.0 으로 고정한다 (maxTorque=1.6, maxPower=400 — 제어기의 limMot=0.25 가
명령 상한이라 플랜트 limitCmd 를 올려봐야 소용없다).

바깥 루프부터 잡으면 안 된다: 고도가 안 잡히면 자세/위치 성적은 전부 낙하의
부산물이라 비교가 무의미하다. 그래서 고도 → 자세 → 위치 순서다.
"""
from __future__ import annotations

import argparse
import itertools
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils import paths  # noqa: E402

paths.bootstrap()

from control import qc  # noqa: E402

TW2 = dict(max_torque=1.6, max_power=400.0)     # T/W = 2.0
ALT_SAT = 120.0


def fly(gains: dict, mode: str = "hover", seconds: float = 8.0, dt: float = 0.001,
        spin_up: float = 1.0, layout=None, mix=None, disturb: float = 0.0):
    """폐루프 1회. 반환 dict(err_final, err_rms, tilt_max, diverged, z_min)."""
    import pybullet as p
    cid = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81, physicsClientId=cid)
    p.setTimeStep(dt, physicsClientId=cid)

    ctl = qc.Controller(alt_cmd_sat=ALT_SAT)
    ctl.set_motor(**TW2)
    if gains:
        ctl.set_gains(**gains)
    if mix:
        ctl.set_mix(**mix)
    if layout is not None:
        ctl.motor_xy = layout
    body = qc.make_body(p, cid, ctl.m_tot, ctl.I_att, ctl.I_yaw, ctl.r_arm,
                        start=(0, 0, 1.0))

    pos0, quat0 = p.getBasePositionAndOrientation(body, physicsClientId=cid)
    for _ in range(int(spin_up / dt)):
        ctl.step([0, 0, 1.0], 0.0, pos0, (0, 0, 0), dt)
        p.resetBasePositionAndOrientation(body, pos0, quat0, physicsClientId=cid)
        p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0], physicsClientId=cid)
    if disturb:
        p.resetBaseVelocity(body, [0, 0, -disturb], [0, 0, 0], physicsClientId=cid)

    n = int(seconds / dt)
    errs = np.zeros(n)
    tilt_max, z_min, diverged = 0.0, 1e9, False
    for k in range(n):
        t = k * dt
        if mode == "hover":
            ref = np.array([0.0, 0.0, 1.0])
        elif mode == "climb":
            ref = np.array([0.0, 0.0, 1.0 if t < 1.0 else 2.0])
        else:                                   # move
            s = np.clip((t - 1.0) / max(seconds - 2.0, 1e-6), 0, 1)
            s = 3 * s ** 2 - 2 * s ** 3
            ref = np.array([2.0 * s, 1.0 * s, 1.0 + 0.5 * s])
        pos, quat = p.getBasePositionAndOrientation(body, physicsClientId=cid)
        rpy = p.getEulerFromQuaternion(quat)
        tilt_max = max(tilt_max, abs(np.degrees(rpy[0])), abs(np.degrees(rpy[1])))
        z_min = min(z_min, pos[2])
        th, dq = ctl.step(ref, 0.0, pos, rpy, dt)
        qc.apply_to_body(p, body, th, dq, ctl.motor_xy, ctl.mix_dir, cid)
        p.stepSimulation(physicsClientId=cid)
        e = float(np.linalg.norm(np.array(pos) - ref))
        errs[k] = e
        if not np.isfinite(e) or e > 25 or tilt_max > 120:
            diverged = True
            errs = errs[:k + 1]
            break
    p.disconnect(physicsClientId=cid)
    tail = errs[int(len(errs) * 0.6):] if len(errs) > 10 else errs
    return {"err_final": float(errs[-1]), "err_rms": float(np.sqrt((tail ** 2).mean())),
            "tilt_max": tilt_max, "diverged": diverged, "z_min": z_min}


def show(tag, r):
    s = "발산" if r["diverged"] else f"{r['err_rms']*1000:8.1f} mm"
    print(f"  {tag:<34}{s:>12}   최종 {r['err_final']*1000:8.1f} mm   "
          f"z최저 {r['z_min']:6.2f} m   기울기 {r['tilt_max']:5.1f}°")
    return r


def stage_alt():
    """고도 루프. kpAlt/kiAlt/kdAlt/limAlt 를 키워 낙하를 잡는다."""
    print("\n=== 1단계: 고도 루프 (mode=hover, T/W 2.0) ===")
    print("기본값 kpAlt 0.5  kiAlt 0.1  kdAlt 0.15  limAlt 10")
    best, best_r = None, None
    show("기본값", fly({}, "hover"))
    for lim in (30.0,):
        for kp, kd in ((20.0, 10.0), (40.0, 16.0), (60.0, 24.0), (40.0, 30.0)):
            g = dict(kpAlt=kp, kdAlt=kd, kiAlt=0.5, limAlt=lim)
            r = show(f"kpAlt {kp:<5} kdAlt {kd:<5} limAlt {lim:<5.0f}",
                     fly(g, "hover"))
            if not r["diverged"] and (best_r is None or r["err_rms"] < best_r["err_rms"]):
                best, best_r = g, r
    if best:
        print(f"\n  -> 최고: {best}  RMS {best_r['err_rms']*1000:.1f} mm")
    else:
        print("\n  -> 호버를 잡는 조합 없음")
    return best


def stage_att(alt_gains):
    """자세 루프. 모터 배치 부호 × 자세 게인 부호를 같이 본다.

    그의 자세 게인은 음수다 (플랜트 이득 b=-0.0296 이 음수라서). 우리 PyBullet
    플랜트는 부호 규약이 다를 수 있으므로 배치 부호와 게인 부호를 함께 스윕한다 —
    둘 중 하나만 뒤집으면 되지 둘 다 뒤집으면 원위치다.
    """
    print("\n=== 2단계: 자세 루프 (mode=move) ===")
    s = 0.15909903
    base = np.array([[+s, -s], [-s, -s], [-s, +s], [+s, +s]])
    best, best_r = None, None
    for sx, sy in itertools.product((1, -1), repeat=2):
        L = base.copy(); L[:, 0] *= sx; L[:, 1] *= sy
        for sg in (+1, -1):
            g = dict(alt_gains)
            g.update(kpAtt=-85.0 * sg, kiAtt=-10.0 * sg, kdAtt=-127.5 * sg)
            r = show(f"layout sx{sx:+d} sy{sy:+d}  자세게인×{sg:+d}",
                     fly(g, "move", layout=L))
            if not r["diverged"] and (best_r is None or r["err_rms"] < best_r["err_rms"]):
                best, best_r = (g, L), r
    if best:
        print(f"\n  -> 최고 RMS {best_r['err_rms']*1000:.1f} mm  "
              f"기울기 {best_r['tilt_max']:.1f}°")
    else:
        print("\n  -> move 를 버티는 조합 없음")
    return best


def stage_pos(att_best):
    """위치 루프. 자세가 잡힌 뒤에만 의미가 있다."""
    print("\n=== 3단계: 위치 루프 (mode=move) ===")
    if att_best is None:
        print("  2단계가 실패해서 생략")
        return None
    g0, L = att_best
    best, best_r = None, None
    for kp, kd in ((3.0, 2.0), (6.0, 3.0), (12.0, 4.8), (18.0, 7.0)):
        for p2a in (1.2, 2.4, 4.0):
            g = dict(g0); g.update(kpPos=kp, kdPos=kd, kpPosZ=kp, kdPosZ=kd,
                                   pos2att=p2a)
            r = show(f"kpPos {kp:<5} kdPos {kd:<5} pos2att {p2a}",
                     fly(g, "move", layout=L))
            if not r["diverged"] and (best_r is None or r["err_rms"] < best_r["err_rms"]):
                best, best_r = g, r
    if best:
        print(f"\n  -> 최고: kpPos {best['kpPos']} kdPos {best['kdPos']} "
              f"pos2att {best['pos2att']}  RMS {best_r['err_rms']*1000:.1f} mm")
    return best


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default="all", choices=("alt", "att", "pos", "all"))
    a = ap.parse_args(argv)

    c = qc.Controller(alt_cmd_sat=ALT_SAT)
    c.set_motor(**TW2)
    print(f"T/W {c.thrust_to_weight():.3f}   m_tot {c.m_tot:.4f} kg   "
          f"altCmdSat {ALT_SAT}")

    alt = stage_alt() if a.stage in ("alt", "all") else \
        dict(kpAlt=60.0, kdAlt=24.0, kiAlt=0.5, limAlt=30.0)
    if a.stage == "alt":
        return 0
    att = stage_att(alt or {})
    if a.stage == "att":
        return 0
    stage_pos(att)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
