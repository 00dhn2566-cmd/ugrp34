"""시나리오별 시계열 리포트 — 시간 / 자세 3축 / 위치오차 / 경유점 통과시각.

`run_scenarios.sh`(플랜트 배터리)와 다르다. 저건 `input/`·`output/` 을 안 쓰고
궤적을 MATLAB 안에서 만든다. 이건 **인터페이스 경로 그대로** 간다:

    input/<mission>.json -> traj_pipeline.py -> output/trajectory.mat
      -> run_traj_baked.m -> sim_result_baked.mat -> 여기

그래서 여기 나오는 숫자는 "상위가 실제로 받게 될 성능"이다.

★ 시나리오마다 산출물을 `output/scenarios/<name>/` 로 **격리해서 보관**한다.
   INTERFACE_SPEC §10-① 때문이다 — `trajectory.mat` 이 `trajectory_hash` 를 안 실어
   보내서 `analyze_flight_log.py` 가 해시를 `output/pipeline_meta.json` 에서 읽는데,
   그 파일은 다음 미션을 생성하는 순간 덮인다. 미션별로 스냅샷을 떠 두면 어느 비행의
   실측인지가 파일 위치로 보장된다 (해시 사슬을 고치기 전까지의 우회다).

사용:
    python report_scenarios.py --run            # 시뮬까지 (MATLAB, 시나리오당 수 분)
    python report_scenarios.py                  # 저장된 결과로 표만
    python report_scenarios.py --only flythrough
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys

import numpy as np
from scipy.io import loadmat

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(HERE, "controller", "Quadcopter-Drone-Model-Simscape")
OUT_ROOT = os.path.join(HERE, "output", "scenarios")
MATLAB = os.environ.get("MATLAB_EXE",
                        r"C:\Program Files\MATLAB\R2026a\bin\matlab.exe")

# (이름, 미션 파일, 왜 이걸 고르나)
SCENARIOS = [
    ("flythrough", "flythrough_mission.json",
     "무정지 4점 통과 — 창을 연달아 지나는 임무의 원형"),
    ("example", "example_mission.json",
     "정지형 5점 + 고도 0.15->6 m — 수직 기동이 섞인 기본형"),
    ("scan", "scan_mission.json",
     "이동 중 yaw 2pi 스캔 — 기수가 도는 유일한 케이스 (자세 3축 중 yaw 축 확인용)"),
]

# 외란 변형. 능력 카드 R1 규약 그대로 (roll 토크 0.3 N*m x 0.3 s).
# 시각은 **가운데 경유점의 계획 통과시각 0.5 s 전** 으로 잡는다 - 창을 지나기 직전에
# 맞는 것이 이 임무의 최악이고, '외란을 맞고도 창을 제때 지나는가' 가 우리가 답해야
# 하는 질문이기 때문이다. 이동 한복판(T/2)은 그 질문을 비껴간다.
DIST_SPEC = "tx:0.3"
DIST_DUR = 0.3
DIST_LEAD_S = 0.5

HIT_R = 0.10           # 통과 판정 반경 [m]
HIT_WINDOW_S = 2.0     # 계획 진입 시각 주변 관측 창 [s] (§7 확정사항 2)
SAMPLE_DT = 1.0        # 표 간격 [s]


def sh(cmd, **kw):
    print("  $ " + " ".join(str(c) for c in cmd))
    return subprocess.run(cmd, check=True, **kw)


def dist_time(name, mission):
    """외란 시각 = 가운데 경유점의 계획 통과시각 - DIST_LEAD_S."""
    tj = os.path.join(OUT_ROOT, name, "trajectory.json")
    if not os.path.isfile(tj):
        return None
    traj = json.load(open(tj, encoding="utf-8"))
    t_plan = np.asarray(traj["t"], float)
    p_plan = np.asarray(traj["pos"], float)
    wps = json.load(open(os.path.join(HERE, "input", mission),
                        encoding="utf-8")).get("waypoints") or []
    if len(wps) < 3:
        return float(t_plan[-1] / 2.0)
    w = np.asarray(wps[len(wps) // 2], float)
    k = int(np.argmin(np.linalg.norm(p_plan - w, axis=1)))
    return max(0.5, float(t_plan[k]) - DIST_LEAD_S)


def run_one(name, mission, dist=False):
    """파이프라인 -> 시뮬 -> 스냅샷. 한 번에 하나만 (RAM 16 GB)."""
    tag = name + ("_dist" if dist else "")
    dst = os.path.join(OUT_ROOT, tag)
    os.makedirs(dst, exist_ok=True)
    print("\n===== [%s] 생성 =====" % tag)
    sh([sys.executable, os.path.join(HERE, "traj_pipeline.py"),
        "--input", os.path.join(HERE, "input", mission)])

    out = os.path.join(HERE, "output")
    for f in ("trajectory.mat", "trajectory.json", "pipeline_meta.json"):
        shutil.copy2(os.path.join(out, f), os.path.join(dst, f))
    shutil.copy2(os.path.join(out, "trajectory.mat"),
                 os.path.join(MODEL_DIR, "trajectory.mat"))

    env = dict(os.environ)
    if dist:
        td = dist_time(name, mission)
        if td is None:
            td = 5.0
        env["TRAJ_DIST"] = DIST_SPEC
        env["TRAJ_DIST_T"] = "%.3f" % td
        env["TRAJ_DIST_DUR"] = "%.3f" % DIST_DUR
        json.dump({"spec": DIST_SPEC, "t": td, "dur": DIST_DUR},
                  open(os.path.join(dst, "disturbance.json"), "w"))
        print("  외란: %s @ t=%.2f s, %.2f s" % (DIST_SPEC, td, DIST_DUR))

    print("===== [%s] 비행 =====" % tag)
    log = os.path.join(dst, "run.log")
    with open(log, "wb") as f:
        subprocess.run([MATLAB, "-batch", "run_traj_baked"],
                       cwd=MODEL_DIR, stdout=f, stderr=subprocess.STDOUT,
                       env=env)
    src = os.path.join(MODEL_DIR, "sim_result_baked.mat")
    if not os.path.isfile(src):
        raise RuntimeError("[%s] 비행 실패 - %s 확인" % (tag, log))
    shutil.copy2(src, os.path.join(dst, "sim_result.mat"))
    os.remove(src)   # 다음 런이 실패해도 옛 결과를 물려받지 않게
    print("  -> %s" % dst)


def _sig(m, key):
    """To Workspace StructureWithTime -> (t, v)."""
    s = m[key]
    return np.ravel(s.time), np.ravel(s.signals.values)


def load_run(name):
    d = os.path.join(OUT_ROOT, name)
    mat = os.path.join(d, "sim_result.mat")
    if not os.path.isfile(mat):
        return None
    m = loadmat(mat, squeeze_me=True, struct_as_record=False)
    t = np.ravel(m["sim_time"])
    act = np.column_stack([np.ravel(m["act_%s1" % a])[:len(t)] for a in "xyz"])
    des = np.column_stack([np.ravel(m["des_%s1" % a])[:len(t)] for a in "xyz"])
    att = {}
    for k, key in (("roll", "real_roll"), ("pitch", "real_pitch"), ("yaw", "real_yaw")):
        ta, va = _sig(m, key)
        att[k] = np.degrees(np.interp(t, ta, va))      # 로그는 rad
    traj = json.load(open(os.path.join(d, "trajectory.json"), encoding="utf-8"))
    # ★ yaw 는 명령이 0 이 아니다 (진행방향 추종 / 스캔). roll/pitch 는 명령이 0 근처라
    # 값을 그대로 봐도 오차지만 yaw 는 아니다 - 계획 yaw 를 빼야 오차가 된다.
    # 안 빼면 스캔 미션의 'yaw RMS 125도' 같은 숫자가 나오는데, 그건 성능이 아니라 명령이다.
    yp = np.degrees(np.interp(t, np.asarray(traj["t"], float),
                              np.asarray(traj["yaw_rad"], float)))
    dy = (att["yaw"] - yp + 180.0) % 360.0 - 180.0      # ±180 랩
    att_err = {"roll": att["roll"], "pitch": att["pitch"], "yaw": dy}
    meta = json.load(open(os.path.join(d, "pipeline_meta.json"), encoding="utf-8"))
    return dict(t=t, act=act, des=des, att=att, att_err=att_err,
                yaw_plan=yp, traj=traj, meta=meta, dir=d)


def wp_passage(R, mission):
    """경유점 통과시각 = **계획과 실측을 같은 반경으로** 재고 그 차이를 본다.

    왜 이렇게 정의하나 (09-06, 두 번 고침):
      1차 '창 안 최근접' -> 종점에서 망가진다. 도착 후 머무는 동안 계속 가까워지므로
         최근접이 창 끝으로 찍혀 있지도 않은 '지연 +2.00 s' 가 나온다.
      2차 '반경 최초 진입' -> 정지형에서 반대로 망가진다. 감속하며 들어오느라 반경
         안에 몇 초씩 머물러, 진입 시각이 계획 통과시각보다 앞서 '-2.00 s' 가 나온다.
         **반경을 고르는 순간 편향이 생긴다.**
      3차(현재) 계획 궤적에도 **같은 반경**을 적용해 진입 시각을 뽑고, 실측 진입과
         비교한다. 반경이 양쪽에 똑같이 작용하므로 상쇄되고, 남는 것은 순수 추종 지연이다.

    창(HIT_WINDOW_S)은 여전히 필요하다 - 없으면 왕복 경로에서 '돌아올 때 스친 것' 을
    통과로 오인한다 (§7 확정사항 2).
    """
    wps = json.load(open(os.path.join(HERE, "input", mission),
                        encoding="utf-8")).get("waypoints") or []
    t_plan = np.asarray(R["traj"]["t"], float)
    p_plan = np.asarray(R["traj"]["pos"], float)
    rows = []
    for i, w in enumerate(wps):
        w = np.asarray(w, float)
        dp = np.linalg.norm(p_plan - w, axis=1)
        inp = np.where(dp < HIT_R)[0]
        # 계획 진입 시각. 반경 안에 안 들어오는 경유점이면 최근접 시각으로 (성형기가
        # 모서리를 깎아 계획 자체가 경유점을 스치는 경우 - fly_through 에서 생긴다)
        tp = float(t_plan[inp[0]]) if inp.size else float(t_plan[int(np.argmin(dp))])
        plan_r_cm = float(dp.min() * 100.0)

        win = np.abs(R["t"] - tp) <= HIT_WINDOW_S
        if not win.any():
            rows.append((i, w, tp, None, None, plan_r_cm, False)); continue
        idx = np.where(win)[0]
        d = np.linalg.norm(R["act"][idx] - w, axis=1)
        hit = np.where(d < HIT_R)[0]
        j = idx[hit[0]] if hit.size else idx[int(np.argmin(d))]
        rows.append((i, w, tp, float(R["t"][j]), float(d.min() * 100.0),
                     plan_r_cm, bool(hit.size)))
    return rows


def report(name, mission, why):
    R = load_run(name)
    print()
    print("=" * 86)
    print("[%s] %s" % (name, why))
    if R is None:
        print("  결과 없음 — `python report_scenarios.py --run --only %s`" % name)
        return
    t, act, des, att = R["t"], R["act"], R["des"], R["att_err"]
    err = (act - des) * 100.0                       # cm
    en = np.linalg.norm(err, axis=1)
    print("  미션 %s | 궤적 hash %s | T=%.2f s" %
          (mission, R["meta"].get("trajectory_hash"), t[-1]))
    print("-" * 86)
    print("%8s | %8s %8s %8s | %8s %8s %8s %8s" %
          ("t[s]", "roll[°]", "pitch[°]", "yaw오차[°]", "ex[cm]", "ey[cm]", "ez[cm]", "|e|[cm]"))
    print("-" * 86)
    tq = np.arange(0.0, t[-1] + 1e-9, SAMPLE_DT)
    for x in tq:
        i = int(np.argmin(np.abs(t - x)))
        print("%8.1f | %8.2f %8.2f %8.2f | %8.2f %8.2f %8.2f %8.2f" %
              (t[i], att["roll"][i], att["pitch"][i], att["yaw"][i],
               err[i, 0], err[i, 1], err[i, 2], en[i]))
    print("-" * 86)
    ipk = int(np.argmax(en))
    print("최대 |e| %.2f cm @ t=%.2f s   |   자세 최대: roll %.2f° / pitch %.2f° / yaw %.2f°" %
          (en[ipk], t[ipk], np.max(np.abs(att["roll"])),
           np.max(np.abs(att["pitch"])), np.max(np.abs(att["yaw"]))))
    print("RMS: |e| %.2f cm | roll %.2f° / pitch %.2f° / yaw %.2f°   |   종점오차 %.2f cm" %
          (float(np.sqrt(np.mean(en ** 2))),
           float(np.sqrt(np.mean(att["roll"] ** 2))),
           float(np.sqrt(np.mean(att["pitch"] ** 2))),
           float(np.sqrt(np.mean(att["yaw"] ** 2))), en[-1]))

    rows = wp_passage(R, mission)
    if rows:
        print()
        print("  경유점 통과 (계획·실측을 같은 반경 %.0f cm 로 재고 진입 시각을 비교)" % (HIT_R*100))
        print("  %3s %-22s %10s %10s %8s %9s %9s" %
              ("#", "경유점 [m]", "계획[s]", "실제[s]", "지연[s]", "계획r[cm]", "실측r[cm]"))
        for i, w, tp, ta, dcm, prc, hit in rows:
            print("  %3d %-22s %10.2f %10s %8s %9s %9s   %s" %
                  (i, "(%.1f, %.1f, %.1f)" % tuple(w), tp,
                   "-" if ta is None else "%.2f" % ta,
                   "-" if ta is None else "%+.2f" % (ta - tp),
                   "%.2f" % prc,
                   "-" if dcm is None else "%.2f" % dcm,
                   "" if hit else "(반경 미진입)"))


def make_figures(todo):
    """시나리오별 그림. 표를 눈으로 훑기 어려워서(런당 수천 샘플) 그림이 본체다.

    한 장에 3단: 자세 3축 / 위치오차 성분 / |오차| + 경유점 통과 표시.
    무외란(실선)과 외란(점선)을 **같은 축에** 겹친다 — 따로 그리면 눈이 두 그림의
    세로 눈금을 못 맞춰서 차이를 과대/과소로 읽는다.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for cand in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
        try:
            matplotlib.font_manager.findfont(cand, fallback_to_default=False)
            plt.rcParams["font.family"] = cand
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["mathtext.fontset"] = "dejavusans"
    plt.rcParams["figure.dpi"] = 130
    plt.rcParams["savefig.bbox"] = "tight"

    outdir = os.path.join(HERE, "figure", "17_scenarios")
    os.makedirs(outdir, exist_ok=True)
    C = {"roll": "#1f77b4", "pitch": "#2ca02c", "yaw": "#d62728"}
    CE = {0: "#1f77b4", 1: "#2ca02c", 2: "#d62728"}
    summary = []

    for name, mission, why in todo:
        A = load_run(name)
        B = load_run(name + "_dist")
        if A is None:
            continue
        fig, ax = plt.subplots(3, 1, figsize=(9.5, 8.4), sharex=True)
        td = None
        if B is not None:
            dj = os.path.join(OUT_ROOT, name + "_dist", "disturbance.json")
            if os.path.isfile(dj):
                d = json.load(open(dj, encoding="utf-8"))
                td = (d["t"], d["t"] + d["dur"], d["spec"])

        for k in ("roll", "pitch", "yaw"):
            ax[0].plot(A["t"], A["att_err"][k], color=C[k], lw=1.2, label="%s" % k)
            if B is not None:
                ax[0].plot(B["t"], B["att_err"][k], color=C[k], lw=1.0, ls="--", alpha=0.75)
        ax[0].set_ylabel("자세 오차 [°]")
        ax[0].legend(loc="upper right", fontsize=8, ncol=3)

        eA = (A["act"] - A["des"]) * 100.0
        for j, lab in enumerate(("ex", "ey", "ez")):
            ax[1].plot(A["t"], eA[:, j], color=CE[j], lw=1.2, label=lab)
            if B is not None:
                eB = (B["act"] - B["des"]) * 100.0
                ax[1].plot(B["t"], eB[:, j], color=CE[j], lw=1.0, ls="--", alpha=0.75)
        ax[1].set_ylabel("위치오차 성분 [cm]")
        ax[1].legend(loc="upper right", fontsize=8, ncol=3)

        nA = np.linalg.norm(eA, axis=1)
        ax[2].plot(A["t"], nA, color="#333333", lw=1.4, label="무외란")
        if B is not None:
            nB = np.linalg.norm((B["act"] - B["des"]) * 100.0, axis=1)
            ax[2].plot(B["t"], nB, color="#d62728", lw=1.4, ls="--", label="외란")
        ax[2].set_ylabel("|위치오차| [cm]")
        ax[2].set_xlabel("시간 [s]")

        rows = wp_passage(A, mission)
        rowsB = wp_passage(B, mission) if B is not None else []
        for i, w, tp, ta, dcm, prc, hit in rows:
            for k in range(3):
                ax[k].axvline(tp, color="#999999", lw=0.7, ls=":", zorder=0)
            ax[2].annotate("wp%d" % i, (tp, ax[2].get_ylim()[1]),
                           fontsize=7.5, color="#666666",
                           ha="center", va="top")
        if td is not None:
            for k in range(3):
                ax[k].axvspan(td[0], td[1], color="#d62728", alpha=0.13, zorder=0)
            ax[0].annotate("외란 %s" % td[2], (td[1], ax[0].get_ylim()[1]),
                           fontsize=8, color="#d62728", ha="left", va="top")
        for k in range(3):
            ax[k].grid(alpha=0.25)
        ax[2].legend(loc="upper left", fontsize=8)
        fig.suptitle("[%s] %s" % (name, why), fontsize=10.5)
        fig.tight_layout()
        pth = os.path.join(outdir, "%s.png" % name)
        fig.savefig(pth)
        plt.close(fig)
        print("  " + pth)
        summary.append((name, rows, rowsB))

    # 통과 지연/오차 요약 한 장
    if summary:
        fig, ax = plt.subplots(2, 1, figsize=(9.5, 5.6), sharex=True)
        labs, dly_a, dly_b, err_a, err_b, seps = [], [], [], [], [], []
        for name, rows, rowsB in summary:
            mb = {r[0]: r for r in rowsB}
            first = True
            for i, w, tp, ta, dcm, prc, hit in rows:
                if ta is None or i == 0:
                    continue          # wp0 = 출발점, 항상 0 - 표만 어지럽힌다
                labs.append(("%s" + chr(10) + "wp%d") % (name if first else "", i))
                first = False
                dly_a.append(ta - tp); err_a.append(dcm)
                rb = mb.get(i)
                dly_b.append(rb[3] - rb[2] if rb and rb[3] is not None else np.nan)
                err_b.append(rb[4] if rb and rb[4] is not None else np.nan)
            seps.append(len(labs) - 0.5)
        x = np.arange(len(labs)); w = 0.38
        for k, (va, vb, ylab) in enumerate(
                ((dly_a, dly_b, "통과 지연 [s]"), (err_a, err_b, "통과 오차 [cm]"))):
            ax[k].bar(x - w/2, va, w, color="#333333", label="무외란")
            ax[k].bar(x + w/2, vb, w, color="#d62728", label="외란")
            ax[k].set_ylabel(ylab)
            ax[k].axhline(0, color="#888", lw=0.8)
            ax[k].grid(alpha=0.25, axis="y")
            for sp in seps[:-1]:
                ax[k].axvline(sp, color="#bbbbbb", lw=0.8, ls="--")
            ax[k].legend(fontsize=8, ncol=2)
        ax[1].set_xticks(x)
        ax[1].set_xticklabels(labs, fontsize=8)
        nan_b = sum(1 for v in dly_b if v != v)
        note = "" if nan_b == 0 else "  (외란 런 %d건 미완)" % nan_b
        fig.suptitle("경유점 통과 — 계획 대비 지연과 오차%s" % note, fontsize=10.5)
        fig.tight_layout()
        pth = os.path.join(outdir, "00_passage.png")
        fig.savefig(pth); plt.close(fig)
        print("  " + pth)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="store_true", help="시뮬까지 실행 (MATLAB)")
    ap.add_argument("--only", help="쉼표 구분 시나리오 이름")
    ap.add_argument("--clean-only", action="store_true", help="외란 런 생략")
    ap.add_argument("--dist-only", action="store_true", help="무외란 런 생략")
    a = ap.parse_args()
    sel = set(a.only.split(",")) if a.only else None
    todo = [s for s in SCENARIOS if sel is None or s[0] in sel]
    if a.run:
        for name, mission, _ in todo:
            if not a.dist_only:
                run_one(name, mission, dist=False)
            if not a.clean_only:
                run_one(name, mission, dist=True)
    for name, mission, why in todo:
        report(name, mission, why)
    print()


if __name__ == "__main__":
    main()
