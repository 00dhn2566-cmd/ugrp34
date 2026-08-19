# -*- coding: utf-8 -*-
"""제어 파트 파이프라인 다이어그램 (matplotlib) → figure/00_pipeline/fig_pipeline.png (+ .svg)
랩미팅 슬라이드용: 16:9, 4개 레인(입력 → 파이프라인 → 실행기 → 사후 루프), 직교 화살표, 큰 글씨.
사용: cd control_seoungjin && python make_pipeline_figure.py
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figure", "00_pipeline")
plt.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

INK, MUTED, EDGE = "#1f1f1f", "#5f5f5f", "#3a3a3a"
PAL = {  # (면, 테두리)
    "ext": ("#FFFFFF", "#1E6FB5"), "sup": ("#FFFFFF", "#C62828"),
    "in": ("#E9F1FB", "#1E6FB5"), "plan": ("#E9F1FB", "#1E6FB5"), "shape": ("#E7F3E4", "#2E7D32"),
    "gate": ("#FFF4D6", "#E08A00"), "out": ("#F1F1F1", "#616161"),
    "plant": ("#FCE9DC", "#C55A11"), "ctrl": ("#F9DFCC", "#C55A11"),
    "post": ("#EEE8F7", "#6A3DAA"),
}
BLUE, RED, ORANGE, PURPLE = "#1E6FB5", "#C62828", "#C55A11", "#6A3DAA"

X0, X1 = 1.45, 15.75            # 내용 영역 좌우


def box(ax, x, y, w, h, title, sub=None, key="out", fs=12, sfs=9.2, lw=1.8, dy=0.24):
    fc, ec = PAL[key]
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.14", fc=fc, ec=ec, lw=lw, zorder=3))
    if sub:
        ax.text(x + w / 2, y + h / 2 + dy, title, ha="center", va="center", fontsize=fs, weight="bold", color=INK, zorder=4)
        ax.text(x + w / 2, y + h / 2 - dy + 0.02, sub, ha="center", va="center", fontsize=sfs, color=MUTED, linespacing=1.3, zorder=4)
    else:
        ax.text(x + w / 2, y + h / 2, title, ha="center", va="center", fontsize=fs, weight="bold", color=INK, zorder=4)
    return (x, y, w, h)


def cx(b): return b[0] + b[2] / 2
def cy(b): return b[1] + b[3] / 2
def top(b): return b[1] + b[3]
def bot(b): return b[1]


def path(ax, pts, color=EDGE, lw=1.8, ls="-", head=True):
    """직교 꺾은선 (마지막 구간에 화살촉)."""
    if len(pts) > 2:
        xs, ys = zip(*pts[:-1])
        ax.plot(xs, ys, color=color, lw=lw, ls=ls, solid_capstyle="round", zorder=2)
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle="-|>" if head else "-", mutation_scale=16, color=color, lw=lw, ls=ls,
                                 shrinkA=0, shrinkB=0, zorder=2))


def label(ax, x, y, s, color=MUTED, fs=9.5, ha="center", va="center", weight="normal"):
    ax.text(x, y, s, ha=ha, va=va, fontsize=fs, color=color, weight=weight, zorder=5,
            bbox=dict(fc="white", ec="none", pad=1.5, alpha=0.9))


def lane(ax, y0, y1, name, note=None):
    ax.add_patch(Rectangle((0.25, y0), 16 - 0.5, y1 - y0, fc="#F7F7F7", ec="none", zorder=0))
    ax.text(0.5, (y0 + y1) / 2, name, rotation=90, ha="center", va="center", fontsize=11, weight="bold", color=MUTED)
    if note:
        ax.text(X0, y1 - 0.13, note, ha="left", va="top", fontsize=9, color=MUTED, zorder=5,
                bbox=dict(fc="#F7F7F7", ec="none", pad=1.5))


def main():
    os.makedirs(OUT, exist_ok=True)
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16); ax.set_ylim(0, 9); ax.axis("off")

    ax.text(0.4, 8.62, "제어 파트 파이프라인 — 미션 JSON에서 비행·회신·사후 보정까지", fontsize=20, weight="bold", color=INK, va="center")
    ax.text(15.6, 8.62, "control_seoungjin · 2026-08-18", fontsize=10.5, color=MUTED, va="center", ha="right")

    # ── 레인 배경 ──
    L1 = (6.35, 7.75); L2 = (4.15, 5.95); L3 = (2.05, 3.85); L4 = (0.3, 1.85)
    lane(ax, *L1, "입력")
    lane(ax, *L2, "파이프라인", "traj_pipeline.py — 동사 API: plan · check · splice · emergency · feedback · estimate · learn · status")
    lane(ax, *L3, "실행기", "소비자 — 같은 trajectory.mat/json 을 읽는 세 실행기 (안에 동일한 제어기 캐스케이드)")
    lane(ax, *L4, "사후 루프", "비행 로그로 다음 비행을 고친다 (게인은 불변)")

    # ── 레인 1: 입력 ──
    y1, h1 = 6.55, 1.0
    rl = box(ax, X0 + 0.15, y1, 2.9, h1, "RL / 경로계획 (윤호)", "mission.json — waypoints · limits · dt", key="ext", fs=12)
    opt = box(ax, 4.75, y1, 3.0, h1, "옵션 사이드카", ".options.json — profile · yaw · shaper\npayload_mass_kg · keep_out · strict", key="ext", fs=12)
    sup = box(ax, 8.6, y1, 3.0, h1, "비행 감독자 (§9)", "flight_supervisor.py\nemergency_cmd → flight_state 모드", key="sup", fs=12)
    rep = box(ax, 13.55, y1, 2.2, h1, "회신 (§7)", "trajectory_report.json\nverdict · adjustments · fidelity", key="ext", fs=12)

    # ── 레인 2: 체인 ①~⑧ ──
    y2, h2, gap = 4.35, 1.05, 0.18
    steps = [
        ("① 미션 로드", "스키마 검증\n코어/옵션 분리", "in"),
        ("② 시간 부여", "path_time 7차 다항\nfly-through", "plan"),
        ("③ 재샘플", "dt 균일\n+ hold 패딩", "plan"),
        ("④ 스무더", "물리 포락선 v/a/j\n허용 한계 = 질량별", "shape"),
        ("⑤ ZVD 셰이퍼", "짐 진자 1.8 Hz\n+ 역위상 오프셋", "shape"),
        ("⑥ 게이트", "v/a/j/snap 검사\nkeep_out 전 샘플", "gate"),
        ("⑦ yaw", "heading · hold\nlook_at · scan", "plan"),
        ("⑧ 저장·회신", "trajectory.mat/json\npipeline_meta · report", "out"),
    ]
    n = len(steps); w2 = (X1 - X0 - gap * (n - 1)) / n
    bx = [box(ax, X0 + i * (w2 + gap), y2, w2, h2, t, s, key=k, fs=11.5, sfs=8.8) for i, (t, s, k) in enumerate(steps)]
    for a, b in zip(bx[:-1], bx[1:]):
        path(ax, [(a[0] + a[2], cy(a)), (b[0], cy(b))])

    # 입력 → ①  (RL·옵션 합류)
    yj = 6.2
    path(ax, [(cx(opt), bot(opt)), (cx(opt), yj), (cx(bx[0]), yj), (cx(bx[0]), top(bx[0]))], color=BLUE)
    path(ax, [(cx(rl), bot(rl)), (cx(rl), yj)], color=BLUE, head=False)
    ax.plot([cx(rl)], [yj], "o", color=BLUE, ms=4, zorder=3)
    # 감독자 → ⑤ (점선 빨강)
    path(ax, [(cx(sup), bot(sup)), (cx(sup), yj), (cx(bx[4]) + 0.25, yj), (cx(bx[4]) + 0.25, top(bx[4]))], color=RED, ls="--")
    label(ax, (cx(sup) + cx(bx[4])) / 2 + 0.1, yj + 0.15, "emergency / splice — 비행 중 새 명령·정지 궤적", color=RED, fs=9)
    # ⑧ → 회신 → RL (점선 파랑, 상단 우회)
    path(ax, [(cx(bx[7]), top(bx[7])), (cx(bx[7]), bot(rep))], color=EDGE)
    yt = 7.95
    path(ax, [(cx(rep), top(rep)), (cx(rep), yt), (cx(rl), yt), (cx(rl), top(rl))], color=BLUE, ls="--")
    label(ax, 8.4, yt + 0.16, "회신 → RL : verdict · adjustments · limits_budget · command_fidelity = RL 보상 신호 (§7.4)", color=BLUE, fs=9.5)
    label(ax, X0 + 0.15, 6.42, "완화 정책: 초과 limits 는 거부 대신 클램프·재시간화 → adjustments 로 통지", fs=8.8, ha="left")

    # ── 레인 3: 실행기 ──
    y3, h3 = 2.25, 1.05
    mat = box(ax, X0, y3, 3.15, h3, "MATLAB · Simscape 구운 모델", "run_traj_baked.m → 모델 워크스페이스\n메모리 수술 (save 금지)", key="plant", fs=11.5, sfs=8.8)
    cpp = box(ax, 4.85, y3, 3.15, h3, "C++ 제어기 (controller_cpp)", "qc_io: trajectory.json → current_state 20~50 Hz\n골든 트레이스 = MATLAB 동일성", key="plant", fs=11.5, sfs=8.8)
    isa = box(ax, 8.25, y3, 2.75, h3, "Isaac Sim 내보내기", "isaacsim_export.py\n윤호 스키마 + hash 사이드카", key="plant", fs=11.5, sfs=8.8)
    ctl = box(ax, 11.6, y3, X1 - 11.6, h3, "제어기 캐스케이드 (1 kHz)", "위치 PID → 비선형 자세 게인 g(|e|) → 자세 PID → yaw\n→ 믹서 + FF √질량 → 모터 PID · 질량 스케줄 · SwingDamper", key="ctrl", fs=11.5, sfs=8.8)
    # ⑧ → 버스 → 세 실행기
    yb = 4.05
    ax.plot([cx(mat), cx(bx[7])], [yb, yb], color=EDGE, lw=1.8, zorder=2)
    path(ax, [(cx(bx[7]), bot(bx[7])), (cx(bx[7]), yb)], color=EDGE, head=False)
    for b in (mat, cpp, isa):
        path(ax, [(cx(b), yb), (cx(b), top(b))], color=EDGE)
        ax.plot([cx(b)], [yb], "o", color=EDGE, ms=4, zorder=3)
    # 실행기 ⊃ 캐스케이드
    path(ax, [(isa[0] + isa[2], cy(isa)), (ctl[0], cy(ctl))], color=ORANGE, ls=":", lw=1.6)
    label(ax, (isa[0] + isa[2] + ctl[0]) / 2, cy(isa) + 0.2, "안에 탑재", color=ORANGE, fs=8.5)

    # ── 레인 4: 사후 루프 ──
    y4, h4 = 0.45, 1.05
    ws = [2.35, 2.35, 2.35, 2.55, 3.4]; g4 = (X1 - X0 - sum(ws)) / 4
    xs = [X0]
    for wv in ws[:-1]:
        xs.append(xs[-1] + wv + g4)
    logb = box(ax, xs[0], y4, ws[0], h4, "비행 로그", "sim_result_*.mat\ncurrent_state 로그", key="post", fs=11.5, sfs=8.8)
    fb = box(ax, xs[1], y4, ws[1], h4, "feedback", "잔류 지터 f₀ → 셰이퍼 갱신\nattitude_feedback · 원장", key="post", fs=11.5, sfs=8.8)
    est = box(ax, xs[2], y4, ws[2], h4, "estimate", "질량 · K_thrust · K_drag · 관성\nparam_estimate.json (§6)", key="post", fs=11.5, sfs=8.8)
    cs = box(ax, xs[3], y4, ws[3], h4, "counter_swing", "꼬리 스윙 FFT → 역위상 오프셋\n실시간 댐퍼 (진행)", key="post", fs=11.5, sfs=8.8)
    perf = box(ax, xs[4], y4, ws[4], h4, "최종 log → 대표 지표", "perf_metrics · figure/09_headline\n최악조건 대표지표 · SPEC v0.3", key="post", fs=11.5, sfs=8.8)
    for a, b in zip((logb, fb, est, cs), (fb, est, cs, perf)):
        path(ax, [(a[0] + a[2], cy(a)), (b[0], cy(b))], color=PURPLE)
    # 실행기 → 로그
    path(ax, [(cx(mat), bot(mat)), (cx(mat), top(logb))], color=PURPLE)
    # counter_swing → ⑤ (다음 비행 기준, 오른쪽 틈으로 우회)
    xg = (isa[0] + isa[2] + ctl[0]) / 2
    path(ax, [(cx(cs), top(cs)), (cx(cs), 1.98), (xg, 1.98), (xg, y2 - 0.16), (cx(bx[4]) - 0.25, y2 - 0.16), (cx(bx[4]) - 0.25, bot(bx[4]))],
         color=PURPLE, ls="--")
    label(ax, xg, (y3 + y2) / 2 + 0.55, "다음 비행 기준에 반영\n(⑤ 오프셋 · 셰이퍼 f₀)", color=PURPLE, fs=8.8)

    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"fig_pipeline.{ext}"), dpi=200, facecolor="white")
    plt.close(fig)
    print("->", os.path.join(OUT, "fig_pipeline.png"))


if __name__ == "__main__":
    main()
