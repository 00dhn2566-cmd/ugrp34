# -*- coding: utf-8 -*-
"""발표용 파이프라인 그림 3장 (16:9, 큰 글씨) → figure/00_pipeline/slide_{1,2,3}_*.png/.svg
  slide_1_system.png     전체 시스템: 비전 → VIO → RL 경로계획 → 제어 → 드론/시뮬 (파트 간 계약 파일)
  slide_2_control.png    제어 파트 파이프라인 (성형 5단 → 제어기 → 비행, 회신·사후 루프) — 단순화
  slide_3_validation.png 검증 파이프라인 (실행기·규모·지표·스펙 채점)
사용: cd control_seoungjin && python make_pipeline_slides.py
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figure", "00_pipeline")
plt.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

INK, MUTED = "#1f1f1f", "#666666"
PAL = {"vision": ("#E3F2FD", "#1E88E5"), "vio": ("#E8F5E9", "#43A047"), "rl": ("#FFF8E1", "#FB8C00"),
       "ctrl": ("#FDECEA", "#D32F2F"), "plant": ("#F3E5F5", "#8E24AA"), "gray": ("#F5F5F5", "#757575"),
       "shape": ("#E8F5E9", "#2E7D32"), "gate": ("#FFF3E0", "#EF6C00"), "post": ("#EDE7F6", "#5E35B1")}


def canvas(title, subtitle=None):
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16); ax.set_ylim(0, 9); ax.axis("off")
    ax.text(0.4, 8.55, title, fontsize=22, weight="bold", color=INK, va="center")
    if subtitle:
        ax.text(0.4, 8.05, subtitle, fontsize=12.5, color=MUTED, va="center")
    return fig, ax


def box(ax, x, y, w, h, title, sub=None, key="gray", fs=14, sfs=10.5, lw=2.0, dy=0.28):
    fc, ec = PAL[key]
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.18", fc=fc, ec=ec, lw=lw))
    if sub:
        ax.text(x + w / 2, y + h / 2 + dy, title, ha="center", va="center", fontsize=fs, weight="bold", color=INK)
        ax.text(x + w / 2, y + h / 2 - dy, sub, ha="center", va="center", fontsize=sfs, color=MUTED, linespacing=1.35)
    else:
        ax.text(x + w / 2, y + h / 2, title, ha="center", va="center", fontsize=fs, weight="bold", color=INK)
    return (x, y, w, h)


def harrow(ax, a, b, text=None, color="#333333", ls="-", lw=2.0, above=True):
    x0 = a[0] + a[2]; y0 = a[1] + a[3] / 2
    x1 = b[0]; y1 = b[1] + b[3] / 2
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>", mutation_scale=18, color=color, lw=lw, ls=ls))
    if text:
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 + (0.32 if above else -0.32), text, ha="center", va="center", fontsize=9.5, color=MUTED,
                bbox=dict(fc="white", ec="none", pad=1.5))


def varrow(ax, p0, p1, color="#333333", ls="-", lw=1.8, text=None, tx=0.15):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=16, color=color, lw=lw, ls=ls))
    if text:
        ax.text((p0[0] + p1[0]) / 2 + tx, (p0[1] + p1[1]) / 2, text, fontsize=9.5, color=color, va="center", ha="left")


def save(fig, name):
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"), dpi=180, facecolor="white")
    plt.close(fig)


# ───────────────────────────── slide 1: 전체 시스템 ─────────────────────────────
def pill(ax, cx, cy, text, color, fs=9.5):
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fs, color=color, linespacing=1.3,
            bbox=dict(boxstyle="round,pad=0.45,rounding_size=0.6", fc="white", ec=color, lw=1.4))


def slide_system():
    fig, ax = canvas("전체 시스템 파이프라인 — 카메라에서 드론까지",
                     "현수 하중 드론의 자율 창문 통과 · 4개 파트가 파일 계약(인터페이스 스펙)으로 결합 · 빨간 박스 = 제어 파트 (이 발표)")
    # 상단 5단 (크게)
    y, h, w = 4.55, 2.15, 2.85
    xs = [0.4, 3.45, 6.5, 9.55, 12.6]
    keys = ["gray", "vision", "vio", "rl", "ctrl"]
    titles = [("카메라 · IMU", "이미지 1280×720\nIMU 각속도·가속도\n(정수 ns 타임스탬프)"),
              ("비전 (길남)", "YOLO-pose 창문 4코너\nHSV 색 → 통과 순서\n(깊이는 안 준다)"),
              ("VIO (태민)", "OpenVINS 상태추정\n창문 3D 복원 (삼각측량)\nROS2 Jazzy"),
              ("경로계획 RL (윤호)", "Isaac Sim 학습\n정책 출력 = 웨이포인트\n(PID 아님)"),
              ("저수준 제어 (성진)", "궤적 성형 5단\n→ 캐스케이드 PID\n→ 모터 명령")]
    bxs = []
    for k, (t, sub), x in zip(keys, titles, xs):
        bxs.append(box(ax, x, y, w, h, t, sub, k, fs=15, sfs=10.5, lw=3.0 if k == "ctrl" else 2.0, dy=0.42))
    for a, b in zip(bxs[:-1], bxs[1:]):
        harrow(ax, a, b, lw=2.4)
    # 계약 파일 pill (화살표 위)
    pills = [("§5 window_detection\nJSON (픽셀 코너·색·순서)", PAL["vision"][1]),
             ("state_window_interface\n드론 상태 + 창문 3D 맵", PAL["vio"][1]),
             ("mission.json\nwaypoints · limits · dt", PAL["rl"][1])]
    for (t, c), a, b in zip(pills, bxs[1:-1], bxs[2:]):
        cx = (a[0] + a[2] + b[0]) / 2
        pill(ax, cx, y + h + 0.55, t, c, fs=9.2)
    pill(ax, (bxs[0][0] + bxs[0][2] + bxs[1][0]) / 2, y + h + 0.55, "raw 이미지\n+ IMU", PAL["gray"][1], fs=9.2)
    # 회신 (제어 → RL): 박스 아래 아크
    ax.add_patch(FancyArrowPatch((bxs[4][0] + 0.9, y), (bxs[3][0] + bxs[3][2] - 0.9, y), arrowstyle="-|>", mutation_scale=18, color=PAL["rl"][1], lw=2.0, ls="--",
                                 connectionstyle="arc3,rad=-0.55"))
    ax.text((bxs[3][0] + bxs[3][2] + bxs[4][0]) / 2 - 1.2, y - 0.85, "trajectory_report.json 회신\nverdict · adjustments · limits_budget · command_fidelity (RL 보상)",
            ha="center", va="top", fontsize=9.2, color=PAL["rl"][1], linespacing=1.3)
    # 플랜트 밴드
    yb, hb = 1.05, 1.55
    plants = [("Isaac Sim / Gazebo", "씬·데이터셋·RL 학습 환경\n(제어: isaacsim_* JSON 내보내기)"),
              ("MATLAB Simscape 모델", "FX450 + 현수 하중 1.8 Hz\n실비행 검증의 기준 플랜트"),
              ("실기 FX450 (예정)", "C++ 제어기 = MATLAB 골든 동일\n배터리 저장착으로 0 kg 상태 제거")]
    pxs = [3.45, 7.65, 11.85]; pw = 3.75
    pbs = [box(ax, x, yb, pw, hb, t, sub, "plant", fs=13.5, sfs=10) for x, (t, sub) in zip(pxs, plants)]
    xc = bxs[4][0] + bxs[4][2] / 2; ybus = yb + hb + 0.6
    ax.plot([xc, xc], [y - 1.15, ybus], color=PAL["ctrl"][1], lw=2.4)
    ax.plot([xc, xc], [y, y - 0.05], color=PAL["ctrl"][1], lw=2.4)
    ax.plot([pbs[0][0] + pw / 2, xc], [ybus, ybus], color=PAL["ctrl"][1], lw=2.4)
    for pb in pbs:
        cxp = pb[0] + pw / 2
        ax.annotate("", xy=(cxp, yb + hb), xytext=(cxp, ybus), arrowprops=dict(arrowstyle="-|>", color=PAL["ctrl"][1], lw=2.4, mutation_scale=18))
    pill(ax, xc, y - 0.6, "trajectory.mat / json\n(+ motor cmd)", PAL["ctrl"][1], fs=9.2)
    ax.text(0.4, yb + hb / 2 + 0.25, "플랜트 3종", fontsize=13, weight="bold", color=PAL["plant"][1], va="center")
    ax.text(0.4, yb + hb / 2 - 0.25, "같은 궤적 파일을\n세 곳이 소비", fontsize=10, color=MUTED, va="center", linespacing=1.3)
    # 각주
    ax.text(0.4, 0.4, "공통 규약: 좌표 world Z-up [m] · 코너 순서 TL→TR→BR→BL · 시간 = 센서/비전/VIO 정수 ns, 제어 JSON float s · 쿼터니언 순서는 인터페이스별(궤적 WXYZ) · GT 깊이/자세는 관측에 넣지 않음",
            fontsize=9.5, color=MUTED)
    save(fig, "slide_1_system")


# ───────────────────────────── slide 2: 제어 파트 ─────────────────────────────
def slide_control():
    fig, ax = canvas("제어 파트 파이프라인 — 미션 JSON에서 비행까지",
                     "traj_pipeline.py 한 진입점(동사 API) · 성형 5단이 '따라갈 수 있는 궤적'만 제어기에 넘기고, 제어기 게인은 손대지 않는다")
    # 입력 + 성형 5단
    y, h, w, gap = 5.75, 1.45, 2.15, 0.25
    inb = box(ax, 0.4, y, 2.4, h, "미션 JSON", "RL 경로계획 →\nwaypoints · limits · dt\n(+옵션: 프로파일·yaw·질량)", "rl", fs=13, sfs=9.5)
    x0 = 3.3
    steps = [("① 시간 부여", "7차 다항 최소시간\nfly-through", "shape"),
             ("② 스무더", "물리 포락선 v/a/j\n허용 한계 = 질량별 실측", "shape"),
             ("③ ZVD 셰이퍼", "짐 진자 1.8 Hz\n입력 성형 (잔류 4.4°→0.6°)", "shape"),
             ("④ 게이트", "v/a/j/snap 검사\nkeep-out 전 샘플", "gate"),
             ("⑤ yaw · 저장", "heading/look_at/scan\ntrajectory.mat/json", "gray")]
    bx = []
    for i, (t, s_, k) in enumerate(steps):
        bx.append(box(ax, x0 + i * (w + gap), y, w, h, t, s_, k, fs=13, sfs=9.5))
    harrow(ax, inb, bx[0])
    for i in range(len(bx) - 1):
        harrow(ax, bx[i], bx[i + 1])
    ax.text(x0, y + h + 0.28, "성형 5단 (traj_pipeline.py) — 초과 limits는 거부 대신 클램프·재시간화 → 회신으로 통지 (\"웬만하면 거부하지 않는다\")",
            fontsize=10.5, color=MUTED)
    # 궤적 → 위치 PID (폴리라인: ⑤ 아래로 → 왼쪽으로 → 위치 PID 위로)
    yc, hc = 3.15, 1.5
    xe = bx[-1][0] + w / 2; ymid = y - 0.45
    ax.plot([xe, xe, 1.85, 1.85], [y, ymid, ymid, yc + hc + 0.05], color="#333333", lw=2.0)
    ax.annotate("", xy=(1.85, yc + hc), xytext=(1.85, yc + hc + 0.06), arrowprops=dict(arrowstyle="-|>", color="#333333", lw=2.0, mutation_scale=18))
    ax.text(8.0, ymid + 0.14, "trajectory.mat / trajectory.json — 기준 위치·yaw 시계열 (dt 균일), 컨트롤러는 이것만 본다", fontsize=10, color=INK, ha="center", va="bottom")
    # 회신 (⑤ 오른쪽 아래)
    rep = box(ax, 13.6, yc, 2.0, hc, "회신 → RL", "trajectory_report.json\nverdict · adjustments\nlimits_budget · fidelity", "rl", fs=12, sfs=9, dy=0.38)
    ax.annotate("", xy=(rep[0] + rep[2] / 2, rep[1] + rep[3]), xytext=(rep[0] + rep[2] / 2, ymid), arrowprops=dict(arrowstyle="-|>", color=PAL["rl"][1], lw=1.8, mutation_scale=16))
    ax.plot([xe, rep[0] + rep[2] / 2], [ymid, ymid], color=PAL["rl"][1], lw=1.8, ls="--")
    # 제어기 캐스케이드
    c1 = box(ax, 0.4, yc, 2.9, hc, "위치 PID", "오차 클램프(posErrSat)\n→ 기울기 명령 (tilt ≤ 60°)", "ctrl", fs=13, sfs=9.5)
    c2 = box(ax, 3.6, yc, 2.9, hc, "자세 PID", "roll/pitch (음수 게인)\nkd 필터 · 출력 클램프", "ctrl", fs=13, sfs=9.5)
    c3 = box(ax, 6.8, yc, 2.9, hc, "yaw PID + 믹서", "FF 호버 트림 ∝ √총질량\n4모터 배분", "ctrl", fs=13, sfs=9.5)
    c4 = box(ax, 10.0, yc, 3.3, hc, "모터 루프 → 플랜트", "Simscape 6-DOF + 현수 하중\n(실기: C++ 동일 코드) → 비행 로그", "plant", fs=13, sfs=9.5)
    harrow(ax, c1, c2); harrow(ax, c2, c3); harrow(ax, c3, c4)
    ax.text(0.4, yc + hc + 0.28, "제어기 캐스케이드 (구운 모델 / C++ 이식) — 게인 = 적재 질량 스케줄(0 kg ↔ 1 kg 앵커 선형), 프로파일 precision / balanced / agile",
            fontsize=10.5, color=MUTED)
    # 사후 루프
    yp, hp = 0.85, 1.35
    f1 = box(ax, 0.4, yp, 3.4, hp, "feedback", "잔류 지터 f₀ 측정 → 셰이퍼 주파수 갱신\n(③에 반영)", "post", fs=12.5, sfs=9.5)
    f2 = box(ax, 4.1, yp, 3.4, hp, "counter_swing (2호기)", "스윙 FFT → 역위상 미세 오프셋\n(오프라인 ③ 뒤 → 실시간 댐퍼)", "post", fs=12.5, sfs=9.5)
    f3 = box(ax, 7.8, yp, 3.4, hp, "estimate", "질량·K_thrust·K_drag 회귀\n→ 게인 스케줄·허용 한계 입력", "post", fs=12.5, sfs=9.5)
    f4 = box(ax, 11.5, yp, 4.1, hp, "비상 감독자 (§9)", "정지 궤적·keep-out 회피·모드 관리\n우선순위 B > C > A-1 > A-2 · splice / emergency", "post", fs=12.5, sfs=9.5)
    ax.text(0.4, yp + hp + 0.28, "사후 루프 — 비행 로그로 다음 비행을 고친다 (게인은 그대로) · 로그 ↓", fontsize=10.5, color=MUTED)
    ax.annotate("", xy=(c4[0] + c4[2] / 2 + 0.6, yp + hp + 0.55), xytext=(c4[0] + c4[2] / 2 + 0.6, yc), arrowprops=dict(arrowstyle="-|>", color=PAL["post"][1], lw=1.6, ls="--", mutation_scale=16))
    save(fig, "slide_2_control")


# ───────────────────────────── slide 3: 검증 파이프라인 ─────────────────────────────
def slide_validation():
    fig, ax = canvas("검증 파이프라인 — 어떤 실행기로, 얼마나, 무엇을 재는가",
                     "구운 모델 무수정 실비행 ≈ 260편 (2026-08-18 하루) · 지표는 전부 PERFORMANCE_SPEC v0.2에 측정법이 있는 것만")
    # 왼쪽: 입력 종류
    y, h = 4.6, 1.35
    i1 = box(ax, 0.4, y + 1.55, 3.2, h, "미션 7편", "정지 배치 34.9 s · 플라이스루 · yaw 4모드\n공격 왕복 2 · 스텝", "rl", fs=13, sfs=9.5)
    i2 = box(ax, 0.4, y, 3.2, h, "배터리 시나리오", "호버 12 s · 토크 펄스 · 고도 스텝 · yaw 90°\n대각 2 m · 바람 5 m/s · 질량 스윕 7점", "rl", fs=13, sfs=9)
    i3 = box(ax, 0.4, y - 1.55, 3.2, h, "튜닝 격자", "0 kg 141점 · 2 kg 47점\n좌표하강 (단축 시뮬 6+10 s)", "rl", fs=13, sfs=9.5)
    # 중앙: 실행기
    e1 = box(ax, 4.5, y + 1.55, 3.6, h, "verify_pipeline.py", "미션 → 파이프라인 → run_traj_baked\n질량별 (--tag, UGRP_PKG_KG)", "ctrl", fs=13, sfs=9.5)
    e2 = box(ax, 4.5, y, 3.6, h, "perf_battery.m", "15+7 케이스 배터리\nPKG / SHAPER / FF env", "ctrl", fs=13, sfs=9.5)
    e3 = box(ax, 4.5, y - 1.55, 3.6, h, "tune_0kg / tune_2kg", "게인 축별 스윕 → CSV\n(save_system 금지, 메모리 수술)", "ctrl", fs=13, sfs=9.5)
    for a, b in ((i1, e1), (i2, e2), (i3, e3)):
        harrow(ax, a, b)
    # 플랜트
    pl = box(ax, 8.9, y - 1.55, 3.0, 4.45, "MATLAB Simscape\n구운 모델", "FX450 + 현수 하중\n0 / 0.25 / 0.5 / 0.75 /\n1 / 1.5 / 2 kg\n\n케이스당 7~100 s\n(i7-13650HX, R2026a)", "plant", fs=14, sfs=10, dy=0.95)
    for e in (e1, e2, e3):
        ax.add_patch(FancyArrowPatch((e[0] + e[2], e[1] + e[3] / 2), (pl[0], pl[1] + pl[3] / 2), arrowstyle="-|>", mutation_scale=16, color="#333333", lw=1.6))
    # 오른쪽: 지표/스펙/그림
    m1 = box(ax, 12.7, y + 1.55, 2.9, h, "지표 산출", "perf_metrics · perf_battery_plots\n추종 RMS·오버슈트·지터·복귀 시간", "post", fs=13, sfs=9.5)
    m2 = box(ax, 12.7, y, 2.9, h, "스펙 채점", "PERFORMANCE_SPEC v0.2 (1 kg)\n+ 0 kg 부록 · 항목별 합격/불합격", "gate", fs=13, sfs=9.5)
    m3 = box(ax, 12.7, y - 1.55, 2.9, h, "그림 · 문서", "figure/ 폴더 01~08\nPERFORMANCE.md · 능력 카드", "post", fs=13, sfs=9.5)
    for m in (m1, m2, m3):
        ax.add_patch(FancyArrowPatch((pl[0] + pl[2], pl[1] + pl[3] / 2), (m[0], m[1] + m[3] / 2), arrowstyle="-|>", mutation_scale=16, color="#333333", lw=1.6))
    # 아래: 독립 검증 2줄
    yb, hb = 0.7, 1.25
    g1 = box(ax, 0.4, yb, 5.0, hb, "골든 트레이스 (C++ ↔ MATLAB)", "같은 입력 → RMS ≤ 2 % FS, corr ≥ 0.99\n실기 코드 = 시뮬 코드 보증", "gray", fs=12.5, sfs=9.5)
    g2 = box(ax, 5.7, yb, 5.0, hb, "독립 6-DOF 적분기 (plant_sim.py)", "DYNAMICS.md 식 vs Simscape 가속 잔차 대조\n(상관 +0.92) — 식이 곧 시뮬레이터", "gray", fs=12.5, sfs=9.5)
    g3 = box(ax, 11.0, yb, 4.6, hb, "회귀 규칙", "게인 변경 = 미션 7편 + 배터리 전부 재실행\n하나라도 깨지면 탈락", "gate", fs=12.5, sfs=9.5)
    ax.text(0.4, yb + hb + 0.3, "독립 검증 · 회귀 규칙", fontsize=10.5, color=MUTED)
    save(fig, "slide_3_validation")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    slide_system(); slide_control(); slide_validation()
    print("->", OUT)
