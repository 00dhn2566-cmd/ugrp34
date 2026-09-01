"""식을 PPT 삽입용 PNG로 렌더링.

LaTeX 설치 없이 matplotlib mathtext 로 그린다. 투명 배경, 300 dpi.
행렬 환경(bmatrix)은 mathtext가 지원하지 않으므로 성분/전치 표기로 쓴다
(발표용으로는 오히려 읽기 쉽다).

출력: output/equations/*.png

사용:
    python make_equation_images.py
"""

from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = "output/equations"

# 제목용 한글 폰트
for cand in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
    try:
        matplotlib.font_manager.findfont(cand, fallback_to_default=False)
        plt.rcParams["font.family"] = cand
        break
    except Exception:
        continue
plt.rcParams["mathtext.fontset"] = "cm"      # Computer Modern (논문 느낌)
plt.rcParams["axes.unicode_minus"] = False


# (파일명, 제목, [식 줄들], 글자크기)
EQUATIONS = [
    ("01_translation", "병진 운동방정식", [
        r"$m\,\dot{v}_W \;=\; R(\eta)\,[\,0,\;0,\;\Sigma T_i\,]^{T}"
        r"\;+\;[\,0,\;0,\;-mg\,]^{T}\;+\;F_{drag}$",
    ], 26),

    ("02_rotation", "회전 운동방정식", [
        r"$I\,\dot{\omega} \;+\; \omega \times (I\,\omega)"
        r"\;=\; \tau_{prop} \;+\; \tau_{drag}$",
    ], 26),

    ("03_mixer", "프로펠러 토크 (믹서)", [
        r"$\tau_{roll}\;=\;\ell\,C_t\,\rho\,D^4\,"
        r"(-n_1^2 + n_2^2 - n_3^2 + n_4^2)$",
        r"$\tau_{pitch}\;=\;\ell\,C_t\,\rho\,D^4\,"
        r"(+n_1^2 + n_2^2 - n_3^2 - n_4^2)$",
        r"$\tau_{yaw}\;=\;\quad C_q\,\rho\,D^5\,"
        r"(-n_1^2 + n_2^2 + n_3^2 - n_4^2)$",
    ], 22),

    ("04_propeller", "프로펠러 추력·반토크", [
        r"$T_i \;=\; C_t\,\rho\,n_i^2\,D^4$",
        r"$Q_i \;=\; C_q\,\rho\,n_i^2\,D^5$",
    ], 26),

    ("05_motor", "모터 회전 동역학", [
        r"$J\,\dot{n}_i \;=\; \tau_i \;-\; Q_i \;-\; b\,n_i$",
    ], 28),

    ("05b_motor_full", "모터 — 명령에서 추력까지", [
        r"$V \;=\; duty \times V_{batt}$",
        r"$\tau_i \;=\; \min\left(\,|cmd_i|\,\tau_{max},\;\; P_{max}/\omega_i\,\right)$",
        r"$J\,\dot{n}_i \;=\; \tau_i \;-\; C_q\,\rho\,n_i^2\,D^5 \;-\; b\,n_i$",
        r"$T_i \;=\; C_t\,\rho\,n_i^2\,D^4$",
    ], 22),

    ("06_drag", "공력 항력 (축별)", [
        r"$f_j \;=\; -\,\mathrm{sign}(v_j)\;\frac{\rho}{2}\;A_j\,C_{d,j}\;v_j^2$",
        r"$\tau_j \;=\; -\,\mathrm{sign}(\omega_j)\;\frac{\rho}{2}\;"
        r"A_{rot,j}\,C_{d,rot,j}\;\omega_j^2$",
    ], 24),

    ("07_attitude", "자세 (쿼터니언 적분)", [
        r"$\dot{q} \;=\; \frac{1}{2}\,q \otimes [\,0,\;\omega\,]$",
        r"$\theta = \arcsin(-R_{31}),\quad"
        r"\phi = \mathrm{atan2}(R_{32},R_{33}),\quad"
        r"\psi = \mathrm{atan2}(R_{21},R_{11})$",
    ], 21),

    ("08_payload", "짐에 따른 물성 (강체 부착)", [
        r"$m(m_p) \;=\; m_{drone} + m_p$",
        r"$z_{cg}(m_p) \;=\; \frac{m_{ch}z_{ch} + m_{rot}z_{rot} + m_p z_p}{m(m_p)}$",
        r"$I_{xx}(m_p) \;=\; I_{ch,x} + m_{ch}(z_{ch}-z_{cg})^2"
        r" + m_{rot}r^2 + m_{rot}(z_{rot}-z_{cg})^2$",
        r"$\qquad\qquad\quad +\; \frac{m_p}{12}(s_y^2+s_z^2) + m_p(z_p-z_{cg})^2$",
    ], 19),

    ("09_hover", "호버 평형", [
        r"$n_{hover}(m_p) \;=\; \sqrt{\frac{m(m_p)\,g}{4\,C_t\,\rho\,D^4}}$",
    ], 26),

    ("10_full", "전체 (요약)", [
        r"$m\,\dot{v}_W = R(\eta)[0,0,\Sigma T_i]^T + [0,0,-mg]^T + F_{drag}$",
        r"$I\,\dot{\omega} + \omega \times (I\omega) = \tau_{prop} + \tau_{drag}$",
        r"$\dot{q} = \frac{1}{2}\,q \otimes [0,\omega]$",
        r"$J\,\dot{n}_i = \tau_i - Q_i - b\,n_i$",
    ], 20),
]


def render(name: str, title: str, lines: list[str], size: int,
           with_title: bool = True) -> str:
    n = len(lines)
    h = 0.62 * n + (0.55 if with_title else 0.15)
    w = max(4.0, 0.052 * size * max(len(s) for s in lines) ** 0.62)
    fig = plt.figure(figsize=(w, h))

    y = 1.0 - (0.34 if with_title else 0.10)
    if with_title:
        fig.text(0.5, 0.955, title, ha="center", va="top",
                 fontsize=int(size * 0.62), color="#333333")
    step = (y - 0.06) / max(n, 1)
    for i, s in enumerate(lines):
        fig.text(0.5, y - step * (i + 0.35), s, ha="center", va="center",
                 fontsize=size, color="black")

    path = f"{OUT}/{name}.png"
    fig.savefig(path, dpi=300, transparent=True, bbox_inches="tight",
                pad_inches=0.15)
    plt.close(fig)
    return path


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    made, failed = [], []
    for name, title, lines, size in EQUATIONS:
        try:
            render(name, title, lines, size)
            made.append(name)
        except Exception as e:
            failed.append((name, str(e)[:80]))
    # 제목 없는 판도 같이 (슬라이드에 제목이 이미 있을 때)
    for name, title, lines, size in EQUATIONS:
        try:
            render(name + "_bare", title, lines, size, with_title=False)
        except Exception:
            pass

    print(f"생성 {len(made)}종 (각각 제목판 + _bare 판) -> {OUT}/")
    for m in made:
        print(f"  {m}")
    if failed:
        print("\n실패:")
        for f, e in failed:
            print(f"  {f}: {e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
