"""지연 보상 MHE (이동 지평 추정) — 사용자 설계 2026-08-28, 구현 2026-08-29.

`delay_compensator.py` 의 되감기-재적분 판을 대체한다. 둘의 차이는
**모르는 가속도 바이어스를 같이 추정하느냐**다:

    되감기 판 : 저장해 둔 가속도가 맞다고 보고 그대로 재생한다.
    MHE  판 : 그 구간에서 바람/질량오차/항력처럼 모델이 틀린 몫을 **바이어스로**
              같이 추정하고, 그 바이어스가 다음 N 스텝의 싼 외삽을 정확하게 만든다.

사용자 말의 "N번에 한번 제대로 돌리고 나머지는 그 지표 기준으로" 에서 **그 지표가
바이어스**다. 고정 이득 알파-베타로는 이걸 못 한다.

## 왜 최적화가 아니라 최소제곱인가 (구현상 제일 중요한 관찰)

창 시작점의 상태를 (p0, v0, b) 로 두면 창 안의 어느 시각 t_k 에서

    p(t_k) = p0 + v0*D_k + 0.5*b*D_k^2 + S_k        D_k = t_k - t0

이고 S_k 는 **측정 가속도의 이중적분**(이미 아는 값)이다. 즉 예측 위치가
(p0, v0, b) 에 대해 **선형**이라, 이 문제는 미지수 3개짜리 가중 최소제곱이고
**3x3 정규방정식 한 번**으로 닫힌 해가 나온다. 반복 최적화도 솔버도 필요 없다.

이게 중요한 이유:
  - 1 kHz 루프에 올릴 수 있다 (부동소수 수십 번)
  - **결정론적**이다 — 반복 횟수가 입력에 따라 안 변한다. 실시간에서 이게 전부다
  - C++ 이식이 자명하다 (3x3 대칭 양정치 -> 촐레스키)

MHE 라 부르는 것은 창을 한 스텝씩 밀며 푸는 **이동 지평** 구조이기 때문이지
비선형 최적화를 돌려서가 아니다. 동역학이 선형이라 그 부분이 공짜로 떨어진다.

## N (무거운 해를 얼마나 자주 푸나) — 정확도 예산이지 계산 예산이 아니다

★ 2026-08-29 정정. 되감기 판의 규칙 N = ceil((tau/dt)/6) 은 **계산량 논리로만**
유도됐다 (되감기 비용이 tau/dt 에 비례하니 N 으로 나눠 상수로 묶는다). 결과가
5 ms -> N=1 / 100 ms -> N=17 인데, 정확도로 보면 **방향이 반대**다:

  - 지연이 클수록 외삽 구간(N*dt)이 길어지고 바이어스도 더 낡는다 = 오차가 커진다
  - 그런데 지연의 대가는 선형이 아니라 **절벽**이다. 같은 크기의 추정 오차가
    30 ms 에서는 추종 몇 cm 지만 60 ms 에서는 임무를 실패시킨다
    (돌풍 배율 30 ms 1.00 -> 40 ms 0.55 -> 60 ms 0.28 -> 80 ms 운용 불가,
     60 ms 에서 yaw 표류 23.8 도 = 창 통과 반FOV 이탈)
  - 즉 옛 규칙은 **제일 안 중요한 곳(5 ms)에 계산을 제일 많이 쓰고 생사가 갈리는
    곳(100 ms)에 제일 아낀다.**

여기서는 N 을 **외삽이 견딜 수 있는 시간**으로 잡는다. 바이어스 불확실도 db 로
외삽하면 위치 오차가 0.5*db*(N*dt)^2 로 자라므로

    N*dt <= sqrt(2 * accuracy_tol_m / db)

그리고 **db 는 최소제곱이 스스로 내놓는다** (정규방정식 역행렬의 대각). 즉 추정기가
"내 바이어스를 얼마나 믿을 수 있나" 를 재서 자기 실행 주기를 정한다.

계산량은 그 다음이다 — **목적이 아니라 제약**. 정확도가 요구하는 N 을 못 감당하면
그건 "이 지연에서는 예측기로 안 된다" 는 답이고, 조용히 나빠지는 것보다 낫다.
어느 쪽이 묶였는지는 n_reason 으로 보고한다 (accuracy / compute / measurement).

## 안전 규약 — 되감기 판과 동일 (센서가 닻이다)

사용자 지적: "센서값이 틀렸다 간주하는 것이라 조심해야 한다. 스펙 깎는 것은 성능
문제인데 이건 **안정성** 문제다." 그래서 네 가지를 그대로 가져온다 —
age_trust(과대보상 금지) / 드롭아웃 시한 / 혁신 게이트 / 이탈 한계.
둘이 물리적으로 말이 안 되게 어긋나면 **언제나 센서가 이긴다**.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

DT_MIN_S = 1e-6
N_MAX = 200            # 측정보다 드물게 돌면 측정이 와도 한참 안 쓰인다
EPS = 1e-12


@dataclass
class MheConfig:
    """축 하나의 MHE 설정.

    가중치는 **표준편차로** 받는다 (분산이나 정보행렬이 아니라). 튜닝하는 사람이
    "이 센서는 몇 cm 쯤 틀린다" 로 생각하지 "분산이 얼마" 로 생각하지 않기 때문이다.

    - meas_std       측정 1 표본의 위치 오차 [m]. VIO 면 cm 단위
    - bias_rw        바이어스가 창 길이 동안 얼마나 변할 수 있나 [m/s^2].
                     이게 사전(prior)의 느슨함이다. 크면 바이어스를 빨리 바꾸고,
                     작으면 이전 해를 오래 믿는다 = 웜스타트의 강도.
    - prior_p_std    위치/속도 사전. 창 시작 상태는 직전 해에서 물려받으므로
    - prior_v_std    보통 느슨하게 둔다 (측정이 결정하게)
    - accuracy_tol_m 싼 외삽이 흘려도 되는 위치 오차 [m]. **N 이 여기서 나온다**
    - compute_budget 제어 스텝당 감당 가능한 해 비용. 정확도가 요구하는 N 을 못
                     감당할 때만 묶인다 (그때 n_reason 이 compute 가 된다)
    """
    dt: float = 0.001
    enabled: bool = False           # ★ 기본 꺼짐 (이중 보상 방지 — 모듈 주석)
    horizon_s: float = 0.06         # 창 길이. 보통 지연에서 정한다
    meas_std: float = 0.02          # [m]
    bias_rw: float = 2.0            # [m/s^2] 창 하나 동안 허용 변화
    prior_p_std: float = 1.0        # [m]
    prior_v_std: float = 1.0        # [m/s]
    accuracy_tol_m: float = 0.005   # [m] — 외삽 허용 오차
    compute_budget: float = 6.0     # 스텝당 허용 비용 (되감기 판과 같은 눈금)
    every_n: int | None = None      # 지정하면 자동 규칙을 무시
    # ── 안전 (되감기 판과 동일 — 모듈 주석 참조) ──────────────────────
    age_trust: float = 0.7          # 보고된 나이의 이 비율만 보상. >1 금지
    dropout_s: float = 0.25         # 측정이 끊기면 추측항법 중단
    innov_max_m: float = 0.50       # 이 이상 벌어지면 센서로 스냅
    v_max: float = 3.0              # 이탈 한계 계산용 [m/s]
    dev_margin_m: float = 0.05      # 이탈 한계 여유 [m]

    def n_from(self, bias_std: float) -> tuple[int, str]:
        """바이어스 불확실도 -> (N, 묶은 이유).

        정확도: 0.5*db*(N*dt)^2 <= tol  ->  N <= sqrt(2*tol/db)/dt
        계산  : 해 비용은 창 길이에 거의 무관(웜스타트)하지만 0 은 아니므로,
                horizon/dt 스텝을 예산으로 나눈 값을 **하한**으로 둔다.
        측정  : N_MAX 상한. 측정보다 드물게 돌 이유가 없다.

        db 가 0 에 가까우면(바이어스를 아주 잘 안다) 정확도 상한이 무한대가 되므로
        그때는 계산 쪽이 묶는다 — 그게 맞다. 잘 아는 값을 자주 다시 풀 이유가 없다.
        """
        if self.every_n is not None:
            return max(1, int(self.every_n)), "fixed"
        dt = max(self.dt, DT_MIN_S)
        db = max(float(bias_std), EPS)
        n_acc = math.sqrt(2.0 * max(self.accuracy_tol_m, 0.0) / db) / dt
        n_cmp = (self.horizon_s / dt) / max(self.compute_budget, EPS)
        # 정확도가 허락하는 것보다 더 자주 돌 이유는 없다. 둘 중 **작은 쪽**을
        # 고르되, 어느 쪽이 묶었는지를 남긴다 — 계산이 묶었으면 그건 "이 지연에서는
        # 정확도 요구를 못 맞춘다" 는 신호라 상위가 알아야 한다.
        if n_cmp > n_acc:
            n, why = n_cmp, "compute"      # 계산 때문에 정확도를 못 지킨다
        else:
            n, why = n_acc, "accuracy"
        n_i = max(1, int(math.floor(n)))
        if n_i >= N_MAX:
            return N_MAX, "measurement"
        return n_i, why


def _solve3(A: list, b: list) -> tuple:
    """3x3 대칭 양정치 -> 촐레스키. 해와 역행렬 대각(= 분산)을 같이 낸다.

    numpy 를 안 쓴다. 이 함수는 그대로 C++ 로 간다 — 1 kHz 루프에 올릴 물건이라
    할당도 예외도 없어야 하고, 반복 횟수가 입력에 안 변해야 한다 (결정론).
    대칭 양정치가 보장되는 이유는 A 가 정규방정식 J^T W J + 사전 이기 때문이다.

    반환: (x, var_diag). 분해가 실패하면(수치적으로 특이) (None, None).
    """
    a = [row[:] for row in A]
    Lc = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(i + 1):
            ssum = a[i][j] - sum(Lc[i][k] * Lc[j][k] for k in range(j))
            if i == j:
                if ssum <= EPS:
                    return None, None      # 관측이 부족하다 — 사전이 약한 것
                Lc[i][i] = math.sqrt(ssum)
            else:
                Lc[i][j] = ssum / Lc[j][j]
    # 전진/후진 대입
    y = [0.0] * 3
    for i in range(3):
        y[i] = (b[i] - sum(Lc[i][k] * y[k] for k in range(i))) / Lc[i][i]
    x = [0.0] * 3
    for i in (2, 1, 0):
        x[i] = (y[i] - sum(Lc[k][i] * x[k] for k in range(i + 1, 3))) / Lc[i][i]
    # 역행렬 대각만 필요하다 (각 성분의 분산). Linv 를 구해 열 노름 제곱을 쓴다.
    Li = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        Li[i][i] = 1.0 / Lc[i][i]
        for j in range(i):
            Li[i][j] = -sum(Lc[i][k] * Li[k][j] for k in range(j, i)) / Lc[i][i]
    var = [sum(Li[k][i] ** 2 for k in range(i, 3)) for i in range(3)]
    return x, var


@dataclass
class _Sample:
    """창 안의 한 스텝. 가속도는 매 스텝, 측정은 올 때만."""
    t: float
    accel: float        # 모델이 아는 가속도 (자세에서 환산) [m/s^2]
    meas: float | None = None   # 그 시각으로 되감은 측정 위치 [m]


@dataclass
class AxisMhe:
    """축 하나. 창을 밀며 (p0, v0, b) 를 풀고, 사이 구간은 그 b 로 외삽한다."""

    cfg: MheConfig = field(default_factory=MheConfig)
    _hist: list = field(default_factory=list, init=False)
    _t: float = field(default=0.0, init=False)          # 현재 시각
    _p0: float = field(default=0.0, init=False)         # 창 시작 상태
    _v0: float = field(default=0.0, init=False)
    _b: float = field(default=0.0, init=False)          # 가속도 바이어스
    _t0: float = field(default=0.0, init=False)
    _bias_std: float = field(default=0.0, init=False)
    _n: int = field(default=1, init=False)
    _n_reason: str = field(default="init", init=False)
    _since_solve: int = field(default=0, init=False)
    _last_meas: float = field(default=0.0, init=False)
    _last_meas_t: float = field(default=0.0, init=False)      # **도착** 시각 (드롭아웃용)
    _last_meas_vt: float = field(default=0.0, init=False)     # **유효** 시각 (이탈 한계용)
    _have_meas: bool = field(default=False, init=False)
    _fault: str = field(default="", init=False)
    _solves: int = field(default=0, init=False)
    _snaps: int = field(default=0, init=False)

    def reset(self, position: float = 0.0, velocity: float = 0.0) -> None:
        self._hist.clear()
        self._t = self._t0 = 0.0
        self._p0, self._v0, self._b = position, velocity, 0.0
        self._bias_std = self.cfg.bias_rw
        self._n, self._n_reason, self._since_solve = 1, "init", 0
        self._last_meas, self._last_meas_t, self._last_meas_vt = position, 0.0, 0.0
        self._have_meas = False
        self._fault = ""
        self._solves = self._snaps = 0

    # ── 예측: 창 시작 상태 + 그 뒤 가속도 이중적분 ────────────────────

    def _integrate(self, upto_t: float) -> tuple:
        """(S, dS) = t0 부터 upto_t 까지 **측정 가속도만의** 이중적분과 적분.

        바이어스 몫은 여기 안 들어간다 — 그건 미지수라 설계행렬 쪽으로 간다.
        사다리꼴이 아니라 전진 오일러다: 창 안에서 가속도는 스텝 상수로 들어오고,
        사다리꼴로 바꾸면 C++ 이식 때 한 스텝 어긋나기 쉬운 것에 비해 이득이 없다.
        """
        s = 0.0; v = 0.0
        for k in range(len(self._hist) - 1):
            a, b_ = self._hist[k], self._hist[k + 1]
            if a.t >= upto_t:
                break
            h = min(b_.t, upto_t) - a.t
            if h <= 0.0:
                continue
            s += v * h + 0.5 * a.accel * h * h
            v += a.accel * h
        return s, v

    def predict_at(self, t: float) -> float:
        """창 시작 상태를 t 로 전파. p = p0 + v0*D + 0.5*b*D^2 + S(t)"""
        d = t - self._t0
        s, _ = self._integrate(t)
        return self._p0 + self._v0 * d + 0.5 * self._b * d * d + s

    def velocity_at(self, t: float) -> float:
        d = t - self._t0
        _, dv = self._integrate(t)
        return self._v0 + self._b * d + dv

    # ── 무거운 쪽: 3x3 정규방정식 ──────────────────────────────────────

    def solve(self) -> bool:
        """창 안의 측정으로 (p0, v0, b) 를 다시 푼다. 성공하면 True.

        웜스타트는 **사전(prior)** 으로 들어간다 — 직전 해를 중심으로, bias_rw 만큼의
        느슨함을 준다. 이게 MHE 의 arrival cost 에 해당하고, 측정이 적을 때 문제를
        정칙화한다 (측정 1개로도 해가 나온다 — 사전이 나머지를 잡아준다).
        """
        cfg = self.cfg
        if not self._hist:
            return False
        # 사전: 직전 해 중심. 위치/속도는 느슨하게, 바이어스는 bias_rw 로.
        wp = 1.0 / max(cfg.prior_p_std, EPS) ** 2
        wv = 1.0 / max(cfg.prior_v_std, EPS) ** 2
        wb = 1.0 / max(cfg.bias_rw, EPS) ** 2
        M = [[wp, 0.0, 0.0], [0.0, wv, 0.0], [0.0, 0.0, wb]]
        r = [wp * self._p0, wv * self._v0, wb * self._b]
        wm = 1.0 / max(cfg.meas_std, EPS) ** 2
        nmeas = 0
        for smp in self._hist:
            if smp.meas is None:
                continue
            d = smp.t - self._t0
            s, _ = self._integrate(smp.t)
            # 설계행렬 한 줄: [1, d, 0.5*d^2], 목표: z - S
            g = (1.0, d, 0.5 * d * d)
            y = smp.meas - s
            for i in range(3):
                for j in range(3):
                    M[i][j] += wm * g[i] * g[j]
                r[i] += wm * g[i] * y
            nmeas += 1
        if nmeas == 0:
            return False
        x, var = _solve3(M, r)
        if x is None:
            return False
        self._p0, self._v0, self._b = x
        self._bias_std = math.sqrt(max(var[2], 0.0))
        self._n, self._n_reason = cfg.n_from(self._bias_std)
        self._solves += 1
        return True

    # ── 매 스텝 ────────────────────────────────────────────────────────

    def tick(self, accel: float, meas: float | None = None,
             meas_age_s: float = 0.0, dt: float | None = None) -> float:
        """한 스텝 전진. 반환은 **현재 시각의 추정 위치**.

        무거운 solve() 는 N 스텝마다만 돈다. 나머지 스텝은 마지막 해의 바이어스로
        전파만 한다 — 곱셈 몇 번이라 사실상 공짜다.

        ★ 사이 구간에 출력을 붙잡아 두면(ZOH) 안 된다. 그러면 N*dt 만큼의 **새
          지연**이 생겨서, 지연을 줄이자고 만든 물건이 지연을 만든다. 그리고
          **측정은 사이 구간에도 계속 받는다** (창에 쌓아둔다) — 다음 solve 가
          그걸 다 쓴다. 안 받으면 그만큼 정보를 버리는 것이다.
        """
        cfg = self.cfg
        h = float(dt) if dt is not None else cfg.dt
        h = max(h, DT_MIN_S)
        self._t += h
        self._hist.append(_Sample(t=self._t, accel=float(accel)))

        if meas is not None:
            self._ingest(float(meas), float(meas_age_s))

        # 창 밖으로 나간 것은 버린다. 창이 밀리면 시작 상태도 같이 밀어야 한다 —
        # 이게 "이동 지평" 의 이동이다. 새 t0 의 상태는 옛 해를 거기까지 전파한 값.
        cut = self._t - max(cfg.horizon_s, h)
        if self._hist and self._hist[0].t < cut:
            new_p = self.predict_at(cut)
            new_v = self.velocity_at(cut)
            self._hist = [s for s in self._hist if s.t >= cut]
            self._t0, self._p0, self._v0 = cut, new_p, new_v

        self._since_solve += 1
        if self._since_solve >= self._n:
            self._since_solve = 0
            self.solve()

        return self._guarded_output()

    def _ingest(self, meas: float, age_s: float) -> None:
        """늦게 온 측정을 **그 측정이 유효했던 시각**에 꽂는다.

        age_trust 를 여기서 쓴다 (사용자 지적: 과대보상은 성능이 아니라 안정성 문제).
        보고된 나이보다 **덜** 되감는다 — 과소보상의 손해는 지연이 일부 남는 것이고,
        과대보상의 손해는 추정을 실제보다 앞에 놓아 양의 되먹임이 되는 것이다.
        """
        cfg = self.cfg
        age = max(0.0, min(float(age_s), cfg.horizon_s)) * min(max(cfg.age_trust, 0.0), 1.0)
        t_meas = self._t - age
        # 혁신 게이트: 예측과 측정이 물리적으로 말이 안 되게 벌어지면 보정을 버리고
        # 센서로 스냅한다. 센서가 닻이고 모델은 사이를 메우는 도구다.
        innov = meas - self.predict_at(t_meas)
        if abs(innov) > cfg.innov_max_m:
            self._snap(meas, t_meas)
            return
        # 가장 가까운 스텝에 붙인다 (창 안에 없으면 버린다 — 너무 낡은 측정)
        best = None; bd = None
        for smp in self._hist:
            d = abs(smp.t - t_meas)
            if bd is None or d < bd:
                best, bd = smp, d
        if best is not None:
            best.meas = meas
        self._last_meas, self._last_meas_t, self._last_meas_vt = meas, self._t, t_meas
        self._have_meas = True
        self._fault = ""

    def _snap(self, meas: float, t_meas: float) -> None:
        """창을 버리고 측정에서 다시 시작한다. 되감기 판과 같은 규약."""
        self._hist = [s for s in self._hist if s.t >= t_meas]
        self._t0 = t_meas
        self._p0, self._b = meas, 0.0
        self._bias_std = self.cfg.bias_rw
        self._last_meas, self._last_meas_t, self._last_meas_vt = meas, self._t, t_meas
        self._have_meas = True
        self._snaps += 1
        self._fault = "innov"

    def anchored_output(self) -> float:
        """★ 출력은 **최신 측정에 얹어** 만든다 (사용자 설계, 2026-08-29 구조 수정).

        원래 설계는 "센서값 + 그 지연 동안의 변화" 였는데, 처음 구현은 창 시작점
        상태를 전파했다 (`predict_at`). 차이가 결정적이다:

            설계 : 출력 = 최신_측정 + (그 측정의 나이만큼의 변화)   닻 = 센서
            구판 : 출력 = 재구성된 창 시작 상태 + 창 전체 적분      닻 = 적합 결과

        구판은 (a) 모델 오차를 창 길이(0.25 s)만큼 적분하고 — 지연(0.06 s)의 4배 —
        (b) 해가 갱신될 때마다 시작점이 통째로 바뀌어 출력이 **점프**한다. 그 점프는
        RMS 로는 안 보이지만(RMS 는 '정확하지만 튀는' 신호를 잘 친다) 폐루프에서는
        위치 제어기의 D 항이 미분해 스파이크가 된다.

        역할을 가른다: **긴 창은 바이어스 관측 전용**(H >~ sqrt(2σ/b) 때문에 길어야
        한다), 그 바이어스를 **최신 센서에 얹어 나이만큼만** 적분한다. b 는
        0.5*b*age^2 로만 들어오므로 해가 바뀌어도 출력이 거의 안 움직인다.
        """
        if not self._have_meas:
            return self.predict_at(self._t)
        age = max(0.0, self._t - self._last_meas_vt)
        v = self.velocity_at(self._last_meas_vt)
        s = 0.0
        i0 = self._index_at(self._last_meas_vt)
        for k in range(i0, len(self._hist) - 1):
            h = self._hist[k + 1].t - self._hist[k].t
            a = self._hist[k].accel + self._b
            s += v * h + 0.5 * a * h * h
            v += a * h
        return self._last_meas + s

    def _index_at(self, t: float) -> int:
        for i, smp in enumerate(self._hist):
            if smp.t >= t:
                return i
        return max(0, len(self._hist) - 1)

    def _guarded_output(self) -> float:
        """추정을 내보내기 전에 두 가지를 더 본다 — 드롭아웃과 이탈 한계."""
        cfg = self.cfg
        p = self.anchored_output()
        if not self._have_meas:
            return p
        gap = self._t - self._last_meas_t          # 마지막 **도착** 이후
        # 드롭아웃: 측정이 끊기면 추측항법을 멈추고 마지막 측정에 눌러앉는다.
        # 없으면 VIO 가 죽어도 혼자 적분하며 그럴듯한 거짓말을 먹인다 = 최악 실패모드.
        # 여기는 '도착' 기준이 맞다 — 링크가 죽었는지를 보는 것이므로.
        if gap > cfg.dropout_s:
            self._fault = "dropout"
            return self._last_meas
        # 이탈 한계: 예측이 물리적으로 불가능한 거리만큼 가지 못하게 자른다.
        # ★ 여기는 '유효' 기준이어야 한다. 마지막 측정은 도착했을 때 이미 age 만큼
        #   낡아 있었으므로, 그 값이 참이던 시점부터 지금까지 흐른 시간은
        #   gap + age 다. gap 만 쓰면 한계가 age 만큼 짧아져 정상 예측까지 자른다 —
        #   실제로 그렇게 짜서 2499 스텝 중 2478 번이 잘렸고, age_trust 를 바꿔도
        #   출력이 안 변하는 것으로 드러났다 (2026-08-29).
        elapsed = self._t - self._last_meas_vt
        lim = cfg.v_max * elapsed + cfg.dev_margin_m
        dev = p - self._last_meas
        if abs(dev) > lim:
            self._fault = "dev"
            return self._last_meas + math.copysign(lim, dev)
        return p

    # ── 보고 ───────────────────────────────────────────────────────────

    @property
    def bias(self) -> float:
        return self._b

    @property
    def bias_std(self) -> float:
        return self._bias_std

    @property
    def every_n(self) -> int:
        return self._n

    @property
    def n_reason(self) -> str:
        """N 을 무엇이 묶었나 — accuracy / compute / measurement / fixed.

        **compute 가 나오면 그건 경고다**: 정확도가 요구하는 만큼 자주 못 푼다는 뜻이고,
        그 지연에서는 예측기가 요구 정확도를 못 맞춘다. 상위가 알아야 한다.
        """
        return self._n_reason

    @property
    def healthy(self) -> bool:
        return self._fault == ""

    @property
    def fault(self) -> str:
        return self._fault


@dataclass
class MheCompensator:
    """3축 묶음. 축끼리 결합이 없다 — 위치 채널은 서로 독립이라 그게 맞다.

    자세 결합은 `accel_xyz` 를 만드는 쪽(자세 -> 횡가속 환산)에서 이미 들어간다.
    `delay_compensator.lateral_accel_from_attitude` 를 그대로 쓸 수 있다.
    """
    cfg: MheConfig = field(default_factory=MheConfig)
    axes: list = field(default_factory=list, init=False)

    def __post_init__(self):
        self.axes = [AxisMhe(cfg=self.cfg) for _ in range(3)]

    def reset(self, pos=(0.0, 0.0, 0.0)) -> None:
        for a, p in zip(self.axes, pos):
            a.reset(p)

    def tick(self, accel_xyz, meas_xyz=None, meas_age_s: float = 0.0,
             dt: float | None = None):
        out = []
        for k, ax in enumerate(self.axes):
            m = None if meas_xyz is None else float(meas_xyz[k])
            out.append(ax.tick(float(accel_xyz[k]), m, meas_age_s, dt))
        return tuple(out)

    @property
    def bias(self):
        return tuple(a.bias for a in self.axes)

    @property
    def every_n(self) -> int:
        """세 축 중 **가장 자주** 돌아야 하는 쪽을 따른다 (보수적)."""
        return min(a.every_n for a in self.axes)

    @property
    def n_reason(self) -> str:
        """어느 하나라도 compute 에 묶였으면 그걸 보고한다 — 경고가 묻히면 안 된다."""
        rs = [a.n_reason for a in self.axes]
        for w in ("compute", "measurement", "accuracy"):
            if w in rs:
                return w
        return rs[0]

    @property
    def healthy(self) -> bool:
        return all(a.healthy for a in self.axes)

    @property
    def fault(self) -> str:
        for a in self.axes:
            if a.fault:
                return a.fault
        return ""

    def snapshot(self) -> dict:
        """상위 보고용. `capability.json` 에 실을 때 이 모양으로 낸다."""
        return {
            "enabled": bool(self.cfg.enabled),
            "every_n": self.every_n,
            "n_reason": self.n_reason,
            "bias_mps2": [round(b, 5) for b in self.bias],
            "bias_std_mps2": [round(a.bias_std, 5) for a in self.axes],
            "healthy": self.healthy,
            "fault": self.fault,
        }
