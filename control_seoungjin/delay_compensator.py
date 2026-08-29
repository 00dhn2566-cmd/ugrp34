"""측정 지연 보상 — 늦게 온 위치를 빠른 채널로 현재까지 밀어 준다 (스미스 예측기).

2026-08-28. 사용자 질문 "시간지연 때문에 진동하는 것은 어쩔 수 없는건가",
"그거 하려고 하면 mpc 해야 하는거 아님?" 에 대한 답의 세 번째 길.

## 왜 이게 필요한가

지연은 위상 여유를 먹는다. 되찾는 길은 셋인데 앞의 둘은 값을 치른다:

  ① 지연을 줄인다        — 가장 확실. 펌웨어 요구사항 30 ms 가 이 길이다.
  ② 루프 대역을 낮춘다   — 외란 복귀가 느려진다. 0 kg 이 그 대가를 치르고 있다
                           (게인이 물러서 160 ms 까지 살지만 외란 복귀가 9 s).
  ③ **지연을 보상한다**  — 대역을 안 깎고 위상을 되찾는 유일한 길. 이 파일.

MPC 가 아니다. MPC 는 매 스텝 최적화를 푸는 물건이라 1 kHz 자세 루프에 안 올라가고,
미래 경로는 이미 상위 계획기가 준다. **상수이고 크기를 아는 지연**에는 예측기가 맞다.

## 무엇으로 예측하나 — 빠른 채널이 느린 채널의 구멍을 메운다

이 기체에는 지연이 다른 두 채널이 있다 (PERFORMANCE §8c T2/T3):

  자세(IMU)  1 kHz, 경로 지연 ~5 ms   ← 빠름
  위치(VIO)  30 Hz,  경로 지연 20~80 ms ← 느림

위치가 τ 만큼 늦었다는 것은 "τ 동안 위치 정보가 없었다" 는 뜻이지 **"τ 동안 아무
정보도 없었다"** 는 뜻이 아니다. 그 구간의 가속도는 자세에서 나온다. 그래서
늦게 온 위치를 시작점으로 삼고, 그 뒤 구간을 가속도로 적분해 현재로 끌어온다.

이건 새로운 발상이 아니라 MSCKF 계열 VIO 가 안에서 하는 일과 같다 (IMU 전파).
**그래서 켜기 전에 반드시 확인할 것** — 추정기가 이미 전파해서 내보내고 있으면
여기서 또 하면 **이중 보상**이다. 그 경우 오차가 두 배로 들어간다.
`enabled` 기본값이 False 인 이유다.

## 정직한 한계

- 모델 오차가 **새로운 오차로 들어온다.** 위치 오차는 가속도 오차 × τ²/2 로 커진다.
  τ 가 크면 보상이 이득보다 해가 된다 -> `max_age_s` 로 자른다.
- 지연 크기를 모르면 못 쓴다. 여기서는 `measAgeS`(INTERFACE_SPEC §8c T8) 가
  그 값을 준다. 그게 틀리면 보상도 틀린다.
- **검증 전에는 켜지 말 것.** 골든 트레이스는 이 기능이 꺼진 상태로 잡혀 있고,
  `enabled=False` 면 이 모듈은 입력을 그대로 돌려주므로 항등이다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# 지연 보상을 시도할 최대 나이 [s]. 이보다 낡으면 보상을 포기하고 측정을 그대로 쓴다.
# 근거: 위치 예측 오차 ~ (가속도 오차)·τ²/2. 가속도 오차를 0.5 m/s² 로 잡으면
# τ=0.12 s 에서 3.6 mm 인데, 이건 추종 예산(1 kg 4 cm)의 9% 다. 그 위로는
# 보상이 벌어들이는 것보다 집어넣는 오차가 빨리 커진다.
MAX_AGE_S = 0.12

# 예측 스텝 사이의 dt 허용 범위 [s] — 스케줄러 지터/정지에 대한 방어.
# `SwingDamper` 규약(§8c T10)과 같은 취지: 이상한 dt 로 적분하면 상태가 튄다.
DT_MIN_S, DT_MAX_S = 1e-4, 0.05

# 자동 N 의 상한 = 측정 주기 (30 Hz 기준 33 스텝).
EVERY_N_MAX = 33


@dataclass
class PredictorConfig:
    """축 하나의 예측기 설정.

    `obs_w` 는 **관측기 대역**이지 제어 이득이 아니다. 늦게 온 측정과 그 시각의
    예측값 사이 차이(혁신)를 현재 상태에 얼마나 반영할지를 정한다. 크면 측정을
    빨리 따라가지만 측정 잡음이 그대로 들어오고, 작으면 모델을 오래 믿는다.

    보정은 **알파-베타 추적기** 형태로 넣는다. 측정 간격 T 에 대해
        극점 b = exp(-obs_w·T),  alpha = 1 - b²,  beta = (1 - b)²
        p += alpha·innov,  v += (beta/T)·innov
    beta 를 T 로 나누는 것이 핵심이다 — 혁신은 [m] 이고 속도는 [m/s] 라, 안 나누면
    차원이 안 맞아 측정 주기가 바뀔 때마다 실효 이득이 달라진다.

    w=12 rad/s (≈1.9 Hz) 는 위치 루프 대역보다 충분히 빠르면서 VIO 잡음보다는
    느린 자리다. **실측으로 조정할 것** — 지금 값은 유도값이지 실측이 아니다.

    ## every_n — 예측기를 매 스텝 돌리지 않는 이유 (사용자 지시 2026-08-28)

    "이거 돌리는 것도 지연이니까." 맞다. 지연을 줄이자고 넣은 물건이 제어 주기를
    잡아먹으면 손익이 뒤집힌다. 그래서 무거운 부분(이력 탐색 + 보정)은 N 스텝마다
    한 번만 돈다. 1 kHz 기준 `every_n=10` 이면 **100 Hz** 인데, 이건
    `docs/FIRMWARE_REQUIREMENTS.md` 의 위치 루프 주기와 같다 — 예측기의 출력을
    쓰는 쪽이 그 루프이므로 그보다 빨리 돌 이유가 없다.
    (같은 취지의 선례: `LatencyTracker` 를 1 kHz 로 돌리다 ~5 Hz 로 데시메이션한 건
     08-23 이다. 그때는 EMA 시정수가 표본 수 기준이라 1 kHz 에서 의미가 달라졌다.)

    ★ 함정 하나. 실행 사이에 출력을 **붙잡아 두면(ZOH) 안 된다.** 그러면 최대
      N·dt 만큼의 새 지연이 생겨서, 지연을 줄이자고 만든 물건이 지연을 만든다.
      사이 구간은 마지막 속도로 선형 외삽한다 — 곱셈 한 번이라 사실상 공짜다.
    """
    dt: float = 0.001                 # 공칭 제어 주기 [s]
    enabled: bool = False             # ★ 기본 꺼짐 (이중 보상 방지 — 위 주석)
    every_n: int | None = None        # None = 지연에서 자동 (아래 주석)
    replay_budget: float = 6.0        # 제어 스텝당 허용 재적분 스텝 수
    max_age_s: float = MAX_AGE_S
    obs_w: float = 12.0               # 관측기 대역 [rad/s]
    blend: float = 1.0                # 0=보상 안 함, 1=완전 보상 (모델 신뢰도)
    hist_s: float = 0.25              # 과거 예측 이력 보관 길이 [s] (≥ max_age_s)

    def alpha_beta(self, meas_dt: float):
        """측정 간격 -> (alpha, beta/T). 위 주석의 유도 그대로."""
        T = min(max(float(meas_dt), DT_MIN_S), 1.0)
        b = math.exp(-self.obs_w * T)
        alpha = 1.0 - b * b
        beta = (1.0 - b) ** 2
        return alpha, beta / T

    def every_n_for(self, age_s: float) -> int:
        """지연 -> 무거운 쪽 실행 주기 [스텝]. 되감기 비용을 예산 안에 묶는다.

        사용자 설계: "N 은 지연되고 있다고 추정되는 시간에 따라서 calib 들어가는 거."
        되감기는 τ/dt 스텝이고 N 스텝마다 한 번 도니, 제어 스텝당 평균 비용은
        (τ/dt)/N 이다. 이걸 `replay_budget` 으로 고정하면

            N = ceil( (τ/dt) / replay_budget )

        지연이 작으면 되감기가 짧으니 자주 돌고, 크면 드물게 돈다 —
        **계산 부하는 지연과 무관하게 일정**하다.

        상한 EVERY_N_MAX 는 측정 주기다. 측정보다 드물게 돌면 측정이 와도 한참
        안 쓰이고, 기다린 만큼 나이가 더 들어 보정이 오히려 어려워진다.
        """
        if self.every_n is not None:
            return max(1, int(self.every_n))
        steps = max(0.0, float(age_s)) / max(self.dt, DT_MIN_S)
        n = math.ceil(steps / max(self.replay_budget, 1e-9))
        return max(1, min(EVERY_N_MAX, int(n)))


@dataclass
class AxisPredictor:
    """한 축의 지연 보상 예측기 (위치 + 속도).

    쓰는 법 (매 제어 스텝):
        pred.step(accel_mps2, dt)          # 빠른 채널(자세)로 전파
        pred.correct(meas_pos, meas_age_s) # 측정이 왔을 때만
        p = pred.position                  # 제어기에 넣을 값

    측정이 매 스텝 오지 않아도 된다 (VIO 30 Hz vs 제어 1 kHz). 안 오면 전파만 한다.

    ## 두 속도로 나뉜다 (사용자 설계 2026-08-28)

    "센서값에서 지연시간 뒤의 현재로 추정해서 넣는 것" 은 비싸고, "값을 좀 빠르게
    추정하는 것" 은 싸다. 그래서 갈라 놓는다:

      빠른 쪽 (매 스텝, `step`)     자세에서 온 가속도로 적분해 현재로 끌어온다.
                                    곱셈 몇 번. 이게 매 스텝 도는 덕에 출력은
                                    **언제나 현재 시각**이다.
      느린 쪽 (N 스텝마다, `correct`) 늦게 온 측정을 이력에서 그 시각과 맞춰 보고
                                    보정한다. 이력 탐색·보간·exp 가 들어가 비싸다.

    `tick()` 이 이 둘을 묶는다. 매 스텝 부르면 알아서 빠른 쪽만 돌리고, N 번째에만
    느린 쪽을 얹는다.
    """
    cfg: PredictorConfig = field(default_factory=PredictorConfig)
    position: float = 0.0
    velocity: float = 0.0
    _t: float = 0.0                                    # 내부 시계 [s]
    _hist: list = field(default_factory=list)          # [(t, p)] 과거 예측 이력
    _primed: bool = False                              # 첫 측정을 받았나
    _t_last_corr: float | None = None                  # 직전 보정 시각 (측정 간격 산출)
    n_fallback: int = 0                                # 보상을 포기한 횟수 (진단)
    n_replay: int = 0                                  # 재적분 스텝 누계 (부하 진단)
    every_n_now: int = 1                               # 현재 적용 중인 N (진단)
    _k: int = 0                                        # 데시메이션 카운터
    _pending: tuple | None = None                      # 아직 안 쓴 최신 측정

    def reset(self, position: float = 0.0, velocity: float = 0.0) -> None:
        self.position, self.velocity = float(position), float(velocity)
        self._t = 0.0
        self._hist = [(0.0, self.position, self.velocity, 0.0)]
        self._primed = False
        self._t_last_corr = None
        self.n_fallback = self.n_replay = 0
        self.every_n_now = 1
        self._k = 0
        self._pending = None

    def step(self, accel: float, dt: float | None = None) -> None:
        """가속도로 한 스텝 전파한다. accel 은 **월드 축** 가속도 [m/s²].

        중력은 호출부에서 이미 뺀 값을 넘긴다 (수평축은 애초에 없고, z 는
        추력가속도 - g). 여기서 좌표계를 추측하지 않는 이유는, 부호를 한 번
        잘못 잡으면 보상이 진동을 **키우기** 때문이다.
        """
        h = self.cfg.dt if dt is None else float(dt)
        h = min(max(h, DT_MIN_S), DT_MAX_S)
        a = float(accel)
        self.position += self.velocity * h + 0.5 * a * h * h
        self.velocity += a * h
        self._t += h
        self._hist.append((self._t, self.position, self.velocity, a))
        cut = self._t - self.cfg.hist_s
        if self._hist[0][0] < cut:
            # 앞쪽만 버린다 (리스트가 시간순이므로 이진탐색 대신 선형으로 충분)
            k = 0
            while k < len(self._hist) - 1 and self._hist[k][0] < cut:
                k += 1
            del self._hist[:k]

    def correct(self, meas_pos: float, meas_age_s: float) -> None:
        """측정이 도착했을 때 부른다. meas_age_s = 이 측정이 얼마나 낡았나 [s].

        핵심은 **혁신을 현재가 아니라 측정 시각에서 계산**한다는 것이다.
        지금 예측값과 τ 전의 측정을 그냥 빼면 그 차이의 대부분이 '그동안 움직인 양'
        이라, 보상이 아니라 되돌리기가 된다.
        """
        m = float(meas_pos)
        age = max(float(meas_age_s), 0.0)

        if not self._primed:
            self._snap(m)
            self._primed = True
            self._t_last_corr = self._t
            return
        if not self.cfg.enabled:
            self._snap(m)                      # 보상 꺼짐 = 측정 그대로 (항등)
            return
        if age > self.cfg.max_age_s:
            self._snap(m)                      # 너무 낡음 — 보상이 해가 된다
            self.n_fallback += 1
            return

        i = self._index_at(self._t - age)
        if i is None:
            self._snap(m)
            self.n_fallback += 1
            return

        t_i, p_i, v_i, a_i = self._hist[i]
        innov = (m - p_i) * self.cfg.blend
        meas_dt = (self._t - self._t_last_corr) if self._t_last_corr is not None else self.cfg.dt
        alpha, beta_over_T = self.cfg.alpha_beta(meas_dt)
        p_i += alpha * innov
        v_i += beta_over_T * innov
        self._hist[i] = (t_i, p_i, v_i, a_i)

        # 되감은 지점부터 저장해 둔 가속도로 다시 적분 — 이게 dynamics 기반 계산이다.
        p, v, t_prev = p_i, v_i, t_i
        for j in range(i + 1, len(self._hist)):
            t_j, _, _, a_j = self._hist[j]
            h = min(max(t_j - t_prev, DT_MIN_S), DT_MAX_S)
            p += v * h + 0.5 * a_j * h * h
            v += a_j * h
            self._hist[j] = (t_j, p, v, a_j)
            t_prev = t_j
        self.n_replay += len(self._hist) - i - 1
        self.position, self.velocity = p, v
        self._t_last_corr = self._t

    def _snap(self, m: float) -> None:
        """보상 없이 측정으로 맞춘다. 이력도 같이 눕혀 다음 혁신을 오염시키지 않는다."""
        self.position, self.velocity = m, 0.0
        self._hist = [(self._t, m, 0.0, 0.0)]

    def tick(self, accel: float, meas_pos: float | None = None,
             meas_age_s: float = 0.0, dt: float | None = None) -> float:
        """매 제어 스텝에서 부른다. 반환 = 제어기에 넣을 현재 위치 추정 [m].

        빠른 쪽(전파)은 매번, 무거운 쪽(되감아 재적분)은 `every_n_now` 스텝마다.
        측정이 왔는데 아직 차례가 아니면 **가장 최근 것 하나만** 들고 있다가 쓴다 —
        밀린 측정을 다 처리해 봐야 마지막 것이 현재에 가장 가깝고, 큐 자체가 비용이다.
        """
        self.step(accel, dt)
        if meas_pos is not None:
            self._pending = (float(meas_pos), float(meas_age_s), self._t)
            self.every_n_now = self.cfg.every_n_for(meas_age_s)
        self._k += 1
        if self._k >= self.every_n_now:
            self._k = 0
            if self._pending is not None:
                m, age, t_seen = self._pending
                # 들고 있던 사이에 더 낡았다. 그 몫을 나이에 더해야 이력에서 맞는
                # 시각을 찾는다 (안 더하면 보정이 과거로 밀린다).
                self.correct(m, age + (self._t - t_seen))
                self._pending = None
        return self.position

    def _index_at(self, t: float):
        """t 시각에 가장 가까운 이력 인덱스. 이력 밖이면 None."""
        h = self._hist
        if len(h) < 2 or t < h[0][0] or t > h[-1][0]:
            return None
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        return lo if (t - h[lo][0]) <= (h[hi][0] - t) else hi


def lateral_accel_from_attitude(roll: float, pitch: float, yaw: float,
                                thrust_acc: float = 9.80665):
    """자세 -> 월드 수평 가속도 (소각). 반환 (a_x, a_y) [m/s²].

    ⚠ **부호는 반드시 골든 트레이스로 확인하고 쓸 것.** 이 저장소의 자세 PID 게인이
    음수인 것(플랜트 이득이 음수)에서 보듯 부호 규약이 직관과 다른 자리가 있다.
    한 번 잘못 잡으면 예측기가 진동을 줄이는 게 아니라 **키운다**.
    그래서 `AxisPredictor.step` 은 가속도를 직접 받고, 이 함수는 편의 도구로만 둔다.
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    ax = thrust_acc * (pitch * cy + roll * sy)
    ay = thrust_acc * (pitch * sy - roll * cy)
    return ax, ay


@dataclass
class DelayCompensator:
    """3축 묶음. 제어기에서 쓰기 편하라고 얹은 얇은 껍데기."""
    cfg: PredictorConfig = field(default_factory=PredictorConfig)
    axes: list = field(default_factory=list)

    def __post_init__(self):
        if not self.axes:
            self.axes = [AxisPredictor(cfg=self.cfg) for _ in range(3)]

    def reset(self, pos=(0.0, 0.0, 0.0)) -> None:
        for a, p in zip(self.axes, pos):
            a.reset(p)

    def step(self, accel_xyz, dt: float | None = None) -> None:
        for a, acc in zip(self.axes, accel_xyz):
            a.step(acc, dt)

    def correct(self, meas_xyz, meas_age_s: float) -> None:
        for a, m in zip(self.axes, meas_xyz):
            a.correct(m, meas_age_s)

    def tick(self, accel_xyz, meas_xyz=None, meas_age_s: float = 0.0,
             dt: float | None = None):
        """매 제어 스텝. 반환 = 제어기에 넣을 (x, y, z) 추정."""
        if meas_xyz is None:
            return tuple(a.tick(acc, None, 0.0, dt)
                         for a, acc in zip(self.axes, accel_xyz))
        return tuple(a.tick(acc, m, meas_age_s, dt)
                     for a, acc, m in zip(self.axes, accel_xyz, meas_xyz))

    @property
    def position(self):
        return tuple(a.position for a in self.axes)

    @property
    def velocity(self):
        return tuple(a.velocity for a in self.axes)

    @property
    def n_fallback(self) -> int:
        return sum(a.n_fallback for a in self.axes)

    @property
    def n_replay(self) -> int:
        return sum(a.n_replay for a in self.axes)
