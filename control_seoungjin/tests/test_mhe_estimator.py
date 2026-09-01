"""MHE 지연 보상 추정기 시험.

되감기 판(`test_delay_compensator.py`)과 겹치는 안전 규약 시험은 최소로 하고,
**MHE 만의 성질**에 집중한다 — 바이어스 관측 가능성, N 규칙, 선형 최소제곱의 정확성.
"""
import math
import pytest

from mhe_estimator import MheConfig, AxisMhe, MheCompensator, _solve3


def _sim(tau, bias, horizon, trust=0.7, T=3.0, dt=0.001, meas_hz=30.0,
         sigma=0.01, use_est=True, cfg_kw=None):
    """모델이 모르는 상수 가속(바람)을 넣고 지연된 측정만 준다."""
    kw = dict(dt=dt, horizon_s=horizon, meas_std=sigma, bias_rw=2.0,
              accuracy_tol_m=0.005, age_trust=trust)
    kw.update(cfg_kw or {})
    est = AxisMhe(cfg=MheConfig(**kw))
    est.reset(0.0, 0.0)
    p = v = 0.0
    err = []; last = -1.0; buf = []; raw = 0.0; guarded = 0; n = 0
    for k in range(int(T / dt)):
        t = k * dt
        a_model = 1.5 * math.sin(2 * math.pi * 0.7 * t)
        a_true = a_model + bias
        p += v * dt + 0.5 * a_true * dt * dt
        v += a_true * dt
        buf.append((t, p))
        meas = None; age = 0.0
        if t - last >= 1.0 / meas_hz:
            c = [q for (s, q) in buf if s <= t - tau]
            if c:
                meas = c[-1]; age = tau; last = t; raw = meas
        ep = est.tick(a_model, meas, age)
        if t > 1.5:
            err.append(abs((ep if use_est else raw) - p))
            n += 1
            if est.fault:
                guarded += 1
    rms = math.sqrt(sum(e * e for e in err) / len(err))
    return rms, est, guarded / max(n, 1)


# ── 선형 최소제곱 자체 ────────────────────────────────────────────────

def test_solve3_matches_known_inverse():
    """촐레스키 해와 분산 대각이 손으로 푼 값과 맞나."""
    A = [[4.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 2.0]]
    b = [1.0, 2.0, 3.0]
    x, var = _solve3(A, b)
    for i in range(3):
        assert abs(sum(A[i][j] * x[j] for j in range(3)) - b[i]) < 1e-9
    # det = 18, 역행렬 대각 = 5/18, 8/18, 11/18
    for got, want in zip(var, (5 / 18, 8 / 18, 11 / 18)):
        assert abs(got - want) < 1e-9


def test_solve3_refuses_singular():
    """관측이 부족하면 None 을 낸다 — 조용히 쓰레기를 내지 않는다."""
    x, var = _solve3([[0.0, 0.0, 0.0]] * 3, [1.0, 1.0, 1.0])
    assert x is None and var is None


# ── 바이어스 관측 가능성 (이 모듈의 핵심 제약) ────────────────────────

def test_bias_needs_long_enough_window():
    """창이 짧으면 바이어스를 **못 본다**. 0.5*b*H^2 가 측정 잡음에 묻힌다.

    이건 구현 결함이 아니라 물리다. 창 길이는 지연이 아니라 **관측 가능성**으로
    정해야 한다는 근거 — H >~ sqrt(2*sigma/b).
    """
    b, sigma = 0.8, 0.01
    h_need = math.sqrt(2 * sigma / b)          # ~0.158 s
    _, short, _ = _sim(0.06, b, horizon=0.06, T=4.0)
    _, long_, _ = _sim(0.06, b, horizon=0.30, T=4.0)
    assert 0.06 < h_need < 0.30                # 시험이 경계를 사이에 두고 있나
    assert abs(short.bias - b) > 0.5           # 짧은 창: 못 본다
    assert abs(long_.bias - b) < 0.2           # 긴 창: 본다
    assert long_.bias_std < short.bias_std     # 그리고 스스로 그걸 안다


def test_bias_std_reports_ignorance():
    """창이 짧으면 db 가 사전(bias_rw)에 가깝게 남는다 = 모른다고 보고한다."""
    _, est, _ = _sim(0.06, 0.8, horizon=0.04, T=2.0)
    assert est.bias_std > 0.8 * est.cfg.bias_rw


# ── 보상 성능 ─────────────────────────────────────────────────────────

@pytest.mark.parametrize("tau", [0.02, 0.06, 0.10])
def test_beats_uncompensated(tau):
    """지연이 얼마든 원 측정을 그대로 쓰는 것보다 나아야 한다."""
    base, _, _ = _sim(tau, 0.3, 0.20, use_est=False)
    got, _, _ = _sim(tau, 0.3, 0.20)
    assert got < 0.5 * base


def test_guards_do_not_fire_in_normal_flight():
    """정상 비행에서 안전장치가 물면 안 된다.

    처음 구현이 이탈 한계를 `v_max * (도착 후 경과)` 로 계산해 2499 스텝 중 2478 번
    잘렸다. 마지막 측정은 도착 시점에 이미 age 만큼 낡아 있으므로 경과는
    `gap + age` 여야 한다. 증상이 고약했던 이유 — 출력이 늘 잘리니 age_trust 를
    바꿔도 RMS 가 **똑같이** 나와서, 손잡이가 안 걸린 것처럼 보였다. (2026-08-29)
    """
    _, _, frac = _sim(0.06, 0.3, 0.20)
    assert frac == 0.0


def test_age_trust_is_monotone():
    """보수적일수록 오차가 크다 — 안정성을 사고 성능을 판다는 게 보여야 한다.

    이 단조성이 깨지면 안전장치가 출력을 물고 있다는 신호다 (위 시험 참조).
    """
    rs = [_sim(0.06, 0.3, 0.20, trust=t)[0] for t in (0.0, 0.5, 0.7, 1.0)]
    assert rs[0] > rs[1] > rs[2] > rs[3]


# ── N 규칙 (2026-08-29 정정: 계산 예산이 아니라 정확도 예산) ──────────

def test_n_shrinks_when_bias_uncertain():
    """바이어스를 모를수록 자주 풀어야 한다.

    옛 규칙(`N = ceil((tau/dt)/6)`)은 반대였다 — 지연이 클수록 N 을 키워서,
    제일 안 중요한 5 ms 에 계산을 제일 많이 쓰고 생사가 갈리는 100 ms 에 제일
    아꼈다. 여기서는 db 가 크면(모르면) N 이 작아진다.
    """
    cfg = MheConfig(dt=0.001, accuracy_tol_m=0.005, horizon_s=0.06)
    n_sure, _ = cfg.n_from(0.05)      # 잘 안다
    n_unsure, _ = cfg.n_from(2.0)     # 모른다
    assert n_unsure < n_sure


def test_n_reason_reports_compute_binding():
    """계산이 정확도를 못 따라가면 그렇게 보고한다 — 조용히 나빠지지 않는다."""
    # 창 50 ms 를 예산 1 로 나누면 N=50 이 하한인데, 정확도는 그보다 훨씬 자주
    # 풀라고 요구한다 -> 계산이 묶는다. (N_MAX 에 닿지 않게 창을 짧게 잡았다 —
    # 거기 닿으면 이유가 measurement 로 바뀐다)
    tight = MheConfig(dt=0.001, accuracy_tol_m=1e-9, horizon_s=0.05,
                      compute_budget=1.0)
    n, why = tight.n_from(0.05)
    assert why == "compute"
    assert n == 50
    loose = MheConfig(dt=0.001, accuracy_tol_m=0.005, horizon_s=0.02,
                      compute_budget=100.0)
    _, why2 = loose.n_from(2.0)
    assert why2 == "accuracy"


def test_every_n_is_at_least_one():
    cfg = MheConfig(dt=0.001, accuracy_tol_m=0.0, horizon_s=0.06)
    n, _ = cfg.n_from(1e9)
    assert n >= 1


def test_fixed_every_n_overrides():
    cfg = MheConfig(every_n=7)
    assert cfg.n_from(1.0) == (7, "fixed")


# ── 안전 규약 ─────────────────────────────────────────────────────────

def test_dropout_holds_last_measurement():
    """측정이 끊기면 추측항법을 멈춘다. 혼자 적분하며 거짓말하면 최악 실패모드."""
    est = AxisMhe(cfg=MheConfig(dt=0.001, dropout_s=0.05, horizon_s=0.10))
    est.reset(0.0, 0.0)
    est.tick(0.0, 1.0, 0.0)
    out = None
    for _ in range(200):                 # 0.2 s 동안 측정 없음
        out = est.tick(5.0)              # 큰 가속도를 줘도
    assert est.fault == "dropout"
    assert abs(out - 1.0) < 1e-9         # 마지막 측정에 눌러앉는다


def test_innovation_gate_snaps_to_sensor():
    """예측과 측정이 말이 안 되게 벌어지면 센서가 이긴다."""
    est = AxisMhe(cfg=MheConfig(dt=0.001, innov_max_m=0.1, horizon_s=0.10))
    est.reset(0.0, 0.0)
    for _ in range(50):
        est.tick(0.0, 0.0, 0.0)
    out = est.tick(0.0, 10.0, 0.0)       # 10 m 점프 = 말이 안 됨
    assert est.fault == "innov"
    assert abs(out - 10.0) < 0.2         # 버리지 않고 센서로 간다


def test_age_trust_cannot_exceed_one():
    """과대보상 금지 — 성능이 아니라 안정성 문제다."""
    est = AxisMhe(cfg=MheConfig(dt=0.001, age_trust=5.0, horizon_s=0.10))
    est.reset(0.0, 0.0)
    for _ in range(20):
        est.tick(0.0)
    est._ingest(0.0, 0.010)
    # 되감은 시각이 10 ms 보다 더 과거로 가면 안 된다
    assert est._last_meas_vt >= est._t - 0.0101


# ── 3축 래퍼 ──────────────────────────────────────────────────────────

def test_compensator_reports_worst_axis():
    """경고가 묻히면 안 된다 — 한 축이라도 compute 에 묶이면 그걸 낸다."""
    c = MheCompensator(cfg=MheConfig(dt=0.001))
    c.reset()
    c.axes[0]._n_reason = "accuracy"
    c.axes[1]._n_reason = "compute"
    c.axes[2]._n_reason = "accuracy"
    assert c.n_reason == "compute"


def test_snapshot_shape():
    c = MheCompensator(cfg=MheConfig(dt=0.001))
    c.reset()
    snap = c.snapshot()
    for k in ("enabled", "every_n", "n_reason", "bias_mps2",
              "bias_std_mps2", "healthy", "fault"):
        assert k in snap
    assert len(snap["bias_mps2"]) == 3


def test_disabled_by_default():
    """★ 기본 꺼짐. OpenVINS 가 이미 IMU 전파하면 이중 보상이다 (태민 확인 전)."""
    assert MheConfig().enabled is False
