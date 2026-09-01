"""사용 전력량 — 추정치 -> 실측 -> 피드백 (2026-08-26, 사용자 요구).

    "먼저 사용 전력 추정치 -> 실제 전력 사용량 확인 이렇게해서 피드백 넣는거"

세 조각이다.

1. **추정치** (`estimate_energy`) — 궤적만 있으면 **비행 전에** 나온다. 상위 계획기가
   임무를 고를 때 "이 경로는 이만큼 쓴다"를 알 수 있게 하는 것이 목적이다.
2. **실측** — 비행/시뮬이 실제로 쓴 양. Simscape 배터리(`diagnose/verify_worstcase.m`),
   Gazebo 추력 적분, 실기 배터리 로그 중 무엇이든 된다. 이 모듈은 값만 받는다.
3. **피드백** (`EnergyModel.calibrated`) — 둘의 비로 효율곱을 교정한다. 다음 추정부터
   그 비가 반영된다.

왜 이 식인가 (운동량 이론):
    쿼드는 추력이 기체 z 축을 향하므로 T*z_b = m*(a - g) -> **T = m*|a - g|**.
    유도 동력(정지 대기 중 호버)은 P_ideal = n_rot * (T/n_rot)^1.5 / sqrt(2*rho*A).
    전기 동력은 P = P_ideal / (FM * eta).
검산: 1 kg 짐(m_tot 2.2726 kg) 호버에서 267 W = 8.5 g/W — 소형 멀티로터 실측 대역.

정직하게: FM(프로펠러 효율) 0.70 과 eta(모터+ESC) 0.80 은 **아직 안 잰 값**이다.
그래서 이 모듈의 요점은 절대값이 아니라 **교정 가능한 구조**다. 실측이 들어오는
순간 그 둘의 곱이 바로잡히고, 그 전까지는 `EnergyEstimate.calibrated=False` 로
"이건 미교정 추정치"라고 달고 다닌다.

가드레일은 INTERFACE_SPEC 6절(플랜트 상수 추정 소비)과 같은 원칙을 따른다:
  - 앵커(기본 상수)는 불변 — 교정은 배수로만 얹는다
  - 한 번에 튀지 않게 램프 (기본 +-20%/회)
  - 표본이 모자라면 반영하지 않는다
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, replace
from typing import Iterable, Optional, Sequence

# ── 앵커: 기체 상수. 갱신 금지 (교정은 EnergyModel.eff_cal 배수로만) ──────────
RHO = 1.225                 # kg/m^3, 해수면 표준
R_PROP = 0.127              # m — 프로펠러 반경 (D = 0.254, qc_motor.hpp 와 동일)
N_ROTOR = 4
G = 9.81                    # m/s^2

DISK_AREA = math.pi * R_PROP * R_PROP        # 로터 1개 디스크 면적 [m^2]

# ── 미측정 효율. 이 둘의 곱이 피드백으로 교정되는 자리다. ────────────────────
FM_DEFAULT = 0.70           # figure of merit (프로펠러)
ETA_DEFAULT = 0.80          # 모터 + ESC

# 교정 배수의 허용 범위. 밖으로 나가면 측정이 잘못됐다고 보는 편이 안전하다
# (효율이 1을 넘거나 0.1 밑으로 떨어지는 것은 물리가 아니라 배선 실수다).
CAL_MIN, CAL_MAX = 0.30, 3.00
CAL_STEP_MAX = 0.20         # 한 번의 피드백으로 움직일 수 있는 최대 비율
MIN_SAMPLES = 3             # 이보다 적은 표본은 반영하지 않는다


@dataclass(frozen=True)
class EnergyModel:
    """전력 추정 모델. `eff_cal` 만이 피드백으로 움직인다."""

    fm: float = FM_DEFAULT
    eta: float = ETA_DEFAULT
    eff_cal: float = 1.0          # 실측/추정 비로 얻은 교정 배수
    calibrated: bool = False      # 실측이 한 번이라도 반영됐는가
    n_samples: int = 0

    @property
    def efficiency(self) -> float:
        """전체 효율곱. P = P_ideal / efficiency."""
        return self.fm * self.eta / self.eff_cal

    def hover_power(self, mass_kg: float) -> float:
        """정지 호버 전기 동력 [W] — 모델이 말이 되는지 보는 눈금."""
        return electrical_power(mass_kg * G, self)

    def calibrated_with(self, ratio_act_over_est: float, n_samples: int) -> "EnergyModel":
        """실측/추정 비를 반영한 새 모델. 원본은 안 건드린다 (frozen).

        `ratio > 1` = 실제로 더 썼다 = 효율을 낮춰야 한다 -> eff_cal 을 키운다
        (efficiency = fm*eta/eff_cal 이므로).
        """
        if n_samples < MIN_SAMPLES:
            return replace(self, n_samples=n_samples)
        if not (math.isfinite(ratio_act_over_est) and ratio_act_over_est > 0):
            return self
        target = self.eff_cal * ratio_act_over_est
        # 램프: 한 번에 CAL_STEP_MAX 이상 못 움직인다
        lo = self.eff_cal * (1.0 - CAL_STEP_MAX)
        hi = self.eff_cal * (1.0 + CAL_STEP_MAX)
        stepped = min(max(target, lo), hi)
        clamped = min(max(stepped, CAL_MIN), CAL_MAX)
        return replace(self, eff_cal=clamped, calibrated=True, n_samples=n_samples)


DEFAULT_MODEL = EnergyModel()


def thrust_from_accel(acc: Sequence[float], mass_kg: float) -> float:
    """지령 가속도 -> 총 추력 [N].  T = m*|a - g|  (쿼드 미분평탄성).

    `acc` 는 world 좌표 가속도 [m/s^2]. 중력은 (0,0,-G) 이므로 a - g 의 z 성분이
    a_z + G 가 된다. 호버(a=0)면 T = m*G, 기울면 수평 성분만큼 커진다 — 별도의
    cos(기울기) 보정이 필요 없다 (같은 식의 다른 표현일 뿐).
    """
    ax, ay, az = (list(acc) + [0.0, 0.0, 0.0])[:3]
    return mass_kg * math.sqrt(ax * ax + ay * ay + (az + G) * (az + G))


def electrical_power(thrust_total_n: float, model: EnergyModel = DEFAULT_MODEL) -> float:
    """총 추력 -> 전기 동력 [W]. 음수 추력은 0 으로 본다 (프로펠러는 밀기만 한다)."""
    if thrust_total_n <= 0.0:
        return 0.0
    per = thrust_total_n / N_ROTOR
    p_ideal = N_ROTOR * per ** 1.5 / math.sqrt(2.0 * RHO * DISK_AREA)
    return p_ideal / model.efficiency


@dataclass(frozen=True)
class EnergyEstimate:
    """상위 계획기에 올리는 한 장. 단위는 Wh / W / s."""

    wh: float
    wh_per_m: float
    p_mean_w: float
    p_peak_w: float
    duration_s: float
    distance_m: float
    mass_kg: float
    calibrated: bool
    efficiency: float

    def as_dict(self) -> dict:
        return {
            "energy_wh": round(self.wh, 5),
            "energy_wh_per_m": round(self.wh_per_m, 5),
            "power_mean_w": round(self.p_mean_w, 2),
            "power_peak_w": round(self.p_peak_w, 2),
            "duration_s": round(self.duration_s, 3),
            "distance_m": round(self.distance_m, 4),
            "mass_kg": round(self.mass_kg, 4),
            # 미교정이면 상위가 그렇게 알고 여유를 둬야 한다. 숨기지 않는다.
            "calibrated": self.calibrated,
            "efficiency": round(self.efficiency, 4),
        }


def estimate_energy(t: Sequence[float],
                    acc: Sequence[Sequence[float]],
                    mass_kg: float,
                    pos: Optional[Sequence[Sequence[float]]] = None,
                    model: EnergyModel = DEFAULT_MODEL) -> EnergyEstimate:
    """시간·가속도 열에서 사용 전력량을 추정한다.

    `acc` 는 궤적 층이 이미 갖고 있는 값이다 (INTERFACE_SPEC 2절 산출물의 ref 가속도,
    또는 path_time 의 속도 프로파일 미분). 그래서 **비행 전에** 부를 수 있다.
    `pos` 를 주면 경로 길이로 Wh/m 를 낸다 (임무 비교에 편한 단위).
    """
    t = list(t)
    acc = [list(a) for a in acc]
    if len(t) < 2 or len(acc) != len(t):
        raise ValueError("t 와 acc 의 길이가 같아야 하고 표본이 2개 이상이어야 한다")

    power = [electrical_power(thrust_from_accel(a, mass_kg), model) for a in acc]
    wh = 0.0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0:
            continue
        wh += 0.5 * (power[i] + power[i - 1]) * dt        # 사다리꼴
    wh /= 3600.0

    dur = t[-1] - t[0]
    dist = 0.0
    if pos is not None:
        p = [list(q) for q in pos]
        for i in range(1, len(p)):
            dist += math.dist(p[i][:3], p[i - 1][:3])
    return EnergyEstimate(
        wh=wh,
        wh_per_m=(wh / dist) if dist > 1e-9 else float("nan"),
        p_mean_w=(wh * 3600.0 / dur) if dur > 1e-9 else 0.0,
        p_peak_w=max(power) if power else 0.0,
        duration_s=dur,
        distance_m=dist,
        mass_kg=mass_kg,
        calibrated=model.calibrated,
        efficiency=model.efficiency,
    )


def estimate_energy_for_trajectory(traj: dict, mass_kg: float,
                                   model: EnergyModel = DEFAULT_MODEL) -> EnergyEstimate:
    """INTERFACE_SPEC 2절 `trajectory.json` 을 그대로 받아 추정한다.

    `acc` 가 들어 있으면 그것을 쓰고, 없으면 위치를 두 번 차분한다 (성형 궤적은
    C3 라 차분이 안전하다 — 날것 웨이포인트에는 쓰지 말 것).
    """
    t = list(traj["t"])
    pos = [list(p) for p in traj["pos"]]
    acc = traj.get("acc")
    if acc is None:
        acc = _second_difference(t, pos)
    return estimate_energy(t, acc, mass_kg, pos=pos, model=model)


def _second_difference(t: Sequence[float], pos: Sequence[Sequence[float]]):
    n = len(t)
    out = [[0.0, 0.0, 0.0] for _ in range(n)]
    for k in range(3):
        for i in range(1, n - 1):
            h1 = t[i] - t[i - 1]
            h2 = t[i + 1] - t[i]
            if h1 <= 0 or h2 <= 0:
                continue
            # 불균등 격자 2차 차분
            out[i][k] = 2.0 * (h1 * pos[i + 1][k] - (h1 + h2) * pos[i][k] + h2 * pos[i - 1][k]) \
                        / (h1 * h2 * (h1 + h2))
        if n >= 3:
            out[0][k] = out[1][k]
            out[n - 1][k] = out[n - 2][k]
    return out


# ── 피드백 파일 (INTERFACE_SPEC 6절과 같은 가드레일) ─────────────────────────

FEEDBACK_NAME = "energy_feedback.json"

# 어느 출처를 교정에 써도 되는가.
#
# 08-26 실측에서 같은 비행에 대해 세 모델이 이렇게 갈렸다 (1 kg 짐, 호버 환산):
#   운동량 이론 추정기    267 W = 8.5 g/W    <- 실기 대역(8~10 g/W)과 일치
#   Simscape 배터리      90 W = 25 g/W      <- 물리적으로 불가능하게 좋다
#   qc_motor.hpp 의 Cq   636 W = 3.6 g/W    <- 비현실적으로 나쁘다
#
# Simscape 배터리가 3배 낙관적인 이유는 그 전기 모델이 SOC 표시용으로 이상화돼 있어
# 모터 동손/ESC 손실을 안 담기 때문으로 보인다. Cq 가 나쁜 이유는 그 값이 프로펠러
# 공력이 아니라 **토크 클램프 0.2 N*m 에서 634 rad/s 가 되도록 역산**된 값이라서다
# (qc_motor.hpp 주석 참조 — 클램프를 재현하려고 맞춘 수이지 공력 계수가 아니다).
#
# 그래서 이 둘로는 교정하지 않는다. 낙관적인 쪽으로 교정하면 계획기가 여유를 3배로
# 착각하고, 그건 배터리가 임무 중에 죽는 방식이다. 교정은 **실기 전력 모듈**이
# 들어온 뒤에 연다.
TRUSTED_SOURCES = frozenset({"power_module", "bms", "flight_log"})


def source_is_trusted(source: str) -> bool:
    """이 출처의 실측으로 추정기를 교정해도 되는가."""
    return source in TRUSTED_SOURCES


def write_feedback(path: str, ratio_act_over_est: float, n_samples: int,
                   source: str, note: str = "") -> None:
    """실측 결과를 피드백 파일로 남긴다. 원자적 쓰기 (tmp -> replace).

    믿을 수 없는 출처(시뮬 배터리 등)도 **기록은 한다** — 나중에 비교할 재료라서.
    다만 `trusted:false` 로 달아 두고, load_feedback 이 그것으로는 교정하지 않는다.
    """
    trusted = source_is_trusted(source)
    payload = {
        "schema_version": "0.1",
        "source": source,
        "trusted": trusted,
        "n_samples": int(n_samples),
        "ratio_act_over_est": float(ratio_act_over_est),
        "note": note or ("energy.py 의 efficiency 를 이 비로 교정. 앵커 상수는 불변."
                         if trusted else
                         "기록만 한다 — 이 출처로는 교정하지 않는다 (TRUSTED_SOURCES 참조)."),
    }
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def load_feedback(path: str, model: EnergyModel = DEFAULT_MODEL) -> EnergyModel:
    """피드백 파일을 읽어 교정된 모델을 돌려준다. 파일이 없으면 원본 그대로."""
    if not os.path.isfile(path):
        return model
    with open(path, encoding="utf-8") as fh:
        d = json.load(fh)
    src = str(d.get("source", ""))
    # 파일이 스스로 trusted 를 적었으면 그것을 쓰고, 없으면 출처 이름으로 판정한다.
    trusted = bool(d.get("trusted", source_is_trusted(src)))
    if not trusted:
        # 조용히 무시하지 않는다 — 왜 안 먹었는지 남는 편이 낫다.
        return replace(model, n_samples=int(d.get("n_samples", 0)))
    return model.calibrated_with(float(d.get("ratio_act_over_est", 1.0)),
                                 int(d.get("n_samples", 0)))


# ══════════════════════════════════════════════════════════════════════════
# 남은 전력 — 누가 무엇을 아는가
# ══════════════════════════════════════════════════════════════════════════
#
# 사용자 질문(08-26): "모터 회전량과 모터의 부하 확인해서 그것을 통해 사용 가능한
# 전력량 확인하는 방식 가능? 아니면 그냥 BMS에서 바로 확인하는 게 맞는 건가."
#
# **잔량은 BMS 다.** 모터 관측으로는 잔량을 알 수 없다. 모터 회전량·부하가 주는 것은
# "지금 얼마나 쓰고 있나"(순간 전력)이지 "얼마나 남았나"가 아니다. 모터에서 잔량을
# 얻으려면 만충 시점부터 적분해야 하는데,
#   - 적분 드리프트가 비행 내내 쌓이고 (되돌릴 기준점이 없다)
#   - 배터리 노화로 실제 용량이 줄어든 것을 못 보고
#   - 온도/내부저항/전압 강하에 따른 가용 에너지 감소를 못 본다.
# 즉 모터 적분은 **열려 있는 루프**다. BMS(또는 전압+전류 센서 + 쿨롱 카운팅)는
# 전압으로 되잡을 수 있는 **닫힌 루프**다.
#
# 그렇다고 모터 관측이 쓸모없는 게 아니다. BMS 가 못 하는 두 가지를 한다.
#   1) **앞으로 쓸 양의 예측** — BMS 는 과거/현재만 안다. "이 경로로 가면 얼마 드나"는
#      estimate_energy() 가 답한다. 계획기가 필요로 하는 건 이쪽이다.
#   2) **교차검증 / 고장 검출** — 모터 유도 전력과 BMS 실측 전력이 벌어지면
#      프로펠러 손상·모터 이상·BMS 오교정 중 하나다. 어느 쪽도 조용히 넘기면 안 된다.
#
# 그래서 역할을 이렇게 나눈다:
#   BMS        -> EnergyBudget.remaining_wh   (남은 양, 진실)
#   추정기      -> EnergyEstimate.wh          (쓸 양, 계획용)
#   계획기      -> EnergyBudget.can_afford()  (둘을 비교해 임무를 고른다)
#   실측/추정 비 -> EnergyModel.calibrated_with()  (추정기를 교정)
#
# 실기 주의: FX450 급에 스마트 BMS 가 달려 있을 가능성은 낮다. 현실적 최소 구성은
# **전력 모듈(전압+전류 센서)** 이고, 거기서 쿨롱 카운팅 + 전압 기반 SoC 보정을 한다.
# 이 모듈은 그 값이 어디서 왔는지(`source`)만 기록하고 계산은 같게 한다.


def power_from_motors(omega_rad_s: Iterable[float],
                      model: EnergyModel = DEFAULT_MODEL,
                      cq: float = 0.01517, d_prop: float = 0.254) -> float:
    """모터 회전량 -> 순간 전기 동력 [W]. **잔량이 아니라 순간 소비다.**

    각 로터의 공력 반토크 Q = Cq*rho*n^2*D^5 [N*m] (n = rev/s) 에 각속도를 곱해
    기계 동력을 얻고, 모터+ESC 효율로 나눈다. Cq 는 `qc_motor.hpp` 와 같은 값이다
    (9차 평형 교정: Q = 0.2 N*m @ 634 rad/s).

    쓰임새는 둘이다 — (a) BMS 실측과 맞대보는 교차검증, (b) BMS 가 없는 시뮬에서의
    대체 실측. 잔량 추정에는 쓰지 말 것 (위 주석 참조).
    """
    p_mech = 0.0
    for w in omega_rad_s:
        w = abs(float(w))
        n = w / (2.0 * math.pi)
        q = cq * RHO * n * n * d_prop ** 5
        p_mech += q * w
    return p_mech / model.eta if model.eta > 0 else float("inf")


@dataclass(frozen=True)
class EnergyBudget:
    """남은 전력. 계획기가 "이 임무를 감당할 수 있나"를 판정하는 자리.

    `remaining_wh` 는 **BMS/전력모듈에서 온 값**이어야 한다. 추정으로 채우지 말 것 —
    채우려거든 `source` 에 그렇게 적고 `trusted=False` 로 둘 것.
    """

    remaining_wh: float
    source: str = "bms"          # bms | power_module | sim_battery | estimated
    trusted: bool = True         # False 면 계획기는 예비율을 더 크게 잡아야 한다
    reserve_frac: float = 0.20   # 착륙/여유로 남겨 둘 몫 (기본 20%)

    @property
    def usable_wh(self) -> float:
        """실제로 임무에 쓸 수 있는 양. 믿을 수 없는 출처면 예비를 두 배로 잡는다."""
        rf = self.reserve_frac if self.trusted else min(0.5, self.reserve_frac * 2.0)
        return max(0.0, self.remaining_wh * (1.0 - rf))

    def can_afford(self, est: EnergyEstimate, margin: float = 1.0) -> bool:
        """추정 소비가 가용량 안에 드는가. `margin` 은 추가 안전계수."""
        need = est.wh * margin
        # 미교정 추정치는 틀릴 수 있다. 그 사실을 판정에 반영한다 (숨기면 사고가 난다).
        if not est.calibrated:
            need *= 1.3
        return need <= self.usable_wh

    def headroom_wh(self, est: EnergyEstimate) -> float:
        """감당하고도 남는 양 [Wh]. 음수면 그만큼 모자란다."""
        need = est.wh if est.calibrated else est.wh * 1.3
        return self.usable_wh - need

    def as_dict(self) -> dict:
        return {
            "remaining_wh": round(self.remaining_wh, 4),
            "usable_wh": round(self.usable_wh, 4),
            "reserve_frac": self.reserve_frac,
            "source": self.source,
            "trusted": self.trusted,
        }


def energy_block(est: EnergyEstimate,
                 budget: Optional[EnergyBudget] = None,
                 margin: float = 1.0) -> dict:
    """상위 보고용 한 블록. capability.json 에 그대로 붙일 수 있는 모양.

    계획기는 이것만 보고 "이 임무 가능한가 / 얼마나 여유 있나"를 판단할 수 있다.
    """
    out = {"estimate": est.as_dict()}
    if budget is not None:
        out["budget"] = budget.as_dict()
        out["affordable"] = budget.can_afford(est, margin)
        out["headroom_wh"] = round(budget.headroom_wh(est), 4)
        # 남은 양으로 이 임무를 몇 번 더 할 수 있나 — RL 보상/임무 선택에 편한 단위
        need = est.wh if est.calibrated else est.wh * 1.3
        out["repeats_possible"] = int(budget.usable_wh // need) if need > 1e-9 else 0
    return out


# ══════════════════════════════════════════════════════════════════════════
# 감쇄의 에너지 대가 — "필요한 만큼만 깎아라"를 수로 만든다
# ══════════════════════════════════════════════════════════════════════════
#
# 사용자 지적(08-26): "사용 전력 문제도 가능한 범위 내에서 최소화시켜야지 그게 다 자원인데".
#
# 맞다. 다만 이 기체에서 에너지는 사실상 **무게 x 시간** 이다 (3 m 이동의 평균 전력이
# 호버 전력과 같다 — 기동 몫이 측정 한계 안이다). 그래서 결론이 껄끄럽다:
#
#     스펙 배율 s 로 깎으면 이동시간이 1/s 로 늘고, **에너지도 대략 1/s 로 는다.**
#
# 즉 지연 강건성(깎아야 산다)과 전력(깎으면 손해)이 **정면으로 맞선다**. 어느 한쪽을
# 최소화하는 것은 답이 아니고, 답은 **게이트를 통과하는 가장 큰 s** 다. 그게
# recovery_watcher 가 하는 일이고 (깨끗하면 s 를 되돌린다), 여기서는 그 선택의
# 대가를 Wh 로 찍어 계획기가 보게 한다.
#
# 주의 — 이 1/s 관계는 **호버 지배 구간**의 근사다. 실제 멀티로터는 전진비행에서
# 유도동력이 줄어 전력-속도 곡선에 최소점이 있는데, 이 모델에는 그 항이 없다.
# 그래서 "빠를수록 무조건 이득"으로 나온다. 전진비행 항을 재기 전에는 이 결론을
# 순항 속도 최적화 근거로 쓰지 말 것. (감쇄 비용 비교에는 유효하다 — 같은 경로를
# 같은 기하로 느리게 가는 것이라 전진비행 이득도 같이 줄기 때문이다.)


def derate_energy_cost(est_at_full: EnergyEstimate, s: float) -> dict:
    """스펙을 `s` 로 깎았을 때 추가로 드는 에너지.

    `est_at_full` 은 s=1.00 (안 깎은 상태) 추정치다. 시간축만 늘리는 감쇄이므로
    (INTERFACE_SPEC 5c: v~s, a~s^2, ...) 지속시간이 1/s 로 늘고 평균 전력은 거의
    그대로다 -> 에너지도 ~1/s.
    """
    if not (0.0 < s <= 1.0):
        raise ValueError("s 는 (0, 1] 이어야 한다")
    wh_derated = est_at_full.wh / s
    extra = wh_derated - est_at_full.wh
    return {
        "s": round(s, 4),
        "energy_wh_at_s1": round(est_at_full.wh, 5),
        "energy_wh": round(wh_derated, 5),
        "extra_wh": round(extra, 5),
        "extra_frac": round(extra / est_at_full.wh, 4) if est_at_full.wh > 1e-12 else 0.0,
        "duration_s": round(est_at_full.duration_s / s, 3),
    }


def cheapest_sufficient_s(est_at_full: EnergyEstimate,
                          budget: EnergyBudget,
                          s_required: float,
                          margin: float = 1.0) -> dict:
    """감쇄 요구(s_required)와 전력 예산을 같이 본다.

    `s_required` 는 5c 절의 지연/회복 규칙이 요구하는 배율이다. 그보다 **더** 깎는
    것은 강건성에 이득이 없으면서 에너지만 먹는다 (과감쇄). 그보다 덜 깎으면 복귀가
    안 산다. 그래서 선택지는 사실 하나뿐이고, 이 함수는 그 하나가 **예산 안에 드는지**를
    본다. 안 들면 임무 자체를 줄여야 한다 — 스펙을 더 깎는 것은 해결책이 아니다.
    """
    cost = derate_energy_cost(est_at_full, s_required)
    need = cost["energy_wh"] * margin
    if not est_at_full.calibrated:
        need *= 1.3
    ok = need <= budget.usable_wh
    return {
        **cost,
        "affordable": ok,
        "usable_wh": round(budget.usable_wh, 4),
        "headroom_wh": round(budget.usable_wh - need, 4),
        # 예산이 모자랄 때 할 수 있는 것: 경로를 줄이는 것이지 더 깎는 것이 아니다.
        "advice": ("ok" if ok else
                   "예산 부족 — 경로를 줄이거나 짐을 덜 것. 더 깎으면 시간이 늘어 더 나빠진다."),
    }


if __name__ == "__main__":       # 눈금 확인용
    m = DEFAULT_MODEL
    for mass, tag in ((2.2726, "1 kg 짐"), (1.2726, "생 드론")):
        p = m.hover_power(mass)
        print("%s: 호버 %.0f W  (%.1f g/W),  10분 비행 %.1f Wh"
              % (tag, p, mass * 1000.0 / p, p / 6.0))
    # 모터 관측 교차검증: 호버 634 rad/s x4
    print("모터 관측 호버 전력: %.0f W  (추정식 %.0f W)"
          % (power_from_motors([634.5] * 4, m), m.hover_power(2.2726)))
