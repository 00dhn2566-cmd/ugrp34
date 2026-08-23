"""스펙 조속기 — 지연·부하·외란을 하나로 묶어 상위 계획기에 보고할 스펙을 만든다.

2026-08-23 신설. 사용자 요구:
    "예상 지연 시간 관련해가지고 계속해서 업데이트 하면서 상위 계획기에 보고할
     spec 을 조정하는 것까지 포함해서"

이 모듈이 `capability.json` 의 **유일한 생산자**다. 그전까지 `capability.py` 는
"한 장을 어떻게 만드나"만 알고 있었고 그걸 누가 언제 만드는지가 비어 있었다.

## 한 틱에서 하는 일

    관측 (지연 표본 / 연산 부하 / 제어 권한 rho)
      -> 융합   LoadGovernor.update(예상, 실측)  = 적용 지연
      -> 환산   지연·질량·외란  ->  한계 (v/a/j/snap)
      -> 판정   바뀌었나? 바뀌었으면 상위에 알리고 다리를 놓는다
      -> 발행   capability.json (원자적 쓰기)

## 두 경로를 다르게 다룬다 (실측 근거)

  · **자세 경로 지연**은 게이트다. 20 ms 부터는 기동을 안 해도 호버가 2.4° 로
    떨린다 (`capability.LAT_ATT_MAX_S` 주석의 실측표). 궤적을 느리게 만들어도
    정지 상태의 불안정은 안 고쳐지므로, 깎는 대신 **임무를 거부**한다.
  · **위치 경로 지연**은 감쇄다. 오차가 속도에 비례해 커지므로 속도를 낮추면 준다.

## 왜 히스테리시스가 필요한가

스펙이 바뀔 때마다 상위가 재계획하면, 재계획이 연산 부하를 올리고 그게 지연을
올려 스펙을 또 바꾼다 — 양의 되먹임. 그래서
  · 올릴 때(스펙 감소)는 즉시, 내릴 때(복귀)는 확인 후 천천히 (`LoadGovernor`)
  · 그 위에 **발행 임계**를 하나 더 둔다: 시계 배율이 `REPUBLISH_EPS` 이상
    달라져야 상위에 "바뀌었다"고 알린다.

사용:
    gov = SpecGovernor(pkg_kg=1.0, profile="precision")
    gov.set_task("smoother", units=1000, rate_hz=2.0)     # 무엇을 얼마나 돌리는지
    ...
    gov.observe_latency(0.045)                            # 상태 나이 등 실측 표본
    gov.observe_rho(0.31, yaw_err_rad=0.05)               # 제어 권한 점유
    out = gov.tick()                                      # capability 갱신 + 발행
    if out["replan_needed"]:
        br = gov.plan_bridge_for(t, base, t_now)          # 재계획 인터벌을 메울 다리
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import capability as cap                                   # noqa: E402
from compute_load import LoadEstimator, LoadGovernor       # noqa: E402
from latency_tracker import LatencyTracker                 # noqa: E402
from recovery_watcher import RecoveryWatcher               # noqa: E402

# 시계 배율이 이만큼 달라져야 '스펙이 바뀌었다'고 상위에 알린다.
# 너무 작으면 경계에서 재계획이 쏟아지고, 너무 크면 감쇄가 늦게 반영된다.
REPUBLISH_EPS = 0.05
# 재계획 예산의 하한 [s] — 계획기가 아무리 빨라도 이만큼은 다리로 메운다고 본다.
MIN_REPLAN_BUDGET_S = 0.10


def scale_from_latency_pos(latency_s: float, gust: bool = False,
                           rho_eff: float | None = None) -> float:
    """위치(VIO) 경로 지연 -> 허용 시계 배율. 실측 앵커 사이 선형 보간.

    gust=False (기본) : `capability._LAT_POS_ANCHORS` — **외란 없는** 조건에서 잰 표.
                        상위에 늘 내보내는 값이 이것이다.
    gust=True         : `_LAT_POS_ANCHORS_GUST` — 이동 중 0.3 N*m 돌풍을 맞고도
                        복귀가 사는 배율. 외란이 **실제로 감지될 때만** 쓴다.

    사용자 규정 (2026-08-23): "디폴트는 외란 없다. 만약 시간 지연에서 외란 생기면
    그때 깎는 거." 기본표에 돌풍을 섞으면 평시에도 최악을 가정한 값이 나가
    임무가 필요 이상으로 느려진다.

    표 밖(더 큰 지연)은 외삽하지 않는다 — 재보다 큰 지연에서 무슨 일이 나는지
    모르는데 추정치를 내면 그게 곧 사고다. 마지막 앵커 값을 유지하고
    `latency_extrapolated` 플래그로 상위가 알게 한다.
    """
    def look(tbl):
        tau = max(float(latency_s), 0.0)
        keys = sorted(tbl)
        if tau <= keys[0]:
            return tbl[keys[0]]
        if tau >= keys[-1]:
            return tbl[keys[-1]]
        hi = min(k for k in keys if k >= tau)
        lo = max(k for k in keys if k <= tau)
        w = 0.0 if hi == lo else (tau - lo) / (hi - lo)
        return tbl[lo] + (tbl[hi] - tbl[lo]) * w

    s_nom = look(cap._LAT_POS_ANCHORS)
    if not gust:
        return s_nom
    s_gust = look(cap._LAT_POS_ANCHORS_GUST)
    if rho_eff is None:
        return s_gust                       # 크기를 모르면 최악(잰 조건)으로 본다
    # 두 표 **사이를 보간**한다. 돌풍 표는 rho ~ GUST_RHO_REF 에 해당하는 한 점에서
    # 잰 것이라, 약한 돌풍에 그 값을 그대로 적용하면 과하게 깎인다.
    # 여기서 가산(combine_scales)을 쓰면 안 된다 — 지연x외란 조합은 **직접 잰** 것이라
    # 또 더하면 이중 계산이다.
    u = min(max(float(rho_eff) / max(cap.GUST_RHO_REF, 1e-9), 0.0), 1.0)
    return s_nom + (s_gust - s_nom) * u


def att_delay_verdict(latency_att_s: float):
    """자세(IMU) 경로 지연 판정 -> (배율, 사유). 감쇄가 아니라 **게이트**."""
    tau = max(float(latency_att_s), 0.0)
    if tau > cap.LAT_ATT_MAX_S:
        return 0.0, "att_latency_unflyable"
    if tau > cap.LAT_ATT_CLEAN_S:
        return cap.LAT_ATT_MARGIN_SCALE, "att_latency_margin"
    return 1.0, None


@dataclass
class SpecGovernor:
    """지연·부하·외란 -> 상위에 보고할 스펙. `capability.json` 의 단일 생산자."""

    pkg_kg: float = 1.0
    profile: str = "precision"
    # 자세 경로 지연은 하드웨어 상수에 가깝다 (IMU -> 제어기). 온라인 추정 대상이
    # 아니라 구성값으로 받는다. 위치 경로만 실시간으로 움직인다.
    latency_att_s: float = 0.003
    write: bool = True
    path: str | None = None

    load: LoadEstimator = field(default_factory=LoadEstimator, init=False)
    gov: LoadGovernor = field(default_factory=LoadGovernor, init=False)
    lat: LatencyTracker = field(default_factory=LatencyTracker, init=False)
    # 실측표가 틀렸을 때 폐루프로 교정 (사용자 설계 08-23). 표 = 피드포워드, 이것 = 피드백.
    rec: RecoveryWatcher = field(default_factory=RecoveryWatcher, init=False)
    bridge_lead_s: float = field(default=0.0, init=False)   # 직전 다리의 수렴 시간

    rho: float = field(default=0.0, init=False)
    yaw_err_rad: float = field(default=0.0, init=False)
    last_scale: float = field(default=1.0, init=False)
    last_cap: dict | None = field(default=None, init=False)
    ticks: int = field(default=0, init=False)
    republishes: int = field(default=0, init=False)

    # ── 관측 입력 ────────────────────────────────────────────────────────
    def set_task(self, name, units, rate_hz):
        """무엇을 얼마나 자주 돌릴 계획인지 — 부하 **예측**(선행)의 입력."""
        self.load.set_task(name, units, rate_hz)

    def observe_compute(self, name, units, elapsed_s):
        """실제 걸린 시간 — 비용 모델 온라인 보정."""
        self.load.observe(name, units, elapsed_s)

    def observe_latency(self, sample_s):
        """지연 실측 표본 (상태 나이 / 명령-응답 왕복 등, INTERFACE_SPEC §8c)."""
        return self.lat.update(sample_s)

    def observe_tracking(self, err_m, ref_ok=True, dt=0.001):
        """추종 오차 한 표본 (제어 주기). `ref_ok` = 지금 기준이 현재 limits 안인가.

        기준이 한계 밖이면 버린다 — 계획이 과한 것을 제어기 탓으로 돌려 스펙을 깎으면
        잘못된 계획이 기체 능력을 갉아먹는 되먹임이 된다 (recovery_watcher 주석).
        """
        self.rec.observe(err_m, ref_ok, dt)

    def observe_rho(self, rho, yaw_err_rad=0.0):
        """제어 권한 점유율. **구간 최대**를 넣어야 한다 (평균은 돌풍을 지운다)."""
        self.rho = float(rho)
        self.yaw_err_rad = float(yaw_err_rad)

    # ── 한 틱 ────────────────────────────────────────────────────────────
    def tick(self, dt=0.2, now=None) -> dict:
        """관측을 스펙 한 장으로 만들고 (바뀌었으면) 발행한다."""
        self.ticks += 1
        predicted = self.load.predicted_latency_s()
        measured = self.lat.predicted_s
        applied = self.gov.update(predicted, measured, dt=dt)

        # 외란이 실제로 붙었을 때만 돌풍 표로 갈아탄다 (사용자 규정: 기본은 외란 없음).
        rho_eff = max(self.rho,
                      abs(self.yaw_err_rad) / math.radians(45.0))
        gust = rho_eff > 0.0
        s_pos = scale_from_latency_pos(applied, gust=gust, rho_eff=rho_eff)
        s_att, att_reason = att_delay_verdict(self.latency_att_s)

        # 외란 쪽 배율은 capability 가 rho 에서 계산하고, 지연 쪽은 여기서 만들어
        # 넘긴다. 합치는 규칙은 **가산** (capability.combine_scales 주석 참조).
        #
        # 정직하게 적어 둘 것 — rho 감쇄와 돌풍 표는 펄스 응답 부분이 **일부 겹친다**
        # (둘 다 rho 에 반응). 그래도 더하는 쪽을 택한다: ① 두 축이 보는 외란의 모양이
        # 다르다 (rho = 지속 외란/yaw 이탈, 표 = 0.3 s 펄스) ② 겹치는 만큼 보수적으로
        # 틀리는 것이 낙관적으로 틀리는 것보다 낫다.
        # 회복 감시(폐루프 교정)도 같은 자원을 먹는 축이라 가산에 넣는다.
        # 판단 주기는 **직전 다리의 수렴 시간**보다 길어야 한다 — 앞선 결정이 반영되기
        # 전에 또 결정하면 발진이다 (recovery_watcher §안전장치 ①).
        s_rec = self.rec.decide(self.bridge_lead_s or None)
        s_lat = cap.combine_scales(s_pos, s_att, s_rec)

        # 표 안이면 실측 배율을, 표 밖이면 해석 규칙(보수적)을 쓴다.
        tbl = cap._LAT_POS_ANCHORS_GUST if gust else cap._LAT_POS_ANCHORS
        in_table = applied <= max(tbl)
        c = cap.build_capability(
            pkg_kg=self.pkg_kg, rho=self.rho, latency_s=applied,
            profile=self.profile, yaw_err_rad=self.yaw_err_rad,
            load={**self.load.snapshot(), **self.gov.snapshot(),
                  "latency_tracker": self.lat.snapshot()},
            now=now,
            latency_scale=(s_lat if in_table else None))

        # 자세 게이트는 이미 s_lat 에 들어가 build_capability 로 넘어갔다.
        # 여기서 또 더하면 **이중 계산**이다 (2026-08-23 실수, 시험이 잡음).
        s_dist = float(c["degraded"]["time_scale"])

        # 위치 표에 0.00 이 들어 있으면 그 지연에서는 **어떤 배율로도** 통과 못한 것이다
        # (안 재본 게 아니라 못 하는 것). 자세 게이트와 같이 임무 거부로 취급한다.
        pos_unflyable = (s_pos == 0.0)
        unflyable = (s_att == 0.0) or pos_unflyable
        if pos_unflyable and "pos_latency_unflyable" not in c["degraded"]["reasons"]:
            c["degraded"]["reasons"].append("pos_latency_unflyable")
        if unflyable:
            # 임무 거부. 한계를 0 으로 내려 상위가 "새 궤적을 주면 안 된다"는 걸
            # 값으로도 알게 한다 (플래그만 두면 놓친다).
            c["limits"] = {k: 0.0 for k in c["limits"]}
            c["degraded"]["time_scale"] = 0.0
            c["degraded"]["active"] = True
            if att_reason and att_reason not in c["degraded"]["reasons"]:
                c["degraded"]["reasons"].append(att_reason)
        elif att_reason and att_reason not in c["degraded"]["reasons"]:
            c["degraded"]["reasons"].append(att_reason)

        c["observed"]["latency_att_s"] = round(float(self.latency_att_s), 5)
        c["observed"]["latency_pos_applied_s"] = round(applied, 5)
        c["degraded"]["scale_sources"] = {
            "disturbance": round(s_dist, 4),
            "latency_pos": round(s_pos, 4),
            "latency_att": round(s_att, 4),
            "recovery": round(s_rec, 4),
        }
        c["observed"]["recovery"] = self.rec.snapshot()
        if s_rec < 1.0 and "recovery_slow" not in c["degraded"]["reasons"]:
            c["degraded"]["reasons"].append("recovery_slow")
        c["degraded"]["mission_allowed"] = not unflyable
        # 표 밖 지연이면 상위가 값을 곧이곧대로 믿으면 안 된다
        c["degraded"]["latency_extrapolated"] = (applied > max(tbl))
        c["degraded"]["latency_table"] = "gust" if gust else "nominal"

        s_now = float(c["degraded"]["time_scale"])
        changed = abs(s_now - self.last_scale) >= REPUBLISH_EPS
        if changed or self.last_cap is None:
            self.last_scale = s_now
            self.republishes += 1
            if self.write:
                cap.write_capability(c, self.path)
        self.last_cap = c
        return {
            "capability": c,
            "time_scale": s_now,
            "replan_needed": changed and s_now < 1.0,
            "mission_allowed": not unflyable,
            "applied_latency_s": applied,
            "replan_budget_s": self.replan_budget_s(),
        }

    def replan_budget_s(self) -> float:
        """계획기가 걸릴 것으로 보는 시간 — 다리 길이의 근거."""
        c = self.load.costs.get("plan_segment")
        seg = c.predict(8.0) if c else 0.05      # 8 세그먼트 기준
        return max(seg + self.load.predicted_latency_s(), MIN_REPLAN_BUDGET_S)

    def plan_bridge_for(self, t, base, t_now, s_now=1.0):
        """지금 스펙으로 재계획 인터벌을 메울 다리를 놓는다.

        `traj_bridge` 를 늦게 import 하는 이유 — scipy 를 끌어오므로, 다리를
        안 쓰는 소비자(예: 스펙만 읽는 감독자)까지 그 비용을 물지 않게.
        """
        from traj_bridge import plan_bridge
        if self.last_cap is None:
            raise RuntimeError("spec_governor: tick() 을 먼저 부를 것")
        base_lim = cap._interp_anchor(self.pkg_kg)["limits"]
        br = plan_bridge(t, base, t_now,
                         limits_new=self.last_cap["limits"],
                         replan_budget_s=self.replan_budget_s(),
                         s_now=s_now, base_limits=base_lim)
        # 감시의 판단 주기 하한으로 되먹인다 (무한대면 최악으로 보수적이 되므로 클램프).
        lead = br.compliant_after_s
        self.bridge_lead_s = lead if lead == lead and lead != float("inf") else 4.0
        return br

    def snapshot(self) -> dict:
        return {
            "ticks": self.ticks,
            "republishes": self.republishes,
            "time_scale": round(self.last_scale, 4),
            "latency": self.lat.snapshot(),
            "recovery": self.rec.snapshot(),
            "load": self.load.snapshot(),
            "governor": self.gov.snapshot(),
        }


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    # 시나리오: 평온 -> 부하 급증(지연) -> 부하 해소. 스펙이 어떻게 움직이나.
    g = SpecGovernor(pkg_kg=1.0, profile="precision", write=False)
    g.set_task("smoother", units=500, rate_hz=1.0)
    print(f"{'틱':>4}{'실측지연':>10}{'적용':>9}{'배율':>7}{'v':>7}{'a':>7}"
          f"{'재계획':>8}  사유")
    for k in range(40):
        if 10 <= k < 22:
            g.set_task("smoother", units=2500, rate_hz=3.0)   # 부하 급증
            sample = 0.075
        else:
            g.set_task("smoother", units=500, rate_hz=1.0)
            sample = 0.012
        g.observe_latency(sample)
        out = g.tick(dt=0.2)
        c = out["capability"]
        if k % 2 == 0 or out["replan_needed"]:
            print(f"{k:>4}{sample:>10.3f}{out['applied_latency_s']:>9.3f}"
                  f"{out['time_scale']:>7.2f}{c['limits']['v']:>7.3f}"
                  f"{c['limits']['a']:>7.3f}"
                  f"{('예' if out['replan_needed'] else '-'):>8}  "
                  f"{','.join(c['degraded']['reasons']) or '-'}")

    print("\n-- 자세 경로 지연 게이트 --")
    for att in (0.003, 0.012, 0.014, 0.018, 0.025):
        gg = SpecGovernor(pkg_kg=1.0, latency_att_s=att, write=False)
        o = gg.tick()
        print(f"  자세지연 {att*1000:>5.0f} ms -> 배율 {o['time_scale']:.2f} "
              f"v {o['capability']['limits']['v']:.3f} "
              f"임무 {'허용' if o['mission_allowed'] else '거부'} "
              f"({','.join(o['capability']['degraded']['reasons']) or '-'})")
