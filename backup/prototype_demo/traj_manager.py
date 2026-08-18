"""임무 상태기계 — 스캔 → 복원 → 통과 → (즉시 재스캔) → 반복.

    python traj_manager.py --n-windows 3 --stability-thr 80

지금까지 스캔 경로와 통과 계획은 서로 모르는 채로 돌았다. 스캔은
``sim/pybullet_stream.py`` 의 고정 패턴이고, 통과 계획은 ``planner.py`` 가 "창문을
이미 안다" 는 전제로 냈다. 이 파일이 그 둘을 하나의 루프로 묶는다.

상태 전이
---------
    SCAN ──관측 누적──▶ RECON ──안정성 판정──┬─ 미달 ─▶ SCAN (재스캔)
                                            └─ 통과 ─▶ APPROACH ─▶ TRAVERSE
    TRAVERSE ─통과 완료─▶ RESCAN ─▶ (다음 창문) SCAN …  또는 DONE

  * SCAN 은 xy / yz / yaw 세 모션을 **전부** 쓴다. 한 축으로만 움직이면 그 축에
    수직인 방향의 깊이가 안 잡힌다 (연속 전진 스캔이 814 mm 였던 이유).
  * TRAVERSE 중에도 관측을 계속 쌓는다 — 통과 구간이 곧 다음 창문의 스캔 구간이다.
  * 통과 직후 그 자리에서 다시 전체 스캔을 돈다 (RESCAN).

"복원 오차" 를 실비행에서 어떻게 재나
-------------------------------------
GT 가 없으므로 **추정 안정성** 을 쓴다: 최근 N 회 복원의 창문 중심 추정이 얼마나
흩어져 있는지 (성분별 표준편차의 노름). 수렴했으면 작고, 아직 흔들리면 크다.

    spread_mm > stability_thr_mm  →  아직 못 믿는다 → 재스캔

한계를 분명히 해 둔다: **틀린 값에 수렴해도 통과시킨다** (편향은 못 잡는다).
그래서 GT 오차를 항상 같이 로깅해서 이 대리 지표가 실제 오차와 얼마나 붙어
가는지 사후에 볼 수 있게 한다 — 대리 지표를 믿을 수 있는지 자체가 측정 대상이다.
"""
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from utils import paths  # noqa: E402

paths.bootstrap()

import planner  # noqa: E402
import traj  # noqa: E402
from control import qc  # noqa: E402
from control import simple as simple_ctl  # noqa: E402
from module import contract  # noqa: E402
from overrides import detections as ovd  # noqa: E402
from overrides import recon_rays  # noqa: E402
from utils import device, scene, viz  # noqa: E402


#: 기체 yaw 를 0 으로 고정한다. 이유 (300회 넘는 폐루프 스윕으로 배제한 결과):
#: 이 PyBullet 플랜트에서 yaw 는 제어 불가다. 자세(pitch/roll)는 추력 차이로 직접
#: 모멘트가 나오지만 yaw 는 프로펠러 반토크뿐인데, 그 반토크를 키우려면 모터 PI 를
#: 빠르게 해야 하고 그러면 자세가 먼저 무너진다 (kpMot 을 올린 48조합 전부 추락).
#: 추중비를 2.0 -> 7.1 로 올리면 오히려 더 나빠진다 (최대 yaw 8° -> 3°) — maxTorque 를
#: 키울수록 호버 명령이 작은 영역에 몰려 회전수 차동이 줄기 때문이다.
#: 근본 원인은 Cq=0.01517 이 "호버 평형을 맞추려고 역산한 값"이라 실기 반토크보다
#: 작다는 것 — 성진 제어기나 게인 문제가 아니라 우리 플랜트 근사의 한계다.
#:
#: 고정해도 되는 근거: 카메라 반화각이 가로 ±46.8° 인데 스캔 원호는 ±25° 다.
#: yaw 를 안 돌려도 창문이 화각 안에 남는다. 게다가 제자리 yaw 회전은 회전 중심이
#: 같아서 시차가 0 이라 삼각측량에 애초에 기여하지 않는다.
YAW_FIXED = 0.0


class State(Enum):
    SCAN = "SCAN"
    RECON = "RECON"
    APPROACH = "APPROACH"
    TRAVERSE = "TRAVERSE"
    RESCAN = "RESCAN"
    DONE = "DONE"


@dataclass
class Config:
    # --- 판정 ---------------------------------------------------------------
    stability_thr_mm: float = 80.0     # 이 값 이하로 수렴해야 통과 허용
    stability_window: int = 5          # 흩어짐을 볼 최근 복원 횟수
    max_scan_rounds: int = 1           # 한 창문에 허용할 스캔 라운드 (무한루프 방지)
    min_parallax_deg: float = 8.0      # 이보다 작으면 안정성과 무관하게 더 스캔
    # --- 스캔 모션 ----------------------------------------------------------
    scan_radius: float = 2.0           # 창문까지 **거리** [m]. 1.2 는 너무 가까워
                                   # 검출 conf 가 0.5 문턱 아래로 떨어진다
    scan_amp: float = 0.25             # 스캔 **진폭** [m]. 거리와 별개 값이다.
                                   # 예전엔 진폭을 거리(2.0 m)로 써서 yz 원호가
                                   # z=0.155 m (바닥) 를 명령했고 매번 추락했다.
    z_min: float = 0.6                 # 명령 z 하한 [m] — 기체 반경 0.159 m 고려
    z_max: float = 1.9
    fail_z_min: float = 0.25           # 실패 판정 — 느슨하면 추락해도 임무가 계속 돈다
    fail_z_max: float = 20.0
    fail_tilt_deg: float = 60.0
    fail_track_m: float = 1.0
    v_frac: float = 0.7                # 궤적 속도/가속 여유율 (샘플링 아티팩트 대비)
    a_frac: float = 0.5
    lat_frac: float = 0.35             # 원심가속 한계 배율 — 원호에서 이게 속도를 정한다
    lead_m: float = 0.8                # 첫 접근 목표를 게이트보다 더 앞에 둘 거리
    settle_s: float = 1.2              # 구간 끝 정지 유지 [s] — 위 hold_s 주석 참고
    # 모션당 웨이포인트 수. 24 는 과했고(0.5 m 원호가 63 s), 6 도 스캔 종료 상태가
    # 나빠서 직후 호버가 전복했다. 4 면 호버가 정상으로 돌아온다 — 실측이고 이유는
    # 아직 모른다 (각속도 |w| 는 오히려 4 쪽이 크다, 즉 |w| 는 판별자가 아니다).
    scan_pts: int = 4                  # (예전 주석) 24 는 과했다 —
                                   # flythrough 가 점마다 세그먼트를 풀어서
                                   # 0.5 m 원호가 63 s 로 늘어났다
    scan_span_deg: float = 50.0        # 원호 폭
    scan_kinds: Tuple[str, ...] = ("xy", "yz")   # yaw 제외 — 아래 YAW_FIXED 참고
    scan_seg_s: float = 2.0            # 모션 하나당 시간 [s]
    # --- 통과 ---------------------------------------------------------------
    # 게이트 standoff. 길남 planner_limits 는 1.5/1.0 인데 그건 창문 간격이
    # 충분히 넓다는 전제다. 여기서는 스캔 앵커(2.0 m)에서 게이트까지 거리가 짧아야
    # 구간 전환이 완만해진다 — 1.5 m 게이트로는 전환에서 매번 전복했다.
    d_app: float = 0.3
    d_exit: float = 0.3
    # --- 실행 ---------------------------------------------------------------
    dt: float = 0.001
    render_every: int = 40             # 제어 40스텝(=25 Hz)마다 카메라 1장
    det_conf_min: float = 0.5
    conf: float = 0.25


@dataclass
class WindowTrack:
    """창문 하나에 대한 복원 이력 + 안정성."""
    order_index: int
    history: List[np.ndarray] = field(default_factory=list)   # center 추정들
    last: Optional[dict] = None
    scan_rounds: int = 0

    def push(self, rec: dict) -> None:
        self.last = rec
        self.history.append(np.asarray(rec["center_w"], float))

    def spread_mm(self, n: int) -> float:
        """최근 n 회 추정의 흩어짐 [mm]. 표본이 모자라면 inf."""
        if len(self.history) < max(2, n):
            return float("inf")
        H = np.array(self.history[-n:])
        return float(np.linalg.norm(H.std(axis=0)) * 1000)

    def quality_mm(self, n: int) -> float:
        """판정용 품질 지표 [mm] = 흩어짐 + 삼각측량 잔차.

        흩어짐만 쓰면 안 된다: center_w 가 mm 로 반올림돼 있어 연속 추정이 같은
        값으로 나오고 흩어짐이 0.0 으로 죽는다 (실측: GT오차 4 mm 짜리와 5766 mm
        짜리가 둘 다 "흩어짐 0.0"). 잔차는 광선들이 한 점에서 얼마나 안 만나는지라
        수렴 여부와 무관하게 항상 정보를 준다.
        """
        if self.last is None:
            return float("inf")
        r = float(self.last.get("resid_mm", float("nan")))
        sp = self.spread_mm(n)
        if not np.isfinite(sp):
            return float("inf")
        return sp + (r if np.isfinite(r) else 0.0)

    def parallax(self) -> float:
        return float(self.last["min_parallax_deg"]) if self.last else 0.0


# --------------------------------------------------------------------------- #
# 스캔 모션 — 앵커 주변에서 창문을 보며 시차를 벌기
# --------------------------------------------------------------------------- #
def _yaw_towards(p, target) -> float:
    d = np.asarray(target, float) - np.asarray(p, float)
    return float(np.arctan2(d[1], d[0]))


def scan_waypoints(kind: str, anchor, target, cfg: Config, n: int = 6):
    """스캔 모션 한 종류 → (waypoints(n,3), yaws(n,)).

    xy   수평 원호 — 좌우 시차. 깊이(x) 추정에 제일 효과적
    yz   수직 원호 — 상하 시차. xy 만으로는 안 잡히는 세로 성분

    yaw 모션은 뺐다: 제자리 회전은 시차가 0 이고, 이 플랜트에선 yaw 자체가 제어
    불가다 (YAW_FIXED 주석 참고).
    """
    A = np.asarray(anchor, float)
    T = np.asarray(target, float)
    v = A - T
    r = float(np.linalg.norm(v[:2])) or cfg.scan_radius   # 창문까지 거리
    amp = float(cfg.scan_amp)                             # 스캔 진폭 (거리와 별개)
    u = np.linspace(-1.0, 1.0, n)

    if kind == "xy":
        # 창문 중심을 도는 원호. 진폭만큼 옆으로 흔들되 거리는 r 로 유지한다.
        dtheta = np.arcsin(np.clip(amp / max(r, 1e-6), -0.99, 0.99)) * u
        base = np.arctan2(v[1], v[0])
        pts = np.stack([T[0] + r * np.cos(base + dtheta),
                        T[1] + r * np.sin(base + dtheta),
                        np.full(n, A[2])], axis=1)
    elif kind == "yz":
        # 세로 시차. x·y 는 앵커 그대로 두고 z 만 진폭만큼 흔든다.
        z = np.clip(A[2] + amp * u, cfg.z_min, cfg.z_max)
        pts = np.stack([np.full(n, A[0]), np.full(n, A[1]), z], axis=1)
    else:
        raise ValueError(f"모르는 스캔 종류: {kind}")
    pts[:, 2] = np.clip(pts[:, 2], cfg.z_min, cfg.z_max)

    # yaw 는 전 구간 고정 (YAW_FIXED). 창문을 향해 돌리면 추락한다.
    yaws = np.full(len(pts), YAW_FIXED)
    return pts, yaws


# --------------------------------------------------------------------------- #
# 상태기계
# --------------------------------------------------------------------------- #
class TrajManager:
    def __init__(self, layout, cfg: Config, planner_cfg: dict, verbose=True):
        self.layout = list(layout)
        self.cfg = cfg
        self.pcfg = planner_cfg
        self.verbose = verbose
        self.state = State.SCAN
        self.idx = 0                                   # 지금 노리는 창문
        self.tracks: Dict[int, WindowTrack] = {}
        self.samples: List[dict] = []                  # 누적 관측 (전 구간)
        self.log: List[dict] = []                      # 상태 전이 기록

    # --- 관측/복원 --------------------------------------------------------
    def add_samples(self, new: List[dict]) -> None:
        self.samples.extend(new)

    def reconstruct(self) -> Dict[int, dict]:
        s = ovd.assign_order_by_passing(self.samples, self.layout) \
            if len(self.layout) > 3 else self.samples
        res = recon_rays.reconstruct(s, det_conf_min=self.cfg.det_conf_min)
        for r in res:
            oi = r["order_index"]
            self.tracks.setdefault(oi, WindowTrack(oi)).push(r)
        return {r["order_index"]: r for r in res}

    # --- 판정 -------------------------------------------------------------
    def target(self) -> Optional[dict]:
        return self.layout[self.idx] if self.idx < len(self.layout) else None

    def assess(self) -> Tuple[bool, str, float]:
        """현재 목표 창문이 통과해도 될 만큼 수렴했나. 반환 (ok, 사유, spread_mm)."""
        w = self.target()
        if w is None:
            return False, "목표 없음", float("inf")
        tr = self.tracks.get(w["order_index"])
        if tr is None or tr.last is None:
            return False, "복원 없음", float("inf")
        sp = tr.quality_mm(self.cfg.stability_window)
        if not np.isfinite(sp):
            return False, f"표본 {len(tr.history)}/{self.cfg.stability_window}", sp
        if self.cfg.stability_thr_mm <= 0:
            return True, "역치 미사용 — 복원 있으면 바로 통과", sp
        if tr.parallax() < self.cfg.min_parallax_deg:
            return False, f"시차각 {tr.parallax():.1f}° < {self.cfg.min_parallax_deg}", sp
        if sp > self.cfg.stability_thr_mm:
            return False, f"품질 {sp:.1f}mm > {self.cfg.stability_thr_mm}", sp
        return True, f"품질 {sp:.1f}mm 수렴", sp

    # --- 경로 생성 --------------------------------------------------------
    def scan_anchor(self, drone_pos) -> np.ndarray:
        """스캔을 돌 기준 위치. 목표 창문 앞 scan_radius 지점 (현재 z 유지)."""
        w = self.target()
        c = np.asarray(w["center"], float)
        # 접근측은 −x 다 (드론이 +x 로 난다). +를 쓰면 창문을 뚫고 지나가서
        # 뒤에서 스캔하게 된다 — 실제로 처음에 그렇게 짜서 복원이 0 이 나왔다.
        a = c - np.array([self.cfg.scan_radius, 0.0, 0.0])
        a[2] = float(np.clip(drone_pos[2], self.cfg.z_min, self.cfg.z_max))
        return a

    def scan_plan(self, drone_pos) -> List[Tuple[np.ndarray, np.ndarray]]:
        """스캔 모션들의 (waypoints, yaws) 리스트.

        원호는 **드론에 가까운 끝에서 진입**하도록 방향을 정한다. 안 그러면 먼 쪽
        끝으로 갔다가 원호를 따라 180° 되돌아오는 경로가 되고, 그 급반전에서
        제어기가 무너진다 (실측: 위치오차 1355 mm, 추락).
        """
        w = self.target()
        A = self.scan_anchor(drone_pos)
        T = np.asarray(w["center"], float)
        P = np.asarray(drone_pos, float)
        out = []
        for k in self.cfg.scan_kinds:
            pts, yaws = scan_waypoints(k, A, T, self.cfg, n=self.cfg.scan_pts)
            if np.linalg.norm(pts[-1] - P) < np.linalg.norm(pts[0] - P):
                pts, yaws = pts[::-1].copy(), yaws[::-1].copy()
            out.append((pts, yaws))
            P = pts[-1]              # 다음 모션은 이 끝에서 이어진다
        return out

    def approach_leg(self, drone_pos, frac: float = 0.55) -> np.ndarray:
        """첫 관측용 **접근 구간** — 복도를 따라 첫 창문 쪽으로 곧게 간다.

        원호 스캔을 여기에 두면 매번 전복했다. 반면 ``window_flight`` 의
        ``start -> approach0`` 구간은 3/3 통과를 낸 검증된 모양이다. 그래서
        같은 모양을 쓴다 — 첫 창문 게이트까지의 직선을 frac 만큼만 간다.

        원호가 필요 없다는 것도 실측으로 확인됐다: 통과 경로 자체가 시차각
        49~50° 를 준다 (원호는 ±25°). 관측 품질은 오히려 이쪽이 낫다.
        """
        P = np.asarray(drone_pos, float)
        w = self.target()
        c = np.asarray(w["center"], float)
        goal = c + np.array([self.cfg.d_app + self.cfg.lead_m, 0.0, 0.0])
        goal[2] = float(np.clip(c[2], self.cfg.z_min, self.cfg.z_max))
        return np.array([P + frac * (goal - P)])

    def scan_path(self, drone_pos) -> Tuple[np.ndarray, np.ndarray]:
        """스캔 모션 전부를 **하나의 연속 경로**로 이어 붙인다.

        xy 원호를 날고 멈췄다가 yz 원호를 새로 시작하면, 두 원호가 서로 다른
        평면에 있어서 전환이 급하다. 실측: xy 는 완주(창문 복원 8.7 mm)하는데
        yz 진입 2 샘플 만에 기울기 60° 로 전복했다. 성진 flythrough 는 원래
        여러 점을 정지 없이 통과하라고 만든 것이므로, 한 경로로 넘기면 전환
        구간이 그냥 곡선의 일부가 되고 v/a/j/snap 도 거기서 함께 보장된다.
        """
        segs = self.scan_plan(drone_pos)
        pts = np.vstack([p for p, _ in segs])
        yaws = np.concatenate([y for _, y in segs])
        return pts, yaws

    def traverse_plan(self, est: dict) -> Tuple[np.ndarray, float]:
        """복원된 창문으로 approach→exit 웨이포인트. 반환 (waypoints, yaw)."""
        wmap = [{"order_index": est["order_index"], "color": est["color"],
                 "center": est["center_w"], "corners_3d": est["corners_w"],
                 "size_wh": [est["width"], est["height"]]}]
        cfg = dict(self.pcfg)
        cfg["d_app"], cfg["d_exit"] = self.cfg.d_app, self.cfg.d_exit
        pl = planner.plan(wmap, start=(0, 0, 1), cfg=cfg, align=False)
        # start / stop 을 빼고 approach·exit 만 (이동은 호출측이 이어붙인다)
        pts = [q for lb, q in zip(pl.labels, pl.waypoints)
               if lb.startswith(("approach", "exit"))]
        return np.array(pts), 0.0

    def note(self, **kw) -> None:
        self.log.append(dict(state=self.state.value, idx=self.idx, **kw))
        if self.verbose:
            bits = "  ".join(f"{k}={v}" for k, v in kw.items())
            print(f"    [{self.state.value:9s}] 창문{self.idx}  {bits}")


# --------------------------------------------------------------------------- #
# 실행 — 상태기계가 낸 구간을 성진 제어기로 실제 비행하며 관측을 쌓는다
# --------------------------------------------------------------------------- #
class Runner:
    """PyBullet 씬 + 성진 제어기 + 검출기를 한 덩어리로."""

    def __init__(self, cfg: Config, seed=5, n_windows=3, spacing=2.6,
                 clutter=10, opening=1.0, weights=None, dev="auto",
                 render_scale=1.0, planner_cfg=None, controller="seoungjin",
                 traj_kind="mine"):
        import pybullet as p
        if paths.CONTROL not in sys.path:
            sys.path.insert(0, paths.CONTROL)      # 성진 path_time 임포트용
        from window_waypoint_planner import load_planner_config
        self.pcfg = planner_cfg or load_planner_config(paths.PLANNER_LIMITS)
        self.p = p
        self.cfg = cfg
        self.render_scale = render_scale
        self.det = device.load_detector(weights, prefer=dev)
        self.env, self.layout = scene.make(seed=seed, n_windows=n_windows,
                                           clutter=clutter, opening=opening,
                                           spacing=spacing)
        self.cid = self.env.CLIENT
        p.setTimeStep(cfg.dt, physicsClientId=self.cid)
        p.resetBasePositionAndOrientation(self.env.DRONE_IDS[0], [0, 0, -50],
                                          [0, 0, 0, 1], physicsClientId=self.cid)
        self.controller = controller
        self.traj_kind = traj_kind
        self.ctl = qc.Controller(
            alt_cmd_sat=qc.PYBULLET_TUNED["alt_cmd_sat"]).apply_tuned()
        # 파이프라인 검증용 대체 제어기 (control/simple.py 머리말 참고).
        # 성진 제어기는 구간 전환에서 전복해서 임무를 끝까지 못 돈다.
        self.simple = simple_ctl.SimpleController(
            mass=self.ctl.m_tot, I=(self.ctl.I_att, self.ctl.I_att, self.ctl.I_yaw))
        self.body = qc.make_body(p, self.cid, self.ctl.m_tot, self.ctl.I_att,
                                 self.ctl.I_yaw, self.ctl.r_arm, start=(0, 0, 1.0))
        self.intr = contract.intrinsics()
        self.T_IC = contract.T_imu_cam()
        self.t_ns = 0
        self.path: List[np.ndarray] = []
        self.contacts = 0
        self.fail_reason: Optional[str] = None
        self._spin_up()

    def _spin_up(self, s=1.5):
        p = self.p
        pos0, q0 = p.getBasePositionAndOrientation(self.body, physicsClientId=self.cid)
        if self.controller == "simple":
            return                      # 기하 제어기는 스핀업이 필요 없다
        for _ in range(int(s / self.cfg.dt)):
            self.ctl.step(pos0, 0.0, pos0, (0, 0, 0), self.cfg.dt)
            p.resetBasePositionAndOrientation(self.body, pos0, q0,
                                              physicsClientId=self.cid)
            p.resetBaseVelocity(self.body, [0, 0, 0], [0, 0, 0],
                                physicsClientId=self.cid)

    def pose(self):
        pos, quat = self.p.getBasePositionAndOrientation(self.body,
                                                         physicsClientId=self.cid)
        return np.asarray(pos, float), quat

    def _observe_now(self) -> Optional[dict]:
        """지금 자세에서 카메라 1장 → §5 샘플. 검출 없으면 detection=None."""
        from eval_recon3d import quat_xyzw_to_rot
        from sim import pybullet_stream as pbs
        pos, quat = self.pose()
        R_WB = quat_xyzw_to_rot(np.asarray(quat, float))
        fwd = R_WB[:, 0]                       # 카메라는 body +x 를 본다
        p_WC, q_WC = pbs.look_at_pose(pos, pos + fwd)
        img = pbs.render_frame(self.p, self.cid, p_WC, pos + fwd, self.intr,
                               scale=self.render_scale)
        msg = pbs.infer_frame_multiclass(self.det, img, self.t_ns,
                                         len(self.path), self.cfg.conf)
        self.t_ns += int(1e9 / 30)
        R_WC = quat_xyzw_to_rot(np.asarray(q_WC, float))
        R_WI, p_WI = contract.camera_pose_to_imu_pose(R_WC, p_WC, self.T_IC)
        return {"t_ns": self.t_ns,
                "p_WI": [float(v) for v in p_WI],
                "q_WI_xyzw": [float(v) for v in pbs._rot_to_quat_xyzw(R_WI)],
                "detection": msg if msg["windows"] else None}

    def fly(self, waypoints, yaws=None, label="", kind=None) -> Tuple[List[dict], bool]:
        """웨이포인트를 따라 비행하며 주기적으로 관측. 반환 (samples, 정상종료)."""
        W = np.atleast_2d(np.asarray(waypoints, float))
        pos_now, _ = self.pose()
        W = np.vstack([pos_now, W])                    # 현 위치에서 이어붙임
        if yaws is not None:
            yaws = np.concatenate([[yaws[0]], np.asarray(yaws, float)])
            yaws = np.unwrap(yaws)

        # 성진 path_time.plan_waypoints_flythrough — v/a/j/snap 을 **축별로**
        # 7차 다항식 차원에서 보장하고 웨이포인트를 정지 없이 정확 통과한다.
        # 내 traj.build 는 시간을 v_max 로만 정해서 a_max 를 1.7배 넘겼고
        # (0.5 m 를 0.94 s 에 = 2.73 m/s^2 vs a_max 1.6), 그 기준을 쫓다가
        # 자세 루프가 0.2 초 만에 발산했다.
        # hold_s 를 넉넉히 준다. flythrough 는 v0(시작 속도)를 인자로 못 받고 항상
        # 정지 상태를 가정하므로, 다음 구간이 시작될 때 v0=0 이 **실제로 참이어야**
        # 한다. 0.3 s 로는 기체가 못 멈춰서 다음 구간 첫 샘플부터 어긋났고,
        # 그걸 쫓다가 APPROACH 진입에서 추락했다 (z 1.0 -> 0.04).
        # 우리 곡률 제한 생성기. 그의 flythrough 는 원호를 점으로 쪼갤수록
        # 감속을 안 해서(꺾임각이 작아짐) 점 개수가 물리를 바꿨다 — 4점이면 살고
        # 6점이면 전복. build_capped 는 곡률로 속도를 묶어서 점 개수와 무관하다.
        # 구간마다 맞는 생성기가 다르다 (실측):
        #   스캔(짧은 원호 2.2 m) -> seoungjin.  내 build 는 짧은 경로에서 a_max 를
        #     넘겨(2.7 vs 1.6) 자세가 발산한다.
        #   통과(긴 직선 10 m)   -> mine.  window_flight 가 이걸로 3/3 (RMS 37 mm).
        #     seoungjin 은 게이트 사이를 크게 돌아 개구부 여유를 못 채운다.
        kind = kind or self.traj_kind
        if kind == "capped":
            t, ref, refv, L, T = traj.build_capped(
                W, self.pcfg, dt=self.cfg.dt, hold_s=self.cfg.settle_s,
                v_frac=self.cfg.v_frac, a_frac=self.cfg.a_frac,
                lat_frac=self.cfg.lat_frac)
        elif kind == "seoungjin":
            t, ref, refv, L, T = traj.build_seoungjin(
                W, self.pcfg, dt=self.cfg.dt, hold_s=self.cfg.settle_s)
        else:
            # window_flight.py 가 3/3 통과(RMS 37 mm)를 낸 그 생성기·파라미터.
            t, ref, refv, L, T = traj.build(
                W, dt=self.cfg.dt, v_max=self.pcfg["limits"]["v_max"],
                v_frac=0.6, smooth_m=0.25, smooth_t=0.35,
                hold_s=self.cfg.settle_s)
        if yaws is not None:
            u = np.linspace(0, 1, len(yaws))
            ry = np.interp(np.linspace(0, 1, len(ref)), u, yaws)
        else:
            ry = np.zeros(len(ref))

        out = []
        for k in range(len(ref)):
            pos, quat = self.pose()
            rpy = self.p.getEulerFromQuaternion(quat)
            if self.controller == "simple":
                self.simple.apply(self.p, self.body, ref[k], float(ry[k]), self.cid,
                                  ref_vel=refv[k])
            else:
                _, w_ang = self.p.getBaseVelocity(self.body, physicsClientId=self.cid)
                th, dq = self.ctl.step(ref[k], float(ry[k]), pos, rpy, self.cfg.dt,
                                       omega=w_ang)
                qc.apply_to_body(self.p, self.body, th, dq, self.ctl.motor_xy,
                                 self.ctl.mix_dir, self.cid)
            self.p.stepSimulation(physicsClientId=self.cid)
            self.path.append(pos)
            if self.p.getContactPoints(bodyA=self.body, physicsClientId=self.cid):
                self.contacts += 1
            if k % self.cfg.render_every == 0:
                out.append(self._observe_now())
            why = self._failed(pos, rpy, ref[k])
            if why:
                self.fail_reason = why
                return out, False
        return out, True

    def _failed(self, pos, rpy, ref) -> Optional[str]:
        """비행 실패 판정. 예전엔 |z|>40 만 봐서 **바닥에 처박혀도 정상 판정**이
        나왔다 (z=0.04 는 유한하고 40 미만). 그 상태로 상태기계가 끝까지 돌면서
        "12구간 정상" 을 찍었고, 카메라는 수평을 보니 검출은 100% 로 계속 나왔다
        (창문이 z=0.28 부터라 바닥에서도 보인다). 조용히 성공한 척한 것이다."""
        if not np.all(np.isfinite(pos)):
            return "비유한 좌표"
        if pos[2] < self.cfg.fail_z_min:
            return f"지면 접촉 z={pos[2]:.2f}m"
        if pos[2] > self.cfg.fail_z_max:
            return f"고도 이탈 z={pos[2]:.2f}m"
        tilt = max(abs(np.degrees(rpy[0])), abs(np.degrees(rpy[1])))
        if tilt > self.cfg.fail_tilt_deg:
            return f"전복 기울기 {tilt:.0f}deg"
        e = float(np.linalg.norm(np.asarray(pos) - np.asarray(ref)))
        if e > self.cfg.fail_track_m:
            return f"기준 이탈 {e*1000:.0f}mm"
        return None

    def close(self):
        self.env.close()


# --------------------------------------------------------------------------- #
# 상태 루프
# --------------------------------------------------------------------------- #
def run_mission(rn: "Runner", mg: "TrajManager", max_segments: int = 60) -> dict:
    """상태기계를 끝까지 돌린다. 반환 요약 dict."""
    gt = {w["order_index"]: np.asarray(w["center"], float) for w in mg.layout}
    trace: List[dict] = []
    alive = True
    seg = 0

    while mg.state is not State.DONE and seg < max_segments and alive:
        w = mg.target()
        if w is None:
            mg.state = State.DONE
            break

        # ---- SCAN / RESCAN : xy → yz → yaw 전부 -------------------------
        if mg.state in (State.SCAN, State.RESCAN):
            tr = mg.tracks.setdefault(w["order_index"], WindowTrack(w["order_index"]))
            tr.scan_rounds += 1
            pos, _ = rn.pose()
            pts, yaws = mg.scan_path(pos)          # xy+yz 를 한 경로로
            new, alive = rn.fly(pts, yaws, label="scan", kind="seoungjin")
            mg.add_samples(new)
            seg += 1
            est = mg.reconstruct()
            mg.note(경로점=len(pts), 관측=len(new), 복원=sorted(est))
            if not alive:
                mg.note(임무중단=rn.fail_reason)
            n_det = sum(1 for x in mg.samples if x.get("detection"))
            mg.note(라운드=tr.scan_rounds, 누적관측=len(mg.samples),
                    검출프레임=n_det)
            mg.state = State.RECON
            continue

        # ---- RECON : 복원 + 안정성 판정 ----------------------------------
        if mg.state is State.RECON:
            est = mg.reconstruct()
            ok, why, sp = mg.assess()
            oi = w["order_index"]
            e_gt = (np.linalg.norm(np.asarray(est[oi]["center_w"], float) - gt[oi]) * 1000
                    if oi in est else float("nan"))
            trace.append({"seg": seg, "window": oi, "spread_mm": sp,
                          "gt_err_mm": e_gt, "ok": ok,
                          "parallax": mg.tracks[oi].parallax() if oi in mg.tracks else 0})
            mg.note(판정=("통과" if ok else "재스캔"), 사유=why,
                    GT오차=f"{e_gt:.0f}mm")
            tr = mg.tracks.get(oi)
            if ok:
                mg.state = State.APPROACH
            elif tr and tr.scan_rounds >= mg.cfg.max_scan_rounds:
                if tr.last is None:
                    mg.note(판정="복원 실패 — 이 창문 건너뜀")
                    mg.idx += 1
                    mg.state = State.DONE if mg.idx >= len(mg.layout) else State.SCAN
                else:
                    mg.note(판정="한도 도달 — 현재 추정으로 진행")
                    mg.state = State.APPROACH
            else:
                mg.state = State.SCAN
            continue

        # ---- APPROACH + TRAVERSE : 게이트로 이동 후 통과 ------------------
        if mg.state is State.APPROACH:
            oi = w["order_index"]
            est = mg.tracks[oi].last
            pts, _ = mg.traverse_plan(est)
            yaw0 = YAW_FIXED
            new, alive = rn.fly(pts[:1], np.array([yaw0]), label="approach")
            mg.add_samples(new); seg += 1
            mg.note(게이트=f"[{pts[0,0]:.2f},{pts[0,1]:.2f},{pts[0,2]:.2f}]")
            if not alive:
                mg.note(임무중단=rn.fail_reason)
                break
            mg.state = State.TRAVERSE
            new, alive = rn.fly(pts[1:], np.array([yaw0]), label="traverse")
            mg.add_samples(new); seg += 1          # 통과 중에도 관측 누적
            mg.note(관측=len(new), 누적=len(mg.samples))
            if not alive:
                mg.note(임무중단=rn.fail_reason)
                break
            mg.state = State.RESCAN                # 통과 직후 즉시 재스캔
            mg.idx += 1
            if mg.idx >= len(mg.layout):
                mg.state = State.DONE
            continue

    return {"trace": trace, "alive": alive, "segments": seg,
            "contacts": rn.contacts, "samples": len(mg.samples),
            "fail_reason": rn.fail_reason}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--n-windows", type=int, default=3)
    ap.add_argument("--spacing", type=float, default=2.6)
    ap.add_argument("--clutter", type=int, default=10)
    ap.add_argument("--weights", default=None)
    ap.add_argument("--device", default="auto", choices=("auto", "cuda", "cpu"))
    ap.add_argument("--stability-thr", type=float, default=80.0,
                    help="중심 추정 흩어짐 역치 [mm] — 이하면 통과, 초과면 재스캔")
    ap.add_argument("--stability-window", type=int, default=5)
    ap.add_argument("--min-parallax", type=float, default=8.0)
    ap.add_argument("--max-scan-rounds", type=int, default=1)
    ap.add_argument("--scan-span", type=float, default=50.0)
    ap.add_argument("--scan-radius", type=float, default=2.0, help="창문까지 거리 [m]")
    ap.add_argument("--scan-amp", type=float, default=0.25, help="스캔 진폭 [m]")
    ap.add_argument("--scan-pts", type=int, default=4, help="스캔 모션당 웨이포인트 수")
    ap.add_argument("--mode", default="single", choices=("single", "staged"),
                    help="single = 스캔1 + 전창문통과1 (경계 1개)")
    ap.add_argument("--traj", default="mine", choices=("mine", "capped", "seoungjin"))
    ap.add_argument("--controller", default="seoungjin",
                    choices=("seoungjin", "simple"),
                    help="simple = 표준 기하 제어기 (파이프라인 검증용 대체)")
    ap.add_argument("--v-frac", type=float, default=0.7)
    ap.add_argument("--a-frac", type=float, default=0.5)
    ap.add_argument("--lat-frac", type=float, default=0.35)
    ap.add_argument("--d-app", type=float, default=0.3, help="창문 앞 게이트 거리 [m]")
    ap.add_argument("--d-exit", type=float, default=0.3, help="창문 뒤 게이트 거리 [m]")
    ap.add_argument("--render-every", type=int, default=60)
    ap.add_argument("--render-scale", type=float, default=1.0)
    ap.add_argument("--fig", default=os.path.join(paths.FIG_DIR, "v2_10_mission.png"))
    a = ap.parse_args(argv)

    from window_waypoint_planner import load_planner_config
    pcfg = load_planner_config(paths.PLANNER_LIMITS)

    cfg = Config(stability_thr_mm=a.stability_thr,
                 stability_window=a.stability_window,
                 min_parallax_deg=a.min_parallax,
                 max_scan_rounds=a.max_scan_rounds,
                 scan_span_deg=a.scan_span, scan_radius=a.scan_radius,
                 scan_amp=a.scan_amp, scan_pts=a.scan_pts,
                 render_every=a.render_every,
                 d_app=a.d_app, d_exit=a.d_exit,
                 v_frac=a.v_frac, a_frac=a.a_frac, lat_frac=a.lat_frac)

    print(f"제어기: {a.controller}"
          + ("   (⚠ 성진 제어기 아님 — control/simple.py 참고)"
             if a.controller == "simple" else ""))
    print(f"게이트 standoff  앞 {cfg.d_app} m / 뒤 {cfg.d_exit} m")
    print(f"역치: 흩어짐 ≤ {cfg.stability_thr_mm} mm (최근 {cfg.stability_window}회), "
          f"시차각 ≥ {cfg.min_parallax_deg}°, 스캔 한도 {cfg.max_scan_rounds}라운드")
    rn = Runner(cfg, seed=a.seed, n_windows=a.n_windows, spacing=a.spacing,
                clutter=a.clutter, weights=a.weights, dev=a.device,
                render_scale=a.render_scale, planner_cfg=pcfg,
                controller=a.controller, traj_kind=a.traj)
    scene.print_layout(rn.layout)
    mg = TrajManager(rn.layout, cfg, pcfg)

    print("\n임무 시작")
    summary = (run_mission_single(rn, mg, a.traj) if a.mode == "single"
               else run_mission(rn, mg))

    gt = {w["order_index"]: np.asarray(w["center"], float) for w in rn.layout}
    print(f"\n구간 {summary['segments']}개, 관측 {summary['samples']}장, "
          f"충돌 {summary['contacts']} 스텝, "
          f"{'정상' if summary['alive'] else '임무 중단: ' + str(summary['fail_reason'])}")
    print("\n창문별 최종 복원:")
    for w in rn.layout:
        oi = w["order_index"]
        tr = mg.tracks.get(oi)
        if tr is None or tr.last is None:
            print(f"  #{oi} {w['color']:6s}  복원 실패")
            continue
        e = np.linalg.norm(np.asarray(tr.last["center_w"], float) - gt[oi]) * 1000
        print(f"  #{oi} {w['color']:6s}  GT오차 {e:7.1f} mm   "
              f"품질 {tr.quality_mm(cfg.stability_window):7.1f} mm   "
              f"시차각 {tr.parallax():5.1f}°   스캔 {tr.scan_rounds}라운드")

    tr_rows = summary["trace"]
    if tr_rows:
        sp = np.array([r["spread_mm"] for r in tr_rows])
        ge = np.array([r["gt_err_mm"] for r in tr_rows])
        m = np.isfinite(sp) & np.isfinite(ge)
        if m.sum() >= 3:
            print(f"\n대리지표 검증: 흩어짐 vs GT오차 상관 "
                  f"{np.corrcoef(sp[m], ge[m])[0,1]:+.2f}  (표본 {m.sum()})")

    if summary.get("plan") is not None:
        P = np.array(rn.path[summary["path_from"]:])
        rows = passage_report(P, rn.layout, pcfg["clearance_margin"])
        print("\n[통과 판정]  u,v = 개구부 중심 기준")
        nok = 0
        for r in rows:
            if not r["passed"]:
                print(f"  {r['color']:6s}  평면 통과 안 함"); continue
            nok += int(r["ok"])
            print(f"  {r['color']:6s}  u={r['u']:+.3f} v={r['v']:+.3f} m   "
                  f"여유 {r['slack']*1000:6.1f} mm   "
                  f"{'통과' if r['ok'] else '여유 미달'}")
        print(f"  -> {nok}/{len(rows)} 창문 안전 통과")
    _plot(a.fig, rn, mg, tr_rows, cfg)
    rn.close()
    return 0 if summary["alive"] and summary["contacts"] == 0 else 1


def _plot(fig_path, rn, mg, trace, cfg):
    viz.use_agg()
    import matplotlib.pyplot as plt
    P = np.array(rn.path)
    fig, ax = plt.subplots(3, 1, figsize=(13, 10))
    for i, (iy, nm) in enumerate(((1, "y [m]"), (2, "z [m]"))):
        ax[i].plot(P[:, 0], P[:, iy], lw=1.0, color="#06c", label="flown")
        for w in mg.layout:
            c = np.asarray(w["center"], float)
            half = (w["ow"] if iy == 1 else w["oh"]) / 2
            ax[i].plot([c[0]] * 2, [c[iy] - half, c[iy] + half], lw=6,
                       color=viz.COL[w["color"]], solid_capstyle="butt")
            tr = mg.tracks.get(w["order_index"])
            if tr and tr.last:
                e = np.asarray(tr.last["center_w"], float)
                ax[i].scatter([e[0]], [e[iy]], s=60, marker="x", color="#000", zorder=6)
        ax[i].set_ylabel(nm); ax[i].grid(alpha=.3)
    ax[0].legend(fontsize=8)
    ax[0].set_title("SCAN(xy+yz+yaw) -> RECON -> APPROACH -> TRAVERSE -> RESCAN",
                    fontsize=11)
    ax[1].set_xlabel("x [m]")
    if trace:
        k = np.arange(len(trace))
        ax[2].plot(k, [r["spread_mm"] for r in trace], "-o", ms=4,
                   color="#e60", label="estimate spread (runtime proxy)")
        ax[2].plot(k, [r["gt_err_mm"] for r in trace], "-s", ms=4,
                   color="#333", label="true error vs GT")
        ax[2].axhline(cfg.stability_thr_mm, ls="--", color="#c33", lw=1.3,
                      label=f"threshold {cfg.stability_thr_mm:.0f} mm")
        ax[2].set_yscale("log"); ax[2].set_xlabel("RECON decision #")
        ax[2].set_ylabel("[mm]"); ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
    fig.suptitle("Mission state machine — scan / reconstruct / traverse", fontsize=12)
    viz.save(fig, os.path.dirname(fig_path), os.path.basename(fig_path), dpi=115)




# --------------------------------------------------------------------------- #
# 단일 궤적 임무 — 비행 2회 (스캔 1 + 통과 1)
# --------------------------------------------------------------------------- #
def run_mission_single(rn: "Runner", mg: "TrajManager", traj_kind: str = "mine"):
    mg.cfg_traj = traj_kind
    """스캔 1회 → 복원 → **전 창문 통과를 한 궤적으로** 1회.

    왜 이 형태인가
    --------------
    구간을 나눠 날면 매 경계에서 전복한다. 궤적 생성기가 "정지 상태에서 출발" 을
    가정하는데 실제 기체는 직전 구간의 잔여 속도·회전을 갖고 있기 때문이다.
    성진 제어기는 각속도 피드백이 없어서 그걸 못 죽이고, 우리 기하 제어기도
    자세 루프 대역이 모자란다.

    반면 ``window_flight.py`` 는 웨이포인트 8개를 **한 궤적으로** 날려서 3/3 통과
    (RMS 37 mm, 충돌 0) 를 냈다. 차이는 오직 경계 개수다.

    그래서 경계를 1 개로 줄인다: 창문을 보려면 스캔이 먼저여야 하니 그것만 떼고,
    그 뒤 approach/exit 전부를 한 궤적에 담는다. 통과 중에도 관측은 계속 쌓인다.
    """
    gt = {w["order_index"]: np.asarray(w["center"], float) for w in mg.layout}
    trace: List[dict] = []

    # ---- 1. 접근 구간 (원호 스캔 대신 복도를 따라 곧게) --------------------
    pos, _ = rn.pose()
    pts = mg.approach_leg(pos)
    new, alive = rn.fly(pts, np.zeros(len(pts)), label="approach-leg",
                        kind=mg.cfg_traj)
    mg.add_samples(new)
    est = mg.reconstruct()
    mg.state = State.RECON
    mg.note(단계="접근구간",
            목표=f"[{pts[-1][0]:.2f},{pts[-1][1]:.2f},{pts[-1][2]:.2f}]",
            관측=len(new), 복원=sorted(est))
    if not alive:
        mg.note(임무중단=rn.fail_reason)
        return {"trace": trace, "alive": False, "segments": 1,
                "contacts": rn.contacts, "samples": len(mg.samples),
                "fail_reason": rn.fail_reason, "plan": None, "path_from": 0}

    for oi, r in est.items():
        e = float(np.linalg.norm(np.asarray(r["center_w"], float) - gt[oi])) * 1000
        trace.append({"window": oi, "gt_err_mm": e,
                      "spread_mm": mg.tracks[oi].quality_mm(mg.cfg.stability_window),
                      "parallax": r["min_parallax_deg"]})
        mg.note(창문=oi, GT오차=f"{e:.0f}mm", 시차각=f"{r['min_parallax_deg']:.0f}deg")

    if not est:
        mg.note(임무중단="복원 0")
        return {"trace": trace, "alive": True, "segments": 1,
                "contacts": rn.contacts, "samples": len(mg.samples),
                "fail_reason": "복원 0", "plan": None, "path_from": 0}

    # ---- 2. 전 창문 통과 계획 (한 번에) -----------------------------------
    wmap = [{"order_index": r["order_index"], "color": r["color"],
             "center": r["center_w"], "corners_3d": r["corners_w"],
             "size_wh": [r["width"], r["height"]]} for r in est.values()]
    pcfg = dict(mg.pcfg)
    pcfg["d_app"], pcfg["d_exit"] = mg.cfg.d_app, mg.cfg.d_exit
    pos, _ = rn.pose()
    pl = planner.plan(wmap, start=tuple(pos), cfg=pcfg)
    mg.state = State.TRAVERSE
    mg.note(단계="통과계획", 웨이포인트=len(pl.waypoints), 상태=planner.describe(pl))

    # ---- 3. 한 궤적으로 비행 ----------------------------------------------
    path_from = len(rn.path)
    W = np.array(pl.waypoints)
    new, alive = rn.fly(W[1:], np.zeros(len(W) - 1), label="traverse-all",
                        kind=mg.cfg_traj)
    mg.add_samples(new)
    mg.note(단계="통과비행", 관측=len(new), 결과=("완주" if alive else rn.fail_reason))

    est2 = mg.reconstruct()
    for oi, r in est2.items():
        e = float(np.linalg.norm(np.asarray(r["center_w"], float) - gt[oi])) * 1000
        mg.note(창문=oi, 통과후GT오차=f"{e:.0f}mm")

    return {"trace": trace, "alive": alive, "segments": 2,
            "contacts": rn.contacts, "samples": len(mg.samples),
            "fail_reason": rn.fail_reason, "plan": pl, "path_from": path_from}


def passage_report(path: np.ndarray, layout, margin: float):
    """궤적이 각 창문 개구부를 실제로 통과했는지 (window_flight.py 와 같은 판정)."""
    UP = np.array([0.0, 0.0, 1.0])
    n = np.array([-1.0, 0.0, 0.0])
    wa = np.cross(UP, n); wa /= np.linalg.norm(wa)
    rows = []
    for w in layout:
        c = np.asarray(w["center"], float)
        d = (path - c) @ n
        hit = None
        for k in range(len(d) - 1):
            if d[k] * d[k + 1] < 0:
                a = d[k] / (d[k] - d[k + 1])
                hit = path[k] + a * (path[k + 1] - path[k])
                break
        if hit is None:
            rows.append({"color": w["color"], "passed": False, "ok": False})
            continue
        du, dv = float((hit - c) @ wa), float((hit - c) @ UP)
        hw, hh = w["ow"] / 2, w["oh"] / 2
        rows.append({"color": w["color"], "passed": True, "u": du, "v": dv,
                     "slack": min(hw - abs(du), hh - abs(dv)),
                     "ok": abs(du) <= hw - margin and abs(dv) <= hh - margin})
    return rows


if __name__ == "__main__":
    raise SystemExit(main())
