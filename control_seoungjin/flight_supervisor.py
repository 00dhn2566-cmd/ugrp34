"""비행 감독자(flight supervisor) — INTERFACE_SPEC §9 v0.2 골격.

비행을 장악하는 단일 프로세스 (PX4 commander 패턴, 사용자 제안 2026-07-18):
모든 미션·비상 명령·컨트롤러 통보를 받아 수락/거부/모드 전환을 판단한다.

    [상위 RL/조종] --미션·비상명령--> [감독자] --수락된 미션--> [궤적 파이프라인]
    [컨트롤러] --이벤트·current_state--> [감독자]     └--궤적--> [컨트롤러]

철칙 3개 (§9 아키텍처 절):
    1. 결정 경로에만 — 감독자는 수 Hz 판단 루프. 30Hz+ 제어는 컨트롤러 내부,
       감독자 비경유.
    2. 반사는 컨트롤러 내장 — B(회생)/C(믹서 배분) 트리거는 컨트롤러가 즉시
       자체 수행하고 감독자에 "통보"만 한다 (뇌/척수 분담). 감독자는 통보를
       받아 mode를 갱신하고 이후 명령을 게이트.
    3. 감독자 부재 시 안전 강하 — flight_state.json의 written_at(하트비트)이
       HEARTBEAT_TIMEOUT_S 이상 끊기면 컨트롤러는 현행 궤적 완주 후 §1
       무명령 default(현재 자리 래치 호버)로 강하. 컨트롤러 측 감시의 기준
       구현은 heartbeat_stale() — C++ 이식은 튜닝/C++ 세션과 협의(보드 ★).

mode의 단일 소유자 = 이 프로세스. flight_state.json(§0 RT 경로, 원자적 쓰기):
    {written_at, mode, active_traj_hash, reason}
    mode: normal | recovering | hover_latched | emergency_stopping | power_degraded
current_state.json(§5)은 물리 상태 보고 전용으로 남는다 (소유권 분리:
컨트롤러=물리, 감독자=판단).

우선순위 (감독자 집행): B > C > A-1 > A-2 > 일반 명령.
낮은 우선순위는 활성 중인 높은 우선순위를 선점하지 못한다.

입력 채널 (골격 v0.1 — 구현 세션 결정: 이벤트는 append 파일):
    emergency_cmd.json     상위 선언형 (A-1 stop / A-2 keep_out_update).
                           written_at가 바뀐 경우만 1회 처리 (중복 방지).
    controller_events.jsonl 컨트롤러 통보 (B/C 반사 진입·해제, 정지 완료 등).
                           append-only, 감독자는 오프셋 이후 새 줄만 소비.

이 골격은 판단·게이트·상태 공표까지만 담당한다. 실제 정지 궤적 생성(§8
emergency 동사)·회피 재계획 호출은 tick()이 돌려주는 action 목록을 보고
러너(또는 후속 단계 코드)가 수행한다 — 감독자가 제어 루프를 만지지 않는
철칙 1의 코드화.

사용:
    python flight_supervisor.py            # 5Hz 판단 루프 (Ctrl+C 종료)
"""

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_pipeline import (                     # noqa: E402
    KEEP_OUT_PATH,
    OUTPUT_DIR,
    _atomic_write_json,
    _parse_ts,
    _rt_dir,
)

PIPELINE_PY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "traj_pipeline.py")

# ---------------------------------------------------------------------------
# 계약 상수 (§9)
# ---------------------------------------------------------------------------

MODES = ("normal", "recovering", "hover_latched",
         "emergency_stopping", "power_degraded")

# 컨트롤러 측 하트비트 감시 임계 (철칙 3). C++/Simulink 이식 시 이 값 참조.
HEARTBEAT_TIMEOUT_S = 1.0

# 비상 우선순위 (§9 확정: B > C > A-1 > A-2 > 일반). 숫자가 클수록 우선.
PRIORITY = {"B": 4, "C": 3, "A1": 2, "A2": 1, None: 0}

# 미션 게이트가 닫히는 모드 (비상 진행 중 — REJECTED_RECOVERING 계열)
GATED_MODES = ("recovering", "emergency_stopping", "power_degraded")

TS_FMT_MS = "%Y-%m-%dT%H-%M-%S.%f"

FLIGHT_STATE_PATH = os.path.join(_rt_dir(), "flight_state.json")
CONTROLLER_EVENTS_PATH = os.path.join(_rt_dir(), "controller_events.jsonl")


def _io_root():
    """명령 채널 루트 (§0): env UGRP_IO_ROOT/active -> 개발 기본 output/ 플랫."""
    root = os.environ.get("UGRP_IO_ROOT")
    if root:
        return os.path.join(root, "active")
    return OUTPUT_DIR


EMERGENCY_CMD_PATH = os.path.join(_io_root(), "emergency_cmd.json")
LEDGER_PATH = os.path.join(OUTPUT_DIR, "feedback_ledger.jsonl")


def _now_str(now=None):
    return (now or datetime.now()).strftime(TS_FMT_MS)[:-3]


def heartbeat_stale(path=None, now=None, timeout_s=HEARTBEAT_TIMEOUT_S):
    """컨트롤러 측 하트비트 감시 기준 구현 (철칙 3).

    flight_state.json이 없거나, 깨졌거나, written_at 나이 > timeout_s면 True.
    True를 본 컨트롤러의 의무: 현행 궤적 완주 후 현재 자리 1회 래치 호버
    (§1 무명령 default). C++ 이식 시 이 판정과 동일해야 한다.
    """
    path = path or FLIGHT_STATE_PATH
    if not os.path.isfile(path):
        return True
    try:
        with open(path, encoding="utf-8") as f:
            st = json.load(f)
        age = ((now or datetime.now())
               - _parse_ts(st["written_at"])).total_seconds()
    except (ValueError, KeyError, json.JSONDecodeError):
        return True
    return age > timeout_s


class FlightSupervisor:
    """mode 단일 소유자 + 명령 게이트 + 우선순위 중재 (골격).

    테스트/러너가 tick()을 호출할 때마다 명령·이벤트를 소비하고 하트비트를
    공표한다. 실행할 일(정지 궤적 생성 등)은 action dict 목록으로 반환.
    """

    def __init__(self, flight_state_path=None, emergency_cmd_path=None,
                 controller_events_path=None, ledger_path=None,
                 keep_out_path=None, current_state_path=None,
                 w_cmd_sat_rad_s=None):
        self.flight_state_path = flight_state_path or FLIGHT_STATE_PATH
        self.emergency_cmd_path = emergency_cmd_path or EMERGENCY_CMD_PATH
        self.controller_events_path = (controller_events_path
                                       or CONTROLLER_EVENTS_PATH)
        self.ledger_path = ledger_path or LEDGER_PATH
        # A-2 구역 영속화: 파이프라인(emergency 동사 --keep-out 기본값)과
        # 같은 파일을 쓴다 — 감독자가 쓰고 파이프라인이 읽는 단방향
        self.keep_out_path = keep_out_path or KEEP_OUT_PATH
        self.current_state_path = current_state_path   # None = §0 RT 기본

        self.mode = "normal"
        self.reason = "boot"
        self.active_traj_hash = None
        self.active_kind = None          # 활성 비상 종류: "B"|"C"|"A1"|None
        self.keep_out = None             # A-2 최신 구역 정의 (zones/inflate_m)
        self._last_cmd_written_at = None  # emergency_cmd 중복 처리 방지
        self._events_offset = 0          # controller_events.jsonl 소비 오프셋
        # C-모드 트리거 감시 (§9 잠정: 포화율>90% 1s 지속 AND 고도 오차 증가).
        # 포화 기준 w_cmd_sat_rad_s는 실측 확정 전 — None이면 감시 비활성.
        self.power_monitor = (_PowerDegradedMonitor(w_cmd_sat_rad_s)
                              if w_cmd_sat_rad_s else None)

        os.makedirs(os.path.dirname(self.flight_state_path), exist_ok=True)

    # -- 상태 공표 ----------------------------------------------------------

    def publish(self, now=None):
        """flight_state.json 원자적 쓰기 = 하트비트 (written_at 갱신)."""
        _atomic_write_json(self.flight_state_path, {
            "written_at": _now_str(now),
            "mode": self.mode,
            "active_traj_hash": self.active_traj_hash,
            "reason": self.reason,
        })

    def _set_mode(self, mode, reason, kind):
        if mode not in MODES:
            raise ValueError(f"모드 아님: {mode}")
        self.mode = mode
        self.reason = reason
        self.active_kind = kind

    def _ledger(self, event, detail, now=None):
        """비상 이벤트를 원장에 기록 (append-only, 추가만 — 기존 스키마 불변)."""
        os.makedirs(os.path.dirname(self.ledger_path), exist_ok=True)
        with open(self.ledger_path, "a", encoding="utf-8") as f:
            f.write(json.dumps({
                "event": event, "at": _now_str(now),
                "mode": self.mode, "traj_hash": self.active_traj_hash,
                "detail": detail,
            }, ensure_ascii=False) + "\n")

    # -- 명령 게이트 (§9: 비상 중 미션 거부) --------------------------------

    def submit_mission(self, mission_ref, traj_hash=None, now=None):
        """상위 미션 수락/거부. 수락 시 mode를 normal로 (hover_latched 해제).

        비상 진행 중(GATED_MODES)이면 REJECTED_RECOVERING으로 거부 —
        reason에 어느 비상인지 실린다 (§9 C-모드: reason=power_degraded).
        """
        if self.mode in GATED_MODES:
            return {"accepted": False,
                    "reject_code": "REJECTED_RECOVERING",
                    "reason": self.mode}
        self._set_mode("normal", f"mission_accepted:{mission_ref}", None)
        if traj_hash is not None:
            self.active_traj_hash = traj_hash
        self.publish(now)
        return {"accepted": True, "reject_code": None, "reason": None}

    # -- 우선순위 중재 -------------------------------------------------------

    def _preempts(self, kind):
        """kind 비상이 현재 활성 비상을 선점할 수 있는가 (B > C > A1 > A2)."""
        return PRIORITY[kind] > PRIORITY[self.active_kind]

    # -- 이벤트 처리 (컨트롤러 통보 — 철칙 2: 반사는 이미 일어났고 통보만) --

    def notify(self, event, now=None):
        """컨트롤러/파이프라인 통보 1건 처리. 반환: action dict 또는 None."""
        etype = event.get("event")
        if etype == "recover_enter":            # B 반사 진입 통보
            if not self._preempts("B"):
                return None
            invalidated = self.active_traj_hash
            self._set_mode("recovering", "B:attitude_recovery", "B")
            # §9 B: 현행 궤적 hash 무효 선언 (원장 기록). 자동 재개 금지 —
            # 회복 후 재개는 상위의 새 미션으로만 (스냅백 위험).
            self.active_traj_hash = None
            self._ledger("traj_hash_invalidated",
                         {"invalidated_hash": invalidated,
                          "cause": "recover_enter"}, now)
            self.publish(now)
            return {"action": "none", "note": "controller reflex owns recovery"}
        if etype == "recover_latched":          # B 완료: 래치 호버 도달
            if self.active_kind == "B":
                self._set_mode("hover_latched", "B:recovered", None)
                self.publish(now)
            return None
        if etype == "power_degraded_enter":     # C-모드 트리거 통보
            if not self._preempts("C"):
                return None
            self._set_mode("power_degraded", "C:thrust_authority_low", "C")
            self.publish(now)
            self._ledger("power_degraded_enter", event.get("detail"), now)
            # 통제 강하 궤적(구역 회피 하강 포함)은 파이프라인 몫 — 러너에 위임
            return {"action": "plan_controlled_descent",
                    "descent_rate_mps": 0.5, "keep_out": self.keep_out}
        if etype == "power_degraded_exit":      # C 해제 (포화율 회복)
            if self.active_kind == "C":
                self._set_mode("hover_latched", "C:authority_recovered", None)
                self.publish(now)
            return None
        if etype == "stop_latched":             # A-1 정지 완료: 래치 도달
            if self.active_kind == "A1":
                self._set_mode("hover_latched", "A1:stopped", None)
                self.publish(now)
            return None
        return None                             # 모르는 이벤트는 무시 (전방 호환)

    # -- 상위 비상 명령 (A 선언형) ------------------------------------------

    def _handle_cmd(self, cmd, now=None):
        ctype = cmd.get("type")
        if ctype == "stop":                     # A-1
            if not self._preempts("A1"):
                self._ledger("stop_deferred",
                             {"blocked_by": self.active_kind}, now)
                return None
            self._set_mode("emergency_stopping", "A1:stop_cmd", "A1")
            self.publish(now)
            self._ledger("emergency_stop", {"cmd": cmd}, now)
            # 정지 궤적 생성(§8 emergency 동사, stop_dist 정확식)은 러너 몫
            return {"action": "plan_emergency_stop",
                    "keep_out": self.keep_out}
        if ctype == "keep_out_update":          # A-2 (모드 아님 — 제약 갱신)
            self.keep_out = {"zones": cmd.get("zones", []),
                             "inflate_m": cmd.get("inflate_m", 0.5)}
            # 영속화: 파이프라인(emergency --keep-out 기본 경로)이 읽는 파일
            _atomic_write_json(self.keep_out_path, self.keep_out)
            self._ledger("keep_out_update", self.keep_out, now)
            # 현행 궤적과 교차하면 회피 재계획 필요 — 검사는 게이트(파이프라인) 몫
            return {"action": "check_keep_out_replan",
                    "keep_out": self.keep_out}
        return None

    # -- 입력 소비 ----------------------------------------------------------

    def _consume_emergency_cmd(self, now=None):
        if not os.path.isfile(self.emergency_cmd_path):
            return None
        try:
            with open(self.emergency_cmd_path, encoding="utf-8") as f:
                cmd = json.load(f)
        except json.JSONDecodeError:
            return None                          # 원자적 쓰기 계약 위반 대비
        wa = cmd.get("written_at")
        if wa is None or wa == self._last_cmd_written_at:
            return None                          # 이미 처리했거나 식별 불가
        self._last_cmd_written_at = wa
        return self._handle_cmd(cmd, now)

    def _consume_controller_events(self, now=None):
        actions = []
        if not os.path.isfile(self.controller_events_path):
            return actions
        with open(self.controller_events_path, encoding="utf-8") as f:
            f.seek(self._events_offset)
            chunk = f.read()
            self._events_offset = f.tell()
        for line in chunk.splitlines():
            line = line.strip()
            if not line:
                continue
            act = self.notify(json.loads(line), now)
            if act:
                actions.append(act)
        return actions

    # -- C-모드 트리거 감시 (§9 잠정 조건 — 실측 확정 대기, 옵트인) ----------

    def _poll_power_monitor(self, now=None):
        if self.power_monitor is None or self.active_kind is not None:
            return None                          # 비상 진행 중엔 중복 트리거 금지
        try:
            from traj_pipeline import load_current_state
            st = load_current_state(self.current_state_path)
        except (ValueError, FileNotFoundError, KeyError):
            return None                          # 상태 없음/낡음 - 판정 보류
        if self.power_monitor.update(st, now):
            return self.notify({"event": "power_degraded_enter",
                                "detail": self.power_monitor.snapshot()}, now)
        return None

    # -- 판단 루프 1회 -------------------------------------------------------

    def tick(self, now=None):
        """이벤트(컨트롤러 통보) -> C 감시 -> 명령(상위 선언) 소비 후 하트비트.

        통보를 먼저 읽는 이유: 우선순위 중재의 전제가 "현재 활성 비상"이고,
        그 진실은 컨트롤러 반사 통보에 있다 (B가 이미 진행 중인데 A-1을
        먼저 읽으면 낮은 우선순위가 순간적으로 이긴다).
        """
        actions = self._consume_controller_events(now)
        act = self._poll_power_monitor(now)
        if act:
            actions.append(act)
        act = self._consume_emergency_cmd(now)
        if act:
            actions.append(act)
        self.publish(now)                        # 하트비트는 매 tick 무조건
        return actions

    # -- 러너: action 실행 (§8 CLI 호출 — 감독자는 판단, 실행은 여기서) ------

    def execute_action(self, action, timeout_s=120, out_dir=None):
        """tick()이 반환한 action 1건 실행. 반환: 실행 결과 dict.

        §8 계약(stdout 마지막 줄 JSON + 종료 코드)으로 파이프라인 호출.
        out_dir: 산출물 경로 재지정 (기본 None = 파이프라인 기본 output/).
        plan_controlled_descent(C 강하 궤적)와 check_keep_out_replan(현행
        미션 회피 스플라이스)은 후속 단계 — 현재는 미실행 보고만.
        """
        kind = action.get("action")
        if kind == "plan_emergency_stop":
            cmd = [sys.executable, PIPELINE_PY, "emergency"]
            if os.path.isfile(self.keep_out_path):
                cmd += ["--keep-out", self.keep_out_path]
            if self.current_state_path:
                cmd += ["--state", self.current_state_path]
            if out_dir:
                cmd += ["--out-dir", out_dir]
            proc = subprocess.run(cmd, capture_output=True, text=True,
                                  encoding="utf-8", errors="replace",
                                  timeout=timeout_s)
            lines = proc.stdout.strip().splitlines()
            try:
                msg = json.loads(lines[-1]) if lines else {}
            except json.JSONDecodeError:
                msg = {"verdict": "error", "raw": lines[-1]}
            result = {"executed": True, "rc": proc.returncode,
                      "verdict": msg.get("verdict"),
                      "trajectory_hash": msg.get("trajectory_hash"),
                      "emergency": msg.get("emergency"),
                      "keep_out": msg.get("keep_out")}
            if proc.returncode == 0 and msg.get("trajectory_hash"):
                # 정지 궤적이 현행 궤적이 된다 (컨트롤러가 로드)
                self.active_traj_hash = msg["trajectory_hash"]
                self.publish()
            self._ledger("emergency_stop_planned",
                         {"rc": proc.returncode,
                          "traj_hash": msg.get("trajectory_hash")})
            return result
        return {"executed": False,
                "note": f"runner 미구현 action: {kind} (후속 단계)"}

    def run(self, rate_hz=5.0):
        period = 1.0 / float(rate_hz)
        print(f"[supervisor] start mode={self.mode} rate={rate_hz}Hz "
              f"state={self.flight_state_path}")
        try:
            while True:
                for act in self.tick():
                    print(f"[supervisor] action: "
                          f"{json.dumps(act, ensure_ascii=False)}")
                    res = self.execute_action(act)
                    print(f"[supervisor] result: "
                          f"{json.dumps(res, ensure_ascii=False)}")
                time.sleep(period)
        except KeyboardInterrupt:
            print("[supervisor] stop (KeyboardInterrupt)")


class _PowerDegradedMonitor:
    """C-모드 트리거 판정 (§9 잠정): 포화율 > 90%가 1s 지속 AND 고도 오차
    증가 추세.

    포화율 = |w_cmd|가 w_sat 이상인 모터 비율의 창 평균. w_sat(모터 명령
    포화 기준)은 실측 확정 전이라 생성자 인자 — 확정 후 기본값 승격.
    고도 오차 = ref_state.pos.z - pos.z (양수 = 기준보다 낮음), 창 안에서
    증가하면 추력 부족으로 고도를 잃는 중.
    """

    WINDOW_S = 1.0
    SAT_RATIO = 0.90
    ALT_ERR_RISE_M = 0.05        # 창 안에서 이만큼 커지면 "증가 추세"

    def __init__(self, w_sat_rad_s):
        self.w_sat = float(w_sat_rad_s)
        self.buf = []            # (t_sim_s, sat_frac, alt_err_m)

    def update(self, state, now=None):
        w = state.get("motors", {}).get("w_cmd")
        ref = state.get("ref_state", {}).get("pos")
        if w is None or ref is None:
            return False
        t = float(state.get("t_sim_s", 0.0))
        sat_frac = sum(1 for wi in w if abs(wi) >= self.w_sat) / len(w)
        alt_err = float(ref[2]) - float(state["pos"][2])
        self.buf.append((t, sat_frac, alt_err))
        self.buf = [b for b in self.buf if t - b[0] <= self.WINDOW_S]
        if len(self.buf) < 3 or self.buf[-1][0] - self.buf[0][0] \
                < 0.8 * self.WINDOW_S:
            return False                          # 창이 아직 안 참
        sat_ok = min(b[1] for b in self.buf) >= self.SAT_RATIO
        alt_rising = (self.buf[-1][2] - self.buf[0][2]) > self.ALT_ERR_RISE_M
        return sat_ok and alt_rising

    def snapshot(self):
        if not self.buf:
            return {}
        return {"sat_frac": self.buf[-1][1], "alt_err_m": self.buf[-1][2],
                "window_s": round(self.buf[-1][0] - self.buf[0][0], 3)}


def main():
    ap = argparse.ArgumentParser(description="flight supervisor (INTERFACE_SPEC 9)")
    ap.add_argument("--rate", type=float, default=5.0, help="tick rate [Hz]")
    args = ap.parse_args()
    FlightSupervisor().run(args.rate)


if __name__ == "__main__":
    main()
