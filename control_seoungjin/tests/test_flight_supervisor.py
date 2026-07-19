"""flight_supervisor.py 단위테스트 — §9 감독자 골격 (게이트/우선순위/하트비트)."""

import json
import os
import sys
from datetime import datetime, timedelta

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import flight_supervisor as fs                  # noqa: E402


@pytest.fixture
def sup(tmp_path):
    return fs.FlightSupervisor(
        flight_state_path=str(tmp_path / "flight_state.json"),
        emergency_cmd_path=str(tmp_path / "emergency_cmd.json"),
        controller_events_path=str(tmp_path / "controller_events.jsonl"),
        ledger_path=str(tmp_path / "feedback_ledger.jsonl"),
        keep_out_path=str(tmp_path / "keep_out.json"),
        current_state_path=str(tmp_path / "current_state.json"),
    )


def _write_cmd(sup, cmd):
    with open(sup.emergency_cmd_path, "w", encoding="utf-8") as f:
        json.dump(cmd, f)


def _append_event(sup, event):
    with open(sup.controller_events_path, "a", encoding="utf-8") as f:
        f.write(json.dumps(event) + "\n")


def _ledger_events(sup):
    if not os.path.isfile(sup.ledger_path):
        return []
    with open(sup.ledger_path, encoding="utf-8") as f:
        return [json.loads(ln)["event"] for ln in f if ln.strip()]


# -- flight_state.json 공표 (mode 단일 소유자) -------------------------------

def test_publish_schema(sup):
    sup.publish()
    with open(sup.flight_state_path, encoding="utf-8") as f:
        st = json.load(f)
    assert set(st) == {"written_at", "mode", "active_traj_hash", "reason"}
    assert st["mode"] == "normal"


def test_tick_refreshes_heartbeat(sup):
    t0 = datetime(2026, 7, 19, 12, 0, 0)
    sup.tick(now=t0)
    sup.tick(now=t0 + timedelta(seconds=1))
    with open(sup.flight_state_path, encoding="utf-8") as f:
        st = json.load(f)
    assert st["written_at"].startswith("2026-07-19T12-00-01")


# -- 컨트롤러 측 하트비트 감시 (철칙 3 기준 구현) ----------------------------

def test_heartbeat_stale_missing_file(tmp_path):
    assert fs.heartbeat_stale(str(tmp_path / "none.json")) is True


def test_heartbeat_fresh_then_stale(sup):
    t0 = datetime(2026, 7, 19, 12, 0, 0)
    sup.publish(now=t0)
    assert fs.heartbeat_stale(sup.flight_state_path,
                              now=t0 + timedelta(seconds=0.5)) is False
    assert fs.heartbeat_stale(sup.flight_state_path,
                              now=t0 + timedelta(seconds=1.5)) is True


def test_heartbeat_stale_on_corrupt_json(sup):
    with open(sup.flight_state_path, "w", encoding="utf-8") as f:
        f.write("{반쯤 써진")
    assert fs.heartbeat_stale(sup.flight_state_path) is True


# -- 미션 게이트 (§9: 비상 중 REJECTED_RECOVERING) ---------------------------

def test_gate_accepts_in_normal(sup):
    res = sup.submit_mission("m1.json", traj_hash="abc123")
    assert res["accepted"] is True
    assert sup.active_traj_hash == "abc123"


@pytest.mark.parametrize("mode", fs.GATED_MODES)
def test_gate_rejects_during_emergency(sup, mode):
    sup.mode = mode
    res = sup.submit_mission("m1.json")
    assert res["accepted"] is False
    assert res["reject_code"] == "REJECTED_RECOVERING"
    assert res["reason"] == mode          # C-모드: reason=power_degraded (§9)


def test_gate_accepts_in_hover_latched_back_to_normal(sup):
    sup.mode = "hover_latched"
    res = sup.submit_mission("m2.json")
    assert res["accepted"] is True
    assert sup.mode == "normal"


# -- A-1 비상 정지 -----------------------------------------------------------

def test_a1_stop_cmd(sup):
    _write_cmd(sup, {"written_at": "2026-07-19T12-00-00", "type": "stop"})
    actions = sup.tick()
    assert sup.mode == "emergency_stopping"
    assert [a["action"] for a in actions] == ["plan_emergency_stop"]
    assert "emergency_stop" in _ledger_events(sup)


def test_a1_cmd_dedupe_same_written_at(sup):
    _write_cmd(sup, {"written_at": "2026-07-19T12-00-00", "type": "stop"})
    assert sup.tick()                      # 1회차 처리
    assert sup.tick() == []                # 같은 written_at은 재처리 금지


def test_a1_stop_latched_ends_in_hover(sup):
    _write_cmd(sup, {"written_at": "2026-07-19T12-00-00", "type": "stop"})
    sup.tick()
    _append_event(sup, {"event": "stop_latched"})
    sup.tick()
    assert sup.mode == "hover_latched"    # §1 무명령 default로 합류
    assert sup.active_kind is None


# -- A-2 금지 구역 ------------------------------------------------------------

def test_a2_keep_out_update_stored(sup):
    zones = [{"shape": "sphere", "center": [1, 2, 3], "radius_m": 1.0}]
    _write_cmd(sup, {"written_at": "2026-07-19T12-00-00",
                     "type": "keep_out_update",
                     "zones": zones, "inflate_m": 0.7})
    actions = sup.tick()
    assert sup.keep_out == {"zones": zones, "inflate_m": 0.7}
    assert actions[0]["action"] == "check_keep_out_replan"
    assert sup.mode == "normal"           # A-2는 모드가 아니라 제약 갱신


def test_a1_after_a2_carries_keep_out(sup):
    """정지 궤적도 구역 회피 대상 (§9 적용 범위: 위치 제어 살아있는 전 모드)."""
    zones = [{"shape": "box", "min": [0, 0, 0], "max": [1, 1, 1]}]
    _write_cmd(sup, {"written_at": "t1", "type": "keep_out_update",
                     "zones": zones})
    sup.tick()
    _write_cmd(sup, {"written_at": "t2", "type": "stop"})
    actions = sup.tick()
    assert actions[0]["action"] == "plan_emergency_stop"
    assert actions[0]["keep_out"]["zones"] == zones


# -- B 회생 (컨트롤러 반사 통보) ---------------------------------------------

def test_b_recover_invalidates_hash_and_gates(sup):
    sup.submit_mission("m1.json", traj_hash="deadbeef")
    _append_event(sup, {"event": "recover_enter"})
    sup.tick()
    assert sup.mode == "recovering"
    assert sup.active_traj_hash is None   # 자동 재개 금지 (스냅백 위험)
    assert "traj_hash_invalidated" in _ledger_events(sup)
    res = sup.submit_mission("m2.json")
    assert res["reject_code"] == "REJECTED_RECOVERING"


def test_b_full_cycle_to_new_mission(sup):
    _append_event(sup, {"event": "recover_enter"})
    sup.tick()
    _append_event(sup, {"event": "recover_latched"})
    sup.tick()
    assert sup.mode == "hover_latched"
    assert sup.submit_mission("m3.json")["accepted"] is True
    assert sup.mode == "normal"


# -- C 추력 부족 + 우선순위 중재 (B > C > A-1 > A-2) --------------------------

def test_c_enter_plans_descent_with_keep_out(sup):
    """C 통제 강하는 수직이 아니라 구역 회피 하강 (사용자 지적 07-19)."""
    zones = [{"shape": "sphere", "center": [0, 0, 0], "radius_m": 2.0}]
    _write_cmd(sup, {"written_at": "t1", "type": "keep_out_update",
                     "zones": zones})
    sup.tick()
    _append_event(sup, {"event": "power_degraded_enter"})
    actions = sup.tick()
    assert sup.mode == "power_degraded"
    descent = [a for a in actions if a["action"] == "plan_controlled_descent"]
    assert descent and descent[0]["keep_out"]["zones"] == zones
    assert descent[0]["descent_rate_mps"] == 0.5


def test_priority_b_preempts_c(sup):
    _append_event(sup, {"event": "power_degraded_enter"})
    sup.tick()
    assert sup.mode == "power_degraded"
    _append_event(sup, {"event": "recover_enter"})
    sup.tick()
    assert sup.mode == "recovering"       # B가 C를 선점


def test_priority_a1_does_not_preempt_c(sup):
    _append_event(sup, {"event": "power_degraded_enter"})
    sup.tick()
    _write_cmd(sup, {"written_at": "t1", "type": "stop"})
    actions = sup.tick()
    assert sup.mode == "power_degraded"   # C 유지 (C > A-1)
    assert all(a["action"] != "plan_emergency_stop" for a in actions)
    assert "stop_deferred" in _ledger_events(sup)


def test_priority_c_does_not_preempt_b(sup):
    _append_event(sup, {"event": "recover_enter"})
    sup.tick()
    _append_event(sup, {"event": "power_degraded_enter"})
    sup.tick()
    assert sup.mode == "recovering"       # B 유지


def test_c_exit_to_hover_latched(sup):
    _append_event(sup, {"event": "power_degraded_enter"})
    sup.tick()
    _append_event(sup, {"event": "power_degraded_exit"})
    sup.tick()
    assert sup.mode == "hover_latched"


# -- 입력 견고성 --------------------------------------------------------------

def test_corrupt_cmd_file_ignored(sup):
    with open(sup.emergency_cmd_path, "w", encoding="utf-8") as f:
        f.write("{깨진 json")
    assert sup.tick() == []               # 원자적 쓰기 위반 대비: 조용히 스킵
    assert sup.mode == "normal"


def test_unknown_event_ignored(sup):
    _append_event(sup, {"event": "future_event_v99"})
    assert sup.tick() == []               # 전방 호환: 모르는 이벤트 무시
    assert sup.mode == "normal"


# -- A-2 keep_out 영속화 (파이프라인 emergency --keep-out 기본 경로) ----------

def test_keep_out_update_persists_file(sup):
    zones = [{"shape": "box", "min": [0, 0, 0], "max": [1, 1, 1]}]
    _write_cmd(sup, {"written_at": "t1", "type": "keep_out_update",
                     "zones": zones, "inflate_m": 0.6})
    sup.tick()
    with open(sup.keep_out_path, encoding="utf-8") as f:
        saved = json.load(f)
    assert saved == {"zones": zones, "inflate_m": 0.6}


# -- C-모드 트리거 감시 (§9 잠정: 포화율 + 고도 오차 증가 추세) ---------------

W_SAT = 700.0


def _write_state(sup, t_sim, w, z, ref_z):
    from datetime import timedelta
    st = {"timestamp": (datetime.now() + timedelta(seconds=5.0))
          .strftime("%Y-%m-%dT%H-%M-%S.%f")[:-3],
          "t_sim_s": t_sim,
          "pos": [0.0, 0.0, z], "vel": [0, 0, 0], "acc": [0, 0, 0],
          "att": {"roll_rad": 0, "pitch_rad": 0, "yaw_rad": 0},
          "ref_state": {"pos": [0.0, 0.0, ref_z], "vel": [0, 0, 0],
                        "acc": [0, 0, 0], "traj_hash": "x",
                        "t_on_traj_s": t_sim},
          "motors": {"w_cmd": list(w)}}
    with open(sup.current_state_path, "w", encoding="utf-8") as f:
        json.dump(st, f)


def _c_sup(tmp_path):
    return fs.FlightSupervisor(
        flight_state_path=str(tmp_path / "flight_state.json"),
        emergency_cmd_path=str(tmp_path / "emergency_cmd.json"),
        controller_events_path=str(tmp_path / "controller_events.jsonl"),
        ledger_path=str(tmp_path / "feedback_ledger.jsonl"),
        keep_out_path=str(tmp_path / "keep_out.json"),
        current_state_path=str(tmp_path / "current_state.json"),
        w_cmd_sat_rad_s=W_SAT)


def test_c_monitor_triggers_on_saturation_and_alt_loss(tmp_path):
    """포화(4모터 >= w_sat) 1s 지속 + 고도 오차 증가 -> power_degraded."""
    sup = _c_sup(tmp_path)
    sat = [720.0, 705.0, 710.0, 715.0]
    all_actions = []
    for k in range(12):                   # 1.1s 창 채우기, 고도 5cm+ 하락
        _write_state(sup, t_sim=k * 0.1, w=sat, z=2.0 - 0.015 * k, ref_z=2.0)
        all_actions += sup.tick()         # 트리거 tick 이후는 중복 방지로 무action
    assert sup.mode == "power_degraded"
    assert any(a.get("action") == "plan_controlled_descent"
               for a in all_actions)


def test_c_monitor_no_trigger_when_altitude_holds(tmp_path):
    """포화라도 고도 유지 중이면 트리거 안 함 (AND 조건 — 오발화 방지)."""
    sup = _c_sup(tmp_path)
    sat = [720.0, 705.0, 710.0, 715.0]
    for k in range(12):
        _write_state(sup, t_sim=k * 0.1, w=sat, z=2.0, ref_z=2.0)
        sup.tick()
    assert sup.mode == "normal"


def test_c_monitor_no_trigger_without_saturation(tmp_path):
    sup = _c_sup(tmp_path)
    ok = [650.0, 640.0, 660.0, 655.0]     # w_sat 미만
    for k in range(12):
        _write_state(sup, t_sim=k * 0.1, w=ok, z=2.0 - 0.015 * k, ref_z=2.0)
        sup.tick()
    assert sup.mode == "normal"


def test_c_monitor_disabled_by_default(sup):
    assert sup.power_monitor is None      # w_sat 실측 확정 전 옵트인


# -- 러너 (action -> §8 CLI 실행) ---------------------------------------------

def test_runner_unknown_action_reports(sup):
    res = sup.execute_action({"action": "plan_controlled_descent"})
    assert res["executed"] is False


def test_runner_executes_emergency_stop(sup, tmp_path):
    """감독자 -> emergency 동사 실전 왕복 (subprocess, §8 계약 파싱)."""
    from datetime import timedelta
    st = {"timestamp": (datetime.now() + timedelta(seconds=30.0))
          .strftime("%Y-%m-%dT%H-%M-%S.%f")[:-3],
          "pos": [0.0, 0.0, 2.0], "vel": [1.0, 0, 0], "acc": [0, 0, 0],
          "att": {"roll_rad": 0, "pitch_rad": 0, "yaw_rad": 0},
          "ref_state": {"pos": [0.0, 0.0, 2.0], "vel": [1.0, 0, 0],
                        "acc": [0, 0, 0], "traj_hash": "x",
                        "t_on_traj_s": 0.0}}
    with open(sup.current_state_path, "w", encoding="utf-8") as f:
        json.dump(st, f)
    _write_cmd(sup, {"written_at": "t1", "type": "stop"})
    actions = sup.tick()
    assert actions and actions[0]["action"] == "plan_emergency_stop"
    res = sup.execute_action(actions[0], out_dir=str(tmp_path / "out"))
    assert res["executed"] is True
    assert res["rc"] == 0, res
    assert res["verdict"] == "accepted"
    assert sup.active_traj_hash == res["trajectory_hash"]
    assert os.path.isfile(tmp_path / "out" / "trajectory.json")
    assert "emergency_stop_planned" in _ledger_events(sup)
