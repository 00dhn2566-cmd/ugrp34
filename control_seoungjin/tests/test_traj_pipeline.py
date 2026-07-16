"""traj_pipeline.py 통합 테스트 — 체인 왕복 + attitude_feedback 핸드셰이크."""

import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import traj_pipeline as tp                      # noqa: E402


@pytest.fixture
def mission(tmp_path):
    cfg = {
        "waypoints": [[0.0, 0.0, 1.0], [0.0, 0.0, 3.0], [1.5, 1.5, 3.0]],
        "limits": {"v_max": 1.0, "a_max": 0.8, "j_max": 2.0, "snap_max": 10.0},
        "dt": 0.01,
        "shaper": {"mode": "zvd", "f_mode_hz": 1.8},
    }
    p = tmp_path / "mission.json"
    p.write_text(json.dumps(cfg), encoding="utf-8")
    return str(p), cfg


class TestPipeline:
    def test_end_to_end_outputs(self, mission, tmp_path):
        path, _ = mission
        out = str(tmp_path / "out")
        res = tp.run(path, out)
        for name in ("trajectory.mat", "trajectory.json", "pipeline_meta.json"):
            assert os.path.isfile(os.path.join(out, name)), f"{name} 미생성"
        meta = json.loads(
            open(os.path.join(out, "pipeline_meta.json"), encoding="utf-8").read())
        # 정품 궤적은 스무더 무개입 (< 2mm)
        assert meta["smoother"]["max_dev_m"] < 0.002
        # 게이트 물리 한계 이내
        assert meta["gate_report"]["vxyPk"] <= tp.PHYS_VMAX * 1.001
        assert meta["gate_report"]["jxyPk"] <= tp.PHYS_JMAX * 1.001
        # 최종 궤적 = 스무딩 + 델타 (지터 상쇄 레이어 분리 보관)
        np.testing.assert_allclose(
            res["shaped"], res["smoothed"] + res["delta"], atol=1e-12)

    def test_limits_over_budget_rejected(self, tmp_path):
        cfg = {
            "waypoints": [[0, 0, 1], [1, 0, 1]],
            # v_max 1.7 > (1-0.2)*2.0 = 1.6 -> 예산 초과
            "limits": {"v_max": 1.7, "a_max": 0.8, "j_max": 2.0, "snap_max": 10.0},
        }
        p = tmp_path / "over.json"
        p.write_text(json.dumps(cfg), encoding="utf-8")
        with pytest.raises(ValueError, match="예산"):
            tp.run(str(p), str(tmp_path / "out"))

    def test_missing_key_dies_loudly(self, tmp_path):
        p = tmp_path / "bad.json"
        p.write_text(json.dumps({"waypoints": [[0, 0, 1], [1, 0, 1]]}),
                     encoding="utf-8")
        with pytest.raises(KeyError, match="limits"):
            tp.run(str(p), str(tmp_path / "out"))


@pytest.fixture
def fb_env(tmp_path, monkeypatch):
    """FEEDBACK_PATH/LEDGER_PATH를 tmp로 격리."""
    fb_path = str(tmp_path / "attitude_feedback.json")
    ledger = str(tmp_path / "feedback_ledger.jsonl")
    monkeypatch.setattr(tp, "FEEDBACK_PATH", fb_path)
    monkeypatch.setattr(tp, "LEDGER_PATH", ledger)
    return fb_path, ledger


class TestRawTrajectoryInput:
    """원시 궤적(스텝 포함) 입구 — 'unit step이 오면 시간 부여해 ramp로'."""

    def _step_mission(self, tmp_path):
        t = [round(0.01 * i, 2) for i in range(1000)]     # 10s
        pos = [[0.0, 0.0, 2.0] if ti < 2.0 else [1.0, 0.0, 2.0] for ti in t]
        cfg = {"trajectory": {"t": t, "pos": pos},
               "limits": {"v_max": 1.0, "a_max": 0.8, "j_max": 2.0,
                          "snap_max": 10.0},
               "dt": 0.01, "shaper": {"mode": "zvd", "f_mode_hz": 1.8}}
        p = tmp_path / "step.json"
        p.write_text(json.dumps(cfg), encoding="utf-8")
        return str(p)

    def test_unit_step_becomes_feasible_scurve(self, tmp_path, fb_env):
        res = tp.run(self._step_mission(tmp_path), str(tmp_path / "out"))
        # 게이트 통과 (스텝이 물리 추종 가능 궤적으로 성형됨)
        assert res["gate_report"]["vxyPk"] <= tp.PHYS_VMAX * 1.001
        assert res["gate_report"]["jxyPk"] <= tp.PHYS_JMAX * 1.001
        # 종점 도달 + 과도 오버슈트 없음
        x = res["shaped"][:, 0]
        assert abs(x[-1] - 1.0) < 0.02
        assert np.max(x) < 1.0 + 0.05

    def test_waypoints_and_trajectory_both_rejected(self, tmp_path):
        cfg = {"waypoints": [[0, 0, 1], [1, 0, 1]],
               "trajectory": {"t": [0, 1], "pos": [[0, 0, 1], [1, 0, 1]]},
               "limits": {"v_max": 1, "a_max": 1, "j_max": 2, "snap_max": 10}}
        p = tmp_path / "both.json"
        p.write_text(json.dumps(cfg), encoding="utf-8")
        with pytest.raises(KeyError, match="정확히 하나"):
            tp.run(str(p), str(tmp_path / "out"))

    def test_nonmonotonic_trajectory_rejected(self, tmp_path):
        cfg = {"trajectory": {"t": [0, 1, 1, 2],
                              "pos": [[0, 0, 1]] * 4},
               "limits": {"v_max": 1, "a_max": 1, "j_max": 2, "snap_max": 10}}
        p = tmp_path / "bad.json"
        p.write_text(json.dumps(cfg), encoding="utf-8")
        with pytest.raises(ValueError, match="단조증가"):
            tp.run(str(p), str(tmp_path / "out"))


class TestSnapAndFlyThrough:
    """snap 4종째 검사(회로 부담 근거) + waypoint 무정지 통과 모드."""

    def test_snap_over_budget_rejected(self, tmp_path):
        cfg = {"waypoints": [[0, 0, 1], [1, 0, 1]],
               "limits": {"v_max": 1.0, "a_max": 0.8, "j_max": 2.0,
                          "snap_max": 70.0}}    # > 0.8*80 = 64
        p = tmp_path / "s.json"
        p.write_text(json.dumps(cfg), encoding="utf-8")
        with pytest.raises(ValueError, match="snap_max"):
            tp.run(str(p), str(tmp_path / "out"))

    def test_waypoint_mission_gate_includes_snap(self, mission, tmp_path):
        _, cfg = mission
        res = tp.build_trajectory(cfg, np.asarray(cfg["waypoints"], float), 1.8)
        assert "sxyPk" in res["gate_report"], "계획층 경로는 snap까지 게이트"
        assert res["gate_report"]["sxyPk"] <= tp.PHYS_SNAP * 1.001

    def test_raw_trajectory_backstop_skips_snap_enforcement(self, tmp_path, fb_env):
        """백스톱 경로: 스무더 뱅뱅 저크 = snap 임펄스가 정상 — 강제 안 함."""
        t = [round(0.01 * i, 2) for i in range(800)]
        pos = [[0.0, 0.0, 2.0] if ti < 2.0 else [1.0, 0.0, 2.0] for ti in t]
        cfg = {"trajectory": {"t": t, "pos": pos},
               "limits": {"v_max": 1.0, "a_max": 0.8, "j_max": 2.0,
                          "snap_max": 10.0}}
        p = tmp_path / "raw.json"
        p.write_text(json.dumps(cfg), encoding="utf-8")
        res = tp.run(str(p), str(tmp_path / "out"))   # snap 강제였으면 raise
        assert res["gate_ok"]
        assert res["gate_report"]["sxyPk"] > tp.PHYS_SNAP, \
            "측정은 되고(마진 보고용) 강제만 안 해야 함"

    def test_fly_through_no_stop_at_interior_waypoints(self, tmp_path, fb_env):
        cfg = {"waypoints": [[0, 0, 2], [2, 0, 2], [4, 1, 2], [6, 0, 2]],
               "limits": {"v_max": 1.0, "a_max": 0.8, "j_max": 2.0,
                          "snap_max": 10.0},
               "waypoint_mode": "fly_through"}
        p = tmp_path / "ft.json"
        p.write_text(json.dumps(cfg), encoding="utf-8")
        res = tp.run(str(p), str(tmp_path / "out"))
        t, pos = res["t"], res["shaped"]
        speed = np.linalg.norm(np.gradient(pos, t, axis=0), axis=1)
        for wp in cfg["waypoints"][1:-1]:
            k = int(np.argmin(np.linalg.norm(pos - np.array(wp), axis=1)))
            d = float(np.linalg.norm(pos[k] - np.array(wp)))
            # 성긴 꼭짓점의 코너 라운딩은 무정지 통과의 본질 (정확히 찍으려면
            # 정지 필요). 운용 레짐은 촘촘 입력이라 코너가 미리 곡선화돼 옴.
            assert d < 0.12, f"경유점 {wp} 과도 이탈 (최근접 {d:.3f}m)"
            assert speed[k] > 0.3 * 1.0, \
                f"경유점 {wp}에서 정지함 (속도 {speed[k]:.2f}m/s) - fly_through 위반"

    def test_fly_through_rejects_replan_ic(self, mission):
        _, cfg = mission
        cfg2 = {**cfg, "waypoint_mode": "fly_through"}
        with pytest.raises(ValueError, match="fly_through"):
            tp.build_trajectory(cfg2, np.asarray(cfg2["waypoints"], float),
                                1.8, v0=np.array([0.3, 0, 0]))


class TestWaypointBatches:
    """상위 입력 구조: waypoint 집합 배치 + 비행 중 새 집합 call 이어붙임."""

    LIM = {"v_max": 1.0, "a_max": 0.8, "j_max": 2.0, "snap_max": 10.0}

    def test_normalize_merge_and_divide(self):
        wp = [[0, 0, 1], [0, 0, 1.005], [0, 0, 3], [4, 0, 3]]
        out = tp.normalize_waypoints(wp, merge_dist=0.01, max_seg_len=1.0)
        assert np.linalg.norm(out[1] - out[0]) > 0.01, "근접점 병합돼야 함"
        seg = np.linalg.norm(np.diff(out, axis=0), axis=1)
        assert np.all(seg <= 1.0 + 1e-9), "긴 구간은 분할돼야 함"

    def test_normalize_all_merged_dies(self):
        with pytest.raises(ValueError, match="2개 미만"):
            tp.normalize_waypoints([[0, 0, 1], [0, 0, 1.001]], merge_dist=0.01)

    def test_collinear_merge_improves_time(self):
        """일직선 촘촘 점 병합 -> 정지 없이 순항 -> 소요시간 단축 (성능 목적)."""
        dense = [[float(x), 0.0, 2.0] for x in (0, 1, 2, 3, 4)]   # 일직선 5점
        merged = tp.normalize_waypoints(dense, collinear_tol=0.05)
        assert len(merged) == 2, "일직선 중간점은 전부 병합돼야 함"

        cfg_base = {"limits": self.LIM, "shaper": {"mode": "none"}}
        res_dense = tp.build_trajectory(
            {**cfg_base}, np.asarray(dense, float), 1.8)
        res_merged = tp.build_trajectory(
            {**cfg_base, "waypoint_prep": {"collinear_tol": 0.05}},
            np.asarray(dense, float), 1.8)
        t_dense, t_merged = res_dense["t"][-1], res_merged["t"][-1]
        assert t_merged < 0.8 * t_dense, \
            f"병합 후 순항으로 빨라져야 함 ({t_dense:.1f}s -> {t_merged:.1f}s)"
        # 굽은 경로는 병합되면 안 됨 (코너점 보존)
        bent = [[0, 0, 2], [2, 0, 2], [2, 2, 2]]
        kept = tp.normalize_waypoints(bent, collinear_tol=0.05)
        assert len(kept) == 3

    def test_divide_long_segment(self):
        out = tp.normalize_waypoints([[0, 0, 2], [5, 0, 2]], max_seg_len=2.0)
        seg = np.linalg.norm(np.diff(out, axis=0), axis=1)
        assert np.all(seg <= 2.0 + 1e-9) and len(out) == 4

    def test_dense_curve_intent_preserved(self):
        """촘촘한 곡선 입력: RDP가 직선은 뭉치고 곡선 형상(의도)은 보존."""
        # 직선 3m (30점) + 반원 호 r=1 (30점) + 직선 3m (30점)
        s1 = [[x, 0.0, 2.0] for x in np.linspace(0, 3, 30)]
        arc = [[3.0 + 1.0 * np.cos(t), 1.0 + 1.0 * np.sin(t), 2.0]
               for t in np.linspace(-np.pi / 2, np.pi / 2, 30)]
        s2 = [[x, 2.0, 2.0] for x in np.linspace(3, 0, 30)]
        dense = np.array(s1 + arc + s2)
        eps = 0.05
        out = tp.normalize_waypoints(dense, collinear_tol=eps)
        # 크게 줄어들되
        assert len(out) < 0.35 * len(dense), f"{len(dense)} -> {len(out)}"
        # 원래 점들이 단순화 폴리라인에서 eps 이상 벗어나지 않아야 (의도 보존)
        def dist_to_polyline(p, poly):
            best = np.inf
            for a, b in zip(poly[:-1], poly[1:]):
                ab = b - a
                L2 = float(np.dot(ab, ab))
                t = np.clip(np.dot(p - a, ab) / L2, 0, 1) if L2 > 0 else 0.0
                best = min(best, float(np.linalg.norm(p - a - t * ab)))
            return best
        worst = max(dist_to_polyline(p, out) for p in dense)
        assert worst <= eps + 1e-9, f"의도 이탈 {worst*100:.1f}cm > ε"

    def test_fly_through_auto_divide(self):
        """fly_through는 긴 직선 자동 분할 (스플라인 휨 방지)."""
        cfg = {"limits": self.LIM, "waypoint_mode": "fly_through",
               "shaper": {"mode": "none"}}
        wp = np.array([[0, 0, 2], [6, 0, 2], [6, 3, 2]], float)
        res = tp.build_trajectory(cfg, wp, 1.8)
        # 6m 직선이 분할돼도 경로는 직선 유지 -> 최대 이탈 작음
        y_on_straight = res["shaped"][res["shaped"][:, 0] < 5.5][:, 1]
        assert np.max(np.abs(y_on_straight)) < 0.10, \
            f"직선 구간 스플라인 휨 {np.max(np.abs(y_on_straight))*100:.0f}cm"

    def test_midflight_new_set_splices_without_stop(self):
        """1번 집합 비행 중 τ에 2번 집합 도착 → 정지 없이 그쪽으로 꺾음."""
        cfg = {"limits": self.LIM, "shaper": {"mode": "zvd", "f_mode_hz": 1.8}}
        set1 = np.array([[0, 0, 2], [4, 0, 2]])
        res1 = tp.build_trajectory(cfg, set1, 1.8)
        tau = 0.5 * res1["t"][-1]              # 1번 이동 한가운데서 call
        set2 = [[2.0, 3.0, 2.5], [0.0, 4.0, 2.0]]
        res = tp.replan_splice(res1, tau, set2, cfg)

        t, pos = res["t"], res["shaped"]
        k = int(np.argmin(np.abs(t - res["splice_at_s"])))
        speed = np.linalg.norm(np.gradient(pos, t, axis=0), axis=1)
        assert speed[k] > 0.2, f"스플라이스 지점에서 정지 (v={speed[k]:.2f})"
        # 게이트 4종(v/a/j/snap) 통과 = 이어붙임이 물리적으로 매끈
        assert res["gate_ok"]
        assert res["gate_report"]["sxyPk"] <= tp.PHYS_SNAP * 1.001
        # 2번 집합 waypoint 통과
        for wp in set2:
            d = np.min(np.linalg.norm(pos - np.array(wp), axis=1))
            assert d < 0.05, f"2차 집합 {wp} 미통과 ({d:.3f}m)"
        # 종점 정지 (배치 기본값)
        assert speed[-1] < 0.05

    def test_splice_continuity_no_derivative_kick(self):
        """스플라이스 경계에서 v/a 점프 없음 (미분킥 방지)."""
        cfg = {"limits": self.LIM, "shaper": {"mode": "none"}}
        set1 = np.array([[0, 0, 2], [4, 0, 2]])
        res1 = tp.build_trajectory(cfg, set1, 1.8)
        res = tp.replan_splice(res1, 0.5 * res1["t"][-1], [[4, 3, 2]], cfg)
        t, pos = res["t"], res["shaped"]
        k = int(np.argmin(np.abs(t - res["splice_at_s"])))
        vel = np.gradient(pos, t, axis=0)
        acc = np.gradient(vel, t, axis=0)
        dv = np.linalg.norm(vel[k + 1] - vel[k - 1])
        da = np.linalg.norm(acc[k + 1] - acc[k - 1])
        assert dv < 0.1, f"스플라이스 속도 점프 {dv:.3f}m/s"
        assert da < 0.5, f"스플라이스 가속 점프 {da:.3f}m/s2"


class TestCurrentState:
    def _write_state(self, path, ts):
        st = {"timestamp": ts,
              "pos": [1.0, 1.0, 2.0], "vel": [0.5, 0.0, 0.0],
              "acc": [0.1, 0.0, 0.0], "yaw_rad": 0.0,
              "ref_state": {"pos": [1.02, 1.0, 2.0], "vel": [0.5, 0.0, 0.0],
                            "acc": [0.1, 0.0, 0.0]}}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(st, f)
        return st

    def test_fresh_state_loaded(self, tmp_path):
        from datetime import datetime
        p = str(tmp_path / "current_state.json")
        self._write_state(p, datetime.now().strftime(tp.TS_FMT + ".%f")[:-3])
        st = tp.load_current_state(p)
        assert st["ref_state"]["pos"] == [1.02, 1.0, 2.0]

    def test_stale_state_rejected(self, tmp_path):
        p = str(tmp_path / "current_state.json")
        self._write_state(p, "2026-07-16T00-00-00.000")
        with pytest.raises(ValueError, match="낡음"):
            tp.load_current_state(p)

    def test_missing_state_dies(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            tp.load_current_state(str(tmp_path / "nope.json"))

    def test_splice_uses_ref_state_not_measured(self, tmp_path):
        """평시 재계획은 ref_state 기준 — 측정 상태 사용은 피드백 성형 함정."""
        from datetime import datetime
        p = str(tmp_path / "current_state.json")
        self._write_state(p, datetime.now().strftime(tp.TS_FMT + ".%f")[:-3])
        st = tp.load_current_state(p)
        wp, v0, a0 = tp.splice_waypoints_from_state(st, [[3.0, 3.0, 2.0]])
        np.testing.assert_allclose(wp[0], [1.02, 1.0, 2.0])   # ref, 측정(1.0) 아님
        np.testing.assert_allclose(v0, [0.5, 0.0, 0.0])
        assert wp.shape == (2, 3)

    def test_emergency_splice_uses_measured(self, tmp_path):
        from datetime import datetime
        p = str(tmp_path / "current_state.json")
        self._write_state(p, datetime.now().strftime(tp.TS_FMT + ".%f")[:-3])
        st = tp.load_current_state(p)
        wp, v0, a0 = tp.splice_waypoints_from_state(
            st, [[3.0, 3.0, 2.0]], emergency=True)
        np.testing.assert_allclose(wp[0], [1.0, 1.0, 2.0])    # 측정 pos

    def test_replan_end_to_end_gate_passes(self, mission, tmp_path):
        """이어붙인 초기조건(v0!=0)으로도 체인 전체가 게이트를 통과."""
        _, cfg = mission
        st = {"ref_state": {"pos": [0.0, 0.0, 1.0], "vel": [0.3, 0.0, 0.0],
                            "acc": [0.0, 0.0, 0.0]}}
        wp, v0, a0 = tp.splice_waypoints_from_state(st, cfg["waypoints"][1:])
        res = tp.build_trajectory(cfg, wp, 1.8, v0=v0, a0=a0)
        assert res["gate_report"]["vxyPk"] <= tp.PHYS_VMAX * 1.001


class TestFeedbackHandshake:
    def test_used_false_consumed_marked_and_ledgered(self, mission, tmp_path, fb_env):
        path, _ = mission
        fb_path, ledger = fb_env
        fb = {"flight_id": "f1", "used": False, "mode_freq_hz": 1.95,
              "trajectory_hash": "abc", "written_at": "2026-07-16T10-00-00",
              "tail": {"pitch_rms_deg": 4.0}}
        with open(fb_path, "w", encoding="utf-8") as f:
            json.dump(fb, f)

        res = tp.run(path, str(tmp_path / "out"))
        assert res["f_mode"] == pytest.approx(1.95), "실측 주파수로 f0 갱신돼야 함"
        after = json.loads(open(fb_path, encoding="utf-8").read())
        assert after["used"] is True, "소비 후 used:true 재기록(핸드셰이크) 누락"
        assert "_consume" not in after, "내부 필드가 파일에 새면 안 됨"
        # 원장 검증 (INTERFACE_SPEC §4: 처리 여부 + 경과 시간 조회 창구)
        lines = [json.loads(l) for l in open(ledger, encoding="utf-8")]
        assert len(lines) == 1
        assert lines[0]["flight_id"] == "f1"
        assert lines[0]["action"]["f_mode_hz"] == [1.8, 1.95]
        assert lines[0]["feedback_age_s"] is not None
        assert lines[0]["residual"]["tail_pitch_rms_deg"] == 4.0

    def test_flight_already_in_ledger_skipped(self, mission, tmp_path, fb_env):
        """used 태그가 유실돼도(false로 재등장) 원장이 이중 보정을 막는다."""
        path, _ = mission
        fb_path, ledger = fb_env
        with open(ledger, "w", encoding="utf-8") as f:
            f.write(json.dumps({"flight_id": "f1"}) + "\n")
        with open(fb_path, "w", encoding="utf-8") as f:
            json.dump({"flight_id": "f1", "used": False, "mode_freq_hz": 1.95}, f)
        res = tp.run(path, str(tmp_path / "out"))
        assert res["f_mode"] == pytest.approx(1.8), "원장 기존 건은 미적용"
        after = json.loads(open(fb_path, encoding="utf-8").read())
        assert after["used"] is True, "태그 복구돼야 함"

    def test_used_true_skipped(self, mission, tmp_path, fb_env):
        path, _ = mission
        fb_path, _ = fb_env
        with open(fb_path, "w", encoding="utf-8") as f:
            json.dump({"used": True, "mode_freq_hz": 1.95}, f)
        res = tp.run(path, str(tmp_path / "out"))
        assert res["f_mode"] == pytest.approx(1.8), "used:true는 건너뛰어야 함"

    def test_absurd_freq_rejected(self, mission, tmp_path, fb_env):
        path, _ = mission
        fb_path, _ = fb_env
        with open(fb_path, "w", encoding="utf-8") as f:
            json.dump({"flight_id": "fx", "used": False, "mode_freq_hz": 55.0}, f)
        with pytest.raises(ValueError, match="비정상"):
            tp.run(path, str(tmp_path / "out"))

    def test_out_of_band_freq_not_applied(self, mission, tmp_path, fb_env):
        """대역(1~3Hz) 밖 실측(예: 4.39Hz 루프 진동)은 f0 갱신 거부.

        A/B/B' 실증: 4.39Hz 추종 tail 12.25° vs 1.8Hz 고수 9.93°.
        """
        path, _ = mission
        fb_path, ledger = fb_env
        with open(fb_path, "w", encoding="utf-8") as f:
            json.dump({"flight_id": "f_oob", "used": False,
                       "mode_freq_hz": 4.39}, f)
        res = tp.run(path, str(tmp_path / "out"))
        assert res["f_mode"] == pytest.approx(1.8), "대역 밖은 갱신 금지"
        after = json.loads(open(fb_path, encoding="utf-8").read())
        assert after["used"] is True, "거부해도 소비 처리(used:true)는 해야 함"
        lines = [json.loads(l) for l in open(ledger, encoding="utf-8")]
        assert lines[-1]["action"]["rejected_out_of_band_hz"] == pytest.approx(4.39)
