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
