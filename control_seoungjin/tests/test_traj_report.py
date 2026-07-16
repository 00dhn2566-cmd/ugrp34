"""traj_report.py — RL 궤도 계약 판정 리포트 테스트."""

import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import traj_report as trp                       # noqa: E402


def _write(tmp_path, cfg, name="m.json"):
    p = tmp_path / name
    p.write_text(json.dumps(cfg), encoding="utf-8")
    return str(p)


GOOD_LIMITS = {"v_max": 1.0, "a_max": 0.8, "j_max": 2.0, "snap_max": 10.0}


class TestStaticReport:
    def test_good_mission_accepted_with_margins(self, tmp_path):
        p = _write(tmp_path, {
            "waypoints": [[0, 0, 1], [0, 0, 3], [2, 0, 3]],
            "limits": GOOD_LIMITS})
        rep, res = trp.static_report(p)
        assert rep["verdict"] == "accepted"
        assert rep["reject_codes"] == []
        assert all(0.0 <= v <= 1.0 for v in rep["margins"].values()), \
            "정상 궤도의 마진은 전부 한계 이내 비율이어야 함"
        assert rep["shaping"]["deviation_max_m"] < 0.01
        assert rep["trajectory"]["hash"]

    def test_over_budget_clamped_with_adjustment(self, tmp_path):
        """v0.2 완화: 예산 초과 -> accepted + LIMITS_CLAMPED 통지."""
        p = _write(tmp_path, {
            "waypoints": [[0, 0, 1], [1, 0, 1]],
            "limits": {**GOOD_LIMITS, "v_max": 1.9}})
        rep, res = trp.static_report(p)
        assert rep["verdict"] == "accepted"
        codes = [a["code"] for a in rep["adjustments"]]
        assert "LIMITS_CLAMPED" in codes
        assert rep["margins"]["vxy"] <= 0.801   # 1.6/2.0

    def test_strict_over_budget_rejected(self, tmp_path):
        p = _write(tmp_path, {
            "waypoints": [[0, 0, 1], [1, 0, 1]],
            "limits": {**GOOD_LIMITS, "v_max": 1.9}, "strict": True})
        rep, res = trp.static_report(p)
        assert rep["verdict"] == "rejected"
        assert rep["reject_codes"][0]["code"] == "LIMITS_OVER_BUDGET"
        assert res is None

    def test_schema_error_code(self, tmp_path):
        p = _write(tmp_path, {"waypoints": [[0, 0, 1], [1, 0, 1]]})
        rep, _ = trp.static_report(p)
        assert rep["verdict"] == "rejected"
        assert rep["reject_codes"][0]["code"] == "SCHEMA_ERROR"

    def test_nonmonotonic_code(self, tmp_path):
        p = _write(tmp_path, {
            "trajectory": {"t": [0, 1, 1, 2], "pos": [[0, 0, 1]] * 4},
            "limits": GOOD_LIMITS})
        rep, _ = trp.static_report(p)
        assert rep["reject_codes"][0]["code"] == "TIME_NOT_MONOTONIC"

    def test_step_trajectory_retimed_and_accepted(self, tmp_path):
        """v0.2 완화: 1m 스텝 원시 궤적 -> 경로 보존 재시간화로 수용.

        공간 의도(0->1m 이동)는 그대로, 시간만 재배분 -> TIME_DILATED 통지.
        (path_time의 존재 이유가 계약으로 승격된 것)
        """
        t = [round(0.01 * i, 2) for i in range(800)]
        pos = [[0.0, 0.0, 2.0] if ti < 2.0 else [1.0, 0.0, 2.0] for ti in t]
        p = _write(tmp_path, {
            "trajectory": {"t": t, "pos": pos}, "limits": GOOD_LIMITS})
        rep, res = trp.static_report(p)
        assert rep["verdict"] == "accepted"
        adj = {a["code"]: a["detail"] for a in rep["adjustments"]}
        assert "TIME_DILATED" in adj
        assert adj["TIME_DILATED"]["dilation"] > 0
        # 재시간화 후 편차는 허용 이내 (경로 보존)
        assert rep["shaping"]["deviation_max_m"] <= trp.RESHAPE_TOL_M
        # 종점 도달 확인
        assert abs(res["shaped"][-1, 0] - 1.0) < 0.02
        kin = {k: v for k, v in rep["margins"].items() if not k.startswith("s")}
        assert all(v <= 1.001 for v in kin.values())

    def test_report_is_json_serializable(self, tmp_path):
        p = _write(tmp_path, {
            "waypoints": [[0, 0, 1], [1, 0, 2]], "limits": GOOD_LIMITS})
        rep, _ = trp.static_report(p)
        json.dumps(rep)   # 직렬화 가능해야 RL 쪽에서 소비 가능
