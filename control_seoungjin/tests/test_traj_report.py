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

    def test_over_budget_rejected_with_code(self, tmp_path):
        p = _write(tmp_path, {
            "waypoints": [[0, 0, 1], [1, 0, 1]],
            "limits": {**GOOD_LIMITS, "v_max": 1.9}})
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

    def test_step_trajectory_reshaped_beyond_tol(self, tmp_path):
        """1m 스텝 원시 궤적: 날 수는 있지만 성형 편차가 커서 RL에 경고."""
        t = [round(0.01 * i, 2) for i in range(800)]
        pos = [[0.0, 0.0, 2.0] if ti < 2.0 else [1.0, 0.0, 2.0] for ti in t]
        p = _write(tmp_path, {
            "trajectory": {"t": t, "pos": pos}, "limits": GOOD_LIMITS})
        rep, res = trp.static_report(p)
        codes = [c["code"] for c in rep["reject_codes"]]
        assert "RESHAPED_BEYOND_TOL" in codes
        assert rep["verdict"] == "rejected"
        # 마진 자체는 이내 (성형이 물리 한계는 지켰음) — 편차가 문제
        assert all(v <= 1.001 for v in rep["margins"].values())

    def test_report_is_json_serializable(self, tmp_path):
        p = _write(tmp_path, {
            "waypoints": [[0, 0, 1], [1, 0, 2]], "limits": GOOD_LIMITS})
        rep, _ = trp.static_report(p)
        json.dumps(rep)   # 직렬화 가능해야 RL 쪽에서 소비 가능
