"""
궤도 판정 리포트 — 상위(경로계획 RL)가 기계로 읽는 합격/불합격 + 성능 metric.

"이런 궤도는 넘기지 마"의 정형화 (INTERFACE_SPEC §7):
  Tier 1 (정적, 즉시): 궤도 JSON → 성형 체인 → verdict + reject_codes[] +
    margins(한계 대비 피크 비율 — RL 보상 성형용 연속 신호) + 성형 개입량.
  Tier 2 (동적, MATLAB 비행 후): --flight-mat 주면 추종 RMS/종점오차/자세
    피크/tail 잔류 지터를 같은 리포트에 병합.

reject_codes (기계 판독용, 안정 계약 — 코드 추가는 가능, 의미 변경 금지):
  SCHEMA_ERROR            입력 JSON 형식 위반 (필수 키 누락, shape 불일치)
  TIME_NOT_MONOTONIC      trajectory.t 단조증가 위반
  LIMITS_OVER_BUDGET      limits가 지터 예산 반영 상한(0.8×물리) 초과
  GATE_EXCEEDED           성형 후에도 v/a/j 피크가 물리 한계 초과 (세부는 margins)
  RESHAPED_BEYOND_TOL     성형 개입(요청 궤적과의 편차)이 허용치 초과 —
                          날 수는 있지만 "네가 보낸 궤적"이 아니게 됨
verdict="accepted"여도 margins가 1.0에 붙어 있으면 여유 없는 궤도 (벌점 권장).

사용:
    python traj_report.py --input input/mission.json
    python traj_report.py --input input/mission.json --flight-mat <sim_result_baked.mat>
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import traj_pipeline as tp                      # noqa: E402
from analyze_flight_log import analyze          # noqa: E402

CONTRACT_VERSION = "0.1"
RESHAPE_TOL_M = 0.30       # 성형 편차 허용치 [m] — 초과 시 RESHAPED_BEYOND_TOL
REPORT_PATH = os.path.join(tp.OUTPUT_DIR, "trajectory_report.json")


def _margins(gate_rep):
    """게이트 피크 / 물리 한계 비율 (1.0 초과 = 위반). RL 벌점용 연속 신호."""
    m = {
        "vxy": gate_rep["vxyPk"] / tp.PHYS_VMAX,
        "axy": gate_rep["axyPk"] / tp.PHYS_AMAX,
        "jxy": gate_rep["jxyPk"] / tp.PHYS_JMAX,
        "vz": gate_rep["vzPk"] / tp.PHYS_VMAX,
        "az": gate_rep["azPk"] / tp.PHYS_AMAX,
        "jz": gate_rep["jzPk"] / tp.PHYS_JMAX,
    }
    if "sxyPk" in gate_rep:
        m["sxy"] = gate_rep["sxyPk"] / tp.PHYS_SNAP
        m["sz"] = gate_rep["szPk"] / tp.PHYS_SNAP
    return m


def static_report(input_path):
    """Tier 1: 정적 판정. (report_dict, res|None) 반환."""
    codes = []
    try:
        cfg, waypoints = tp.load_mission(input_path)
    except (KeyError, FileNotFoundError) as e:
        codes.append({"code": "SCHEMA_ERROR", "detail": str(e)})
        return {"verdict": "rejected", "reject_codes": codes,
                "contract_version": CONTRACT_VERSION}, None
    except ValueError as e:
        msg = str(e)
        code = ("LIMITS_OVER_BUDGET" if "예산" in msg
                else "TIME_NOT_MONOTONIC" if "단조증가" in msg
                else "SCHEMA_ERROR")
        codes.append({"code": code, "detail": msg})
        return {"verdict": "rejected", "reject_codes": codes,
                "contract_version": CONTRACT_VERSION}, None

    f_mode = float(cfg.get("shaper", {}).get("f_mode_hz", tp.F_MODE_DEFAULT))
    res = tp.build_trajectory(cfg, waypoints, f_mode, gate_error=False)

    margins = _margins(res["gate_report"])
    dev = float(np.max(res["smoother_info"]["maxDev"]))
    if not res["gate_ok"]:
        worst = max(margins, key=margins.get)
        codes.append({"code": "GATE_EXCEEDED",
                      "detail": f"성형 후에도 물리 한계 초과 (최악 채널 {worst})",
                      "value": margins[worst], "limit": 1.0})
    if dev > RESHAPE_TOL_M:
        codes.append({"code": "RESHAPED_BEYOND_TOL",
                      "detail": "성형 편차 초과 - 요청 궤적과 실비행 궤적이 다름",
                      "value": dev, "limit": RESHAPE_TOL_M})

    report = {
        "verdict": "accepted" if not codes else "rejected",
        "reject_codes": codes,
        "margins": {k: round(v, 4) for k, v in margins.items()},
        "shaping": {
            "deviation_max_m": round(dev, 4),
            "xy_share_applied": float(res["smoother_info"]["xy_share_applied"]),
            "jitter_delta_max_m": round(float(np.max(np.abs(res["delta"]))), 4),
        },
        "trajectory": {
            "hash": res["trajectory_hash"],
            "duration_s": round(float(res["t"][-1]), 3),
            "n_samples": int(len(res["t"])),
            "shaper": {"mode": res["shaper_mode"], "f_mode_hz": res["f_mode"]},
        },
        "flight": None,
        "contract_version": CONTRACT_VERSION,
    }
    return report, res


def add_flight_metrics(report, mat_path):
    """Tier 2: 비행 로그 성능 metric 병합 (analyze_flight_log 재사용)."""
    fl = analyze(mat_path)
    report["flight"] = {
        "track_rms_cm": fl["moving"]["track_rms_cm"],
        "att_peak_deg": fl["moving"]["att_peak_deg"],
        "tail_pitch_rms_deg": fl["tail"]["pitch_rms_deg"],
        "tail_roll_rms_deg": fl["tail"]["roll_rms_deg"],
        "residual_mode_freq_hz": fl["mode_freq_hz"],
    }
    return report


def main():
    ap = argparse.ArgumentParser(description="궤도 판정 리포트 (RL 계약)")
    ap.add_argument("--input", required=True)
    ap.add_argument("--flight-mat", default=None,
                    help="비행 후 sim_result_baked.mat — Tier 2 metric 병합")
    ap.add_argument("--out", default=REPORT_PATH)
    args = ap.parse_args()

    report, res = static_report(args.input)
    if args.flight_mat:
        if report["verdict"] != "accepted":
            print("[경고] rejected 궤도에 비행 metric 병합 요청 - 무시")
        else:
            add_flight_metrics(report, args.flight_mat)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    tp._atomic_write_json(args.out, report)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"[write] {args.out}")


if __name__ == "__main__":
    main()
