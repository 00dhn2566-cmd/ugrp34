"""
모터 입력↔센서 출력 회귀로 플랜트 상수 추정 → output/param_estimate.json.

핸드오프 "K_thrust/K_drag 추정 → PID 자동 스케일"의 추정기 구현 + 질량·관성 확장:
  1. K̂_thrust : 프로펠러별 T = k·w² 회귀 (집중계수 [N/(rad/s)²] —
                 parameters.m의 sT=Kthrust_ref/Kthrust 스케일이 먹는 형태)
  2. K̂_drag  : yaw 각가속 = (k_d/Izz)·Σ(±w²) 회귀 (Izz 공칭 전제, 핸드오프 방식)
  3. 질량 m̂  : 병진 z 평형 m·(z̈+g) = ΣT·cosφcosθ — 추력이 직접 로깅되므로
                 K 없이 독립 추정 가능 (짐 탑재/투하 감지용)
  4. Îxx/Îyy : roll/pitch 각가속 = (L/I)·ΔT 회귀 (팔길이 L 기지 전제)
                 Îzz는 K_drag 공칭으로 역산.

각 추정치에 R²(결정계수)를 신뢰도로 동승 — 임계(기본 0.9) 미달 항목은
"low_confidence"로 표시하고 소비 측이 반영하지 않는다 (급변 방지).
프로펠러 부호/배치(mixer)는 후보 조합 중 최고 R²를 자동 선택 (배치 무가정).

사용:
    python estimate_params.py [--mat <sim_result_baked.mat>] [--dry-run]
"""

import argparse
import json
import os
import sys
from datetime import datetime

import numpy as np
from scipy.io import loadmat

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_pipeline import OUTPUT_DIR, TS_FMT, _atomic_write_json  # noqa: E402
from analyze_flight_log import DEFAULT_MAT, META_PATH, _ts_struct  # noqa: E402

G = 9.80665
# 공칭값 출처: TUNING_STATUS §Y 17차 실측 (diagnose_inertia_measure.m + qc_phys 합성)
ARM_LENGTH_M = 0.159        # X쿼드 축별 모멘트 팔 [m] (휠베이스 450mm / 2 / √2)
IZZ_NOMINAL = 2.124e-2      # I_yaw 실측 합성 [kg m²] (K_drag 회귀 전제)
KDRAG_NOMINAL = 0.597       # parameters.m Kdrag_ref (Izz 역산용)
# 대조 기대값: m_tot=2.2726kg (드론 1.2726 + 패키지 1.0), I_att=1.713e-2
R2_CONFIDENT = 0.90
TILT_MAX_DEG = 10.0         # 질량 회귀에 쓰는 소기울기 구간
Z_AIRBORNE_M = 1.0          # 질량 추정 유효 고도: 짐(섀시 -8.6cm 현수)이 지면에서
                            # 완전히 떨어진 구간만 — 저고도에선 짐이 접촉 물리로
                            # 지면에 앉아 추력이 드론만 듦 (1.21kg 오측 실증)

ESTIMATE_PATH = os.path.join(OUTPUT_DIR, "param_estimate.json")


def _lin_fit(x, y):
    """y = a·x (+b) 최소자승 → (기울기 a, R²)."""
    A = np.column_stack([x, np.ones_like(x)])
    (a, b), res, *_ = np.linalg.lstsq(A, y, rcond=None)
    y_hat = a * x + b
    ss_res = float(np.sum((y - y_hat) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return float(a), float(r2)


def _resample(t_src, v_src, t):
    return np.interp(t, t_src, v_src)


def _ang_accel(t, ang):
    """각도 시계열 → 각가속 (2회 미분, 가변 스텝 대응)."""
    w = np.gradient(ang, t)
    return np.gradient(w, t)


def _best_combo_fit(x_parts, y):
    """±1 부호 후보 조합 중 최고 R² 회귀 선택 (프로펠러 배치 무가정).

    x_parts: (4, N) 프로펠러별 신호. 후보 = 부호 2개+/2개- 조합 (총 6 - 대칭 3).
    """
    combos = [(1, 1, -1, -1), (1, -1, 1, -1), (1, -1, -1, 1)]
    best = (0.0, -np.inf, None)
    for c in combos:
        x = sum(ci * xi for ci, xi in zip(c, x_parts))
        if np.std(x) < 1e-9:
            continue
        a, r2 = _lin_fit(x, y)
        if r2 > best[1]:
            best = (a, r2, c)
    return best


def estimate(mat_path):
    m = loadmat(mat_path, squeeze_me=True, struct_as_record=False)
    req = ["real_roll", "real_pitch", "real_yaw", "real_vz", "real_z",
           "prop1_T", "prop2_T", "prop3_T", "prop4_T",
           "prop1_w", "prop2_w", "prop3_w", "prop4_w"]
    missing = [k for k in req if k not in m]
    if missing:
        raise KeyError(f"로그 변수 없음: {missing} — run_traj_baked.m 태핑"
                       " 버전 확인 (파라미터 추정 신호는 2026-07-16 추가)")

    # 공통 시간축 (자세 신호 기준, 양끝 0.5s 버림 — 과도/패딩 제거)
    t_ref, roll = _ts_struct(m, "real_roll")
    t = t_ref[(t_ref > 0.5) & (t_ref < t_ref[-1] - 0.5)]
    sig = {}
    for k in req:
        ts, vs = _ts_struct(m, k)
        sig[k] = _resample(ts, vs, t)

    T_all = [sig[f"prop{i}_T"] for i in range(1, 5)]
    w2_all = [sig[f"prop{i}_w"] ** 2 for i in range(1, 5)]

    out = {}

    # 1) K̂_thrust: T = k·w² (4개 프로펠러 합산 회귀)
    kt, r2_kt = _lin_fit(np.concatenate(w2_all), np.concatenate(T_all))
    out["k_thrust_lumped"] = {"value": kt, "unit": "N/(rad/s)^2", "r2": r2_kt}

    # 3) 질량: m·(z̈+g) = ΣT·cosφcosθ — 공중(z>Z_AIRBORNE_M) + 소기울기 구간.
    #    값은 준정적 평형(호버 평균), 신뢰도는 동적 회귀 R² (실검증: 예시 미션
    #    실로그에서 2.2712kg / 실측 2.2726kg, 오차 0.06%)
    zdd = np.gradient(sig["real_vz"], t)
    airborne = (sig["real_z"] > Z_AIRBORNE_M) & \
               (np.abs(np.degrees(sig["real_roll"])) < TILT_MAX_DEG) & \
               (np.abs(np.degrees(sig["real_pitch"])) < TILT_MAX_DEG)
    if not np.any(airborne):
        raise ValueError(
            f"질량 추정 불가: z>{Z_AIRBORNE_M}m 공중 구간 없음 (저고도에선 "
            "현수 짐이 지면 접촉 -> 추력이 총질량을 안 듦)")
    T_eff = sum(T_all) * np.cos(sig["real_roll"]) * np.cos(sig["real_pitch"])
    m_dyn, r2_m = _lin_fit((zdd + G)[airborne], T_eff[airborne])
    quasi = airborne & (np.abs(sig["real_vz"]) < 0.1) & (np.abs(zdd) < 0.1)
    m_hat = float(np.mean(T_eff[quasi]) / G) if np.any(quasi) else m_dyn
    out["mass_kg"] = {"value": m_hat, "unit": "kg", "r2": r2_m,
                      "note": "기체+짐 총질량. 값=공중 준정적 평형, r2=동적 회귀"}

    # 2) K̂_drag: yaw 각가속 = (kd/Izz)·Σ(±w²), Izz 공칭 전제
    yaw_dd = _ang_accel(t, np.unwrap(sig["real_yaw"]))
    slope_d, r2_d, combo_d = _best_combo_fit(w2_all, yaw_dd)
    out["k_drag_lumped"] = {
        "value": abs(slope_d) * IZZ_NOMINAL, "unit": "N·m/(rad/s)^2",
        "r2": r2_d, "assumes": {"Izz_nominal": IZZ_NOMINAL},
        "sign_combo": combo_d}

    # 4) 관성: roll/pitch 각가속 = (L/I)·ΔT → I = L/slope
    for axis, sig_name in (("Ixx", "real_roll"), ("Iyy", "real_pitch")):
        acc = _ang_accel(t, sig[sig_name])
        slope, r2, combo = _best_combo_fit(T_all, acc)
        val = ARM_LENGTH_M / abs(slope) if abs(slope) > 1e-12 else None
        out.setdefault("inertia", {})[axis] = {
            "value": val, "unit": "kg·m^2", "r2": r2,
            "assumes": {"arm_length_m": ARM_LENGTH_M}, "sign_combo": combo}
    # Izz: yaw 회귀 기울기 = kd/Izz에 K_drag 공칭 대입 (K̂_drag와 상보 —
    # 같은 회귀에서 하나를 알면 다른 쪽이 나옴)
    izz = KDRAG_NOMINAL / abs(slope_d) if abs(slope_d) > 1e-12 else None
    out["inertia"]["Izz"] = {
        "value": izz, "unit": "kg·m^2", "r2": r2_d,
        "assumes": {"Kdrag_nominal": KDRAG_NOMINAL}}

    # 신뢰도 판정
    def _walk(d):
        for k, v in d.items():
            if isinstance(v, dict):
                if "r2" in v:
                    v["confident"] = bool(v["r2"] >= R2_CONFIDENT)
                else:
                    _walk(v)
    _walk(out)
    return out


def write_estimate(est, path=ESTIMATE_PATH, meta_path=META_PATH):
    traj_hash = None
    if os.path.isfile(meta_path):
        with open(meta_path, encoding="utf-8") as f:
            traj_hash = json.load(f).get("trajectory_hash")
    doc = {"estimated_at": datetime.now().strftime(TS_FMT),
           "trajectory_hash": traj_hash,
           "r2_confident_threshold": R2_CONFIDENT,
           "estimates": est,
           "usage": "confident:true 항목만 parameters.m 반영 권장 "
                    "(sT/sQ 자동 스케일 — 급변 방지 램프 권장)"}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _atomic_write_json(path, doc)
    print(f"[write] {path}")
    return doc


def main():
    ap = argparse.ArgumentParser(description="모터↔센서 회귀 플랜트 상수 추정")
    ap.add_argument("--mat", default=DEFAULT_MAT)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    est = estimate(args.mat)
    print(json.dumps(est, indent=2, ensure_ascii=False))
    if not args.dry_run:
        write_estimate(est)


if __name__ == "__main__":
    main()
