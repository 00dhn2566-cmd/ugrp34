#!/usr/bin/env python3
"""성진 산출물 → 윤호(Isaac Sim) 기대 형식 변환기.

성진 내부 계약(INTERFACE_SPEC §2 trajectory.json, 비행 로그 .mat)과 윤호
`reinforcement_yunho/interface/`의 두 스키마는 **모양이 다르다**. 이 모듈이 그
경계를 넘긴다 — 여기 말고 다른 곳에서 isaacsim_* 파일을 만들지 말 것.

    §2 output/trajectory.json                 -> output/isaacsim_trajectory.json
      {dt, trajectory_hash, controller_profile,   {fps, frames:[{time, position,
       t[], pos[][3], yaw_rad[]}                    yaw_rad, orientation_quat_wxyz}]}

    비행 로그 sim_result*.mat                 -> output/isaacsim_motor_commands.json
      {sim_time, motor_cmd_w1..motor_cmd_w4}      {fps, frames:[{time, motor_cmd_w[4]}]}

주의 (윤호 스키마는 `additionalProperties: false`):
  - 스키마에 없는 키는 **한 개도** 넣을 수 없다. `trajectory_hash`,
    `controller_profile`, `written_at`을 동봉하고 싶어도 못 넣는다 — 대조가
    필요하면 별도 파일(`*.meta.json`)로 빼고, 이 두 파일은 순수 스키마만 유지.
  - `time`은 float 초 (성진 제어 클럭). 정수 나노초는 비전/VIO 스트림 몫.
  - 쿼터니언은 **WXYZ**, world<-body, yaw-only: [cos(y/2), 0, 0, sin(y/2)].
    (윤호 rl/ 쪽 drone_state는 XYZW라 서로 다르다 — 여기서는 interface/ 규약을 따른다.)

`motor_cmd_w`의 뜻 (윤호 스키마 `isaacsim_motor_commands.schema.json`,
CONVENTIONS.md "Control→sim boundary is the per-motor angular-velocity setpoint"):
  - 로터별 각속도 **setpoint** (제어기 믹서 출력, rad/s). 플랜트가 실제로 돈 속도
    (Simscape 실측 `prop1_w..prop4_w`)가 **아니다** — 모터/프로펠러 물리는 Isaac
    Sim 몫이라 실측값을 넘기면 플랜트를 두 번 돌리는 셈이 된다.
  - **부호 없음**(unsigned). CW/CCW 회전 방향은 Isaac Sim 로터 설정이 정하므로
    크기만 내보낸다 (Simulink 모터 2·3은 내장 역회전이라 실측 w가 음수로 찍히는데,
    그 부호는 플랜트 사정이지 명령의 일부가 아니다 — TUNING_STATUS 9차).
  - 로그 변수: 제어기 setpoint 채널 `motor_cmd_w1..motor_cmd_w4`(sample/run_sample_sim.m
    이 남기는 이름, 단위 rad/s)를 읽는다. 구운 모델(run_traj_baked) 로그처럼
    setpoint 채널이 없고 실측 `prop*_w`만 있으면 **기본은 오류** — `--allow-measured`
    를 주면 |prop*_w| 로 대체하되 그 사실을 stdout 에 ★경고로 남긴다 (대역일 뿐
    계약값이 아님). 구운 모델에 setpoint 로깅을 추가하는 것이 정답(controller_cpp
    README "cmd 탭 추가").

★ 미확정: 로터 인덱스 → 기하/회전방향 매핑은 **윤호 결정 대기**
  (interface/README.md). 이 모듈은 `motor_cmd_w = [w1, w2, w3, w4]` 를
  **Simulink 모터 1~4 번호 순서 그대로** 내보낸다. 윤호가 매핑을 공표하면
  --rotor-order 로 재배열할 것.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

DEFAULT_OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def _atomic_write_json(path: str, obj) -> None:
    """임시파일→rename 원자적 쓰기 (§0 공통 규칙)."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def yaw_to_quat_wxyz(yaw_rad: float):
    """yaw-only 회전 → 단위 쿼터니언 [w, x, y, z] (world<-body)."""
    half = 0.5 * float(yaw_rad)
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


# ---------------------------------------------------------------------------
# §2 trajectory.json -> isaacsim_trajectory.json
# ---------------------------------------------------------------------------

def trajectory_to_isaacsim(traj: dict) -> dict:
    """§2 궤적 dict → 윤호 isaacsim_trajectory 스키마 dict."""
    for key in ("dt", "t", "pos", "yaw_rad"):
        if key not in traj:
            raise KeyError(f"trajectory.json에 필수 키 '{key}' 없음")
    t, pos, yaw = traj["t"], traj["pos"], traj["yaw_rad"]
    if not (len(t) == len(pos) == len(yaw)):
        raise ValueError(f"길이 불일치: t={len(t)} pos={len(pos)} yaw={len(yaw)}")

    dt = float(traj["dt"])
    if dt <= 0:
        raise ValueError(f"dt는 양수여야 함 (현재 {dt})")

    frames = []
    for ti, pi, yi in zip(t, pos, yaw):
        if len(pi) != 3:
            raise ValueError(f"pos 항목은 [x,y,z]여야 함 (현재 {len(pi)}개)")
        frames.append({
            "time": float(ti),
            "position": [float(c) for c in pi],
            "yaw_rad": float(yi),
            "orientation_quat_wxyz": yaw_to_quat_wxyz(yi),
        })
    return {"fps": 1.0 / dt, "frames": frames}


# ---------------------------------------------------------------------------
# 비행 로그 .mat -> isaacsim_motor_commands.json
# ---------------------------------------------------------------------------

SETPOINT_KEY = "motor_cmd_w{idx}"     # 제어기 로터 각속도 setpoint (rad/s) — 계약값
MEASURED_KEY = "prop{idx}_w"          # Simscape 실측 로터 속도 (signed) — 대역 전용


def _mat_signal(v):
    """To Workspace StructureWithTime 또는 Array 둘 다 허용."""
    import numpy as np
    vals = getattr(getattr(v, "signals", None), "values", v)
    return np.asarray(vals, float).ravel()


def flight_mat_to_motor_commands(mat_path: str, rotor_order=(1, 2, 3, 4),
                                 allow_measured: bool = False,
                                 info: dict | None = None) -> dict:
    """비행 로그 .mat (sim_time + motor_cmd_w{1..4}) → 모터 명령 스키마 dict.

    `motor_cmd_w`는 스키마대로 **로터 각속도 setpoint, unsigned** 이다.
    - 우선 setpoint 채널 `motor_cmd_w{i}` 를 읽는다.
    - 없으면 KeyError. `allow_measured=True` 일 때만 실측 `prop{i}_w` 의 절댓값으로
      대체한다 (플랜트 상태를 명령 자리에 넣는 대역 — 계약값이 아님).
    - 어느 쪽이든 abs() 를 적용한다: CW/CCW 부호는 Isaac Sim 로터 설정 몫.

    rotor_order: 출력 인덱스 0..3에 넣을 Simulink 모터 번호. 기본은 항등
    (1,2,3,4) — 윤호가 인덱스→기하 매핑을 공표하기 전까지의 잠정값.
    info: dict 를 주면 `source` ("setpoint" | "measured") 를 채워 준다.
    """
    try:
        from scipy.io import loadmat
    except ImportError as exc:                       # pragma: no cover
        raise SystemExit(f"scipy 필요: {exc}") from exc

    S = loadmat(mat_path, squeeze_me=True, struct_as_record=False)
    if "sim_time" not in S:
        raise KeyError(f"{mat_path}에 sim_time 없음 (run_sample_sim/run_traj_baked 로깅 확인)")

    import numpy as np
    t = np.asarray(S["sim_time"], float).ravel()

    have_setpoint = all(SETPOINT_KEY.format(idx=i) in S for i in rotor_order)
    if have_setpoint:
        source = "setpoint"
    elif allow_measured and all(MEASURED_KEY.format(idx=i) in S for i in rotor_order):
        source = "measured"
    else:
        missing = [SETPOINT_KEY.format(idx=i) for i in rotor_order
                   if SETPOINT_KEY.format(idx=i) not in S]
        raise KeyError(
            f"{mat_path}에 로터 setpoint 채널 {missing} 없음 - motor_cmd_w는 "
            "제어기 setpoint(unsigned rad/s)다. 실측 prop*_w 로 대역하려면 "
            "allow_measured=True (--allow-measured) 를 명시할 것")
    if info is not None:
        info["source"] = source

    key = SETPOINT_KEY if source == "setpoint" else MEASURED_KEY
    # unsigned: CW/CCW 회전 방향은 Isaac Sim 로터 설정이 정한다 (스키마 주석)
    cols = [np.abs(_mat_signal(S[key.format(idx=idx)])) for idx in rotor_order]

    n = min(len(t), *(len(c) for c in cols))
    if n == 0:
        raise ValueError("로그 길이 0 — 시뮬 결과 확인")
    t, cols = t[:n], [c[:n] for c in cols]

    # fps는 명목값 (모터 레이트는 가변 — 프레임별 time이 권위, 스키마 주석 참조)
    span = float(t[-1] - t[0])
    fps = (n - 1) / span if span > 0 else 1.0

    frames = [{"time": float(t[k]),
               "motor_cmd_w": [float(c[k]) for c in cols]}
              for k in range(n)]
    return {"fps": fps, "frames": frames}


# ---------------------------------------------------------------------------
# 검증 (윤호 검증기가 닿으면 사용, 없으면 건너뜀)
# ---------------------------------------------------------------------------

def _yunho_validate(obj: dict, kind: str) -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    ifc_dir = os.path.join(here, os.pardir, "reinforcement_yunho", "interface")
    ifc_dir = os.path.abspath(ifc_dir)
    if not os.path.isfile(os.path.join(ifc_dir, "schemas.py")):
        return "검증 생략 (윤호 interface/ 없음)"
    sys.path.insert(0, ifc_dir)
    try:
        import schemas as ifc
        ifc.validate(obj, kind=kind)
        return "윤호 스키마 VALID"
    except Exception as exc:                          # noqa: BLE001
        return f"윤호 스키마 REJECT: {str(exc).splitlines()[0][:120]}"
    finally:
        sys.path.remove(ifc_dir)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="성진 산출물 → 윤호(Isaac Sim) 기대 형식으로 변환")
    ap.add_argument("--trajectory", help="§2 trajectory.json 경로")
    ap.add_argument("--flight-mat", help="비행 로그 sim_result_baked.mat 경로")
    ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR, help="출력 디렉터리")
    ap.add_argument("--rotor-order", default="1,2,3,4",
                    help="출력 인덱스 0..3 ← Simulink 모터 번호 (윤호 매핑 확정 시 변경)")
    ap.add_argument("--allow-measured", action="store_true",
                    help="로그에 setpoint 채널(motor_cmd_w1..4)이 없을 때 실측 |prop*_w| 로 "
                         "대체 (대역 — 계약값 아님, ★경고 출력)")
    args = ap.parse_args(argv)

    if not args.trajectory and not args.flight_mat:
        ap.error("--trajectory 또는 --flight-mat 중 하나는 필요")
    os.makedirs(args.out_dir, exist_ok=True)

    if args.trajectory:
        with open(args.trajectory, encoding="utf-8") as f:
            traj = json.load(f)
        out = trajectory_to_isaacsim(traj)
        path = os.path.join(args.out_dir, "isaacsim_trajectory.json")
        _atomic_write_json(path, out)
        print(f"[write] {path}  frames={len(out['frames'])} fps={out['fps']:.1f}"
              f"  ({_yunho_validate(out, 'trajectory')})")
        if traj.get("trajectory_hash"):
            # 스키마에 못 넣는 대조 정보는 별도 파일로 (additionalProperties:false)
            meta = os.path.join(args.out_dir, "isaacsim_trajectory.meta.json")
            _atomic_write_json(meta, {
                "trajectory_hash": traj.get("trajectory_hash"),
                "controller_profile": traj.get("controller_profile"),
                "source": os.path.basename(args.trajectory),
            })
            print(f"[write] {meta}  (스키마 외 대조 정보)")

    if args.flight_mat:
        order = tuple(int(x) for x in args.rotor_order.split(","))
        if sorted(order) != [1, 2, 3, 4]:
            raise SystemExit(f"--rotor-order는 1~4의 순열이어야 함: {args.rotor_order}")
        info = {}
        out = flight_mat_to_motor_commands(args.flight_mat, rotor_order=order,
                                           allow_measured=args.allow_measured,
                                           info=info)
        path = os.path.join(args.out_dir, "isaacsim_motor_commands.json")
        _atomic_write_json(path, out)
        w = [fr["motor_cmd_w"] for fr in out["frames"]]
        flat = [v for row in w for v in row]
        print(f"[write] {path}  frames={len(out['frames'])} fps={out['fps']:.1f}"
              f"  ({_yunho_validate(out, 'motor')})")
        print(f"        w 범위 {min(flat):.1f} ~ {max(flat):.1f} rad/s (unsigned setpoint), "
              f"rotor_order=모터{order}  ★인덱스→기하 매핑은 윤호 확정 대기")
        if info.get("source") == "measured":
            print("        ★경고: setpoint 채널 없음 → 실측 |prop*_w| 로 대체함. "
                  "계약값(제어기 setpoint)이 아니라 대역이다 - 구운 모델에 "
                  "motor_cmd_w1..4 로깅을 추가할 것")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
