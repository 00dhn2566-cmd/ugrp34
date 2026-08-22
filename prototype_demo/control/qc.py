"""성진 C++ 제어기 ctypes 바인딩 + PyBullet 플랜트 접합.

    from control import qc
    ctl = qc.Controller(drone_mass=1.2726, pkg_mass=1.0)
    thrust, dragQ = ctl.step(ref_pos, ref_yaw, meas_pos, meas_rpy, dt)

이 파일이 하는 일은 딱 두 가지다.
  1. ``libqc_bridge.so`` 를 열어 ``qc_step`` + ``qc_motor`` 를 파이썬에서 부른다.
  2. 그 결과(모터별 thrust[N], dragQ[N·m])를 PyBullet 강체에 인가한다 —
     성진이 ``main_trace.cpp:198`` 에 [PLANT HOOK] 으로 지정해 둔 접합점.

⚠ **폐루프 검증 상태**
그의 ``docs/HANDOFF_CPP_GAZEBO.md`` 는 미확정 4건(믹서 차동 부호표, 측정 필터 배선,
고도 PID 클램프, RBI 회전 완전성)이 골든 트레이스로 닫히기 전에는 **폐루프 비행
금지**라고 못박아 두었다. 골든 트레이스는 MATLAB 이 있어야 뽑는데 여기엔 없다.
따라서 이 모듈로 얻은 폐루프 결과는 **잠정**이며, 발산하더라도 그것이 곧 그의
제어기가 틀렸다는 뜻은 아니다 (배선 가정이 틀렸을 수 있다).
그가 허용한 범위 — 빌드/스모크/인터페이스 왕복/[PLANT HOOK] 접합 코드 — 까지가
확정이고, 그 위는 전부 물음표다.
"""
from __future__ import annotations

import ctypes
import os
from typing import Tuple

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_LIB = os.path.join(_HERE, "libqc_bridge.so")

PROFILES = {"precision": 0, "balanced": 1, "agile": 2}

#: 프로펠러 반토크를 기체에 인가할지. **기본 False.**
#:
#: 이 플랜트에서 yaw 는 제어 불가다 (8개 가설 300회 폐루프 스윕으로 확인:
#: mixYaw 부호 3종, 토크 프레임, yaw 게인 3000배 대역, limMot, limYaw,
#: 모터 PI, 추중비 2.0~7.1 — 전부 배제. 근본 원인은 Cq=0.01517 이 호버 평형을
#: 맞추려고 역산한 값이라 실기 반토크보다 작다는 것).
#:
#: 제어할 수 없는데 인가만 하면 순수 외란이 된다. 그리고 그 외란이 yaw 를 87° 까지
#: 밀면 성진 제어기의 위치→자세 변환이 깨진다 — 그가 [TODO-verify] 로 남긴
#: "RBI 회전 완전성 (현재 yaw만 반영한 1차 근사)" 항목이다. 결과는 위치 발산·추락.
#:
#: 그래서 프로토타입에서는 끈다. yaw 를 안 쓰기로 했으므로 임무에 손실이 없다
#: (카메라 반화각 ±46.8° > 스캔 진폭이 만드는 시선각). 실기·Gazebo 검증에서는
#: 반드시 켜야 하고, 그때는 Cq 를 실측값으로 바꿔야 한다.
YAW_REACTION = False

#: 각속도 감쇠 토크 계수 [N·m/(rad/s)]. **임시 조치 — 실기엔 없는 힘이다.**
#:
#: 왜 필요한가: 성진 QcInput 에는 각속도 필드가 없다 (measRpy 만). 제어기는 각도를
#: 후방차분한 D 항으로만 감쇠를 만드는데, 그 대역이 좁아서 스캔이 남긴 각속도를
#: 못 죽인다. 실측: 스캔 종료 시 자세는 1.1° 로 멀쩡한데 |w| = 1.8 rad/s (103°/s)
#: 였고, **제자리 호버만 시켜도 0.2 초 만에 180° 로 전복**했다.
#:
#: 그의 README 가 "해법은 속도 캐스케이드 (이식 후 1순위 개선 항목)" 이라고 적어둔
#: 그 항목이다. 제대로 하려면 그의 구조체에 각속도를 넣고 내부 루프를 하나 더
#: 만들어야 하는데, 그건 그의 파일 수정이라 지금 방침(무수정) 밖이다.
#: 여기서는 플랜트 쪽에 점성 감쇠를 걸어 같은 효과를 낸다 — 물리적으로는
#: "회전에 저항하는 공기" 를 과장한 것이고, 실기·Gazebo 검증에서는 0 으로 두고
#: 속도 캐스케이드를 제대로 넣어야 한다.
ANG_DAMPING = 0.0

#: 각속도 리드 보상 [s]. 제어기에 넘기는 **자세 측정값**에 각속도를 섞는다:
#:
#:     rpy_eff = rpy + RATE_LEAD * omega
#:
#: 그의 QcInput 에는 각속도 필드가 없어서 자세 루프의 감쇠가 각도 후방차분 D 항
#: 하나뿐이고, 그 대역이 좁아 스캔이 남긴 회전(|w| 1.8 rad/s)을 못 죽인다.
#: 측정값에 rate 를 미리 섞어 주면 "곧 이만큼 더 기울 것" 을 제어기가 지금 보게 되어
#: 자세 루프에 rate 피드백이 하나 생긴 것과 같아진다 (리드 보상 = 내부 rate 루프의
#: 1차 등가). 그의 소스는 한 줄도 안 건드린다 — 우리가 만드는 입력만 바꾼다.
#:
#: 그의 README 가 "해법은 속도 캐스케이드 (이식 후 1순위 개선 항목)" 라고 적어둔
#: 그 항목의 최소 구현이다. 제대로 하려면 그의 구조체에 각속도를 넣고 내부 루프를
#: 따로 돌려야 한다.
RATE_LEAD = 0.0

#: PyBullet 플랜트에서 실측으로 찾은 설정 (scripts/sweep_layout.py + tune_gains.py).
#: 그의 소스는 무수정 — 전부 QcConfig/MotorParams 값과 우리 쪽 모터 배정이다.
#:
#: motor_order: 모터 인덱스 → 코너 배정. 순열 24가지 전수조사에서 [3,2,1,0] 만
#:   살아남았다 (96 조합 중 2개 생존, 나머지 하나는 이것의 180° 등가). 처음 추측한
#:   [0,1,2,3] 은 정확히 역순이라 자세 피드백이 양의 되먹임이 됐다.
#: gains: 자세 게인은 **그의 음수 부호 그대로** 두고 크기만 0.2배. 부호를 뒤집으면
#:   발산한다 — "자세 게인 음수는 의도" 라는 그의 주석이 여기서도 맞았다.
#:   위치 게인(kpPos 12 / kdPos 4.8 / pos2att 2.4)은 그의 기본값이 이미 최적이라 안 건드림.
#: alt_cmd_sat: 30 이면 바이어스(100.9 rev/s)가 클램프에 먹혀 추력이 호버의 8% 에 묶인다.
PYBULLET_TUNED = {
    "alt_cmd_sat": 120.0,
    "motor": dict(max_torque=1.6, max_power=400.0),     # T/W 2.0
    "motor_order": [3, 2, 1, 0],
    "gains": dict(kpAlt=60.0, kdAlt=24.0, kiAlt=0.5, limAlt=30.0,
                  kpAtt=-17.0, kiAtt=-2.0, kdAtt=-25.5, limAtt=10.0),
}


def _load():
    if not os.path.exists(_LIB):
        raise FileNotFoundError(
            f"{_LIB} 없음 — 먼저 빌드: bash {os.path.join(_HERE, 'build.sh')}")
    lib = ctypes.CDLL(_LIB)
    d = ctypes.POINTER(ctypes.c_double)
    lib.qcb_init.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double,
                             ctypes.c_int, ctypes.c_double]
    lib.qcb_init.restype = None
    lib.qcb_phys.argtypes = [d, d, d]
    lib.qcb_arm_geometry.argtypes = [d, d]
    lib.qcb_mixdir.argtypes = [d]
    lib.qcb_step.argtypes = [d, ctypes.c_double, d, d, ctypes.c_double,
                             d, d, d, d, d]
    lib.qcb_step.restype = None
    lib.qcb_set_mix.argtypes = [d, d, d, d]
    lib.qcb_set_motor.argtypes = [ctypes.c_double] * 6
    lib.qcb_set_cmd.argtypes = [ctypes.c_double] * 2
    lib.qcb_preset_w.argtypes = [ctypes.c_double]
    lib.qcb_max_thrust.argtypes = []
    lib.qcb_max_thrust.restype = ctypes.c_double
    lib.qcb_set_gains.argtypes = [ctypes.c_double] * 17
    lib.qcb_get_gains.argtypes = [d]
    lib.qcb_set_lims.argtypes = [ctypes.c_double] * 2
    lib.qcb_set_motor_pi.argtypes = [ctypes.c_double] * 2
    return lib


#: qcb_get_gains / qcb_set_gains 의 인자 순서
GAIN_NAMES = ("kpPos", "kiPos", "kdPos", "kpPosZ", "kdPosZ", "pos2att",
              "kpAtt", "kiAtt", "kdAtt", "limAtt",
              "kpAlt", "kiAlt", "kdAlt", "limAlt",
              "kpYaw", "kiYaw", "kdYaw")


def _arr(n=4):
    return (ctypes.c_double * n)()


def _p(a):
    return ctypes.cast(a, ctypes.POINTER(ctypes.c_double))


class Controller:
    """성진 제어기 한 대. 내부에 그의 모터 플랜트도 같이 돈다."""

    def __init__(self, drone_mass: float = 1.2726, pkg_mass: float = 1.0,
                 pkg_size: float = 0.14, profile: str = "balanced",
                 alt_cmd_sat: float = -1.0):
        """alt_cmd_sat < 0 이면 그의 기본값(30) 그대로. qc_bridge.cpp 주석 참고."""
        self.lib = _load()
        self.lib.qcb_init(float(drone_mass), float(pkg_mass), float(pkg_size),
                          PROFILES[profile], float(alt_cmd_sat))
        self.alt_cmd_sat = alt_cmd_sat
        self.drone_mass = float(drone_mass)
        self.pkg_mass = float(pkg_mass)
        self.profile = profile
        a, b, c = _arr(1), _arr(1), _arr(1)
        self.lib.qcb_phys(_p(a), _p(b), _p(c))
        self.I_att, self.I_yaw, self.m_tot = a[0], b[0], c[0]
        r, D = _arr(1), _arr(1)
        self.lib.qcb_arm_geometry(_p(r), _p(D))
        self.r_arm, self.prop_D = r[0], D[0]
        md = _arr(4)
        self.lib.qcb_mixdir(_p(md))
        self.mix_dir = np.array(list(md))
        # 모터 위치 (X 배치, body frame). 모터 순서는 그의 믹서표 기준:
        #   mixPitch +--+ / mixRoll --++  ->  1:(+x,-y) 2:(-x,-y) 3:(-x,+y) 4:(+x,+y)
        s = self.r_arm
        self.motor_xy = np.array([[+s, -s], [-s, -s], [-s, +s], [+s, +s]])
        self._buf = {k: _arr(4) for k in ("thrust", "dragQ", "w", "cmd")}
        self._pr = _arr(2)

    def step(self, ref_pos, ref_yaw: float, meas_pos, meas_rpy,
             dt: float, omega=None) -> Tuple[np.ndarray, np.ndarray]:
        """omega = 기체 각속도 (rad/s, world). RATE_LEAD > 0 이면 리드 보상에 쓴다."""
        rpy = np.asarray(meas_rpy, float)
        if omega is not None and RATE_LEAD > 0:
            rpy = rpy + RATE_LEAD * np.asarray(omega, float)
        rp = (ctypes.c_double * 3)(*[float(v) for v in ref_pos])
        mp = (ctypes.c_double * 3)(*[float(v) for v in meas_pos])
        mr = (ctypes.c_double * 3)(*[float(v) for v in rpy])
        b = self._buf
        self.lib.qcb_step(_p(rp), float(ref_yaw), _p(mp), _p(mr), float(dt),
                          _p(b["thrust"]), _p(b["dragQ"]), _p(b["w"]),
                          _p(b["cmd"]), _p(self._pr))
        self.last = {k: np.array(list(v)) for k, v in b.items()}
        self.last["cmd_pitch"], self.last["cmd_roll"] = self._pr[0], self._pr[1]
        return self.last["thrust"], self.last["dragQ"]

    def hover_thrust_ratio(self) -> float:
        """직전 스텝 총추력 / 중력. 1.0 근처면 호버 평형."""
        return float(self.last["thrust"].sum() / (self.m_tot * 9.81))

    # --- 임의 오버라이드 (그의 소스 무수정, QcConfig/MotorParams 값만 덮음) ----
    def set_mix(self, pitch=None, roll=None, yaw=None, direction=None) -> None:
        """믹서 차동 부호표. 그의 최우선 미확정 항목이라 자유롭게 실험 가능."""
        def c(v):
            return (ctypes.c_double * 4)(*[float(x) for x in v]) if v is not None else None
        self.lib.qcb_set_mix(_p(c(pitch)) if pitch is not None else None,
                             _p(c(roll)) if roll is not None else None,
                             _p(c(yaw)) if yaw is not None else None,
                             _p(c(direction)) if direction is not None else None)
        if direction is not None:
            self.mix_dir = np.asarray(direction, float)

    def set_motor(self, max_torque=-1, limit_cmd=-1, Ct=-1, Cq=-1,
                  Vbatt=-1, max_power=-1) -> None:
        """모터 플랜트. 기본값은 최대추력 = mg 라 상승 여력이 0이다."""
        self.lib.qcb_set_motor(float(max_torque), float(limit_cmd), float(Ct),
                               float(Cq), float(Vbatt), float(max_power))

    def set_cmd(self, dir_gain=0.0, cmd_lim_deg=-1.0) -> None:
        self.lib.qcb_set_cmd(float(dir_gain), float(cmd_lim_deg))

    def preset_w(self, w: float) -> None:
        """모터를 평형 회전수에서 출발시킨다 (패드 스핀업 대체)."""
        self.lib.qcb_preset_w(float(w))

    def apply_tuned(self, cfg: dict = None) -> "Controller":
        """PYBULLET_TUNED 를 적용한다. alt_cmd_sat 은 생성 시에만 먹으므로
        ``Controller(alt_cmd_sat=PYBULLET_TUNED['alt_cmd_sat'])`` 로 만들 것."""
        cfg = cfg or PYBULLET_TUNED
        self.set_motor(**cfg["motor"])
        self.set_gains(**cfg["gains"])
        self.motor_xy = self.motor_xy[cfg["motor_order"]]
        return self

    def set_lims(self, lim_mot: float = -1.0, lim_yaw: float = -1.0) -> None:
        """제어기 명령 클램프. 최대추력 = min(limMot, plant limitCmd) x maxTorque."""
        self.lib.qcb_set_lims(float(lim_mot), float(lim_yaw))

    def set_motor_pi(self, kp: float = -1.0, ki: float = -1.0) -> None:
        """모터 PI. yaw 대역폭을 정하는 값 — 기본 kpMot 0.00375, kiMot 4.5e-4."""
        self.lib.qcb_set_motor_pi(float(kp), float(ki))

    def get_gains(self) -> dict:
        a = _arr(len(GAIN_NAMES))
        self.lib.qcb_get_gains(_p(a))
        return dict(zip(GAIN_NAMES, list(a)))

    def set_gains(self, **kw) -> None:
        """게인 덮어쓰기 (이름은 GAIN_NAMES). 내부에서 qc_bind 를 다시 부른다.

        자세 게인이 음수인 건 그의 플랜트 이득이 음수라서다 — 우리 PyBullet 플랜트는
        부호 규약이 다를 수 있으므로 부호까지 열어 둔다.
        """
        bad = set(kw) - set(GAIN_NAMES)
        if bad:
            raise KeyError(f"모르는 게인 {bad} — 가능: {GAIN_NAMES}")
        nan = float("nan")
        self.lib.qcb_set_gains(*[float(kw.get(n, nan)) for n in GAIN_NAMES])

    def max_thrust(self) -> float:
        """현재 모터 설정의 최대 총추력 [N]."""
        return float(self.lib.qcb_max_thrust())

    def thrust_to_weight(self) -> float:
        return self.max_thrust() / (self.m_tot * 9.81)


def apply_to_body(p, body: int, thrust: np.ndarray, dragQ: np.ndarray,
                  motor_xy: np.ndarray, mix_dir: np.ndarray, client: int) -> None:
    """[PLANT HOOK] — 모터별 추력/반토크를 PyBullet 강체에 인가한다.

    추력: 각 모터 위치에서 body +z 방향 힘 (LINK_FRAME) → 롤/피치 모멘트가 자동 생성.
    반토크: 프로펠러 공력 반작용. 회전 방향(mixDir)의 **반대** 부호로 yaw 축에 건다.
    """
    for i in range(4):
        p.applyExternalForce(body, -1, forceObj=[0, 0, float(thrust[i])],
                             posObj=[float(motor_xy[i, 0]), float(motor_xy[i, 1]), 0.0],
                             flags=p.LINK_FRAME, physicsClientId=client)
    # 반토크는 **월드 프레임**으로 직접 변환해서 건다.
    # pybullet 의 applyExternalTorque 는 base(link -1) 에 LINK_FRAME 으로 주면
    # 무시되는 것으로 실측됐다 (yaw 40° 명령에 기체가 3° 도 안 돌았다).
    # 계산상 토크는 0.32 N·m, I_yaw 0.021 이라 865 °/s^2 가 나와야 정상이다.
    if ANG_DAMPING > 0:
        _, w_ang = p.getBaseVelocity(body, physicsClientId=client)
        tau_d = -ANG_DAMPING * np.asarray(w_ang, float)
        p.applyExternalTorque(body, -1, torqueObj=[float(v) for v in tau_d],
                              flags=p.WORLD_FRAME, physicsClientId=client)
    if not YAW_REACTION:
        return                      # 반토크 생략 — YAW_REACTION 주석 참고
    yaw_torque = float(-(mix_dir * dragQ).sum())
    _, quat = p.getBasePositionAndOrientation(body, physicsClientId=client)
    R = np.array(p.getMatrixFromQuaternion(quat), dtype=float).reshape(3, 3)
    tau_w = R @ np.array([0.0, 0.0, yaw_torque])
    p.applyExternalTorque(body, -1, torqueObj=[float(v) for v in tau_w],
                          flags=p.WORLD_FRAME, physicsClientId=client)


def make_body(p, client: int, mass: float, I_att: float, I_yaw: float,
              r_arm: float, start=(0.0, 0.0, 1.0)) -> int:
    """제어기의 물성(qc_phys 결과)과 **같은 질량·관성**을 가진 기체를 만든다.

    CF2X(27 g) URDF 를 쓰면 안 된다 — 성진 게인은 1.2726 kg + 1 kg 짐 기준이다.
    형상은 시각용일 뿐이고, 물리는 질량/관성 텐서가 전부다.
    """
    col = p.createCollisionShape(p.GEOM_BOX,
                                 halfExtents=[r_arm, r_arm, 0.04],
                                 physicsClientId=client)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[r_arm, r_arm, 0.04],
                              rgbaColor=[0.15, 0.15, 0.18, 1], physicsClientId=client)
    body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=col,
                             baseVisualShapeIndex=vis, basePosition=list(start),
                             physicsClientId=client)
    p.changeDynamics(body, -1, localInertiaDiagonal=[I_att, I_att, I_yaw],
                     linearDamping=0.0, angularDamping=0.0, physicsClientId=client)
    return body
