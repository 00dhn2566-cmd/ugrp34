"""표준 기하 제어기 (Lee et al.) — 파이프라인 검증용 **대체** 제어기.

    from control import simple
    ctl = simple.SimpleController(mass=2.2726, I=(0.0171, 0.0171, 0.0212))
    ctl.apply(p, body, ref_pos, ref_yaw, client)

왜 이게 있나
------------
성진 제어기는 이 PyBullet 플랜트에서 **구간을 나눠 날면** 매번 전복한다. 원인은
그의 ``QcInput`` 에 각속도 필드가 없어서(자세 감쇠가 각도 후방차분 D 항 하나뿐)
직전 구간이 남긴 회전을 못 죽이는 것이다 — 그의 README 가 "해법은 속도 캐스케이드
(이식 후 1순위 개선 항목)" 라고 적어둔 그 항목이다.

이 파일은 그 문제를 우회해서 **비전→계획→통과 파이프라인 자체를 검증**하기 위한
것이다. 각속도를 직접 받아 자세 루프를 닫으므로 구간 전환에서 안 죽는다.

⚠ **이건 성진 제어기가 아니다.** 이 제어기로 나온 결과를 "성진 제어기가 창문을
통과했다" 로 읽으면 안 된다. 그쪽 통합은 여전히 미해결이고, 그가 [TODO-verify] 로
남긴 항목들과 각속도 피드백 부재가 남아 있다.

수식 (Lee, Leok, McClamroch 2010 — SE(3) 기하 제어)
    a_des  = kp·e_p + kd·e_v + g·ẑ
    thrust = m · a_des · R[:,2]                 (body z 로 투영)
    b3_des = a_des / |a_des|                    (원하는 기체 위쪽 방향)
    e_R    = ½·vee(R_desᵀR − RᵀR_des)
    torque = −k_R·e_R − k_Ω·Ω
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

G = 9.81


def _vee(M: np.ndarray) -> np.ndarray:
    return np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]]) * 0.5


class SimpleController:
    """위치 PD + SE(3) 자세 제어. 각속도를 직접 받으므로 구간 전환에 강건하다."""

    def __init__(self, mass: float, I=(0.0171, 0.0171, 0.0212),
                 kp=6.0, kd=4.5, kR=0.9, kW=0.28,
                 tilt_max_deg: float = 30.0, thrust_max: float = None):
        self.m = float(mass)
        self.I = np.diag(np.asarray(I, float))
        self.kp, self.kd, self.kR, self.kW = kp, kd, kR, kW
        self.tilt_max = np.radians(tilt_max_deg)
        self.thrust_max = thrust_max or (2.0 * self.m * G)   # T/W 2.0
        self.last = {}

    def compute(self, ref_pos, ref_yaw: float, pos, vel, R: np.ndarray,
                omega, ref_vel=None) -> Tuple[float, np.ndarray]:
        """반환 (총추력 [N], body 토크 [N·m] (3,)).

        ref_vel 을 안 주면 "기준이 정지해 있다" 고 가정한다. 기준이 1.1 m/s 로
        움직이는 구간에서는 그 가정이 통째로 속도 오차가 되어, 앞에서는 뒤처지고
        기준이 멈추면 붙은 속도를 감쇠만으로 죽이느라 오버슛한다.
        실측: 기준 x=0.84 에 멈췄는데 드론은 1.83 까지 갔다 (오차 1001 mm).
        """
        e_p = np.asarray(ref_pos, float) - np.asarray(pos, float)
        rv = np.zeros(3) if ref_vel is None else np.asarray(ref_vel, float)
        e_v = rv - np.asarray(vel, float)
        a_des = self.kp * e_p + self.kd * e_v + np.array([0.0, 0.0, G])

        # 기울기 제한: 수평 성분을 잘라 과도한 자세 명령을 막는다
        a_h, a_z = a_des[:2], max(a_des[2], 0.5 * G)
        h_max = a_z * np.tan(self.tilt_max)
        nh = np.linalg.norm(a_h)
        if nh > h_max:
            a_h = a_h * (h_max / nh)
        a_des = np.array([a_h[0], a_h[1], a_z])

        b3 = a_des / np.linalg.norm(a_des)
        thrust = float(np.clip(self.m * float(a_des @ R[:, 2]), 0.0, self.thrust_max))

        b1c = np.array([np.cos(ref_yaw), np.sin(ref_yaw), 0.0])
        b2 = np.cross(b3, b1c)
        n2 = np.linalg.norm(b2)
        if n2 < 1e-6:                       # b3 가 b1c 와 나란하면 대체 축
            b1c = np.array([0.0, 1.0, 0.0])
            b2 = np.cross(b3, b1c); n2 = np.linalg.norm(b2)
        b2 /= n2
        R_des = np.column_stack([np.cross(b2, b3), b2, b3])

        e_R = _vee(R_des.T @ R - R.T @ R_des)
        e_W = np.asarray(omega, float)      # 목표 각속도 0
        tau = -self.kR * (self.I @ e_R) - self.kW * (self.I @ e_W)
        self.last = {"thrust": thrust, "tau": tau, "tilt_cmd": float(
            np.degrees(np.arccos(np.clip(b3[2], -1, 1))))}
        return thrust, tau

    def apply(self, p, body: int, ref_pos, ref_yaw: float, client: int,
              ref_vel=None) -> None:
        """PyBullet 강체에서 상태를 읽어 힘/토크를 인가한다 (한 스텝)."""
        pos, quat = p.getBasePositionAndOrientation(body, physicsClientId=client)
        vel, omega = p.getBaseVelocity(body, physicsClientId=client)
        R = np.array(p.getMatrixFromQuaternion(quat), float).reshape(3, 3)
        thrust, tau = self.compute(ref_pos, ref_yaw, pos, vel, R, omega, ref_vel)
        p.applyExternalForce(body, -1, forceObj=[0.0, 0.0, thrust],
                             posObj=[0.0, 0.0, 0.0], flags=p.LINK_FRAME,
                             physicsClientId=client)
        # 토크는 월드로 변환해서 인가 (base 링크의 LINK_FRAME 토크는 무시된다)
        p.applyExternalTorque(body, -1, torqueObj=[float(v) for v in (R @ tau)],
                              flags=p.WORLD_FRAME, physicsClientId=client)
