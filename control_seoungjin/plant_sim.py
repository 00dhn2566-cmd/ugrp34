"""독립 6-DOF 플랜트 수치적분기 — 골든 트레이스 대조용.

목적
----
`docs/DYNAMICS.md`에 복원한 운동방정식을 **Simscape와 완전히 독립적으로** 구현한다.
같은 초기조건·같은 모터 입력을 두 쪽에 넣고 궤적이 일치하면,
"문서의 식이 곧 시뮬레이터가 푸는 식"임이 주장이 아니라 실증이 된다.

구현한 식 (DYNAMICS.md 대응)
---------------------------
§3.1  m v̇_W = R [0,0,ΣT] + [0,0,-mg] + F_drag
§3.2  I ω̇ + ω×(Iω) = τ_prop + τ_drag
§4    자세는 쿼터니언 적분 (오일러각은 출력 시 변환)
§5.1  T = Ct ρ n² D⁴,  Q = Cq ρ n² D⁵      (n: rev/s)
§6.1  f = -sign(v) (ρ/2) A Cd v²
§10.3 J ṅ = τ - Q - b n

주의
----
- 계수는 **물리 현실 쌍**(Ct=0.1072, Cq=0.01517, n은 rev/s)을 쓴다.
  모델 내부의 kT=9.79는 단위 흡수값이라 여기서는 쓰지 않는다 (DYNAMICS.md §5.3).
- 믹서 부호표(MIXER)와 유효 모멘트 암(LEVER_EFF)은 골든 트레이스 잔차 대조로
  확정됐다 (`compare_golden.py`, 상관 +0.92). 개별 모터 암은 식별 불가.

사용
----
    python plant_sim.py            # 자체 검증 (호버 평형 / 자유낙하 / 스핀다운)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


# ---------------------------------------------------------------- 파라미터


@dataclass
class PlantParams:
    """기체 물성·공력 상수. 기본값은 짐 1 kg 구성 (DYNAMICS.md §9)."""

    # 질량·관성 (CoM 기준)
    mass: float = 2.2726
    inertia: np.ndarray = field(
        default_factory=lambda: np.diag([1.71095e-2, 1.71595e-2, 2.12362e-2])
    )
    arm: float = 0.225 / math.sqrt(2)      # X-쿼드 모터 반경 [m]

    # 환경
    g: float = 9.80665
    rho: float = 1.225

    # 프로펠러 (물리 현실 쌍)
    diameter: float = 0.254
    ct: float = 0.1072
    cq: float = 0.01517

    # 모터
    rotor_inertia: float = 1.26e-5        # [kg m^2], 시정수 0.02 s 역산
    rotor_damping: float = 1e-7           # [N m /(rad/s)]
    max_torque: float = 0.8               # [N m]
    max_power: float = 160.0              # [W]

    # 모멘트 암 해석 선택 (DYNAMICS.md §11 #5)
    #   False = 네 로터 균등 기여 (X-쿼드 가정, 현재 기본)
    #   True  = CAD 기하 기반 비균등 가중치 (LEVER_GEOM)
    use_geometric_lever: bool = False

    # 공력 항력 — 병진 / 회전
    cd_lin: np.ndarray = field(default_factory=lambda: np.array([0.35, 0.35, 0.6]))
    area_lin: np.ndarray = field(default_factory=lambda: np.array([0.0875, 0.0900, 0.2560]))
    cd_rot: np.ndarray = field(default_factory=lambda: np.array([0.2, 0.2, 0.2]))
    area_rot: np.ndarray = field(default_factory=lambda: np.array([0.512, 0.512, 0.256]))

    def hover_speed_rps(self) -> float:
        """호버 평형 프로펠러 회전속도 [rev/s]."""
        thrust_each = self.mass * self.g / 4.0
        return math.sqrt(thrust_each / (self.ct * self.rho * self.diameter**4))


# 믹서 차동 부호표 — [roll, pitch] × [M1..M4]
#
# 골든 트레이스 잔차 대조로 확정 (2026-08-18, `compare_golden.py`).
#
# 방법: 각 축을 강하게 여기하는 런에서 프로펠러 차동량과 **각가속도**를 최소제곱 대조.
#   x이동 런: pitch 각가속 20.1 rad/s^2, 상관 +0.916, 유효계수 0.09286 m
#   y이동 런: roll  각가속 20.4 rad/s^2, 상관 +0.918, 유효계수 0.09320 m
# 두 유효계수가 0.4% 이내로 일치 -> roll/pitch 대칭 자기검증 통과.
#
# ⚠ 이전 판(정착 각도로 유추한 부호)은 **틀렸다**. 일정 각도를 유지하는 구간은
#   순토크가 0에 가까워 각도-차동 관계로 토크 부호를 유추할 수 없다.
#   각가속도와의 직접 상관이 옳은 검정이다 (뉴턴 법칙, 순간값).
MIXER = np.array(
    [
        [-1.0, +1.0, -1.0, +1.0],   # roll
        [+1.0, +1.0, -1.0, -1.0],   # pitch
    ]
)

# 유효 모멘트 암 [m] — 위 두 런의 최소제곱 이득 평균.
# 가정했던 X-쿼드 균등값 0.159 m 가 아니다.
# ⚠ 개별 모터의 암은 **식별 불가**: 호버에서 네 추력이 거의 같아 4파라미터 적합의
#   조건수가 1.1e4 이고 R^2 개선이 없다 (0.8404 vs 1파라미터 0.839).
#   개별 암을 보려면 모터를 독립 가진해야 한다 (개루프 명령 필요).
LEVER_EFF = 0.0930

# 각 모터의 회전 방향. 호버 실측 w = [+632.8, -632.3, -635.1, +634.6] 에서 확정.
# yaw 반토크는 회전 방향의 반대이므로 τ_yaw ∝ -SPIN_DIR·Q = [-1,+1,+1,-1]·Q,
# 이는 위 yaw 실측 차동 부호와 일치한다.
SPIN_DIR = np.array([+1.0, -1.0, -1.0, +1.0])


# 기하 기반 모멘트 암 [m] — CAD 암 Transform 방향에서 환산 (DYNAMICS.md §11.0).
# 로터는 90도 간격이되 축에서 -11.7도 회전한 "+" 배치다 (45도 X-쿼드가 아님).
#   각도  M1 78.3, M2 -11.7, M3 168.3, M4 -101.7 [deg],  반지름 R = 0.225 m
# roll  성분 = R sin(theta),  pitch 성분 = -R cos(theta)  (부호는 실측 믹서에 맞춤)
_THETA = np.deg2rad([78.3, -11.7, 168.3, -101.7])
_R_ROTOR = 0.225
LEVER_GEOM = np.vstack([
    _R_ROTOR * np.sin(_THETA),     # roll
    -_R_ROTOR * np.cos(_THETA),    # pitch
])


# ---------------------------------------------------------------- 쿼터니언


def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """쿼터니언 [w,x,y,z] -> 회전행렬 (body -> world)."""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_derivative(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    """q̇ = 0.5 * q ⊗ [0, ω_body]."""
    w, x, y, z = q
    p, r, s = omega_body
    return 0.5 * np.array(
        [
            -x * p - y * r - z * s,
            w * p + y * s - z * r,
            w * r - x * s + z * p,
            w * s + x * r - y * p,
        ]
    )


def quat_to_euler_zyx(q: np.ndarray) -> tuple[float, float, float]:
    """쿼터니언 -> (roll, pitch, yaw) [rad], Z-Y-X 순서 (DYNAMICS.md §4.2)."""
    R = quat_to_rot(q)
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


# ---------------------------------------------------------------- 플랜트

# 상태 벡터 배치 (총 17)
IDX_POS = slice(0, 3)     # 월드 위치
IDX_VEL = slice(3, 6)     # 월드 속도
IDX_QUAT = slice(6, 10)   # 자세 쿼터니언 [w,x,y,z]
IDX_OMEGA = slice(10, 13)  # 바디 각속도
IDX_N = slice(13, 17)     # 프로펠러 회전속도 [rev/s], 크기(부호 없음)

STATE_DIM = 17


class Plant:
    """DYNAMICS.md의 식을 그대로 구현한 6-DOF 강체 + 액추에이터."""

    def __init__(self, params: PlantParams | None = None):
        self.p = params or PlantParams()
        self.inv_inertia = np.linalg.inv(self.p.inertia)

    # -------------------------------------------------- 요소 모델

    def thrust(self, n_rps: np.ndarray) -> np.ndarray:
        """§5.1  T = Ct ρ n² D⁴  (모터별)."""
        p = self.p
        return p.ct * p.rho * n_rps**2 * p.diameter**4

    def drag_torque(self, n_rps: np.ndarray) -> np.ndarray:
        """§5.1  Q = Cq ρ n² D⁵  (프로펠러 공력 반토크, 크기)."""
        p = self.p
        return p.cq * p.rho * n_rps**2 * p.diameter**5

    def aero_force(self, vel_world: np.ndarray, wind_world: np.ndarray) -> np.ndarray:
        """§6.1  f = -sign(v) (ρ/2) A Cd v²  (월드 축별)."""
        p = self.p
        v = vel_world - wind_world
        return -np.sign(v) * 0.5 * p.rho * p.area_lin * p.cd_lin * v**2

    def aero_torque(self, omega_body: np.ndarray) -> np.ndarray:
        """§6.1의 회전 채널."""
        p = self.p
        w = omega_body
        return -np.sign(w) * 0.5 * p.rho * p.area_rot * p.cd_rot * w**2

    def body_torque(self, n_rps: np.ndarray) -> np.ndarray:
        """프로펠러가 만드는 바디 토크 [roll, pitch, yaw]  (§10.2).

        ⚠ 모멘트 암 가중치는 미확정 (DYNAMICS.md §11 #5, §11.0).
          현재는 네 로터가 균등 기여한다고 보고 `arm` 스칼라를 쓴다.
          그런데 CAD 암 방향을 환산하면 로터가 90도 간격이되 축에서 -11.7도
          회전한 "+" 배치라, 기하대로면 roll은 M1/M4, pitch는 M2/M3가 주도한다.
          부호는 두 해석이 같으므로 안전하나, 골든 트레이스 정량 대조에서
          가중치가 문제되면 `LEVER_GEOM`으로 교체할 것.
        """
        p = self.p
        T = self.thrust(n_rps)
        Q = self.drag_torque(n_rps)
        if p.use_geometric_lever:
            tau_roll = float(LEVER_GEOM[0] @ T)
            tau_pitch = float(LEVER_GEOM[1] @ T)
        else:
            tau_roll = LEVER_EFF * float(MIXER[0] @ T)
            tau_pitch = LEVER_EFF * float(MIXER[1] @ T)
        tau_yaw = float((-SPIN_DIR) @ Q)   # 반작용은 회전 방향의 반대
        return np.array([tau_roll, tau_pitch, tau_yaw])

    # -------------------------------------------------- 미분

    def derivative(
        self,
        x: np.ndarray,
        motor_torque: np.ndarray,
        wind_world: np.ndarray | None = None,
    ) -> np.ndarray:
        p = self.p
        if wind_world is None:
            wind_world = np.zeros(3)

        vel = x[IDX_VEL]
        q = quat_normalize(x[IDX_QUAT])
        omega = x[IDX_OMEGA]
        n = np.maximum(x[IDX_N], 0.0)

        R = quat_to_rot(q)

        # --- 병진 (§3.1)
        total_thrust = float(np.sum(self.thrust(n)))
        f_thrust_world = R @ np.array([0.0, 0.0, total_thrust])
        f_gravity = np.array([0.0, 0.0, -p.mass * p.g])
        f_drag = self.aero_force(vel, wind_world)
        acc = (f_thrust_world + f_gravity + f_drag) / p.mass

        # --- 회전 (§3.2)  I ω̇ + ω×(Iω) = τ
        tau = self.body_torque(n) + self.aero_torque(omega)
        gyro = np.cross(omega, p.inertia @ omega)
        omega_dot = self.inv_inertia @ (tau - gyro)

        # --- 자세 (§4)
        q_dot = quat_derivative(q, omega)

        # --- 액추에이터 (§10.3)  J ṅ = τ - Q - b n   (rad/s 기준으로 계산 후 rev/s 환산)
        omega_rot = n * 2.0 * math.pi                      # [rad/s]
        tau_cmd = np.clip(motor_torque, 0.0, p.max_torque)
        with np.errstate(divide="ignore", invalid="ignore"):
            power_limit = np.where(omega_rot > 1.0, p.max_power / np.maximum(omega_rot, 1e-9), np.inf)
        tau_cmd = np.minimum(tau_cmd, power_limit)
        n_dot_rad = (tau_cmd - self.drag_torque(n) - p.rotor_damping * omega_rot) / p.rotor_inertia
        n_dot = n_dot_rad / (2.0 * math.pi)

        dx = np.zeros(STATE_DIM)
        dx[IDX_POS] = vel
        dx[IDX_VEL] = acc
        dx[IDX_QUAT] = q_dot
        dx[IDX_OMEGA] = omega_dot
        dx[IDX_N] = n_dot
        return dx

    # -------------------------------------------------- 적분

    def step_rk4(self, x: np.ndarray, motor_torque: np.ndarray, dt: float,
                 wind_world: np.ndarray | None = None) -> np.ndarray:
        k1 = self.derivative(x, motor_torque, wind_world)
        k2 = self.derivative(x + 0.5 * dt * k1, motor_torque, wind_world)
        k3 = self.derivative(x + 0.5 * dt * k2, motor_torque, wind_world)
        k4 = self.derivative(x + dt * k3, motor_torque, wind_world)
        x_next = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        x_next[IDX_QUAT] = quat_normalize(x_next[IDX_QUAT])
        x_next[IDX_N] = np.maximum(x_next[IDX_N], 0.0)
        return x_next

    def simulate(self, x0: np.ndarray, torque_fn, t_end: float, dt: float = 1e-3,
                 wind_fn=None):
        """torque_fn(t, x) -> 모터 토크 4개. 결과는 (t, X) 배열."""
        steps = int(round(t_end / dt))
        ts = np.zeros(steps + 1)
        xs = np.zeros((steps + 1, STATE_DIM))
        x = x0.copy()
        xs[0] = x
        for k in range(steps):
            t = k * dt
            wind = wind_fn(t) if wind_fn else None
            x = self.step_rk4(x, np.asarray(torque_fn(t, x), dtype=float), dt, wind)
            ts[k + 1] = t + dt
            xs[k + 1] = x
        return ts, xs

    # ------------------------------------------------ 프로펠러 속도 규정 모드
    #
    # 골든 트레이스 대조용. 모터 동역학을 건너뛰고 n(t)를 직접 규정하면
    # **강체 + 공력 층만** 분리해서 검증할 수 있다.
    # (모터 층은 스핀업 시정수로 따로 검증)

    def derivative_prescribed_n(
        self,
        x: np.ndarray,
        n_rps: np.ndarray,
        wind_world: np.ndarray | None = None,
    ) -> np.ndarray:
        p = self.p
        if wind_world is None:
            wind_world = np.zeros(3)

        vel = x[IDX_VEL]
        q = quat_normalize(x[IDX_QUAT])
        omega = x[IDX_OMEGA]
        n = np.maximum(np.asarray(n_rps, dtype=float), 0.0)

        R = quat_to_rot(q)
        f_thrust_world = R @ np.array([0.0, 0.0, float(np.sum(self.thrust(n)))])
        f_gravity = np.array([0.0, 0.0, -p.mass * p.g])
        acc = (f_thrust_world + f_gravity + self.aero_force(vel, wind_world)) / p.mass

        tau = self.body_torque(n) + self.aero_torque(omega)
        omega_dot = self.inv_inertia @ (tau - np.cross(omega, p.inertia @ omega))

        dx = np.zeros(STATE_DIM)
        dx[IDX_POS] = vel
        dx[IDX_VEL] = acc
        dx[IDX_QUAT] = quat_derivative(q, omega)
        dx[IDX_OMEGA] = omega_dot
        dx[IDX_N] = 0.0          # 규정값이므로 적분하지 않음
        return dx

    def simulate_prescribed_n(self, x0: np.ndarray, n_fn, t_end: float,
                              dt: float = 1e-3, wind_fn=None):
        """n_fn(t) -> 프로펠러 회전속도 4개 [rev/s]. 모터 동역학 없이 강체만 적분."""
        steps = int(round(t_end / dt))
        ts = np.zeros(steps + 1)
        xs = np.zeros((steps + 1, STATE_DIM))
        x = x0.copy()
        x[IDX_N] = np.asarray(n_fn(0.0), dtype=float)
        xs[0] = x

        def rk4(xc: np.ndarray, t: float) -> np.ndarray:
            wind = wind_fn(t) if wind_fn else None
            nh = np.asarray(n_fn(t + 0.5 * dt), dtype=float)
            k1 = self.derivative_prescribed_n(xc, n_fn(t), wind)
            k2 = self.derivative_prescribed_n(xc + 0.5 * dt * k1, nh, wind)
            k3 = self.derivative_prescribed_n(xc + 0.5 * dt * k2, nh, wind)
            k4 = self.derivative_prescribed_n(xc + dt * k3, n_fn(t + dt), wind)
            xn = xc + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
            xn[IDX_QUAT] = quat_normalize(xn[IDX_QUAT])
            return xn

        for k in range(steps):
            t = k * dt
            x = rk4(x, t)
            x[IDX_N] = np.asarray(n_fn(t + dt), dtype=float)
            ts[k + 1] = t + dt
            xs[k + 1] = x
        return ts, xs

    def replay_csv(self, csv_path: str, dt: float = 1e-3, pos0=(0.0, 0.0, 0.0)):
        """Simscape가 기록한 n(t) CSV를 그대로 재생한다.

        CSV 형식: time, n1, n2, n3, n4   (n은 rev/s)
        골든 트레이스 대조의 파이썬 쪽 진입점.
        """
        data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
        t_col, n_cols = data[:, 0], data[:, 1:5]

        def n_fn(t: float) -> np.ndarray:
            return np.array([np.interp(t, t_col, n_cols[:, j]) for j in range(4)])

        x0 = initial_state(pos=pos0, n_rps=0.0)
        x0[IDX_N] = n_fn(0.0)
        return self.simulate_prescribed_n(x0, n_fn, t_end=float(t_col[-1]), dt=dt)


# ---------------------------------------------------------------- 유틸


def initial_state(pos=(0.0, 0.0, 0.0), n_rps: float = 0.0) -> np.ndarray:
    x = np.zeros(STATE_DIM)
    x[IDX_POS] = np.asarray(pos, dtype=float)
    x[IDX_QUAT] = np.array([1.0, 0.0, 0.0, 0.0])
    x[IDX_N] = n_rps
    return x


def hover_torque(plant: Plant) -> float:
    """호버 평형을 유지하는 모터 토크 (= 프로펠러 반토크와 균형)."""
    n = plant.p.hover_speed_rps()
    return float(plant.drag_torque(np.array([n]))[0] + plant.p.rotor_damping * n * 2 * math.pi)


# ---------------------------------------------------------------- 자체 검증


def _check_hover() -> bool:
    plant = Plant()
    n_h = plant.p.hover_speed_rps()
    tau_h = hover_torque(plant)
    x0 = initial_state(pos=(0, 0, 1.0), n_rps=n_h)
    ts, xs = plant.simulate(x0, lambda t, x: [tau_h] * 4, t_end=5.0, dt=1e-3)

    dz = xs[-1, IDX_POS][2] - 1.0
    att = np.rad2deg(np.abs(quat_to_euler_zyx(xs[-1, IDX_QUAT])))
    n_end = xs[-1, IDX_N]
    ok = abs(dz) < 0.02 and att.max() < 0.5

    print("[호버 평형]")
    print(f"  평형 회전속도 : {n_h:8.3f} rev/s = {n_h*60:7.1f} rpm = {n_h*2*math.pi:7.1f} rad/s")
    print(f"  모터당 추력   : {plant.thrust(np.array([n_h]))[0]:8.4f} N"
          f"   (필요 {plant.p.mass*plant.p.g/4:.4f} N)")
    print(f"  평형 토크     : {tau_h:8.5f} N·m")
    print(f"  5초 후 고도차 : {dz*100:+8.3f} cm")
    print(f"  5초 후 자세   : roll {att[0]:.4f}  pitch {att[1]:.4f}  yaw {att[2]:.4f} deg")
    print(f"  회전속도 유지 : {n_end.min():.3f} ~ {n_end.max():.3f} rev/s")
    print(f"  => {'통과' if ok else '실패'}")
    return ok


def _check_freefall() -> bool:
    plant = Plant()
    x0 = initial_state(pos=(0, 0, 10.0), n_rps=0.0)
    ts, xs = plant.simulate(x0, lambda t, x: [0.0] * 4, t_end=1.0, dt=1e-3)

    z = xs[-1, IDX_POS][2]
    z_ideal = 10.0 - 0.5 * plant.p.g * 1.0**2       # 항력 없을 때
    vz = xs[-1, IDX_VEL][2]
    # 항력이 있으므로 이상적 자유낙하보다 덜 떨어져야 한다
    ok = z_ideal < z < 10.0

    print("\n[자유낙하 1초]")
    print(f"  고도          : 10.000 -> {z:.4f} m   (항력 무시 시 {z_ideal:.4f} m)")
    print(f"  낙하 속도     : {vz:+.4f} m/s")
    print(f"  항력 기여     : {(z - z_ideal)*100:+.2f} cm (양수여야 정상)")
    print(f"  => {'통과' if ok else '실패'}")
    return ok


def _check_spindown() -> bool:
    """토크를 끊으면 프로펠러가 공력 반토크로 감속하는지 (시정수 확인)."""
    plant = Plant()
    n_h = plant.p.hover_speed_rps()
    x0 = initial_state(pos=(0, 0, 100.0), n_rps=n_h)
    ts, xs = plant.simulate(x0, lambda t, x: [0.0] * 4, t_end=1.0, dt=1e-3)
    n = xs[:, IDX_N][:, 0]
    target = n_h * math.exp(-1.0)
    idx = int(np.argmin(np.abs(n - target)))
    ok = 0.0 < ts[idx] < 0.5

    print("\n[스핀다운]")
    print(f"  {n_h:.2f} -> {n[-1]:.2f} rev/s (1초)")
    print(f"  1/e 도달 시각 : {ts[idx]:.4f} s")
    print(f"  => {'통과' if ok else '실패'}")
    return ok


def main() -> int:
    print("=" * 62)
    print("독립 6-DOF 플랜트 적분기 자체 검증")
    print("=" * 62)
    results = [_check_hover(), _check_freefall(), _check_spindown()]
    print("\n" + "=" * 62)
    print(f"결과: {sum(results)}/{len(results)} 통과")
    print("=" * 62)
    print("\n다음 단계: 같은 초기조건·모터 입력을 Simscape에 주고 궤적 대조")
    print("           (DYNAMICS.md §9 골든 트레이스)")
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
