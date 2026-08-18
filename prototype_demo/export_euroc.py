"""EuRoC MAV (MH) 포맷 덤프 — 태민 VIO 가 바로 먹는 형태로.

    python window_flight.py --observe --export-euroc /home/yoonho/euroc_out

왜 EuRoC 포맷인가
-----------------
ETH 의 EuRoC MAV 가 VIO 벤치마크 표준이라 OpenVINS·VINS-Mono·ORB-SLAM3 가 전부
이 디렉터리 구조를 그대로 읽는다. 경로만 바꿔 물리면 된다.

    mav0/
      cam0/data/<나노초>.png  +  data.csv
      imu0/data.csv                     timestamp, wx,wy,wz, ax,ay,az
      state_groundtruth_estimate0/data.csv
      cam0/sensor.yaml, imu0/sensor.yaml, body.yaml

핵심 규약 3가지 (여기서 지키는 것)
----------------------------------
1. **세 스트림이 같은 나노초 시계를 쓴다.** VIO 는 그 시간차로 상태를 추정하므로
   시계가 어긋나면 아무리 데이터가 정확해도 발산한다.
2. **가속도계는 specific force 를 잰다** — 중력을 포함한다. 자유낙하 중이면 0 이
   나오고, 정지 호버 중이면 +9.81 m/s^2 (기체 위쪽) 이 나온다. 순수 운동가속도를
   내보내면 VIO 가 중력을 두 번 빼서 즉시 발산한다.
3. **레버암을 반영한다.** IMU 는 카메라에서 68.9 mm 떨어져 있어서 (T_IC), 기체가
   회전하면 두 지점의 가속도가 다르다:  a_imu = a_body + w x (w x r) + alpha x r.
   각속도 1 rad/s 만 돼도 원심 성분이 0.069 m/s^2 라 무시 못 한다.

노이즈는 EuRoC ADIS16448 스펙을 따른다 (imu0/sensor.yaml 의 값). 화이트 노이즈 +
랜덤워크 바이어스. --no-noise 로 끌 수 있다.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List

import numpy as np

# EuRoC MH 의 imu0/sensor.yaml 값 (ADIS16448)
GYR_NOISE = 1.6968e-04      # rad/s/sqrt(Hz)
GYR_WALK = 1.9393e-05       # rad/s^2/sqrt(Hz)
ACC_NOISE = 2.0000e-03      # m/s^2/sqrt(Hz)
ACC_WALK = 3.0000e-03       # m/s^3/sqrt(Hz)
G_WORLD = np.array([0.0, 0.0, -9.81])


@dataclass
class EurocRecorder:
    """비행 루프에서 매 스텝 호출되어 IMU·GT 를 쌓고, 끝나면 EuRoC 로 쓴다."""
    T_IC: np.ndarray                       # cam -> imu (태민 파일 값)
    intr: dict
    dt: float = 0.001
    imu_hz: float = 200.0
    cam_hz: float = 20.0
    noise: bool = True
    seed: int = 0

    t_ns: int = 0
    imu: List[tuple] = field(default_factory=list)      # (t, wx..az)
    gt: List[tuple] = field(default_factory=list)       # (t, p, q, v)
    cam: List[tuple] = field(default_factory=list)      # (t, image)
    _prev_v: np.ndarray = None
    _prev_w: np.ndarray = None
    _bg: np.ndarray = None
    _ba: np.ndarray = None
    _rng: np.random.Generator = None
    _k: int = 0

    def __post_init__(self):
        self._rng = np.random.default_rng(self.seed)
        self._bg = np.zeros(3)
        self._ba = np.zeros(3)
        self.imu_every = max(1, int(round(1.0 / (self.imu_hz * self.dt))))
        self.cam_every = max(1, int(round(1.0 / (self.cam_hz * self.dt))))
        # 프레임 규약 (overrides/frames.py 참고, 태민 요청 반영):
        #   IMU 프레임 = GT body 프레임 = PyBullet 기체 프레임 (x 전방/y 좌/z 상)
        #   T_IC = IMU 프레임에서 본 **카메라** pose  (회전 R_VC, 병진 = 장착 오프셋)
        # 예전 코드는 body 를 카메라 프레임으로 보고 R_WI = R_WB·R_IC 를 냈다.
        # 그래서 t=0 에서 GT 자세가 R_IC 로 나왔고 (태민이 실측으로 잡아낸 그것),
        # 복원 경로(R_WI = R_WC·R_ICᵀ)와 서로 다른 프레임이 됐다. 이제 둘 다 R_WB 다.
        R_IC, p_IC = self.T_IC[:3, :3], self.T_IC[:3, 3]
        if np.linalg.norm(p_IC) > 1e-9:
            raise ValueError(
                "T_IC 병진이 0 이 아니다. 우리 시뮬은 카메라를 기체 원점에서 "
                "렌더하므로 카메라 오프셋을 넣으려면 렌더 위치도 같이 옮겨야 한다.")
        self.r_body_imu = np.zeros(3)           # IMU 는 기체 원점에 있다
        self.R_BI = np.eye(3)                   # IMU 프레임 = 기체 프레임

    # ------------------------------------------------------------------ #
    def step(self, p, body, client, image=None):
        """제어 루프 매 스텝 호출. image 는 이 스텝에 찍었으면 넘긴다."""
        t = self.t_ns
        self.t_ns += int(round(self.dt * 1e9))
        self._k += 1

        pos, quat = p.getBasePositionAndOrientation(body, physicsClientId=client)
        vel, omg = p.getBaseVelocity(body, physicsClientId=client)
        pos = np.asarray(pos, float); vel = np.asarray(vel, float)
        omg = np.asarray(omg, float)                       # world 각속도
        R_WB = np.array(p.getMatrixFromQuaternion(quat), float).reshape(3, 3)

        if self._k % self.imu_every == 0:
            self.imu.append((t, *self._imu_sample(R_WB, vel, omg)))
        if image is not None:
            self.cam.append((t, image))
        if self._k % self.imu_every == 0:
            # GT 는 IMU 프레임 기준으로 낸다 (EuRoC 의 state_groundtruth 규약)
            p_WI = pos + R_WB @ self.r_body_imu
            R_WI = R_WB @ self.R_BI
            v_WI = vel + np.cross(omg, R_WB @ self.r_body_imu)
            self.gt.append((t, p_WI, _rot_to_wxyz(R_WI), v_WI))

    def _imu_sample(self, R_WB, vel, omg):
        """(wx,wy,wz, ax,ay,az) — IMU 좌표계, specific force, 노이즈 포함."""
        dt_s = self.imu_every * self.dt
        a_lin = ((vel - self._prev_v) / dt_s) if self._prev_v is not None \
            else np.zeros(3)
        alpha = ((omg - self._prev_w) / dt_s) if self._prev_w is not None \
            else np.zeros(3)
        self._prev_v, self._prev_w = vel.copy(), omg.copy()

        # 레버암: 기체 원점 -> IMU 위치. world 좌표에서 더한다.
        r_w = R_WB @ self.r_body_imu
        a_imu_w = a_lin + np.cross(omg, np.cross(omg, r_w)) + np.cross(alpha, r_w)

        # specific force = (운동가속도 - 중력) 을 센서 좌표로. 정지 호버면 +9.81 z.
        R_WI = R_WB @ self.R_BI
        acc = R_WI.T @ (a_imu_w - G_WORLD)
        gyr = R_WI.T @ omg

        if self.noise:
            sn_g = GYR_NOISE / np.sqrt(dt_s)
            sn_a = ACC_NOISE / np.sqrt(dt_s)
            self._bg += self._rng.normal(0, GYR_WALK * np.sqrt(dt_s), 3)
            self._ba += self._rng.normal(0, ACC_WALK * np.sqrt(dt_s), 3)
            gyr = gyr + self._bg + self._rng.normal(0, sn_g, 3)
            acc = acc + self._ba + self._rng.normal(0, sn_a, 3)
        return (*gyr, *acc)

    # ------------------------------------------------------------------ #
    def write(self, root: str) -> dict:
        from PIL import Image
        mav = os.path.join(root, "mav0")
        d_cam = os.path.join(mav, "cam0", "data")
        d_imu = os.path.join(mav, "imu0")
        d_gt = os.path.join(mav, "state_groundtruth_estimate0")
        for d in (d_cam, d_imu, d_gt):
            os.makedirs(d, exist_ok=True)

        with open(os.path.join(mav, "cam0", "data.csv"), "w") as f:
            f.write("#timestamp [ns],filename\n")
            for t, img in self.cam:
                fn = f"{t}.png"
                Image.fromarray(img).save(os.path.join(d_cam, fn))
                f.write(f"{t},{fn}\n")

        with open(os.path.join(d_imu, "data.csv"), "w") as f:
            f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
                    "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
                    "a_RS_S_z [m s^-2]\n")
            for row in self.imu:
                f.write("%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n" % row)

        with open(os.path.join(d_gt, "data.csv"), "w") as f:
            f.write("#timestamp,p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],"
                    "q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z [],"
                    "v_RS_R_x [m s^-1],v_RS_R_y [m s^-1],v_RS_R_z [m s^-1]\n")
            for t, pp, qq, vv in self.gt:
                f.write("%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n"
                        % (t, *pp, *qq, *vv))

        self._write_yaml(mav)
        return {"cam": len(self.cam), "imu": len(self.imu), "gt": len(self.gt),
                "root": mav,
                "dur_s": (self.imu[-1][0] - self.imu[0][0]) / 1e9 if self.imu else 0}

    def _write_yaml(self, mav):
        i = self.intr
        # EuRoC 규약: T_BS = "body 프레임에서 본 센서 pose".
        #   body = IMU = 기체 프레임 (x 전방/y 좌/z 상)  ->  imu0 의 T_BS 는 항등
        #   cam0 의 T_BS = T_IC (표준 전방 카메라 회전 R_VC, 병진 = 장착 오프셋)
        T = self.T_IC
        cam_yaml = f"""%YAML:1.0
sensor_type: camera
comment: T_BS = body(=IMU) 프레임에서 본 카메라 pose. 회전은 표준 전방 카메라
  R_VC = [[0,0,1],[-1,0,0],[0,-1,0]] (OpenCV 광학: z 전방/x 우/y 하). 카메라는
  기체 원점에 body +x 를 보게 장착.
T_BS:
  cols: 4
  rows: 4
  data: {np.round(T.flatten(), 9).tolist()}
rate_hz: {self.cam_hz:.0f}
resolution: [{int(i['width'])}, {int(i['height'])}]
camera_model: pinhole
intrinsics: [{i['fx']:.1f}, {i['fy']:.1f}, {i['cx']:.1f}, {i['cy']:.1f}]
distortion_model: none
distortion_coefficients: [0.0, 0.0, 0.0, 0.0]
"""
        imu_yaml = f"""%YAML:1.0
sensor_type: imu
comment: body 프레임 = IMU 프레임이므로 T_BS 는 항등이다. GT(state_groundtruth
  _estimate0)도 이 IMU 프레임 기준 위치/자세다 (카메라 위치가 아니다).
T_BS:
  cols: 4
  rows: 4
  data: {np.round(np.eye(4).flatten(), 9).tolist()}
rate_hz: {self.imu_hz:.0f}
gyroscope_noise_density: {GYR_NOISE}
gyroscope_random_walk: {GYR_WALK}
accelerometer_noise_density: {ACC_NOISE}
accelerometer_random_walk: {ACC_WALK}
noise_enabled: {str(self.noise).lower()}
"""
        with open(os.path.join(mav, "cam0", "sensor.yaml"), "w") as f:
            f.write(cam_yaml)
        with open(os.path.join(mav, "imu0", "sensor.yaml"), "w") as f:
            f.write(imu_yaml)
        with open(os.path.join(mav, "body.yaml"), "w") as f:
            f.write("%YAML:1.0\n"
                    "comment: body frame = IMU frame = 기체 원점 "
                    "(x 전방 / y 좌 / z 상). GT 위치는 **IMU 위치**다.\n")


def _rot_to_wxyz(R):
    """회전행렬 -> (w,x,y,z). EuRoC GT 는 wxyz 순서다 (xyzw 아님)."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return (0.25 * s, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s)
    i = int(np.argmax([R[0,0], R[1,1], R[2,2]]))
    if i == 0:
        s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        return ((R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s)
    if i == 1:
        s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        return ((R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s)
    s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
    return ((R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s)
