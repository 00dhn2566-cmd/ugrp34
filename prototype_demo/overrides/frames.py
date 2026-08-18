"""§좌표 프레임 규약 — 태민 노드의 ``T_IC`` 를 **우리 쪽에서** 표준값으로 덮는다.

왜 덮는가
---------
그의 ``window_recon_node.py`` 상수 ``T_IC`` 의 회전은 이렇다::

    z_cam 열 = [0.004, 0.026, 0.9997]      # 카메라 광축이 IMU +z (위) 를 향함

OpenCV 광학 프레임(z=전방, x=우, y=하) 규약으로 읽으면 **"카메라가 위를 본다"** 가
된다. 그런데 우리 시뮬 카메라는 기체 +x (비행 방향) 를 본다. 시뮬 안에서는 양쪽 끝을
같은 값으로 맞춰 놨으니 자기모순은 없었지만, OpenVINS 처럼 규약을 곧이곧대로 믿는
외부 도구에 넘기면 어긋난다. (태민이 실제로 ``R_ICᵀ·R_VC`` 로 되돌려 쓰고 있었다.)

게다가 우리 코드 안에서도 두 경로가 갈라져 있었다::

    복원 경로   R_WI = R_WC · R_ICᵀ        (contract.camera_pose_to_imu_pose)
    EuRoC 경로  R_WI = R_WB · R_IC         (export_euroc)

R_IC 가 표준값이 아니라서 이 둘이 서로 다른 프레임을 냈다. t=0 에서 GT 자세가
정확히 R_IC 로 나온 게 그 증거다. 표준값으로 바꾸면 두 경로가 **같은 프레임**으로
수렴한다 (아래 참고).

무엇으로 덮는가
---------------
IMU 프레임 = GT body 프레임 = **x 전방 / y 좌 / z 상** (PyBullet 기체 프레임 그대로).
카메라는 그 +x 를 보게 장착. 그러면 T_imu_cam (IMU 프레임에서 본 카메라 pose) 의
회전은 표준 전방 카메라 값이다::

    R_VC = [[0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]]

    열 0 = x_cam(우)   -> [0,-1,0] = IMU -y = 기체 우측   ✓
    열 1 = y_cam(하)   -> [0,0,-1] = IMU -z = 아래        ✓
    열 2 = z_cam(전방) -> [1,0,0]  = IMU +x = 비행 방향   ✓

병진은 실제 장착 오프셋. 우리 시뮬은 카메라가 기체 원점에 있으므로 0 이다.

이러면 정합이 맞는가 (검산)
---------------------------
``pbs.look_at_pose`` 는 OpenCV 광학 R_WC 를 낸다. 수평 자세로 +x 를 볼 때
R_WC = R_VC 다. 그러므로

    복원 경로   R_WI = R_WC · R_VCᵀ = R_WB · R_VC · R_VCᵀ = R_WB   ✓
    EuRoC 경로  R_WI = R_WB                                          ✓

둘 다 기체 프레임으로 떨어진다. 갈라져 있던 게 붙는다.

주의
----
태민 노드의 상수 자체는 **안 고친다** (팀 파일 무수정 원칙). 대신 이 모듈의
:func:`install` 이 ``contract.T_imu_cam`` 을 바꿔치기해서, 우리 파이프라인의 모든
소비자(복원기 / EuRoC 내보내기 / 브리지 / 지표)가 같은 값을 보게 한다. 그의 노드를
**ROS 로 직접 돌릴 때는** 그 파일의 상수도 같이 바꿔야 한다. 우리 쪽만 바꾸면
시뮬 안에서는 맞고 그의 실행에서는 안 맞는다.
"""
from __future__ import annotations

import numpy as np

#: 표준 전방 카메라 회전 (IMU 프레임에서 본 카메라). OpenVINS/OpenCV 규약.
R_VC = np.array([[0.0, 0.0, 1.0],
                 [-1.0, 0.0, 0.0],
                 [0.0, -1.0, 0.0]])

#: 카메라 장착 오프셋 [m]. 우리 시뮬은 카메라가 기체 원점에 있으므로 0.
P_IC = np.zeros(3)


def T_imu_cam() -> np.ndarray:
    """IMU(=기체) 프레임에서 본 카메라 pose, 4x4."""
    T = np.eye(4)
    T[:3, :3] = R_VC
    T[:3, 3] = P_IC
    return T


_installed = False


def install(verbose: bool = True) -> np.ndarray:
    """``contract.T_imu_cam`` 을 표준값으로 바꿔친다. 프로세스 전체에 먹는다."""
    global _installed
    from module import contract
    T = T_imu_cam()
    contract.T_imu_cam = lambda path=None, _T=T: _T.copy()
    if verbose and not _installed:
        print("[frames] T_imu_cam 을 표준 전방 카메라 값으로 덮음 "
              "(R_VC, 병진 0). IMU 프레임 = 기체 프레임 (x 전방/y 좌/z 상)")
    _installed = True
    return T
