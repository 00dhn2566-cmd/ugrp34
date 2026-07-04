"""합성 씬 생성기 — 가상 창문 3개 + 카메라 접근 궤적 (순수 로직, I/O 없음).

용도: 윤호의 Isaac Sim 데이터셋 도착 전에 태민(VIO)이 삼각측량 융합을
착수할 수 있도록, §4.3 라벨 라인(메모리 상)과 GT pose를 프레임 단위로 생성한다.
§5 메시지 변환은 여기서 하지 않는다 — make_stream.py가 기존 gt_stream 어댑터를
경유시킨다 (어댑터 실전 검증 겸용).

좌표·투영 관례 (README_stream.md에 명문화된 계약과 동일):
- world: Z-up, X-전방, 미터. camera: OpenCV (+Z 광축, +X 우, +Y 하).
- pose = T_world_cam: position t_wc, quaternion (x,y,z,w).
  X_cam = R_wc^T (X_world - t_wc),  u = fx*Xc/Zc + cx,  v = fy*Yc/Zc + cy.
- 창문은 수직 평면(v1: pitch 0 고정, 파라미터로는 지원), normal은 접근측을 향함.
  corner 순서(§4.3): 접근측에서 본(시선 = -n) 좌상→우상→우하→좌하.

배치 랜덤화는 spec §4.1 준수 (창문 1~5개 中 3개, yaw는 ±60° 범위 내 ±15° 사용).
spec 미규정 값 — 기본값: 전방(X) 간격 4~6m, 횡 ±1.5m, 높이 1.5±0.5m,
크기 w,h ∈ [0.8,1.2]m.
"""

import numpy as np
import yaml

from color_judge import load_color_config

IMG_W, IMG_H = 1280, 720  # §2 원본 해상도
UP = np.array([0.0, 0.0, 1.0])
MIN_DEPTH_M = 0.05  # corner Zc가 이보다 작으면 그 창문은 해당 프레임 제외
FRAME_INTERVAL_NS = 33_333_333  # 30 Hz
DEFAULT_T0_NS = 1_720_000_000_000_000_000


def load_intrinsics(path):
    """synth_intrinsics.yaml → dict (+ K 행렬 추가)."""
    with open(path, encoding="utf-8") as f:
        intr = yaml.safe_load(f)
    intr["K"] = np.array(
        [[intr["fx"], 0.0, intr["cx"]], [0.0, intr["fy"], intr["cy"]], [0.0, 0.0, 1.0]]
    )
    return intr


def _normalize(v):
    return v / np.linalg.norm(v)


def _window_corners(center, normal, w, h):
    """접근측(시선 = -n)에서 본 좌상→우상→우하→좌하 (§4.3 순서)."""
    viewer_right = _normalize(np.cross(-normal, UP))
    half_w, half_h = w / 2.0, h / 2.0
    return np.array(
        [
            center + half_h * UP - half_w * viewer_right,  # TL
            center + half_h * UP + half_w * viewer_right,  # TR
            center - half_h * UP + half_w * viewer_right,  # BR
            center - half_h * UP - half_w * viewer_right,  # BL
        ]
    )


def make_scene(seed, n_windows=3, pitch_deg=0.0):
    """창문 n개 씬 생성 (시드 고정 재현성). 색↔순서는 color_order.yaml 기준.

    v1은 pitch 0 고정 사용 — 0이 아닌 값을 주면 ValueError (수직 평면 가정 유지).
    """
    if pitch_deg != 0.0:
        raise ValueError("v1은 pitch 0만 지원 (수직 평면 가정)")
    from pathlib import Path

    config = load_color_config(Path(__file__).resolve().parent / "color_order.yaml")
    order_to_color = {c["order_index"]: name for name, c in config["colors"].items()}

    rng = np.random.default_rng(seed)
    windows, x = [], 0.0
    for i in range(n_windows):
        x += rng.uniform(4.0, 6.0)          # spec 미규정 — 기본값: 전방 간격 4~6m
        y = rng.uniform(-1.5, 1.5)          # spec 미규정 — 기본값: 횡 ±1.5m
        z = 1.5 + rng.uniform(-0.5, 0.5)    # spec 미규정 — 기본값: 높이 1.5±0.5m
        w = rng.uniform(0.8, 1.2)           # spec 미규정 — 기본값: 크기 0.8~1.2m
        h = rng.uniform(0.8, 1.2)
        yaw = rng.uniform(-15.0, 15.0)      # spec §4.1 ±60° 범위 내 (v1은 ±15° 사용)
        # 기준 normal = -X (접근측 = 시작점 쪽), yaw는 Z축 회전
        rad = np.radians(yaw)
        normal = np.array([-np.cos(rad), -np.sin(rad), 0.0])
        center = np.array([x, y, z])
        windows.append(
            {
                "order_index": i,
                "color": order_to_color[i],
                "center": center,
                "normal": normal,
                "size_wh": [w, h],
                "yaw_deg": yaw,
                "corners_3d": _window_corners(center, normal, w, h),
            }
        )
    return {"seed": seed, "windows": windows}


def _look_at_rotation(eye, target, prev_R=None):
    """R_wc = [x_cam y_cam z_cam] (열벡터). eye≈target이면 직전 자세 유지."""
    d = target - eye
    if np.linalg.norm(d) < 1e-6:
        if prev_R is None:
            raise ValueError("look-at 목표가 시점과 일치하고 직전 자세도 없음")
        return prev_R
    z_cam = _normalize(d)
    x_cam = _normalize(np.cross(z_cam, UP))
    y_cam = np.cross(z_cam, x_cam)
    return np.column_stack([x_cam, y_cam, z_cam])


def _rotation_to_quat_xyzw(R):
    """회전행렬 → quaternion (x,y,z,w). numpy 자체 구현 (새 의존성 금지)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2.0
        q = np.empty(3)
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        w = (R[k, j] - R[j, k]) / s
        x, y, z = q
    return _normalize(np.array([x, y, z, w]))


def make_trajectory(scene, hz=30, speed=1.5, sway_amp=0.3, start_back=4.0, start_side=0.4,
                    t0_ns=DEFAULT_T0_NS):
    """waypoint(시작점 + 창문 중심들) 직선 보간 + 횡 sway 궤적.

    sway offset = sway_amp*sin(pi*s)*perp — 양 끝 s=0,1에서 0이라 창문 중심은
    정확히 통과. 목적: 삼각측량 시차 확보.
    시선은 다음 미통과 창문 look-at, 마지막 통과 후 직전 방향 유지.
    """
    if hz != 30:
        raise ValueError("timestamp 간격이 30Hz 고정(33_333_333ns)이라 hz=30만 지원")
    w0 = scene["windows"][0]
    viewer_right0 = _normalize(np.cross(-w0["normal"], UP))
    start = w0["center"] + start_back * w0["normal"] + start_side * viewer_right0
    waypoints = [start] + [w["center"] for w in scene["windows"]]

    poses, k, prev_R = [], 0, None
    for seg in range(len(waypoints) - 1):
        a, b = waypoints[seg], waypoints[seg + 1]
        seg_vec = b - a
        seg_len = np.linalg.norm(seg_vec)
        seg_dir = seg_vec / seg_len
        perp = _normalize(np.cross(seg_dir, UP))
        n_steps = max(1, int(np.ceil(seg_len / speed * hz)))
        target = waypoints[seg + 1]  # 다음 미통과 창문 중심
        for i in range(n_steps):  # 끝점은 다음 구간의 s=0 (마지막 구간만 아래서 추가)
            s = i / n_steps
            pos = a + s * seg_vec + sway_amp * np.sin(np.pi * s) * perp
            R = _look_at_rotation(pos, target, prev_R)
            poses.append(
                {
                    "timestamp": t0_ns + k * FRAME_INTERVAL_NS,
                    "position": pos,
                    "R_wc": R,
                    "quat_xyzw": _rotation_to_quat_xyzw(R),
                }
            )
            prev_R, k = R, k + 1
    # 마지막 창문 중심 프레임: 통과할 창문이 없으므로 직전 방향 유지
    poses.append(
        {
            "timestamp": t0_ns + k * FRAME_INTERVAL_NS,
            "position": waypoints[-1],
            "R_wc": prev_R,
            "quat_xyzw": _rotation_to_quat_xyzw(prev_R),
        }
    )
    return poses


def project(scene, pose, K):
    """창문별 4 corner 투영 → [{order_index, corners_px(4x2), vis[4]}].

    어느 corner든 Zc <= 0.05m이면 그 창문은 해당 프레임에서 제외.
    화면 밖 corner는 투영 좌표 유지 + vis=0 (정책 C).
    """
    R_t = pose["R_wc"].T
    t = pose["position"]
    out = []
    for w in scene["windows"]:
        cam = (R_t @ (np.asarray(w["corners_3d"]) - t).T).T  # (4,3)
        if np.any(cam[:, 2] <= MIN_DEPTH_M):
            continue
        uv = (K @ cam.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        vis = [
            1 if (0 <= u < IMG_W and 0 <= v < IMG_H) else 0
            for u, v in uv
        ]
        out.append({"order_index": w["order_index"], "corners_px": uv, "vis": vis})
    return out


def to_label_lines(projections):
    """project() 결과 → §4.3 라벨 라인 (1280x720 정규화, 소수 6자리).

    화면 밖 corner는 0~1 이탈 좌표를 그대로 둔다 — 이 스트림은 태민(VIO)용이지
    학습용 라벨이 아니므로 클리핑하지 않는다.
    """
    lines = []
    for p in projections:
        uv = np.asarray(p["corners_px"], dtype=np.float64)
        norm = uv / np.array([IMG_W, IMG_H])
        cx, cy = (norm.min(axis=0) + norm.max(axis=0)) / 2.0
        bw, bh = norm.max(axis=0) - norm.min(axis=0)
        fields = [str(p["order_index"])] + [f"{v:.6f}" for v in (cx, cy, bw, bh)]
        for (u, v), vis in zip(norm, p["vis"]):
            fields += [f"{u:.6f}", f"{v:.6f}", str(vis)]
        lines.append(" ".join(fields))
    return lines
