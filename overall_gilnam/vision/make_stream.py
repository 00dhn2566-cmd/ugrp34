"""태민(VIO)용 샘플 스트림 생성 CLI.

    python make_stream.py --seed 42 --out sample_stream/

프레임 루프: synth_scene 투영 → §4.3 라벨 라인(메모리) → 기존 gt_stream 어댑터
(직접 조립 금지 — 어댑터 실전 검증이 목적의 일부) → §5 메시지 + GT pose 쌍을
jsonl 한 줄로 기록. 산출 3종: sample_stream.jsonl / scene_gt.json / README_stream.md.
"""

import argparse
import json
from pathlib import Path

from color_judge import load_color_config
from gt_stream import labels_to_message
from synth_scene import load_intrinsics, make_scene, make_trajectory, project, to_label_lines

VISION_DIR = Path(__file__).resolve().parent

# 기록 시 반올림 자릿수 (용량 관리 — 총 500KB 미만 목표). timestamp는 int 유지.
PX_DECIMALS = 2      # 픽셀 좌표·center
METER_DECIMALS = 4   # 위치·3D 좌표
QUAT_DECIMALS = 6    # quaternion

CONVENTIONS_MD = """\
## 좌표·투영 관례 (계약 — 이 스트림의 모든 수치가 따르는 규칙)

- world: Z-up, X-전방, 미터 단위.
- camera: OpenCV 관례 (+Z 광축, +X 우, +Y 하). u = fx·Xc/Zc + cx, v = fy·Yc/Zc + cy.
- pose = T_world_cam (world 상 카메라 자세): position t_wc[3], orientation quaternion (x,y,z,w).
  변환: X_cam = R_wc^T (X_world − t_wc), 투영행렬 P = K·[R_wc^T | −R_wc^T·t_wc].
- 이 스트림에서는 **body ≡ camera** 로 선언한다 (IMU-카메라 외부 캘리브레이션 항등 고정).
- look-at: z_cam = normalize(target − eye), x_cam = normalize(cross(z_cam, [0,0,1])),
  y_cam = cross(z_cam, x_cam), R_wc = [x_cam y_cam z_cam] (열벡터).
- 창문: 수직 평면(pitch 0), normal n은 접근측을 향함. 접근측에서 본(시선 = −n)
  corner 순서 좌상→우상→우하→좌하:
  viewer_right = normalize(cross(−n, [0,0,1])), up = [0,0,1]
  TL = c + (h/2)up − (w/2)viewer_right, TR = c + (h/2)up + (w/2)viewer_right,
  BR = c − (h/2)up + (w/2)viewer_right, BL = c − (h/2)up − (w/2)viewer_right.
"""


def _round_list(values, nd):
    return [round(float(v), nd) for v in values]


def _round_message(msg):
    """§5 메시지의 픽셀 값만 반올림 (구조·타입 불변, timestamp int 유지)."""
    for w in msg["windows"]:
        w["corners"] = [_round_list(pt, PX_DECIMALS) for pt in w["corners"]]
        w["center"] = _round_list(w["center"], PX_DECIMALS)
    return msg


def generate_stream(seed=42, out_dir="sample_stream"):
    """스트림 3종 파일 생성. 반환: 통계 dict (프레임 수, 경로)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    intr = load_intrinsics(VISION_DIR / "synth_intrinsics.yaml")
    config = load_color_config(VISION_DIR / "color_order.yaml")
    scene = make_scene(seed)
    traj = make_trajectory(scene)

    jsonl_path = out / "sample_stream.jsonl"
    with open(jsonl_path, "w", encoding="utf-8") as f:
        for frame_id, pose in enumerate(traj):
            lines = to_label_lines(project(scene, pose, intr["K"]))
            msg = _round_message(labels_to_message(lines, pose["timestamp"], frame_id, config))
            record = {
                "vision": msg,
                "pose": {
                    "timestamp": pose["timestamp"],
                    "frame": "world",
                    "position": _round_list(pose["position"], METER_DECIMALS),
                    "orientation": _round_list(pose["quat_xyzw"], QUAT_DECIMALS),
                },
            }
            f.write(json.dumps(record, ensure_ascii=False) + "\n")

    scene_gt = {
        "seed": seed,
        "intrinsics": {k: v for k, v in intr.items() if k != "K"},
        "conventions": "world Z-up X-forward (m) / camera OpenCV +Z optical / "
                       "pose T_world_cam quat xyzw / corners TL->TR->BR->BL from approach side / "
                       "body==camera — 전문은 README_stream.md",
        "windows": [
            {
                "order_index": w["order_index"],
                "color": w["color"],
                "center": _round_list(w["center"], METER_DECIMALS),
                "normal": _round_list(w["normal"], QUAT_DECIMALS),
                "size_wh": _round_list(w["size_wh"], METER_DECIMALS),
                "corners_3d": [_round_list(c, METER_DECIMALS) for c in w["corners_3d"]],
            }
            for w in scene["windows"]
        ],
    }
    gt_path = out / "scene_gt.json"
    gt_path.write_text(json.dumps(scene_gt, ensure_ascii=False, indent=2), encoding="utf-8")

    readme_path = out / "README_stream.md"
    readme_path.write_text(_readme_md(seed, len(traj)), encoding="utf-8")
    return {"frames": len(traj), "jsonl": jsonl_path, "scene_gt": gt_path, "readme": readme_path}


def _readme_md(seed, n_frames):
    return f"""# sample_stream — 태민(VIO)용 합성 §5 + GT pose 스트림

> 생성: `python make_stream.py --seed {seed} --out sample_stream/` (overall_gilnam/vision/에서)
> 프레임 {n_frames}개 @30Hz. 창문 3개(red→green→blue = order 0→1→2) 순서 통과 궤적.
> intrinsics는 `../synth_intrinsics.yaml`의 임시값 — 윤호가 spec §6 기입 시 교체 후 재생성.

{CONVENTIONS_MD}

## 파일 스키마

### sample_stream.jsonl — 한 줄 = 한 프레임

```json
{{"vision": <§5 메시지 (window_detection_spec_v0.2)>,
  "pose": {{"timestamp": <int ns>, "frame": "world",
           "position": [x,y,z], "orientation": [qx,qy,qz,qw]}}}}
```

- `vision.timestamp == pose.timestamp` (int ns, 30Hz 간격 33,333,333ns).
- **GT 스트림이므로 det_conf = color_conf = 1.0.**
- vis=0인 corner는 화면 밖 추정 좌표(0~1280/0~720 이탈 가능) — 정책 C와 동일 의미.
- 반올림: 픽셀 소수 2자리, 미터 4자리, quat 6자리 (timestamp는 int).

### scene_gt.json — 정답 대조용

시드, intrinsics 사본, 창문별 order_index/color/center/normal/size_wh/corners_3d(world, §4.3
corner 순서와 동일: TL→TR→BR→BL).

## 태민 착수 가이드 (삼각측량)

1. 같은 창문이 4 corner 모두 vis=1로 보이는, 시차 있는 두 프레임 A·B를 고른다
   (position 차이 ≥ 0.5m 권장 — sway 덕에 순수 전진 구간에도 횡 시차 있음).
2. 각 프레임의 pose로 P = K·[R_wc^T | −R_wc^T·t_wc] 구성 (quat→R 후).
3. `cv2.triangulatePoints(P_A, P_B, corners_A.T, corners_B.T)` → 동차좌표 나눗셈.
4. `scene_gt.json`의 해당 창문 `corners_3d`와 대조 — **목표 오차 mm 수준**
   (생성기 자기일관성은 `tests/test_synth_scene.py`의 왕복 테스트로 보증됨).

문의: 길남 (규격 변경 시 window_detection_spec 갱신 후 공유).
"""


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="합성 §5+pose 샘플 스트림 생성")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--out", type=str, default="sample_stream")
    args = parser.parse_args()
    stats = generate_stream(seed=args.seed, out_dir=args.out)
    print(f"frames={stats['frames']} → {stats['jsonl']}")
