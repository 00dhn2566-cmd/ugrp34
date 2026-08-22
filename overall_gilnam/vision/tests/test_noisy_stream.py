"""noisy_stream 테스트 — 캘리브레이션 정합·주입 스키마·재현성."""
import numpy as np

from vision_msg import N_CORNERS
from noisy_stream import P_TAIL, DEFAULT_DROP, calibrate_mixture, make_noisy_records

MEAN_PX, P95_PX = 8.87, 36.6


def _sample_radial(sigma_core, sigma_tail, p_tail, n, seed=0):
    """혼합 모델에서 2D 반경 오차 표본 추출 (테스트 전용 몬테카를로)."""
    rng = np.random.default_rng(seed)
    sigmas = np.where(rng.random(n) < p_tail, sigma_tail, sigma_core)
    xy = rng.normal(0.0, 1.0, (n, 2)) * sigmas[:, None]
    return np.linalg.norm(xy, axis=1)


def test_calibration_matches_measured_stats():
    sc, st = calibrate_mixture(MEAN_PX, P95_PX)
    assert 0.0 < sc < st  # 코어보다 꼬리가 넓어야 함
    r = _sample_radial(sc, st, P_TAIL, 200_000)
    assert abs(r.mean() - MEAN_PX) / MEAN_PX < 0.03
    assert abs(np.percentile(r, 95) - P95_PX) / P95_PX < 0.05


SAMPLE_RECORDS = [
    {
        "vision": {
            "timestamp": 1_720_000_000_000_000_000 + i * 33_333_333,
            "frame_id": i,
            "windows": [
                {
                    "order_index": 0, "color": "red",
                    "corners": [[560.0, 298.0], [721.0, 296.0], [721.0, 423.0], [560.0, 421.0]],
                    "corner_vis": [1, 1, 1, 1], "center": [640.5, 359.5],
                    "det_conf": 1.0, "color_conf": 1.0,
                }
            ],
        },
        "pose": {"timestamp": 1_720_000_000_000_000_000 + i * 33_333_333, "frame": "world",
                 "position": [0.1 * i, 0.0, 1.5], "orientation": [0.0, 0.0, 0.0, 1.0]},
    }
    for i in range(200)
]


def test_schema_preserved_and_pose_untouched():
    out = make_noisy_records(SAMPLE_RECORDS, scale=1.0, seed=1234,
                             mean_px=8.87, p95_px=36.6, p_tail=P_TAIL, drop_prob=0.0)
    assert len(out) == len(SAMPLE_RECORDS)
    for rec, src in zip(out, SAMPLE_RECORDS):
        assert rec["pose"] == src["pose"]
        msg = rec["vision"]
        assert isinstance(msg["timestamp"], int) and msg["timestamp"] == src["vision"]["timestamp"]
        for w in msg["windows"]:
            assert len(w["corners"]) == N_CORNERS
            assert w["corner_vis"] == [1, 1, 1, 1]
            assert w["det_conf"] == 1.0  # 기하 외 필드 불변 (설계)


def test_seed_reproducible_and_scale_zero_identity():
    a = make_noisy_records(SAMPLE_RECORDS, 1.0, 1234, 8.87, 36.6, P_TAIL, DEFAULT_DROP)
    b = make_noisy_records(SAMPLE_RECORDS, 1.0, 1234, 8.87, 36.6, P_TAIL, DEFAULT_DROP)
    assert a == b
    zero = make_noisy_records(SAMPLE_RECORDS, 0.0, 1234, 8.87, 36.6, P_TAIL, 0.0)
    src_corners = SAMPLE_RECORDS[0]["vision"]["windows"][0]["corners"]
    assert zero[0]["vision"]["windows"][0]["corners"] == src_corners


def test_drop_probability_applied():
    dropped = make_noisy_records(SAMPLE_RECORDS, 1.0, 1234, 8.87, 36.6, P_TAIL, drop_prob=0.5)
    n_kept = sum(len(r["vision"]["windows"]) for r in dropped)
    assert 60 <= n_kept <= 140  # 이항(200, 0.5)의 넉넉한 구간
