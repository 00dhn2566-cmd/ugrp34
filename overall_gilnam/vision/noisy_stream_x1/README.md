# noisy_stream_x1 — 실측 노이즈 주입 §5 + GT pose 스트림 (태민 재검증용)

> 원본: sample_stream/ (seed 42, 302프레임). 노이즈: 1차 학습 모델 실측
> (corner 반경 오차 평균 8.87px / p95 36.6px, 2026-08-02 본 판정)을 2성분
> 가우시안 혼합으로 정합, 미검출 3.9%는 창문 단위 드롭.
> 재생성: `python noisy_stream.py --stream sample_stream/sample_stream.jsonl --out noisy_stream_x1 --scales 1 --seed 1234`

- 형식·좌표 관례는 sample_stream/README_stream.md와 동일 (§5 + pose, scene_gt.json 사본 포함).
- det_conf/color_conf는 GT값(1.0) 유지 — 기하 노이즈만 주입.
- 요청: 7/4과 동일한 방법으로 3D 복원 → 오차 공유 (길남 재현 구현과 교차 검증,
  결과 비교표: overall_gilnam/docs/eval_target_derivation.md).
