# vision — 창문 검출·색 판정·§5 출력 모듈

> 담당: 류길남. 기준 문서: `window_detection_spec_v0.2.md`(리포 루트), `model_decisions.md`(이 폴더).
> 데이터셋 도착 전에 완성 가능한 코드 골격 — 모델 학습이 끝나면 검출부만 꽂으면 된다.

## 구성

| 파일 | 역할 |
|---|---|
| `color_order.yaml` | 색↔순서 매핑 + HSV 판정 구간 config (spec §3.1의 실행 기준) |
| `window_pose.yaml` | YOLO-pose 학습 데이터 정의 (spec §4.3) |
| `color_judge.py` | HSV 색 판정: corner 테두리 밴드 샘플링 → (color, order_index, color_conf) |
| `vision_msg.py` | §5 메시지 빌더 — 모든 출력(모델·GT)이 여길 거쳐 규격 준수 보장 |
| `gt_stream.py` | GT 라벨(§4.3 txt) → §5 메시지 어댑터 (spec §4.4, 모델 학습 전 파이프라인 검증용) |
| `model_decisions.md` | 모델 구조 확정 기록 (7건) |

데이터 흐름:

```
[학습 전]  시뮬 GT 라벨 ──ᐳ gt_stream ──┐
[학습 후]  YOLO-pose 검출 ─ᐳ color_judge ─┴─ᐳ vision_msg ──ᐳ §5 JSON ──ᐳ VIO(태민)
```

## 테스트

```
python -m pip install -r requirements.txt
python -m pytest tests/ -q        # 이 폴더(vision/)에서 실행
```

합성 이미지로 HSV 판정(테두리 밴드가 개구부 내부 배경에 오염되지 않는지 포함),
§5 메시지 규격, GT 라벨 역정규화(→720p)를 검증한다.

## 남은 일 (의존성 대기)

- 데이터셋 수령(윤호) → `window_pose.yaml`의 `path:` 기입 → 학습 (`model_decisions.md` 기준 명령)
- 시뮬 렌더 색 확인(윤호) → `color_order.yaml` HSV 구간 미세조정 (spec §7)
- 학습 후: 검출 결과를 `color_judge` + `vision_msg`로 잇는 추론 래퍼 작성 (det_conf = 박스 conf)
