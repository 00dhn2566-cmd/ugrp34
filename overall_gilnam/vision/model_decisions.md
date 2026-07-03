# 창문 검출 모델 — 구조 확정 기록

> 대상: 비전 파트(류길남)의 4-corner 검출 모델. 기준 문서: `window_detection_spec_v0.2`(§2·§3·§4.3·§5), `To_do_checklist_gilnam` §1.
> 성격: §4.3(라벨)·§5(출력) 규격은 **불변** — 그 사이의 구현 선택을 기록한다. 결정 변경 시 이 문서를 갱신하고 팀 공유.
> 작성: 2026-07-03

---

## 확정 사항 (7건)

| # | 항목 | 결정 | 근거 요약 |
|---|---|---|---|
| 1 | 베이스 모델 | Ultralytics `yolo11s-pose` (COCO-pose 사전학습 → 파인튜닝) | 문서·안정성 우선. `ultralytics` 버전은 requirements에 핀 고정 (재현성) |
| 2 | 모델 크기 | **s**로 시작 (n은 빠른 실험용) | corner 픽셀 오차가 곧 태민 삼각측량 오차. s@640은 실시간(30Hz) 여유 충분 |
| 3 | keypoint 구성 | `kpt_shape: [4,3]` + **`flip_idx: [1,0,3,2]`** | §4.3 라벨과 일치. flip_idx 누락 시 좌우반전 증강이 corner 순서 의미를 오염 (최다 빈출 치명 실수) |
| 4 | 클래스 전략 | 라벨은 §4.3 그대로(class=order_index) 유지 + **학습 `single_cls=True`** → 모델은 "창문" 1클래스, 색·순서는 HSV 후처리 전담 | §3 확장성 원칙(색 추가 = 테이블 갱신, 재학습 불필요) / 3,000장이 클래스 분산 없이 기하 학습에 집중 / det_conf(기하)·color_conf(색) 의미 분리 / 모델↔HSV 불일치 중재 규칙 불필요. 라벨·규격 무변경, 플래그 제거만으로 3클래스 회귀 가능 |
| 5 | 증강 정책 | hue/saturation 지터 허용, `flipud` 기본 off 유지 | 4번 덕분에 가능 — 색 판정은 증강이 닿지 않는 원본 프레임에서 수행. (3클래스 회귀 시에는 `hsv_h=0` 필수 — 3클래스의 숨은 비용) |
| 6 | 입력 해상도 | `imgsz=640` 확정 (spec §7 잔여 항목 해소). 추론은 **원본 1280×720 프레임을 그대로 입력** | Ultralytics가 letterbox 역변환 후 원본 좌표로 반환 → §2 "리사이즈 좌표 유출 금지"가 구조적으로 보장. 원거리 corner 오차 과대 시 960 재학습 비교 |
| 7 | §5 매핑·평가 | det_conf = 박스 conf / center = corner 4점 평균 / color_conf = HSV 판정 점수(`color_order.yaml`). 평가는 mAP 외에 **corner 평균 픽셀 오차(720p)·거리 구간별** 측정 | HSV 샘플링은 bbox 내부가 아니라 corner 사각형 테두리 밴드(개구부 내부는 배경). 초기 목표 가안: 평균 ≤3px, 원거리 ≤5px — 1차 학습 후 조정 |

## 기준 학습 명령

```
yolo pose train model=yolo11s-pose.pt data=window_pose.yaml imgsz=640 single_cls=True epochs=100
```

## 파생 파일

- `vision/window_pose.yaml` — 데이터 정의 (§4.3의 기계 판독본, 데이터 도착 전 작성 완료)
- `vision/color_order.yaml` — §3.1 색↔순서·HSV 실행 config (코드가 읽는 단일 기준, spec 테이블과 동시 갱신)

## 미결 · 후속

- [ ] HSV 판정 구간 실렌더 검증·미세조정 (spec §7 — 윤호 시뮬 환경 이후)
- [ ] `window_pose.yaml`의 `path:`를 데이터셋 수령 후 로컬 절대경로로 수정 (데이터셋은 git 미포함)
- [ ] 평가 목표치(3px/5px) 확정 — 1차 학습 결과 확인 후
- [ ] 클래스 전략(4번) 재평가 — 1차 학습 후 필요 시 플래그 전환만으로 3클래스 실험

---

*변경 이력: 2026-07-03 v1 작성 (결정 7건).*
