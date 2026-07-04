# overall_gilnam — 파이프라인 총괄 + 비전(창문 탐지)

> 담당: 류길남 — 파이프라인 전반 설계·감독 + 비전(창문 4-corner 검출, HSV 색 판정) 실무
> 기준 문서: [프로젝트 개요 (루트 README)](../README.md) · [window_detection_spec_v0.2.md](../window_detection_spec_v0.2.md)

## 폴더 구조

| 경로 | 내용 |
|---|---|
| [docs/To_do_checklist_gilnam.md](docs/To_do_checklist_gilnam.md) | 상세 진행 체크리스트 (항목별 완료 근거 병기) |
| [docs/state_window_interface_spec_v0_1.md](docs/state_window_interface_spec_v0_1.md) | 태민(드론 상태+창문 3D) → 궤적 생성자 인터페이스 **후보안** (미확정, 회의용) |
| [docs/pipeline_overview.svg](docs/pipeline_overview.svg) | 전체 파이프라인·담당자 블록 다이어그램 (루트 README에 삽입) |
| [docs/pipeline_data_flow.png](docs/pipeline_data_flow.png) | 인터페이스 후보안의 데이터 흐름 참조 이미지 |
| [vision/](vision/) | 비전 코드 전체 — 모듈·config·실행 명령은 [vision/README.md](vision/README.md) |
| [vision/model_decisions.md](vision/model_decisions.md) | 검출 모델 구조 확정 7건 (yolo11s-pose, kpt_shape [4,3], flip_idx [1,0,3,2], single_cls 학습, imgsz 640 등) |
| [vision/sample_stream/](vision/sample_stream/) | 태민(VIO)용 합성 §5+GT pose 샘플 스트림 — 착수 가이드는 [README_stream.md](vision/sample_stream/README_stream.md) |

## 작업 이력 (2026-07-02 ~ 07-04)

**07-02 — 규격 확정**
- [window_detection_spec_v0.2.md](../window_detection_spec_v0.2.md) 확정 — 데이터셋 생성(§4)·VIO 전달(§5) 규격 통일본. 윤호(데이터 생성)·류길남(학습)·태민(VIO 전달)의 단일 기준

**07-03 — 설계·코드 골격**
- 모델 구조 확정 7건 기록 + 학습·색판정 config 커밋 ([vision/model_decisions.md](vision/model_decisions.md))
- 비전 코드 골격 TDD 구현: HSV 색 판정 [vision/color_judge.py](vision/color_judge.py) (color_order.yaml 단일 기준, corner 테두리 밴드 샘플링) · §5 메시지 빌더 [vision/vision_msg.py](vision/vision_msg.py) (필드 검증) · GT 라벨→§5 어댑터 [vision/gt_stream.py](vision/gt_stream.py)
- 태민→궤적 생성자 인터페이스 후보안 v0.1 작성 ([docs/state_window_interface_spec_v0_1.md](docs/state_window_interface_spec_v0_1.md))

**07-04 — 합성 스트림 · 학습 리허설**
- 합성 씬 생성기 + 태민용 샘플 스트림 ([vision/synth_scene.py](vision/synth_scene.py), [vision/make_stream.py](vision/make_stream.py) → [vision/sample_stream/](vision/sample_stream/)): 좌표·투영 관례 명문화, 삼각측량 왕복 테스트로 자기일관성 보증 — 윤호 합류 전 태민이 융합을 착수하고 정답지(scene_gt.json)로 채점까지 가능
- 학습 리허설: 토이 120장 생성([vision/make_toy_dataset.py](vision/make_toy_dataset.py)) → yolo11n-pose 5에폭 스모크 학습 → corner 오차 평가 표 산출([vision/eval_corners.py](vision/eval_corners.py)) — 데이터→학습→예측→평가 루프 검증 완료. ultralytics==8.4.87 핀, 학습 산출물은 gitignore 처리
- 전 모듈 테스트 동반: pytest 28개 통과 (vision/tests/, 2026-07-04 실측)

## 현재 상태

- **완료**: 데이터 없이 진행 가능한 비전 파트 전체 — 규격·모델 설계·코드 골격·샘플 스트림·평가 스크립트·학습 루프 검증
- **대기 (의존)**:
  - 윤호 — 데이터셋 생성, 시뮬 렌더 색 확인(HSV 구간 미세조정), intrinsics 기입(spec §6)
  - 태민 — 샘플 스트림으로 삼각측량 융합 착수·인터페이스 검증
  - 팀 — 본 학습용 CUDA GPU 확보 논의 (윤호 시뮬 머신 겸용 여부 포함) → 윤호 GPU 클러스터(40GB×20)로 해소, [gpu_jobs_yunho.md](../reinforcement_yunho/docs/gpu_jobs_yunho.md) 참조
- 상세 체크리스트: [docs/To_do_checklist_gilnam.md](docs/To_do_checklist_gilnam.md)

## 다음 계획

데이터 도착 → 본 학습(yolo11s-pose, 100에폭 — [model_decisions](vision/model_decisions.md) 기준) → 추론 래퍼(검출→색 판정→§5 출력) → 태민과 인터페이스 검증.
