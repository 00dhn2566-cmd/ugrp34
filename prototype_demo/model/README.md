# prototype/model — 가중치 놓는 곳

리포 루트 `.gitignore` 에 `*.pt` 가 있어서 **가중치는 git 에 안 올라갑니다.**
받아서 이 폴더에 넣어 주세요.

| 파일 | 무엇 | 도메인 | 어디서 |
|---|---|---|---|
| `pyb_openframe_best.pt` | YOLO11s-pose 파인튜닝 (3클래스) | **뚫린 테두리 창문** | 아래 재현 명령 또는 윤호 |
| `window_yolo11s_best.pt` | 원본 1차 학습 (single_cls) | 꽉 찬 색판 창문 | 윤호 GPU 클러스터 Job 1 (`window_models.tgz`) |
| `ppo_window_3win.zip` | PPO 경로 정책 (창문 3개) | — | `window_models.tgz` |

## 두 검출 가중치의 차이 — 섞어 쓰면 안 됩니다

- **`window_yolo11s_best.pt` (원본)**: `single_cls=True` 로 학습돼 **색을 모릅니다.**
  클래스가 `{0: 'item'}` 하나뿐이고, 색·통과순서는 `color_judge.py` 의 HSV 후처리가
  붙입니다. 창문이 겹치면 앞 창문 색을 뒤 창문에도 붙이는 실패가 납니다
  (실측: 세 창문 전부 red 로 판정, 삼각측량이 서로 다른 창문을 하나로 융합).
  자기 도메인(꽉 찬 색판)에서는 keypoint mAP50-95 **0.927**.

- **`pyb_openframe_best.pt` (파인튜닝)**: 백본 동결(`freeze=11`), 넥+헤드만 학습,
  `single_cls=False` 로 **3클래스(red/green/blue)**. 색을 네트워크가 직접 냅니다.
  뚫린 테두리 창문에 맞춰져 있어 뒤 창문이 개구부로 보이고, 겹침 오판이 구조적으로
  사라집니다.

`pipeline_demo.py` 는 기본으로 파인튜닝 가중치를 씁니다. 원본으로 비교하려면:

```bash
WEIGHTS=prototype/model/window_yolo11s_best.pt \
  bash prototype/scripts/run_pipeline.sh --pane
```
(`--pane` 은 창문을 꽉 채워 원본이 학습한 도메인으로 되돌립니다.)

## 파인튜닝 가중치 재현

```bash
conda activate ugrp
cd reinforcement_yunho

# 1) 데이터셋 (PyBullet 렌더 + GT 코너 투영 라벨, 3클래스)
python sim/pybullet_dataset.py --out ~/models/pyb_ds --num-frames 1024 --seed 1

# 2) 파인튜닝 (MX450 2GB 기준 batch=4 가 실측 최속, 약 40~50분)
python sim/finetune_pybullet.py

# 3) 결과 복사
cp ~/models/finetune/pyb_openframe/weights/best.pt \
   ../prototype/model/pyb_openframe_best.pt
```

라벨은 창문 코너를 투영해서 만들기 때문에 **어노테이션이 필요 없습니다** — 프레임 수만
늘리면 데이터가 늘어납니다.
