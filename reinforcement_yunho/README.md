# reinforcement_yunho — 시뮬레이션 · 데이터셋 · 강화학습 (조윤호)

이 폴더는 UGRP 드론 파이프라인에서 **조윤호 담당**(Isaac Sim 환경 · 합성 데이터셋
자동생성 · 강화학습 환경)의 코드다. 팀 규격(`../window_detection_spec_v0.2.md`,
`../README.md §7`, `../overall_gilnam/docs/state_window_interface_spec_v0_1.md`)에
맞춰 정렬돼 있다.

> **설계 원칙**: Isaac Sim / GPU / 무거운 라이브러리가 필요한 부분은 전부
> import-guard된 **스텁**이고, 그 안의 *실제 로직*(투영·라벨·분할·검증·환경 스텝·
> 관측 변환)은 순수 `numpy`+`pyyaml`로 **여기서 바로 돌아가고 테스트됨**.
> 검증: `python3 tests/test_integration.py` (전 모듈 통과).

## 구조

| 경로 | 무엇 | 소비자 | 지금 실행? |
|---|---|---|---|
| `common/` | intrinsics + 투영/좌표 수학 (단일 진실원) | 내부 | ✅ |
| `sim/` | Isaac Sim 씬 생성(Replicator) · 데이터셋/§5스트림/태민 bag 내보내기 | 길남·태민 | ✅ 순수로직 / ⛔ 렌더는 Isaac 필요 |
| `rl/` | gym env(목업물리로 구동)·보상·train(스모크·체크포인트)·evaluate·**state_window_adapter** | 윤호 | ✅ 목업 / ⛔ Isaac백엔드·SB3 스텁 |
| `interface/` | 성진 궤적/모터 스키마(초 단위) + **waypoints_config**(RL→제어 seam) | 성진 | ✅ |
| `calib/` | OpenVINS `estimator_config.yaml` + Kalibr 카메라/IMU 체인 | 태민 | ✅ |
| `scripts/` | `gen_intrinsics.py` → 길남 스키마 `synth_intrinsics.yaml` | 길남 | ✅ |
| `tests/` | 전 모듈 통합 스모크 | — | ✅ |

계약서: [`CONVENTIONS.md`](CONVENTIONS.md) — 좌표계·코너순서·**인터페이스별 쿼터니언
순서표**·시간(센서=ns/제어=초)·RL 경계. 핸드오프 요약: [`HANDOFF.md`](HANDOFF.md).

## 지금 바로 되는 것 (numpy+pyyaml만)

```bash
cd reinforcement_yunho
python3 tests/test_integration.py                 # 전 모듈 물려 도는지

# 카메라 숫자 → 길남 §6 파일 (길남 스키마: width,height,fx,fy,cx,cy,distortion)
python3 scripts/gen_intrinsics.py --width 1280 --height 720 --hfov 90

# RL 파이프라인 스모크 (gpu_jobs Job 2: 학습→체크포인트→재로드)
python3 rl/train.py --smoke
# 평가 (spec §7.4): baseline 랜덤 씬 → 성공률/충돌률/평균통과시간 CSV
python3 rl/evaluate.py --policy baseline --num-scenes 20 --seed 0 --out eval.csv
```

## 팀 규격에 맞춘 것 (정렬 완료)

- **vision(길남)**: 라벨 17-토큰 YOLO-pose가 `gt_stream.parse_label_line`과 정확히
  일치. §5 스트림은 **길남의 `gt_stream.py`/`vision_msg.py`를 그대로 호출**(재구현 X).
  `eval_corners`용 `meta.jsonl`도 같이 씀. `window_pose.yaml`은 길남 소유 → path만 세팅.
- **control(성진)**: 궤적/모터 JSON `time`을 **정수 ns→float 초**로 수정(성진 실제 출력).
  성진의 진짜 입력인 **waypoints+limits+dt** 스키마 추가 = RL 정책 출력이 꽂히는 지점.
- **VIO(태민)**: `estimator_config.yaml`(모노 기본) + Kalibr 체인. 카메라-IMU 외부
  파라미터 방향 고정(camchain=`T_cam_imu`, 태민 recon=그 역행렬). ns/스테레오/노이즈 명시.
- **RL 관측(state_window_interface)**: `rl/state_window_adapter.py`가 실제 인터페이스
  dict(드론상태+창문맵)에서 학습과 **동일한 17차원 관측**을 유도. 학습 루프는 이 규격을
  타지 **않고**(스펙 §4) in-sim 관측을 쓰되, 그로부터 *유도 가능*하게 맞춤. 쿼터니언 XYZW.

## 아직 스텁 (윤호/팀이 채울 것) — 자세히는 [`HANDOFF.md`](HANDOFF.md)

1. **카메라 실제 intrinsics** → `gen_intrinsics.py`로 만들어 길남 `synth_intrinsics.yaml`에
2. **카메라↔IMU 외부 파라미터** + **IMU 노이즈 4개** + **update_rate** → `calib/`
3. **mono vs stereo** 결정 (`estimator_config.yaml` 기본 모노)
4. **로터 인덱스→기하 + CW/CCW** 매핑 (Isaac rotor config) → 성진에 공유
5. **Isaac 씬**: `sim/scene_gen.py`의 텍스처 배경/재질을 실제 USD 에셋으로, 창문 색을
   `color_order.yaml` HSV 대역 안으로
6. **보상 가중치** (`rl/configs/reward_default.yaml` 스텁 — 설계 담당 미지정)

## 문서 매핑

- `docs/To_do_checklist_yunho.md` 0번 → `scripts/gen_intrinsics.py`·`calib/`·`interface/`
- 1번(Isaac 씬) → `sim/scene_gen.py` / 2번(데이터셋) → `sim/export_*.py`
- 3번(RL 환경) → `rl/` (README §7.3–7.6 파일별 매핑은 `rl/README.md`)
- `docs/gpu_jobs_yunho.md` Job1 → `sim/`이 만든 데이터로 길남 YOLO / Job2 → `rl/train.py --smoke`
  / Job3 → `rl/` config 스윕 / Job4 → `rl/evaluate.py`

각 폴더 `README.md`에 파일별 설명·실행법·핸드오프 상세.
