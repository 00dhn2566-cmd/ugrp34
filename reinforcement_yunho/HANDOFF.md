# 핸드오프 — 윤호가 팀에게 주는 것 + 윤호가 정할 것

각 팀원에게 넘길 산출물과, 그걸 만들려면 **윤호가 먼저 결정해야 하는 숫자**를 정리.
"내가 못 정하는 것 = 설계/하드웨어 결정"이라 팀/윤호 몫.

## → 길남 (vision / YOLO-pose)
**주는 것**: Isaac Sim으로 만든 진짜 데이터셋 (`sim/export_dataset.py` 출력)
- `images/{train,val,test}` + `labels/{train,val,test}` (17-토큰 YOLO-pose, 80/10/10)
- `meta.jsonl` (길남 `eval_corners.py`용, 거리 정보 포함)
- §6 intrinsics: `scripts/gen_intrinsics.py`로 만들어 `overall_gilnam/vision/synth_intrinsics.yaml`에 기입
**윤호 결정**: 카메라 실제 fx,fy,cx,cy (현재 길남 placeholder 600/600/640/360)
**주의**: 창문 재질 색이 `color_order.yaml` HSV 대역 안(S≥100,V≥80)에 들어와야 색판정 성공.
`window_pose.yaml`은 **길남 소유** → 윤호는 `path:`만 세팅.

## → 태민 (VIO / OpenVINS)
**주는 것**:
- `calib/estimator_config.yaml`(모노 기본) + `kalibr_imu_chain.yaml` + `kalibr_imucam_chain.yaml`
- 테스트 비행 데이터: `sim/export_vio.py` → EuRoC-ASL 폴더(`mav0/cam0/data` + `imu0` +
  `state_groundtruth_estimate0/data.csv`), ns 타임스탬프, GT 쿼터니언 WXYZ
**윤호 결정**:
1. **카메라↔IMU 외부 파라미터**(4×4) — CAD/씬그래프에서. camchain엔 `T_cam_imu`(Kalibr 방향)로,
   태민 `window_recon_node.py`는 그 **역행렬**(cam-in-IMU) 사용 → 방향 헷갈리지 말 것
2. **IMU 노이즈 4개**(accel/gyro × noise_density/random_walk) + update_rate(200Hz) —
   Isaac IMU 센서 설정값 = kalibr_imu_chain 값 (동일하게)
3. **intrinsics를 태민 노드 소스에도** 전파 (`window_recon_node.py`/`window_sim_node.py`의
   FX/FY/CX/CY 상수, 현재 600/600/640/360 하드코딩)
4. **mono vs stereo** (기본 모노; 스테레오면 cam1 블록 + use_stereo/max_cameras)

## → 성진 (control / PID)
**주는 것**: 확정된 Isaac Sim JSON 스키마 (현재 `interface/*.schema.json` = 초 단위, **provisional**)
- `isaacsim_trajectory.json` / `isaacsim_motor_commands.json` (time=float 초, 성진 실제 출력과 일치)
- RL→제어 seam: `interface/waypoints_config.schema.json` (waypoints+limits+dt) — 정책 출력이 여기 꽂힘
**윤호 결정**:
1. **로터 인덱스→기하 + CW/CCW 회전 부호** 매핑 (Isaac rotor config) → 성진 `motor_cmd_w[0..3]` 순서 정합
2. Isaac Sim JSON 스키마 최종 서명 (성진은 이거 확정돼야 변환 로직 고정)
3. dt(기본 0.01s)·per-axis limit 값 확인

## RL 내부 (윤호 자신)
- **보상 가중치** `rl/configs/reward_default.yaml` = 스텁 (설계 담당 미지정, 체크리스트 4번)
- 창문 normal **±방향/코너 winding** 규약 (state spec §3.1 미확정) — 확정 전엔 GT 스트림에
  explicit normal 실어보내기 (`rl/state_window_adapter.py`가 그대로 씀)
- state_window_interface v0.1 = **후보안**. 회의에서 축 조합 바뀌면 adapter 재검토.

## 우선순위 (체크리스트 0번 = 남들 대기)
1. **카메라 화각 결정** → `gen_intrinsics.py` → 길남 §6 + 태민 intrinsics 동시 해제
2. **카메라-IMU 외부파라미터 + IMU 노이즈** → 태민 OpenVINS
3. **Isaac 씬** (`sim/scene_gen.py` 텍스처배경/재질 실물화) → 데이터셋 → 길남 Job 1
4. **로터 매핑 + JSON 스키마 서명** → 성진
