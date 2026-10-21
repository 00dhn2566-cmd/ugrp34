# 인터페이스 정합성 수정 시도 (fix trial) — 2026-10-21

> 2026-10-21 · 류길남 · [interface_review_2026-10-21.md](interface_review_2026-10-21.md)의 confirmed 13건을 모듈별 Claude agent가 **각자 git worktree에서 수정**하고, 독립 agent가 diff·테스트를 재검증한 결과. 본인이 branch·테스트·E2E를 직접 확인했다.
> **팀원 모듈 branch는 제안(proposal)이다. 담당자가 diff를 보고 merge 여부를 정한다. 본인(길남)이 대신 merge하지 않는다.**

## 0. 보는 법 — `git pull` 후

| branch | 담당 | 내용 | 커밋 |
|---|---|---|---|
| `proposal/vision-hsv-band-outside` | 류길남 | HSV 색 판정 band를 corner 바깥으로 (spec §2.5) | 1 |
| `proposal/vio-interface-fixes` | 박태민 | T_IC 표준값, TUM 변환 awk, `/window_positions` §6.2 필드, policy C | 4 |
| `proposal/control-interface-fixes` | 박성진 | `motor_cmd_w` setpoint·unsigned, isaacsim producer 단일화, limits 문서, §1/§7 모순 | 4 |
| `proposal/rl-interface-fixes` | 조윤호 | normal 부호, GT pose body frame, renderer frame+개구부, calib 소유권 | 4 |
| `integration/interface-trial-2026-10-21` | (통합 확인용) | 위 넷을 main에 merge한 상태. 충돌 0 | 17 |

```bash
cd 빈폴더어어
git fetch
git log --oneline main..origin/proposal/vio-interface-fixes      # 내 모듈 커밋 목록
git diff main...origin/proposal/vio-interface-fixes               # 내 모듈 diff
git checkout -b review origin/integration/interface-trial-2026-10-21   # 합친 상태로 테스트해 보기
```

각 commit의 본문에 어떤 스펙 조항 때문에 바꿨는지 적혀 있다. **반대하면 그 branch를 merge하지 않으면 된다.** 대신 스펙(§6.1 T_IC, §3.1 normal 부호, §2.5 창문 정의, motor_cmd_w 의미) 중 어느 쪽이 틀렸는지 회의에서 말해 달라. 코드와 스펙이 서로 다른 채로는 통합이 안 된다.

## 0.1 통합 branch 결과 (본인 직접 실행, 2026-10-21)

- 네 branch merge: **conflict 0** (36 files, +874/−264). 모듈 폴더가 겹치지 않아서다.
- 전체 pytest (vision+planning+integration, control, rl): **439 passed, 8 skipped**. main 기준 429 → 439 (신규 테스트 10개, regression 0).
- `overall_gilnam/integration/e2e_rehearsal.py` 스케일 스윕: **main과 통합 branch 결과 JSON이 완전히 동일.** 이유는 E2E가 고정 입력(`vision/sample_stream/sample_stream.jsonl`, GT order_index 포함)으로 길남 모듈 안의 복원→계획만 돌리기 때문이다. 즉 "합쳐도 안 깨진다"는 확인이지, 팀원 수정이 파이프라인 품질을 바꿨는지는 **이 E2E로는 안 보인다.**
- E2E가 못 본 열린 이음새 1개 (검증자 지적): `proposal/rl-interface-fixes`가 `export_stream.py` 기본 pose frame을 body(IMU)로 바꿨는데, 길남 쪽 `eval_recon3d.py`·`noisy_stream.py`·`e2e_rehearsal.py`는 pose를 camera pose로 읽는다. 새 default로 만든 stream을 그대로 넣으면 조용히 틀린 삼각측량이 나온다. sample_stream 계약을 body frame으로 옮길지 길남이 결정해야 한다 (§4 본인 항목 참조).
- 참고: main의 x1 게이트 오차(blue 103/89mm)가 08-08 보고서(146/108mm)와 다른 것은 8월 이후 main 변경 때문이며 이번 branch와 무관하다.

## 1. 한 줄 결론

finding 13건 중 **fixed 12 / partial 1 / skipped 0**. 검증자가 `correct=false`로 본 건은 **1건** (control_seoungjin, limits semantics — docs-only라 contract 불일치가 남음). 모든 변경은 `proposal/*` branch에 있고 **main에는 병합하지 않았다**. 네 branch를 모두 합친 `integration/interface-trial-2026-10-21`도 같이 올렸다.

## 2. 모듈별 결과

### 2.1 vision_gilnam — 담당 류길남

- branch: `proposal/vision-hsv-band-outside`
- tests: baseline `82 passed in 13.74s` → after `83 passed in 9.97s` (reverse-direction test 1개 추가). 검증자 재실행: worktree 83 passed / main 82 passed.

| # | finding | status | verifier correct | regression_risk | commit |
|---|---|---|---|---|---|
| 1 | HSV color sampling band lies inside the corner polygon, but spec §2.5 defines corners as the opening's inner edge (frame is outside) | fixed | true | low | `5caf4b5` |

### 2.2 vio_taemin — 담당 박태민

- branch: `proposal/vio-interface-fixes`
- tests: pytest suite 없음 (ROS2/WSL only). baseline = `py_compile` OK + awk 합성 입력에서 결함 재현. after = `py_compile` OK, offline ROS-stub harness RECON/SIM TEST OK, 수정된 awk 출력 정상, `bash -n run_6to10.sh` OK. live ROS2 실행은 Windows에서 불가.

| # | finding | status | verifier correct | regression_risk | commit |
|---|---|---|---|---|---|
| 1 | T_IC in both nodes is the EuRoC cam0->IMU extrinsic, not the spec §6.1 standard forward-camera R_IC | fixed | true | medium | `df7f5c6` |
| 2 | TUM conversion in commands/ scrambles both quaternion orders and collapses EuRoC GT timestamps | fixed | true | low | `bd0ea8d` |
| 3 | /window_positions message shape does not match the state_window interface draft (§6.2 field names, no timestamp/frame/normal/track_id/conf) | fixed | true | low | `69b6358` |
| 4 | window_sim_node emits sentinel [-1,-1] for behind-camera corners and a visible-only center, diverging from policy C | fixed | true | low | `6a3eb5e` |

### 2.3 control_seoungjin — 담당 박성진

- branch: `proposal/control-interface-fixes`
- tests: baseline `337 passed, 8 skipped in 55.76s` → after `343 passed, 8 skipped in 25.44s` (`tests/test_isaacsim_export.py` 신규 6개, regression 0). 검증자 재실행: worktree 343 passed / main 337 passed.

| # | finding | status | verifier correct | regression_risk | commit |
|---|---|---|---|---|---|
| 1 | motor_cmd_w exported from measured rotor speed (plant state), not the controller setpoint the schema defines | fixed | true | medium | `a8e53c2` |
| 2 | Contract names sample/run_and_log.py as the real entry point and isaacsim writer; module says entry is traj_pipeline.py and isaacsim_export.py is the sole producer | fixed | true | low | `b66679f` |
| 3 | limits semantics: schema promises per-axis enforcement, module gates on xy-norm, applies x0.7 xy share and an unstated 0.8x physical cap | **partial** | **false** | none | `ba9ba9b` |
| 4 | INTERFACE_SPEC §1 says limits over budget are rejected while §7 and code clamp; §7 report contract_version 0.1 vs traj_report.py 0.2 | fixed | true | none | `2975e77` |

### 2.4 rl_yunho — 담당 조윤호

- branch: `proposal/rl-interface-fixes`
- tests: baseline `10 passed in 1.58s` → after `13 passed in 1.13s` (신규 3개: test_window_normal_sign, test_frame_outside_corners_opening_transparent, test_near_frame_occludes_far_window_but_opening_shows_it). `sim/smoke_test.py` ALL PASS, `PYTHONUTF8=1 python calib/smoke_test.py` ALL PASS (PYTHONUTF8 없이는 pre-existing cp949 UnicodeDecodeError, 미수정). 검증자 재실행: 13 passed + 두 smoke test 통과.

| # | finding | status | verifier correct | regression_risk | commit |
|---|---|---|---|---|---|
| 1 | Corner-derived window normal uses the sign the spec explicitly forbids | fixed | true | low | `9b84d6d` |
| 2 | GT pose for 태민 is written in the camera frame (body ≡ camera), not the IMU/body frame the spec fixes | fixed | true | medium | `73cc690` |
| 3 | CPU fallback renderer draws windows as filled colour panes, contrary to spec §2.5 frame-plus-open-aperture definition | fixed | true | medium | `c3d8651` |
| 4 | calib/ assigns T_cam_imu ownership to the drone CAD/Isaac scene (identity placeholder), while the spec makes taemin's constant the single truth with a fixed non-identity value | fixed | true | low | `e0a237f` |

## 3. 검증자가 문제 삼은 항목 (correct=false)

**control_seoungjin — "limits semantics: schema promises per-axis enforcement, module gates on xy-norm, applies x0.7 xy share and an unstated 0.8x physical cap"** (status partial, commit `ba9ba9b`)

> Docs-only commit (INTERFACE_SPEC §1 table, INPUT_FORMAT.md:56, EXTERNAL_INTERFACE.md policy bullet); no code changed, so the module still does NOT follow the cited contract text (waypoints_config.schema.json:24 'Enforced PER-AXIS, not on the vector magnitude'; interface/README.md:46). The new documentation is, however, accurate against the code: traj_pipeline.py PHYS 2.0/2.0/10/80 x (1-JITTER_MARGIN 0.2) = 1.6/1.6/8/64 clamp with per-key np.minimum and _limits_clamped -> traj_report LIMITS_CLAMPED (strict -> ValueError); traj_shaping.smooth_with_axis_sharing xy_share=0.7 when simultaneous; traj_gate uses np.hypot on xy for v/a/j. Fixer correctly marked this partial: resolution needs either yunho's schema/README wording to change to 'per-axis at planning stage, then clamp/share/norm gate' or the control code to change, neither of which this diff does. Pipeline risk of the diff itself: none.

## 4. 다른 모듈에 넘겨야 하는 변경 요청 (cross_module_requests)

### → reinforcement_yunho (조윤호)

- (from vision) `export_dataset` / `procedural_render.py` / sim scene: 라벨링된 YOLO-pose corner가 OPENING의 inner edge(spec §2.5)인지, coloured frame이 그 corner 바깥에만 렌더되는지 확인 요청. `To_do_checklist_gilnam.md` line 21의 2026-08-02 기록(old INSIDE band로 윤호 val 150장 94.8%)은 real render가 frame outer edge/centre에 corner를 두거나 frame이 inner band와 겹친다는 뜻일 수 있음. 이번 fix 후 vision band는 corner 바깥을 샘플링하므로 real dataset에서 validation 재실행 + labeler convention 정렬 후 retrain(체크리스트 ⓑ).
- (from vio) `prototype_demo` — `window_flight.py`, `utils/metrics.py`, `overrides/recon_rays.py`, `backup/prototype_demo/traj_manager.py`가 legacy `/window_positions` key(center_w/corners_w/width/height/min_parallax_deg)를 읽음. deprecated alias로 아직 동작하지만 §6.2 이름(center/corners_3d/size_wh, conf)으로 이전 요망. `overrides/recon_rays.py`도 legacy shape를 emit하므로 report() 대체용이면 §6.2에 맞출 것.
- (from control) `interface/waypoints_config.schema.json:5`, `interface/README.md §1 + 'Handoff'`: entry point를 `control_seoungjin/sample/run_and_log.py`에서 `control_seoungjin/traj_pipeline.py`(`python traj_pipeline.py plan --input input/<mission>.json`)로 변경. '성진 itself does NO validation' 문장은 stale — `traj_pipeline.load_mission`이 key 검증·budget clamp·schema 오류 reject(SCHEMA_ERROR) 수행.
- (from control) `interface/isaacsim_trajectory.schema.json:5`, `interface/README.md §3`: `isaacsim_trajectory.json` / `isaacsim_motor_commands.json`의 writer는 `control_seoungjin/isaacsim_export.py`(sole producer, run_and_log.py는 이제 위임)이며 run_and_log.py가 아님.
- (from control) `interface/waypoints_config.schema.json:24` ('Enforced PER-AXIS') 및 `interface/README.md` Conventions bullet: per-axis는 planning stage에서만 참. 문구를 "planning per-axis; 각 key를 (1-0.2)x physical = v 1.6 / a 1.6 / j 8 / snap 64로 clamp (LIMITS_CLAMPED, strict -> reject); x·y 동시 이동 시 x0.7 xy share; 최종 gate는 xy NORM(v2.0/a2.0/j10) + z per-axis"로 변경. source of truth = `control_seoungjin/INTERFACE_SPEC.md §1 '`limits` 적용 규칙'` table.
- (from control) `interface/isaacsim_motor_commands.schema.json:34`: motor_cmd_w description에 'unsigned (non-negative)' 명시 고려 — 성진 측이 이제 abs() 값을 emit하고 spin sign은 Isaac rotor config 소유.

### → visual_imaging_taemin (박태민)

- (from rl) `window_recon_node.py` T_IC가 아직 EuRoC cam0->IMU 값(lines 19-23)이라는 요청 — **단, 이번 trial의 vio branch commit `df7f5c6`에서 spec §6.1 값으로 이미 교체됨** (rl worktree는 vio worktree를 보지 못해 생긴 요청). `reinforcement_yunho/common.T_IC_SPEC`, `calib/kalibr_imucam_chain.yaml`과 일치 여부는 두 branch 병합 후 확인.
- (from rl, optional) `/window_positions`에 `normal` field 없음 → RL adapter가 corners_3d에서 §3.1 공식으로 유도(TL->TR->BR->BL winding 가정). 다른 winding을 emit할 가능성이 있으면 explicit `normal` 추가 요망. (역시 vio commit `69b6358`에서 normal field가 추가됨 — 병합 후 확인.)
- (from vio, 태민/윤호 CONVENTIONS.md) commands step 7: OpenVINS `state_estimate.txt`는 JPL q_GtoI quaternion, TUM/ROS는 Hamilton q_ItoG(= conjugate). translation-only evo_ape는 무관하나 rotation error 평가 시 x,y,z 부호 반전 필요. openvins_source submodule 미checkout이라 미검증 — live install에서 확인.

### → control_seoungjin (박성진)

- `controller` submodule (MATLAB, 미수정): `run_traj_baked`에 To Workspace tap `motor_cmd_w1..motor_cmd_w4`(mixer rotor angular-velocity setpoint, rad/s) + `sim_time` 추가 → baked-model path가 `--allow-measured` 없이 contract setpoint를 export하도록.

### → overall_gilnam (류길남, 본인)

- (from vision) `window_detection_spec_v0.2.md` §2.5 또는 §3에 "colour sampling은 4 corner 바로 바깥 band에서 수행"한다는 한 줄 추가 고려 — VIO/RL consumer가 color_conf의 픽셀 출처를 알 수 있게.
- (from vio) `NEWS.md` line 12, `overall_gilnam/docs/meeting_agenda_2026-08-29.md` line 70의 open 태민 action('T_IC -> standard R_IC', '/window_positions field names -> §6.2', '§5 center = 4-point mean')은 `df7f5c6` / `69b6358` / `6a3eb5e`로 완료 → tick off.
- (from vio) `overall_gilnam/docs/state_window_interface_spec_v0_1.md` §6.2 아직 미확정. spec owner 결정 필요: (a) 같은 색 window 2개일 때 track_id semantics (현재 track_id == order_index, accumulator가 order_index로 key), (b) conf 정의 — node는 `clip(min_parallax/10°, 0, 1)` 사용; 확정 또는 재정의.
- (from rl, informational) `reinforcement_yunho/sim/export_stream.py`가 기본값으로 IMU/body pose를 emit(spec §6.1); body≡camera sample_stream contract는 `pose_frame='camera'`일 때만 재현. 검증자 note: `overall_gilnam/vision/eval_recon3d.py`(line 105, body≡camera identity 가정), `noisy_stream.py`, `integration/e2e_rehearsal.py`는 stream pose block을 camera pose로 읽으므로 새 default로 쓴 stream은 silently mis-triangulate — sample_stream contract를 body frame으로 옮길지 결정 필요.

### rl_yunho 자체 메모 (미수정)

- `reinforcement_yunho/tests/test_integration.py`의 test_common_projection / test_vision_labels_and_split이 left-handed Rww(det -1)를 사용. production builder는 right-handed. 해당 pixel-ordering test에는 무해하나 convention 예시로는 오해 소지 — scope 밖.

## 5. 다음 액션

- **본인 모듈(overall_gilnam)**: `proposal/vision-hsv-band-outside`의 `5caf4b5`를 diff 검토 후 main에 merge. 이어서 `To_do_checklist_gilnam.md` line 21(94.8% 기록 stale) 갱신, 윤호의 §2.5 renderer(`c3d8651`)로 재생성한 dataset에서 HSV 검증 재실행, §4의 spec 결정 2건(track_id, conf)과 sample_stream pose frame 결정을 본인이 처리.
- **팀원 모듈**: `proposal/*` branch를 push했으므로 각 담당자는 branch 이름 + commit 해시 + 검증자 regression_risk/note(특히 medium 항목: 태민 T_IC EuRoC-bag rehearsal 재실행 필요, 성진 baked path `--allow-measured` workflow 변경, 윤호 export_stream default 변경·seed 재현성 변경)를 전달하고 본인 확인 후 merge 여부를 결정하게 한다. §4의 cross_module_requests는 해당 모듈 담당자에게 그대로 전달. control #3(limits)은 윤호 schema/README 문구 변경으로 닫는 것이 fixer·verifier 공통 의견.
- branch 확인 명령 (`빈폴더어어/`에서): `git fetch && git diff main...origin/proposal/vio-interface-fixes` (다른 branch도 동일 형식)