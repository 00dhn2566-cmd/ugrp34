# rl_planner 결과 — RL 계획기 vs 상태기계 (2026-10-27 밤 실행)

> 설계·사용법은 [README.md](README.md). 이 문서는 무엇을 돌렸고 무엇이 나왔는지만 적는다.
> 숫자는 모두 이 PC(RTX 4060 Ti 8 GB, Ubuntu 26.04)에서 재현 가능 — 명령은 §6.

<!-- RESULTS_SUMMARY -->

## 1. 무엇을 비교했나

- **RL**: rsl_rl PPO 정책 (actor 256-256-128, critic 512-256-128, 비대칭 — 비평가만 GT 사용). 10 Hz 로 웨이포인트 변위를 냄.
- **상태기계 (기준선)**: `backup/prototype_demo/traj_manager.py` + `window_flight.py --seq` 이식
  (`planners/fsm.py`). RL 과 **같은 행동 공간·같은 인지 추정치·같은 저수준 제어기**를 쓴다.
  파라미터는 검증용 시드(4242)에서 32개 조합 그리드 탐색으로 고른 최적값(§4) — 평가 시드(12345)와 분리.
- **장면**: 평가마다 같은 시드의 512 장면. 두 계획기는 장면·센서 잡음 난수열까지 동일 (`eval_harness.py`).
- **레벨**: L3 창문 3개·실측 잡음(σ 2~9 px, 누락 ≤10%) / L4 창문 3~5개 + 창문 yaw ±20° /
  L5 스트레스: σ 4~12 px, 누락 5~20%, yaw ±30°, 간격 2.6~3.2 m.
- **백엔드**: Isaac Sim 6.2 + Isaac Lab 3.0 (PhysX) 와 순수 torch. 같은 MDP — 정책을 한쪽에서 학습해 다른 쪽에서 평가(교차).

지표: 성공률(전 창문을 순서대로 통과), 충돌률(벽·프레임 접촉/지면/이탈/전복/순서 위반), 완료 시간(성공 에피소드),
통과 여유(통과 지점에서 개구부 가장자리까지 여유 − 기체 반경 0.30 m; 가운데 통과 시 최대 ≈ 0.2 m),
통과 시 추정 오차(통과 순간 그 창문 중심 추정의 GT 오차).

<!-- RESULTS_TABLES -->

## 6. 재현

```bash
export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y LD_LIBRARY_PATH=/data/isaac/compat_libs/root/usr/lib/x86_64-linux-gnu
PY=/data/envs/isaac/bin/python; cd reinforcement_yunho
$PY rl_planner/scripts/train.py --backend isaac --num-envs 4096 --max-iterations 2000 --run-name main_isaac
$PY rl_planner/scripts/train.py --backend torch --num-envs 4096 --max-iterations 1500 --run-name main_torch --set seed=1
$PY rl_planner/scripts/tune_fsm.py --levels 4 5 --num-scenes 512 --seed 4242          # 상태기계 튜닝 (검증 시드)
$PY rl_planner/scripts/evaluate.py --backend isaac --planners fsm,rl --policy <run>/export --level 4 \
    --config /data/runs/rl_planner/fsm_tuning/best.yaml
bash /data/runs/eval_matrix.sh                                                        # 전체 평가 매트릭스
```
