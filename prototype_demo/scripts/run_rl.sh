#!/usr/bin/env bash
# 학습된 PPO 정책 평가 + 비행 GIF/궤적 렌더
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"
P="${POLICY:-$PROTO/model/ppo_window_3win.zip}"
[[ -f "$P" ]] || { echo "정책 없음: $P  (model/README.md 참고)" >&2; exit 2; }
exec "$PY" "$PROTO/rl_demo.py" --policy "$P" "$@"
