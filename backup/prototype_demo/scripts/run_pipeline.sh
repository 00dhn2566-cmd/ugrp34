#!/usr/bin/env bash
# 이미지 -> YOLO 코너 -> 삼각측량 -> 웨이포인트 (전 구간, 오프라인)
#   종료 코드 0=전 창문 복원+경고0, 1=부분, 2=가중치 없음
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"
W="${WEIGHTS:-$PROTO/model/pyb_openframe_best.pt}"
[[ -f "$W" ]] || { echo "가중치 없음: $W  (model/README.md 참고)" >&2; exit 2; }
exec "$PY" "$PROTO/pipeline_demo.py" --weights "$W" "$@"
