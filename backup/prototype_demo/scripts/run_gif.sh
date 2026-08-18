#!/usr/bin/env bash
# 현재 가중치로 비행 GIF 를 뽑는다 — 카메라 시점 + 실시간 검출 오버레이.
#
#   bash scripts/run_gif.sh                    # 25초 GIF, 기본 가중치
#   bash scripts/run_gif.sh --seconds 40       # 더 길게
#   bash scripts/run_gif.sh --mode scan        # 복도 전진+yaw 경로
#   OUT=/tmp/a.gif bash scripts/run_gif.sh     # 출력 위치 지정
#
# 종료 코드 0=성공, 2=가중치 없음
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

OUT="${OUT:-/home/yoonho/fig/6_render/v2_flight.gif}"
SECONDS_TOTAL="${SECONDS_TOTAL:-25}"

echo "env      $ENV_KIND"
echo "out      $OUT"
"$PY" "$PROTO/make_gif.py" --out "$OUT" --seconds "$SECONDS_TOTAL" "$@"

# 다 만들었으면 바로 띄운다 (WSL 이면 Windows 기본 뷰어, 아니면 xdg-open)
if [[ -f "$OUT" ]] && [[ -z "${NO_OPEN:-}" ]]; then
  if command -v wslview >/dev/null 2>&1; then wslview "$OUT" >/dev/null 2>&1 || true
  elif command -v explorer.exe >/dev/null 2>&1; then explorer.exe "$(wslpath -w "$OUT")" >/dev/null 2>&1 || true
  elif command -v xdg-open  >/dev/null 2>&1; then xdg-open  "$OUT" >/dev/null 2>&1 || true
  fi
fi
