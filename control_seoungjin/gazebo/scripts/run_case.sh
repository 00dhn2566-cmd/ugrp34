#!/usr/bin/env bash
# run_case.sh — 케이스 하나를 헤드리스로 돌리고 CSV 를 남긴다.
#
#   bash scripts/run_case.sh <이름> [QC_KEY=VAL ...]
#
# 예:
#   bash scripts/run_case.sh hover10      QC_MODE=hover QC_TEND=10
#   bash scripts/run_case.sh pulse_y      QC_MODE=hover QC_TEND=12 QC_PULSETORQUE=0.3 QC_PULSESTARTS=6
#   bash scripts/run_case.sh tau60        QC_MODE=hover QC_TEND=12 QC_POSDELAYS=0.060 QC_PULSETORQUE=0.3 QC_PULSESTARTS=6
#   bash scripts/run_case.sh probe_pitch  QC_MODE=probe QC_TEND=3  QC_PROBECHANNEL=pitch QC_PROBEU=1.0
#   bash scripts/run_case.sh line         QC_MODE=traj  QC_TEND=30 QC_TRAJECTORY=/abs/path/trajectory.json
#
# QC_* 는 그대로 플러그인 설정을 덮어쓴다 (SDF 태그명을 대문자로 한 것).
# 스크립트 전용 변수:
#   QC_WORLD   월드 파일 (기본 worlds/fx450_qc_1kg.sdf)
#   QC_TEND    시뮬 종료 시각 [s] (기본 10). --iterations 로 환산해 스스로 종료한다.
#   QC_VERBOSE gz 로그 레벨 (기본 3 — [qc] 설정 요약이 보인다)
#
# 고정 스텝 1 ms 라 --iterations = TEND*1000. 벽시계와 무관하게 결정론적으로 돈다
# (월드의 real_time_factor 는 0 = 최대 속도).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$HERE/out"
mkdir -p "$OUT"

if [ $# -lt 1 ]; then
  echo "사용법: bash scripts/run_case.sh <이름> [QC_KEY=VAL ...]" >&2
  exit 2
fi
NAME="$1"; shift
for kv in "$@"; do
  case "$kv" in
    *=*) export "$kv" ;;
    *) echo "인자는 KEY=VAL 형태여야 한다: $kv" >&2; exit 2 ;;
  esac
done

WORLD="${QC_WORLD:-$HERE/worlds/fx450_qc_1kg.sdf}"
TEND="${QC_TEND:-10}"
VERB="${QC_VERBOSE:-3}"
SO_DIR="$OUT/plugin_build"

[ -f "$WORLD" ] || { echo "월드 없음: $WORLD (python3 worlds/gen_worlds.py 먼저)" >&2; exit 1; }
[ -f "$SO_DIR/libqc_gz_controller.so" ] || {
  echo "플러그인이 안 빌드돼 있다: $SO_DIR/libqc_gz_controller.so" >&2
  echo "  bash preflight.sh" >&2; exit 1; }

ITER="$(python3 -c "print(int(round(float('$TEND')/0.001)))")"

export QC_LOG="$OUT/$NAME.csv"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$SO_DIR${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

echo "[$NAME] world=$(basename "$WORLD") T=${TEND}s iter=$ITER -> $QC_LOG"
gz sim -s -r --iterations "$ITER" -v "$VERB" "$WORLD"

if [ ! -s "$QC_LOG" ]; then
  echo "[$NAME] 로그가 비었다 — 플러그인이 안 붙었을 가능성 (GZ_SIM_SYSTEM_PLUGIN_PATH / filename 대조)" >&2
  exit 1
fi
echo "[$NAME] $(($(wc -l < "$QC_LOG") - 1)) 행"
