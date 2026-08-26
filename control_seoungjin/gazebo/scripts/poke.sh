#!/usr/bin/env bash
# poke.sh — 돌고 있는 시뮬에 **밖에서** 외란을 찔러 넣는다 (Gazebo ApplyLinkWrench API).
#
#   bash scripts/poke.sh <fx|fy|fz|tx|ty|tz>=<값> ...  [--world <이름>] [--persist|--clear]
#
# 예:
#   bash scripts/poke.sh fx=5                      # +x 로 5 N 한 스텝 (충격)
#   bash scripts/poke.sh ty=0.3 --persist          # pitch 0.3 N*m 지속
#   bash scripts/poke.sh fy=3 tz=0.2 --persist     # 힘과 토크를 동시에
#   bash scripts/poke.sh --clear                   # 지속 렌치 전부 해제
#
# 언제 이걸 쓰나: 시나리오를 미리 정해 돌리는 건 run_matrix.sh 가 하고, 이건 **손으로
# 찔러 보는** 용도다 (지금 이 상태에서 옆에서 밀면 어떻게 되나). 재현이 필요한 실험은
# run_case.sh 의 QC_* 로 박아 두는 쪽이 맞다 — 이건 시각이 안 남는다.
#
# ⚠ ApplyLinkWrench 의 힘은 **무게중심**에 걸린다. "옆면에 맞았다"를 표현하려면
#    등가 토크를 같이 줘야 한다 (tau = r x F). 작용점을 그대로 주고 싶으면
#    플러그인 쪽 QC_DISTPOINTX/Y/Z 를 쓸 것 — 그쪽은 r x F 를 자동으로 만든다.
#
# 한 번 주는 렌치(--persist 없음)는 **한 물리 스텝(1 ms)만** 작용한다. 1 ms 짜리
# 충격량은 아주 작으니, 눈에 보이는 반응을 원하면 --persist 로 걸었다가 --clear 로
# 끄는 편이 낫다.
set -euo pipefail

WORLD="${QC_GZ_WORLD:-fx450_qc_1kg}"
ENTITY="${QC_GZ_ENTITY:-fx450::base_link}"
FX=0; FY=0; FZ=0; TX=0; TY=0; TZ=0
MODE="once"

while [ $# -gt 0 ]; do
  case "$1" in
    --world)   WORLD="$2"; shift 2 ;;
    --entity)  ENTITY="$2"; shift 2 ;;
    --persist) MODE="persist"; shift ;;
    --clear)   MODE="clear"; shift ;;
    fx=*) FX="${1#fx=}"; shift ;;
    fy=*) FY="${1#fy=}"; shift ;;
    fz=*) FZ="${1#fz=}"; shift ;;
    tx=*) TX="${1#tx=}"; shift ;;
    ty=*) TY="${1#ty=}"; shift ;;
    tz=*) TZ="${1#tz=}"; shift ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "모르는 인자: $1" >&2; exit 2 ;;
  esac
done

command -v gz >/dev/null || { echo "gz 명령이 없다" >&2; exit 1; }

if [ "$MODE" = "clear" ]; then
  gz topic -t "/world/$WORLD/wrench/clear" -m gz.msgs.Entity \
     -p "name: \"$ENTITY\", type: LINK"
  echo "지속 렌치 해제: $ENTITY"
  exit 0
fi

TOPIC="/world/$WORLD/wrench"
[ "$MODE" = "persist" ] && TOPIC="/world/$WORLD/wrench/persistent"

MSG="entity: {name: \"$ENTITY\", type: LINK}, \
wrench: {force: {x: $FX, y: $FY, z: $FZ}, torque: {x: $TX, y: $TY, z: $TZ}}"

echo "-> $TOPIC"
echo "   F = ($FX, $FY, $FZ) N   T = ($TX, $TY, $TZ) N*m   [$MODE]"
gz topic -t "$TOPIC" -m gz.msgs.EntityWrench -p "$MSG"
