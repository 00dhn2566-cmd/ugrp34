#!/usr/bin/env bash
# 성능 시나리오 세트 (2026-09-06)
#
# 지금까지의 배터리는 "손잡이를 훑는" 물건이었다 (지연 스윕, kp 스윕, nl_e0 스윕).
# 이건 다르다: **손잡이를 고정하고 임무 조건만 바꾼다.** 한 지연(현실대역 20 ms /
# 자세 5 ms)에서 질량 3점과 외란 축 3개를 돌려, 능력 카드에 그대로 들어갈 표를 만든다.
#
# 짝 규약: 외란 런마다 **같은 조건의 무외란 런**을 짝으로 돌린다. 복귀 지표를
# "y 가 2 cm 밴드 안" 이 아니라 "무외란 런과의 차이" 로 재기 위해서다. 그래야
#   (a) pitch/x 축 외란도 잴 수 있고 (지금 지표는 y 만 봐서 pitch 가 측정 불가),
#   (b) 이동 추종오차가 복귀 지표에 섞이지 않는다.
# 지표 계산은 control_seoungjin/eval_scenarios.py 가 ts_*.csv 를 짝지어서 한다.
#
# 규칙: 한 번에 한 시뮬 (RAM 16 GB). save_system 금지.
set -u
MATLAB="${MATLAB_EXE:-/c/Program Files/MATLAB/R2026a/bin/matlab.exe}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
LOGDIR="$HERE/results/scenarios"
mkdir -p "$LOGDIR"

run() {                       # run <이름> <env 나열...>
    local name="$1"; shift
    echo "=== [$name] $* ==="
    ( cd "$ROOT" && env "$@" "$MATLAB" -batch \
        "cd(fullfile(pwd,'diagnose')); verify_worstcase" ) \
        > "$LOGDIR/$name.log" 2>&1
    echo "    -> $LOGDIR/$name.log (exit $?)"
}

# ── 1군: 질량 3점 × (무외란 기준 / roll 0.3 N*m 외란), 전부 20 ms ──────────
run p0_base  WC_PKG=0 WC_ONLY=K WC_NODIST=1
run p0_dist  WC_PKG=0 WC_ONLY=K
run p1_base  WC_PKG=1 WC_ONLY=K WC_NODIST=1
# 1 kg 은 축 3개를 한 프로세스에서 묶는다 (콜드 스타트 1회 절약)
run p1_dist  WC_PKG=1 WC_ONLY=K,KP,KY
# 2 kg 은 게인 법칙이 겨냥은 했는데 실측이 없던 점이다 (capability._ANCHORS[2.0]
# 이 1 kg 복사본). QC_PKG_MASS 로 parameters.m 단계에서 질량을 바꿔야 게인도 따라온다.
run p2_base  WC_PKG=2 QC_PKG_MASS=2 WC_ONLY=K WC_NODIST=1
run p2_dist  WC_PKG=2 QC_PKG_MASS=2 WC_ONLY=K

echo "완료. 지표: python control_seoungjin/eval_scenarios.py"
