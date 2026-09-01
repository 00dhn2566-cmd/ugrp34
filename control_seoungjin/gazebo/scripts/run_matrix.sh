#!/usr/bin/env bash
# run_matrix.sh — 검증 행렬. 순서가 곧 논리다: 앞 단계가 깨지면 뒤 단계 숫자는 의미가 없다.
#
#   bash scripts/run_matrix.sh [probe|base|dist|delay|mass|all]     (기본 all)
#
#   probe  개루프 3축 플랜트 프로브 — 부호/이득/야우 권한. **폐루프보다 먼저.**
#   base   무외란 호버 + (있으면) 궤적 추종 — 기준 성적
#   dist   외란 응답 (토크 펄스 / 정상풍) — 능력카드 R1 대응
#   delay  지연 스윕 tau = 0~120 ms x (무외란 / 펄스 / 정상풍) + 자세 지연 관문
#          -> 08-23 지연 스펙표(무외란표 + 돌풍표)의 독립 재측정
#   mass   0 kg 월드 회귀 — 질량 1차식이 Gazebo 에서도 사는지
#
# 결과는 out/*.csv. 해석은 analyze/gz_metrics.py 가 한다.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
RUN="bash scripts/run_case.sh"
W1="$HERE/worlds/fx450_qc_1kg.sdf"
W0="$HERE/worlds/fx450_qc_0kg.sdf"

# 궤적이 있으면 궤적 케이스를 돌리고, 없으면 호버로 대체한다.
# (INTERFACE_SPEC 2절 산출물. 날것 스텝을 넣지 말 것 — 절대 규칙 3.)
TRAJ="${QC_TRAJ:-$HERE/../output/trajectory.json}"
HAVE_TRAJ=0
[ -f "$TRAJ" ] && HAVE_TRAJ=1

WHAT="${1:-all}"

do_probe() {
  echo; echo "########## probe — 개루프 플랜트 프로브 ##########"
  # 중력을 상쇄해 띄워 두고 한 축만 차동을 넣는다. 병진이 없으므로 각가속도가
  # 곧 플랜트 이득이다. yaw 는 반토크 차동으로만 생기므로 여기서 0 이 나오면
  # 믹서 표가 회전방향 패턴과 어긋난 것이다 (mixYaw != +-mixDir).
  for ch in pitch roll yaw; do
    $RUN "probe_$ch" QC_WORLD="$W1" QC_MODE=probe QC_TEND=3 \
        QC_PROBECHANNEL="$ch" QC_PROBEU=1.0 QC_PROBESTARTS=1.0 QC_PROBEDURS=1.0 \
        QC_LOGRATEHZ=1000
  done
  # 대조군: C++ 헤더 표 그대로. yaw 권한이 0 이라는 예측을 실제로 확인한다.
  $RUN "probe_yaw_headertable" QC_WORLD="$W1" QC_MODE=probe QC_TEND=3 \
      QC_MIXERTABLE=header QC_PROBECHANNEL=yaw QC_PROBEU=1.0 \
      QC_PROBESTARTS=1.0 QC_PROBEDURS=1.0 QC_LOGRATEHZ=1000
}

do_base() {
  echo; echo "########## base — 무외란 기준 ##########"
  $RUN hover10 QC_WORLD="$W1" QC_MODE=hover QC_TEND=13 QC_TAKEOFFS=3 QC_HOVERZ=1.0
  if [ "$HAVE_TRAJ" = "1" ]; then
    $RUN traj_clean QC_WORLD="$W1" QC_MODE=traj QC_TEND=40 QC_TAKEOFFS=3 QC_TRAJECTORY="$TRAJ"
  else
    echo "  (궤적 없음: $TRAJ — 궤적 케이스 건너뜀. traj_pipeline.py 산출물을 QC_TRAJ 로 주면 된다)"
  fi
}

do_dist() {
  echo; echo "########## dist — 외란 응답 ##########"
  # 능력카드 R1 과 같은 자극: 0.3 N*m x 0.3 s. Simulink qctest.torque_pulse 와 동일.
  $RUN pulse_y  QC_WORLD="$W1" QC_MODE=hover QC_TEND=16 QC_TAKEOFFS=3 \
      QC_PULSETORQUE=0.3 QC_PULSEAXIS=y QC_PULSESTARTS=8 QC_PULSEDURS=0.3
  $RUN pulse_x  QC_WORLD="$W1" QC_MODE=hover QC_TEND=16 QC_TAKEOFFS=3 \
      QC_PULSETORQUE=0.3 QC_PULSEAXIS=x QC_PULSESTARTS=8 QC_PULSEDURS=0.3
  $RUN pulse_z  QC_WORLD="$W1" QC_MODE=hover QC_TEND=16 QC_TAKEOFFS=3 \
      QC_PULSETORQUE=0.15 QC_PULSEAXIS=z QC_PULSESTARTS=8 QC_PULSEDURS=0.3
  # 정상풍: 펄스와 달리 적분기를 계속 밀어 rho(권한 점유율)를 올린다.
  # capability.json 의 gust 표가 이 상황을 가정한 것이다.
  $RUN wind_2n  QC_WORLD="$W1" QC_MODE=hover QC_TEND=16 QC_TAKEOFFS=3 QC_WINDX=2.0
  # 같은 바람이 **무게중심이 아닌 곳**에 걸리면 힘 + 모멘트가 같이 생긴다.
  # 짐이 아래(z=-0.082)에 매달려 있으니 거기 맞는 경우가 실제로 있을 법하다.
  $RUN wind_2n_offcg QC_WORLD="$W1" QC_MODE=hover QC_TEND=16 QC_TAKEOFFS=3       QC_WINDX=2.0 QC_DISTPOINTZ=-0.082
  # 힘 펄스 (돌풍 한 방). 토크 펄스와 달리 병진을 직접 민다.
  $RUN fpulse_x QC_WORLD="$W1" QC_MODE=hover QC_TEND=16 QC_TAKEOFFS=3       QC_PULSEFORCE=6.0 QC_PULSEFORCEAXIS=x QC_PULSESTARTS=8 QC_PULSEDURS=0.3
  # 힘 + 토크 동시 (옆에서 때리는 돌풍의 실제 모습).
  $RUN fpulse_mix QC_WORLD="$W1" QC_MODE=hover QC_TEND=16 QC_TAKEOFFS=3       QC_PULSEFORCEX=6.0 QC_PULSETORQUEY=0.3 QC_PULSESTARTS=8 QC_PULSEDURS=0.3
}

do_delay() {
  echo; echo "########## delay — 지연 스윕 (08-23 스펙표 독립 재측정) ##########"
  # Simscape 에서 잰 위치 지연 앵커: 0/20 ms 무손실, 40 ms 0.88, 60 ms 0.75,
  # 80 ms 0.37, 120 ms 0. capability.py::_LAT_POS_ANCHORS.
  # 여기서는 같은 tau 에 같은 자극을 넣고 **Gazebo 가 같은 절벽을 보이는지** 만 본다.
  # (배율 s 자체는 궤적 층 몫이라 여기서 재지 않는다 — analyze/compare_plants.py 참조.)
  for ms in 0 20 40 60 80 120; do
    tau="$(python3 -c "print($ms/1000.0)")"
    $RUN "tau${ms}_clean" QC_WORLD="$W1" QC_MODE=hover QC_TEND=14 QC_TAKEOFFS=3 \
        QC_POSDELAYS="$tau"
    $RUN "tau${ms}_pulse" QC_WORLD="$W1" QC_MODE=hover QC_TEND=18 QC_TAKEOFFS=3 \
        QC_POSDELAYS="$tau" QC_PULSETORQUE=0.3 QC_PULSEAXIS=y \
        QC_PULSESTARTS=8 QC_PULSEDURS=0.3
  done
  # 지연 x 정상풍. 08-23 은 **무외란과 돌풍에 표를 따로** 뒀다
  # (capability.py::_LAT_POS_ANCHORS_GUST — 40 ms 에서 벌써 0.55, 80 ms 에서 0).
  # 무외란 표보다 훨씬 가파른데 그 근거도 한 플랜트뿐이라 여기서 다시 잰다.
  # 펄스(순간 충격)와 달리 정상풍은 적분기를 계속 밀어 rho(권한 점유율)를 올린다 —
  # 돌풍 표가 가정한 상황이 이쪽이다.
  for ms in 0 20 40 60 80; do
    tau="$(python3 -c "print($ms/1000.0)")"
    $RUN "tau${ms}_gust" QC_WORLD="$W1" QC_MODE=hover QC_TEND=18 QC_TAKEOFFS=3 \n        QC_POSDELAYS="$tau" QC_WINDX=2.0
  done

  # 자세 경로 지연은 감쇄가 아니라 **관문**이라는 것이 08-23 결론이다
  # (LAT_ATT_MAX_S = 16 ms). Gazebo 에서도 16 ms 근처에서 무너지는지 본다.
  for ms in 3 8 12 16 20; do
    tau="$(python3 -c "print($ms/1000.0)")"
    $RUN "att${ms}" QC_WORLD="$W1" QC_MODE=hover QC_TEND=14 QC_TAKEOFFS=3 \
        QC_ATTDELAYS="$tau"
  done
}

do_mass() {
  echo; echo "########## mass — 0 kg 회귀 ##########"
  # 0 kg 은 게인 법칙이 가장 예민한 구간이다 (혼돈 구간 실측). Gazebo 에서
  # 무너지면 Simscape 과적합이었다는 뜻이 된다.
  $RUN hover_0kg QC_WORLD="$W0" QC_MODE=hover QC_TEND=13 QC_TAKEOFFS=3 \
      QC_PKGMASS=0.0 QC_HOVERZ=1.0
  $RUN pulse_0kg QC_WORLD="$W0" QC_MODE=hover QC_TEND=16 QC_TAKEOFFS=3 \
      QC_PKGMASS=0.0 QC_PULSETORQUE=0.15 QC_PULSEAXIS=y QC_PULSESTARTS=8 QC_PULSEDURS=0.3
}

case "$WHAT" in
  probe) do_probe ;;
  base)  do_base ;;
  dist)  do_dist ;;
  delay) do_delay ;;
  mass)  do_mass ;;
  all)   do_probe; do_base; do_dist; do_delay; do_mass ;;
  *) echo "모름: $WHAT (probe|base|dist|delay|mass|all)" >&2; exit 2 ;;
esac

echo
echo "완료. 해석:"
echo "  python3 analyze/gz_metrics.py out/*.csv"
echo "  python3 analyze/compare_plants.py"
