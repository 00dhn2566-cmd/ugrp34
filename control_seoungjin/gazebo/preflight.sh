#!/usr/bin/env bash
# preflight.sh — Gazebo 머신에서 가장 먼저 돌리는 한 줄. 설치는 하지 않는다
# (그 머신엔 이미 Gazebo 가 깔려 있고 직선 비행까지 돌려 본 상태다).
#
#   bash preflight.sh
#
# 하는 일 — 앞 단계가 깨지면 거기서 멈춘다. 조용히 통과시키지 않는다:
#   0. 필요한 것이 있는지 확인만 (없으면 무엇을 깔면 되는지 알려주고 종료)
#   1. 월드 재생성 + 믹서 표 자기검증        <- Gazebo 없이도 도는 검사
#   2. 제어기 단독 스모크 (qc_trace --smoke)  <- Gazebo 없이도 도는 검사
#   3. 플러그인 빌드
#   4. SDF 파싱 확인 (gz sdf -k)
#   5. 무외란 호버 3초 헤드리스 -> 로그가 생기고 경사가 유한한지
#
# 5까지 통과하면 scripts/run_matrix.sh 로 본 검증에 들어간다.
# 헤드리스(`gz sim -s`)라 GUI/GPU/WSLg 는 필요 없다.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
OUT="$HERE/out"
mkdir -p "$OUT"

step() { echo; echo "== $* =="; }
die()  { echo "[중단] $*" >&2; exit 1; }

step "0. 있는 것 확인 (설치는 안 한다)"
command -v gz      >/dev/null || die "gz 명령이 없다. Gazebo Harmonic 이 깔린 셸에서 돌릴 것."
command -v cmake   >/dev/null || die "cmake 없음 (apt install cmake)"
command -v g++     >/dev/null || die "g++ 없음 (apt install g++)"
command -v python3 >/dev/null || die "python3 없음"
echo "gz: $(gz --version 2>/dev/null | head -1)"
# 플러그인 컴파일에는 런타임이 아니라 개발 헤더가 필요하다. ros-gz 만 깔린 머신엔
# 헤더가 없을 수 있어서, 없으면 무엇을 깔면 되는지 알려주고 멈춘다.
MISSING=""
for pkg in libgz-sim8-dev libgz-plugin2-dev libsdformat14-dev; do
  dpkg -s "$pkg" >/dev/null 2>&1 || MISSING="$MISSING $pkg"
done
if [ -n "$MISSING" ]; then
  echo "개발 헤더가 없다:$MISSING" >&2
  echo "  sudo apt install -y$MISSING" >&2
  die "위 한 줄만 깔면 된다 (Gazebo 본체는 이미 있음)"
fi

step "1. 월드 생성 + 믹서 표 자기검증"
python3 worlds/gen_worlds.py || die "믹서 표가 자기검증을 통과하지 못했다. 표를 고치기 전에는 폐루프 금지."

step "2. 제어기 단독 스모크 (Gazebo 무관)"
CPP="$HERE/../controller_cpp"
cmake -S "$CPP" -B "$OUT/cpp_build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$OUT/cpp_build" -j"$(nproc)" >/dev/null
"$OUT/cpp_build/qc_trace" --smoke || die "제어기 산수 스모크 실패 — Gazebo 이전 문제다"

step "3. 플러그인 빌드"
cmake -S "$HERE/plugin" -B "$OUT/plugin_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$OUT/plugin_build" -j"$(nproc)"
SO="$OUT/plugin_build/libqc_gz_controller.so"
[ -f "$SO" ] || die "플러그인 .so 가 안 생겼다: $SO"
echo "플러그인: $SO"

step "4. SDF 파싱"
for w in worlds/fx450_qc_1kg.sdf worlds/fx450_qc_0kg.sdf; do
  gz sdf -k "$w" >/dev/null || die "SDF 파싱 실패: $w"
  echo "  ok $w"
done

step "5. 무외란 호버 3초 (헤드리스)"
bash scripts/run_case.sh preflight_hover \
  QC_MODE=hover QC_TEND=3 QC_HOVERZ=1.0 QC_TAKEOFFS=2.0 || die "호버 실행 실패"

python3 - "$OUT/preflight_hover.csv" <<'PY'
import csv, sys, math
rows = list(csv.DictReader(open(sys.argv[1])))
if len(rows) < 10:
    raise SystemExit("[중단] 로그 행이 %d 개뿐이다 — 플러그인이 안 붙었을 수 있다 "
                     "(GZ_SIM_SYSTEM_PLUGIN_PATH 확인)" % len(rows))
z = [float(r["z"]) for r in rows]
rp = [(float(r["roll"]), float(r["pitch"])) for r in rows]
if not all(math.isfinite(v) for v in z):
    raise SystemExit("[중단] 고도에 NaN/Inf")
tilt = max(math.degrees(math.hypot(a, b)) for a, b in rp)
print("행 %d개 / 최종 고도 %.3f m / 최대 경사 %.2f deg" % (len(rows), z[-1], tilt))
if tilt > 30:
    raise SystemExit("[중단] 경사 %.1f deg — 부호 사슬이 틀렸을 가능성. "
                     "먼저 `bash scripts/run_matrix.sh probe` 로 3축 프로브를 볼 것." % tilt)
print("호버 스모크 통과")
PY

step "완료"
cat <<'EOS'
다음 단계:
  bash scripts/run_matrix.sh            # 프로브 -> 호버 -> 궤적 -> 외란 -> 지연 스윕
  python3 analyze/gz_metrics.py out/*.csv
  python3 analyze/compare_plants.py     # Simulink 성적표와 나란히
EOS
