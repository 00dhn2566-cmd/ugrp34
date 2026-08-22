#!/usr/bin/env bash
# 성진 C++ 제어기 + 우리 래퍼 -> libqc_bridge.so (ctypes 용).
# 그의 소스는 읽기만 한다.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$(dirname "$(dirname "$HERE")")/control_seoungjin/controller_cpp"
[[ -d "$SRC" ]] || { echo "성진 소스 없음: $SRC" >&2; exit 1; }
g++ -std=c++17 -O2 -fPIC -shared -Wall \
    -I"$SRC/include" \
    "$SRC/src/qc_controller.cpp" "$SRC/src/qc_io.cpp" "$HERE/qc_bridge.cpp" \
    -o "$HERE/libqc_bridge.so"
echo "built $HERE/libqc_bridge.so"
