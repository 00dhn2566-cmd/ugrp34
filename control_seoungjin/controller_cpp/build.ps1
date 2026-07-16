# controller_cpp 빌드 (msys64 MinGW g++)
# 주의: mingw64\bin이 PATH에 없으면 cc1plus가 DLL을 못 찾아 '조용히' 실패한다 (0xC0000135).
$env:PATH = "C:\msys64\mingw64\bin;" + $env:PATH
$gxx = "C:\msys64\mingw64\bin\g++.exe"
if (-not (Test-Path $gxx)) { Write-Error "g++ 없음: $gxx"; exit 1 }
$root = $PSScriptRoot
& $gxx -std=c++17 -O2 -Wall -Wextra -static -I "$root\include" `
    "$root\src\qc_controller.cpp" "$root\src\qc_io.cpp" "$root\src\main_trace.cpp" `
    -o "$root\qc_trace.exe"
if ($LASTEXITCODE -eq 0) { Write-Host "빌드 성공: $root\qc_trace.exe" } else { exit $LASTEXITCODE }
