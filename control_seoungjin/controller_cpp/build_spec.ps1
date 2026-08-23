# 지연/스펙 트레이스 빌드 (언어 간 대조용). 본 빌드는 build.ps1.
$env:PATH = "C:\msys64\mingw64\bin;" + $env:PATH
$gxx = "C:\msys64\mingw64\bin\g++.exe"
if (-not (Test-Path $gxx)) { Write-Error "g++ 없음: $gxx"; exit 1 }
$root = $PSScriptRoot
& $gxx -std=c++17 -O2 -Wall -Wextra -static -I "$root\include" `
    "$root\src\main_spec_trace.cpp" -o "$root\qc_spec_trace.exe"
if ($LASTEXITCODE -eq 0) { Write-Host "OK: $root\qc_spec_trace.exe" } else { exit $LASTEXITCODE }
