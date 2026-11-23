# Build and run the host verification suite with MinGW gcc.
#
# The simulator's portable core (nmea.c / rtcm.c / sim.c) is compiled for the
# host alongside a faithful replica of ArduPilot's AP_GPS_NMEA parser, so the
# bytes the ESP32 would emit are checked against the flight controller's real
# decoding rules rather than against our own expectations.

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$out  = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $out | Out-Null

$exe = Join-Path $out 'test_nmea.exe'

Write-Host "== compiling host tests =="
& gcc -std=c11 -Wall -Wextra -Wno-unused-parameter -O1 -g `
    -o $exe `
    (Join-Path $PSScriptRoot 'test_nmea.c') `
    (Join-Path $root 'firmware\main\nmea.c') `
    (Join-Path $root 'firmware\main\rtcm.c') `
    (Join-Path $root 'firmware\main\sim.c') `
    -lm

if ($LASTEXITCODE -ne 0) {
    Write-Host "COMPILE FAILED" -ForegroundColor Red
    exit 1
}

Write-Host "== running =="
& $exe
$rc = $LASTEXITCODE
if ($rc -ne 0) {
    Write-Host "TESTS FAILED (exit $rc)" -ForegroundColor Red
} else {
    Write-Host "TESTS PASSED" -ForegroundColor Green
}
exit $rc
