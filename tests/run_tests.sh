#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
mkdir -p "$BUILD_DIR"

EXE="$BUILD_DIR/test_nmea"

echo "== Compiling host tests =="
gcc -std=gnu11 -Wall -Wextra -Wno-unused-parameter -O2 -g \
    -I"$ROOT_DIR/firmware/main" \
    -o "$EXE" \
    "$SCRIPT_DIR/test_nmea.c" \
    "$ROOT_DIR/firmware/main/nmea.c" \
    "$ROOT_DIR/firmware/main/rtcm.c" \
    "$ROOT_DIR/firmware/main/sim.c" \
    -lm

echo "== Running host tests =="
"$EXE"
echo "== Host tests completed successfully =="
