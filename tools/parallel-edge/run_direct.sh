#!/usr/bin/env bash
# usage: run_direct.sh <PolyFEM_bin> <scene.json> <outdir> [ENV=VAL ...]
# Single-thread debug run of one scene; prints the step_metrics summary line.
set -u
BIN=$1; SCENE=$2; OUT=$3; shift 3
mkdir -p "$OUT"
( cd "$OUT" && env OMP_NUM_THREADS=1 "$@" "$BIN" -j "$SCENE" -o . --max_threads 1 --log_level debug > run.log 2>&1 )
echo "$* :: $(python3 "$(dirname "$0")/step_metrics.py" "$OUT/run.log")"
