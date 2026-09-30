#!/usr/bin/env bash
# usage: run_qemu.sh <qemu cpu model> <PolyFEM_bin> <scene.json> <outdir> [ENV=VAL ...]
# Runs the scene under qemu-user with the given emulated CPU (vendor, feature
# flags and therefore MKL/glibc run-time dispatch); TCG has no AVX-512.
set -u
CPU=$1; BIN=$2; SCENE=$3; OUT=$4; shift 4
mkdir -p "$OUT"
( cd "$OUT" && env OMP_NUM_THREADS=1 MKL_VERBOSE=1 "$@" qemu-x86_64 -cpu "$CPU" -L / "$BIN" -j "$SCENE" -o . --max_threads 1 --log_level debug > run.log 2> qemu.err )
echo "$CPU $* :: $(python3 "$(dirname "$0")/step_metrics.py" "$OUT/run.log") $(grep -m1 'MKL_VERBOSE oneMKL' "$OUT/run.log" | sed 's/.*for Intel(R) 64 architecture //' | cut -c1-70)"
