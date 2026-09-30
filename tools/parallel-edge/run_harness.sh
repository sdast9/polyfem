#!/usr/bin/env bash
# usage: run_harness.sh <unit_tests> <data_dir> <outdir> [ENV=VAL ...]
# Runs gcp-contact/parallel-edge/run.json through the scene harness (run_manifest_env).
set -u
UT=$1; DATA=$2; OUT=$3; shift 3
mkdir -p "$OUT"; echo "gcp-contact/parallel-edge/run.json" > "$OUT/manifest.txt"
( cd "$OUT" && env OMP_NUM_THREADS=1 POLYFEM_RUN_MANIFEST="$OUT/manifest.txt" POLYFEM_RUN_DATA_DIR="$DATA" "$@" "$UT" run_manifest_env > harness.log 2>&1 )
echo "$* :: harness: $(grep -c 'Authenticated' "$OUT/harness.log") authenticated, $(grep -c 'Violating' "$OUT/harness.log") violations, $(grep -c 'iteration limit' "$OUT/harness.log") limit hits; $(tail -2 "$OUT/harness.log" | head -1)"
