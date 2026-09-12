#!/bin/zsh
# RB-22 A/B identity check: run the five public semi-implicit smokes and the
# RB-03 Q1 hex scene single-threaded with two binaries, dump the FE collision
# proxy of each, and compare proxy files and final solutions bit-for-bit.
#   run_ab_smokes.sh <baseline PolyFEM_bin> <candidate PolyFEM_bin> <fresh output dir>
set -u
BASE=$1; CAND=$2; OUT=$3
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
SCENES=$ROOT/scenes/semi-implicit
[[ -e "$OUT" ]] && { echo "refusing to overwrite $OUT"; exit 2; }
mkdir -p "$OUT"
for label in baseline candidate; do
    bin=$BASE; [[ $label == candidate ]] && bin=$CAND
    shasum -a 256 "$bin" >> "$OUT/binaries.txt"
    for scene in quasistatic-adaptive quasistatic-semi quasistatic-semi-alhess quasistatic-semi-friction transient-semi; do
        d=$OUT/$label/$scene; mkdir -p "$d"
        (cd "$SCENES" && POLYFEM_DUMP_COLLISION_PROXY=$d/proxy.obj "$bin" --json $scene.json -o "$d" --log_level debug --max_threads 1 > "$d/run.log" 2>&1; echo $? > "$d/exit.txt")
    done
    d=$OUT/$label/rb03-hex-q1; mkdir -p "$d"
    (cd "$ROOT/tools/rb03/hex-scene" && POLYFEM_DUMP_COLLISION_PROXY=$d/proxy.obj "$bin" --json quasistatic-semi-hex.json -o "$d" --log_level debug --max_threads 1 > "$d/run.log" 2>&1; echo $? > "$d/exit.txt")
    python3 "$ROOT/tools/rb18/summarize_smokes.py" "$OUT/$label" --output "$OUT/$label-endpoints.json" > "$OUT/$label-summary.txt"
done
echo "== exit codes / proxy identity / solution identity =="
for scene in quasistatic-adaptive quasistatic-semi quasistatic-semi-alhess quasistatic-semi-friction transient-semi rb03-hex-q1; do
    b=$OUT/baseline/$scene; c=$OUT/candidate/$scene
    pb=$(shasum -a 256 "$b/proxy.obj" | cut -c1-16); pc=$(shasum -a 256 "$c/proxy.obj" | cut -c1-16)
    echo "$scene exit=$(cat $b/exit.txt)/$(cat $c/exit.txt) proxy=$pb/$pc $([[ $pb == $pc ]] && echo IDENTICAL || echo DIFFERENT)"
done | tee "$OUT/identity.txt"
python3 - "$OUT" <<'PY' | tee -a "$OUT/identity.txt"
import json, sys
out = sys.argv[1]
a = json.load(open(f"{out}/baseline-endpoints.json")); b = json.load(open(f"{out}/candidate-endpoints.json"))
for k in a:
    ea, eb = a[k], b[k]
    same = ea.get("solution_sha256") == eb.get("solution_sha256")
    diff = max(abs(x - y) for x, y in zip(ea.get("_values", []), eb.get("_values", []))) if "_values" in ea and "_values" in eb else None
    print(f"{k}: steps {ea.get('steps')}/{eb.get('steps')} solution {'IDENTICAL' if same else 'DIFFERENT'} max|diff|={diff}")
PY
