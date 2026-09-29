#!/bin/bash
# usage: bb/run.sh ARM THREADS  -- resume ball-burst from R0's state_30 for steps 31-32 with the arm's controller
set -u
W=/Users/stevenabramowitch/Downloads/fable_polyfem/default-controller-work
A=$1; T=${2:-8}; RUN=$W/bb/s31-$A; BIN=$W/bin/PolyFEM_bin-main
[ -f "$RUN/command.txt" ] && grep -q exit= "$RUN/command.txt" && { echo "skip $A"; exit 0; }
mkdir -p "$RUN/out"
{ date -u +%FT%TZ; echo "bin=$BIN params=restart-s31-$A.json threads=$T"; shasum -a 256 "$BIN" "$W/bb/params-s31-$A.json" "$W/bb/restart-s31-$A.json"; pmset -g batt | head -2; } > "$RUN/command.txt"
cd "$W/bb"
/usr/bin/time -l "$BIN" -j "restart-s31-$A.json" -o "$RUN/out" --max_threads $T --log_level debug > "$RUN/stdout.txt" 2> "$RUN/stderr.txt"
echo "exit=$? end=$(date -u +%FT%TZ)" >> "$RUN/command.txt"
