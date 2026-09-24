#!/usr/bin/env python3
"""Print the trim of a scene's cheapest completed pinned run (fewest accepted
iterations, exit 0, all requested steps) in an EF-01 evidence directory.

usage: ef01_best.py EVIDENCE_DIR SCENE
"""
import json, sys
from pathlib import Path

ev, scene = Path(sys.argv[1]), sys.argv[2]
best = None
for run in (ev / 'runs').glob(f'pin-{scene}-k*'):
    row = json.loads((run / 'row.json').read_text()) if (run / 'row.json').exists() else None
    if not row or row.get('exit_code') != 0 or row.get('scene') != scene:
        continue
    att = run / 'output/solver-attempts.jsonl'
    its = sum(1 for l in att.read_text().splitlines() if '"kind":"accepted"' in l)
    if best is None or its < best[0]:
        best = (its, row['trim_pin'])
if best is None:
    raise SystemExit(f'no completed pinned run for {scene}')
print(best[1])
