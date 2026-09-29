#!/usr/bin/env python3
"""Initial trim estimate decisions (force-weighted mode) per run.

usage: estimates.py RUN [RUN ...]

Counts the distinct initial-estimate controller decisions, how many were accepted, and lists (step, cosine,
estimate / trim before, accepted) for each.
"""
import json, sys
from pathlib import Path

for run in sys.argv[1:]:
    rows = [json.loads(l) for l in open(Path(run) / 'output/trim-predictors.jsonl')]
    # A decision stays in the records until the next one replaces it:
    # count each distinct decision once.
    d, seen = [], None
    for r in rows:
        x = r.get('controller_decision') or {}
        if x.get('source') == 'initial_estimate' and x != seen:
            d.append((r.get('step'), x))
        seen = x
    acc = [x for x in d if x[1].get('accepted')]
    print(json.dumps(dict(run=Path(run).name, estimates=len(d), accepted=len(acc),
                          detail=[(s, round(x['cosine'], 3), round(x['estimate'] / x['before'], 3), x['accepted']) for s, x in d][:12])))
