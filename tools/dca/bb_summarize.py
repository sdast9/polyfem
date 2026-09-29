#!/usr/bin/env python3
"""Per-step summary of a ball-burst resume (bb/s31-ARM): iterations, stall restarts,
trim moves/reversals by source, max excursion, min gap, outcome."""
import json, re, sys
from pathlib import Path
for arm in sys.argv[1:]:
    d = Path('/Users/stevenabramowitch/Downloads/fable_polyfem/default-controller-work/bb') / f's31-{arm}'
    log = (d / 'stdout.txt').read_text(errors='replace')
    rows = [json.loads(l) for l in (d / 'out/trim-predictors.jsonl').read_text().splitlines() if l.strip()]
    steps = {}
    for r in rows:
        s = steps.setdefault(r['step'], {'its': 0, 'moves': 0, 'rev': 0, 'up': 0, 'down': 0, 'first': r['trim'], 'lo': r['trim'], 'hi': r['trim'], 'prev': None, 'sign': None, 'mingap': 9, 'sources': {}})
        if r['event'] == 'iteration':
            s['its'] += 1
        t = r['trim']
        if s['prev'] is not None and t != s['prev']:
            sg = 1 if t > s['prev'] else -1
            s['moves'] += 1; s['up' if sg > 0 else 'down'] += 1
            s['rev'] += int(s['sign'] is not None and sg != s['sign']); s['sign'] = sg
            src = (r.get('controller_decision') or {}).get('source') or r['event']
            s['sources'][src] = s['sources'].get(src, 0) + 1
        s['prev'] = t; s['lo'] = min(s['lo'], t); s['hi'] = max(s['hi'], t)
        g = (r.get('gap') or {}).get('min')
        if g is not None: s['mingap'] = min(s['mingap'], g)
    stalls = len(re.findall(r'stall detected', log))
    crit = re.findall(r'\[critical\] (.*)', log)
    print(f'{arm}: stall restarts total {stalls}; {"FAILED: " + crit[0][:120] if crit else "completed"}; exit line: {(d / "command.txt").read_text().strip().splitlines()[-1]}')
    for k, s in sorted(steps.items()):
        exc = max(s['hi'] / s['first'], s['first'] / s['lo'])
        print(f'  step {k}: its {s["its"]} moves {s["moves"]} ({s["up"]} up/{s["down"]} down) reversals {s["rev"]} excursion {exc:.3g} trim {s["first"]:.3g}->{s["prev"]:.3g} min gap {s["mingap"]:.3f} sources {s["sources"]}')
