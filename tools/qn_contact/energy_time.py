#!/usr/bin/env python3
"""Energy at the start of each accepted iteration vs elapsed wall time,
pairing the k-th accepted row with the k-th 'Line search finished' log line."""
import json, re, sys
from datetime import datetime
def series(run):
    log = open(f'runs/{run}/run.log').read().splitlines()
    t0 = None; ts = []
    for l in log:
        m = re.match(r'\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]', l)
        if not m: continue
        t = datetime.strptime(m.group(1), '%Y-%m-%d %H:%M:%S.%f')
        if t0 is None and 'Rollback point of step 1' in l: t0 = t
        if 'Line search finished' in l and t0 is not None: ts.append((t - t0).total_seconds())
    rows = [json.loads(l) for l in open(f'runs/{run}/output/solver-attempts.jsonl') if '"accepted"' in l]
    return [(ts[i], r['step'], r['energy_objective_at_x0'], (r.get('solver') or {}).get('direction', {}).get('gradient_euclidean_norm')) for i, r in enumerate(rows) if i < len(ts)]
marks = [float(x) for x in sys.argv[3:]] if len(sys.argv) > 3 else [60, 120, 300, 600, 900, 1200, 1500, 1800, 2400, 3000, 3500]
a, b = series(sys.argv[1]), series(sys.argv[2])
def at(s, t):
    best = None
    for x in s:
        if x[0] <= t: best = x
    return best
print(f"{'t[s]':>6} | {sys.argv[1]:>30} | {sys.argv[2]:>30}")
for t in marks:
    x, y = at(a, t), at(b, t)
    f = lambda z: f"step {z[1]} E {z[2]:.4e} |g| {(z[3] or float('nan')):.2e}" if z else '-'
    print(f"{t:6.0f} | {f(x):>30} | {f(y):>30}")
