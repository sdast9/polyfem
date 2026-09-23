#!/usr/bin/env python3
"""Per-iteration table from a run's solver-attempts.jsonl (accepted rows)."""
import json, sys, math
from pathlib import Path
run = Path(sys.argv[1]); every = int(sys.argv[2]) if len(sys.argv) > 2 else 1
rows = [json.loads(l) for l in (run / 'output/solver-attempts.jsonl').read_text().splitlines()]
acc = [r for r in rows if r.get('kind') == 'accepted']
kinds = {}
for r in rows: kinds[r.get('kind')] = kinds.get(r.get('kind'), 0) + 1
print('row kinds', kinds)
def g(d, *k):
    for x in k:
        if not isinstance(d, dict) or x not in d: return None
        d = d[x]
    return d
print(f"{'#':>5} {'step':>4} {'it':>4} {'E(x0)':>12} {'|g|':>10} {'|p|/|g|':>9} {'alpha':>9} {'a/feas':>7} {'feas':>9} {'trialLinf':>9} {'bound':>7} {'lsit':>4} {'src':>28} gen")
for i, r in enumerate(acc):
    if i % every and i != len(acc) - 1: continue
    s = r.get('solver') or {}
    print(f"{i:5d} {r.get('step'):4} {r.get('iteration'):4} {r.get('energy_objective_at_x0', float('nan')):12.5e} "
          f"{g(s,'direction','gradient_euclidean_norm') or float('nan'):10.3e} {g(s,'direction','norm_over_gradient_norm') or float('nan'):9.2e} "
          f"{g(s,'accepted','alpha') or float('nan'):9.2e} {g(s,'line_search','accepted_over_feasible') or float('nan'):7.2g} "
          f"{g(s,'line_search','feasible_step_size') or float('nan'):9.2e} {g(r,'trial','linf') or float('nan'):9.2e} {g(r,'trial','step_bound') or float('nan'):7.2g} "
          f"{g(s,'line_search','iterations') or 0:4} {str(g(s,'strategy_state','direction_source'))[:28]:>28} {s.get('objective_generation')}")
