#!/usr/bin/env python3
"""One line per run: outcome, per-step accepted iterations, restarts,
Hessian factorizations, wall time, and (optionally) error vs a reference run.

usage: summarize.py [--ref RUN] RUN...
"""
import json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from compare_vtu import read_field


def load(run):
    run = Path(run)
    row = json.loads((run / 'row.json').read_text()) if (run / 'row.json').exists() else {}
    att = run / 'output/solver-attempts.jsonl'
    rows = [json.loads(l) for l in att.read_text().splitlines()] if att.exists() else []
    man = run / 'output/run-manifest.json'
    steps = json.loads(man.read_text()).get('steps', []) if man.exists() else []
    return row, rows, steps


def refreshes(rows):
    """Each sub-solve (minimize_index) builds a fresh solver whose counter
    starts at 1: the total is the sum of the per-sub-solve maxima."""
    best = {}
    for r in rows:
        v = ((r.get('solver') or {}).get('strategy_state') or {}).get('preconditioner_refreshes')
        if v is None:
            continue
        key = (r.get('step'), r.get('minimize_index'))
        best[key] = max(best.get(key, 0), v)
    return sum(best.values())


def main():
    args = sys.argv[1:]
    ref = None
    if args and args[0] == '--ref':
        ref, args = Path(args[1]), args[2:]
    for run in args:
        run = Path(run)
        row, rows, steps = load(run)
        acc = [r for r in rows if r.get('kind') == 'accepted']
        per_step = {}
        for r in acc:
            per_step[r['step']] = per_step.get(r['step'], 0) + 1
        try:  # the method actually run, also when the scene file chose it
            method = json.loads((run / 'input.json').read_text())['solver']['nonlinear'].get('solver', 'Newton')
        except (OSError, KeyError, ValueError):
            method = row.get('method')
        is_newton = 'Newton' in str(method)
        fact = len(acc) if is_newton else refreshes(rows)
        restarts = [s.get('termination', {}).get('restarts') for s in steps]
        reasons = sorted({s.get('termination', {}).get('termination_reason') for s in steps} - {None})
        err = ''
        if ref is not None and ref != run:
            errs = []
            k = 1
            while (run / f'output/step_{k}.vtu').exists() and (ref / f'output/step_{k}.vtu').exists():
                ua, ur = read_field(run / f'output/step_{k}.vtu', 'solution'), read_field(ref / f'output/step_{k}.vtu', 'solution')
                errs.append(np.linalg.norm(ua - ur) / max(np.linalg.norm(ur), 1e-300))
                k += 1
            err = ' relerr/step ' + ' '.join(f'{e:.1e}' for e in errs)
        print(f"{run.name:28s} exit={row.get('exit_code')} wall={row.get('wall_seconds')}s steps_ok={len(steps)} "
              f"iters={len(acc)} per_step={[per_step[k] for k in sorted(per_step)]} factorizations={fact} "
              f"restarts={restarts} reasons={reasons}{err}")


if __name__ == '__main__':
    main()
