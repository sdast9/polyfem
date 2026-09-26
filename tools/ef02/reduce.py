#!/usr/bin/env python3
"""EF-02/03 metrics, occupancy and trim reversals. Does not turn missing evidence into a pass."""
import argparse, json, math, sys
from collections import Counter
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ef01'))
import ef01_reduce as red
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ef04'))
from ef04_tables import attempts, trim_walk


def controller(run):
    n = inside = expanded = reversals = moves = 0
    old = sign = step = None
    sources = Counter()
    for row in red.jsonl(run/'output/trim-predictors.jsonl'):
        if row.get('step') != step:
            old = sign = None
            step = row.get('step')
        trim = row.get('trim')
        if trim is not None and old is not None and trim != old:
            direction = 1 if trim > old else -1
            reversals += int(sign is not None and direction != sign)
            moves += 1
            sign = direction
            decision = (row.get('controller_decision') or {}) if row.get('event') in ('initial_estimate', 'force_band') else {}
            sources[f"{decision.get('source',row.get('event'))}:{'up' if direction>0 else 'down'}"] += 1
        old = trim
        if row.get('event') != 'iteration': continue
        gap = (row.get('force_weighted') or {}).get('mean_gap')
        if gap is not None and math.isfinite(gap):
            n += 1; inside += int(.35 <= gap <= .5); expanded += int(.325 <= gap <= .525)
    return dict(observations=n, in_band=inside, expanded_band=expanded,
                occupancy=inside/n if n else None, reversals=reversals, moves=moves, sources=dict(sources))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('evidence', type=Path);p.add_argument('--ef01', type=Path)
    a=p.parse_args();out=[]
    for run in sorted((a.evidence/'runs').iterdir()):
        if not (run/'row.json').exists(): continue
        row = json.loads((run/'row.json').read_text())
        scene = row['scene']
        ref=a.ef01/'runs'/f'ref-{scene}' if a.ef01 else None
        if ref and not ref.exists(): ref=None
        r=red.reduce_run(run,ref);r.update(controller=controller(run),attempts=attempts(run),trim_walk=trim_walk(run))
        errors = []
        config = json.loads((run/'input.json').read_text())
        streams = ['solver-attempts.jsonl', 'physical-diagnostics.jsonl']
        if config.get('solver', {}).get('contact', {}).get('barrier_stiffness') == 'semi_implicit':
            streams.append('trim-predictors.jsonl')
        for name in streams:
            path = run/'output'/name
            if not path.exists():
                errors.append(f'missing {name}')
                continue
            with path.open() as stream:
                for line_number, line in enumerate(stream, 1):
                    try:
                        json.loads(line)
                    except json.JSONDecodeError:
                        errors.append(f'{name}:{line_number}: invalid JSON')
        r['diagnostic_errors'] = errors
        r['evidence_complete'] = (not errors and row.get('exit_code') == 0
                                  and not row.get('timed_out')
                                  and r['steps_completed'] == row.get('steps_requested'))
        out.append(r)
    (a.evidence/'ef02-metrics.json').write_text(json.dumps(out,indent=2)+'\n')
    lines=['# EF-02/03 measurements','','| Run | Exit | Steps | Iterations per step | Seconds | In-band observations | Reversals | Balance |','|---|---:|---:|---|---:|---|---:|---|']
    for r in out:
        c=r['controller'];balance=[s.get('physical_balance_pass') for s in r['steps'].values()]
        lines.append(f"| {r['label']} | {r['exit_code']} | {r['steps_completed']} | {r['iterations_per_step']} | {r['wall_seconds']} | {c['in_band']}/{c['observations']} | {c['reversals']} | {balance} |")
    (a.evidence/'ef02-tables.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))
if __name__=='__main__': main()
