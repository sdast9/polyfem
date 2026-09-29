#!/usr/bin/env python3
"""Per-step summary of synthetic.py runs: iterations, stall retunes, trim path,
controller decisions, and the free contacts' realized gaps (the record's
`excluding_fully` statistics, which exclude the clamped A-B pair).

usage: synthetic_reduce.py E [--json OUT]
"""
import argparse, json
from collections import Counter
from pathlib import Path


def summarize(run):
    row = json.loads((run / 'row.json').read_text()) if (run / 'row.json').exists() else {}
    path = run / 'output/trim-predictors.jsonl'
    steps = {}
    if path.exists():
        for line in path.open():
            r = json.loads(line)
            s = steps.setdefault(r.get('step'), dict(iterations=0, stall_retunes=0, trims=[], decisions=Counter(), last=None, first_contact_iteration=None))
            s['trims'].append(r['trim'])
            ev = r.get('event')
            if ev == 'iteration':
                s['iterations'] += 1
                s['last'] = r
            elif ev == 'stall_retune':
                s['stall_retunes'] += 1
            d = r.get('controller_decision')
            if d and d.get('source'):
                s['decisions'][d['source'] + (':' + d['context'] if d.get('context') else '')] += 1
            c = r.get('clamped') or {}
            if s['first_contact_iteration'] is None and c and c['active']['free'] + c['active']['partly'] > 0:
                s['first_contact_iteration'] = r.get('iteration')
    out = dict(run=run.name, exit_code=row.get('exit_code'), timed_out=row.get('timed_out'), wall_seconds=row.get('wall_seconds'),
               conditioning_cap_lines=(run / 'run.log').read_text(errors='replace').count('Conditioning cap on first contact') if (run / 'run.log').exists() else None,
               steps=[])
    for k in sorted(steps, key=lambda x: (x is None, x)):
        s = steps[k]
        c = (s['last'] or {}).get('clamped') or {}
        ex = c.get('excluding_fully') or {}
        out['steps'].append(dict(step=k, iterations=s['iterations'], stall_retunes=s['stall_retunes'],
                                 trim_start=s['trims'][0], trim_end=s['trims'][-1], trim_max=max(s['trims']),
                                 decisions=dict(s['decisions']),
                                 active=c.get('active'), free_min_gap=ex.get('min_gap'), free_band_rms=ex.get('band_rms'),
                                 all_min_gap=(c.get('all') or {}).get('min_gap')))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('evidence', type=Path)
    ap.add_argument('--json')
    a = ap.parse_args()
    res = [summarize(r) for r in sorted(a.evidence.iterdir()) if r.is_dir()]
    if a.json:
        Path(a.json).write_text(json.dumps(res, indent=1) + '\n')
    for r in res:
        print(f"{r['run']}: exit={r['exit_code']} timeout={r['timed_out']} cap_lines={r['conditioning_cap_lines']}")
        for s in r['steps']:
            g = lambda v: f'{v:.3g}' if isinstance(v, (int, float)) else v
            print(f"   step {s['step']}: its={s['iterations']} stalls={s['stall_retunes']} trim {g(s['trim_start'])}->{g(s['trim_end'])} (max {g(s['trim_max'])})"
                  f" free min/band gap={g(s['free_min_gap'])}/{g(s['free_band_rms'])} all min={g(s['all_min_gap'])} decisions={s['decisions']}")


if __name__ == '__main__':
    main()
