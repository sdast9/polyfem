#!/usr/bin/env python3
"""Compare runs of the free-DOF gradient balance with a reference run.

usage: compare.py --pair CASE REF RUN [--pair ...] [--json OUT]

For each pair (e.g. the option-off run of the base binary as REF and a
gradient_balance_dofs = free run, or two repeats): exit, accepted Newton
iterations (total and per step), stall retunes, AL passes, the trim path
(first / last / max, number of calibration raises = trim increases at a
refresh, endpoint refresh or stall retune record without another controller
decision; trim_lowers counts every decrease), the Newton iteration records
of the trim-predictor stream (the synthetic runs write no attempt stream),
the free contacts' minimum gap at each step's last iteration
(`clamped.excluding_fully.min_gap`, else `gap.min`), byte identity of the
exported VTU files and the relative L2 solution difference per step.
Works on tools/clamped/sequence.py runs and tools/balance/synthetic.py runs.
"""
import argparse, hashlib, json, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ef01'))
import ef01_reduce as red  # noqa: E402

CALIBRATING = ('refresh', 'refresh_endpoint', 'stall_retune')


def vtu_hashes(run):
    return {f.name: hashlib.file_digest(f.open('rb'), 'sha256').hexdigest() for f in sorted((run / 'output').glob('*.vtu'))}


def trim_path(run):
    rows = red.jsonl(run / 'output/trim-predictors.jsonl')
    trims = [r['trim'] for r in rows if 'trim' in r]
    raises, small, prev = [], 0, None
    min_gap = {}
    for r in rows:
        t = r.get('trim')
        if prev is not None and t is not None and r.get('event') in CALIBRATING and t > prev and not r.get('controller_decision'):
            raises.append(dict(step=r.get('step'), event=r.get('event'), before=prev, after=t))
            small += t / prev - 1 < 1e-2
        prev = t if t is not None else prev
        if r.get('event') == 'iteration':
            c = (r.get('clamped') or {}).get('excluding_fully') or {}
            g = c.get('min_gap', (r.get('gap') or {}).get('min'))
            min_gap[r.get('step')] = g
    gaps = [g for g in min_gap.values() if g is not None]
    lowers = sum(1 for a, b in zip(trims, trims[1:]) if b < a)
    return dict(iteration_records=sum(1 for r in rows if r.get('event') == 'iteration'), trim_lowers=lowers,
                trim_first=trims[0] if trims else None, trim_last=trims[-1] if trims else None,
                trim_max=max(trims) if trims else None, trim_changes=sum(1 for a, b in zip(trims, trims[1:]) if a != b),
                calibration_raises=len(raises), calibration_raises_below_1pct=small,
                calibration_raise_events={e: sum(1 for x in raises if x['event'] == e) for e in CALIBRATING},
                free_min_gap_min=min(gaps) if gaps else None,
                free_min_gap_last_iteration_per_step=[min_gap[k] for k in sorted(min_gap, key=lambda x: (x is None, x))])


def summary(run):
    r = red.reduce_run(run)
    steps = r['steps'].values()
    return dict(label=run.name, exit_code=r['exit_code'], timed_out=r['timed_out'], wall_seconds=r['wall_seconds'],
                steps_completed=r['steps_completed'], iterations=r['iterations'], iterations_per_step=r['iterations_per_step'],
                stall_retunes=sum(v.get('stall_retunes') or 0 for v in steps),
                al_passes=sum(v.get('al_passes') or 0 for v in steps),
                physical_balance_pass=[v.get('physical_balance_pass') for v in steps], **trim_path(run))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pair', nargs=3, action='append', metavar=('CASE', 'REF', 'RUN'), required=True)
    ap.add_argument('--json')
    a = ap.parse_args()
    out = []
    for case, ref, run in a.pair:
        ref, run = Path(ref), Path(run)
        e = dict(case=case, ref=summary(ref), run=summary(run))
        hr, hm = vtu_hashes(ref), vtu_hashes(run)
        e['vtu_files'] = len(hm)
        e['vtu_identical'] = bool(hr) and hr == hm
        e['relative_solution_difference'] = red.solution_error(run, ref)
        out.append(e)
        d = e['relative_solution_difference']
        f = lambda s: (f"exit {s['exit_code']} steps {s['steps_completed']} its {s['iterations']} stalls {s['stall_retunes']} AL {s['al_passes']}"
                       f" iter-records {s['iteration_records']} trim {s['trim_first']:.3g}->{s['trim_last']:.3g} max {s['trim_max']:.3g} raises {s['calibration_raises']}"
                       f" (<1% {s['calibration_raises_below_1pct']}) lowers {s['trim_lowers']} min gap {s['free_min_gap_min']:.3g}" if s['trim_first'] is not None
                       else f"exit {s['exit_code']} its {s['iterations']} (no trim records)")
        print(f"{case}: identical={e['vtu_identical']} ({e['vtu_files']} VTU) max rel diff {max(d) if d else None}")
        print(f"   ref {ref.name}: {f(e['ref'])}")
        print(f"   run {run.name}: {f(e['run'])}")
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1) + '\n')


if __name__ == '__main__':
    main()
