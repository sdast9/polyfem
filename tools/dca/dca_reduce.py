#!/usr/bin/env python3
"""Reduce default-controller-assessment runs to compact per-run JSON (qoi.json).

usage: dca_reduce.py RUN_DIR...   (writes RUN_DIR/qoi.json, prints one line each)

Per run: EF-01/EF-02 metrics (iterations, restarts, occupancy, trim moves and
reversals by source, attempts) and, per accepted step (last physical-diagnostics
record of the step), scalar observables:
  ke, elastic, barrier       kinetic / elastic / barrier energy (internal units)
  reaction                   net support force on the system, per axis (sum of
                             the AL reaction full-DOF vector over nodes)
  contact_force_l2           L2 norm of the barrier-contact force vector
  active, gap_mean, band_rms, gap_min   endpoint active contacts and gaps / dhat
  trim, balance              trim at the endpoint, physical_balance_pass
  termination                termination reason of the step's final solve
"""
import json, math, sys
from pathlib import Path

W = Path('/Users/stevenabramowitch/Downloads/fable_polyfem')
sys.path.insert(0, str(W / 'polyfem/tools/ef01'))
sys.path.insert(0, str(W / 'polyfem/tools/ef02'))
sys.path.insert(0, str(W / 'polyfem/tools/ef04'))
import ef01_reduce as red  # noqa: E402
import reduce as ef02red  # noqa: E402


def num(v):
    if isinstance(v, dict):
        v = v.get('value')
    return v if isinstance(v, (int, float)) and v is not None and math.isfinite(v) else None


def step_qoi(d):
    c = d.get('contact') or {}
    g = c.get('gap_statistics') or {}
    dhat = c.get('dhat') or 1.0
    out = {
        'time': num(d.get('time')), 'ke': num(d.get('kinetic_energy')),
        'elastic': num(d.get('elastic_energy')), 'barrier': num(d.get('barrier_energy')),
        'external_work': num(d.get('external_work_cumulative')),
        'active': c.get('active_count'), 'gap_mean': g.get('mean_over_dhat'),
        'band_rms': g.get('band_rms_over_dhat'),
        'gap_min': (num(c.get('min_gap')) / dhat) if num(c.get('min_gap')) is not None else None,
        'trim': c.get('trim_or_global_stiffness'),
        'balance': (d.get('physical_balance_pass') or {}).get('value'),
        'termination': (d.get('termination') or {}).get('termination_reason'),
    }
    reaction = None
    for r in d.get('reactions') or []:
        v = (r.get('full_dof_vector') or {}).get('value')
        if v and len(v) % 3 == 0:
            s = [sum(v[i::3]) for i in range(3)]
            reaction = s if reaction is None else [a + b for a, b in zip(reaction, s)]
    out['reaction'] = reaction
    for f in d.get('forms') or []:
        if f.get('name') == 'barrier-contact':
            v = (f.get('gradient_force_units') or {}).get('value') or []
            out['contact_force_l2'] = math.sqrt(sum(x * x for x in v)) if v else 0.0
    return out


def reduce(run):
    run = Path(run)
    r = red.reduce_run(run)
    try:
        r['controller'] = ef02red.controller(run)
        r['attempts'] = ef02red.attempts(run)
    except Exception as e:  # reducer must not hide a run
        r['controller_error'] = repr(e)
    per = {}
    path = run / 'output/physical-diagnostics.jsonl'
    if path.exists():
        with path.open() as f:
            for line in f:
                try:
                    d = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if d.get('outcome') == 'accepted':
                    per[d['step']] = step_qoi(d)
    r['qoi'] = per
    row = json.loads((run / 'row.json').read_text()) if (run / 'row.json').exists() else {}
    r['note'] = row.get('note')
    r['steps_requested'] = row.get('steps_requested')
    # per-step trim excursion relative to the step's first trim (all records)
    exc = {}
    for p in red.jsonl(run / 'output/trim-predictors.jsonl'):
        t = p.get('trim')
        if t:
            e = exc.setdefault(p['step'], [t, t, t])
            e[1], e[2] = min(e[1], t), max(e[2], t)
    r['max_excursion'] = max([max(e[2] / e[0], e[0] / e[1]) for e in exc.values()] + [1.0])
    # force-band decisions: vetoed softenings, and the force-weighted / rms gap
    # ratio at accepted iterations (narrow gap distributions put the fw target
    # below the average-gap collapse threshold sqrt(trim_lower))
    band = {'decisions': 0, 'proposed_down': 0, 'vetoed_down': 0, 'applied_down': 0, 'applied_up': 0}
    est = {'accepted': 0, 'rejected': 0, 'accepted_steps': [], 'factors': [], 'cosines': []}
    ratios, collapse_after_seed = [], 0
    seeded_step = None
    for p in red.jsonl(run / 'output/trim-predictors.jsonl'):
        d = p.get('controller_decision') or {}
        if p.get('event') == 'force_band':
            band['decisions'] += 1
            pf, f = d.get('proposed_factor'), d.get('factor')
            if pf is not None and pf < 1:
                band['proposed_down'] += 1
                if d.get('collapse_guard') and (f is None or f >= 1):
                    band['vetoed_down'] += 1
            if f is not None and f < 1:
                band['applied_down'] += 1
            if f is not None and f > 1:
                band['applied_up'] += 1
        if p.get('event') == 'initial_estimate':
            if d.get('accepted'):
                est['accepted'] += 1
                est['accepted_steps'].append(p['step'])
                if d.get('before'):
                    est['factors'].append(d['after'] / d['before'])
                est['cosines'].append(d.get('cosine'))
                seeded_step = p['step']
            else:
                est['rejected'] += 1
        if p.get('event') == 'iteration':
            fw = (p.get('force_weighted') or {}).get('mean_gap')
            rms = (p.get('gap') or {}).get('rms')
            if fw and rms:
                ratios.append(fw / rms)
            if seeded_step == p.get('step') and d.get('source') == 'collapse':
                collapse_after_seed += 1
    ratios.sort()
    band['fw_over_rms_median'] = ratios[len(ratios) // 2] if ratios else None
    est['collapse_decisions_in_seeded_steps'] = collapse_after_seed
    r['band'] = band
    r['estimate'] = est
    (run / 'qoi.json').write_text(json.dumps(r, default=str) + '\n')
    return r


def line(r):
    c = r.get('controller') or {}
    bal = [q.get('balance') for q in r['qoi'].values()]
    return (f"{r['label']:32s} exit={r['exit_code']} to={int(bool(r['timed_out']))} steps={r['steps_completed']}/{r['steps_requested']}"
            f" its={r['iterations']} restarts={r['log'].get('restarts')} moves={c.get('moves')} rev={c.get('reversals')}"
            f" occ={c.get('in_band')}/{c.get('observations')} exc={r['max_excursion']:.3g}"
            f" balF={bal.count(False)}/{len(bal)} wall={r['wall_seconds']}")


if __name__ == '__main__':
    for run in sys.argv[1:]:
        if (Path(run) / 'row.json').exists():
            print(line(reduce(run)), flush=True)
