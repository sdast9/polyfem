#!/usr/bin/env python3
"""Gradient balance over all DOFs vs free DOFs, from trim-predictors.jsonl.

usage: reduce.py RUN_DIR [RUN_DIR ...] [--json OUT]

Reads the full trim-predictor records' `clamped.gradient_balance` block
(docs/clamped-contacts-20260928.md): calibrate_trim's balance over all
collisions and all DOFs (`all`) and restricted to the reduced solve's free
DOFs (`free_dofs`: Dirichlet rows of both gradients zeroed). From the norms
and cosines it reconstructs the row split

    <gB,gE> = dot_free + dot_dirichlet, ||gB||^2 = bf + bd, ||gE||^2 = ef + ed

and factors the full-DOF cosine as

    cos_all = cos_free * (dot_all / dot_free) * sqrt(bf / (bf + bd)) * sqrt(ef / (ef + ed))
              = cos_free * cross * barrier_share * energy_share,

    kappa_all = kappa_free * cross * barrier_share^2.

barrier_share < 1: the clamped half of partly clamped contacts (obstacle
side, clamped body side) dilutes the balance. energy_share < 1: Dirichlet
reactions in the energy gradient dilute the cosine only (kappa is
unaffected). cross != 1: the Dirichlet rows of the two gradients correlate.

Also reported: the gate (cos >= 0.1) per variant and its flips, the records
at which each variant's balance exceeds the trim in force (a raise under
upward-only calibration, counted at calibrating events only), and
kappa / trim (the free balance equals the trim at an equilibrium without
friction, AL or other forces). The `gradient_balance` top-level block names
the active mode (`dofs: free`) when the opt-in is on.
"""
import argparse, json, math, statistics
from collections import Counter
from pathlib import Path

CALIBRATING = ('refresh', 'refresh_endpoint', 'stall_retune')
GATE = 0.1


def rows(path):
    with open(path) as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def dist(values):
    v = sorted(x for x in values if x is not None and math.isfinite(x))
    if not v:
        return None
    q = lambda p: v[min(len(v) - 1, int(p * len(v)))]
    return dict(n=len(v), min=v[0], p10=q(.1), p50=q(.5), p90=q(.9), max=v[-1])


def split(g, base='all'):
    """Row split from the controller's block (all, or excluding_fully under
    clamped_contacts exclude_statistics) and free_dofs; None when either is
    degenerate. Fully clamped collisions act on Dirichlet rows only, so the
    free rows are the same for both bases."""
    a, f = g.get(base, {}), g.get('free_dofs')
    if not f or a.get('kappa_gb') is None or f.get('kappa_gb') is None:
        return None
    ba, ea, ca = a['barrier_gradient_norm'], a['energy_gradient_norm'], a['cos_opposition']
    bf_, ef_, cf = f['barrier_gradient_norm'], f['energy_gradient_norm'], f['cos_opposition']
    dot_all, dot_free = -ca * ba * ea, -cf * bf_ * ef_
    s = dict(kappa_all=a['kappa_gb'], kappa_free=f['kappa_gb'], cos_all=ca, cos_free=cf,
             gate_all=a.get('gate_passes'), gate_free=f.get('gate_passes'),
             barrier_share=bf_ / ba if ba > 0 else None, energy_share=ef_ / ea if ea > 0 else None,
             cross=dot_all / dot_free if dot_free != 0 else None,
             barrier_dirichlet_sq_share=max(0.0, 1 - (bf_ / ba) ** 2) if ba > 0 else None,
             energy_dirichlet_sq_share=max(0.0, 1 - (ef_ / ea) ** 2) if ea > 0 else None)
    s['ratio'] = s['kappa_free'] / s['kappa_all'] if s['kappa_all'] else None
    return s


def reduce_run(run):
    run = Path(run)
    path = run / 'output/trim-predictors.jsonl'
    out = dict(run=run.name, records=0, full=0)
    if not path.exists():
        out['missing'] = 'trim-predictors.jsonl'
        return out
    keys = ('ratio', 'cos_all', 'cos_free', 'barrier_share', 'energy_share', 'cross',
            'barrier_dirichlet_sq_share', 'energy_dirichlet_sq_share')
    series = {k: [] for k in keys}
    fot, aot, bases = [], [], set()
    gate = Counter()
    raise_all, raise_free = Counter(), Counter()
    events, partly, clamped_vertices, mode_dofs = Counter(), [], None, set()
    calibrations = Counter()
    first_trim = last_trim = None
    for r in rows(path):
        out['records'] += 1
        if first_trim is None:
            first_trim = r.get('trim')
        last_trim = r.get('trim')
        d = r.get('controller_decision') or {}
        if isinstance(d, dict) and d.get('source'):
            calibrations[d['source']] += 1
        c = r.get('clamped')
        if c is None:
            continue
        clamped_vertices = c.get('clamped_vertex_count')
        if c['active']['partly']:
            partly.append(c['active']['partly'])
        g = c.get('gradient_balance')
        top = r.get('gradient_balance')
        if top is not None and 'energy_gradient_norm' in top:
            mode_dofs.add(top.get('dofs', 'all'))
        if not g:
            continue
        base = 'excluding_fully' if c.get('mode') == 'exclude_statistics' else 'all'
        bases.add(base)
        s = split(g, base)
        if s is None:
            continue
        out['full'] += 1
        ev = r.get('event')
        events[ev] += 1
        for k in keys:
            series[k].append(s[k])
        gate[(bool(s['gate_all']), bool(s['gate_free']))] += 1
        trim = r.get('trim')
        if trim:
            fot.append(s['kappa_free'] / trim)
            aot.append(s['kappa_all'] / trim)
            if ev in CALIBRATING:
                if s['gate_all'] and s['kappa_all'] > trim * (1 + 1e-12):
                    raise_all[ev] += 1
                if s['gate_free'] and s['kappa_free'] > trim * (1 + 1e-12):
                    raise_free[ev] += 1
    if not out['full']:
        out['missing'] = 'clamped.gradient_balance.free_dofs'
        return out
    out.update(
        balance_dofs=sorted(mode_dofs), controller_base=sorted(bases), events=dict(events), clamped_vertex_count=clamped_vertices,
        partly_active=dist(partly),
        **{k: dist(v) for k, v in series.items()},
        kappa_free_over_trim=dist(fot), kappa_all_over_trim=dist(aot),
        gate=dict(both=gate[(True, True)], all_only=gate[(True, False)], free_only=gate[(False, True)],
                  neither=gate[(False, False)]),
        would_raise=dict(all=dict(raise_all), free=dict(raise_free)),
        trim_first=first_trim, trim_last=last_trim, decisions=dict(calibrations))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--json')
    a = ap.parse_args()
    result = [reduce_run(r) for r in a.runs]
    if a.json:
        Path(a.json).write_text(json.dumps(result, indent=1) + '\n')
    for r in result:
        if 'missing' in r:
            print(f"{r['run']}: missing {r['missing']}")
            continue
        p = lambda k: (f"{r[k]['p50']:.3g} [{r[k]['min']:.3g}, {r[k]['max']:.3g}]" if r[k] else '-')
        print(f"{r['run']}: full {r['full']} dofs {r['balance_dofs']} clamped {r['clamped_vertex_count']}"
              f" | ratio {p('ratio')} | cos all {p('cos_all')} free {p('cos_free')}"
              f" | shares B {p('barrier_share')} E {p('energy_share')} cross {p('cross')}"
              f" | kf/trim {p('kappa_free_over_trim')} ka/trim {p('kappa_all_over_trim')}"
              f" | gate {r['gate']} | raise {r['would_raise']}")


if __name__ == '__main__':
    main()
