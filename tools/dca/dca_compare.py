#!/usr/bin/env python3
"""Ensemble comparison of controller arms (default-controller assessment).

usage: dca_compare.py --group SCENE-sN [--runs-root runs] [--json out.json]

Runs are named ARM-SCENE-sN-REAL (REAL = acc1|cholmod|simplicial|r1|r2...).
Arm P is the production reference ensemble. For every other arm X and every
step t (relative L2 distance d(a,b) = |ua-ub| / mean(|ua|,|ub|) of `solution`):

  E_P(t)  mean pairwise distance inside P      (production's own spread)
  D_X(t)  mean distance between X and P runs   (cross distance)
  rho_X = sum_t D_X(t) / sum_t E_P(t) over steps with E_P(t) > DELTA
          (two-sample test in the spirit of the energy distance: if X draws
          from P's distribution, D_X ~ E_P and rho ~ 1; a systematic offset
          b gives D ~ sqrt(E_P^2 + b^2))
  steps with E_P(t) <= DELTA ("deterministic regime"): max D_X(t)

Observables (per step, from qoi.json): |net reaction|, kinetic, elastic
energy, contact-force norm. Deviation of X's ensemble mean from P's mean,
normalized by (P's range + TOL * max_t |P mean|); reported as the fraction of
steps above 1 and the time-aggregated ratio. Balance flags: false counts per
run. Cost: iterations, restarts, failures.
"""
import argparse, itertools, json, math, sys
from pathlib import Path
import numpy as np

W = Path('/Users/stevenabramowitch/Downloads/fable_polyfem')
sys.path.insert(0, str(W / 'polyfem/tools/qn_contact'))
from compare_vtu import read_field  # noqa: E402

DELTA = 1e-6
TOL = 0.01
QOIS = ('reaction', 'ke', 'elastic', 'contact_force_l2')


def dist(a, b):
    n = 0.5 * (np.linalg.norm(a) + np.linalg.norm(b))
    return float(np.linalg.norm(a - b) / n) if n > 0 else 0.0


def qval(q, key):
    v = q.get(key)
    if key == 'reaction':
        return math.sqrt(sum(x * x for x in v)) if v else None
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--group', required=True, help='e.g. IT-s200')
    ap.add_argument('--runs-root', default='runs')
    ap.add_argument('--arms', default='P,E,F,B,Er,Br')
    ap.add_argument('--json')
    ap.add_argument('--offset', type=int, default=0, help='output/data/file_index_offset of resumed runs')
    ap.add_argument('--ref', help='tight reference run directory (relative error per step)')
    a = ap.parse_args()
    root = Path(a.runs_root)
    runs = {}
    for d in sorted(root.iterdir()):
        arm, _, rest = d.name.partition('-')
        if rest.startswith(a.group + '-') and (d / 'row.json').exists() and arm in a.arms.split(','):
            runs.setdefault(arm, []).append(d)
    if 'P' not in runs:
        raise SystemExit('no production runs')
    meta = {}
    for arm, ds in runs.items():
        for d in ds:
            q = json.loads((d / 'qoi.json').read_text()) if (d / 'qoi.json').exists() else None
            meta[d.name] = q
    # steps present in every run
    def nsteps(d):
        k = 0
        while (d / f'output/step_{a.offset + k + 1}.vtu').exists():
            k += 1
        return k
    steps = min(nsteps(d) for ds in runs.values() for d in ds)
    sol = {}
    for ds in runs.values():
        for d in ds:
            sol[d.name] = [read_field(d / f'output/step_{a.offset + k}.vtu', 'solution') for k in range(1, steps + 1)]
    P = [d.name for d in runs['P']]
    EP = []
    for t in range(steps):
        pairs = [dist(sol[x][t], sol[y][t]) for x, y in itertools.combinations(P, 2)]
        EP.append(float(np.mean(pairs)) if pairs else float('nan'))
    out = {'group': a.group, 'steps_compared': steps, 'production_runs': P,
           'E_P': EP, 'E_P_max': max(EP) if EP else None, 'arms': {}}

    def qseries(names, key):
        rows = []
        for n in names:
            q = (meta.get(n) or {}).get('qoi') or {}
            rows.append([qval(q.get(str(t + 1)) or q.get(t + 1) or {}, key) for t in range(steps)])
        return rows

    for arm, ds in runs.items():
        X = [d.name for d in ds]
        entry = {'runs': X}
        cost = []
        for n in X:
            m = meta.get(n) or {}
            c = m.get('controller') or {}
            qo = m.get('qoi') or {}
            cost.append({'run': n, 'exit': m.get('exit_code'), 'timed_out': m.get('timed_out'),
                         'steps': m.get('steps_completed'), 'iterations': m.get('iterations'),
                         'restarts': (m.get('log') or {}).get('restarts'), 'moves': c.get('moves'),
                         'reversals': c.get('reversals'), 'in_band': c.get('in_band'),
                         'observations': c.get('observations'), 'max_excursion': m.get('max_excursion'),
                         'balance_false': sum(1 for v in qo.values() if v.get('balance') is False),
                         'sources': c.get('sources')})
        entry['cost'] = cost
        if a.ref:
            ref = [read_field(Path(a.ref) / f'output/step_{a.offset + k}.vtu', 'solution') for k in range(1, steps + 1)]
            entry['ref_error'] = {x: [float(np.linalg.norm(sol[x][t] - ref[t]) / np.linalg.norm(ref[t])) for t in range(steps)] for x in X}
        if arm != 'P':
            D, Ex = [], []
            # Realization-matched pairs (same linear solver = same roundoff
            # seed) are excluded, so an arm identical to P gives rho = 1.
            real = lambda n: n.rsplit('-', 1)[-1]
            cross = [(x, p) for x in X for p in P if real(x) != real(p) or len(P) == 1]
            for t in range(steps):
                D.append(float(np.mean([dist(sol[x][t], sol[p][t]) for x, p in cross])))
                pairs = [dist(sol[x][t], sol[y][t]) for x, y in itertools.combinations(X, 2)]
                Ex.append(float(np.mean(pairs)) if pairs else float('nan'))
            live = [t for t in range(steps) if EP[t] > DELTA]
            det = [t for t in range(steps) if not EP[t] > DELTA]
            entry['D'] = D
            entry['E_X'] = Ex
            entry['rho'] = (sum(D[t] for t in live) / sum(EP[t] for t in live)) if live else None
            entry['rho_max_step'] = max((D[t] / EP[t] for t in live), default=None)
            entry['det_steps'] = len(det)
            entry['det_max_D'] = max((D[t] for t in det), default=None)
            entry['D_max'] = max(D) if D else None
            qo = {}
            for key in QOIS:
                ps, xs = qseries(P, key), qseries(X, key)
                devs, num, den = [], 0.0, 0.0
                pm_all = [np.mean([r[t] for r in ps if r[t] is not None]) if any(r[t] is not None for r in ps) else None for t in range(steps)]
                scale = max((abs(v) for v in pm_all if v is not None), default=0.0)
                for t in range(steps):
                    pv = [r[t] for r in ps if r[t] is not None]
                    xv = [r[t] for r in xs if r[t] is not None]
                    if not pv or not xv:
                        continue
                    allowed = (max(pv) - min(pv)) + TOL * scale
                    dev = abs(np.mean(xv) - np.mean(pv))
                    num += dev
                    den += allowed
                    devs.append(dev / allowed if allowed > 0 else (0.0 if dev == 0 else math.inf))
                qo[key] = {'steps': len(devs), 'fraction_outside': (sum(d > 1 for d in devs) / len(devs)) if devs else None,
                           'aggregate_ratio': (num / den) if den > 0 else None, 'max_ratio': max(devs) if devs else None,
                           'scale': scale}
            entry['observables'] = qo
        out['arms'][arm] = entry
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1) + '\n')
    print(f"{a.group}: {steps} steps; production runs {P}; E_P max {out['E_P_max']:.3g}")
    for arm, e in out['arms'].items():
        its = [c['iterations'] for c in e['cost']]
        fails = [c['run'] for c in e['cost'] if c['exit'] != 0 or c['timed_out']]
        bal = [c['balance_false'] for c in e['cost']]
        rs = [c['restarts'] for c in e['cost']]
        s = f"  {arm:3s} its={its} restarts={rs} balF={bal} fails={fails}"
        if 'ref_error' in e:
            s += ' ref_err(max/last)=' + ','.join(f"{max(v):.2e}/{v[-1]:.2e}" for v in e['ref_error'].values())
        if arm != 'P':
            s += (f" rho={e['rho'] if e['rho'] is None else round(e['rho'], 3)} rho_max={e['rho_max_step'] if e['rho_max_step'] is None else round(e['rho_max_step'], 2)}"
                  f" det_steps={e['det_steps']} det_maxD={e['det_max_D'] if e['det_max_D'] is None else '%.2g' % e['det_max_D']} Dmax={e['D_max']:.3g}")
            s += ' obs=' + ','.join(f"{k}:{v['aggregate_ratio']:.2f}/{v['fraction_outside']:.2f}" for k, v in e['observables'].items() if v['aggregate_ratio'] is not None)
        print(s)


if __name__ == '__main__':
    main()
