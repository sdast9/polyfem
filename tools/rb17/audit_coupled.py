#!/usr/bin/env python3
"""Independent arithmetic audit of saved RB-17 coupled records; no solver runs."""
import argparse
from decimal import Decimal, localcontext
import json
import math
from pathlib import Path

import numpy as np


def barrier(d, h):
    return 0. if d >= h else -2*(d*d-h*h)**2*math.log(d/h)


def force(d, h):
    if d >= h:
        return 0.
    a = d*d-h*h
    return 4*d*a*math.log(d*d/(h*h))+2*a*a/d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    provenance = json.loads((args.evidence/'provenance.json').read_text())
    count, max_force_res, max_event_error, max_telescoping = 0, 0., 0., 0.
    rows = []
    for path in sorted(args.evidence.glob('case-*.json')):
        record = json.loads(path.read_text())
        c = record['summary']
        cfg = next(x for x in provenance['manifest']['cases'] if x['id'] == c['case'])
        s = c['length_scale']
        W = np.array(record['W'])
        p = np.array(cfg['p'])*s
        if c['order'] == 'reverse':
            p = p[::-1]
        d = np.array(record['oracle']['gap'])
        k = np.array(c['k_normalized'])/s**4
        if c['order'] == 'reverse':
            k = k[::-1]
        F = k*np.array([force(x, s) for x in d])
        defect = float(np.max(np.abs(d-p-W @ F))/s)
        assert defect < 1e-8, (path, defect)
        max_force_res = max(max_force_res, defect)
        count += 1
        old = np.array(provenance['manifest']['initial_gap'])*s
        current_k = np.array(c['initial_k_normalized'])/s**4
        if c['order'] == 'reverse':
            current_k = current_k[::-1]
        initial_energy = sum(a*barrier(x, s) for a, x in zip(current_k, old))
        displacement_energy = 0.
        parameter_energy = 0.
        for i, solve in enumerate(record['solves']):
            for point in solve['path']:
                new = np.array(point['gap'])
                displacement_energy += sum(a*(barrier(y, s)-barrier(x, s)) for a, x, y in zip(current_k, old, new))
                old = new
            if i < len(record['events']):
                event = record['events'][i]
                assert np.max(np.abs(np.array(event['gap'])-old))/s < 1e-12
                assert np.allclose(event['old_k'], current_k, rtol=1e-14, atol=0)
                delta = sum((b-a)*barrier(x, s) for a, b, x in zip(event['old_k'], event['new_k'], old))
                err = abs(delta-event['energy_change'])/(1+abs(delta))
                assert err < 1e-12
                assert event['displacement_work'] == 0 and event['history_reset']
                parameter_energy += delta
                max_event_error = max(max_event_error, err)
                current_k = np.array(event['new_k'])
                count += 3
        end_energy = sum(a*barrier(x, s) for a, x in zip(current_k, old))
        telescope = abs(end_energy-initial_energy-displacement_energy-parameter_energy)/(1+abs(end_energy)+abs(initial_energy))
        assert telescope < 1e-12
        max_telescoping = max(max_telescoping, telescope)
        count += 1
        rows.append({'case': c['case'], 'factor': c['factor'], 'order': c['order'],
                     'length_scale': s, 'arithmetic': c['arithmetic'], 'oracle_coupling_defect': defect})
    # 60-digit evaluation at the selected heldout equilibrium, using no compiled
    # barrier primitive, to separate scalar-controller failure from Newton noise.
    record = next(json.loads(p.read_text()) for p in args.evidence.glob('case-*.json')
                  if (lambda c: c['case'] == 'heldout_strong_coupling' and c['factor'] == 2 and c['order'] == 'forward' and c['length_scale'] == 1 and c['arithmetic'] == 'stable_quadratic_control')(json.loads(p.read_text())['summary']))
    with localcontext() as ctx:
        ctx.prec = 60
        D = Decimal
        d = [D(str(x)) for x in record['oracle']['gap']]
        k = [D(str(x)) for x in record['summary']['k_normalized']]
        F = [a*(4*x*(x*x-1)*(x*x).ln()+2*(x*x-1)**2/x) for a, x in zip(k, d)]
        predicted = [D('.3')+F[0]+D('.8')*F[1], D('.1')+D('.8')*F[0]+F[1]]
        error = max(abs(a-b) for a, b in zip(d, predicted))
        U = D('.9').sqrt()
        assert error < D('1e-10') and d[0] > U+D('.008')
        # The independent scalar interval target is unchanged despite violation.
        L = D('.5').sqrt()
        def f(x):
            return 4*x*(x*x-1)*(x*x).ln()+2*(x*x-1)**2/x
        target = (((L-D('.3'))/f(L))*((U-D('.3'))/f(U))).sqrt()
        assert abs(target-k[0]) < D('1e-12')
        count += 2
        precise = {'gap_above_upper': str(d[0]-U), 'coupling_equation_error': str(error),
                   'scalar_target_k': str(target), 'observed_k': str(k[0]),
                   'other_contact_gap_contribution': str(D('.8')*F[1])}
    result = {'checks': count, 'maximum_independent_coupling_defect': max_force_res,
              'maximum_event_relative_error': max_event_error, 'maximum_telescoping_error': max_telescoping,
              'heldout_decimal_check': precise, 'cases': rows,
              'limits': 'Frozen spring equilibrium and conservative contact bookkeeping; no FEM, CCD, friction, external load work or production validation.'}
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'cases'}))


if __name__ == '__main__':
    main()
