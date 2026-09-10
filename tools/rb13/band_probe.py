#!/usr/bin/env python3
"""RB-13 stage 2; see stage2-protocol.md for predeclared scope and tolerances."""

import argparse
from decimal import Decimal, getcontext
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import sys

getcontext().prec = 60
D = lambda x: Decimal(str(x))
GAP_TOL = 2e-11
RESIDUAL_TOL = 2e-10


def f(d, h=D(1)):
    """Physical-distance force in Decimal, independent of compiled IPC code."""
    r = d / h
    return h**3 * (8*r*(r*r-1)*r.ln() + 2*(r*r-1)**2/r)


def interval(Klo, Khi, plo, phi, lower, upper, h=D(1)):
    vals = (Klo, Khi, plo, phi, lower, upper, h)
    if not all(x.is_finite() for x in vals):
        raise ValueError('nonfinite reference input')
    if not (0 < Klo <= Khi and plo <= phi and 0 < lower < upper < h):
        raise ValueError('invalid SPD stiffness, enclosure or physical band')
    raw_lo = Khi * max(lower-plo, D(0)) / f(lower, h)
    raw_hi = Klo * max(upper-phi, D(0)) / f(upper, h)
    if plo > upper:
        status = 'inactive_band_demand'
    elif phi > upper:
        status = 'mixed_band_applicability'
    elif raw_lo > raw_hi:
        status = 'empty'
    elif raw_hi == 0:
        status = 'zero_only'
    else:
        status = 'nonempty'
    # Raw values are diagnostics only for the excluded demand regimes.
    bounds = None if status in ('inactive_band_demand', 'mixed_band_applicability') else (raw_lo, raw_hi)
    return status, bounds, (raw_lo, raw_hi)


def decimal_root(K, p, k, h=D(1)):
    if k == 0 or p >= h:
        return p
    lo, hi = D(0), h
    for _ in range(120):
        mid = (lo+hi)/2
        if K*(mid-p) - k*f(mid, h) < 0:
            lo = mid
        else:
            hi = mid
    return (lo+hi)/2


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ipc-source', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--cxx', default='/usr/bin/c++')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    here, ipc = Path(__file__).resolve().parent, args.ipc_source.resolve()
    sources = [here/'band_bridge.cpp', ipc/'src/ipc/barrier/barrier.cpp']
    command = [args.cxx, '-std=c++17', '-O2', '-I'+str(ipc/'src'),
               *map(str, sources), '-o', str(args.output/'band_bridge')]
    provenance = {'command': command, 'python': sys.version, 'platform': platform.platform(),
                  'ipc_head': subprocess.check_output(['git', '-C', str(ipc), 'rev-parse', 'HEAD'], text=True).strip(),
                  'hashes': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                             sources+[Path(__file__), here/'stage2-protocol.md', ipc/'src/ipc/barrier/barrier.hpp']}}
    (args.output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    build = subprocess.run(command, text=True, capture_output=True)
    (args.output/'compile.log').write_text(build.stdout+build.stderr)
    build.check_returncode()
    groups, failures, queries = {}, [], []

    def require(group, condition, detail=None, error=None):
        row = groups.setdefault(group, {'checks': 0, 'failed': 0, 'max_error': None})
        row['checks'] += 1
        if error is not None:
            row['max_error'] = max(row['max_error'] or 0., float(error))
        if not condition:
            row['failed'] += 1
            failures.append({'group': group, 'detail': detail})

    def add(group, K, p, k, band=None, expected=None, relation=None, status='ok'):
        q = {'group': group, 'K': K, 'p': p, 'k': k, 'h': D(1),
             'band': band, 'expected': expected, 'relation': relation, 'status': status}
        queries.append(q)
        return len(queries)-1

    exact, monotonic = [], []
    for low, high in [('.2', '.55'), ('.4', '.7'), ('.65', '.9')]:
        lower, upper = D(low), D(high)
        for K in map(D, ('.5', '6', '50')):
            for p in (D('-.2'), D(0), lower/2, lower, (lower+upper)/2, upper, upper+D('.05'), D(1), D('1.2')):
                status, bounds, raw = interval(K, K, p, p, lower, upper)
                expected_status = 'inactive_band_demand' if p > upper else ('zero_only' if p == upper else 'nonempty')
                require('exact_classification', status == expected_status)
                exact.append({'band': [str(lower), str(upper)], 'K': str(K), 'p': str(p),
                              'status': status, 'bounds': None if bounds is None else list(map(str, bounds))})
                if bounds is None:
                    require('inactive_has_no_interval', bounds is None)
                    for k in (D(0), D('.1'), D(1)):
                        add('inactive_roots', K, p, k, relation=('above', upper))
                    continue
                klo, khi = bounds
                seq = []
                for i in range(5):
                    k = klo+(khi-klo)*D(i)/4
                    root_expected = max(lower, p) if i == 0 else (upper if i == 4 else None)
                    seq.append(add('exact_coverage', K, p, k, (lower, upper), root_expected))
                monotonic.append(seq)
                if klo > 0:
                    add('exact_below_lower', K, p, klo*D('.99'), relation=('below', lower))
                if khi > 0:
                    add('exact_above_upper', K, p, khi*D('1.01'), relation=('above', upper))
                else:
                    add('zero_only_positive_k', K, p, D('.001'), relation=('above', upper))

    boxes = []
    lower, upper = D('.4'), D('.7')
    specs = [('positive', '4', '6', '.1', '.2', 'nonempty'),
             ('zero_lower', '4', '6', '.45', '.55', 'nonempty'),
             ('zero_only', '4', '6', '.45', '.7', 'zero_only'),
             ('empty', '1', '100', '-.2', '.65', 'empty'),
             ('inactive', '4', '6', '.8', '.9', 'inactive_band_demand'),
             ('mixed', '4', '6', '.5', '.9', 'mixed_band_applicability'),
             ('mixed_compressed', '4', '6', '-.2', '.9', 'mixed_band_applicability')]
    for name, kl, kh, pl, ph, expected_status in specs:
        Klo, Khi, plo, phi = map(D, (kl, kh, pl, ph))
        status, bounds, raw = interval(Klo, Khi, plo, phi, lower, upper)
        require('rectangle_classification', status == expected_status, name)
        boxes.append({'name': name, 'K': [kl, kh], 'p': [pl, ph], 'status': status,
                      'bounds': None if bounds is None else list(map(str, bounds)),
                      'raw_diagnostic': list(map(str, raw))})
        if status in ('nonempty', 'zero_only'):
            klo, khi = bounds
            grid = {}
            for i in range(11):
                K = Klo+(Khi-Klo)*D(i)/10
                for j in range(11):
                    p = plo+(phi-plo)*D(j)/10
                    seq = []
                    for t in range(5):
                        k = klo+(khi-klo)*D(t)/4
                        idx = add('rectangle_coverage', K, p, k, (lower, upper))
                        grid[i, j, t] = idx
                        seq.append(idx)
                    monotonic.append(seq)
            # Positive k roots decrease with K, increase with p; non-strict at k=0.
            for i in range(11):
                for t in range(5):
                    monotonic.append([grid[i, j, t] for j in range(11)])
            for j in range(11):
                for t in range(5):
                    monotonic.append([grid[i, j, t] for i in reversed(range(11))])
            if klo > 0:
                add('rectangle_lower_tightness', Khi, plo, klo*D('.99'), relation=('below', lower))
            add('rectangle_upper_tightness', Klo, phi,
                khi*D('1.01') if khi > 0 else D('.001'), relation=('above', upper))
        elif status == 'empty':
            require('empty_preserves_order', bounds[0] > bounds[1])
            # Neither violating endpoint is selected as a fallback.
            add('empty_upper_fails_lower', Khi, plo, bounds[1], relation=('below', lower))
            add('empty_lower_fails_upper', Klo, phi, bounds[0], relation=('above', upper))
        else:
            require('excluded_box_has_no_interval', bounds is None)
            # Raw [0,0] does not certify the all-box band.
            add('raw_zero_counterexample', Klo, phi, raw[1], relation=('above', upper))

    _, enclosing, _ = interval(D(4), D(6), D('.1'), D('.2'), lower, upper)
    add('outside_enclosure_counterexample', D(100), D('.1'), enclosing[0], relation=('below', lower))
    # This entire correlated family has k=1 and d=.5 by construction.
    plo, phi = D('.5')-f(D('.5')), D('.5')-f(D('.5'))/10
    cstatus, cbounds, _ = interval(D(1), D(10), plo, phi, lower, upper)
    require('correlated_box_empty', cstatus == 'empty')
    for i in range(25):
        K = D(1)+D(9)*D(i)/24
        p = D('.5')-f(D('.5'))/K
        add('correlated_family_feasible', K, p, D(1), (lower, upper), D('.5'))

    invalid = [(D(0), D(1), D(0), D(0), lower, upper),
               (D(2), D(1), D(0), D(0), lower, upper),
               (D(1), D(2), D(1), D(0), lower, upper),
               (D(1), D(2), D(0), D(0), D(0), upper),
               (D(1), D(2), D(0), D(0), lower, D(1)),
               (D('NaN'), D(2), D(0), D(0), lower, upper)]
    for values in invalid:
        try:
            interval(*values)
            rejected = False
        except ValueError:
            rejected = True
        require('invalid_reference_rejected', rejected)
    for p in (D('-.2'), D(0)):
        add('unprotected_gap', D(1), p, D(0), status='no_positive_equilibrium')
    add('invalid_bridge_K', D(0), D('.1'), D(1), status='invalid_input')
    add('invalid_bridge_k', D(1), D('.1'), D(-1), status='invalid_input')

    # Dropping decreasing f breaks uniqueness: f=d^2, K=k=1, p=.1.
    toy_roots = [(D(1)-D('.6').sqrt())/2, (D(1)+D('.6').sqrt())/2]
    for d in toy_roots:
        require('increasing_force_counterexample', abs(d-D('.1')-d*d) < D('1e-50'))
    require('increasing_force_two_roots', D(0) < toy_roots[0] < toy_roots[1] < D(1))

    requests = ''.join(' '.join(str(q[k]) for k in ('K', 'p', 'k', 'h'))+'\n' for q in queries)
    (args.output/'root-input.txt').write_text(requests)
    run = subprocess.run([str(args.output/'band_bridge')], input=requests, text=True, capture_output=True, check=True)
    (args.output/'root-output.txt').write_text(run.stdout)
    (args.output/'root-stderr.txt').write_text(run.stderr)
    rows = run.stdout.splitlines()
    if len(rows) != len(queries):
        raise RuntimeError('incomplete root bridge result')
    root_values = []
    for index, (q, row) in enumerate(zip(queries, rows)):
        parts = row.split()
        status = parts[0]
        require('bridge_status', status == q['status'], index)
        if status != 'ok':
            root_values.append(None)
            continue
        d, residual, width, F, curvature = map(float, parts[1:])
        root_values.append(d)
        require('root_residual', abs(residual) <= RESIDUAL_TOL, index, abs(residual))
        require('root_bracket', 0 <= width <= GAP_TOL, index, width)
        require('repulsive_force', F >= 0, index)
        require('positive_total_tangent', float(q['K'])+float(q['k'])*curvature > 0, index)
        if d < 1:
            dd = D(str(d))
            reference_curvature = -8*(3*dd*dd-1)*dd.ln()-14*dd*dd+12+2/(dd*dd)
            error = abs(curvature-float(reference_curvature))/(1+abs(float(reference_curvature)))
            require('physical_curvature_identity', error <= 1e-11, index, error)
            qgap = d*d
            positive_bound = (2*(1-qgap)*(7*qgap+1)/qgap if qgap >= 1/3
                              else 6*(1-qgap)*(qgap+3))
            require('curvature_positive_bound', curvature+1e-11*(1+abs(curvature)) >= positive_bound > 0, index)
        if q['band']:
            lo, hi = map(float, q['band'])
            violation = max(lo-d, d-hi, 0.)
            require(q['group'], violation <= GAP_TOL, index, violation)
        if q['expected'] is not None:
            error = abs(d-float(q['expected']))
            require('known_endpoint_or_manufactured_root', error <= GAP_TOL, index, error)
        if q['relation']:
            direction, limit = q['relation']
            require(q['group'], d < float(limit)-GAP_TOL if direction == 'below' else d > float(limit)+GAP_TOL, index)
        if index % 173 == 0:
            reference = decimal_root(q['K'], q['p'], q['k'])
            error = abs(d-float(reference))
            require('independent_decimal_root', error <= GAP_TOL, index, error)
    for sequence in monotonic:
        for left, right in zip(sequence, sequence[1:]):
            a, b = root_values[left], root_values[right]
            require('root_monotonicity', a is not None and b is not None and a <= b+GAP_TOL)

    # Save full query metadata locally; publish only aggregate checks and small fixtures.
    (args.output/'queries.json').write_text(json.dumps(queries, default=str, indent=2)+'\n')
    result = {'schema': 'polyfem.rb13.stage2', 'production_law_changed': False,
              'scope': 'isolated SPD affine-gap model; exact-arithmetic theorem; no application enclosure certified',
              'root_queries': len(queries), 'successful_roots': sum(v is not None for v in root_values),
              'checks': sum(g['checks'] for g in groups.values()),
              'failed_checks': sum(g['failed'] for g in groups.values()),
              'groups': groups, 'failures': failures, 'exact_fixtures': exact, 'boxes': boxes,
              'correlated_family': {'K': ['1', '10'], 'p_bounds': [str(plo), str(phi)],
                   'box_status': cstatus, 'box_bounds': list(map(str, cbounds)), 'k': '1', 'root': '.5'},
              'increasing_force_roots': list(map(str, toy_roots))}
    (args.output/'results.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(f"{result['checks']-result['failed_checks']}/{result['checks']} checks passed; "
          f"{result['successful_roots']}/{len(queries)} queries returned positive equilibria")
    for failure in failures:
        print(json.dumps(failure))
    return int(bool(failures))


if __name__ == '__main__':
    sys.exit(main())
