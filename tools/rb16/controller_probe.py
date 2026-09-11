#!/usr/bin/env python3
"""Isolated RB-16 spring timing comparison; see stage1-protocol.md."""
import argparse
import ctypes
from decimal import Decimal, localcontext
import hashlib
import itertools
import json
import math
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time

L, U = math.sqrt(.5), math.sqrt(.9)
TOL = 1e-10


class Probe:
    def __init__(self, library):
        self.lib = ctypes.CDLL(str(library))
        for name in ('value', 'force', 'curvature'):
            fn = getattr(self.lib, 'rb13_' + name)
            fn.argtypes = [ctypes.c_double, ctypes.c_double]
            fn.restype = ctypes.c_double
            setattr(self, name, lambda d, fn=fn: fn(d, 1.))
        self.checks = {}
        self.failures = []

    def check(self, group, condition, detail=None):
        self.checks[group] = self.checks.get(group, 0) + 1
        if not condition:
            self.failures.append({'group': group, 'detail': detail})

    def residual(self, d, K, p, k):
        spring = [a*(x-y) for a, x, y in zip(K, d, p)]
        force = [a*self.force(x) for a, x in zip(k, d)]
        g = [a-b for a, b in zip(spring, force)]
        return g, max(map(abs, g))/(1+max(map(abs, spring))+max(map(abs, force)))

    def energy(self, d, K, p, k):
        return sum(.5*a*(x-y)**2+c*self.value(x) for a, x, y, c in zip(K, d, p, k))

    def barrier_difference(self, old, new):
        # Same ordinary squared-distance potential, evaluated without cancellation.
        if old >= 1 or new >= 1:
            return self.value(new)-self.value(old)
        delta = new-old
        difference_of_squares = delta*(new+old)
        difference_of_fourths = difference_of_squares*(new*new+old*old-2)
        return -2*math.fsum([difference_of_fourths*math.log(old),
                            (new*new-1)**2*math.log1p(delta/old)])

    def energy_difference(self, old, new, K, p, k):
        return math.fsum(a*(x-y)*(z-x)+.5*a*(z-x)**2+c*self.barrier_difference(x, z)
                         for x, z, a, y, c in zip(old, new, K, p, k))

    def root(self, K, p, k):
        if p >= 1:
            return p
        lo, hi = 0., 1.
        for _ in range(100):
            mid = (lo+hi)/2
            if mid in (lo, hi):
                break
            if K*(mid-p)-k*self.force(mid) < 0:
                lo = mid
            else:
                hi = mid
        return (lo+hi)/2

    def stats(self, d, p):
        active = [x for x in d if x <= 1]
        mean2 = sum(x*x for x in active)/len(active) if active else None
        eligible = [i for i, y in enumerate(p) if y <= U]
        return {'minimum': min(d), 'median': statistics.median(d), 'maximum': max(d),
                'rms_active': math.sqrt(mean2) if mean2 is not None else None,
                'severity': min(mean2, 100*min(active)**2) if active else None,
                'active': len(active), 'eligible': len(eligible),
                'in_band': sum(L-1e-8 <= d[i] <= U+1e-8 for i in eligible)}

    def interval(self, K, p, variant):
        if variant == 'unavailable':
            return {'status': 'unavailable', 'K': None, 'p': None}
        width, dk, bias = {'exact': (0., 0., 0.), 'narrow': (.01, .1, 0.),
                           'wide': (.3, None, 0.), 'misleading': (0., 0., .6)}[variant]
        km, kp = (K*.1, K*10) if dk is None else (K*(1-dk), K*(1+dk))
        pm, pp = p+bias-width, p+bias+width
        result = {'K': K, 'p': p+bias, 'K_bounds': [km, kp], 'p_bounds': [pm, pp]}
        if pp > U:
            result['status'] = 'inactive' if pm > U else 'mixed'
            return result
        lo, hi = kp*max(L-pm, 0)/self.force(L), km*(U-pp)/self.force(U)
        result.update(lower=lo, upper=hi, status='empty' if lo > hi else 'zero_only' if hi == 0 else 'nonempty')
        return result

    def event(self, d, old, new, K, p, reason):
        delta = sum((b-a)*self.value(x) for x, a, b in zip(d, old, new))
        return {'reason': reason, 'd': d[:], 'old_k': old[:], 'new_k': new[:],
                'energy_change': delta, 'displacement_work': 0.,
                'old_residual': self.residual(d, K, p, old)[1],
                'new_residual': self.residual(d, K, p, new)[1],
                'history_invalidated': True}

    def timing_update(self, d, k, base, anchor, age, post=False):
        s = self.stats(d, [0.]*len(d))
        trim = k[0]/base[0]
        factor = 1.
        if s['severity'] is not None:
            if s['severity'] < .5 and (post or age >= 3):
                factor = min(256., max(2., math.sqrt(.5/s['severity'])))
                if not post:
                    factor = max(1., min(factor, anchor*256/trim))
            elif s['rms_active']**2 > .9 and (post or age >= 30):
                factor = .5
        newtrim = min(2**32, max(2**-32, trim*factor))
        return [a*newtrim for a in base]

    def solve(self, d, K, p, k, base, timing=False):
        d, k = d[:], k[:]
        events, path = [], []
        evaluations, age, anchor = 0, self.timing_age if timing else 0, k[0]/base[0]
        start_energy = sum(a*self.value(x) for a, x in zip(k, d))
        displacement_change = 0.
        status = 'iteration_limit'
        for it in range(101):
            g, res = self.residual(d, K, p, k)
            evaluations += 1
            if res <= TOL:
                status = 'converged'
                break
            if it == 100:
                break
            direction = [-v/(a+c*self.curvature(x)) for v, a, c, x in zip(g, K, k, d)]
            step = min([1.] + [-.99*x/v for x, v in zip(d, direction) if v < 0])
            slope = sum(a*b for a, b in zip(g, direction))
            frozen = tuple(k)
            for search in range(60):
                trial = [x+step*v for x, v in zip(d, direction)]
                evaluations += 1
                if min(trial) > 0 and self.energy_difference(d, trial, K, p, k) <= 1e-4*step*slope:
                    break
                step *= .5
            else:
                status = 'line_search_limit'
                break
            self.check('frozen_direction_search', tuple(k) == frozen)
            change = sum(a*(self.value(y)-self.value(x)) for a, x, y in zip(k, d, trial))
            displacement_change += change
            d = trial
            path.append({'d': d[:], 'k': k[:], 'residual_before_step': res,
                         'step': step, 'backtracks': search})
            age += 1
            if timing:
                new = self.timing_update(d, k, base, anchor, age)
                if new != k:
                    events.append(self.event(d, k, new, K, p, 'in_newton_trim'))
                    k, age = new, 0
        if timing:
            self.timing_age = age
        final_energy = sum(a*self.value(x) for a, x in zip(k, d))
        accounting = final_energy-start_energy-displacement_change-sum(e['energy_change'] for e in events)
        self.check('energy_state_identity', abs(accounting) <= 1e-9*(1+abs(final_energy)+abs(start_energy)), accounting)
        if status == 'converged':
            error = max(abs(x-self.root(a, y, c)) for x, a, y, c in zip(d, K, p, k))
            self.check('independent_root', error <= 1e-8, error)
        return {'d': d, 'k': k, 'status': status, 'iterations': len(path),
                'evaluations': evaluations, 'residual': self.residual(d, K, p, k)[1],
                'events': events, 'path': path, 'barrier_displacement_change': displacement_change,
                'energy_identity_error': accounting}

    def trajectory(self, fixture, subdivisions, policy, variant='exact', factor=2, window=1, hysteresis=0.):
        K, base = {'weak': ([1.], [.02]), 'strong': ([1.], [100.]),
                   'heterogeneous': ([1., 100.], [.02, 2.])}[fixture]
        knots = [1.2, -.5, .94, 1.2, -.3, 1.2]
        loads = [knots[0]]+[a+(b-a)*j/subdivisions for a, b in zip(knots, knots[1:]) for j in range(1, subdivisions+1)]
        d, k = [1.2]*len(K), base[:]
        endpoints, events, solves = [], [], []
        self.timing_age = 0
        started = time.perf_counter()
        for load_index, load in enumerate(loads):
            p = [load] if len(K) == 1 else [load, .5*load-.25]
            intervals = [self.interval(a, y, variant) for a, y in zip(K, p)]
            observed, corrections = [0]*len(K), 0
            while True:
                solved = self.solve(d, K, p, k, base, policy == 'in_newton')
                solves.append(solved)
                events.extend(dict(e, load_index=load_index) for e in solved['events'])
                d, k = solved['d'], solved['k']
                disposition = 'numerical_failure' if solved['status'] != 'converged' else 'frozen_endpoint'
                if solved['status'] != 'converged' or policy != 'outer':
                    break
                new = k[:]
                waiting = False
                statuses = []
                for i, interval in enumerate(intervals):
                    if interval['status'] != 'nonempty':
                        statuses.append(interval['status'])
                        continue
                    outside = d[i] < L-hysteresis or d[i] > U+hysteresis
                    observed[i] = observed[i]+1 if outside else 0
                    if not outside:
                        statuses.append('within_trigger')
                        continue
                    if observed[i] < window:
                        waiting = True
                        continue
                    lo, hi = interval['lower'], interval['upper']
                    target = math.sqrt(lo*hi) if lo > 0 else hi/2
                    new[i] = max(k[i]/factor, min(k[i]*factor, target))
                    statuses.append('correction' if new[i] != k[i] else 'prediction_no_progress')
                if new == k:
                    if waiting:
                        continue
                    disposition = '|'.join(sorted(set(statuses)))
                    break
                if corrections >= 12:
                    disposition = 'correction_budget'
                    break
                event = self.event(d, k, new, K, p, 'fixed_load_interval')
                events.append(dict(event, load_index=load_index))
                k, observed = new, [0]*len(K)
                corrections += 1
            prediction_errors = [abs(d[i]-self.root(interval['K'], interval['p'], k[i]))
                                 if interval['K'] is not None else None for i, interval in enumerate(intervals)]
            endpoint = {'load_index': load_index, 'p': p, 'd': d[:], 'k': k[:],
                        'residual': solved['residual'], 'numerical_status': solved['status'],
                        'controller_disposition': disposition, 'corrections': corrections,
                        'intervals': intervals, 'prediction_errors': prediction_errors,
                        'statistics': self.stats(d, p)}
            endpoints.append(endpoint)
            if solved['status'] != 'converged':
                break
            if policy == 'post_publication':
                new = self.timing_update(d, k, base, k[0]/base[0], 0, post=True)
                if new != k:
                    event = self.event(d, k, new, K, p, 'post_publication_next_load')
                    events.append(dict(event, load_index=load_index))
                    endpoint['next_state_residual_at_old_endpoint'] = event['new_residual']
                    endpoint['next_k'] = new[:]
                    k = new
        trajectory_energy_change = sum(a*self.value(x) for a, x in zip(k, d))
        trajectory_accounting_error = trajectory_energy_change - math.fsum(
            [s['barrier_displacement_change'] for s in solves] + [e['energy_change'] for e in events])
        self.check('trajectory_energy_state_identity', abs(trajectory_accounting_error) <=
                   1e-9*(1+abs(trajectory_energy_change)), trajectory_accounting_error)
        directions = [[] for _ in K]
        for event in events:
            for i, (a, b) in enumerate(zip(event['old_k'], event['new_k'])):
                if a != b:
                    directions[i].append(1 if b > a else -1)
        eligible = sum(e['statistics']['eligible'] for e in endpoints)
        inband = sum(e['statistics']['in_band'] for e in endpoints)
        return {'fixture': fixture, 'subdivisions': subdivisions, 'policy': policy,
                'variant': variant, 'factor': factor, 'window': window, 'hysteresis': hysteresis,
                'summary': {'completed': len(endpoints) == len(loads) and endpoints[-1]['numerical_status'] == 'converged',
                            'endpoints': len(endpoints), 'eligible_contact_endpoints': eligible,
                            'in_band': inband, 'occupancy': inband/eligible if eligible else None,
                            'newton_iterations': sum(s['iterations'] for s in solves),
                            'evaluations': sum(s['evaluations'] for s in solves), 'solves': len(solves),
                            'retunes': len(events),
                            'reversals': sum(sum(a != b for a, b in zip(seq, seq[1:])) for seq in directions),
                            'correction_budget_endpoints': sum(e['controller_disposition'] == 'correction_budget' for e in endpoints),
                            'minimum_gap': min(e['statistics']['minimum'] for e in endpoints),
                            'max_prediction_error': max((v for e in endpoints for v in e['prediction_errors'] if v is not None), default=None),
                            'coefficient_energy_change': sum(e['energy_change'] for e in events),
                            'trajectory_energy_identity_error': trajectory_accounting_error,
                            'barrier_displacement_change': sum(s['barrier_displacement_change'] for s in solves),
                            'wall_seconds': time.perf_counter()-started},
                'endpoints': endpoints, 'events': events, 'solves': solves}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ipc-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out, ipc = args.output.resolve(), args.ipc_source.resolve()
    out.mkdir(parents=True, exist_ok=False)
    here = Path(__file__).resolve().parent
    bridge = here.parent/'rb13/scope_bridge.cpp'
    sources = [bridge, ipc/'src/ipc/barrier/barrier.cpp']
    library = out/'barrier.dylib'
    command = ['/usr/bin/c++', '-std=c++17', '-O2', '-shared', '-fPIC', '-I'+str(ipc/'src'), *map(str, sources), '-o', str(library)]
    prov = {'command': command, 'argv': sys.argv, 'platform': platform.platform(), 'python': sys.version,
            'ipc_head': subprocess.check_output(['git', '-C', str(ipc), 'rev-parse', 'HEAD'], text=True).strip(),
            'polyfem_head': subprocess.check_output(['git', '-C', str(here), 'rev-parse', 'HEAD'], text=True).strip(),
            'hashes': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources+[Path(__file__), here/'stage1-protocol.md', ipc/'src/ipc/barrier/barrier.hpp']}}
    (out/'provenance.json').write_text(json.dumps(prov, indent=2)+'\n')
    built = subprocess.run(command, capture_output=True, text=True)
    (out/'compile.log').write_text(built.stdout+built.stderr)
    built.check_returncode()
    probe = Probe(library)
    # Independent high-precision oracle for stable energy differences.
    with localcontext() as context:
        context.prec = 60
        for old, new in ((.4, .8), (.8, .8000000001), (.95, .950000000001)):
            x, y = Decimal(old), Decimal(new)
            exact = -2*((y*y-1)**2*y.ln()-(x*x-1)**2*x.ln())
            actual = probe.barrier_difference(old, new)
            probe.check('stable_energy_difference', abs(actual-float(exact)) <= 1e-12*abs(float(exact)))
    cases = []
    for fixture, subdivisions in itertools.product(('weak', 'strong', 'heterogeneous'), (4, 8, 16)):
        for policy in ('in_newton', 'post_publication'):
            cases.append(probe.trajectory(fixture, subdivisions, policy))
        for factor, window, margin in itertools.product((2, 4), (1, 2), (0., .01)):
            cases.append(probe.trajectory(fixture, subdivisions, 'outer', factor=factor, window=window, hysteresis=margin))
        for variant in ('narrow', 'wide', 'misleading', 'unavailable'):
            cases.append(probe.trajectory(fixture, subdivisions, 'outer', variant=variant))
    for case in cases:
        for event in case['events']:
            probe.check('event_state', event['displacement_work'] == 0 and event['history_invalidated'])
            if case['policy'] == 'outer':
                probe.check('bounded_update', all(1/case['factor']-1e-14 <= b/a <= case['factor']+1e-14
                            for a, b in zip(event['old_k'], event['new_k'])))
                endpoint = case['endpoints'][event['load_index']]
                for i, interval in enumerate(endpoint['intervals']):
                    if interval['status'] != 'nonempty':
                        probe.check('unavailable_protection_retained', event['old_k'][i] == event['new_k'][i])
        for endpoint in case['endpoints']:
            probe.check('positive_endpoint', min(endpoint['d']) > 0 and min(endpoint['k']) > 0)
            if endpoint['numerical_status'] == 'converged':
                probe.check('actual_coefficient_endpoint', endpoint['residual'] <= TOL)
        if case['variant'] == 'unavailable':
            probe.check('unavailable_protection_retained', not case['events'])
    # Standalone controls do not depend on a cycle completing successfully.
    mean_control = probe.stats([.2, .99], [-.5, -.5])
    probe.check('mean_not_per_contact', .5 <= mean_control['rms_active']**2 <= .9 and mean_control['in_band'] == 0)
    box_controls = []
    for variant in ('exact', 'narrow'):
        interval = probe.interval(1., -.5, variant)
        k = math.sqrt(interval['lower']*interval['upper'])
        roots = [probe.root(K, p, k) for K, p in itertools.product(interval['K_bounds'], interval['p_bounds'])]
        probe.check('enclosing_box', all(L <= d <= U for d in roots))
        box_controls.append({'variant': variant, 'interval': interval, 'k': k, 'roots': roots})
    unavailable = probe.interval(1., -.5, 'unavailable')
    empty = probe.interval(1., .5, 'wide')
    probe.check('explicit_interval_states', unavailable['status'] == 'unavailable' and empty['status'] == 'empty', empty)
    wrong = probe.interval(1., -.5, 'misleading')
    kwrong = math.sqrt(wrong['lower']*wrong['upper'])
    wrongroot = probe.root(1., -.5, kwrong)
    # A deliberately larger bias offers an unmistakable inactive-demand failure.
    missed = probe.interval(1., .4, 'misleading')
    weakroot = probe.root(1., .4, .0001)
    probe.check('misleading_predictor', missed['status'] == 'inactive' and weakroot < L)
    d = probe.root(1., -.5, 1.)
    event = probe.event([d], [1.], [2.], [1.], [-.5], 'endpoint_identity_control')
    probe.check('post_publication_not_equilibrium', event['old_residual'] < TOL and event['new_residual'] > .01)
    n, a, b = 4096, .4, .8
    integral = (b-a)/(3*n)*sum((1 if i in (0, n) else 4 if i % 2 else 2)*probe.force(a+(b-a)*i/n) for i in range(n+1))
    quadrature_error = abs(integral+probe.value(b)-probe.value(a))
    probe.check('independent_path_work', quadrature_error < 1e-10, quadrature_error)
    probe.check('separation_recontact', any(e['statistics']['active'] == 0 for c in cases for e in c['endpoints']) and any(e['statistics']['active'] > 0 and e['load_index'] > 0 for c in cases for e in c['endpoints']))
    # Store every failed or incomplete trajectory, including its final iterate.
    compact = [{k: v for k, v in c.items() if k not in ('solves', 'events', 'endpoints')} for c in cases]
    results = {'scope': 'spring timing controls; not production controller execution or FEM validation',
               'checks': probe.checks, 'check_count': sum(probe.checks.values()), 'failures': probe.failures,
               'case_count': len(cases), 'completed': sum(c['summary']['completed'] for c in cases),
               'cases': compact, 'controls': {'mean_counterexample': mean_control, 'boxes': box_controls,
               'empty': empty, 'misleading_root': wrongroot, 'misleading_inactive': missed,
               'weak_unprotected_demand_root': weakroot, 'post_publication_event': event,
               'quadrature_error': quadrature_error}}
    (out/'trajectories.json').write_text(json.dumps(cases, indent=2, allow_nan=False)+'\n')
    (out/'results.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'checks': results['check_count'], 'failures': probe.failures, 'cases': len(cases), 'completed': results['completed']}))
    return bool(probe.failures)


if __name__ == '__main__':
    sys.exit(main())
