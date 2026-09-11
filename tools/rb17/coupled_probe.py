#!/usr/bin/env python3
"""RB-17 approved scalar controller, exact coupled mechanics, isolated diagnostics."""
import argparse
import ctypes
import hashlib
import itertools
import json
import math
from pathlib import Path
import platform
import resource
import subprocess
import sys
import time

import numpy as np


class Probe:
    def __init__(self, library, manifest):
        self.lib = ctypes.CDLL(str(library))
        for name in ('value', 'force', 'curvature'):
            fn = getattr(self.lib, 'rb13_' + name)
            fn.argtypes = [ctypes.c_double, ctypes.c_double]
            fn.restype = ctypes.c_double
            setattr(self, name, lambda d, h, fn=fn: np.array([fn(float(x), h) for x in np.atleast_1d(d)]))
        self.m = manifest
        self.checks, self.failures = {}, []

    def check(self, name, ok, detail=None):
        self.checks[name] = self.checks.get(name, 0) + 1
        if not ok:
            self.failures.append({'check': name, 'detail': detail})

    def residual(self, d, A, p, k, h):
        noncontact, contact = A @ (d-p), k*self.force(d, h)
        g = noncontact-contact
        scale = 1/h + np.linalg.norm(noncontact) + np.linalg.norm(contact)
        return g, float(np.linalg.norm(g)/scale)

    def energy(self, d, A, p, k, h):
        return float(.5*(d-p) @ A @ (d-p) + k @ self.value(d, h))

    def barrier_delta(self, old, new, h):
        ans = []
        for a, b in zip(old/h, new/h):
            if a >= 1 or b >= 1:
                ans.append(float((self.value([b*h], h)-self.value([a*h], h))[0]))
            else:
                t = b-a
                ds = t*(a+b)
                ans.append(-2*h**4*math.fsum([ds*(b*b+a*a-2)*math.log(a),
                                             (b*b-1)**2*math.log1p(t/a)]))
        return np.array(ans)

    def solve(self, initial, A, p, k, h, arithmetic):
        d = initial.copy()
        path, status = [], 'iteration_limit'
        evaluations = 0
        for it in range(self.m['newton_limit']+1):
            g, res = self.residual(d, A, p, k, h)
            evaluations += 1
            if res <= self.m['residual_tolerance']:
                status = 'converged'
                break
            if it == self.m['newton_limit']:
                break
            B = A+np.diag(k*self.curvature(d, h))
            direction = np.linalg.solve(B, -g)
            slope = float(g @ direction)
            self.check('descent', slope < 0 and math.isfinite(slope))
            step = min([1.] + [-.99*x/v for x, v in zip(d, direction) if v < 0])
            frozen = k.copy()
            e = self.energy(d, A, p, k, h)
            for search in range(self.m['backtrack_limit']):
                trial = d+step*direction
                evaluations += 1
                if arithmetic == 'direct':
                    delta = self.energy(trial, A, p, k, h)-e
                else:
                    v = trial-d
                    delta = float(math.fsum([float(v @ A @ (d-p)), float(.5*v @ A @ v),
                                            float(k @ self.barrier_delta(d, trial, h))]))
                if min(trial) > 0 and delta <= self.m['armijo']*step*slope:
                    break
                step *= .5
            else:
                status = 'line_search_limit'
                break
            self.check('frozen_search', np.array_equal(k, frozen))
            path.append({'gap': trial.tolist(), 'residual_before': res, 'step': step,
                         'backtracks': search, 'energy_delta': delta})
            d = trial
        return {'gap': d.tolist(), 'status': status, 'residual': self.residual(d, A, p, k, h)[1],
                'iterations': len(path), 'evaluations': evaluations, 'path': path}

    def oracle(self, initial, A, p, k, h):
        # Independent cyclic coordinate minimization. Each coordinate derivative
        # is strictly increasing, bracketed and solved using bisection only.
        d = initial.copy()
        res = None
        for cycle in range(20000):
            for i in range(len(d)):
                cross = float(A[i] @ (d-p)-A[i, i]*(d[i]-p[i]))
                free = p[i]-cross/A[i, i]
                lo, hi = 0., max(h, free)
                for _ in range(90):
                    mid = .5*(lo+hi)
                    if mid == lo or mid == hi:
                        break
                    v = A[i, i]*(mid-p[i])+cross-k[i]*self.force([mid], h)[0]
                    if v < 0:
                        lo = mid
                    else:
                        hi = mid
                d[i] = .5*(lo+hi)
            res = self.residual(d, A, p, k, h)[1]
            if res <= 1e-12:
                break
        return {'gap': d.tolist(), 'residual': res, 'cycles': cycle+1,
                'converged': res <= 1e-12}

    def feasible_reference(self, W, p, h):
        L, U = h*math.sqrt(.5), h*math.sqrt(.9)
        # Exactly six halfplanes for two forces, G F <= b.
        G = np.vstack([W, -W, -np.eye(2)])
        b = np.r_[np.full(2, U)-p, p-np.full(2, L), [0., 0.]]
        vertices = []
        for ids in itertools.combinations(range(6), 2):
            B = G[list(ids)]
            if abs(np.linalg.det(B)) < 1e-14*np.linalg.norm(B)**2:
                continue
            force = np.linalg.solve(B, b[list(ids)])
            if np.max(G @ force-b) <= 1e-12*h:
                vertices.append(force)
        if not vertices:
            return {'feasible': False, 'vertices': []}
        F = np.mean(vertices, axis=0)
        d = p+W @ F
        return {'feasible': True, 'vertices': [v.tolist() for v in vertices],
                'force': F.tolist(), 'gap': d.tolist()}

    def run(self, case, factor, s, order, arithmetic):
        start = time.perf_counter()
        H = np.array(case['H'], float)/s**2
        J = np.array(case['J'], float)
        p = s*np.array(case['p'], float)
        if order == 'reverse':
            J, p = J[::-1].copy(), p[::-1].copy()
        Z = np.linalg.solve(H, J.T)
        W = J @ Z
        A = np.linalg.solve(W, np.eye(2))
        self.check('spd', min(np.linalg.eigvalsh(H)) > 0 and min(np.linalg.eigvalsh(W)) > 0)
        d = s*np.array(self.m['initial_gap'])
        # One full-system, normalized local direction per abstract parent.
        k = np.array([row @ H @ row/(row @ row)/s**2 for row in J])
        initial_k = k.copy()
        L, U = s*math.sqrt(.5), s*math.sqrt(.9)
        solves, events, observations = [], [], []
        outcome = 'unresolved_correction_budget'
        estimation_seconds = 0.
        for outer in range(self.m['max_corrections']+1):
            solved = self.solve(d, A, p, k, s, arithmetic)
            solves.append(solved)
            d = np.array(solved['gap'])
            if solved['status'] != 'converged':
                outcome = 'numerical_failure'
                break
            est_start = time.perf_counter()
            # Reconstruct full displacement, residual and fresh shared estimate.
            u = Z @ np.linalg.solve(W, d-p)
            r = H @ u
            chol = np.linalg.cholesky(H)
            response = np.linalg.solve(chol.T, np.linalg.solve(chol, np.column_stack([J.T, r])))
            fresh_W = J @ response[:, :2]
            prediction = d-J @ response[:, 2]
            K = 1/np.diag(fresh_W)
            estimation_seconds += time.perf_counter()-est_start
            self.check('exact_shared_prediction', np.max(np.abs(prediction-p))/s <= 1e-12)
            self.check('full_equilibrium', np.linalg.norm(H @ u-J.T @ (k*self.force(d, s)))/(1/s+np.linalg.norm(H @ u)) < 1e-8)
            new = k.copy()
            applicable = prediction <= U
            inside = np.logical_and(d >= L, d <= U)
            for i in range(2):
                if not applicable[i] or inside[i]:
                    continue
                lo = K[i]*max(L-prediction[i], 0)/self.force([L], s)[0]
                hi = K[i]*(U-prediction[i])/self.force([U], s)[0]
                if hi <= 0 or lo > hi:
                    continue
                target = math.sqrt(lo*hi) if lo > 0 else hi/2
                new[i] = np.clip(target, k[i]/factor, k[i]*factor)
            observations.append({'gap': d.tolist(), 'k': k.tolist(), 'p': prediction.tolist(),
                                 'K': K.tolist(), 'inside': inside.tolist(), 'applicable': applicable.tolist(),
                                 'proposed_k': new.tolist()})
            if np.all(np.logical_or(inside, ~applicable)):
                outcome = 'band_satisfied'
                break
            # Equality within arithmetic noise is a fixed-point diagnostic; it
            # never changes acceptance or relabels a violation as success.
            if np.max(np.abs(new/k-1)) <= 1e-13:
                outcome = 'unresolved_coefficient_fixed_point'
                break
            if outer == self.m['max_corrections']:
                break
            delta = float((new-k) @ self.value(d, s))
            self.check('event_energy', abs(self.energy(d, A, p, new, s)-self.energy(d, A, p, k, s)-delta) < 1e-10*(1+abs(delta)))
            self.check('bounded_update', min(new/k) >= 1/factor-1e-14 and max(new/k) <= factor+1e-14)
            events.append({'gap': d.tolist(), 'old_k': k.tolist(), 'new_k': new.tolist(),
                           'energy_change': delta, 'displacement_work': 0., 'history_reset': True})
            k = new
        oracle = self.oracle(d, A, p, k, s)
        self.check('independent_equilibrium', oracle['converged'], case['id'])
        if solved['status'] == 'converged':
            self.check('independent_gap_agreement', np.max(np.abs(np.array(oracle['gap'])-d))/s < 1e-8)
        feasible = self.feasible_reference(W, p, s)
        if feasible['feasible']:
            refd, F = np.array(feasible['gap']), np.array(feasible['force'])
            self.check('positive_feasible_reference', min(F) > 0)
            refk = F/self.force(refd, s)
            verified = self.oracle(np.full(2, .8*s), A, p, refk, s)
            self.check('feasible_reference_verified', verified['converged'] and np.max(np.abs(np.array(verified['gap'])-refd))/s < 1e-8)
            feasible.update(k=refk.tolist(), independent=verified)
        else:
            # Only the declared positive incompatible case is expected empty.
            self.check('infeasible_expected', case['id'] == 'positive_incompatible')
            a, b = (0, 1) if order == 'forward' else (1, 0)
            bound = p[a]+W[a, b]/W[b, b]*(L-p[b])
            self.check('analytic_infeasibility', W[a, b] > 0 and bound > U)
            feasible['analytic_lower_gap'] = float(bound)
            feasible['upper_band'] = U
        # FD control at a fixed interior state, away from support transitions.
        testd = np.array([.55, .65])*s
        eps = 1e-5*s
        exact_g, _ = self.residual(testd, A, p, k, s)
        exact_h = A+np.diag(k*self.curvature(testd, s))
        for i in range(2):
            v = np.zeros(2); v[i] = eps
            gd = (self.energy(testd+v, A, p, k, s)-self.energy(testd-v, A, p, k, s))/(2*eps)
            hd = (self.residual(testd+v, A, p, k, s)[0]-self.residual(testd-v, A, p, k, s)[0])/(2*eps)
            self.check('gradient_fd', abs(gd-exact_g[i])/(1/s+abs(exact_g[i])) < 1e-7)
            self.check('hessian_fd', np.linalg.norm(hd-exact_h[:, i])/(1/s**2+np.linalg.norm(exact_h[:, i])) < 1e-7)
        unorder = slice(None, None, -1) if order == 'reverse' else slice(None)
        summary = {'case': case['id'], 'holdout': case.get('holdout', False), 'factor': factor,
                   'length_scale': s, 'order': order, 'arithmetic': arithmetic, 'outcome': outcome,
                   'gap_normalized': (d[unorder]/s).tolist(), 'k_normalized': (k[unorder]*s**4).tolist(),
                   'initial_k_normalized': (initial_k[unorder]*s**4).tolist(),
                   'residual': solved['residual'], 'numerical_status': solved['status'],
                   'solves': len(solves), 'iterations': sum(x['iterations'] for x in solves),
                   'events': len(events), 'band_feasible': feasible['feasible'],
                   'in_band': int(sum(np.logical_and(d >= L-1e-8*s, d <= U+1e-8*s))),
                   'oracle_gap_normalized': (np.array(oracle['gap'])[unorder]/s).tolist(),
                   'estimation_seconds': estimation_seconds, 'wall_seconds': time.perf_counter()-start}
        return {'summary': summary, 'W': W.tolist(), 'observations': observations, 'solves': solves,
                'events': events, 'oracle': oracle, 'feasible_reference': feasible}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ipc-source', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, default=Path(__file__).with_name('coupled-cases.json'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out, ipc = args.output.resolve(), args.ipc_source.resolve()
    out.mkdir(parents=True, exist_ok=False)
    manifest = json.loads(args.manifest.read_text())
    here = Path(__file__).resolve().parent
    sources = [here.parent/'rb13/scope_bridge.cpp', ipc/'src/ipc/barrier/barrier.cpp']
    library = out/'barrier.dylib'
    command = ['/usr/bin/c++', '-std=c++17', '-O2', '-shared', '-fPIC', '-I'+str(ipc/'src'), *map(str, sources), '-o', str(library)]
    prov = {'command': command, 'argv': sys.argv, 'platform': platform.platform(), 'python': sys.version,
            'numpy': np.__version__, 'manifest': manifest,
            'ipc_head': subprocess.check_output(['git', '-C', str(ipc), 'rev-parse', 'HEAD'], text=True).strip(),
            'polyfem_head': subprocess.check_output(['git', '-C', str(here), 'rev-parse', 'HEAD'], text=True).strip(),
            'hashes': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources+[Path(__file__), args.manifest, here/'coupled-protocol.md', ipc/'src/ipc/barrier/barrier.hpp']}}
    (out/'provenance.json').write_text(json.dumps(prov, indent=2)+'\n')
    built = subprocess.run(command, capture_output=True, text=True)
    (out/'compile.log').write_text(built.stdout+built.stderr)
    built.check_returncode()
    probe = Probe(library, manifest)
    records = []
    for case, factor, scale, order, arithmetic in itertools.product(manifest['cases'], manifest['factors'], manifest['length_scales'], manifest['parent_orders'], manifest['arithmetic']):
        result = probe.run(case, factor, scale, order, arithmetic)
        records.append(result)
        (out/('case-%03d.json' % len(records))).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    # Compare only independently converged corresponding outcomes; retained
    # arithmetic failures are explicitly excluded, not silently called covariant.
    covariance = []
    for case, factor, arithmetic in itertools.product(manifest['cases'], manifest['factors'], manifest['arithmetic']):
        group = [r['summary'] for r in records if r['summary']['case'] == case['id'] and r['summary']['factor'] == factor and r['summary']['arithmetic'] == arithmetic]
        base = next(r for r in group if r['length_scale'] == 1 and r['order'] == 'forward')
        for r in group:
            available = r['numerical_status'] == base['numerical_status'] == 'converged'
            err = float(np.max(np.abs(np.array(r['gap_normalized'])-base['gap_normalized']))) if available else None
            covariance.append({'case': case['id'], 'factor': factor, 'arithmetic': arithmetic,
                               'scale': r['length_scale'], 'order': r['order'], 'available': available, 'gap_error': err})
            if available:
                probe.check('unit_order_covariance', err < 1e-8 and r['outcome'] == base['outcome'], covariance[-1])
    results = {'scope': manifest['scope'], 'checks': probe.checks, 'check_count': sum(probe.checks.values()),
               'failures': probe.failures, 'case_count': len(records), 'cases': [r['summary'] for r in records],
               'covariance': covariance, 'peak_rss_native': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
               'peak_rss_units': 'bytes on macOS; kilobytes on Linux', 'source_hashes': prov['hashes']}
    (out/'results.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'checks': results['check_count'], 'failures': probe.failures, 'cases': len(records)}))
    return bool(probe.failures)


if __name__ == '__main__':
    sys.exit(main())
