#!/usr/bin/env python3
"""RB-13 stage 3. See stage3-protocol.md for fixtures and fixed tolerances."""

import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import sys

sys.dont_write_bytecode = True
from reference_probe import solve_spd, dot


def mv(H, x):
    return [dot(row, x) for row in H]


def transpose(A):
    return list(map(list, zip(*A)))


def norm(x):
    return math.sqrt(dot(x, x))


def compliance(H, J):
    n = len(J)
    if len(H) != n or any(len(row) != n for row in H):
        raise ValueError('matrix/J dimensions')
    if not all(math.isfinite(v) for row in H for v in row) or not all(map(math.isfinite, J)):
        raise ValueError('nonfinite reference data')
    v = solve_spd(H, J)
    C = dot(J, v)
    if not math.isfinite(C) or C <= 0:
        raise ValueError('no finite positive gap compliance')
    return 1/C, v


class Barrier:
    def __init__(self, library):
        self.lib = ctypes.CDLL(str(library))
        self.root_records = []
        for name in ('value', 'force', 'curvature'):
            fn = getattr(self.lib, 'rb13_'+name)
            fn.argtypes = [ctypes.c_double, ctypes.c_double]
            fn.restype = ctypes.c_double
            setattr(self, name, fn)

    def root(self, g, k, h=1.):
        if not k > 0 or not g(h) > 0:
            raise ValueError('scalar fixture needs positive k and bracketed root')
        lo, hi = 0., 1.
        for _ in range(120):
            mid = (lo+hi)/2
            if mid in (lo, hi):
                break
            d = mid*h
            if g(d)-k*self.force(d, h) < 0:
                lo = mid
            else:
                hi = mid
        d = (lo+hi)*h/2
        F = k*self.force(d, h)
        residual = abs(g(d)-F)/(abs(g(h))+abs(F))
        self.root_records.append({'h': h, 'k': k, 'root': d, 'force': F,
                                  'normalized_residual': residual, 'relative_bracket_width': hi-lo})
        return d

    def coupled(self, H, J, p, k):
        n, m = len(H), len(J)
        u, history = [0.]*n, []

        def state(x):
            d = [a+b for a, b in zip(p, mv(J, x))]
            if min(d) <= 0:
                return None
            energy = .5*dot(x, mv(H, x))+sum(k[i]*self.value(d[i], 1.) for i in range(m))
            return d, energy

        for it in range(80):
            d, energy = state(u)
            F = [k[i]*self.force(d[i], 1.) for i in range(m)]
            grad = [a-b for a, b in zip(mv(H, u), mv(transpose(J), F))]
            residual = norm(grad)/(1+norm(mv(H, u))+norm(F))
            history.append({'iteration': it, 'gaps': d, 'energy': energy, 'residual': residual})
            if residual <= 1e-10:
                return {'u': u, 'gaps': d, 'forces': F, 'residual': residual, 'history': history}
            tangent = [[H[a][b]+sum(k[i]*self.curvature(d[i], 1.)*J[i][a]*J[i][b]
                                    for i in range(m)) for b in range(n)] for a in range(n)]
            direction = solve_spd(tangent, [-v for v in grad])
            step = 1.
            for _ in range(60):
                trial = [a+step*b for a, b in zip(u, direction)]
                trial_state = state(trial)
                if trial_state is not None and trial_state[1] <= energy+1e-4*step*dot(grad, direction):
                    u = trial
                    break
                step *= .5
            else:
                raise RuntimeError('coupled reference line search exhausted')
        raise RuntimeError('coupled reference iteration limit')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ipc-source', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--cxx', default='/usr/bin/c++')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    here, ipc = Path(__file__).resolve().parent, args.ipc_source.resolve()
    library = args.output.resolve()/'scope_bridge.dylib'
    sources = [here/'scope_bridge.cpp', ipc/'src/ipc/barrier/barrier.cpp']
    command = [args.cxx, '-std=c++17', '-O2', '-shared', '-fPIC', '-I'+str(ipc/'src'),
               *map(str, sources), '-o', str(library)]
    provenance = {'command': command, 'python': sys.version, 'platform': platform.platform(),
                  'ipc_head': subprocess.check_output(['git', '-C', str(ipc), 'rev-parse', 'HEAD'], text=True).strip(),
                  'hashes': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources+
                             [Path(__file__), here/'reference_probe.py', here/'stage3-protocol.md', ipc/'src/ipc/barrier/barrier.hpp']}}
    (args.output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    build = subprocess.run(command, capture_output=True, text=True)
    (args.output/'compile.log').write_text(build.stdout+build.stderr)
    build.check_returncode()
    b = Barrier(library)
    checks, failures = {}, []

    def require(group, condition, detail=None, error=None):
        row = checks.setdefault(group, {'checks': 0, 'failed': 0, 'max_error': None})
        row['checks'] += 1
        if error is not None:
            row['max_error'] = max(row['max_error'] or 0., float(error))
        if not condition:
            row['failed'] += 1
            failures.append({'group': group, 'detail': detail})

    def close(group, actual, expected, tol=1e-10, absolute=False):
        err = abs(actual-expected)/(1 if absolute else 1+abs(expected))
        require(group, math.isfinite(err) and err <= tol, [actual, expected], err)

    springs = []
    for A, B in [(1., 1.), (2., 8.), (1., 100.), (10., 100.), (1., 1e3), (1., 1e6)]:
        H, J = [[A, 0.], [0., B]], [-1., 1.]
        K, v = compliance(H, J)
        close('series_stiffness', K, A*B/(A+B))
        F, p, target = K*.3, .2, .5
        k = F/b.force(target, 1.)
        root = b.root(lambda d: K*(d-p), k)
        close('series_root', root, target, 2e-11, absolute=True)
        u = [z*F for z in v]
        close('series_gap_displacement', dot(J, u), .3)
        for actual, expected in zip(mv(H, u), [z*F for z in J]):
            close('series_full_force_balance', actual, expected)
        close('support_reaction_balance', sum(-z for z in mv(H, u)), 0.)
        r = dot(J, mv(H, J))/dot(J, J)
        Kdir = r/dot(J, J)
        require('restricted_vs_relaxed', Kdir >= K-1e-10)
        close('rayleigh_source_expression', r, (A+B)/2)
        springs.append({'A': A, 'B': B, 'K_eff': K, 'rayleigh_r': r, 'K_restricted_gap': Kdir,
                        'force': F, 'k': k, 'root': root, 'u': u})
    rigid = []
    for ratio in (1., 1e2, 1e4, 1e6):
        K, _ = compliance([[2., 0.], [0., 2*ratio]], [-1., 1.])
        error = (2-K)/2
        close('rigid_limit_error', error, 1/(1+ratio))
        rigid.append({'B_over_A': ratio, 'relative_error_to_rigid': error})
    prescribed_K, _ = compliance([[2.]], [-1.])
    close('explicit_prescribed_endpoint', prescribed_K, 2.)
    close('zero_padded_obstacle_rayleigh', dot([-1., 1.], mv([[2., 0.], [0., 0.]], [-1., 1.]))/2, 1.)

    def dynamic(dt, speed, L=1., Fscale=1., T=1.):
        stiffness = [4*Fscale/L, 12*Fscale/L]
        mass = [Fscale*T*T/L, 3*Fscale*T*T/L]
        dtc = dt*T
        velocity = [speed/2*L/T, -speed/2*L/T]
        predictor = [dtc*v for v in velocity]
        diag = [a+m/dtc**2 for a, m in zip(stiffness, mass)]
        rhs = [m/dtc**2*x for m, x in zip(mass, predictor)]
        free = [r/H for r, H in zip(rhs, diag)]
        p = .6*L+free[1]-free[0]
        K = diag[0]*diag[1]/sum(diag)
        return {'stiffness': stiffness, 'mass': mass, 'dt': dtc, 'velocity': velocity,
                'predictor': predictor, 'H': diag, 'rhs': rhs, 'free': free, 'p': p, 'K': K}

    dynamics = []
    for dt in (.1, .2, .4):
        fixed = dynamic(dt, 2.)
        fixed_k = fixed['K']*(.5-fixed['p'])/b.force(.5, 1.)
        for speed in (0., 1., 2., 4.):
            case = dynamic(dt, speed)
            close('velocity_independent_tangent', case['K'], fixed['K'])
            close('euler_free_gap', case['p'], .6-speed*dt/(1+4*dt*dt))
            demand = max(case['K']*(.5-case['p']), 0.)
            case.update({'speed': speed, 'demand': demand, 'k_estimate': demand/b.force(.5, 1.)})
            case['fixed_reference_k'] = fixed_k
            case['root_with_fixed_reference_k'] = b.root(lambda d: case['K']*(d-case['p']), fixed_k)
            if demand > 0:
                d = b.root(lambda d: case['K']*(d-case['p']), case['k_estimate'])
                close('euler_target_root', d, .5, 2e-11, absolute=True)
                Fb = case['k_estimate']*b.force(d, 1.)
                u = [(r+s*Fb)/H for r, s, H in zip(case['rhs'], [-1, 1], case['H'])]
                close('euler_reconstructed_gap', .6+u[1]-u[0], d)
                for H, x, r, sign in zip(case['H'], u, case['rhs'], [-1, 1]):
                    close('euler_full_residual', H*x-r-sign*Fb, 0.)
                case['target_root'] = d
            else:
                case['target_root'] = None
                case['status'] = 'no_repulsive_target_demand; no protection decision'
            dynamics.append(case)

    base = dynamic(.2, 2.)
    kbase = base['K']*(.5-base['p'])/b.force(.5, 1.)
    units = []
    base_energy = None
    # Base occurs first to define the independently scaled energy control.
    scales = [(L, F, T) for L in (1., .001, 1000.) for F in (1., .01, 100.) for T in (1., .1, 10.)]
    for L, Fs, T in scales:
        case = dynamic(.2, 2., L, Fs, T)
        k = kbase*Fs/L**3
        h = L
        d = b.root(lambda gap: case['K']*(gap-case['p']), k, h)
        force = k*b.force(d, h)
        u = [(r+s*force)/H for r, s, H in zip(case['rhs'], [-1, 1], case['H'])]
        energy = sum(.5*a*x*x+.5*m/case['dt']**2*(x-pred)**2
                     for a, m, x, pred in zip(case['stiffness'], case['mass'], u, case['predictor']))+k*b.value(d, h)
        if base_energy is None:
            base_energy, base_u = energy, u
        close('unit_root', d/L, .5, 2e-11, absolute=True)
        close('unit_free_gap', case['p']/L, base['p'])
        close('unit_stiffness', case['K']*L/Fs, base['K'])
        close('unit_force', force/Fs, kbase*b.force(.5, 1.))
        for actual, expected in zip(u, base_u):
            close('unit_displacement', actual/L, expected)
        close('unit_physical_energy', energy/(Fs*L), base_energy)
        close('unit_weighted_objective', energy*case['dt']**2/(Fs*L*T*T), base_energy*.2**2)
        residual = abs(case['K']*(d-case['p'])-force)/(case['K']*h+abs(force))
        require('unit_root_residual', residual <= 2e-10, error=residual)
        units.append({'length_scale': L, 'force_scale': Fs, 'time_scale': T, 'root': d,
                      'force': force, 'k': k, 'physical_incremental_energy': energy})

    nonlinear, error_sequences = [], []
    for alpha in (0., 10., 100.):
        errors = []
        for anchor in (.1, .3, .45, .5):
            g = lambda d: 6*(d-.1)+alpha*(d-.1)**3
            H = 6+3*alpha*(anchor-.1)**2
            p = anchor-g(anchor)/H
            k = H*(.5-p)/b.force(.5, 1.)
            actual = b.root(g, k)
            exact_k = g(.5)/b.force(.5, 1.)
            exact_root = b.root(g, exact_k)
            close('nonlinear_exact_target', exact_root, .5, 2e-11, absolute=True)
            residual = abs(g(actual)-k*b.force(actual, 1.))/(6+abs(g(actual)))
            require('nonlinear_actual_residual', residual <= 2e-10, error=residual)
            if alpha == 0 or anchor == .5:
                close('nonlinear_exact_linearization_cases', actual, .5, 2e-11, absolute=True)
            else:
                require('nonlinear_prediction_counterexample', actual < .5-1e-8)
            errors.append(abs(actual-.5))
            nonlinear.append({'alpha': alpha, 'anchor': anchor, 'H_local': H, 'p_local': p,
                              'k_local': k, 'k_exact_target': exact_k, 'actual_root': actual,
                              'gap_prediction_error': actual-.5, 'actual_residual_at_predicted_target': g(.5)-k*b.force(.5, 1.)})
        error_sequences.append(errors)
        for x, y in zip(errors, errors[1:]):
            require('nonlinear_anchor_error_trend', x+2e-11 >= y)
    local_upper = 6*(.55-.1)/b.force(.55, 1.)
    failed_band_root = b.root(lambda d: 6*(d-.1)+100*(d-.1)**3, local_upper)
    require('local_tangent_band_counterexample', failed_band_root < .45-1e-3)

    H = [[2., 0., 0.], [0., 3., 0.], [0., 0., 5.]]
    J = [[1., -1., 0.], [0., 1., -1.]]
    p, target = [.2, .25], [.5, .55]
    lifts = [solve_spd(H, row) for row in J]
    W = [[dot(a, v) for v in lifts] for a in J]
    expected_W = [[5/6, -1/3], [-1/3, 8/15]]
    for a, e in zip(sum(W, []), sum(expected_W, [])):
        close('coupled_compliance_matrix', a, e)
    delta = [a-z for a, z in zip(target, p)]
    force = solve_spd(W, delta)
    for actual, expected in zip(force, [.78, 1.05]):
        close('coupled_required_force', actual, expected)
    k = [F/b.force(d, 1.) for F, d in zip(force, target)]
    coupled = b.coupled(H, J, p, k)
    for actual, expected in zip(coupled['gaps'], target):
        close('coupled_target_root', actual, expected, 2e-9, absolute=True)
    close('coupled_action_reaction', sum(mv(transpose(J), coupled['forces'])), 0.)
    diagonal_k = [delta[i]/W[i][i]/b.force(target[i], 1.) for i in range(2)]
    diagonal = b.coupled(H, J, p, diagonal_k)
    require('ignoring_coupling_counterexample', max(abs(a-e) for a, e in zip(diagonal['gaps'], target)) > .01)
    incompatible_force = solve_spd([[2., 1.], [1., 2.]], [.1, .5])
    close('joint_target_requires_attraction', incompatible_force[0], -.1)
    close('joint_target_second_force', incompatible_force[1], .3)

    invalid = [([[1., -1.], [-1., 1.]], [-1., 1.], 'rigid null mode, relative J'),
               ([[1., -1.], [-1., 1.]], [1., 1.], 'rigid null mode, sensitive J'),
               ([[-1., 0.], [0., 2.]], [0., 1.], 'indefinite H despite positive selected curvature'),
               ([[1., 0.], [0., 1.]], [0., 0.], 'zero J'),
               ([[1., 1.], [0., 1.]], [1., 0.], 'nonsymmetric H'),
               ([[float('nan')]], [1.], 'nonfinite H')]
    rejections = []
    for matrix, jac, name in invalid:
        try:
            compliance(matrix, jac)
            rejected = False
        except ValueError:
            rejected = True
        require('unsupported_tangent_rejected', rejected, name)
        rejections.append({'case': name, 'rejected': rejected})
    supported, _ = compliance([[1.]], [-1.])
    close('explicit_support_removes_null_mode', supported, 1.)
    tiny, _ = compliance([[1e-10, 0.], [0., 1.]], [1., 0.])
    close('tiny_positive_not_floored', tiny/1e-10, 1.)
    require('tiny_positive_preserved', 0 < tiny < 1e-9)

    for row in b.root_records:
        require('all_scalar_root_residuals', math.isfinite(row['normalized_residual']) and row['normalized_residual'] <= 2e-10,
                row, row['normalized_residual'])
        require('all_scalar_root_brackets', row['relative_bracket_width'] <= 2e-11, error=row['relative_bracket_width'])
        require('all_scalar_roots_positive', row['root'] > 0)

    result = {'schema': 'polyfem.rb13.stage3', 'production_law_changed': False,
              'checks': sum(x['checks'] for x in checks.values()), 'failed_checks': len(failures),
              'groups': checks, 'failures': failures, 'springs': springs, 'rigid_limit': rigid,
              'dynamics': dynamics, 'unit_conversions': units, 'nonlinear': nonlinear,
              'local_band_counterexample': {'band': [.45, .55], 'k_upper_local': local_upper, 'actual_root': failed_band_root},
              'coupled': {'H': H, 'J': J, 'W': W, 'p': p, 'target': target, 'forces_required': force,
                          'k': k, 'solution': coupled, 'diagonal_k': diagonal_k, 'diagonal_solution': diagonal,
                          'incompatible_joint_target_forces': incompatible_force},
              'invalid_tangent_cases': rejections, 'tiny_K_eff': tiny, 'scalar_roots': b.root_records}
    (args.output/'results.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(f"{result['checks']-len(failures)}/{result['checks']} checks passed")
    for failure in failures:
        print(json.dumps(failure))
    return int(bool(failures))


if __name__ == '__main__':
    sys.exit(main())
