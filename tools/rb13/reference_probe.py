#!/usr/bin/env python3
"""RB-13 stage 1 only. Standard-library analytical checks plus real IPC primitives.

Predeclared tolerances: direct identities 1e-11*(1+abs(reference));
barrier FD force 2e-7 and curvature 2e-6 after dhat dimensional normalization.
The decimal energy-only FD uses 60 digits and step 1e-5*dhat, away from endpoints.
These thresholds cover truncation at d/dhat >= .1, not near-singular geometry.
"""

import argparse
from decimal import Decimal, localcontext
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import sys


def solve_spd(a, b):
    """Tiny Cholesky reference solve; no inverse, projection, or pseudoinverse."""
    n = len(b)
    l = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            if a[i][j] != a[j][i]:
                raise ValueError('nonsymmetric reference tangent')
            v = a[i][j] - sum(l[i][r] * l[j][r] for r in range(j))
            if i == j:
                if not math.isfinite(v) or v <= 0:
                    raise ValueError('reference tangent must be SPD')
                l[i][j] = math.sqrt(v)
            else:
                l[i][j] = v / l[j][j]
    y = [0.0] * n
    for i in range(n):
        y[i] = (b[i] - sum(l[i][j] * y[j] for j in range(i))) / l[i][i]
    x = [0.0] * n
    for i in reversed(range(n)):
        x[i] = (y[i] - sum(l[j][i] * x[j] for j in range(i + 1, n))) / l[i][i]
    return x


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def energy_decimal(d, h):
    s, support = d * d, h * h
    return -(s - support) ** 2 * (s / support).ln()


def shape(d, h):
    """Independent physical-distance expression and its explicit derivatives."""
    r = d / h
    log_r = math.log(r)
    b = -2 * h**4 * (r*r - 1)**2 * log_r
    f = h**3 * (8*r*(r*r-1)*log_r + 2*(r*r-1)**2/r)
    curvature = h**2 * (-8*(3*r*r-1)*log_r - 14*r*r + 12 + 2/(r*r))
    return b, f, curvature


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ipc-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cxx', default='/usr/bin/c++')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    ipc = args.ipc_source.resolve()
    here = Path(__file__).resolve().parent
    sources = [here/'barrier_bridge.cpp', ipc/'src/ipc/barrier/barrier.cpp']
    command = [args.cxx, '-std=c++17', '-O2', '-I'+str(ipc/'src'),
               *map(str, sources), '-o', str(args.output/'barrier_bridge')]
    provenance = {'compile': command, 'python': sys.version, 'platform': platform.platform(),
                  'ipc_head': subprocess.check_output(['git', '-C', str(ipc), 'rev-parse', 'HEAD'], text=True).strip(),
                  'hashes': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in sources + [Path(__file__), ipc/'src/ipc/barrier/barrier.hpp']}}
    (args.output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    compile_run = subprocess.run(command, capture_output=True, text=True)
    (args.output/'compile.log').write_text(compile_run.stdout+compile_run.stderr)
    compile_run.check_returncode()
    grid = [(h, r*h) for h in (.01, 1., 100.) for r in (.1, .25, .5, .8, .95)]
    input_text = ''.join(f'{d*d:.17g} {h*h:.17g}\n' for h, d in grid)
    run = subprocess.run([str(args.output/'barrier_bridge')], input=input_text,
                         capture_output=True, text=True, check=True)
    (args.output/'bridge-input.txt').write_text(input_text)
    (args.output/'bridge-output.txt').write_text(run.stdout)
    checks = []

    def check(name, actual, expected, tolerance=1e-11):
        error = abs(actual-expected)/(1+abs(expected))
        checks.append({'name': name, 'actual': actual, 'expected': expected,
                       'scaled_error': error, 'tolerance': tolerance,
                       'pass': math.isfinite(error) and error <= tolerance})

    rows = run.stdout.splitlines()
    if len(rows) != len(grid):
        raise RuntimeError('incomplete compiled barrier output')
    barriers = []
    for (h, d), row in zip(grid, rows):
        b, bs, bss = map(float, row.split())
        force, tangent = -2*d*bs, 2*bs+4*d*d*bss
        bp, fp, tp = shape(d, h)
        prefix = f'h={h},r={d/h:g}'
        check(prefix+'/value', b/h**4, bp/h**4)
        check(prefix+'/force-chain', force/h**3, fp/h**3)
        check(prefix+'/tangent-chain', tangent/h**2, tp/h**2)
        with localcontext() as ctx:
            ctx.prec = 60
            dd, hh = Decimal(str(d)), Decimal(str(h))
            eps = hh * Decimal('0.00001')
            e0 = energy_decimal(dd, hh)
            em, ep = energy_decimal(dd-eps, hh), energy_decimal(dd+eps, hh)
            fd_force = float(-(ep-em)/(2*eps))
            fd_tangent = float((ep-2*e0+em)/(eps*eps))
        check(prefix+'/energy-FD-force', force/h**3, fd_force/h**3, 2e-7)
        check(prefix+'/energy-FD-tangent', tangent/h**2, fd_tangent/h**2, 2e-6)
        barriers.append({'dhat': h, 'd': d, 'b': b, 'unit_force': force,
                         'unit_tangent': tangent})

    # Condense a quadratic in free coordinates. A fixed second full DOF
    # contributes its coupling through the reduced residual, not free compliance.
    hfull = [[6., 2.], [2., 5.]]
    prescribed, load, gap_offset = .1, .8, .3
    zfree = (load-hfull[0][1]*prescribed)/hfull[0][0]
    dfree = gap_offset+zfree-prescribed
    j, hfree = [1.], [[hfull[0][0]]]
    lift = solve_spd(hfree, j)
    keff = 1/dot(j, lift)
    target, dhat = .5, 1.
    needed = keff*(target-dfree)
    coefficient = needed/shape(target, dhat)[1]
    check('prescribed predictor', dfree, .3)
    check('free compliance', keff, 6.)
    check('required repulsion', needed, 1.2)
    ztarget = target-gap_offset+prescribed
    force_at_target = coefficient*shape(target, dhat)[1]
    check('full free force balance', 6*ztarget+2*prescribed-load-force_at_target, 0.)
    # Root from the full free equilibrium, not the k-estimate expression.
    lo, hi = 1e-10, dhat
    for _ in range(100):
        mid = (lo+hi)/2
        z = mid-gap_offset+prescribed
        residual = 6*z+2*prescribed-load-coefficient*shape(mid, dhat)[1]
        if residual < 0:
            lo = mid
        else:
            hi = mid
    root = (lo+hi)/2
    check('independent equilibrium root', root, target)
    # Non-axis-aligned J tests the compliance solve and constrained minimizer.
    jac = [1., -.5]
    v = solve_spd(hfull, jac)
    compliance = dot(jac, v)
    delta = .2
    displacement = [x*delta/compliance for x in v]
    check('gap-constrained displacement', dot(jac, displacement), delta)
    check('condensed quadratic energy',
          .5*dot(displacement, [dot(row, displacement) for row in hfull]),
          .5*delta*delta/compliance)
    for i, matrix in enumerate(([[0.]], [[-1.]])):
        try:
            solve_spd(matrix, [1.])
            rejected = False
        except ValueError:
            rejected = True
        check(f'invalid tangent rejected {i}', float(rejected), 1.)

    # Actual source-derived objective scales; algebraic controls, not execution
    # of PolyFEM integrator classes. No extra mass is added to H_obj/a.
    integrators = []
    dt, mass, spring, rest, external = .2, 2., 7., .4, -3.
    xprev, vprev, aprev = .6, -.7, .8
    for name, a, predictor in [
        ('implicit_euler_second_order', dt*dt, xprev+dt*vprev),
        ('newmark_beta_0.25', .25*dt*dt, xprev+dt*vprev+dt*dt*(.5-.25)*aprev),
        ('bdf2_second_order', (2/3*dt)**2,
         4/3*xprev-1/3*.65+(2/3*dt)*(4/3*vprev-1/3*(-.5))),
    ]:
        hobj = a*spring+mass
        hphys = hobj/a
        free = (a*(spring*rest+external)+mass*predictor)/hobj
        check(name+'/physical tangent', hphys, spring+mass/a)
        check(name+'/physical residual', spring*(free-rest)-external+mass/a*(free-predictor), 0.)
        check(name+'/scaled compliance', 1/(a/hobj), hphys)
        integrators.append({'name': name, 'a': a, 'predictor': predictor,
                            'H_objective': hobj, 'H_physical': hphys, 'free': free})

    # Frozen coefficient, weight, trim, normalization and scalar collision weight.
    a, normalization, trim, omega, kcache = .04, 2., 3., 4., 5.
    b, f, tangent = shape(.5, 1.)
    objective_coefficient = a*trim*omega*kcache/normalization
    physical_coefficient = trim*omega*kcache/normalization
    check('objective to physical energy', objective_coefficient*b/a, physical_coefficient*b)
    check('objective to physical force', objective_coefficient*f/a, physical_coefficient*f)
    check('objective to physical tangent', objective_coefficient*tangent/a, physical_coefficient*tangent)
    # Expose a wrong chain rule and a wrong interpretation of the controller.
    _, bs, bss = map(float, rows[7].split())  # h=1, d=.5
    wrong_tangent = 4*.5**2*bss
    check('missing chain term is detectably wrong', float(abs(wrong_tangent-tangent) > 1.), 1.)
    check('single contact lower squared threshold', math.sqrt(.5)**2, .5)
    check('single contact upper squared threshold', math.sqrt(.9)**2, .9)
    gaps = [.01, .8]
    mean_sq = sum(d*d for d in gaps)/len(gaps)
    severity = min(mean_sq, 100*min(d*d for d in gaps))
    check('controller severity example', severity, .01)
    check('RMS differs from mean gap', float(abs(math.sqrt(mean_sq)-sum(gaps)/2) > .1), 1.)

    result = {'schema': 'polyfem.rb13.stage1', 'production_law_changed': False,
              'scope': 'isolated affine-gap SPD quadratic; ordinary unweighted barrier; no trajectory',
              'checks_passed': sum(c['pass'] for c in checks), 'checks_total': len(checks),
              'barriers': barriers, 'spring': {'d_free': dfree, 'target': target,
                  'K_eff': keff, 'force_needed': needed, 'k': coefficient, 'root': root,
                  'barrier_tangent': coefficient*shape(target, dhat)[2]},
              'integrator_algebra': integrators,
              'counterexamples': {'tangent_omitting_2Bs': wrong_tangent,
                                 'correct_unit_tangent': tangent,
                                 'mean_squared_gap': mean_sq, 'severity': severity},
              'checks': checks}
    (args.output/'results.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(f"{result['checks_passed']}/{len(checks)} checks passed")
    for c in checks:
        if not c['pass']:
            print(json.dumps(c))
    return 0 if all(c['pass'] for c in checks) else 1


if __name__ == '__main__':
    sys.exit(main())
