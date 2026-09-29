#!/usr/bin/env python3
"""Synthetic scenes for the free-DOF gradient balance (docs/gradient-balance-free-dofs-20260929.md).

usage: synthetic.py --binary B --out E [--scene D1 ...] [--mode default ...]

Built like tools/clamped/synthetic.py: the public smoke's cube.mesh (unit
cube, 5x5x5 vertices, spacing 0.25) as the free body C, NeoHookean E 1e7,
nu 0.45, rho 1000, dhat 1e-3, Eigen::SimplicialLDLT, one thread, the
production controller stated explicitly (band_statistic rms, no initial
estimate, clamped_contacts exclude_statistics). What C touches decides
which Dirichlet rows enter the full-DOF balance:

  D1  drop, coarse obstacle   C falls under gravity (rhs 9.81 along z) from
                              0.05 above slab.obj (4 vertices): the obstacle
                              half of every contact sums onto 4 nodes
  D2  drop, fine obstacle     as D1 onto a 31x31 grid obstacle (spacing 0.1)
  D3  drop, clamped block     as D1 onto a second cube.mesh whose whole
                              surface is clamped (shifted 0.125 in x and y)
  D4  drop, elastic support   control: as D3 with only the support's bottom
                              face clamped; C touches free DOFs only, the
                              Dirichlet rows carry the support's reaction
  Q1  press, clamped block    quasistatic: C's top face pushed down 0.25 t
                              onto the clamped block of D3 (start gap 0.5 dhat)
  Q2  press, elastic support  control of Q1 with the support of D4

Drops: transient (implicit Euler), dt 0.01, 40 steps; presses: 4 steps of 0.25.
Modes: default (key absent), all, free. Writes E/<scene>-<mode>/{scene.json,
run.log,row.json,output/}; the grid obstacle is written to E/grid31.obj.
"""
import argparse, hashlib, json, subprocess, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SMOKE = HERE.parents[1] / 'scenes' / 'semi-implicit'
DHAT = 1e-3
ALL = {'box': [[-0.1, -0.1, -0.1], [1.1, 1.1, 1.1]], 'relative': True}
TOP = {'box': [[-0.1, -0.1, 0.99], [1.1, 1.1, 1.1]], 'relative': True}
BOTTOM = {'box': [[-0.1, -0.1, -0.1], [1.1, 1.1, 0.01]], 'relative': True}


def grid_obstacle(path, n=31, lo=-1.0, hi=2.0, z=-0.02):
    h = (hi - lo) / (n - 1)
    lines = [f'v {lo + i * h:.6g} {lo + j * h:.6g} {z}' for j in range(n) for i in range(n)]
    for j in range(n - 1):
        for i in range(n - 1):
            a = j * n + i + 1
            lines += [f'f {a} {a + 1} {a + n + 1}', f'f {a} {a + n + 1} {a + n}']
    path.write_text('\n'.join(lines) + '\n')


def scene(name, mode, grid):
    drop = name.startswith('D')
    geometry, dirichlet = [], []
    if drop:
        geometry.append({'mesh': str(SMOKE / 'cube.mesh'), 'transformation': {'translation': [0, 0, 0.05]}})
    else:
        z0 = 0.5 * DHAT
        geometry.append({'mesh': str(SMOKE / 'cube.mesh'), 'transformation': {'translation': [0, 0, z0]},
                         'surface_selection': [dict(TOP, id=2)]})
        dirichlet.append({'id': 2, 'value': [0, 0, '-0.25*t']})
    if name in ('D1', 'D2'):
        geometry.append({'mesh': str(SMOKE / 'slab.obj') if name == 'D1' else str(grid), 'is_obstacle': True})
    else:
        clamped = name in ('D3', 'Q1')
        geometry.append({'mesh': str(SMOKE / 'cube.mesh'), 'transformation': {'translation': [0.125, 0.125, -1.0]},
                         'surface_selection': [dict(ALL if clamped else BOTTOM, id=3)]})
        dirichlet.append({'id': 3, 'value': [0, 0, 0]})
    si = {'band_statistic': 'rms', 'initial_trim_estimate': False, 'clamped_contacts': 'exclude_statistics'}
    if mode != 'default':
        si['gradient_balance_dofs'] = mode
    bc = {'dirichlet_boundary': dirichlet}
    if drop:
        bc['rhs'] = [0, 0, 9.81]
    return {
        'geometry': geometry,
        'materials': {'type': 'NeoHookean', 'E': 1e7, 'nu': 0.45, 'rho': 1000},
        'time': {'tend': 0.4, 'dt': 0.01, 'quasistatic': False} if drop else {'tend': 1.0, 'dt': 0.25, 'quasistatic': True},
        'contact': {'enabled': True, 'dhat': DHAT},
        'boundary_conditions': bc,
        'solver': {'contact': {'barrier_stiffness': 'semi_implicit', 'semi_implicit': si},
                   'linear': {'solver': 'Eigen::SimplicialLDLT'},
                   'nonlinear': {'advanced': {'iteration_diagnostics': True}}},
        'output': {'trim_predictors': True, 'physical_diagnostics': False,
                   'paraview': {'file_name': 'sim.pvd', 'volume': True, 'surface': False, 'options': {'use_hdf5': False}}},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--scene', action='append', choices=['D1', 'D2', 'D3', 'D4', 'Q1', 'Q2'])
    ap.add_argument('--mode', action='append', choices=['default', 'all', 'free'])
    ap.add_argument('--timeout', type=float, default=1800)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    grid = (a.out / 'grid31.obj').resolve()
    if not grid.exists():
        grid_obstacle(grid)
    digest = hashlib.file_digest(a.binary.open('rb'), 'sha256').hexdigest()
    for name in a.scene or ['D1', 'D2', 'D3', 'D4', 'Q1', 'Q2']:
        for mode in a.mode or ['default', 'free']:
            run = (a.out / f'{name}-{mode}').resolve()
            if run.exists():
                raise SystemExit(f'{run} exists')
            (run / 'output').mkdir(parents=True)
            (run / 'scene.json').write_text(json.dumps(scene(name, mode, grid), indent=1) + '\n')
            cmd = [str(a.binary.resolve()), '--json', str(run / 'scene.json'), '-o', str(run / 'output'),
                   '--max_threads', '1', '--log_level', 'debug']
            row = dict(scene=name, mode=mode, command=cmd, binary_sha256=digest, started=time.strftime('%Y-%m-%dT%H:%M:%S'))
            t = time.monotonic()
            with (run / 'run.log').open('w') as log:
                try:
                    row['exit_code'] = subprocess.run(cmd, cwd=run, stdout=log, stderr=subprocess.STDOUT, timeout=a.timeout).returncode
                    row['timed_out'] = False
                except subprocess.TimeoutExpired:
                    row['exit_code'], row['timed_out'] = None, True
            row['wall_seconds'] = round(time.monotonic() - t, 2)
            (run / 'row.json').write_text(json.dumps(row, indent=1) + '\n')
            print(json.dumps({k: row[k] for k in ('scene', 'mode', 'exit_code', 'timed_out', 'wall_seconds')}), flush=True)


if __name__ == '__main__':
    main()
