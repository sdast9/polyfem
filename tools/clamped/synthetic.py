#!/usr/bin/env python3
"""Synthetic scenes with contacts between clamped primitives (docs/clamped-contacts-20260928.md).

usage: synthetic.py --binary B --out E [--scene S1 ...] [--mode keep ...]

Every scene is quasistatic, uses the public smoke's cube.mesh (unit cube, 5x5x5
vertices) and slab.obj (obstacle at z = -0.02), dhat 1e-3, NeoHookean E 1e7,
Eigen::SimplicialLDLT, one thread, and the production controller stated
explicitly (band_statistic rms, no initial estimate). The free cube C is
pushed down onto the slab by its top face (partly clamped contacts). Blocks A
and B have their whole surface clamped (value 0), so every A-B contact is
fully clamped. Gaps are in dhat.

  S1  pinch       A-B gap 0.05 dhat, below the 0.0707 dhat collapse pair threshold
  S2  band        A-B gap 0.97 dhat, above the band's upper edge sqrt(0.9) = 0.949
  S3  average     A-B gap 0.5 dhat, below the band's lower edge sqrt(0.5) = 0.707
                  (the average-gap collapse term); C starts 0.3 above the slab,
                  so its first contact is born at step 6 of 8
  S5  first       A-B gap 0.8 dhat, inside the band [0.707, 0.949]; C as in S3:
                  the clamped pair is the only contact until step 6
  S0, S0late  controls: C and the slab only (as S1/S2 and as S3/S5), no blocks
  S4  al-hazard   no C; block P (whole surface prescribed, +x 0.3) is driven into
                  clamped block B 0.1 away: the Dirichlet target overlaps B, and
                  every P-B pair is fully clamped
  S4a             as S4 with only P's top and bottom faces prescribed (its side
                  faces are free, so P-B candidates keep a free vertex)

Writes E/<scene>-<mode>/{scene.json,run.log,row.json,output/}. Modes:
keep, exclude_statistics, exclude_collisions, and default (the key absent).
"""
import argparse, hashlib, json, subprocess, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SMOKE = HERE.parents[1] / 'scenes' / 'semi-implicit'
DHAT = 1e-3
ALL = {'box': [[-0.1, -0.1, -0.1], [1.1, 1.1, 1.1]], 'relative': True}


def block(x, y=2.0, z=0.0, sel_id=3, scale=1.0):
    return {'mesh': str(SMOKE / 'cube.mesh'), 'transformation': {'translation': [x, y, z], 'scale': [scale] * 3},
            'surface_selection': [dict(ALL, id=sel_id)]}


def scene(name, mode):
    geometry, dirichlet = [], []
    tend, dt = 1.0, 0.25
    if name in ('S0', 'S0late', 'S1', 'S2', 'S3', 'S5'):
        z0 = 0.3 if name in ('S0late', 'S3', 'S5') else 0.0
        geometry.append({'mesh': str(SMOKE / 'cube.mesh'), 'transformation': {'translation': [0, 0, z0]},
                         'surface_selection': [{'id': 2, 'box': [[-0.1, -0.1, 0.99], [1.1, 1.1, 1.1]], 'relative': True}]})
        dirichlet.append({'id': 2, 'value': [0, 0, '-0.25*t']})
        if not name.startswith('S0'):
            gap = {'S1': 0.05, 'S2': 0.97, 'S3': 0.5, 'S5': 0.8}[name] * DHAT
            geometry += [block(-3.0), block(-2.0 + gap)]
            dirichlet.append({'id': 3, 'value': [0, 0, 0]})
        geometry.append({'mesh': str(SMOKE / 'slab.obj'), 'is_obstacle': True})
        if name in ('S0late', 'S3', 'S5'):
            tend = 2.0
    else:  # S4/S4a: P prescribed +x, B clamped 0.1 beyond P's +x face
        faces = [dict(ALL, id=4)] if name == 'S4' else [
            {'id': 4, 'box': [[-0.1, -0.1, 0.99], [1.1, 1.1, 1.1]], 'relative': True},
            {'id': 4, 'box': [[-0.1, -0.1, -0.1], [1.1, 1.1, 0.01]], 'relative': True}]
        geometry.append({'mesh': str(SMOKE / 'cube.mesh'), 'transformation': {'translation': [0, 3, 0]},
                         'surface_selection': faces})
        dirichlet.append({'id': 4, 'value': ['0.3*t', 0, 0]})
        geometry.append(block(1.1, y=3.0))
        dirichlet.append({'id': 3, 'value': [0, 0, 0]})
    # clamped_contacts explicit except in mode "default" (key absent): its
    # default changed from keep to exclude_statistics on 2026-09-29.
    si = {'band_statistic': 'rms', 'initial_trim_estimate': False}
    if mode != 'default':
        si['clamped_contacts'] = mode
    return {
        'geometry': geometry,
        'materials': {'type': 'NeoHookean', 'E': 1e7, 'nu': 0.45, 'rho': 1000},
        'time': {'tend': tend, 'dt': dt, 'quasistatic': True},
        'contact': {'enabled': True, 'dhat': DHAT},
        'boundary_conditions': {'dirichlet_boundary': dirichlet},
        'solver': {'contact': {'barrier_stiffness': 'semi_implicit', 'semi_implicit': si},
                   'linear': {'solver': 'Eigen::SimplicialLDLT'},
                   'nonlinear': {'advanced': {'iteration_diagnostics': True}}},
        'output': {'trim_predictors': True, 'physical_diagnostics': False,
                   'paraview': {'file_name': 'sim.pvd', 'volume': True, 'surface': False, 'options': {'use_hdf5': False}}},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--scene', action='append', choices=['S0', 'S0late', 'S1', 'S2', 'S3', 'S4', 'S4a', 'S5'])
    ap.add_argument('--mode', action='append', choices=['keep', 'exclude_statistics', 'exclude_collisions', 'default'])
    ap.add_argument('--timeout', type=float, default=1800)
    a = ap.parse_args()
    digest = hashlib.file_digest(a.binary.open('rb'), 'sha256').hexdigest()
    for name in a.scene or ['S0', 'S0late', 'S1', 'S2', 'S3', 'S5', 'S4']:
        for mode in a.mode or ['keep', 'exclude_statistics', 'exclude_collisions']:
            run = (a.out / f'{name}-{mode}').resolve()
            if run.exists():
                raise SystemExit(f'{run} exists')
            (run / 'output').mkdir(parents=True)
            (run / 'scene.json').write_text(json.dumps(scene(name, mode), indent=1) + '\n')
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
