#!/usr/bin/env python3
"""Default-controller assessment (2026-09-29): one run of one arm on one scene.

usage: dca_run.py LABEL --scene S --arm P|E|F|B[...] [--steps N] [--threads T]
                  [--realization acc1|cholmod|simplicial] [--set PTR=JSON ...]

Wraps tools/ef02/run.py (EF-01 driver: Newton, physical diagnostics and trim
predictors on, coefficient events discarded, solution-only VTU). Every arm sets
the controller options explicitly, whatever the scene file says (R4's export
selects force_weighted + estimate: docs/r4-production-speedup-20260928.md):

  P  production     band_statistic rms,            initial_trim_estimate false
  E  estimate only  band_statistic rms,            initial_trim_estimate true
  F  fw only        band_statistic force_weighted, estimate false, collapse_guard_basis pair
  B  both           band_statistic force_weighted, estimate true,  collapse_guard_basis pair
  Er / Br           E / B with initial_trim_estimate_scope "run"

clamped_contacts is left at the binary's default (exclude_statistics since
eb8286b7c) unless the scene sets it. Realizations change only the linear
solver (IT-reproducibility record: deterministic single-threaded realizations).
Scenes added here: PP pup_push, FB fibers, MT mesh_tissue (held out: never
used by EF-01..EF-07), ITF = IT with friction_coefficient 0.3 (synthetic).
"""
import argparse, json, subprocess, sys
from pathlib import Path

W = Path('/Users/stevenabramowitch/Downloads/fable_polyfem')
SI = '/solver/contact/semi_implicit/'
ARMS = {
    'P': {'band_statistic': 'rms', 'initial_trim_estimate': False},
    'E': {'band_statistic': 'rms', 'initial_trim_estimate': True},
    'F': {'band_statistic': 'force_weighted', 'initial_trim_estimate': False, 'collapse_guard_basis': 'pair'},
    'B': {'band_statistic': 'force_weighted', 'initial_trim_estimate': True, 'collapse_guard_basis': 'pair'},
}
ARMS['Er'] = dict(ARMS['E'], initial_trim_estimate_scope='run')
ARMS['Br'] = dict(ARMS['B'], initial_trim_estimate_scope='run')
REALIZATIONS = {'acc1': None, 'cholmod': 'Eigen::CholmodSupernodalLLT',
                'simplicial': 'Eigen::SimplicialLDLT'}
EXTRA_SCENES = {
    'PP': ('test_cases/pup_push/input', 'params.json'),
    'FB': ('test_cases/fibers/input', 'params.json'),
    'MT': ('test_cases/mesh_tissue/input', 'params.json'),
    'ITF': ('test_cases/inertia_test/input', 'params.json'),
}
SCENE_SETS = {'ITF': ['/contact/friction_coefficient=0.3']}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('label')
    ap.add_argument('--scene', required=True)
    ap.add_argument('--arm', required=True, choices=sorted(ARMS))
    ap.add_argument('--steps', type=int, default=1)
    ap.add_argument('--threads', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=3600)
    ap.add_argument('--realization', default='acc1', choices=sorted(REALIZATIONS))
    ap.add_argument('--binary', default=str(W / 'default-controller-work/bin/PolyFEM_bin-main'))
    ap.add_argument('--out', default=str(W / 'default-controller-work'))
    ap.add_argument('--set', action='append', default=[])
    a = ap.parse_args()

    sys.path.insert(0, str(W / 'polyfem/tools/ef01'))
    import ef01_run
    for key, (d, f) in EXTRA_SCENES.items():
        ef01_run.qn_run.SCENES[key] = W / d
        ef01_run.qn_run.SCENE_FILE[key] = f
    for key, name in [('smoke-adaptive', 'quasistatic-adaptive.json'),
                      ('smoke-alhess', 'quasistatic-semi-alhess.json'),
                      ('smoke-friction', 'quasistatic-semi-friction.json')]:
        ef01_run.qn_run.SCENES[key] = W / 'polyfem/scenes/semi-implicit'
        ef01_run.qn_run.SCENE_FILE[key] = name

    sets = [f'{SI}{k}={json.dumps(v)}' for k, v in ARMS[a.arm].items()]
    sets += SCENE_SETS.get(a.scene, [])
    if REALIZATIONS[a.realization]:
        sets.append(f'/solver/linear/solver={json.dumps(REALIZATIONS[a.realization])}')
    sets += a.set
    argv = [a.label, '--scene', a.scene, '--steps', str(a.steps), '--threads', str(a.threads),
            '--timeout', str(a.timeout), '--out', a.out, '--binary', a.binary,
            '--note', f'arm={a.arm} realization={a.realization}']
    for s in sets:
        argv += ['--set', s]
    sys.argv = [sys.argv[0]] + argv
    ef01_run.main()


if __name__ == '__main__':
    main()
