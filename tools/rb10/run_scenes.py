"""RB-10 coupled friction fixtures: generate isolated scenes, run them, summarize.

Usage:
  python3 tools/rb10/run_scenes.py --binary build/PolyFEM_bin --output /absolute/fresh/dir \
      [--stage 2|3|classic|all] [--threads 1] [--only name,name] [--dry-run]

Every run is an isolated copy of the public `scenes/semi-implicit` cube/slab
fixture (never modified in place) with the RB-04 physical diagnostics on. The
kinematics are prescribed on the cube's top face (surface selection 2): a
normal press of `press` by t = t_press, then the fixture's tangential program.
Stage 2 is the fixture matrix at the default friction policy (budget 1,
epsv 1e-3), quasistatic and transient (ImplicitEuler). Stage 3 is the
predeclared budget/smoothing sensitivity on `slide_plus` and `reversal`.
`classic` runs one classic-adaptive friction case with budget 2 to observe
the lag-loop ordering in the log. Nothing here runs Teseo or a private scene.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

HERE = Path(__file__).resolve().parent
SCENES = HERE.parents[1] / 'scenes' / 'semi-implicit'
DT, TEND, T_PRESS, PRESS, RATE = 0.05, 1.0, 0.2, 0.05, 0.25
BUDGETS = [1, 2, 4, 8]
EPSVS = [1e-3, 1e-2, 1e-1, 1.0]
# Stages 2/3/classic were run before the RB-10 decision at the then defaults
# (budget 1, RB-18 F6 trim following); they write those settings explicitly
# so the historical evidence stays reproducible after the default change.
HISTORICAL_LAG = 'follow_stiffness'

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('--binary', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--stage', default='2', choices=['2', '3', 'classic', 'ab', 'defaults', 'all'])
parser.add_argument('--threads', type=int, default=1)
parser.add_argument('--only', default='')
parser.add_argument('--dry-run', action='store_true', help='write the scenes, run nothing')
parser.add_argument('--timeout', type=float, default=1800)
args = parser.parse_args()
binary = args.binary.resolve()
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)


def ramp(rate, t0):
    return f'{rate}*max(0,t-{t0})'


def kinematics(name):
    """Top-face displacement expressions [ux, uy, uz] and options per fixture."""
    press = f'-{PRESS}*min(t/{T_PRESS},1)'
    base = {'ux': '0', 'uy': '0', 'uz': press, 'obstacle_ux': None, 'wall': False, 'mu': 0.3}
    if name == 'zero_friction_slide':
        base.update(ux=ramp(RATE, T_PRESS), mu=0.0)
    elif name == 'slide_plus':
        base.update(ux=ramp(RATE, T_PRESS))
    elif name == 'slide_minus':
        base.update(ux=ramp(-RATE, T_PRESS))
    elif name == 'reversal':
        # +RATE from t_press to 0.6, then -RATE back to ux = 0 at t = 1
        base.update(ux=f'{RATE}*(max(0,t-{T_PRESS})-2*max(0,t-0.6))')
    elif name == 'separation_recontact':
        # press, hold, lift back to the initial gap (separated: 0.02 > dhat), re-press
        base.update(ux=ramp(RATE, T_PRESS),
                    uz=f'-{PRESS}*min(t/{T_PRESS},1)+{PRESS}*min(max(0,t-0.4)/0.2,1)-{PRESS}*min(max(0,t-0.7)/0.2,1)')
    elif name == 'moving_obstacle':
        base.update(obstacle_ux=ramp(-RATE, T_PRESS))
    elif name == 'corner_coupled':
        # a half-height vertical wall obstacle at x = 1.02: the slide runs into it
        base.update(ux=ramp(RATE, T_PRESS), wall=True)
    else:
        raise ValueError(name)
    return base


def scene_json(kin, model, mu, epsv, budget, barrier='semi_implicit', friction_lag=None):
    """budget None / friction_lag None: leave the key out (the solver's current default)."""
    geometry = [
        {'mesh': 'cube.mesh', 'surface_selection': [{'id': 2, 'axis': '+z', 'position': 0.99}]},
        {'mesh': 'slab.obj', 'is_obstacle': True},
    ]
    bcs = {'dirichlet_boundary': [{'id': 2, 'value': [kin['ux'], kin['uy'], kin['uz']]}]}
    if kin['obstacle_ux'] is not None:
        geometry[1]['surface_selection'] = 3
        bcs['obstacle_displacements'] = [{'id': 3, 'value': [kin['obstacle_ux'], '0', '0']}]
    if kin['wall']:
        geometry.append({'mesh': 'wall.obj', 'is_obstacle': True})
    scene = {
        'geometry': geometry,
        'materials': {'type': 'NeoHookean', 'E': 1e7, 'nu': 0.45, 'rho': 1000},
        'time': {'tend': TEND, 'dt': DT, 'quasistatic': model == 'quasistatic', 'integrator': 'ImplicitEuler'},
        'contact': {'enabled': True, 'dhat': 1e-3, 'friction_coefficient': mu, 'epsv': epsv},
        'boundary_conditions': bcs,
        'solver': {
            'contact': {'barrier_stiffness': barrier},
            'linear': {'solver': 'Eigen::SimplicialLDLT'},
        },
        'output': {'log': {'level': 'debug'}, 'paraview': {'file_name': 'run.pvd'}, 'physical_diagnostics': True},
    }
    if budget is not None:
        scene['solver']['contact']['friction_iterations'] = budget
    if friction_lag is not None:
        scene['solver']['contact']['semi_implicit'] = {'friction_lag': friction_lag}
    return scene


def smoke_json(friction_lag=None, budget=1):
    """budget None / friction_lag None: leave the key out (the solver's current default)."""
    """Isolated copy of the public quasistatic-semi-friction smoke with the diagnostics on."""
    scene = json.loads((SCENES / 'quasistatic-semi-friction.json').read_text())
    scene['output']['physical_diagnostics'] = True
    scene['output']['paraview']['file_name'] = 'run.pvd'
    if budget is not None:
        scene['solver']['contact']['friction_iterations'] = budget
    if friction_lag is not None:
        scene['solver']['contact']['semi_implicit'] = {'friction_lag': friction_lag}
    return scene


def configurations(stage):
    runs = []
    if stage in ('2', 'all'):
        for name in ['zero_friction_slide', 'slide_plus', 'slide_minus', 'reversal', 'separation_recontact', 'moving_obstacle', 'corner_coupled']:
            for model in ['quasistatic', 'transient']:
                kin = kinematics(name)
                runs.append({'name': f'{name}-{model}', 'fixture': name, 'model': model, 'mu': kin['mu'], 'epsv': EPSVS[0], 'budget': BUDGETS[0], 'barrier': 'semi_implicit', 'friction_lag': HISTORICAL_LAG, 'stage': 2})
    if stage in ('3', 'all'):
        for name in ['slide_plus', 'reversal']:
            for model in ['quasistatic', 'transient']:
                kin = kinematics(name)
                for budget in BUDGETS[1:]:
                    runs.append({'name': f'{name}-{model}-budget{budget}', 'fixture': name, 'model': model, 'mu': kin['mu'], 'epsv': EPSVS[0], 'budget': budget, 'barrier': 'semi_implicit', 'friction_lag': HISTORICAL_LAG, 'stage': 3})
                for epsv in EPSVS[1:]:
                    runs.append({'name': f'{name}-{model}-epsv{epsv:g}', 'fixture': name, 'model': model, 'mu': kin['mu'], 'epsv': epsv, 'budget': BUDGETS[0], 'barrier': 'semi_implicit', 'friction_lag': HISTORICAL_LAG, 'stage': 3})
    if stage in ('ab', 'all'):
        # RB-10 friction_lag A/B: the default (follow_stiffness, RB-18 F6) against
        # the opt-in realized_force lag, on the fixtures with in-solve trim
        # actions and on the public friction smoke (trim 2 -> 16 over 4 steps).
        for name in ['slide_plus', 'slide_minus', 'separation_recontact', 'corner_coupled']:
            for model in ['quasistatic', 'transient']:
                kin = kinematics(name)
                for mode in ['follow_stiffness', 'realized_force']:
                    runs.append({'name': f'{name}-{model}-{mode}', 'fixture': name, 'model': model, 'mu': kin['mu'], 'epsv': EPSVS[0], 'budget': BUDGETS[0], 'barrier': 'semi_implicit', 'friction_lag': mode, 'stage': 'ab'})
        for mode in ['follow_stiffness', 'realized_force']:
            runs.append({'name': f'smoke-quasistatic-semi-friction-{mode}', 'fixture': 'smoke', 'model': 'quasistatic', 'mu': 0.3, 'epsv': 1e-3, 'budget': 1, 'barrier': 'semi_implicit', 'friction_lag': mode, 'stage': 'ab'})
            # one lag correction: how much of the mode difference a second lag iteration removes
            runs.append({'name': f'smoke-quasistatic-semi-friction-{mode}-budget2', 'fixture': 'smoke', 'model': 'quasistatic', 'mu': 0.3, 'epsv': 1e-3, 'budget': 2, 'barrier': 'semi_implicit', 'friction_lag': mode, 'stage': 'ab'})
    if stage in ('classic', 'all'):
        for model in ['quasistatic', 'transient']:
            kin = kinematics('slide_plus')
            runs.append({'name': f'slide_plus-{model}-classic-adaptive-budget2', 'fixture': 'slide_plus', 'model': model, 'mu': kin['mu'], 'epsv': EPSVS[0], 'budget': 2, 'barrier': 'adaptive', 'stage': 'classic'})
    if stage in ('defaults', 'all'):
        # The solver's current defaults (no friction_iterations / friction_lag
        # key written): the validation of the RB-10 decision.
        for name in ['zero_friction_slide', 'slide_plus', 'slide_minus', 'reversal', 'separation_recontact', 'moving_obstacle', 'corner_coupled']:
            for model in ['quasistatic', 'transient']:
                kin = kinematics(name)
                runs.append({'name': f'{name}-{model}-defaults', 'fixture': name, 'model': model, 'mu': kin['mu'], 'epsv': EPSVS[0], 'budget': None, 'barrier': 'semi_implicit', 'friction_lag': None, 'stage': 'defaults'})
        runs.append({'name': 'smoke-quasistatic-semi-friction-defaults', 'fixture': 'smoke', 'model': 'quasistatic', 'mu': 0.3, 'epsv': 1e-3, 'budget': None, 'barrier': 'semi_implicit', 'friction_lag': None, 'stage': 'defaults'})
    return runs


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


# Half-height wall at x = 1.02 (z up to .5): it blocks the sliding lower half
# of the cube while the prescribed top face (z = 1) passes over it. A
# full-height wall is infeasible under the prescribed top motion (the first
# attempt stalled against CCD at the first wall contact; retained in the
# evidence as corner_coupled-quasistatic-v1-infeasible-wall).
WALL = """v 1.02 -1 -1
v 1.02 2 -1
v 1.02 2 0.5
v 1.02 -1 0.5
f 1 2 3
f 1 3 4
"""

only = set(filter(None, args.only.split(',')))
matrix = []
for cfg in configurations(args.stage):
    if only and cfg['name'] not in only:
        continue
    run_dir = out / cfg['name']
    if (run_dir / 'run.json').exists():
        matrix.append(json.loads((run_dir / 'run.json').read_text()))
        continue
    run_dir.mkdir(parents=True, exist_ok=True)
    if cfg['fixture'] == 'smoke':
        kin = {'wall': False}
        scene = smoke_json(cfg.get('friction_lag'), cfg['budget'])
    else:
        kin = kinematics(cfg['fixture'])
        scene = scene_json(kin, cfg['model'], cfg['mu'], cfg['epsv'], cfg['budget'], cfg['barrier'], cfg.get('friction_lag'))
    (run_dir / 'scene.json').write_text(json.dumps(scene, indent=2) + '\n')
    shutil.copy(SCENES / 'cube.mesh', run_dir / 'cube.mesh')
    shutil.copy(SCENES / 'slab.obj', run_dir / 'slab.obj')
    if kin['wall']:
        (run_dir / 'wall.obj').write_text(WALL)
    record = dict(cfg)
    record['inputs'] = {f: sha(run_dir / f) for f in ['scene.json', 'cube.mesh', 'slab.obj'] + (['wall.obj'] if kin['wall'] else [])}
    record['binary_sha256'] = sha(binary)
    record['command'] = [str(binary), '--json', 'scene.json', '-o', 'output', '--log_level', 'debug', '--max_threads', str(args.threads)]
    if args.dry_run:
        record['status'] = 'not run (dry run)'
        matrix.append(record)
        continue
    start = time.time()
    with open(run_dir / 'run.log', 'w') as log:
        try:
            proc = subprocess.run(record['command'], cwd=run_dir, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
            record['exit_status'] = proc.returncode
        except subprocess.TimeoutExpired:
            record['exit_status'] = 'timeout'
    record['wall_seconds'] = time.time() - start
    text = (run_dir / 'run.log').read_text()
    record['error_lines'] = sum('[error]' in line for line in text.splitlines())
    record['lagging_warnings'] = sum('Lagging failed to converge' in line for line in text.splitlines())
    record['lagging_converged_lines'] = sum('Lagging converged' in line for line in text.splitlines())
    pvd = run_dir / 'output' / 'run.pvd'
    if pvd.exists():
        import re
        record['saved_times'] = [float(t) for t in re.findall(r'timestep="([^"]+)"', pvd.read_text())]
    diagnostics = run_dir / 'output' / 'physical-diagnostics.jsonl'
    if diagnostics.exists():
        lines = diagnostics.read_text().splitlines()
        record['diagnostic_records'] = len(lines)
        record['accepted_steps'] = sum(json.loads(line).get('outcome') == 'accepted' for line in lines)
    expected_steps = round(scene['time']['tend'] / scene['time']['dt'])
    record['expected_steps'] = expected_steps
    record['status'] = 'complete' if record.get('exit_status') == 0 and record.get('accepted_steps') == expected_steps else 'incomplete or failed'
    (run_dir / 'run.json').write_text(json.dumps(record, indent=2) + '\n')
    matrix.append(record)
    print(f"{cfg['name']:<48} exit={record.get('exit_status')} steps={record.get('accepted_steps')} errors={record['error_lines']} {record['wall_seconds']:.1f}s", flush=True)

(out / 'matrix.json').write_text(json.dumps(matrix, indent=2) + '\n')
print(json.dumps({'runs': len(matrix), 'complete': sum(r.get('status') == 'complete' for r in matrix)}))
