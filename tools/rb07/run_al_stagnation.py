"""RB-07 bounded AL stagnation: reproduction and end-to-end checks through PolyFEM_bin.

python3 tools/rb07/run_al_stagnation.py --output /absolute/fresh/dir --binary build/PolyFEM_bin \
    --stage reproduce|validate [--timeout 600] [--budget '{"max_passes": 12, ...}']

Public inputs only (the semi-implicit cube-on-slab fixture). Every scene is a
single quasistatic step, single-threaded, with the RB-04 diagnostics and the
RB-12 run manifest on:

  compatible-multipass   the top face is pressed 0.3 into the cube in one step
                         with a low initial AL weight and an AL subsolve that is
                         interrupted after four Newton iterations: the snap
                         from the first passes inverts the top layer, later
                         passes make it feasible (a useful multi-pass
                         continuation, must succeed with and without a budget)
  incompatible-collision the bottom face is prescribed to move 0.05 below its
                         rest position; the slab is 0.02 below it, so the snap
                         always crosses the obstacle (finite energy, no
                         inversion, blocked by CCD)
  incompatible-crush     the top face is prescribed to move down by the whole
                         cube height; the snap inverts the elements under it
                         (energy not finite, invalid) and no pass can make it
                         feasible

`--stage reproduce` runs the three scenes under a wall-clock timeout and
extracts the pass history (weight, subsolve outcome and iterations, BC error,
relative progress) from the debug log: at the baseline the incompatible scenes
do not stop. `--stage validate` runs them with the budget given by --budget
(and the compatible scene with and without it) and checks the named failure,
its exit status, the manifest's `al_stagnation` record, the RB-06 rollback and
that nothing of the failed step was published. Standard-library Python only.
Preserves every run; never overwrites the output directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
SCENES = ROOT / 'scenes/semi-implicit'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def pvd_frames(path):
    if not Path(path).exists():
        return None
    text = Path(path).read_text()
    return sorted(int(part.split('step_')[1].split('.vtm')[0]) for part in text.split('file="')[1:])


class Check:
    def __init__(self):
        self.rows = []
        self.failed = 0

    def __call__(self, run, name, ok, detail=''):
        self.rows.append(dict(run=run, check=name, passed=bool(ok), detail=detail))
        if not ok:
            self.failed += 1
        print(f"  [{'pass' if ok else 'FAIL'}] {name}{(': ' + str(detail)) if detail else ''}", flush=True)


def base_scene():
    config = json.loads((SCENES / 'quasistatic-semi.json').read_text())
    config['time'] = {'tend': 1.0, 'dt': 1.0, 'quasistatic': True}
    config['output']['paraview']['file_name'] = 'run.pvd'
    config['output'].update(stats=True, physical_diagnostics=True, manifest='run-manifest.json')
    return config


def scene_compatible():
    config = base_scene()
    config['boundary_conditions']['dirichlet_boundary'][0]['value'] = ['0', '0', '-0.3*t']
    config['solver']['augmented_lagrangian'] = {
        'initial_weight': 1e4, 'scaling': 2.0, 'max_weight': 1e8,
        'nonlinear': {'max_iterations': 4, 'allow_out_of_iterations': True}}
    return config


def scene_collision():
    config = base_scene()
    config['geometry'][0]['surface_selection'] = [{'id': 3, 'axis': '-z', 'position': 0.01}]
    config['boundary_conditions']['dirichlet_boundary'] = [{'id': 3, 'value': ['0', '0', '-0.05*t']}]
    return config


def scene_crush():
    config = base_scene()
    config['boundary_conditions']['dirichlet_boundary'][0]['value'] = ['0', '0', '-1.0*t']
    return config


SCENE_BUILDERS = {'compatible-multipass': scene_compatible,
                  'incompatible-collision': scene_collision,
                  'incompatible-crush': scene_crush}

PASS_RE = re.compile(r'Solving AL Problem with weight ([0-9.eE+-]+)')
FINISHED_RE = re.compile(r'Finished: (.*?) took .*?\(iters=(\d+)')
ERROR_RE = re.compile(r'Current error = ([0-9.eE+-]+|nan|inf)')
ETA_RE = re.compile(r'Current eta = ([0-9.eE+-]+|nan|-inf|inf)')
INITIAL_RE = re.compile(r'Initial error = ([0-9.eE+-]+)')
STALL_RE = re.compile(r'Line-search stall detected|Hard stall persists|stall persisted|stall persists')


def pass_history(log_path):
    """Per-pass records of the first step's AL stage from the debug log."""
    passes, current, initial = [], None, None
    for line in Path(log_path).read_text(errors='replace').splitlines():
        m = INITIAL_RE.search(line)
        if m and initial is None:
            initial = float(m.group(1))
        m = PASS_RE.search(line)
        if m:
            current = dict(weight=float(m.group(1)), subsolves=[], stall_events=0)
            passes.append(current)
            continue
        if current is None:
            continue
        if STALL_RE.search(line):
            current['stall_events'] += 1
        m = FINISHED_RE.search(line)
        if m:
            current['subsolves'].append(dict(reason=m.group(1), iterations=int(m.group(2))))
        m = ERROR_RE.search(line)
        if m:
            current['bc_error'] = float(m.group(1))
        m = ETA_RE.search(line)
        if m:
            current['eta'] = float(m.group(1))
            current = None  # the pass record is complete at its eta line
    return dict(initial_error=initial, passes=passes)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--binary', type=Path, default=ROOT / 'build/PolyFEM_bin')
    p.add_argument('--stage', choices=['reproduce', 'validate'], default='reproduce')
    p.add_argument('--timeout', type=float, default=600.0, help='wall-clock limit per run in seconds')
    p.add_argument('--budget', type=json.loads, default=None, help='solver/augmented_lagrangian/budget for --stage validate')
    p.add_argument('--scenes', nargs='*', default=None)
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    binary = args.binary.resolve()
    check = Check()
    results = {'binary_sha256': sha256(binary), 'stage': args.stage, 'timeout': args.timeout,
               'budget': args.budget, 'runs': [], 'checks': check.rows}

    def run(name, scene, budget=None):
        directory = out / name
        directory.mkdir()
        config = SCENE_BUILDERS[scene]()
        for asset in ('cube.mesh', 'slab.obj'):
            shutil.copy2(SCENES / asset, directory / asset)
        config['output']['directory'] = str(directory / 'output')
        if budget is not None:
            config['solver'].setdefault('augmented_lagrangian', {})['budget'] = budget
        (directory / 'params.json').write_text(json.dumps(config, indent=2) + '\n')
        cmd = [str(binary), '--json', str(directory / 'params.json'), '--log_level', 'debug', '--max_threads', '1']
        started = time.monotonic()
        timed_out = False
        with (directory / 'run.log').open('w') as log:
            try:
                proc = subprocess.run(cmd, cwd=directory, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
                code = proc.returncode
            except subprocess.TimeoutExpired:
                timed_out = True
                code = None
        history = pass_history(directory / 'run.log')
        result = dict(name=name, scene=scene, budget=budget, command=cmd, exit=code, timed_out=timed_out,
                      wall_seconds=time.monotonic() - started, al_passes=len(history['passes']),
                      initial_error=history['initial_error'], passes=history['passes'],
                      input_sha256={f.name: sha256(f) for f in directory.iterdir() if f.suffix in ('.json', '.obj', '.mesh')})
        results['runs'].append(result)
        (out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
        print(f"{name}: exit {code}{' (timed out)' if timed_out else ''} {result['wall_seconds']:.1f}s, AL passes {len(history['passes'])}", flush=True)
        return directory / 'output', result

    scenes = args.scenes or list(SCENE_BUILDERS)
    if args.stage == 'reproduce':
        for scene in scenes:
            output, result = run(scene, scene)
            for k, rec in enumerate(result['passes']):
                sub = rec['subsolves'][-1] if rec['subsolves'] else {}
                print(f"    pass {k + 1}: weight {rec['weight']:g} outcome '{sub.get('reason', '?')}' iters {sub.get('iterations', '?')} "
                      f"bc_error {rec.get('bc_error', float('nan')):.6g} eta {rec.get('eta', float('nan')):.4g} stalls {rec['stall_events']}")
    else:
        if args.budget is None:
            sys.exit('--stage validate needs --budget')
        if 'compatible-multipass' in scenes:
            # Off/on equivalence on a continuation the budget does not exhaust:
            # same passes, byte-identical endpoint, exit 0 both ways.
            control_output, control = run('compatible-multipass-off', 'compatible-multipass')
            check('compatible-multipass-off', 'exit 0', control['exit'] == 0, control['exit'])
            check('compatible-multipass-off', 'more than three AL passes', control['al_passes'] > 3, control['al_passes'])
            on_output, on = run('compatible-multipass-on', 'compatible-multipass', args.budget)
            check('compatible-multipass-on', 'exit 0', on['exit'] == 0, on['exit'])
            check('compatible-multipass-on', 'same number of AL passes', on['al_passes'] == control['al_passes'], (on['al_passes'], control['al_passes']))
            if (on_output / 'step_1.vtu').exists() and (control_output / 'step_1.vtu').exists():
                check('compatible-multipass-on', 'endpoint frame byte-identical to the budget-off run', sha256(on_output / 'step_1.vtu') == sha256(control_output / 'step_1.vtu'))
            manifest_on = on_output / 'run-manifest.json'
            if manifest_on.exists():
                steps = json.loads(manifest_on.read_text()).get('steps', [])
                check('compatible-multipass-on', 'accepted step with no al_stagnation record', bool(steps) and steps[0]['outcome'] == 'accepted' and steps[0].get('al_stagnation') is None)
        for scene in ('incompatible-collision', 'incompatible-crush'):
            if scene not in scenes:
                continue
            output, result = run(scene, scene, args.budget)
            check(scene, 'stopped before the timeout', not result['timed_out'], result['wall_seconds'])
            check(scene, 'exit status 1 (named failure)', result['exit'] == 1, result['exit'])
            manifest_path = output / 'run-manifest.json'
            check(scene, 'manifest written', manifest_path.exists())
            if manifest_path.exists():
                manifest = json.loads(manifest_path.read_text())
                check(scene, 'completion failed', manifest['completion']['status'] == 'failed', manifest['completion'].get('status'))
                steps = manifest.get('steps', [])
                check(scene, 'one step record, the failed attempt', len(steps) == 1 and steps[0]['outcome'] == 'failed_attempt', [s.get('outcome') for s in steps])
                if steps:
                    last = steps[-1]
                    check(scene, 'phase augmented_lagrangian', last.get('phase') == 'augmented_lagrangian', last.get('phase'))
                    stagnation = last.get('al_stagnation')
                    check(scene, 'al_stagnation record present', isinstance(stagnation, dict), stagnation if not isinstance(stagnation, dict) else stagnation.get('reason'))
                    if isinstance(stagnation, dict):
                        check(scene, 'passes within the configured budget', stagnation.get('passes', 10 ** 9) <= max(args.budget.get('max_passes', 10 ** 9) or 10 ** 9, 1), stagnation.get('passes'))
                        check(scene, 'history has one record per pass plus the start', len(stagnation.get('history', [])) == stagnation.get('passes', -1) + 1, len(stagnation.get('history', [])))
                        check(scene, 'last pass: snap not feasible, blocking gate named', stagnation.get('last_pass', {}).get('gate', {}).get('feasible') is False and isinstance(stagnation.get('last_pass', {}).get('gate', {}).get('blocked_by'), str), stagnation.get('last_pass', {}).get('gate'))
                        if stagnation.get('reason') == 'stagnation':
                            check(scene, 'stagnation: no progress signal over the window', not any(stagnation.get('progress', {}).values()), stagnation.get('progress'))
                            check(scene, 'stagnation: the window ran at the weight ceiling', all(r.get('at_ceiling') for r in stagnation['history'][-stagnation['window']:]), stagnation.get('window'))
                    rollback = last.get('rollback', {})
                    check(scene, 'rollback performed and verified', rollback.get('performed') is True and rollback.get('verified') is True, rollback)
                    check(scene, 'error names the AL stage', 'augmented' in str(last.get('error', '')).lower() or 'AL ' in str(last.get('error', '')), last.get('error'))
            check(scene, 'no frame of the failed step', not (output / 'step_1.vtu').exists())
            check(scene, 'PVD lists the initial frame only', pvd_frames(output / 'run.pvd') in ([0], None), pvd_frames(output / 'run.pvd'))
            records_path = output / 'physical-diagnostics.jsonl'
            if records_path.exists():
                records = read_jsonl(records_path)
                check(scene, 'RB-04 failure record with rollback announced', bool(records) and records[-1]['outcome'] == 'failed_attempt' and records[-1].get('rollback', {}).get('performed') == 'after this record')
    results['failed_checks'] = check.failed
    (out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    print(f"{check.failed} failed check(s)" if check.rows else 'done')
    return 1 if check.failed else 0


if __name__ == '__main__':
    sys.exit(main())
