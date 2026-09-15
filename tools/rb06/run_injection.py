"""RB-06 end-to-end checks of the failed-attempt rollback through PolyFEM_bin.

python3 tools/rb06/run_injection.py --output /absolute/fresh/evidence [--binary build/PolyFEM_bin]

Runs the public transient and friction fixtures single-threaded with the
diagnostics and the run manifest on, once as a control and once per failure
injection (solver/advanced/failure_injection), plus the RB-04 real-failure
fixture (a stall budget that exhausts its restarts). For every failing run it
checks the exit status, that the manifest records the failed attempt with a
performed and verified rollback, that nothing of the failed step was
published (no frame, no PVD entry, no energy row), that the RB-04 failure
record announces the rollback and carries the restored coordinates (= the
previous accepted endpoint), that the coefficient-event stream ends the step
with the rollback event, and that every frame written before the failure is
byte-identical to the control's. Standard-library Python only. Preserves
every run; never overwrites the output directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
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
    text = Path(path).read_text()
    return sorted(int(part.split('step_')[1].split('.vtm')[0]) for part in text.split('file="')[1:])


def csv_rows(path):
    return [line for line in Path(path).read_text().splitlines() if line.strip()][1:]


class Check:
    def __init__(self):
        self.rows = []
        self.failed = 0

    def __call__(self, run, name, ok, detail=''):
        self.rows.append(dict(run=run, check=name, passed=bool(ok), detail=detail))
        if not ok:
            self.failed += 1
        print(f"  [{'pass' if ok else 'FAIL'}] {name}{(': ' + str(detail)) if detail else ''}", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--binary', type=Path, default=ROOT / 'build/PolyFEM_bin')
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    binary = args.binary.resolve()
    check = Check()
    results = {'binary_sha256': sha256(binary), 'runs': [], 'checks': check.rows}

    def run(name, scene, injection=None, extra=None):
        directory = out / name
        directory.mkdir()
        config = json.loads((SCENES / (scene + '.json')).read_text())
        for asset in ('cube.mesh', 'slab.obj'):
            shutil.copy2(SCENES / asset, directory / asset)
        config['output'].update(directory=str(directory / 'output'), stats=True, physical_diagnostics=True)
        config['output']['paraview']['file_name'] = 'run.pvd'
        if injection is not None:
            config['solver'].setdefault('advanced', {})['failure_injection'] = injection
        for pointer, value in (extra or {}).items():
            node = config
            keys = pointer.strip('/').split('/')
            for key in keys[:-1]:
                node = node.setdefault(key, {})
            node[keys[-1]] = value
        (directory / 'params.json').write_text(json.dumps(config, indent=2) + '\n')
        cmd = [str(binary), '--json', str(directory / 'params.json'), '--log_level', 'debug', '--max_threads', '1']
        started = time.monotonic()
        with (directory / 'run.log').open('w') as log:
            proc = subprocess.run(cmd, cwd=directory, stdout=log, stderr=subprocess.STDOUT)
        result = dict(name=name, scene=scene, injection=injection, extra=extra, command=cmd, exit=proc.returncode,
                      wall_seconds=time.monotonic() - started,
                      input_sha256={f.name: sha256(f) for f in directory.iterdir() if f.suffix in ('.json', '.obj', '.mesh')})
        results['runs'].append(result)
        (out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
        print(name, 'exit', proc.returncode, f'{result["wall_seconds"]:.1f}s', flush=True)
        return directory / 'output', result

    def frames(directory):
        return {f.name: sha256(f) for f in directory.glob('step_*.vtu')}

    def check_failed_attempt(name, output, control_output, step, expect_exit=1):
        """A failed solve attempt of `step`: rolled back, nothing published."""
        manifest = json.loads((output / 'run-manifest.json').read_text())
        check(name, 'exit status', results['runs'][-1]['exit'] == expect_exit, results['runs'][-1]['exit'])
        check(name, 'manifest completion failed', manifest['completion']['status'] == 'failed' and manifest['completion']['exit_status'] == expect_exit)
        steps = manifest['steps']
        check(name, f'manifest has {step} step records', len(steps) == step, len(steps))
        last = steps[-1]
        check(name, 'last step is the failed attempt', last['step'] == step and last['outcome'] == 'failed_attempt', (last['step'], last['outcome']))
        rollback = last.get('rollback', {})
        check(name, 'rollback performed and verified', rollback.get('performed') is True and rollback.get('verified') is True, rollback)
        check(name, 'no frame of the failed step', not (output / f'step_{step}.vtu').exists() and not (output / f'step_{step}.vtm').exists())
        check(name, 'PVD lists frames before the failure only', pvd_frames(output / 'run.pvd') == list(range(step)), pvd_frames(output / 'run.pvd'))
        check(name, 'energy rows before the failure only', len(csv_rows(output / 'energy.csv')) == step, len(csv_rows(output / 'energy.csv')))
        records = read_jsonl(output / 'physical-diagnostics.jsonl')
        check(name, 'RB-04 records: accepted steps then the failure', [r['outcome'] for r in records] == ['accepted'] * (step - 1) + ['failed_attempt'])
        failure = records[-1]
        check(name, 'failure record announces the rollback', failure.get('rollback', {}).get('performed') == 'after this record')
        previous = records[-2]['endpoint']['value'] if step > 1 else None
        start = failure.get('solve_start', {}).get('value')
        if previous is not None:
            check(name, 'restored coordinates = previous accepted endpoint', start == previous)
        else:
            check(name, 'restored coordinates recorded', isinstance(start, list) and len(start) > 0)
        events = read_jsonl(output / 'coefficient-events.jsonl')
        step_events = [e for e in events if e['step'] == step]
        check(name, 'coefficient stream ends the step with the rollback event', bool(step_events) and step_events[-1]['operation'] == 'rollback' and sum(e['operation'] == 'rollback' for e in events) == 1)
        if control_output is not None:
            mine, theirs = frames(output), frames(control_output)
            same = all(mine[f] == theirs.get(f) for f in mine)
            check(name, 'frames before the failure identical to the control', same and set(mine) == {f'step_{i}.vtu' for i in range(step)}, sorted(mine))
        return manifest

    def check_publication_failure(name, output, control_output, step, after):
        manifest = json.loads((output / 'run-manifest.json').read_text())
        check(name, 'exit status', results['runs'][-1]['exit'] == 1, results['runs'][-1]['exit'])
        check(name, 'manifest completion failed', manifest['completion']['status'] == 'failed')
        last = manifest['steps'][-1]
        check(name, 'the solve of the step was accepted', last['step'] == step and last['outcome'] == 'accepted' and 'rollback' not in last)
        published = list(range(step + 1)) if after else list(range(step))
        check(name, 'PVD frames', pvd_frames(output / 'run.pvd') == published, pvd_frames(output / 'run.pvd'))
        check(name, 'frame files', (output / f'step_{step}.vtu').exists() == after)
        check(name, 'energy rows', len(csv_rows(output / 'energy.csv')) == len(published), len(csv_rows(output / 'energy.csv')))
        mine, theirs = frames(output), frames(control_output)
        check(name, 'published frames identical to the control', all(mine[f] == theirs.get(f) for f in mine) and set(mine) == {f'step_{i}.vtu' for i in published}, sorted(mine))

    # Controls.
    transient_control, _ = run('transient-control', 'transient-semi')
    check('transient-control', 'exit status 0', results['runs'][-1]['exit'] == 0)
    friction_control, _ = run('friction-control', 'quasistatic-semi-friction')
    check('friction-control', 'exit status 0', results['runs'][-1]['exit'] == 0)
    restart = {'enabled': True, 'soft_iteration_limit': 4, 'min_iterations': 0, 'max_restarts': 8}
    stall_control, _ = run('stall-control', 'transient-semi', extra={'/solver/contact/semi_implicit/restart': restart})
    check('stall-control', 'exit status 0', results['runs'][-1]['exit'] == 0)

    # Injected failures of step 2 in every solve phase.
    for name, scene, injection, control in (
            ('after-al', 'transient-semi', {'phase': 'after_al', 'step': 2}, transient_control),
            ('reduced-iteration-1', 'transient-semi', {'phase': 'reduced', 'step': 2, 'iteration': 1}, transient_control),
            ('reduced-runtime-error', 'transient-semi', {'phase': 'reduced', 'step': 2, 'iteration': 1, 'kind': 'runtime_error'}, transient_control),
            ('lagging-1', 'quasistatic-semi-friction', {'phase': 'lagging', 'step': 2, 'iteration': 1}, friction_control)):
        output, _ = run(name, scene, injection)
        check_failed_attempt(name, output, control, 2)
    output, _ = run('after-stall-retune', 'transient-semi', {'phase': 'after_stall_retune', 'step': 2},
                    extra={'/solver/contact/semi_implicit/restart': restart})
    manifest = check_failed_attempt('after-stall-retune', output, stall_control, 2)
    check('after-stall-retune', 'the step had at least one stall retune', manifest['steps'][-1].get('stall_retunes', 0) >= 1, manifest['steps'][-1].get('stall_retunes'))

    # Publication failures of an accepted solve.
    output, _ = run('before-publication', 'transient-semi', {'phase': 'before_publication', 'step': 2})
    check_publication_failure('before-publication', output, transient_control, 2, after=False)
    output, _ = run('after-publication', 'transient-semi', {'phase': 'after_publication', 'step': 2})
    check_publication_failure('after-publication', output, transient_control, 2, after=True)

    # A real (not injected) failure: the RB-04 fixture exhausts its restart
    # budget and the reduced solve reports non-convergence at step 1.
    output, _ = run('real-failure', 'quasistatic-semi', extra={'/solver/contact/semi_implicit/restart': {
        'enabled': True, 'soft_iteration_limit': 1, 'max_restarts': 1, 'min_iterations': 0}})
    manifest = check_failed_attempt('real-failure', output, None, 1)
    check('real-failure', 'named non-convergence failure', 'did not converge' in (manifest['steps'][-1].get('error') or ''), manifest['steps'][-1].get('error'))

    results['passed'] = check.failed == 0
    results['failed_checks'] = check.failed
    (out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    print('passed' if results['passed'] else f'{check.failed} check(s) FAILED', flush=True)
    return 0 if results['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
