"""RB-04 fixed-dhat, one-axis load/time refinement with durable partial results.

Endpoint dot products define discrete work estimates, not continuum certification.
Usage: python3 tools/rb04/run_refinement.py --scene quasistatic-semi --output /fresh/path
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/pf08'))
from measure_fem import measure


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(directory, dt, scene):
    output = directory/'output'
    path = output/'physical-diagnostics.jsonl'
    records = [json.loads(s) for s in path.read_text().splitlines()] if path.exists() else []
    events_path = output/'coefficient-events.jsonl'
    events = [json.loads(s) for s in events_path.read_text().splitlines()] if events_path.exists() else []
    result = {'frames': [], 'initial_state_verified': False}
    if not records or not events:
        result['unavailable_reason'] = 'No endpoint or coefficient-event records'
        return result
    first = events[0]
    prev_x = np.asarray(first['coordinates'])
    # These selected fixtures start undeformed and at rest without body loading.
    assert np.count_nonzero(prev_x) == 0
    assert first['after']['objective'] == 0
    assert np.count_nonzero(first['after']['gradient_objective']) == 0
    initial = measure(output/'step_0.vtu', 1e7, .45, 0)
    assert abs(initial['elastic_energy']) < 1e-10
    result['initial_state_verified'] = True
    prev_reaction = np.zeros_like(prev_x)
    prev_gradients = {}
    work_right = work_trapezoid = 0.
    costs_right, costs_trapezoid = {}, {}
    for record in records:
        if record['outcome'] != 'accepted':
            result['failure_record'] = record
            continue
        assert record['residual_complete'] and 'measurement_error' not in record
        step = record['step']
        reference = measure(output/f'step_{step}.vtu', 1e7, .45, step*dt)
        assert abs(reference['elastic_energy']-record['elastic_energy']['value'])/(1+abs(reference['elastic_energy'])) < 1e-9
        assert abs(reference['min_det_F']-record['min_det_F']['value']) < 1e-10
        assert abs(reference['bc_error']-record['bc_error_inf']['value']) < 1e-10
        x = np.asarray(record['endpoint']['value'])
        dx = x-prev_x
        assert len(record['reactions']) == 1
        reaction = np.asarray(record['reactions'][0]['full_dof_vector']['value'])
        gradients = {f['name']: np.asarray(f['gradient_force_units']['value']) for f in record['forms']}
        # No unaccounted distributed loading in this bounded fixture protocol.
        for name, gradient in gradients.items():
            if name not in ('elastic', 'barrier-contact', 'friction', 'inertia'):
                assert np.linalg.norm(gradient) < 1e-10, name
        work_right += float(reaction@dx)
        work_trapezoid += float(.5*(reaction+prev_reaction)@dx)
        for name, gradient in gradients.items():
            costs_right[name] = costs_right.get(name, 0.)+float(gradient@dx)
            costs_trapezoid[name] = costs_trapezoid.get(name, 0.)+float(.5*(gradient+prev_gradients.get(name, np.zeros_like(gradient)))@dx)
        c = record['contact']
        post = [e for e in events if e['step'] == step and e['phase'] == 'between_steps_after_endpoint']
        result['frames'].append(dict(step=step, time=step*dt,
            elastic_energy=record['elastic_energy']['value'], barrier_energy=record['barrier_energy']['value'],
            kinetic_energy=record['kinetic_energy']['value'],
            free_residual=record['free_residual_norm']['value'], bc_error=record['bc_error_inf']['value'],
            min_det_F=record['min_det_F']['value'],
            min_gap_over_dhat=c['min_gap']['value']/c['dhat'] if c['min_gap']['value'] is not None else None,
            gap_scope=c['gap_scope'], active_count=c['active_count'],
            trim=c['trim_or_global_stiffness'], refresh_id=c['refresh_id'],
            reaction_z_elastic=reference['top_reaction'][2],
            prescribed_work_right=work_right, prescribed_work_trapezoid=work_trapezoid,
            component_costs_right=dict(costs_right), component_costs_trapezoid=dict(costs_trapezoid),
            prescribed_work_minus_component_costs_right=work_right-sum(costs_right.values()),
            elastic_right_quadrature_excess=costs_right['elastic']-reference['elastic_energy'],
            lagging=record['lagging'],
            post_endpoint_free_contact_force_change=post[-1]['free_contact_force_change_norm'] if post else None))
        prev_x, prev_reaction, prev_gradients = x, reaction, gradients
    result['last_step'] = result['frames'][-1]['step'] if result['frames'] else 0
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene', choices=['quasistatic-semi', 'transient-semi', 'quasistatic-semi-friction'], required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--lower', nargs='+', type=float, choices=[.1, .5, .8], default=[.1, .5, .8])
    p.add_argument('--dt', nargs='+', type=float, choices=[.25, .125, .0625], default=[.25, .125, .0625])
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out/'runner-at-start.py').write_bytes(Path(__file__).read_bytes())
    binary = ROOT/'build/PolyFEM_bin'
    result = {'binary_sha256': sha(binary), 'runner_sha256': sha(Path(__file__)),
        'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'scene': args.scene, 'protocol': {'lower': args.lower, 'upper': .9, 'dt': args.dt, 'timeout_seconds': 120},
        'work_convention': 'Right endpoint and trapezoidal gradient dot physical nodal increment; resistance costs use +gradient. Friction uses returned updated-lag state, not necessarily solve lag. Event energy sums are not included.',
        'limits': 'Single mesh/dhat; no feature-jump or exact contact-work integral; no complete physical balance or chosen engineering tolerance.', 'runs': []}
    def save():
        (out/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    save()
    for lower in args.lower:
        for dt in args.dt:
            directory = out/f'lower-{lower:g}-dt-{dt:g}'
            directory.mkdir()
            config = json.loads((ROOT/'scenes/semi-implicit'/f'{args.scene}.json').read_text())
            config['time']['dt'] = dt
            config['solver']['contact'].setdefault('semi_implicit', {}).update(trim_lower=lower, trim_upper=.9)
            config['output'].update(directory=str(directory/'output'), stats=True, physical_diagnostics=True)
            for asset in ('cube.mesh', 'slab.obj'):
                shutil.copy2(ROOT/'scenes/semi-implicit'/asset, directory/asset)
            (directory/'params.json').write_text(json.dumps(config, indent=2)+'\n')
            row = dict(lower=lower, upper=.9, dt=dt, directory=directory.name,
                input_sha256={p.name: sha(p) for p in directory.iterdir()}, status='running')
            result['runs'].append(row)
            command = [str(binary), '--json', str(directory/'params.json'), '--log_level', 'debug']
            row['command'] = command
            save()
            start = time.monotonic()
            with (directory/'run.log').open('w') as log:
                try:
                    row['exit'] = subprocess.run(command, cwd=directory, stdout=log, stderr=subprocess.STDOUT, timeout=120).returncode
                    row['status'] = 'exited'
                except subprocess.TimeoutExpired:
                    row.update(exit=None, status='timed_out')
            row['wall_seconds'] = time.monotonic()-start
            save()
            try:
                row['measurements'] = analyze(directory, dt, args.scene)
                row['complete'] = row['exit'] == 0 and row['measurements'].get('last_step') == round(1/dt)
            except Exception as e:
                row['analysis_error'] = repr(e)
                row['complete'] = False
            save()
            print(args.scene, lower, dt, row['status'], row['exit'], 'complete', row['complete'], row.get('analysis_error', ''), flush=True)


if __name__ == '__main__':
    main()
