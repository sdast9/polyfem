"""RB-04 bounded public on/off and failed-attempt checks; preserve every run.

python3 tools/rb04/run_endpoints.py --output /absolute/fresh/evidence
Requires NumPy and an already built PolyFEM_bin. No private scenes.
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
sys.path.insert(0, str(ROOT / 'tools/pf08'))
sys.path.insert(0, str(ROOT / 'tools/rb04'))
from measure_fem import arrays, measure
from check_solver_attempts import check_run as check_attempts


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--contact-path', action='store_true', help='Enable expensive path observations in diagnostic-on runs')
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    binary = ROOT / 'build/PolyFEM_bin'
    results = {'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(), 'runs': []}
    def run(name, enabled, failure=False):
        directory = out / (name + ('-failure' if failure else '') + ('-on' if enabled else '-off'))
        directory.mkdir()
        config = json.loads((ROOT / 'scenes/semi-implicit' / (name + '.json')).read_text())
        for asset in ('cube.mesh', 'slab.obj'):
            shutil.copy2(ROOT / 'scenes/semi-implicit' / asset, directory / asset)
        config['output']['paraview']['options'] = {'velocity': True, 'acceleration': True}
        config['output'].update(directory=str(directory / 'output'), stats=True, physical_diagnostics=enabled)
        if args.contact_path:
            config['output']['physical_diagnostics_contact_path'] = enabled
        if failure:
            # Existing opt-in test budget, not a production default change.
            config['solver']['contact']['semi_implicit'] = {'restart': {
                'soft_iteration_limit': 1, 'max_restarts': 1, 'min_iterations': 0}}
        (directory / 'params.json').write_text(json.dumps(config, indent=2) + '\n')
        cmd = [str(binary), '--json', str(directory / 'params.json'), '--log_level', 'debug']
        started = time.monotonic()
        with (directory / 'run.log').open('w') as log:
            proc = subprocess.run(cmd, cwd=directory, stdout=log, stderr=subprocess.STDOUT)
        result = dict(name=directory.name, command=cmd, exit=proc.returncode,
                      wall_seconds=time.monotonic()-started,
                      input_sha256={f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                                    for f in directory.iterdir() if f.suffix in ('.json', '.obj', '.mesh')})
        results['runs'].append(result)
        (out / 'results.json').write_text(json.dumps(results, indent=2)+'\n')
        print(directory.name, proc.returncode, flush=True)
        return directory, result

    for name in ('quasistatic-semi', 'transient-semi', 'quasistatic-semi-friction'):
        off, off_result = run(name, False)
        on, on_result = run(name, True)
        assert off_result['exit'] == on_result['exit'] == 0
        assert not (off / 'output/physical-diagnostics.jsonl').exists()
        records = [json.loads(line) for line in (on / 'output/physical-diagnostics.jsonl').read_text().splitlines()]
        assert len(records) == 4
        checks = []
        cumulative = dict(external_work_increment=0., frictional_dissipation_increment=0., retuning_energy_change=0.)
        previous_barrier_energy = None
        for row in records:
            assert row['outcome'] == 'accepted' and row['version'] == 2
            assert 'measurement_error' not in row, row.get('measurement_error')
            assert row['residual_complete']
            step = row['step']
            a = arrays(off / 'output' / f'step_{step}.vtu')
            b = arrays(on / 'output' / f'step_{step}.vtu')
            # Predeclared 1e-10 displacement threshold. Byte identity is reported separately.
            difference = float(np.max(np.abs(a['displacement']-b['displacement'])))
            assert difference < 1e-10, difference
            physical = measure(on / 'output' / f'step_{step}.vtu', 1e7, .45, step*.25)
            elastic = next(f for f in row['forms'] if f['name'] == 'elastic')
            measured_e = elastic['objective_divided_by_acceleration_scaling']['value']
            e_error = abs(physical['elastic_energy']-measured_e)/(1+abs(measured_e))
            assert e_error < 1e-9, e_error
            assert abs(physical['min_det_F']-row['min_det_F']['value']) < 1e-10
            assert abs(physical['bc_error']-row['bc_error_inf']['value']) < 1e-10
            # The prescribed top nodes have exactly (0,0,-.25*t) displacement;
            # no top contact/traction in this fixture. Compare full diagnostic reactions.
            endpoint = np.asarray(row['endpoint']['value']).reshape(-1, 3)
            top = np.all(endpoint == np.array([0., 0., -.25*step*.25]), axis=1)
            reaction = np.asarray(row['reactions'][0]['full_dof_vector']['value']).reshape(-1, 3)[top].sum(axis=0)
            reaction_error = np.linalg.norm(reaction-physical['top_reaction'])/1e7
            # Transient reactions also include inertia, checked below via kinetic energy.
            if name != 'transient-semi':
                assert reaction_error < 1e-9, reaction_error
            # Version 2: the VTU kinematics are those of the saved endpoint
            # (ImplicitEuler differences of the saved displacements), no
            # longer the integrator history head of the previous step.
            previous = arrays(on / 'output' / f'step_{step-1}.vtu')
            endpoint_velocity = (b['displacement']-previous['displacement'])/.25
            vtu_velocity_error = float(np.max(np.abs(b['velocity']-endpoint_velocity)))
            assert vtu_velocity_error < 1e-10, vtu_velocity_error
            previous_velocity = previous['velocity'] if step > 1 else np.zeros_like(endpoint_velocity)
            vtu_acceleration_error = float(np.max(np.abs(b['acceleration']-(endpoint_velocity-previous_velocity)/.25)))
            assert vtu_acceleration_error < 1e-9, vtu_acceleration_error
            if name == 'transient-semi':
                kinetic = 0.
                starts = np.r_[0, b['offsets'][:-1]]
                for start, end, kind in zip(starts, b['offsets'], b['types']):
                    if kind != 10: continue
                    tet = b['connectivity'][start:end]
                    X, v = b['points'][tet], endpoint_velocity[tet]
                    volume = abs(np.linalg.det((X[1:]-X[0]).T))/6
                    # Exact P1 consistent mass integral: integral Ni Nj = V(1+deltaij)/20.
                    kinetic += .5*1000*volume/20*(np.sum(v*v)+np.sum(v.sum(axis=0)**2))
                kinetic_error = abs(kinetic-row['kinetic_energy']['value'])/(1+kinetic)
                assert kinetic_error < 1e-9, kinetic_error
            else:
                assert row['kinetic_energy']['value'] is None
                kinetic_error = None
            components = np.array([f['gradient_force_units']['value'] for f in row['forms']])
            assert np.linalg.norm(components.sum(axis=0)-row['full_residual']['value']) < 1e-8
            # Version 2 right-endpoint increments (docs/rb-04-work-convention.md).
            assert row['physical_balance_pass']['value'] is None and row['physical_balance_pass']['unavailable_reason']
            dx = np.asarray(row['accepted_displacement']['value'])
            support = sum(np.dot(r['full_dof_vector']['value'], dx) for r in row['reactions'])
            assert abs(row['support_work_increment']['value']-support) < 1e-8*(1+abs(support))
            assert row['body_load_work_increment']['value'] == 0  # no body/traction load in the public fixtures
            assert row['external_work_increment']['value'] == row['support_work_increment']['value']+row['body_load_work_increment']['value']
            # Independent support work: the reconstructed elastic top reaction
            # times the prescribed increment (0, 0, -.0625). Transient reactions
            # include inertia, so only the quasistatic fixtures compare.
            independent_support_work = float(np.dot(physical['top_reaction'], [0., 0., -.0625])) if name != 'transient-semi' else None
            support_work_error = (abs(row['support_work_increment']['value']-independent_support_work)/(1+abs(independent_support_work))
                                  if independent_support_work is not None else None)
            if support_work_error is not None:
                assert support_work_error < 1e-8, support_work_error
            friction = row['frictional_dissipation_increment']
            if name == 'quasistatic-semi-friction':
                g = np.asarray(row['lagging']['friction_before_update']['gradient_force_units'])
                assert abs(friction['value']-np.dot(g, dx)) < 1e-8*(1+abs(friction['value']))
            else:
                assert friction['value'] == 0 and 'No friction form' in friction['convention']
            start_energy = row['barrier_start_energy_with_endpoint_snapshot']['value']
            if previous_barrier_energy is None:
                # First step: theta_(n-1) is the state at solve start; the fixture starts out of contact.
                assert row['barrier_energy_at_solve_start']['value'] == 0
                previous_barrier_energy = row['barrier_energy_at_solve_start']['value']
            assert abs(row['previous_endpoint_barrier_energy']['value']-previous_barrier_energy) <= 1e-12*(1+abs(previous_barrier_energy))
            retune = start_energy-previous_barrier_energy
            assert abs(row['retuning_energy_change']['value']-retune) <= 1e-9*(1+abs(retune))
            previous_barrier_energy = row['barrier_energy']['value']
            for key, cumulative_key in (('external_work_increment', 'external_work_cumulative'),
                                        ('frictional_dissipation_increment', 'frictional_dissipation_cumulative'),
                                        ('retuning_energy_change', 'retuning_energy_change_cumulative')):
                cumulative[key] += row[key]['value']
                assert abs(row[cumulative_key]['value']-cumulative[key]) <= 1e-9*(1+abs(cumulative[key]))
            assert row['attempt_summary']['accepted_iterations'] > 0
            assert row['contact']['candidate_count']['value'] > 0
            checks.append(dict(step=step, displacement_on_off_max=difference,
                               elastic_energy_relative_error=e_error,
                               top_reaction_error_over_E=reaction_error,
                               kinetic_energy_relative_error=kinetic_error,
                               vtu_velocity_error=vtu_velocity_error,
                               vtu_acceleration_error=vtu_acceleration_error,
                               free_residual_norm=row['free_residual_norm']['value'],
                               min_det_F=row['min_det_F']['value'],
                               bc_error=row['bc_error_inf']['value'],
                               lagging=row['lagging'],
                               termination=row['termination'].get('termination_reason'),
                               kinetic_energy=row['kinetic_energy'],
                               support_work_increment=row['support_work_increment']['value'],
                               independent_support_work=independent_support_work,
                               support_work_error=support_work_error,
                               frictional_dissipation_increment=friction['value'],
                               retuning_energy_change=row['retuning_energy_change']['value'],
                               barrier_energy=row['barrier_energy']['value'],
                               barrier_energy_at_solve_start=row['barrier_energy_at_solve_start']['value'],
                               attempt_summary=row['attempt_summary'],
                               proposed_displacement=row['proposed_displacement']))
        on_result['checks'] = checks
        on_result['attempts'] = check_attempts(on)
    off, a = run('quasistatic-semi', False, True)
    on, b = run('quasistatic-semi', True, True)
    assert a['exit'] != 0 and a['exit'] == b['exit']
    rows = [json.loads(line) for line in (on / 'output/physical-diagnostics.jsonl').read_text().splitlines()]
    assert rows[-1]['outcome'] == 'failed_attempt' and rows[-1]['version'] == 2
    assert rows[-1]['accepted_displacement']['value'] is None
    assert rows[-1]['error'] and rows[-1]['termination']['exception']
    assert rows[-1]['termination']['subsolve_state_at_failure']['restarts'] == 1
    assert rows[-1]['termination']['subsolve_state_at_failure']['outcome'] == 'interrupted'
    assert 'Retained caller' in rows[-1]['coordinate_state']
    assert not list((on/'output').glob('step_1.vtu'))
    # Version 2: the failed attempt exposes its last internal Newton iterate,
    # the stall retune and both minimize calls; increments stay unavailable.
    iterate = rows[-1]['last_internal_iterate']
    assert iterate['value'] is not None and iterate['iteration'] >= 1 and iterate['minimize_index'] == 2
    assert iterate['value'] != rows[-1]['endpoint']['value']
    assert rows[-1]['attempt_summary']['stall_retunes'] == 1 and rows[-1]['attempt_summary']['minimize_calls'] == 2
    for key in ('external_work_increment', 'frictional_dissipation_increment', 'retuning_energy_change'):
        assert rows[-1][key]['value'] is None and rows[-1][key]['unavailable_reason']
    b['failure_record'] = rows[-1]
    b['attempts'] = check_attempts(on)
    results['passed'] = True
    (out / 'results.json').write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
