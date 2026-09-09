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
from measure_fem import arrays, measure


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
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
        config['output']['paraview']['options'] = {'velocity': True}
        config['output'].update(directory=str(directory / 'output'), stats=True, physical_diagnostics=enabled)
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
        for row in records:
            assert row['outcome'] == 'accepted' and row['version'] == 1
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
            if name == 'transient-semi':
                kinetic = 0.
                previous = arrays(on / 'output' / f'step_{step-1}.vtu')
                # Public fixture uses ImplicitEuler. Legacy VTU velocity is v_prev
                # because nonlinear output precedes integrator advancement.
                endpoint_velocity = (b['displacement']-previous['displacement'])/.25
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
            for key in ('external_work', 'retuning_energy_change', 'frictional_dissipation'):
                assert row[key]['value'] is None and row[key]['unavailable_reason']
            checks.append(dict(step=step, displacement_on_off_max=difference,
                               elastic_energy_relative_error=e_error,
                               top_reaction_error_over_E=reaction_error,
                               kinetic_energy_relative_error=kinetic_error,
                               free_residual_norm=row['free_residual_norm']['value'],
                               min_det_F=row['min_det_F']['value'],
                               bc_error=row['bc_error_inf']['value'],
                               lagging=row['lagging'],
                               termination=row['termination'].get('termination_reason'),
                               kinetic_energy=row['kinetic_energy']))
        on_result['checks'] = checks
    off, a = run('quasistatic-semi', False, True)
    on, b = run('quasistatic-semi', True, True)
    assert a['exit'] != 0 and a['exit'] == b['exit']
    rows = [json.loads(line) for line in (on / 'output/physical-diagnostics.jsonl').read_text().splitlines()]
    assert rows[-1]['outcome'] == 'failed_attempt'
    assert rows[-1]['accepted_displacement']['value'] is None
    assert rows[-1]['error'] and rows[-1]['termination']['exception']
    assert rows[-1]['termination']['subsolve_state_at_failure']['restarts'] == 1
    assert rows[-1]['termination']['subsolve_state_at_failure']['outcome'] == 'interrupted'
    assert 'Retained caller' in rows[-1]['coordinate_state']
    assert not list((on/'output').glob('step_1.vtu'))
    b['failure_record'] = rows[-1]
    results['passed'] = True
    (out / 'results.json').write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
