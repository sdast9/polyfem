"""Common-coefficient contact storage and paired friction-state work estimates."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def analyze(run):
    cfg = json.loads((run/'params.json').read_text())
    records = [json.loads(s) for s in (run/'output/physical-diagnostics.jsonl').read_text().splitlines()]
    events = [json.loads(s) for s in (run/'output/coefficient-events.jsonl').read_text().splitlines()]
    result = dict(run=run.name, evidence_subpath=str(Path(run.parent.name)/run.name), dt=cfg['time']['dt'],
        quasistatic=cfg['time'].get('quasistatic', False),
        friction_coefficient=cfg['contact'].get('friction_coefficient', 0.),
        binary_sha256=json.loads((run.parent/'results.json').read_text())['binary_sha256'], frames=[])
    prev_x = np.asarray(events[0]['coordinates'])
    assert np.count_nonzero(prev_x) == 0 and events[0]['after']['objective'] == 0
    prev_barrier = parameter_change = motion_change = 0.
    before_work = after_work = 0.
    for r in records:
        if r['outcome'] != 'accepted':
            result['failure'] = {k: r[k] for k in ('step', 'outcome', 'error')}
            continue
        x = np.asarray(r['endpoint']['value'])
        dx = x-prev_x
        start_common = r['barrier_start_energy_with_endpoint_snapshot']['value']
        endpoint_barrier = r['barrier_energy']['value']
        parameter_change += start_common-prev_barrier
        motion_change += endpoint_barrier-start_common
        assert abs(parameter_change+motion_change-endpoint_barrier) < 1e-9*(1+abs(endpoint_barrier))
        frame = dict(step=r['step'], barrier_energy=endpoint_barrier,
            barrier_start_energy_with_endpoint_snapshot=start_common,
            accumulated_parameter_energy_change_at_physical_starts=parameter_change,
            accumulated_fixed_snapshot_motion_energy_change=motion_change,
            free_residual_post_update=r['free_residual_norm']['value'],
            free_residual_pre_update_friction=r['free_residual_norm_with_pre_update_friction']['value'])
        # Independent route when H/surface/cap snapshot stays frozen from the
        # initial refresh to this endpoint: only global trim can change.
        starts = [e for e in events if e['step'] == r['step'] and e['operation'] == 'refresh'
                  and e['phase'] != 'between_steps_after_endpoint' and np.array_equal(e['coordinates'], prev_x)]
        if starts:
            initial = starts[0]
            a, b = initial['after']['state'], r['contact']
            if a['refresh_id'] == b['refresh_id']:
                expected = initial['after']['objective']/initial['acceleration_scaling']*b['trim_or_global_stiffness']/a['trim_or_global_stiffness']
                error = start_common-expected
                assert abs(error) < 1e-9*(1+abs(expected))
                frame['common_snapshot_reference_error'] = error
        lag = r['lagging']
        if 'friction_before_update' in lag and 'gradient_force_units' in lag['friction_before_update']:
            before = np.asarray(lag['friction_before_update']['gradient_force_units'])
            after = np.asarray(lag['friction_after_update']['gradient_force_units'])
            endpoint = np.asarray(next(f for f in r['forms'] if f['name'] == 'friction')['gradient_force_units']['value'])
            assert np.linalg.norm(after-endpoint) < 1e-9*(1+np.linalg.norm(endpoint))
            before_work += float(before@dx)
            after_work += float(after@dx)
            frame.update(accumulated_friction_work_before_update=before_work,
                         accumulated_friction_work_after_update=after_work,
                         lagging_state=lag['state'])
        result['frames'].append(frame)
        prev_x, prev_barrier = x, endpoint_barrier
    result['complete'] = len(result['frames']) == round(cfg['time']['tend']/cfg['time']['dt'])
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('runs', nargs='+', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    result = {'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'convention': 'Change coefficient state at prior physical coordinates, then move at fixed endpoint snapshot. Motion energy is not asserted equal to a gradient line integral across feature jumps.',
        'friction_convention': 'Right endpoint force-gradient dot physical increment, separately before/after final lag update; no lag policy or physical threshold change.',
        'runs': [analyze(run) for run in args.runs]}
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    for r in result['runs']:
        print(r['run'],r['dt'],r['complete'],r['frames'][-1] if r['frames'] else r.get('failure'))


if __name__ == '__main__':
    main()
