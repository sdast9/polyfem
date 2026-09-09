"""Check event streams from run_endpoints.py and retain per-phase totals.

Totals describe algorithmic changes along solver iterates, not physical work.
They cannot be subtracted blindly from an accepted-endpoint energy budget.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('evidence', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    runs = json.loads((args.evidence/'results.json').read_text())
    result = {'binary_sha256': runs['binary_sha256'], 'runs': [],
              'interpretation': 'Coefficient objective changes along optimization iterates; not mechanical trajectory work',
              'roundoff_screen': '1e-10*(1+abs(before objective)+abs(after objective))/acceleration_scaling; reporting only'}
    for run in runs['runs']:
        path = args.evidence/run['name']/'output/coefficient-events.jsonl'
        if run['name'].endswith('-off'):
            assert not path.exists()
            continue
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        endpoint_rows = {r['step']: r for r in (json.loads(line) for line in
            path.with_name('physical-diagnostics.jsonl').read_text().splitlines())}
        assert rows
        assert len({r['run_id'] for r in rows}) == 1
        assert [r['event_id'] for r in rows] == list(range(rows[0]['event_id'], rows[0]['event_id']+len(rows)))
        groups = defaultdict(list)
        unavailable = 0
        for row in rows:
            assert row['schema'] == 'polyfem.coefficient-event' and row['version'] == 1
            assert row['operation'] in ('refresh', 'post_step', 'stall_retune', 'calibration')
            delta = row['objective_change_at_fixed_coordinates']
            if delta is None:
                assert row.get('unavailable_reason')
                unavailable += 1
                continue
            before, after = row['before']['objective'], row['after']['objective']
            assert abs(delta-(after-before)) <= 1e-10*(1+abs(before)+abs(after))
            physical = delta/row['acceleration_scaling']
            assert abs(row['energy_change_at_fixed_coordinates']-physical) <= 1e-10*(1+abs(physical))
            screen = 1e-10*(1+abs(before)+abs(after))/row['acceleration_scaling']
            groups[(row['step'], row['phase'])].append((physical, screen, row['free_contact_force_change_norm']))
            if row['phase'] == 'between_steps_after_endpoint':
                endpoint = endpoint_rows[row['step']]
                assert np.array_equal(row['coordinates'], endpoint['endpoint']['value'])
                contact = next(f for f in endpoint['forms'] if f['name'] == 'barrier-contact')
                a = np.asarray(row['before']['gradient_objective'])/row['acceleration_scaling']
                b = np.asarray(contact['gradient_force_units']['value'])
                assert np.linalg.norm(a-b) < 1e-9*(1+np.linalg.norm(b))
                assert abs(before/row['acceleration_scaling']-endpoint['barrier_energy']['value']) < 1e-9*(1+abs(endpoint['barrier_energy']['value']))
        summary = dict(name=run['name'], event_count=len(rows), unavailable_count=unavailable,
                       operations=dict(Counter(r['operation'] for r in rows)), phases=[])
        for (step, phase), values in sorted(groups.items()):
            summary['phases'].append(dict(step=step, phase=phase, count=len(values),
                above_roundoff_screen_count=sum(abs(v) > screen for v, screen, _ in values),
                signed_energy_change=sum(v for v, _, _ in values),
                absolute_energy_change=sum(abs(v) for v, _, _ in values),
                max_free_contact_force_change_norm=max(f for _, _, f in values)))
        if 'failure' in run['name']:
            assert any(r['operation'] == 'stall_retune' for r in rows)
        else:
            assert {r['step'] for r in rows if r['phase'] == 'between_steps_after_endpoint'} == {1, 2, 3, 4}
        result['runs'].append(summary)
        print(summary['name'], 'events', len(rows), 'unavailable', unavailable, flush=True)
    result['passed'] = True
    args.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
