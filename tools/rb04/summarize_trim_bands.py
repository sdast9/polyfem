"""Summarize a completed pilot without interpreting partial work as balance."""
import argparse
import json
from pathlib import Path

import numpy as np

from run_trim_bands import measure


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('evidence', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    raw = json.loads((args.evidence/'results.json').read_text())
    summary = {k: raw[k] for k in ('source_commit', 'binary_sha256', 'runner_sha256', 'scope')}
    summary['work_scope'] = ('Trapezoidal prescribed top work from independently reconstructed elastic reaction. '
        'Valid support reaction for these quasistatic fixtures; transient excludes inertial reaction. '
        'Initial barrier energy, coefficient events, friction slip dissipation and integration error unavailable; '
        'work minus elastic energy is NOT a balance defect.')
    summary['runs'] = []
    for row in raw['runs']:
        directory = args.evidence/f"{row['scene']}-{row['band']}"/'output'
        result = {k: row[k] for k in ('scene', 'band', 'lower', 'upper', 'exit', 'timed_out', 'complete',
            'wall_seconds', 'observed_trim_log_events', 'observed_refresh_log_events')}
        result['endpoints'] = []
        initial = measure(directory/'step_0.vtu', 1e7, .45, 0)
        prev = initial
        work = 0.
        for e in row['endpoints']:
            if e['outcome'] != 'accepted':
                result['endpoints'].append(e)
                continue
            ref = e['independent_elastic']
            # Reuse the endpoint stage's declared reconstruction tolerances.
            # These validate the measurements, not physical acceptance.
            assert e['measurement_error'] is None, e['measurement_error']
            assert abs(ref['elastic_energy']-e['elastic_energy']['value'])/(1+abs(ref['elastic_energy'])) < 1e-9
            assert abs(ref['min_det_F']-e['min_det_F']['value']) < 1e-10
            assert abs(ref['bc_error']-e['bc_error_inf']['value']) < 1e-10
            work += .5*(prev['top_reaction'][2]+ref['top_reaction'][2])*(-.25)*(ref['time']-prev['time'])
            prev = ref
            c = e['contact']
            result['endpoints'].append(dict(step=e['step'], reaction_z=ref['top_reaction'][2],
                residual=e['free_residual_norm']['value'], bc_error=e['bc_error_inf']['value'],
                min_det_F=e['min_det_F']['value'], min_gap_over_dhat=c['min_gap']['value']/c['dhat'],
                trim=c['trim_or_global_stiffness'], refresh_id=c['refresh_id'],
                active_count=c['active_count'], zero_coefficients=c['coefficient_zero_count'],
                nonfinite_coefficients=c['coefficient_nonfinite_count'],
                elastic_energy=e['elastic_energy']['value'], barrier_energy=e['barrier_energy']['value'],
                lagging=e['lagging'], top_work_trapezoid=work,
                work_minus_elastic_change=work-ref['elastic_energy']+initial['elastic_energy']))
        if row['complete']:
            base = args.evidence/f"{row['scene']}-default"/'output'
            current = [json.loads(s) for s in (directory/'physical-diagnostics.jsonl').read_text().splitlines()]
            reference = [json.loads(s) for s in (base/'physical-diagnostics.jsonl').read_text().splitlines()]
            result['max_displacement_difference_from_default'] = max(float(np.max(np.abs(
                np.array(a['endpoint']['value'])-b['endpoint']['value']))) for a, b in zip(current, reference))
        summary['runs'].append(result)
    args.output.write_text(json.dumps(summary, indent=2)+'\n')
    for r in summary['runs']:
        e = r['endpoints'][-1]
        print(r['scene'], r['band'], r['complete'], 'reaction', e.get('reaction_z'),
              'residual', e.get('residual'), 'gap/dhat', e.get('min_gap_over_dhat'))


if __name__ == '__main__':
    main()
