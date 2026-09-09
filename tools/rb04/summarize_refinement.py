"""Reconstruct saved refinement runs; never rerun solvers or erase raw failures."""
import argparse
import hashlib
import json
from pathlib import Path

from run_refinement import analyze
from work_reference import verify_transient


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('evidence', nargs='+', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    result = {'postprocessor_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'analysis_source_sha256': hashlib.sha256(Path(__file__).with_name('run_refinement.py').read_bytes()).hexdigest(),
              'studies': []}
    for out in args.evidence:
        raw = json.loads((out/'results.json').read_text())
        study = {k: raw[k] for k in ('source_commit', 'binary_sha256', 'runner_sha256', 'scene', 'protocol', 'work_convention', 'limits')}
        study['evidence_directory'] = out.name
        study['runs'] = []
        for row in raw['runs']:
            r = {k: row[k] for k in ('lower', 'upper', 'dt', 'directory', 'status', 'exit', 'wall_seconds')}
            r['original_analysis_error'] = row.get('analysis_error')
            m = analyze(out/row['directory'], row['dt'], raw['scene'])
            if raw['scene'] == 'transient-semi':
                verify_transient(out/row['directory'], row['dt'], m['frames'])
            r['frames'] = m['frames']
            r['initial_state_verified'] = m['initial_state_verified']
            r['complete'] = row['exit'] == 0 and m.get('last_step') == round(1/row['dt'])
            if 'failure_record' in m:
                f = m['failure_record']
                r['failure'] = {k: f[k] for k in ('step', 'outcome', 'phase', 'termination', 'contact', 'free_residual_norm')}
                path = out/row['directory']/'output/coefficient-events.jsonl'
                events = [json.loads(s) for s in path.read_text().splitlines()]
                r['failure']['maximum_observed_active_contacts'] = max(e['after'].get('evaluated_state',{}).get('active_count', 0) for e in events)
                r['failure']['observed_trim_values'] = sorted(set(e['after']['state']['trim_or_global_stiffness'] for e in events))
            study['runs'].append(r)
        result['studies'].append(study)
        print(raw['scene'], sum(r['complete'] for r in study['runs']), '/', len(study['runs']), 'complete')
    args.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
