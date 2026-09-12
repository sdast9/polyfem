"""Check solver-attempts.jsonl streams against their endpoint records.

python3 tools/rb04/check_solver_attempts.py /absolute/evidence --output /absolute/summary.json

For every diagnostic-on run of run_endpoints.py: each step's stream starts
every PolySolve minimize with a `start` row (iteration 0), accepted rows are
numbered consecutively, every accepted fraction lies in (0, step_bound],
rejected rows carry a finite step bound, and the endpoint record's attempt
summary, last proposal and termination iteration count agree with the rows.
Failed attempts must carry the last internal iterate. Norms are objective-side
bookkeeping; they certify nothing physical.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path


def check_run(directory):
    output = Path(directory) / 'output'
    rows = [json.loads(line) for line in (output / 'solver-attempts.jsonl').read_text().splitlines()]
    endpoints = [json.loads(line) for line in (output / 'physical-diagnostics.jsonl').read_text().splitlines()]
    assert rows and endpoints
    assert len({r['run_id'] for r in rows}) == 1 and rows[0]['run_id'] == endpoints[0]['run_id']
    by_step = defaultdict(list)
    for row in rows:
        assert row['schema'] == 'polyfem.solver-attempt' and row['version'] == 1
        assert row['kind'] in ('start', 'accepted', 'rejected')
        by_step[row['step']].append(row)
    summary = {'name': Path(directory).name, 'steps': []}
    for endpoint in endpoints:
        step_rows = by_step[endpoint['step']]
        assert step_rows, endpoint['step']
        minimize_ids = sorted({r['minimize_index'] for r in step_rows})
        assert minimize_ids == list(range(1, len(minimize_ids) + 1))
        accepted_rows = [r for r in step_rows if r['kind'] == 'accepted']
        rejected_rows = [r for r in step_rows if r['kind'] == 'rejected']
        starts = [r for r in step_rows if r['kind'] == 'start']
        assert len(starts) == len(minimize_ids)
        for m in minimize_ids:
            group = [r for r in step_rows if r['minimize_index'] == m]
            assert group[0]['kind'] == 'start' and group[0]['iteration'] == 0
            assert group[0]['gradient_norm_objective_at_x0'] is None
            accepted = [r['iteration'] for r in group if r['kind'] == 'accepted']
            assert accepted == list(range(1, len(accepted) + 1)), (m, accepted)
        max_fraction_excess = 0.
        for row in accepted_rows:
            trial, accepted = row['trial'], row['accepted']
            assert trial['norm'] >= 0 and trial['step_bound'] is not None and 0 < trial['step_bound'] <= 1
            assert accepted['fraction_of_trial'] > 0
            max_fraction_excess = max(max_fraction_excess, accepted['fraction_of_trial'] - trial['step_bound'])
            assert accepted['fraction_of_trial'] <= trial['step_bound'] + 1e-9
            assert abs(accepted['norm'] - accepted['fraction_of_trial'] * trial['norm']) <= 1e-9 * (1 + trial['norm'])
            assert trial['validity_checks'] >= trial['validity_rejections'] >= 0
        for row in rejected_rows:
            assert row['trial']['step_bound'] is not None and row['accepted'] is None
        attempt = endpoint['attempt_summary']
        assert attempt['minimize_calls'] == len(minimize_ids)
        assert attempt['accepted_iterations'] == len(accepted_rows)
        assert attempt['rejected_proposals'] == len(rejected_rows)
        assert attempt['validity_checks'] == sum(r['trial']['validity_checks'] for r in accepted_rows + rejected_rows)
        assert attempt['step_bound_limited'] == sum(r['trial']['step_bound'] < 1 for r in accepted_rows)
        assert attempt['line_search_truncated'] == sum(
            r['accepted']['fraction_of_trial'] < r['trial']['step_bound'] * (1 - 1e-12) for r in accepted_rows)
        candidates = attempt['broad_phase_candidates']
        # ALSolver's feasibility checks also build a swept candidate set.
        assert candidates['builds'] == len(accepted_rows) + len(rejected_rows) + attempt['feasibility_checks'], (candidates, len(accepted_rows), len(rejected_rows), attempt['feasibility_checks'])
        assert candidates['max'] >= candidates['last'] > 0
        assert endpoint['contact']['candidate_count']['value'] == candidates['last']
        proposal = endpoint['proposed_displacement']
        if accepted_rows:
            last = accepted_rows[-1]
            assert proposal['iteration'] == last['iteration'] and proposal['minimize_index'] == last['minimize_index']
            assert proposal['trial_norm'] == last['trial']['norm']
            assert proposal['accepted_fraction_of_trial'] == last['accepted']['fraction_of_trial']
        else:
            assert proposal.get('value') is None and proposal.get('unavailable_reason')
        if endpoint['outcome'] == 'accepted':
            # The last subsolve's PolySolve iteration count is the last minimize's accepted rows.
            last_minimize = [r for r in accepted_rows if r['minimize_index'] == minimize_ids[-1]]
            assert endpoint['termination']['iterations'] == len(last_minimize), (endpoint['termination']['iterations'], len(last_minimize))
            assert 'last_internal_iterate' not in endpoint
        else:
            iterate = endpoint['last_internal_iterate']
            assert iterate['value'] is not None and len(iterate['value']) == len(endpoint['endpoint']['value'])
            assert iterate['iteration'] >= 1 and iterate['value'] != endpoint['endpoint']['value']
        summary['steps'].append(dict(
            step=endpoint['step'], outcome=endpoint['outcome'], minimize_calls=len(minimize_ids),
            accepted=len(accepted_rows), rejected=len(rejected_rows), feasibility_checks=attempt['feasibility_checks'],
            step_bound_limited=attempt['step_bound_limited'], line_search_truncated=attempt['line_search_truncated'],
            stall_retunes=attempt['stall_retunes'], candidates=candidates, max_fraction_minus_bound=max_fraction_excess,
            last_trial_norm=accepted_rows[-1]['trial']['norm'] if accepted_rows else None))
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('evidence', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    runs = json.loads((args.evidence / 'results.json').read_text())
    result = {'binary_sha256': runs['binary_sha256'], 'runs': [],
              'interpretation': 'Per-iteration trial sweeps, step bounds and accepted fractions of the nonlinear solve; solver bookkeeping, not physical accuracy'}
    for run in runs['runs']:
        directory = args.evidence / run['name']
        if run['name'].endswith('-off'):
            assert not (directory / 'output/solver-attempts.jsonl').exists()
            continue
        result['runs'].append(check_run(directory))
    result['passed'] = True
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({r['name']: [(s['step'], s['outcome'], s['accepted'], s['rejected']) for s in r['steps']] for r in result['runs']}))


if __name__ == '__main__':
    main()
