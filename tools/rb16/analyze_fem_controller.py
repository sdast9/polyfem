#!/usr/bin/env python3
"""Summarize retained RB-16 stage-2 outputs without changing their disposition."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def summarize(record):
    endpoints = record.get('endpoints', [])
    solves = record.get('solves', [])
    events = record.get('events', [])
    active = [e for e in endpoints if e['active']]
    eligible = [e for e in endpoints if e['prediction'].get('p', math.inf) <= .1*math.sqrt(.9)]
    retunes = [e for e in events if e['before']['trim_or_k'] != e['after']['trim_or_k'] or
               e['before']['effective_k'] != e['after']['effective_k']]
    signs = []
    for event in retunes:
        before, after = event['before']['effective_k'], event['after']['effective_k']
        if not before or not after:
            before, after = event['before']['trim_or_k'], event['after']['trim_or_k']
        if before != after:
            signs.append(1 if after > before else -1)
    refreshes = [e for e in events if e['reason'] == 'post_publication_refresh']
    errors = [abs(s['prediction_error_over_dhat']) for s in solves if 'prediction_error_over_dhat' in s]
    quadrature = [h['quadrature_disagreement'] for s in solves for h in s['history'] if 'quadrature_disagreement' in h]
    recontact, previous = 0, False
    for e in endpoints:
        if e['active'] and not previous:
            recontact += 1
        previous = bool(e['active'])
    converged = [e for e in endpoints if e['numerical_status'] == 'converged']
    return {'config': record['config'], 'checks': record.get('checks', 0),
            'probe_checks_passed': record.get('passed', False), 'completed': record.get('completed', False),
            'endpoints': len(endpoints), 'expected_endpoints': 1+5*record['config'].get('subdivisions', 4),
            'last_numerical_status': endpoints[-1]['numerical_status'] if endpoints else None,
            'last_residual': endpoints[-1]['residual'] if endpoints else None,
            'prescribed_step_failure': record.get('prescribed_step_failure'),
            'active_endpoints': len(active), 'active_in_band': sum(e['in_band'] for e in active),
            'active_occupancy': sum(e['in_band'] for e in active)/len(active) if active else None,
            'tangent_applicable_endpoints': len(eligible),
            'tangent_applicable_in_band': sum(e['in_band'] for e in eligible),
            'tangent_applicable_occupancy': sum(e['in_band'] for e in eligible)/len(eligible) if eligible else None,
            'prediction_unavailable_endpoints': sum('p' not in e['prediction'] for e in endpoints),
            'controller_dispositions': dict(Counter(e['controller_disposition'] for e in endpoints)),
            'interval_states': dict(Counter(e['prediction']['status'] for e in endpoints)),
            'minimum_gap': min((e['gap'] for e in endpoints), default=None),
            'minimum_det': min((e['min_det'] for e in endpoints), default=None),
            'maximum_bc_error': max((e['bc_error'] for e in endpoints), default=None),
            'maximum_action_reaction_error': max((e['contact_balance_error'] for e in endpoints), default=None),
            'maximum_converged_residual': max((e['residual'] for e in converged), default=None),
            'contact_activation_episodes': recontact, 'separated_endpoints': len(endpoints)-len(active),
            'newton_steps': sum('backtracks' in h for s in solves for h in s['history']),
            'solves': len(solves), 'solver_evaluations': sum(s['evaluations'] for s in solves),
            'quadrature_gradient_evaluations': sum(s.get('quadrature_gradient_evaluations', 0) for s in solves),
            'retunes': len(retunes), 'coefficient_callbacks': len(events),
            'retune_reversals': sum(a != b for a, b in zip(signs, signs[1:])),
            'maximum_prediction_error_over_dhat': max(errors, default=None),
            'maximum_quadrature_disagreement': max(quadrature, default=None),
            'maximum_post_publication_residual': max((e['after']['residual'] for e in refreshes), default=None),
            'maximum_post_publication_force_change': max((abs(e['after']['normal_force']-e['before']['normal_force']) for e in refreshes), default=None),
            'coefficient_energy_change': record.get('coefficient_energy_change'),
            'contact_displacement_energy_change': record.get('contact_displacement_energy_change'),
            'energy_identity_error': record.get('energy_identity_error'), 'cost_us': record.get('cost_us')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    groups, hashes = {}, {}
    for group in ('probe-01', 'integral-01', 'top-01'):
        rows = []
        for path in sorted((args.evidence/group).glob('case-*/result.json')):
            record = json.loads(path.read_text())
            rows.append(summarize(record))
            hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        groups[group] = {'cases': len(rows), 'completed': sum(r['completed'] for r in rows),
                         'checks': sum(r['checks'] for r in rows), 'rows': rows}
    controls = []
    for path in sorted((args.evidence/'derivative-controls').glob('case-*/result.json')):
        record = json.loads(path.read_text())
        controls.append({k: record[k] for k in ('config', 'checks', 'passed', 'derivative_control')})
    diagnostic = json.loads((args.evidence/'arithmetic-diagnostic/case-00/result.json').read_text())
    result = {'scope': 'real contact forms and assembled FEM in an experimental Newton driver',
              'groups': groups, 'derivative_controls': controls,
              'arithmetic_diagnostic': diagnostic['solves'][-1]['terminal_arithmetic_diagnostic'],
              'input_hashes': hashes}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: {v: r[v] for v in ('cases', 'completed', 'checks')} for k, r in groups.items()}))


if __name__ == '__main__':
    main()
