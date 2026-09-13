"""RB-10 summary tables from the analyzer's endpoints.json files.

Usage: python3 tools/rb10/summarize.py --evidence /abs/outputs/rb-10/<timestamp> [--out tools/rb10/results-YYYYMMDD.json]

Reads stage2-fixtures, stage3-sensitivity, stage3-classic and
stage4-friction-lag-ab (whichever exist), prints markdown tables and writes a
compact JSON with the per-run summaries and the per-step quantities the record
quotes (nothing is recomputed here).
"""
import argparse
import json
from pathlib import Path


def load(evidence, stage):
    path = evidence / stage / 'endpoints.json'
    if not path.exists():
        return []
    return json.loads(path.read_text())


def fmt(v, digits=3):
    if v is None:
        return 'n/a'
    if isinstance(v, (list, tuple)):
        return '[' + ', '.join(fmt(x, digits) for x in v) + ']'
    if isinstance(v, float):
        return f'{v:.{digits}g}'
    return str(v)


def sliding_stats(run):
    """Solved-lag and updated-lag ratios over the steps whose updated lag is in full sliding (> .98)."""
    frames = [f for f in run['frames'] if isinstance(f.get('friction_updated_lag'), dict) and (f['friction_updated_lag'].get('ratio_to_mu_N_endpoint') or 0) > .98]
    if not frames:
        return None
    pre = [f['friction_solved_lag']['ratio_to_mu_N_endpoint'] for f in frames]
    post = [f['friction_updated_lag']['ratio_to_mu_N_endpoint'] for f in frames]
    lag = [abs(f['lag_error_relative']) for f in frames if f.get('lag_error_relative') is not None]
    return {'steps': [f['step'] for f in frames], 'solved_lag_ratio_min_max': [min(pre), max(pre)], 'updated_lag_ratio_min_max': [min(post), max(post)],
            'lag_error_max': max(lag) if lag else None}


def stick_stats(run):
    """Contact creep per step over the top increment while the updated lag is below sliding."""
    frames = [f for f in run['frames'] if f.get('contact_slip_over_top_increment_x') is not None and isinstance(f.get('friction_updated_lag'), dict) and 0 < (f['friction_updated_lag'].get('ratio_to_mu_N_endpoint') or 0) < .9]
    if not frames:
        return None
    return {'steps': [f['step'] for f in frames], 'slip_over_top_increment': [round(f['contact_slip_over_top_increment_x'], 4) for f in frames],
            'max_abs_contact_slip_x': max(f['contact_slip_max_abs_x'] for f in frames)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--out', type=Path, default=None)
    args = parser.parse_args()
    out = {'evidence': str(args.evidence), 'stages': {}}
    for stage in ['stage2-fixtures', 'stage3-sensitivity', 'stage3-classic', 'stage4-friction-lag-ab']:
        runs = load(args.evidence, stage)
        if not runs:
            continue
        rows = []
        print(f'\n### {stage}\n')
        print('| run | steps | status | sliding ratio solved / updated | lag error (sliding) | D min | D negative steps | D cumulative | support work | free residual solved / updated lag | Newton (all minimizes) | trims |')
        print('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
        for run in runs:
            s = run.get('summary') or {}
            sl = sliding_stats(run)
            row = {'name': run['name'], 'fixture': run['fixture'], 'model': run['model'], 'mu': run['mu'], 'epsv': run['epsv'], 'budget': run['budget'], 'barrier': run['barrier'],
                   'friction_lag': s.get('friction_lag_mode'), 'accepted': run.get('accepted'), 'status': run['status'], 'exit_status': run.get('exit_status'),
                   'failed_attempts': run.get('failed_attempts'), 'sliding': sl, 'stick': stick_stats(run),
                   'dissipation_min': s.get('dissipation_min'), 'dissipation_negative_steps': s.get('dissipation_negative_steps'),
                   'dissipation_cumulative': s.get('dissipation_cumulative'), 'total_support_work': s.get('total_support_work'),
                   'max_free_residual_solved_lag': s.get('max_free_residual_solved_lag'), 'max_free_residual_updated_lag': s.get('max_free_residual_updated_lag'),
                   'max_equilibrium_work_defect': s.get('max_equilibrium_work_defect'), 'accepted_iterations_total': s.get('accepted_iterations_total_all_minimizes'),
                   'minimize_calls_total': s.get('minimize_calls_total'), 'restarts_total': s.get('restarts_total'), 'min_det_F': s.get('min_det_F'),
                   'trim_history': s.get('trim_history'), 'lagging_states': s.get('lagging_states'),
                   'per_step': [{k: (round(f[k], 6) if isinstance(f.get(k), float) else f.get(k)) for k in ('step', 'time', 'normal_force_endpoint', 'trim', 'contacting_nodes', 'frictional_dissipation_increment', 'support_work_increment', 'contact_slip_over_top_increment_x', 'free_residual_norm_solved_lag', 'free_residual_norm_updated_lag', 'lag_error_relative', 'accepted_iterations_all_minimizes')}
                                | {'support_force_x_z': [round(f['support_force_on_body'][0], 3), round(f['support_force_on_body'][2], 3)] if f.get('support_force_on_body') else None,
                                   'solved_lag_ratio': (f.get('friction_solved_lag') or {}).get('ratio_to_mu_N_endpoint'), 'updated_lag_ratio': (f.get('friction_updated_lag') or {}).get('ratio_to_mu_N_endpoint'),
                                   'solved_lag_force_x_y': [round(v, 3) for v in (f.get('friction_solved_lag') or {}).get('force_on_body', [])[:2]] or None,
                                   'interfaces': [{'obstacle': i['obstacle'], 'normal_force': round(i['normal_force'], 3), 'friction_tangential': round(i['friction_tangential'], 3), 'ratio_to_mu_N': i['ratio_to_mu_N']} for i in (f.get('friction_solved_lag') or {}).get('interfaces', [])] if run['fixture'] == 'corner_coupled' else None}
                                for f in run['frames']] if stage in ('stage2-fixtures', 'stage4-friction-lag-ab') else None}
            rows.append(row)
            trims = sorted(set(t for t in (s.get('trim_history') or []) if t is not None))
            print(f"| {run['name']} | {run.get('accepted')} | {run['status']} | {fmt(sl['solved_lag_ratio_min_max'] if sl else None, 5)} / {fmt(sl['updated_lag_ratio_min_max'] if sl else None, 7)} | {fmt(sl['lag_error_max'] if sl else None, 2)} | {fmt(s.get('dissipation_min'), 2)} | {s.get('dissipation_negative_steps')} | {fmt(s.get('dissipation_cumulative'), 6)} | {fmt(s.get('total_support_work'), 6)} | {fmt(s.get('max_free_residual_solved_lag'), 2)} / {fmt(s.get('max_free_residual_updated_lag'), 3)} | {s.get('accepted_iterations_total_all_minimizes')} | {trims} |")
        out['stages'][stage] = rows
    if args.out:
        # per-run records on their own lines: readable diffs without the size of full indentation
        lines = ['{', f' "evidence": {json.dumps(out["evidence"])},', ' "stages": {']
        for si, (stage, rows) in enumerate(out['stages'].items()):
            lines.append(f'  {json.dumps(stage)}: [')
            for ri, row in enumerate(rows):
                lines.append('   ' + json.dumps(row, separators=(',', ':')) + (',' if ri + 1 < len(rows) else ''))
            lines.append('  ]' + (',' if si + 1 < len(out['stages']) else ''))
        lines += [' }', '}']
        args.out.write_text('\n'.join(lines) + '\n')
        print(f'\nwrote {args.out}')


if __name__ == '__main__':
    main()
