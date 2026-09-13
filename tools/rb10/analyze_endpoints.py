"""RB-10 endpoint analysis of the physical-diagnostics records written by run_scenes.py.

Usage: python3 tools/rb10/analyze_endpoints.py --runs /abs/dir [--out results.json]

Per accepted endpoint (all in internal force/length/energy units, RB-04 record
version 2): total support force on the body's prescribed DOFs, total barrier
and friction forces on the body (and per obstacle interface: the obstacle DOF
sums, i.e. the force the body receives through that interface), the sliding
ratio |F_t| / (mu N) with the solved-lag (pre-update) and the endpoint-rebuilt
(post-update) friction, the lag error between them, whether the solved-lag
friction opposes the top's increment (and, for quasistatic runs, the total
displacement the potential actually uses), the mean slip of the contacting
nodes against the prescribed increment (stick/slip), the RB-04 right-endpoint
work increments and the numerical residuals on both sides of the final lag
update. Nothing is inferred: a missing quantity is reported as unavailable.
"""
import argparse
import json
from pathlib import Path
import numpy as np

OBSTACLE_DOFS = 12  # 4-vertex quad obstacle x 3


def vec(entry):
    if entry is None:
        return None
    if isinstance(entry, dict):
        entry = entry.get('value')
    return None if entry is None else np.asarray(entry, dtype=float)


def val(entry):
    if isinstance(entry, dict):
        return entry.get('value')
    return entry


def totals(v, n_body, obstacles):
    """Sum a node-major DOF vector over the body and over each obstacle."""
    body = v[:n_body].reshape(-1, 3).sum(axis=0)
    per = []
    for i in range(obstacles):
        seg = v[n_body + i * OBSTACLE_DOFS:n_body + (i + 1) * OBSTACLE_DOFS]
        per.append(seg.reshape(-1, 3).sum(axis=0))
    return body, per


def analyze_run(run_dir):
    meta = json.loads((run_dir / 'run.json').read_text())
    scene = json.loads((run_dir / 'scene.json').read_text())
    mu = scene['contact']['friction_coefficient']
    epsv = scene['contact'].get('epsv', 1e-3)
    obstacles = sum(1 for g in scene['geometry'] if g.get('is_obstacle'))
    quasistatic = scene['time']['quasistatic']
    dt = scene['time']['dt']
    path = run_dir / 'output' / 'physical-diagnostics.jsonl'
    result = {'name': run_dir.name, 'configured_name': meta['name'], 'fixture': meta['fixture'], 'model': meta['model'], 'mu': mu, 'epsv': epsv,
              'budget': scene['solver']['contact'].get('friction_iterations', 'solver default'), 'barrier': meta['barrier'],
              'friction_lag': scene['solver']['contact'].get('semi_implicit', {}).get('friction_lag', 'solver default'),
              'status': meta.get('status'), 'exit_status': meta.get('exit_status'), 'frames': [], 'failed_attempts': []}
    if not path.exists():
        result['unavailable_reason'] = 'no physical-diagnostics.jsonl'
        return result
    records = [json.loads(line) for line in path.read_text().splitlines()]
    prev_E = prev_B = prev_K = 0.0
    prev_x = None
    cumulative_dissipation = 0.0
    for r in records:
        if r['outcome'] != 'accepted':
            result['failed_attempts'].append({'step': r['step'], 'outcome': r['outcome'], 'error': r.get('error'), 'phase': r.get('phase')})
            continue
        x = vec(r['endpoint'])
        n_body = x.size - obstacles * OBSTACLE_DOFS
        dx = vec(r['accepted_displacement'])
        if prev_x is not None and not np.allclose(prev_x + dx, x, atol=1e-12, rtol=0):
            raise AssertionError(f"{meta['name']} step {r['step']}: accepted_displacement is not endpoint - previous endpoint")
        grads = {f['name']: vec(f.get('gradient_force_units')) for f in r['forms']}
        g_c = grads.get('barrier-contact')
        g_e = grads.get('elastic')
        g_i = grads.get('inertia')
        lag = r.get('lagging', {})
        g_f_pre = vec(lag.get('friction_before_update', {}).get('gradient_force_units')) if isinstance(lag.get('friction_before_update'), dict) else None
        g_f_post = vec(lag.get('friction_after_update', {}).get('gradient_force_units')) if isinstance(lag.get('friction_after_update'), dict) else None
        reaction = vec(r['reactions'][0]['full_dof_vector']) if r.get('reactions') else None
        frame = {'step': r['step'], 'time': val(r['time']),
                 'termination': r['termination'].get('termination_reason'), 'newton_iterations': r['termination'].get('iterations'),
                 'accepted_iterations_all_minimizes': (r.get('attempt_summary') or {}).get('accepted_iterations'),
                 'minimize_calls': (r.get('attempt_summary') or {}).get('minimize_calls'),
                 'restarts': r['termination'].get('restarts'), 'lagging_state': lag.get('state'), 'lagging_iterations': lag.get('iteration'),
                 'updated_lag_residual_objective': lag.get('updated_lag_residual_norm_objective'), 'lag_tolerance': lag.get('tolerance'),
                 'free_residual_norm_updated_lag': val(r.get('free_residual_norm')),
                 'free_residual_norm_solved_lag': val(r.get('free_residual_norm_with_pre_update_friction')),
                 'active_contacts': r.get('contact', {}).get('active_count'), 'min_gap': val(r.get('contact', {}).get('min_gap')),
                 'trim': r.get('contact', {}).get('trim_or_global_stiffness'),
                 'friction_lag_recorded': r.get('contact', {}).get('friction_lag') if isinstance(r.get('contact'), dict) else None,
                 'elastic_energy': val(r.get('elastic_energy')), 'barrier_energy': val(r.get('barrier_energy')), 'kinetic_energy': val(r.get('kinetic_energy')),
                 'support_work_increment': val(r.get('support_work_increment')),
                 'frictional_dissipation_increment': val(r.get('frictional_dissipation_increment')),
                 'retuning_energy_change': val(r.get('retuning_energy_change')), 'min_det_F': val(r.get('min_det_F'))}
        # Prescribed top increment: mean accepted displacement over DOFs carrying a reaction.
        if reaction is not None:
            top = np.abs(reaction[:n_body]).reshape(-1, 3).sum(axis=1) > 0
            top_dx = dx[:n_body].reshape(-1, 3)[top].mean(axis=0) if top.any() else np.full(3, np.nan)
            top_u = x[:n_body].reshape(-1, 3)[top].mean(axis=0) if top.any() else np.full(3, np.nan)
            S_body, _ = totals(reaction, n_body, obstacles)
            frame['support_force_on_body'] = S_body.tolist()
            frame['top_increment'] = top_dx.tolist()
            frame['top_displacement'] = top_u.tolist()
        else:
            top_dx = top_u = np.full(3, np.nan)
        if g_c is not None:
            Fc_body, Fc_obs = totals(g_c, n_body, obstacles)
            frame['barrier_force_on_body'] = (-Fc_body).tolist()
            frame['barrier_force_from_obstacle'] = [f.tolist() for f in Fc_obs]
            N_end = -Fc_body[2]
            contacting = np.abs(g_c[:n_body]).reshape(-1, 3).sum(axis=1) > 0
            frame['contacting_nodes'] = int(contacting.sum())
            if contacting.any():
                slip = dx[:n_body].reshape(-1, 3)[contacting]
                frame['contact_slip_mean'] = slip.mean(axis=0).tolist()
                frame['contact_slip_max_abs_x'] = float(np.abs(slip[:, 0]).max())
                frame['contact_total_displacement_mean'] = x[:n_body].reshape(-1, 3)[contacting].mean(axis=0).tolist()
                frame['contact_slip_over_top_increment_x'] = float(slip[:, 0].mean() / top_dx[0]) if abs(top_dx[0]) > 1e-14 else None
        else:
            N_end = np.nan
        frame['normal_force_endpoint'] = N_end
        for label, g in (('solved_lag', g_f_pre), ('updated_lag', g_f_post)):
            if g is None:
                frame[f'friction_{label}'] = {'unavailable_reason': 'no friction observation'}
                continue
            Ff_body, Ff_obs = totals(g, n_body, obstacles)
            F = -Ff_body
            entry = {'force_on_body': F.tolist(), 'force_from_obstacle': [f.tolist() for f in Ff_obs],
                     'tangential_magnitude': float(np.hypot(F[0], F[1])),
                     'ratio_to_mu_N_endpoint': float(np.hypot(F[0], F[1]) / (mu * N_end)) if mu > 0 and N_end > 0 else None,
                     'opposes_top_increment_x': bool(F[0] * top_dx[0] < 0) if abs(top_dx[0]) > 1e-14 else None,
                     'opposes_top_total_x': bool(F[0] * top_u[0] < 0) if abs(top_u[0]) > 1e-14 else None,
                     'gradient_dot_increment': float(g @ dx),
                     'balance_all_dofs': float(np.abs(g.reshape(-1, 3).sum(axis=0)).max())}
            # per-obstacle interface: normal from the barrier, tangential from the friction
            if g_c is not None:
                interfaces = []
                for i, (fc, ff) in enumerate(zip(Fc_obs, Ff_obs)):
                    N_i = float(np.linalg.norm(fc))
                    n_hat = fc / N_i if N_i > 0 else np.zeros(3)
                    t_vec = ff - (ff @ n_hat) * n_hat
                    interfaces.append({'obstacle': i, 'normal_force': N_i, 'friction_tangential': float(np.linalg.norm(t_vec)),
                                       'friction_normal_component': float(ff @ n_hat),
                                       'ratio_to_mu_N': float(np.linalg.norm(t_vec) / (mu * N_i)) if mu > 0 and N_i > 0 else None})
                entry['interfaces'] = interfaces
            frame[f'friction_{label}'] = entry
        if g_f_pre is not None and g_f_post is not None:
            a, b = -totals(g_f_pre, n_body, obstacles)[0], -totals(g_f_post, n_body, obstacles)[0]
            frame['lag_error_relative'] = float((np.hypot(a[0], a[1]) - np.hypot(b[0], b[1])) / np.hypot(b[0], b[1])) if np.hypot(b[0], b[1]) > 0 else None
        # RB-04 right-endpoint budget pieces (identity terms, not path integrals)
        W = frame['support_work_increment']
        E, B = frame['elastic_energy'], frame['barrier_energy']
        K = frame['kinetic_energy'] if frame['kinetic_energy'] is not None else 0.0
        Df = frame['frictional_dissipation_increment']
        P = frame['retuning_energy_change']
        if None not in (W, E, B, Df):
            C = {name: float(g @ dx) for name, g in grads.items() if g is not None}
            if g_f_pre is not None:
                C['friction'] = float(g_f_pre @ dx)
            frame['right_endpoint_work'] = C
            frame['equilibrium_work_defect'] = float(W - sum(C.values()))
            frame['budget_unexplained'] = float(W - (E - prev_E) - (B - prev_B) - (K - prev_K) - Df + (P or 0.0))
            cumulative_dissipation += Df
        frame['frictional_dissipation_cumulative_recomputed'] = cumulative_dissipation
        result['frames'].append(frame)
        prev_x, prev_E, prev_B, prev_K = x, E, B, K
    # Summaries over the sliding program (after the press phase)
    frames = result['frames']
    result['accepted'] = len(frames)
    if frames:
        def sel(f, key, sub=None):
            v = f.get(key)
            if sub is not None:
                v = v.get(sub) if isinstance(v, dict) else None
            return v
        press_end = 0.2 if meta.get('fixture') != 'smoke' else 0.0
        slide = [f for f in frames if f['time'] is not None and f['time'] > press_end + 1e-9]
        ratios_pre = [sel(f, 'friction_solved_lag', 'ratio_to_mu_N_endpoint') for f in slide]
        ratios_post = [sel(f, 'friction_updated_lag', 'ratio_to_mu_N_endpoint') for f in slide]
        ratios_pre = [v for v in ratios_pre if v is not None]
        ratios_post = [v for v in ratios_post if v is not None]
        diss = [f['frictional_dissipation_increment'] for f in frames if f['frictional_dissipation_increment'] is not None]
        result['summary'] = {
            'sliding_steps': len(slide),
            'ratio_solved_lag_min_max': [min(ratios_pre), max(ratios_pre)] if ratios_pre else None,
            'ratio_updated_lag_min_max': [min(ratios_post), max(ratios_post)] if ratios_post else None,
            'lag_error_relative_max_abs': max(abs(f['lag_error_relative']) for f in slide if f.get('lag_error_relative') is not None) if any(f.get('lag_error_relative') is not None for f in slide) else None,
            'dissipation_min': min(diss) if diss else None,
            'dissipation_negative_steps': [f['step'] for f in frames if f['frictional_dissipation_increment'] is not None and f['frictional_dissipation_increment'] < 0],
            'dissipation_cumulative': cumulative_dissipation,
            'opposes_increment_all_sliding_steps': all(sel(f, 'friction_solved_lag', 'opposes_top_increment_x') in (True, None) for f in slide),
            'steps_not_opposing_increment': [f['step'] for f in slide if sel(f, 'friction_solved_lag', 'opposes_top_increment_x') is False],
            'max_free_residual_solved_lag': max([f['free_residual_norm_solved_lag'] for f in frames if f['free_residual_norm_solved_lag'] is not None], default=None),
            'max_free_residual_updated_lag': max([f['free_residual_norm_updated_lag'] for f in frames if f['free_residual_norm_updated_lag'] is not None], default=None),
            'lagging_states': sorted(set(str(f['lagging_state']) for f in frames)),
            'max_equilibrium_work_defect': max(abs(f['equilibrium_work_defect']) for f in frames if 'equilibrium_work_defect' in f) if any('equilibrium_work_defect' in f for f in frames) else None,
            'max_budget_unexplained': max(abs(f['budget_unexplained']) for f in frames if 'budget_unexplained' in f) if any('budget_unexplained' in f for f in frames) else None,
            'total_support_work': sum(f['support_work_increment'] for f in frames if f['support_work_increment'] is not None),
            'newton_iterations_total': sum(f['newton_iterations'] or 0 for f in frames),
            'accepted_iterations_total_all_minimizes': sum(f['accepted_iterations_all_minimizes'] or 0 for f in frames),
            'minimize_calls_total': sum(f['minimize_calls'] or 0 for f in frames),
            'restarts_total': sum(f['restarts'] or 0 for f in frames),
            'min_det_F': min(f['min_det_F'] for f in frames if f['min_det_F'] is not None) if any(f['min_det_F'] is not None for f in frames) else None,
            'trim_history': [f['trim'] for f in frames],
            # the mode the solver actually used (diagnostic_state), and the budget it reached
            'friction_lag_mode': next((f['friction_lag_recorded'] for f in frames if f.get('friction_lag_recorded')), result['friction_lag']),
            'max_lagging_iterations_reached': max((f['lagging_iterations'] or 0) for f in frames),
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--runs', type=Path, required=True)
    parser.add_argument('--out', type=Path, default=None)
    args = parser.parse_args()
    runs = sorted(p.parent for p in args.runs.glob('*/run.json'))
    results = [analyze_run(run) for run in runs]
    out = args.out or (args.runs / 'endpoints.json')
    out.write_text(json.dumps(results, indent=1) + '\n')
    for res in results:
        s = res.get('summary') or {}
        print(f"{res['name']:<48} accepted={res.get('accepted')} status={res['status']} "
              f"ratio_pre={s.get('ratio_solved_lag_min_max')} ratio_post={s.get('ratio_updated_lag_min_max')} "
              f"diss_min={s.get('dissipation_min')} neg_steps={s.get('dissipation_negative_steps')} "
              f"not_opposing={s.get('steps_not_opposing_increment')} lag_err={s.get('lag_error_relative_max_abs')}")


if __name__ == '__main__':
    main()
