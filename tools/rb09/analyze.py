"""RB-09 endpoint analysis: quantities of interest, references and the declared checks.

Usage: python3 tools/rb09/analyze.py --runs /abs/matrix-dir [--out results.json]

Reads every run directory written by run_matrix.py (run.json, scene.json,
output/physical-diagnostics.jsonl, output/nodes.txt). For each accepted
endpoint it measures, from the record alone (nothing recomputed from solver
internals): the support force on the prescribed top DOFs, the barrier force
on the body and per obstacle, the inertia force (transient), the gap of every
bottom-face node above the floor plane, the band coverage, the coefficient
range against the batch floor/cap, the lateral displacement of the free face
x = 1 (block benchmark), the energies, work increments and the solver cost.
The block benchmark endpoints are compared with the exact Neo-Hookean
uniaxial state (tools/rb09/reference.py); the clamped benchmark with its
same-mesh hard-contact reference runs. A missing quantity is reported as
unavailable, never as zero.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import reference as ref  # noqa: E402

BAND = (math.sqrt(0.5), math.sqrt(0.9))   # gap/dhat band of the squared-gap [.5, .9] dhat^2 controller
DIM = 3


def val(entry):
    if isinstance(entry, dict):
        return entry.get('value')
    return entry


def vec(entry):
    v = val(entry)
    return None if v is None else np.asarray(v, dtype=float)


def load_records(run_dir):
    path = run_dir / 'output' / 'physical-diagnostics.jsonl'
    if not path.exists():
        return None
    return [json.loads(line) for line in path.read_text().splitlines()]


def obstacle_vertex_count(run_dir, scene):
    n = 0
    for g in scene['geometry']:
        if g.get('is_obstacle'):
            n += sum(1 for line in (run_dir / g['mesh']).read_text().splitlines() if line.startswith('v '))
    return n


def body_ranges(nodes, scene, unit_scale):
    """Node index sets: bottom face of the lowest block, top face of the top
    block, the x = 1 face of the lowest block, and the per-block node masks."""
    s = unit_scale
    z = nodes[:, 2]
    blocks = []
    if any(g.get('transformation', {}).get('translation') for g in scene['geometry']):
        # stacked: lower block z in [0, 1], upper in [1+g0, 2+g0] (scaled)
        blocks.append(z <= 1.0 * s + 1e-9)
        blocks.append(z > 1.0 * s + 1e-9)
    else:
        blocks.append(np.ones(len(nodes), bool))
    lower, upper = blocks[0], blocks[-1]
    sets = {
        'bottom': np.where(lower & (np.abs(z - z[lower].min()) < 1e-9 * max(1, s)))[0],
        'top': np.where(upper & (np.abs(z - z[upper].max()) < 1e-9 * max(1, s)))[0],
        'side_x1': np.where(lower & (np.abs(nodes[:, 0] - nodes[lower][:, 0].max()) < 1e-9 * max(1, s)))[0],
        'blocks': blocks,
    }
    if len(blocks) == 2:
        sets['interface_lower_top'] = np.where(lower & (np.abs(z - z[lower].max()) < 1e-9))[0]
        sets['interface_upper_bottom'] = np.where(upper & (np.abs(z - z[upper].min()) < 1e-9))[0]
    return sets


def analyze_run(run_dir):
    meta = json.loads((run_dir / 'run.json').read_text())
    scene = json.loads((run_dir / 'scene.json').read_text())
    s = meta.get('unit_scale', 1.0)
    out = {'name': meta['name'], 'group': meta.get('group'), 'benchmark': meta.get('benchmark'), 'status': meta.get('status'),
           'exit_status': meta.get('exit_status'), 'wall_seconds': meta.get('wall_seconds'), 'config': {k: meta.get(k) for k in
           ('dhat', 'dt', 'tend', 'rate', 'E', 'nu', 'rho', 'n_refs', 'quasistatic', 'barrier', 'unit_scale', 'unload_at', 'stacked', 'hard_reference', 'plane_lift')},
           'frames': [], 'failed_attempts': []}
    records = load_records(run_dir)
    if records is None:
        out['unavailable_reason'] = 'no physical-diagnostics.jsonl'
        return out
    nodes_path = run_dir / 'output' / 'nodes.txt'
    if not nodes_path.exists():
        out['unavailable_reason'] = 'no nodes.txt'
        return out
    nodes_all = np.loadtxt(nodes_path)
    n_obs = obstacle_vertex_count(run_dir, scene)
    n_body_nodes = nodes_all.shape[0] - n_obs
    nodes = nodes_all[:n_body_nodes]
    n_body = n_body_nodes * DIM
    sets = body_ranges(nodes, scene, s)
    g0 = 0.02 * s
    floor_z = -g0
    dhat = scene['contact']['dhat']
    quasistatic = scene['time']['quasistatic']
    E, nu = meta['E'], meta['nu']
    H = 1.0 * s
    is_block = meta.get('benchmark') == 'block'
    hard = meta.get('hard_reference', False)
    max_abs_R = 0.0
    for r in records:
        if r['outcome'] != 'accepted':
            out['failed_attempts'].append({'step': r['step'], 'outcome': r['outcome'], 'error': r.get('error'), 'phase': r.get('phase')})
            continue
        x = vec(r['endpoint'])
        if x.size != nodes_all.shape[0] * DIM:
            raise AssertionError(f"{meta['name']}: endpoint size {x.size} != {nodes_all.shape[0] * DIM}")
        u = x[:n_body].reshape(-1, DIM)
        t = val(r['time'])
        grads = {f['name']: vec(f.get('gradient_force_units')) for f in r['forms']}
        reaction = vec(r['reactions'][0]['full_dof_vector']) if r.get('reactions') else None
        frame = {'step': r['step'], 'time': t, 'termination': r['termination'].get('termination_reason'),
                 'newton_iterations': r['termination'].get('iterations'),
                 'accepted_iterations_all_minimizes': (r.get('attempt_summary') or {}).get('accepted_iterations'),
                 'minimize_calls': (r.get('attempt_summary') or {}).get('minimize_calls'),
                 'stall_retunes': (r.get('attempt_summary') or {}).get('stall_retunes'),
                 'restarts': r['termination'].get('restarts'), 'solve_wall_seconds': val(r.get('solve_wall_seconds')),
                 'free_residual_norm': val(r.get('free_residual_norm')), 'grad_norm_at_termination': r['termination'].get('gradNorm'),
                 'elastic_energy': val(r.get('elastic_energy')), 'barrier_energy': val(r.get('barrier_energy')),
                 'kinetic_energy': val(r.get('kinetic_energy')), 'support_work_increment': val(r.get('support_work_increment')),
                 'retuning_energy_change': val(r.get('retuning_energy_change')), 'min_det_F': val(r.get('min_det_F'))}
        contact = r.get('contact') if isinstance(r.get('contact'), dict) else {}
        frame['active_contacts'] = contact.get('active_count')
        frame['trim'] = contact.get('trim_or_global_stiffness')
        frame['coefficient_range'] = val(contact.get('coefficient_range'))
        frame['batch_floor'] = contact.get('batch_floor')
        frame['batch_cap'] = contact.get('batch_cap')
        frame['batch_median'] = contact.get('batch_median')
        frame['coefficient_zero_count'] = contact.get('coefficient_zero_count')
        frame['curvature_fallbacks'] = [contact.get('curvature_fallback_count'), contact.get('curvature_abs_fallback_count'), contact.get('curvature_global_fallback_count')]
        frame['continued_count'] = contact.get('continued_count')
        frame['fresh_count'] = contact.get('fresh_count')
        frame['min_gap_record'] = val(contact.get('min_gap'))
        # prescribed displacement of the top face
        top_u = u[sets['top']]
        frame['top_uz_mean'] = float(top_u[:, 2].mean())
        frame['top_uz_spread'] = float(np.ptp(top_u[:, 2]))
        delta = -frame['top_uz_mean']
        frame['delta'] = delta
        # support force on the top face (reactions carry the constrained DOFs)
        if reaction is not None:
            R = reaction[:n_body].reshape(-1, DIM)
            frame['support_force_total'] = R.sum(axis=0).tolist()
            frame['support_force_top_z'] = float(R[sets['top'], 2].sum())
            max_abs_R = max(max_abs_R, abs(frame['support_force_top_z']))
            frame['support_force_symmetry_x'] = float(R[:, 0].sum())
            frame['support_force_symmetry_y'] = float(R[:, 1].sum())
            if hard:
                bottom_R = R[sets['bottom'], 2]
                frame['bottom_reaction_z_total'] = float(bottom_R.sum())
                # the floor pushes the body up: a compressive constraint reaction has the sign of +z on the body
                frame['bottom_reaction_tensile_count'] = int(np.sum(bottom_R * np.sign(bottom_R.sum()) < 0))
                frame['bottom_reaction_min_max'] = [float(bottom_R.min()), float(bottom_R.max())]
        gc = grads.get('barrier-contact')
        if gc is not None:
            Fc = -gc[:n_body].reshape(-1, DIM)       # force on the body = -gradient
            frame['contact_force_on_body'] = Fc.sum(axis=0).tolist()
            for b, mask in enumerate(sets['blocks']):
                frame[f'contact_force_block{b}'] = Fc[mask].sum(axis=0).tolist()
            frame['contact_force_from_obstacles'] = []
            offset = n_body
            for g in scene['geometry']:
                if g.get('is_obstacle'):
                    nv = sum(1 for line in (run_dir / g['mesh']).read_text().splitlines() if line.startswith('v '))
                    seg = -gc[offset:offset + nv * DIM].reshape(-1, DIM).sum(axis=0)
                    frame['contact_force_from_obstacles'].append(seg.tolist())
                    offset += nv * DIM
            contacting = np.abs(Fc).sum(axis=1) > 0
            frame['contacting_nodes'] = int(contacting.sum())
            bottom = sets['bottom']
            bottom_contacting = bottom[contacting[bottom]]
            frame['bottom_nodes'] = int(len(bottom))
            frame['bottom_contacting_nodes'] = int(len(bottom_contacting))
            frame['contacting_nodes_off_bottom'] = int(contacting.sum() - len(bottom_contacting))
            gaps = nodes[bottom, 2] + u[bottom, 2] - floor_z
            frame['gap_all_bottom_nodes'] = {'mean': float(gaps.mean()), 'min': float(gaps.min()), 'max': float(gaps.max())}
            if len(bottom_contacting):
                gc_ = nodes[bottom_contacting, 2] + u[bottom_contacting, 2] - floor_z
                ratio = gc_ / dhat
                frame['gap'] = {'mean': float(gc_.mean()), 'min': float(gc_.min()), 'max': float(gc_.max()), 'spread': float(np.ptp(gc_))}
                frame['gap_over_dhat'] = {'mean': float(ratio.mean()), 'min': float(ratio.min()), 'max': float(ratio.max())}
                frame['band_coverage'] = float(np.mean((ratio >= BAND[0]) & (ratio <= BAND[1])))
                frame['below_band_fraction'] = float(np.mean(ratio < BAND[0]))
                frame['above_band_fraction'] = float(np.mean(ratio > BAND[1]))
            if len(sets['blocks']) == 2:
                lo, up = sets['interface_lower_top'], sets['interface_upper_bottom']
                zi_lo = nodes[lo, 2] + u[lo, 2]
                zi_up = nodes[up, 2] + u[up, 2]
                frame['interface'] = {'lower_top_z_mean': float(zi_lo.mean()), 'upper_bottom_z_mean': float(zi_up.mean()),
                                      'gap_mean': float(zi_up.mean() - zi_lo.mean()),
                                      'force_on_lower_from_interface_z': float(Fc[lo, 2].sum()), 'force_on_upper_from_interface_z': float(Fc[up, 2].sum())}
        gi = grads.get('inertia')
        frame['inertia_force_on_body'] = (-gi[:n_body].reshape(-1, DIM).sum(axis=0)).tolist() if gi is not None else None
        ge = grads.get('elastic')
        if ge is not None:
            Fe = -ge[:n_body].reshape(-1, DIM)
            frame['elastic_force_top_z'] = float(Fe[sets['top'], 2].sum())
        # lateral displacement of the free face x = 1 (lower block)
        side = u[sets['side_x1'], 0]
        frame['side_ux'] = {'mean': float(side.mean()), 'min': float(side.min()), 'max': float(side.max())}
        # coefficient coverage: active range against the batch floor/cap
        cr, fl, cp = frame['coefficient_range'], frame['batch_floor'], frame['batch_cap']
        if cr and fl is not None:
            frame['coefficient_at_floor'] = bool(cr[0] <= fl * (1 + 1e-12))
            frame['coefficient_at_cap'] = bool(cp is not None and cr[1] >= cp * (1 - 1e-12))
            frame['coefficient_range_over_median'] = [cr[0] / frame['batch_median'], cr[1] / frame['batch_median']] if frame['batch_median'] else None
        # --- block benchmark: exact references and checks --------------------
        # Everything is compared in SI: lengths measured in the run's units are
        # divided by s (1 for metres, 1000 for the mm run), forces are N in
        # both systems (mm-N-s), energies N*mm = J * s.
        if is_block:
            n_blocks = len(sets['blocks'])
            delta_si = delta / s
            g0_si, H_si = 0.02, 1.0
            delta_per_block = (delta_si - n_blocks * g0_si) / n_blocks
            hard_ref = ref.block_reference(delta_per_block + g0_si, g0_si, H_si, E, nu)
            frame['reference_hard'] = {k: hard_ref[k] for k in ('lz', 'l', 'R_z', 'side_displacement', 'W')}
            dR = ref.reaction_sensitivity(hard_ref['lz'], E, nu)
            c = abs(dR) / (H_si * abs(hard_ref['R_z']))
            dl = (ref.uniaxial(hard_ref['lz'] + 1e-6, E, nu)['l'] - ref.uniaxial(hard_ref['lz'] - 1e-6, E, nu)['l']) / 2e-6
            frame['reference_conditioning'] = {'dR_dlz': dR, 'c_relative_reaction_per_metre': c, 'dl_dlz': dl}
            R_z = frame.get('support_force_top_z')
            checks = {}
            if R_z is not None and 'contact_force_block0' in frame:
                inertia_z = frame['inertia_force_on_body'][2] if frame['inertia_force_on_body'] is not None else 0.0
                if n_blocks == 2:
                    floor = frame['contact_force_from_obstacles'][0][2]
                    bal = abs(-floor + R_z + inertia_z) / abs(R_z)
                    checks['T10_floor_vs_top'] = {'value': bal, 'threshold': 1e-5, 'pass': bal <= 1e-5}
                    inter = frame['interface']['force_on_upper_from_interface_z']
                    bal_i = abs(inter + R_z) / abs(R_z)
                    checks['T10_interface_vs_top'] = {'value': bal_i, 'threshold': 1e-5, 'pass': bal_i <= 1e-5}
                else:
                    L_z = frame['contact_force_block0'][2]
                    scale = max(abs(R_z), max_abs_R)
                    bal = abs(L_z + R_z + inertia_z) / scale
                    checks['T1_force_balance'] = {'value': bal, 'threshold': 1e-5, 'pass': bal <= 1e-5, 'scale': scale}
            if R_z is not None and 'gap' in frame:
                R_ref = hard_ref['R_z']
                e_R = abs(R_z - R_ref) / abs(R_ref)
                gbar_si = frame['gap']['mean'] / s
                spread_si = frame['gap']['spread'] / s
                if n_blocks == 2 and 'interface' in frame:
                    # each block is compressed by (delta - 2 g0 + g_floor + g_interface) / 2
                    lo, up = sets['interface_lower_top'], sets['interface_upper_bottom']
                    gi = (nodes[up, 2] + u[up, 2]).mean() - (nodes[lo, 2] + u[lo, 2]).mean()
                    gi_spread = float(np.ptp(nodes[up, 2] + u[up, 2]) + np.ptp(nodes[lo, 2] + u[lo, 2]))
                    frame['interface']['gap_over_dhat'] = gi / dhat
                    gbar_si = float((frame['gap']['mean'] + gi) / 2 / s)
                    spread_si = float(max(frame['gap']['spread'], gi_spread) / s)
                    frame['effective_gap_per_block_si'] = gbar_si
                e_pred = c * gbar_si
                frame['hard_contact_error'] = {'e_R': float(e_R), 'e_pred_from_mean_gap': float(e_pred), 'signed': float((R_z - R_ref) / abs(R_ref))}
                checks['T2_error_explained_by_gap'] = {'value': float(abs(e_R - e_pred)), 'threshold': float(0.1 * e_pred + 1e-5), 'pass': bool(abs(e_R - e_pred) <= 0.1 * e_pred + 1e-5)}
                cons = ref.block_reference(delta_per_block + g0_si, g0_si, H_si, E, nu, gap=gbar_si)
                c_cons = abs(ref.reaction_sensitivity(cons['lz'], E, nu)) / abs(cons['R_z'])
                e_cons = abs(R_z - cons['R_z']) / abs(cons['R_z'])
                thr3 = c_cons * spread_si / H_si + 1e-6
                frame['barrier_consistent'] = {'R_ref': cons['R_z'], 'e_R': e_cons, 'signed': (R_z - cons['R_z']) / abs(cons['R_z']), 'lz': cons['lz'], 'l': cons['l'],
                                               'side_displacement_si': cons['side_displacement']}
                checks['T3_barrier_consistent_reaction'] = {'value': float(e_cons), 'threshold': float(thr3), 'pass': bool(e_cons <= thr3),
                                                             'a_priori_threshold': c_cons * (BAND[1] - BAND[0]) * dhat / s / H_si + 1e-6}
                e_side = abs(frame['side_ux']['mean'] / s - cons['side_displacement'])
                thr4 = abs(dl) * spread_si / H_si + 1e-8
                checks['T4_lateral_displacement'] = {'value': float(e_side), 'threshold': float(thr4), 'pass': bool(e_side <= thr4),
                                                     'side_ux_spread_si': (frame['side_ux']['max'] - frame['side_ux']['min']) / s}
                frame['elastic_energy_reference_at_mean_gap_si'] = cons['W'] * n_blocks   # unit volume per block
                frame['elastic_energy_si'] = frame['elastic_energy'] / s if frame['elastic_energy'] is not None else None
            if meta.get('unload_at') and R_z is not None:
                frame['unload_state'] = {'active_contacts': frame.get('active_contacts'), 'R_z': R_z}
            frame['checks'] = checks
        out['frames'].append(frame)
    if out['frames']:
        f = out['frames'][-1]
        out['endpoint'] = {k: f.get(k) for k in ('time', 'delta', 'support_force_top_z', 'contact_force_on_body', 'contact_force_from_obstacles', 'gap', 'gap_over_dhat',
                                                 'band_coverage', 'coefficient_range', 'coefficient_at_floor', 'coefficient_at_cap', 'trim', 'active_contacts', 'side_ux',
                                                 'reference_hard', 'hard_contact_error', 'barrier_consistent', 'checks', 'bottom_reaction_tensile_count', 'bottom_reaction_z_total',
                                                 'inertia_force_on_body', 'kinetic_energy', 'elastic_energy', 'elastic_energy_si', 'elastic_energy_reference_at_mean_gap_si', 'min_det_F', 'unload_state', 'reference_conditioning', 'effective_gap_per_block_si', 'interface')}
        out['cost'] = {'newton_iterations_total': sum(fr.get('accepted_iterations_all_minimizes') or 0 for fr in out['frames']),
                       'minimize_calls_total': sum(fr.get('minimize_calls') or 0 for fr in out['frames']),
                       'stall_retunes_total': sum(fr.get('stall_retunes') or 0 for fr in out['frames']),
                       'restarts_total': sum(fr.get('restarts') or 0 for fr in out['frames']),
                       'solve_wall_seconds_total': sum(fr.get('solve_wall_seconds') or 0 for fr in out['frames']),
                       'wall_seconds': meta.get('wall_seconds')}
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs', type=Path, required=True)
    p.add_argument('--out', type=Path)
    a = p.parse_args()
    results = []
    for run_dir in sorted(d for d in a.runs.iterdir() if (d / 'run.json').exists()):
        results.append(analyze_run(run_dir))
    out = a.out or (a.runs / 'endpoints.json')
    out.write_text(json.dumps(results, indent=1) + '\n')
    for r in results:
        e = r.get('endpoint') or {}
        chk = e.get('checks') or {}
        flags = ' '.join(f"{k.split('_')[0]}:{'ok' if v.get('pass') else ('--' if v.get('pass') is None else 'FAIL')}" for k, v in chk.items())
        gap = e.get('gap_over_dhat') or {}
        print(f"{r['name']:<34} {r['status']:<10} R_z={e.get('support_force_top_z')!s:>22} gap/dhat={gap.get('mean', float('nan')):.4f} band={e.get('band_coverage')!s:>6} "
              f"eR={(e.get('hard_contact_error') or {}).get('e_R', float('nan')):.3e} {flags}")


if __name__ == '__main__':
    main()
