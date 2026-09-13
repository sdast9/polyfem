"""RB-09 sweep-level summary of the analyzed matrix (T5-T10, T12-T14 and the tables).

Usage: python3 tools/rb09/summarize.py --endpoints /abs/block/endpoints.json [--clamped /abs/clamped/endpoints.json] --out tools/rb09/results-YYYYMMDD.json

Per-run checks (T1-T4, T11) live in analyze.py / spring_probe.cpp; this
script fits the dhat-sweep rate, compares the sweeps' spreads, the unit
conversion pair, the transient runs against the quasistatic barrier-consistent
reference, the unload endpoint, the stacked interfaces, and the clamped
benchmark against its same-mesh hard-contact references. Missing runs are
reported as unavailable.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

BAND_WIDTH = math.sqrt(0.9) - math.sqrt(0.5)


def by_name(results):
    return {r['name']: r for r in results}


def endpoint(r):
    return (r or {}).get('endpoint') or {}


def rel(a, b):
    return abs(a - b) / abs(b)


def block_summary(res):
    R = by_name(res)
    out = {'runs': {}, 'checks': {}}
    for name, r in R.items():
        e = endpoint(r)
        out['runs'][name] = {
            'status': r['status'], 'group': r['group'], 'config': r['config'],
            'R_z': e.get('support_force_top_z'), 'R_hard': (e.get('reference_hard') or {}).get('R_z'),
            'e_R_hard': (e.get('hard_contact_error') or {}).get('e_R'), 'e_pred': (e.get('hard_contact_error') or {}).get('e_pred_from_mean_gap'),
            'e_R_consistent': (e.get('barrier_consistent') or {}).get('e_R'),
            'gap_over_dhat': e.get('gap_over_dhat'), 'band_coverage': e.get('band_coverage'), 'trim': e.get('trim'),
            'coefficient_range': e.get('coefficient_range'), 'at_floor': e.get('coefficient_at_floor'), 'at_cap': e.get('coefficient_at_cap'),
            'side_ux_mean': (e.get('side_ux') or {}).get('mean'), 'active_contacts': e.get('active_contacts'),
            'checks': {k: v.get('pass') for k, v in (e.get('checks') or {}).items()},
            'cost': r.get('cost'), 'failed_attempts': r.get('failed_attempts'),
        }
    base = R.get('block-base')
    eb = endpoint(base)

    # T5: dhat sweep rate (hard-contact error vs dhat)
    sweep = [(R[n]['config']['dhat'], endpoint(R[n])['hard_contact_error']['e_R']) for n in
             ['block-dhat0.004', 'block-dhat0.002', 'block-base', 'block-dhat0.0005'] if n in R and endpoint(R[n]).get('hard_contact_error')]
    if len(sweep) >= 3:
        d = np.log([s[0] for s in sweep])
        e = np.log([s[1] for s in sweep])
        slope = float(np.polyfit(d, e, 1)[0])
        out['checks']['T5_dhat_rate'] = {'points': sweep, 'slope': slope, 'threshold': [0.85, 1.15], 'pass': 0.85 <= slope <= 1.15}
    else:
        out['checks']['T5_dhat_rate'] = {'unavailable_reason': f'{len(sweep)} sweep points'}

    def spread_check(tag, names):
        vals = [(n, endpoint(R[n]).get('support_force_top_z')) for n in names if n in R and endpoint(R[n]).get('support_force_top_z') is not None]
        if len(vals) < 2:
            return {'unavailable_reason': f'{len(vals)} runs'}
        Rs = [v for _, v in vals]
        e0 = endpoint(R[names[0]])
        c = e0['reference_conditioning']['c_relative_reaction_per_metre'] if e0.get('reference_conditioning') else None
        dhat = R[names[0]]['config']['dhat']
        thr = 2 * c * BAND_WIDTH * dhat * abs(e0['reference_hard']['R_z']) if c else None
        spread = max(Rs) - min(Rs)
        all_pass = all(all(v.get('pass', True) for v in (endpoint(R[n]).get('checks') or {}).values()) for n in names if n in R)
        return {'runs': vals, 'spread': spread, 'threshold': thr, 'pass': (thr is not None and spread <= thr and all_pass), 'all_T1_T4_pass': all_pass}
    out['checks']['T6_mesh_sweep'] = spread_check('mesh', ['block-base', 'block-nrefs1', 'block-nrefs2'])
    out['checks']['T6_increment_sweep'] = spread_check('increment', ['block-base', 'block-dt0.125', 'block-dt0.0625'])

    # T7: unit conversion (raw numbers; with the dimensional solver constants converted; with an equal force tolerance)
    rows = []
    for n in ['block-units-mm', 'block-units-mm-converted', 'block-units-mm-equal-tolerance']:
        mm = endpoint(R.get(n))
        if mm.get('support_force_top_z') is None or eb.get('support_force_top_z') is None:
            rows.append({'name': n, 'unavailable_reason': 'missing run'})
            continue
        dR = rel(mm['support_force_top_z'], eb['support_force_top_z'])
        dg = abs(mm['gap_over_dhat']['mean'] - eb['gap_over_dhat']['mean'])
        fr = R[n]['frames'][-1]
        rows.append({'name': n, 'R_mm': mm['support_force_top_z'], 'relative_difference': dR, 'gap_over_dhat': mm['gap_over_dhat']['mean'], 'gap_ratio_difference': dg,
                     'pass': dR <= 1e-6 and dg <= 1e-6, 'newton_iterations': R[n]['cost']['newton_iterations_total'], 'restarts': R[n]['cost']['restarts_total'],
                     'trim': mm.get('trim'), 'free_residual_N': fr.get('free_residual_norm'), 'checks': {k: v.get('pass') for k, v in (mm.get('checks') or {}).items()},
                     'e_R_hard': (mm.get('hard_contact_error') or {}).get('e_R'), 'e_pred': (mm.get('hard_contact_error') or {}).get('e_pred_from_mean_gap')})
    out['checks']['T7_unit_conversion'] = {'R_m': eb.get('support_force_top_z'), 'gap_over_dhat_m': (eb.get('gap_over_dhat') or {}).get('mean'), 'newton_m': R['block-base']['cost']['newton_iterations_total'],
                                           'free_residual_m_N': R['block-base']['frames'][-1].get('free_residual_norm'), 'trim_m': eb.get('trim'), 'variants': rows, 'threshold': 1e-6,
                                           'pass': bool(rows) and all(r.get('pass') for r in rows)}

    # T8: transient dt sweep against the quasistatic barrier-consistent reference at the run's own mean gap
    tr = []
    for n in ['block-transient-dt0.25', 'block-transient-dt0.125', 'block-transient-dt0.0625', 'block-transient-dt0.03125']:
        e = endpoint(R.get(n))
        if e.get('support_force_top_z') is None:
            continue
        tr.append({'name': n, 'dt': R[n]['config']['dt'], 'R_z': e['support_force_top_z'], 'e_consistent': e['barrier_consistent']['e_R'],
                   'signed': e['barrier_consistent']['signed'], 'e_hard': e['hard_contact_error']['e_R'], 'gap_over_dhat': e['gap_over_dhat']['mean'],
                   'inertia_z': (e.get('inertia_force_on_body') or [None, None, None])[2], 'kinetic_energy': e.get('kinetic_energy'),
                   'T1': (e['checks'].get('T1_force_balance') or {}).get('pass'), 'newton': R[n]['cost']['newton_iterations_total']})
    if tr:
        errs = [t['e_consistent'] for t in tr]
        mono = all(errs[i + 1] <= errs[i] * (1 + 1e-9) for i in range(len(errs) - 1))
        order = None
        if len(tr) >= 3:
            # observed order from the three finest dt on R_z differences (Richardson)
            r1, r2, r3 = [t['R_z'] for t in tr[-3:]]
            if (r2 - r3) != 0 and (r1 - r2) / (r2 - r3) > 0:
                order = float(math.log2((r1 - r2) / (r2 - r3)))
        out['checks']['T8_transient'] = {'runs': tr, 'max_e_consistent': max(errs), 'threshold': 1e-2, 'non_increasing': mono,
                                         'observed_order_R_z': order, 'pass': max(errs) <= 1e-2 and all(t['T1'] for t in tr)}
    else:
        out['checks']['T8_transient'] = {'unavailable_reason': 'no transient run'}

    # T9: unload
    ul = R.get('block-unload')
    if ul and ul.get('frames'):
        frames = ul['frames']
        loading = {round(f['delta'], 9): f for f in frames if f['time'] <= 1.0 + 1e-9}
        rows = []
        for f in frames:
            if f['time'] > 1.0 + 1e-9:
                twin = loading.get(round(f['delta'], 9))
                rows.append({'time': f['time'], 'delta': f['delta'], 'R_z': f.get('support_force_top_z'), 'R_z_loading_twin': twin.get('support_force_top_z') if twin else None,
                             'active': f.get('active_contacts'), 'gap_over_dhat': (f.get('gap_over_dhat') or {}).get('mean'), 'trim': f.get('trim'),
                             'T2': (f.get('checks') or {}).get('T2_error_explained_by_gap', {}).get('pass')})
        last = frames[-1]
        Rmax = max(abs(f.get('support_force_top_z') or 0) for f in frames)
        final_ok = last.get('active_contacts') == 0 and abs(last.get('support_force_top_z') or 0) <= 1e-6 * Rmax
        trims = [f.get('trim') for f in frames]
        out['checks']['T9_unload'] = {'unloading_steps': rows, 'final_active_contacts': last.get('active_contacts'), 'final_R_z': last.get('support_force_top_z'),
                                      'R_max': Rmax, 'trim_history': trims, 'pass': final_ok and all(r['T2'] for r in rows if r['active'])}
    else:
        out['checks']['T9_unload'] = {'unavailable_reason': 'missing run'}

    # T10: stacked
    st = endpoint(R.get('block-stacked'))
    if st.get('checks'):
        out['checks']['T10_stacked'] = {'checks': st['checks'], 'R_z': st.get('support_force_top_z'), 'floor': st.get('contact_force_from_obstacles'),
                                        'gap_over_dhat_floor': st.get('gap_over_dhat'), 'e_R_hard': (st.get('hard_contact_error') or {}).get('e_R'),
                                        'pass': all(v.get('pass') for v in st['checks'].values())}
        fr = R['block-stacked']['frames'][-1]
        out['checks']['T10_stacked']['interface'] = fr.get('interface')
    else:
        out['checks']['T10_stacked'] = {'unavailable_reason': 'missing run'}

    # comparisons without pass/fail: material, density, speed, controllers
    out['tables'] = {}
    for tag, names in {'material': ['block-E1e+06', 'block-base', 'block-E1e+08'],
                       'density': ['block-transient-dt0.125', 'block-transient-rho10'],
                       'speed': ['block-transient-rate0.125', 'block-transient-rate0.25', 'block-transient-rate0.5'],
                       'controller': ['block-base', 'block-adaptive', 'block-fixed']}.items():
        rows = []
        for n in names:
            r = R.get(n)
            e = endpoint(r)
            if not e:
                rows.append({'name': n, 'unavailable_reason': 'missing run'})
                continue
            rows.append({'name': n, 'E': r['config']['E'], 'rho': r['config']['rho'], 'rate': r['config']['rate'], 'barrier': r['config']['barrier'],
                         'R_z': e.get('support_force_top_z'), 'R_hard': e['reference_hard']['R_z'], 'e_R_hard': (e.get('hard_contact_error') or {}).get('e_R'),
                         'e_R_consistent': (e.get('barrier_consistent') or {}).get('e_R'), 'gap_over_dhat': e.get('gap_over_dhat'), 'band_coverage': e.get('band_coverage'),
                         'trim_or_kappa': e.get('trim'), 'coefficient_range': e.get('coefficient_range'), 'checks': {k: v.get('pass') for k, v in (e.get('checks') or {}).items()},
                         'newton': r['cost']['newton_iterations_total'], 'restarts': r['cost']['restarts_total'], 'wall_seconds': r['cost']['wall_seconds'],
                         'inertia_z': (e.get('inertia_force_on_body') or [None] * 3)[2], 'kinetic_energy': e.get('kinetic_energy')})
        out['tables'][tag] = rows
    return out


def clamped_summary(res):
    R = by_name(res)
    out = {'runs': {}, 'checks': {}}
    for name, r in R.items():
        e = endpoint(r)
        out['runs'][name] = {'status': r['status'], 'group': r['group'], 'config': r['config'], 'R_z': e.get('support_force_top_z'),
                             'gap_over_dhat': e.get('gap_over_dhat'), 'band_coverage': e.get('band_coverage'), 'trim': e.get('trim'),
                             'bottom_reaction_tensile_count': e.get('bottom_reaction_tensile_count'), 'bottom_reaction_z_total': e.get('bottom_reaction_z_total'),
                             'side_ux_mean': (e.get('side_ux') or {}).get('mean'), 'cost': r.get('cost'), 'failed_attempts': r.get('failed_attempts')}
    # T12/T13 per mesh
    meshes = []
    for n in (0, 1, 2):
        b, h, l = endpoint(R.get(f'clamped-nrefs{n}')), endpoint(R.get(f'clamped-hard-nrefs{n}')), endpoint(R.get(f'clamped-hard-lift-nrefs{n}'))
        row = {'n_refs': n}
        if b.get('support_force_top_z') is None or h.get('support_force_top_z') is None:
            row['unavailable_reason'] = 'missing barrier or hard run'
            meshes.append(row)
            continue
        Rb, Rh = b['support_force_top_z'], h['support_force_top_z']
        row.update(R_barrier=Rb, R_hard=Rh, e_R=rel(Rb, Rh), signed=(Rb - Rh) / abs(Rh), gap_over_dhat=b['gap_over_dhat'], band_coverage=b['band_coverage'],
                   tensile_count=h.get('bottom_reaction_tensile_count'), hard_bottom_reaction_total=h.get('bottom_reaction_z_total'))
        if l.get('support_force_top_z') is not None:
            lift = R[f'clamped-hard-lift-nrefs{n}']['config']['plane_lift']
            c_h = abs(l['support_force_top_z'] - Rh) / lift / abs(Rh)
            row['c_h_relative_per_metre'] = c_h
            e_pred = c_h * b['gap']['mean']
            row['e_pred_from_mean_gap'] = e_pred
            row['T12'] = {'value': abs(row['e_R'] - e_pred), 'threshold': 0.1 * e_pred + 1e-5, 'pass': abs(row['e_R'] - e_pred) <= 0.1 * e_pred + 1e-5}
        meshes.append(row)
    out['checks']['T12_T13_meshes'] = meshes
    hard = [m for m in meshes if 'R_hard' in m]
    if len(hard) == 3:
        r0, r1, r2 = [m['R_hard'] for m in hard]
        order = math.log2((r0 - r1) / (r1 - r2)) if (r1 - r2) != 0 and (r0 - r1) / (r1 - r2) > 0 else None
        out['checks']['T13_hard_reference_convergence'] = {'R_hard': [r0, r1, r2], 'differences': [r0 - r1, r1 - r2], 'observed_order': order,
                                                            'richardson_limit': (r2 + (r2 - r1) / (2 ** order - 1)) if order else None,
                                                            'barrier_minus_hard': [m['R_barrier'] - m['R_hard'] for m in hard]}
    # T14 increments / transient / dhat
    inc = [(n, endpoint(R[n]).get('support_force_top_z')) for n in ['clamped-nrefs0', 'clamped-dt0.125', 'clamped-dt0.0625'] if n in R]
    if len(inc) >= 2 and all(v is not None for _, v in inc) and meshes and meshes[0].get('c_h_relative_per_metre'):
        Rs = [v for _, v in inc]
        thr = 2 * meshes[0]['c_h_relative_per_metre'] * BAND_WIDTH * 1e-3 * abs(Rs[0])
        out['checks']['T14_increment'] = {'runs': inc, 'spread': max(Rs) - min(Rs), 'threshold': thr, 'pass': max(Rs) - min(Rs) <= thr}
    tr = []
    for n in ['clamped-transient-dt0.25', 'clamped-transient-dt0.125', 'clamped-transient-dt0.0625']:
        e = endpoint(R.get(n))
        if e.get('support_force_top_z') is not None:
            tr.append({'name': n, 'dt': R[n]['config']['dt'], 'R_z': e['support_force_top_z'], 'gap_over_dhat': e['gap_over_dhat']['mean'],
                       'inertia_z': (e.get('inertia_force_on_body') or [None] * 3)[2], 'kinetic_energy': e.get('kinetic_energy')})
    q = endpoint(R.get('clamped-nrefs0')).get('support_force_top_z')
    if tr and q is not None:
        for t in tr:
            t['relative_to_quasistatic'] = (t['R_z'] - q) / abs(q)
        out['checks']['T14_transient'] = {'quasistatic_R_z': q, 'runs': tr, 'max_relative': max(abs(t['relative_to_quasistatic']) for t in tr), 'threshold': 1e-2,
                                          'pass': max(abs(t['relative_to_quasistatic']) for t in tr) <= 1e-2}
    dh = []
    for n in ['clamped-dhat0.002', 'clamped-nrefs0', 'clamped-dhat0.0005']:
        e = endpoint(R.get(n))
        if e.get('support_force_top_z') is not None and meshes and 'R_hard' in meshes[0]:
            dh.append({'name': n, 'dhat': R[n]['config']['dhat'], 'R_z': e['support_force_top_z'], 'e_R_hard': rel(e['support_force_top_z'], meshes[0]['R_hard']),
                       'gap_over_dhat': e['gap_over_dhat']['mean'], 'band_coverage': e['band_coverage']})
    if len(dh) >= 3:
        slope = float(np.polyfit(np.log([d['dhat'] for d in dh]), np.log([d['e_R_hard'] for d in dh]), 1)[0])
        out['checks']['T14_dhat'] = {'runs': dh, 'slope': slope, 'pass': 0.85 <= slope <= 1.15}
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--endpoints', type=Path, required=True)
    p.add_argument('--clamped', type=Path)
    p.add_argument('--spring', type=Path)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    out = {'block': block_summary(json.loads(a.endpoints.read_text()))}
    if a.clamped and a.clamped.exists():
        out['clamped'] = clamped_summary(json.loads(a.clamped.read_text()))
    if a.spring and a.spring.exists():
        pr = json.loads(a.spring.read_text())
        out['spring'] = {'passed': pr.get('passed'), 'checks': pr.get('checks'), 'cases': [
            {k: c.get(k) for k in ('k', 'dhat', 'passed', 'max_gap_error_over_dhat', 'max_spring_vs_barrier_force', 'max_form_vs_analytical_gradient', 'max_hard_contact_identity')}
            | {'steps': [{k: s.get(k) for k in ('step', 'anchor_y', 'gap_over_dhat', 'first_iterate_gap_over_dhat', 'trim_at_endpoint', 'trim_after_between_steps_refresh',
                                                 'newton_iterations', 'hard_contact_relative_error', 'kappa_s')} for s in c.get('steps', [])]}
            for c in pr.get('cases', [])]}
    a.out.write_text(json.dumps(out, indent=1) + '\n')
    for section, content in out.items():
        print(f'== {section} ==')
        checks = content.get('checks')
        if isinstance(checks, dict):
            for k, v in checks.items():
                print(f"  {k}: pass={v.get('pass') if isinstance(v, dict) else v}")
        else:
            print(f"  passed={content.get('passed')} checks={checks}")


if __name__ == '__main__':
    main()
