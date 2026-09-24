#!/usr/bin/env python3
"""EF-01 tables from an evidence directory (run from anywhere).

usage: ef01_tables.py EVIDENCE_DIR [--scene S ...] [--ref-label PATTERN]

For every scene: the pinned-trim sweep (runs pin-SCENE-kK) next to the
production run (prod-SCENE) and any other runs named *-SCENE-*: iterations,
wall, restarts by trigger, trial-cap/CCD binding, endpoint gap statistics
(controller rms, force-weighted mean, p10/p50/p90), active pairs,
physical_balance_pass, the small-alpha iterations that sat at the feasible
bound (H-E), and the solution error against the reference run (default:
ref-SCENE if it exists). Then the predictor table: the candidates of the
first full record with contact in the production run, next to the sweep's
cheapest trim. Writes tables.md and metrics.json into the evidence directory.
"""
import argparse, json, math, re, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import ef01_reduce as red  # noqa: E402


def small_alpha_at_bound(run):
    rows = red.jsonl(Path(run) / 'output/solver-attempts.jsonl')
    small = at_bound = 0
    for r in rows:
        if r.get('kind') != 'accepted':
            continue
        s = r.get('solver') or {}
        alpha = (s.get('accepted') or {}).get('alpha')
        ratio = (s.get('line_search') or {}).get('accepted_over_feasible')
        if alpha is None or alpha >= 0.01:
            continue
        small += 1
        if ratio is not None and ratio >= 0.999:
            at_bound += 1
    return small, at_bound


def first_full(run):
    for r in red.jsonl(Path(run) / 'output/trim-predictors.jsonl'):
        if r.get('event') != 'iteration' and (r.get('active_count') or 0) > 0:
            return r
    return None


def fmt(v, spec='.3g'):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return '-'
    return format(v, spec)


def kval(label):
    m = re.search(r'-k(-?\d+)$', label)
    return int(m.group(1)) if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('evidence')
    ap.add_argument('--scene', action='append')
    a = ap.parse_args()
    ev = Path(a.evidence)
    runs = sorted(p for p in (ev / 'runs').iterdir() if (p / 'row.json').exists())
    scenes = a.scene or sorted({json.loads((p / 'row.json').read_text())['scene'] for p in runs})
    out, metrics = [], {}
    for scene in scenes:
        mine = [p for p in runs if json.loads((p / 'row.json').read_text())['scene'] == scene]
        ref = next((p for p in mine if p.name.startswith(f'ref-{scene}')), None)
        results = []
        for p in mine:
            r = red.reduce_run(p, ref)
            r['small_alpha'], r['small_alpha_at_bound'] = small_alpha_at_bound(p)
            results.append(r)
        results.sort(key=lambda r: (r['trim_pin'] is None, -(r['trim_pin'] or 0), r['label']))
        metrics[scene] = results
        out.append(f'\n## {scene}' + (f' (reference {ref.name})' if ref else '') + '\n')
        out.append('| run | trim | exit | wall s | its (per step) | restarts | cap | CCD | small-α at bound | trim end | rms gap | fw mean gap | p10/p50/p90 | pairs | balance | err (per step) |')
        out.append('|' + '---|' * 16)
        for r in results:
            last = r['steps'][max(r['steps'])] if r['steps'] else {}
            p = last.get('predictors') or {}
            g = p.get('gap') or {}
            fw = p.get('force_weighted') or {}
            log = r['log']
            exit_text = 'timeout' if r['timed_out'] else str(r['exit_code'])
            out.append(
                f"| {r['label']} | {fmt(r['trim_pin']) if r['trim_pin'] is not None else 'ctrl'} | {exit_text} | {fmt(r['wall_seconds'], '.0f')} | "
                f"{r['iterations']} {r['iterations_per_step']} | {log.get('restarts_by_trigger') or '-'} | "
                f"{fmt(log.get('trial_cap_active_fraction'), '.2f')} | {fmt(log.get('ccd_bound_fraction'), '.2f')} | "
                f"{r['small_alpha_at_bound']}/{r['small_alpha']} | {fmt(p.get('trim_last'))} | {fmt(g.get('rms'))} | "
                f"{fmt(fw.get('mean_gap'))} | {fmt(g.get('p10'), '.2g')}/{fmt(g.get('p50'), '.2g')}/{fmt(g.get('p90'), '.2g')} | "
                f"{p.get('active_pairs', '-')} | {[s.get('physical_balance_pass') for s in r['steps'].values()]} | "
                f"{' '.join(fmt(e, '.1e') for e in r.get('relative_error_per_step', [])) or '-'} |")
        # predictors of the production run's first contact refresh
        prod = next((p for p in mine if p.name == f'prod-{scene}'), None)
        full = first_full(prod) if prod else None
        if full:
            gb = full.get('gradient_balance') or {}
            hr = full.get('hessian_diagonal_ratio') or {}
            m = full.get('multiplicity') or {}
            done = [r for r in results if r['trim_pin'] is not None and not r['timed_out'] and r['exit_code'] == 0]
            best = min(done, key=lambda r: r['iterations']) if done else None
            out.append('')
            out.append(f"Predictors at the first contact refresh of prod-{scene} (step {full['step']}, event {full['event']}, "
                       f"{full.get('active_count')} pairs, trim then {fmt(full.get('trim'))}): "
                       f"κ_gb {fmt(gb.get('kappa_gb'))} (cos {fmt(gb.get('cos_opposition'), '.2f')}, gate {gb.get('gate_passes')}), "
                       f"trim for unit median Hessian ratio {fmt(hr.get('trim_for_unit_median'))}, for unit sum {fmt(hr.get('trim_for_unit_sum'))}, "
                       f"conditioning-cap trim {fmt(full.get('conditioning_cap_trim'))}, multiplicity mean/p90/max "
                       f"{fmt(m.get('mean'))}/{fmt(m.get('p90'))}/{fmt(m.get('max'))}; "
                       f"cheapest completed pin: {fmt(best['trim_pin']) if best else '-'} ({best['iterations'] if best else '-'} its).")
    (ev / 'tables.md').write_text('\n'.join(out) + '\n')
    (ev / 'metrics.json').write_text(json.dumps(metrics, indent=1, default=str) + '\n')
    print('\n'.join(out))


if __name__ == '__main__':
    main()
