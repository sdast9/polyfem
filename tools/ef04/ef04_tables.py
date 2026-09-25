#!/usr/bin/env python3
"""EF-04 tables: stall trigger on the absolute alpha vs the feasible bound.

usage: ef04_tables.py EF04_DIR [--ef01 EF01_DIR] [--ref SCENE=LABEL ...] [--json OUT]

Per run (EF04_DIR/runs/*, plus the EF-01 production baselines of the same
scenes when --ef01 is given): exit, wall, accepted iterations per step, stall
restarts by trigger (run.log), the small-alpha iterations (accepted alpha <
0.01, the production alpha_threshold) split into "at the feasible bound"
(accepted/feasible >= 0.999) and "backtracked", the end trim, the trial-cap and
CCD binding fractions, physical_balance_pass, and the relative L2 error of
`solution` per step against EF-01's pinned tight reference (ref-SCENE).
Also (EF-04b): the solve attempts (count and longest; an attempt is one
PolySolve minimize, ended by convergence or a stall restart) and step 1's trim
walk (accepted iterations until the trim first leaves 1 and first reaches
2^-10) and its trim moves by source (in-solve controller, stall retune,
other refreshes). --ref SCENE=LABEL takes the error against a run of EF04_DIR itself, for
scenes EF-01 has no reference of (the held-out IT and BB).
Reuses tools/ef01/ef01_reduce.py; writes EF04_DIR/tables.md and metrics.json.
"""
import argparse, json, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'ef01'))
import ef01_reduce  # noqa: E402

BASELINES = {  # EF-01 production runs (binary PolyFEM_bin-ef01, default trigger)
    'smoke-qs': ['prod-smoke-qs'], 'smoke-tr': ['prod-smoke-tr'],
    'R1': ['prod-R1', 'rep-R1-prod'], 'BBT': ['prod-BBT'],
    'R4': ['prod-R4', 'rep-R4-prod', 's5-R4-prod', 's5-R4-k-12'],
}
ALPHA, AT_BOUND = 0.01, 0.999


def small_alpha(run):
    small = at_bound = backtracked = unknown = 0
    for r in ef01_reduce.jsonl(Path(run) / 'output/solver-attempts.jsonl'):
        if r.get('kind') != 'accepted':
            continue
        a = ((r.get('solver') or {}).get('accepted') or {}).get('alpha')
        if a is None or not a < ALPHA:
            continue
        small += 1
        ratio = ((r.get('solver') or {}).get('line_search') or {}).get('accepted_over_feasible')
        if ratio is None:
            unknown += 1
        elif ratio >= AT_BOUND:
            at_bound += 1
        else:
            backtracked += 1
    return {'small_alpha': small, 'at_bound': at_bound, 'backtracked': backtracked, 'unclassified': unknown}


def attempts(run):
    """Accepted iterations per (step, minimize_index): one PolySolve minimize each."""
    per = {}
    for r in ef01_reduce.jsonl(Path(run) / 'output/solver-attempts.jsonl'):
        if r.get('kind') == 'accepted':
            key = (r.get('step'), r.get('minimize_index'))
            per[key] = per.get(key, 0) + 1
    lengths = list(per.values())
    return {'count': len(lengths), 'longest': max(lengths, default=0), 'over_500': sum(1 for n in lengths if n > 500)}


def trim_walk(run, step=1):
    """Accepted iterations of STEP until the trim first leaves 1 and first reaches 2^-10."""
    first_below_1 = first_2m10 = None
    k = 0
    for r in ef01_reduce.jsonl(Path(run) / 'output/trim-predictors.jsonl'):
        if r.get('step') != step or r.get('event') != 'iteration':
            continue
        k += 1
        t = r.get('trim')
        if t is None:
            continue
        if first_below_1 is None and t < 1:
            first_below_1 = k
        if first_2m10 is None and t <= 2.0 ** -10:
            first_2m10 = k
    return {'below_1': first_below_1, 'at_2m10': first_2m10}


def trim_moves(run, step=1):
    """STEP's trim changes by source: between consecutive records, attributed to
    the later record's event (iteration = in-solve controller, stall_retune =
    the restart's retune, refresh/refresh_endpoint = other refreshes)."""
    moves = {}
    prev = None
    for r in ef01_reduce.jsonl(Path(run) / 'output/trim-predictors.jsonl'):
        if r.get('step') != step or r.get('trim') is None:
            continue
        if prev is not None and r['trim'] != prev:
            src = {'iteration': 'in_solve', 'stall_retune': 'retune'}.get(r.get('event'), 'other')
            key = f"{src}_{'down' if r['trim'] < prev else 'up'}"
            moves[key] = moves.get(key, 0) + 1
        prev = r['trim']
    return moves


def fmt_moves(m):
    return ', '.join(f"{src} {m.get(src + '_down', 0)}↓/{m.get(src + '_up', 0)}↑"
                     for src in ('in_solve', 'retune', 'other') if m.get(src + '_down') or m.get(src + '_up')) or '-'


def basis(row):
    o = (row.get('overrides') or {})
    b = o.get('/solver/contact/semi_implicit/restart/alpha_basis', 'absolute')
    t = o.get('/solver/contact/semi_implicit/restart/feasible_ratio_threshold')
    b = b if t is None else f'{b} ({t})'
    soft = o.get('/solver/contact/semi_implicit/restart/soft_iteration_limit')
    if soft is not None:
        b += ', soft ' + ('off' if soft <= 0 else str(soft))
    return b


def fmt_err(errs):
    return ', '.join(f'{e:.1e}' for e in errs) if errs else '-'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('--ef01')
    ap.add_argument('--ref', action='append', default=[],
                    help='SCENE=LABEL: error against the run LABEL of this directory')
    ap.add_argument('--json')
    a = ap.parse_args()
    local_refs = dict(item.split('=', 1) for item in a.ref)
    root = Path(a.dir)
    runs = sorted((root / 'runs').iterdir()) if (root / 'runs').exists() else []
    by_scene = {}
    for run in runs:
        if not (run / 'row.json').exists():
            continue
        row = json.loads((run / 'row.json').read_text())
        by_scene.setdefault(row['scene'], []).append((run, 'ef04'))
    ef01 = Path(a.ef01) if a.ef01 else None
    if ef01:
        for scene, labels in BASELINES.items():
            if scene in by_scene:
                by_scene[scene] = [(ef01 / 'runs' / l, 'ef01') for l in labels if (ef01 / 'runs' / l / 'row.json').exists()] + by_scene[scene]
    out, md = [], ['# EF-04 tables', '',
                   'small α = accepted α < 0.01; "at bound" = accepted/feasible ≥ 0.999. '
                   'Error = relative L2 of `solution` per step against EF-01 `ref-SCENE` (pinned trim, tight tolerance). '
                   'Baselines from EF-01 ran binary `PolyFEM_bin-ef01` (sha256 1df8e4cb…).', '']
    for scene, items in by_scene.items():
        ref = ef01 / 'runs' / f'ref-{scene}' if ef01 and (ef01 / 'runs' / f'ref-{scene}').exists() else None
        if scene in local_refs:
            ref = root / 'runs' / local_refs[scene]
        md += [f'## {scene}', '', f"Error against `{ref.name if ref else '-'}`.", '',
               '| run | source | basis | exit | wall s | its (per step) | restarts by trigger | attempts: n / longest | step-1 trim walk: <1 / ≤2⁻¹⁰ at it | step-1 trim moves | small α: at bound / backtracked | trim end | cap / CCD bound | balance | error per step |',
               '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
        for run, source in items:
            r = ef01_reduce.reduce_run(run, ref)
            row = json.loads((run / 'row.json').read_text())
            r['source'] = source
            r['basis'] = basis(row)
            r['small_alpha'] = small_alpha(run)
            r['attempts'] = attempts(run)
            r['trim_walk'] = trim_walk(run)
            r['trim_moves'] = trim_moves(run)
            out.append(r)
            last = r['steps'].get(max(r['steps'], default=None), {}) if r['steps'] else {}
            trim = ((last or {}).get('predictors') or {}).get('trim_last')
            log = r['log']
            f = lambda v: f'{v:.2f}' if v is not None else '-'
            sa = r['small_alpha']
            status = 'timeout' if r['timed_out'] else r['exit_code']
            md.append(
                f"| {r['label']} | {source} | {r['basis']} | {status} | {r['wall_seconds']} | {r['iterations']} ({', '.join(map(str, r['iterations_per_step']))}) | "
                f"{log.get('restarts_by_trigger') or {}} | {r['attempts']['count']} / {r['attempts']['longest']} | "
                f"{r['trim_walk']['below_1'] or '-'} / {r['trim_walk']['at_2m10'] or '-'} | {fmt_moves(r['trim_moves'])} | {sa['small_alpha']}: {sa['at_bound']} / {sa['backtracked']} | "
                f"{trim if trim is None else f'{trim:.3g}'} | {f(log.get('trial_cap_active_fraction'))} / {f(log.get('ccd_bound_fraction'))} | "
                f"{''.join('T' if s.get('physical_balance_pass') else ('F' if s.get('physical_balance_pass') is False else '-') for s in r['steps'].values())} | "
                f"{fmt_err(r.get('relative_error_per_step'))} |")
        md.append('')
    (root / 'tables.md').write_text('\n'.join(md) + '\n')
    Path(a.json or root / 'metrics.json').write_text(json.dumps(out, indent=1, default=str) + '\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
