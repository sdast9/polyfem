#!/usr/bin/env python3
"""Reduce EF-01 runs to per-run metrics (JSON) and a one-line table.

usage: ef01_reduce.py [--ref RUN] [--json OUT.json] RUN...

Per run and per completed step:
  accepted Newton iterations over all sub-solves (= factorizations for Newton),
  sub-solves, AL passes, restarts by trigger (run.log), the trim at the end of
  the step and its range, the band statistic and gap distribution of the last
  post-step record (trim-predictors.jsonl), active pairs, trial-cap and CCD
  binding (run.log: `trial_clamp` of each line search's broad phase against its
  `collision_free_step_size`, the method of the qn-contact F17), assembly and
  linear-solve time, physical_balance_pass (physical-diagnostics.jsonl), and
  the relative L2 error of `solution` against --ref.
"""
import argparse, json, re, statistics, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'qn_contact'))
from compare_vtu import read_field  # noqa: E402
import numpy as np  # noqa: E402

CLAMP = re.compile(r'Broad phase over trial step: \d+ candidates \(trial Linf=[^,]+, trial_clamp=([0-9.eE+-]+)\)')
LS_DONE = re.compile(r'Line search finished \(nan_free_step_size=([0-9.eE+-]+) collision_free_step_size=([0-9.eE+-]+|inf) descent_step_size=([0-9.eE+-]+|inf) final_step_size=([0-9.eE+-]+|inf)\)')
STALL = re.compile(r'Line-search stall detected \(trigger: (.*?)\); retuning')
AL_SOLVE = re.compile(r'Solving AL Problem with weight ([0-9.eE+-]+)')
AL_INIT = re.compile(r'Using hessian-scaled initial AL weight: ([0-9.eE+-]+)')
TIMING = re.compile(r'\[timing\S*\]\[SparseNewton\] assembly: ([0-9.eE+-]+)s; linear_solve: ([0-9.eE+-]+)s')
TIMING_ANSI = re.compile(r'\x1b\[[0-9;]*m')


def trigger_kind(text):
    if text.startswith('soft iteration budget'):
        return 'soft_budget'
    if 'and the soft iteration budget' in text:
        return 'alpha_and_soft'
    if text.startswith('alpha <'):
        return 'alpha'
    return 'other'


def jsonl(path):
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass  # a run killed mid-write leaves a partial last line
    return rows


def log_metrics(log_text):
    lines = [TIMING_ANSI.sub('', l) for l in log_text.splitlines()]
    searches, clamp = [], None
    stalls, al_weights, al_init = [], [], None
    assembly = linear = 0.0
    for l in lines:
        m = CLAMP.search(l)
        if m:
            clamp = float(m.group(1))
            continue
        m = LS_DONE.search(l)
        if m:
            free = float(m.group(2))
            searches.append((clamp if clamp is not None else 1.0, free, float(m.group(4))))
            clamp = None
            continue
        m = STALL.search(l)
        if m:
            stalls.append(trigger_kind(m.group(1)))
            continue
        m = AL_SOLVE.search(l)
        if m:
            al_weights.append(float(m.group(1)))
            continue
        m = AL_INIT.search(l)
        if m:
            al_init = float(m.group(1))
            continue
        m = TIMING.search(l)
        if m:
            assembly += float(m.group(1))
            linear += float(m.group(2))
    n = len(searches)
    capped = [s for s in searches if s[0] < 1]
    kept = [min(1.0, s[1] / min(1.0, s[0])) for s in searches if s[0] > 0]
    ccd_bound = [k for k in kept if k < 1 - 1e-9]
    kept_capped = [min(1.0, s[1] / s[0]) for s in capped if s[0] > 0]
    out = {
        'line_searches': n,
        'trial_cap_active_fraction': len(capped) / n if n else None,
        'ccd_bound_fraction': len(ccd_bound) / n if n else None,
        'ccd_kept_of_capped_sweep_median': statistics.median(kept_capped) if kept_capped else None,
        'restarts_by_trigger': {k: stalls.count(k) for k in sorted(set(stalls))},
        'restarts': len(stalls),
        'al_solve_weights': al_weights,
        'al_initial_weight': al_init,
        'assembly_seconds': round(assembly, 2),
        'linear_solve_seconds': round(linear, 2),
    }
    return out


def step_predictors(rows):
    """Per step: trim trajectory and the last post-step (endpoint-like) record."""
    per = {}
    for r in rows:
        per.setdefault(r['step'], []).append(r)
    out = {}
    for step, rs in sorted(per.items()):
        trims = [r['trim'] for r in rs]
        last_it = [r for r in rs if r['event'] == 'iteration']
        last = last_it[-1] if last_it else rs[-1]
        full = [r for r in rs if r['event'] != 'iteration']
        entry = {
            'trim_first': trims[0], 'trim_last': trims[-1],
            'trim_min': min(trims), 'trim_max': max(trims),
            'trim_changes': sum(1 for a, b in zip(trims, trims[1:]) if a != b),
            'records': len(rs), 'full_records': len(full),
            'active_pairs': last.get('active_count'),
            'gap': {k: last.get('gap', {}).get(k) for k in ('rms', 'mean', 'min', 'p10', 'p50', 'p90', 'max')},
            'force_weighted': {k: (last.get('force_weighted') or {}).get(k) for k in ('mean_gap', 'rms_gap', 'gap_at_force_fraction', 'force_share_of_closest_tenth')},
            'multiplicity': {k: (last.get('multiplicity') or {}).get(k) for k in ('mean', 'p90', 'max', 'incidence_weighted_mean')},
        }
        out[step] = entry
    return out


def solution_error(run, ref):
    errs = []
    k = 1
    while (run / f'output/step_{k}.vtu').exists() and (ref / f'output/step_{k}.vtu').exists():
        try:
            ua = read_field(run / f'output/step_{k}.vtu', 'solution')
            ur = read_field(ref / f'output/step_{k}.vtu', 'solution')
        except (KeyError, ValueError):
            break
        if ua.shape != ur.shape:
            break
        errs.append(float(np.linalg.norm(ua - ur) / max(np.linalg.norm(ur), 1e-300)))
        k += 1
    return errs


def reduce_run(run, ref=None):
    run = Path(run)
    row = json.loads((run / 'row.json').read_text()) if (run / 'row.json').exists() else {}
    attempts = jsonl(run / 'output/solver-attempts.jsonl')
    acc = [r for r in attempts if r.get('kind') == 'accepted']
    per_step = {}
    subsolves = {}
    for r in acc:
        per_step[r['step']] = per_step.get(r['step'], 0) + 1
        subsolves.setdefault(r['step'], set()).add(r.get('minimize_index'))
    man = run / 'output/run-manifest.json'
    steps = json.loads(man.read_text()).get('steps', []) if man.exists() else []
    phys = {r['step']: r for r in jsonl(run / 'output/physical-diagnostics.jsonl')}
    log = log_metrics((run / 'run.log').read_text(errors='replace')) if (run / 'run.log').exists() else {}
    preds = step_predictors(jsonl(run / 'output/trim-predictors.jsonl'))
    result = {
        'label': run.name, 'scene': row.get('scene'), 'trim_pin': row.get('trim_pin'),
        'overrides': row.get('overrides'), 'exit_code': row.get('exit_code'),
        'timed_out': row.get('timed_out'), 'wall_seconds': row.get('wall_seconds'),
        'binary_sha256': row.get('binary_sha256'), 'steps_completed': len([s for s in steps if s.get('outcome') == 'accepted']),
        'iterations': len(acc), 'iterations_per_step': [per_step[k] for k in sorted(per_step)],
        'subsolves_per_step': [len(subsolves[k]) for k in sorted(subsolves)],
        'log': log, 'steps': {},
    }
    for s in steps:
        k = s.get('step')
        sub = s.get('subsolves') or []
        p = phys.get(k, {})
        flag = p.get('physical_balance_pass') or {}
        result['steps'][k] = {
            'outcome': s.get('outcome'), 'wall_seconds': s.get('wall_seconds'),
            'termination_reason': (s.get('termination') or {}).get('termination_reason'),
            'stall_retunes': s.get('stall_retunes'),
            'al_passes': max([e.get('al_pass') or 0 for e in sub] + [0]),
            'subsolves': len(sub),
            'physical_balance_pass': flag.get('value'),
            'free_residual_ratio': flag.get('free_residual_ratio'),
            'predictors': preds.get(k),
        }
    for k, v in preds.items():  # a step cut by the timeout has records but no manifest entry
        if k not in result['steps']:
            result['steps'][k] = {'outcome': 'incomplete', 'predictors': v}
    if ref is not None and Path(ref).resolve() != run.resolve():
        result['reference'] = Path(ref).name
        result['relative_error_per_step'] = solution_error(run, Path(ref))
    return result


def line(r):
    last = r['steps'].get(max(r['steps'], default=None), {}) if r['steps'] else {}
    p = (last or {}).get('predictors') or {}
    log = r['log']
    err = ' err=' + ','.join(f'{e:.1e}' for e in r.get('relative_error_per_step', [])) if r.get('relative_error_per_step') else ''
    pin = f"{r['trim_pin']:.3g}" if r['trim_pin'] is not None else 'ctrl'
    frac = lambda v: f'{v:.2f}' if v is not None else '-'
    return (f"{r['label']:30s} pin={pin:>9s} exit={r['exit_code']} to={int(bool(r['timed_out']))} wall={r['wall_seconds']}"
            f" steps={r['steps_completed']} its={r['iterations']} per_step={r['iterations_per_step']}"
            f" restarts={log.get('restarts_by_trigger')} cap={frac(log.get('trial_cap_active_fraction'))}"
            f" ccd={frac(log.get('ccd_bound_fraction'))} trim_end={p.get('trim_last', float('nan')):.3g}"
            f" rms={p.get('gap', {}).get('rms') or float('nan'):.3f} pairs={p.get('active_pairs')}"
            f" balance={[s.get('physical_balance_pass') for s in r['steps'].values()]}{err}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ref')
    ap.add_argument('--json')
    ap.add_argument('runs', nargs='+')
    a = ap.parse_args()
    results = [reduce_run(r, a.ref) for r in a.runs if (Path(r) / 'row.json').exists()]
    for r in results:
        print(line(r))
    if a.json:
        Path(a.json).write_text(json.dumps(results, indent=1, default=str) + '\n')


if __name__ == '__main__':
    main()
