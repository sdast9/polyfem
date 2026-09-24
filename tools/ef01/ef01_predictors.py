#!/usr/bin/env python3
"""Every full EF-01 predictor record of a run, one line each.

usage: ef01_predictors.py RUN [--every N]

Columns: step, event, active pairs, trim in force, the two-sided
gradient-balance trim κ_gb with its cosine (the calibration applies it only
upward and only when cos >= 0.1), the trims that would make the median / the
summed barrier Hessian diagonal equal the elastic one on contact DOFs, the
unclamped conditioning-cap trim, the controller's rms gap, the force-weighted
mean gap and the incidence-weighted contact multiplicity.
"""
import argparse, json
from pathlib import Path


def g(d, *keys):
    for k in keys:
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d


def f(v, spec='.3g'):
    return '-' if v is None else format(v, spec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--every', type=int, default=1)
    a = ap.parse_args()
    rows = []
    for line in (Path(a.run) / 'output/trim-predictors.jsonl').read_text().splitlines():
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        if r.get('event') != 'iteration' and (r.get('active_count') or 0) > 0:
            rows.append(r)
    print(f"{'step':>4} {'event':>16} {'pairs':>6} {'trim':>9} {'k_gb':>9} {'cos':>6} {'t_med':>9} {'t_sum':>9} {'t_cap':>9} {'rms':>6} {'fw':>6} {'mult':>6}")
    for i, r in enumerate(rows):
        if i % a.every and i != len(rows) - 1:
            continue
        print(f"{r['step']:4} {r['event']:>16} {r['active_count']:6} {f(r['trim']):>9} {f(g(r, 'gradient_balance', 'kappa_gb')):>9} "
              f"{f(g(r, 'gradient_balance', 'cos_opposition'), '.2f'):>6} {f(g(r, 'hessian_diagonal_ratio', 'trim_for_unit_median')):>9} "
              f"{f(g(r, 'hessian_diagonal_ratio', 'trim_for_unit_sum')):>9} {f(r.get('conditioning_cap_trim')):>9} "
              f"{f(g(r, 'gap', 'rms'), '.3f'):>6} {f(g(r, 'force_weighted', 'mean_gap'), '.3f'):>6} "
              f"{f(g(r, 'multiplicity', 'incidence_weighted_mean'), '.1f'):>6}")


if __name__ == '__main__':
    main()
