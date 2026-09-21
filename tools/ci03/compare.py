"""CI-03: compare run sets of ab_runner.py metric by metric.

Usage:
  python3 tools/ci03/compare.py --output /absolute/evidence/dir before:default after:fi1 [after:default ...]

Prints, for every fixture, the six harness metrics of each `<label>:<variant>`
run and the largest relative difference of each run against the first one
(|a - b| / max(|b|, 1e-5), the harness's own normalisation). Reads
`summary-<label>.json`.
"""
import argparse
import json
from pathlib import Path

METRICS = ['err_l2', 'err_h1', 'err_h1_semi', 'err_linf', 'err_linf_grad', 'err_lp']


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('runs', nargs='+', help='label:variant, the first is the base')
    args = parser.parse_args()
    summaries = {}
    table = {}
    for spec in args.runs:
        label, variant = spec.split(':')
        if label not in summaries:
            summaries[label] = json.loads((args.output / f'summary-{label}.json').read_text())
        for r in summaries[label]['runs']:
            if r['variant'] == variant:
                table.setdefault(r['slug'], {})[spec] = r.get('computed')
    base = args.runs[0]
    result = {}
    for slug, runs in table.items():
        print(f'## {slug}')
        print('| run | ' + ' | '.join(METRICS) + ' | max rel diff vs ' + base + ' |')
        print('| --- |' + ' ---: |' * (len(METRICS) + 1))
        for spec, computed in runs.items():
            if computed is None:
                print(f'| {spec} | (no metrics) |')
                continue
            b = runs.get(base)
            diff = None
            if b:
                diff = max(abs(computed[k] - b[k]) / max(abs(b[k]), 1e-5) for k in METRICS)
            result.setdefault(slug, {})[spec] = {'computed': computed, 'max_rel_diff_vs_base': diff}
            cells = ' | '.join(f'{computed[k]:.17g}' for k in METRICS)
            print(f"| {spec} | {cells} | {'—' if diff is None else f'{diff:.3e}'} |")
        print()
    name = 'compare-' + '-vs-'.join(s.replace(':', '_') for s in args.runs) + '.json'
    (args.output / name).write_text(json.dumps({'base': base, 'runs': args.runs, 'fixtures': result}, indent=2) + '\n')


if __name__ == '__main__':
    main()
