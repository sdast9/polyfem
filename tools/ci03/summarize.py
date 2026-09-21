"""CI-03: compact, publishable results from an evidence directory.

Usage:
  python3 tools/ci03/summarize.py --evidence /absolute/evidence/dir --out tools/ci03/results-YYYYMMDD.json

Reads every `summary-<label>.json` of ab_runner.py and `fixtures/make_fixtures.json`
(if present) and writes one JSON with, per run, the six harness metrics, the
relative error against the stored reference, the harness verdict, the lag
re-solve counts and the diagnostics summary -- without the per-file hashes and
paths of the full records, which stay in the evidence directory.
"""
import argparse
import json
from pathlib import Path

KEEP = ['label', 'slug', 'variant', 'fixture', 'exit', 'wall_seconds', 'harness_verdict', 'harness_violations',
        'copy_identical_to_pinned', 'overlay', 'computed', 'relative_error_vs_reference', 'max_relative_error_vs_reference',
        'lag', 'diagnostics', 'effective_friction_iterations', 'effective_barrier_stiffness', 'effective_friction_coefficient']


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = {'evidence': args.evidence.name, 'run_sets': {}, 'fixtures': None}
    for path in sorted(args.evidence.glob('summary-*.json')):
        summary = json.loads(path.read_text())
        result['run_sets'][summary['label']] = {
            'unit_tests_sha256': summary['unit_tests_sha256'],
            'runs': [{k: r[k] for k in KEEP if k in r} for r in summary['runs']],
        }
    made = args.evidence / 'fixtures' / 'make_fixtures.json'
    if made.is_file():
        result['fixtures'] = json.loads(made.read_text())
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print(f'{args.out}: {sum(len(s["runs"]) for s in result["run_sets"].values())} runs in {len(result["run_sets"])} run sets')


if __name__ == '__main__':
    main()
