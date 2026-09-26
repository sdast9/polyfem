#!/usr/bin/env python3
"""Compare a completed EF-02/03 matrix without treating missing steps as agreement.

Run reduce.py first. Candidate screen runs may supply the non-R4 cases.
Reports measured repeat spreads; it does not invent an accuracy tolerance.
"""
import argparse
import itertools
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ef01'))
from ef01_reduce import solution_error


def compare_pair(root, a, b, count):
    errors = solution_error(root / 'runs' / a['label'], root / 'runs' / b['label'])
    flags_a = [a['steps'].get(str(k), {}).get('physical_balance_pass') for k in range(1, count + 1)]
    flags_b = [b['steps'].get(str(k), {}).get('physical_balance_pass') for k in range(1, count + 1)]
    missing = [k for k, (x, y) in enumerate(zip(flags_a, flags_b), 1) if x is None or y is None]
    changed = [k for k, (x, y) in enumerate(zip(flags_a, flags_b), 1) if x is not None and y is not None and x != y]
    return dict(a=a['label'], b=b['label'], expected_steps=count,
                compared_steps=len(errors), solution_complete=len(errors) == count and all(math.isfinite(e) for e in errors),
                relative_solution_error=errors, balance_missing_steps=missing,
                balance_changed_steps=changed, balance_unchanged=not missing and not changed)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('evidence', type=Path)
    p.add_argument('--candidate-prefix', default='')
    a = p.parse_args()
    rows = json.loads((a.evidence / 'ef02-metrics.json').read_text())
    expected = [("R4", 1), ("R4", 5), ("R1", 3), ("BBT", 20), ("IT", 200), ("BB", 1)]
    expected += [(s, 4) for s in ("smoke-qs", "smoke-tr", "smoke-adaptive", "smoke-alhess", "smoke-friction")]
    groups = {key: {"production": [], "candidate": []} for key in expected}
    for r in rows:
        label = r['label']
        if label.startswith('production-'):
            mode = 'production'
        elif label.startswith(a.candidate_prefix + 'candidate-') or label.startswith(a.candidate_prefix + 'screen-'):
            mode = 'candidate'
        else:
            continue
        config = json.loads((a.evidence / 'runs' / label / 'row.json').read_text())
        key = (r['scene'], config['steps_requested'])
        if key[0] == 'R3':  # explicitly outside acceptance
            continue
        groups.setdefault(key, {'production': [], 'candidate': []})[mode].append(r)
    output = []
    for (scene, count), group in sorted(groups.items()):
        prod, cand = group['production'], group['candidate']
        cross = [compare_pair(a.evidence, x, y, count) for x in cand for y in prod]
        repeats = {mode: [compare_pair(a.evidence, x, y, count) for x, y in itertools.combinations(group[mode], 2)]
                   for mode in group}
        output.append(dict(scene=scene, steps=count,
            production=[r['label'] for r in prod], candidate=[r['label'] for r in cand],
            expected_repeats=2 if scene == 'R4' else 1,
            evidence_complete=len(prod) >= (2 if scene == 'R4' else 1) and len(cand) >= (2 if scene == 'R4' else 1) and all(r['evidence_complete'] for r in prod + cand),
            production_iterations=[r['iterations'] for r in prod],
            candidate_iterations=[r['iterations'] for r in cand],
            all_candidate_iterations_le_all_production=bool(prod and cand) and max(r['iterations'] for r in cand) <= min(r['iterations'] for r in prod),
            cross=cross, repeats=repeats,
            production_repeat_envelope_available=bool(repeats['production']) and all(r['solution_complete'] for r in repeats['production'])))
    target = a.evidence / 'ef02-comparisons.json'
    target.write_text(json.dumps(output, indent=2) + '\n')
    for r in output:
        print(json.dumps({k: r[k] for k in ('scene', 'steps', 'evidence_complete', 'production_iterations', 'candidate_iterations', 'all_candidate_iterations_le_all_production', 'production_repeat_envelope_available')}))
    print(target)


if __name__ == '__main__':
    main()
