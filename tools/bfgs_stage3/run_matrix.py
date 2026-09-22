"""Wolfe line-search comparison for stage 3 of the BFGS convergence audit.

Runs the five public semi-implicit smokes to answer two questions about the
opt-in `line_search/method: Wolfe`:

1. Is it inert where it is not selected? The Newton smokes at their own
   settings must reproduce a reference record's frames byte for byte (pass the
   stage 4 `matrix-results.json` as --reference), with the per-iteration
   diagnostics off and on, and every diagnostic-on stream must pass the RB-04
   attempt checker (schema version 3, with the extension build identity).
2. What does it do where it is selected? L-BFGS under RobustArmijo (the
   default) and under Wolfe at c2 = 0.9 / 0.5 / 0.1, from identical settings,
   in the configuration stage 4 showed converging (the stall controller's soft
   iteration budget removed -- a diagnostic setting, not a production one);
   then the audit's pair and dense BFGS at production settings.

Nothing here changes a production default.

    python3 run_matrix.py --build /abs/polyfem/build --output /abs/fresh/dir \
        --reference /abs/outputs/bfgs-stage4/<stamp>/matrix/matrix-results.json
"""

import argparse
import concurrent.futures
import importlib.util
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


stage4 = load('bfgs_stage4_run_matrix', HERE.parent / 'bfgs_stage4' / 'run_matrix.py')
rb04 = load('rb04_check_solver_attempts', HERE.parent / 'rb04' / 'check_solver_attempts.py')

SMOKES = stage4.SMOKES
PAIR = stage4.PAIR
ERROR_RE = re.compile(r'-- (L2|H1|Linf) error: ([0-9eE+.\-]+|nan|inf)')

UNINTERRUPTED = {'/solver/nonlinear/max_iterations': 20000}
NO_SOFT_BUDGET = {'/solver/contact/semi_implicit/restart/soft_iteration_limit': -1}


def wolfe(c2):
    return {'/solver/nonlinear/line_search/method': 'Wolfe',
            '/solver/nonlinear/line_search/Wolfe/c2': c2}


def matrix():
    """label, scene, method, diagnostics, overrides, note."""
    plan = []
    for scene in SMOKES:
        plan.append((f'{scene}-newton-off', scene, None, False, {},
                     'Newton control at the scene settings, diagnostics off; frames must match the reference'))
        plan.append((f'{scene}-newton-on', scene, None, True, {},
                     'Newton control, diagnostics on; frames must match the row above, stream must pass the RB-04 checker'))
    for scene in SMOKES:
        plan.append((f'{scene}-newton-wolfe', scene, None, True, wolfe(.9),
                     'Newton under Wolfe (c2 = 0.9) at production settings'))
    for scene in SMOKES:
        # quasistatic-adaptive carries no semi-implicit restart block; its
        # only limit is PolySolve's max_iterations (stage 4).
        base = dict(UNINTERRUPTED)
        if scene != 'quasistatic-adaptive':
            base.update(NO_SOFT_BUDGET)
        plan.append((f'{scene}-lbfgs-uninterrupted-robustarmijo', scene, 'L-BFGS', True, base,
                     'Diagnostic setting (soft budget removed): RobustArmijo reference'))
        for c2 in (.9, .5, .1):
            plan.append((f'{scene}-lbfgs-uninterrupted-wolfe-{c2}', scene, 'L-BFGS', True,
                         {**base, **wolfe(c2)},
                         f'Diagnostic setting (soft budget removed): Wolfe, c2 = {c2}'))
    # One factor: the approximate-Wolfe energy slack (Hager and Zhang's 1e-6
    # by default; 0 leaves only the Armijo roundoff bound).
    for epsilon in (0., 1e-9):
        base = {**UNINTERRUPTED, **NO_SOFT_BUDGET, **wolfe(.9),
                '/solver/nonlinear/line_search/Wolfe/approximate_wolfe_epsilon': epsilon}
        plan.append((f'quasistatic-semi-lbfgs-uninterrupted-wolfe-0.9-slack-{epsilon:g}', 'quasistatic-semi',
                     'L-BFGS', True, base,
                     f'Diagnostic setting (soft budget removed): Wolfe c2 = 0.9, approximate_wolfe_epsilon = {epsilon:g}'))
    for scene in PAIR:
        for c2 in (.9, .1):
            plan.append((f'{scene}-lbfgs-production-wolfe-{c2}', scene, 'L-BFGS', True, wolfe(c2),
                         f'Production settings under L-BFGS: Wolfe, c2 = {c2}'))
    # Attempt-stream regressions: stage 4 rows whose line searches fail and
    # are rejected, which the RB-04 checker could not pass before stage 3
    # repaired the observer (docs/bfgs-wolfe-line-search-20260922.md).
    for label, scene, overrides in (
            ('observer-quasistatic-semi-lbfgs-armijo', 'quasistatic-semi', {'/solver/nonlinear/line_search/method': 'Armijo'}),
            ('observer-quasistatic-semi-lbfgs-backtracking', 'quasistatic-semi', {'/solver/nonlinear/line_search/method': 'Backtracking'}),
            ('observer-quasistatic-adaptive-lbfgs', 'quasistatic-adaptive', {})):
        plan.append((label, scene, 'L-BFGS', True, overrides,
                     'Attempt-stream regression: rejected proposals must pass the RB-04 checker'))
    plan.append(('quasistatic-semi-bfgs-production-robustarmijo', 'quasistatic-semi', 'BFGS', True, {},
                 'Dense BFGS at production settings (stage 4 comparison row)'))
    for c2 in (.9, .1):
        plan.append((f'quasistatic-semi-bfgs-production-wolfe-{c2}', 'quasistatic-semi', 'BFGS', True, wolfe(c2),
                     f'Dense BFGS at production settings: Wolfe, c2 = {c2}'))
    return plan


def summarize_wolfe(path):
    """The Wolfe search's own record, per accepted iteration."""
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    accepted = [r for r in rows if r['kind'] == 'accepted' and isinstance(r.get('solver'), dict)]
    records = [stage4.dig(r['solver'], 'line_search', 'wolfe') for r in accepted]
    records = [w for w in records if isinstance(w, dict)]
    alpha = [stage4.dig(r['solver'], 'accepted', 'alpha') for r in accepted]
    outcomes = {}
    for w in records:
        outcomes[w['outcome']] = outcomes.get(w['outcome'], 0) + 1
    ratios = [w.get('accepted_slope_over_initial_slope') for w in records]
    trials = [r['trial'] for r in rows if isinstance(r.get('trial'), dict)]
    return {
        'searches': len(records),
        'outcomes': outcomes,
        'evaluations': sum(w['evaluations'] for w in records),
        'gradient_evaluations': sum(w['gradient_evaluations'] for w in records),
        'growth_sweeps': sum(w['growth_sweeps'] for w in records),
        'objective_restarts': sum(w['objective_restarts'] for w in records),
        'accepted_alpha_above_one': sum(1 for a in alpha if isinstance(a, (int, float)) and a > 1),
        'accepted_alpha': stage4.quantiles(alpha),
        'accepted_slope_over_initial_slope': stage4.quantiles(ratios),
        'growth_unavailable': {k: sum(1 for w in records if w.get('growth_unavailable') == k)
                               for k in {w.get('growth_unavailable') for w in records} - {None}},
        'attempt_stream_extensions': sum(t.get('extensions', 0) for t in trials),
        'attempt_stream_extensions_refused': sum(t.get('extensions_refused', 0) for t in trials),
    }


def run_one(binary, source, out, entry):
    label, scene, method, diagnostics, overrides, note = entry
    row = stage4.run(binary, source, out, label, scene, method, diagnostics, overrides, note)
    directory = out / 'runs' / label
    text = (directory / 'run.log').read_text(errors='replace')
    row['errors'] = {k: float(v) for k, v in ERROR_RE.findall(text)}
    attempts = directory / 'output' / 'solver-attempts.jsonl'
    if attempts.exists():
        row['wolfe'] = summarize_wolfe(attempts)
        try:
            checked = rb04.check_run(directory)
            row['attempt_check'] = {'passed': True, 'steps': len(checked['steps'])}
        except Exception as e:  # the checker's assertion is the finding
            row['attempt_check'] = {'passed': False, 'error': f'{type(e).__name__}: {e}'[:2000]}
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, default=None,
                        help='a matrix-results.json whose <scene>-newton-off frames the controls must reproduce')
    parser.add_argument('--only', default=None, help='substring filter on the run label')
    parser.add_argument('--jobs', type=int, default=1,
                        help='concurrent single-threaded runs; wall times are then contended')
    args = parser.parse_args()

    build = args.build.resolve()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    binary = build / 'PolyFEM_bin'
    source = build.parent / 'scenes/semi-implicit'

    plan = [e for e in matrix() if not args.only or args.only in e[0]]
    record = {'binary_sha256': stage4.sha(binary), 'scenes': str(source), 'jobs': args.jobs, 'runs': []}
    results = out / 'matrix-results.json'
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(run_one, binary, source, out, e): e[0] for e in plan}
        for future in concurrent.futures.as_completed(futures):
            row = future.result()
            record['runs'].append(row)
            record['runs'].sort(key=lambda r: [e[0] for e in plan].index(r['label']))
            results.write_text(json.dumps(record, indent=2) + '\n')
            print(json.dumps({k: row.get(k) for k in
                              ('label', 'exit_code', 'frames', 'wall_seconds', 'accepted_rows',
                               'restart_triggers', 'attempt_check')}
                             | {'wolfe_outcomes': (row.get('wolfe') or {}).get('outcomes')}), flush=True)

    by_label = {r['label']: r for r in record['runs']}
    reference = {}
    if args.reference:
        reference = {r['label']: r for r in json.loads(args.reference.read_text())['runs']}
    control = []
    for scene in SMOKES:
        off, on = by_label.get(f'{scene}-newton-off'), by_label.get(f'{scene}-newton-on')
        if not (off and on):
            continue
        ref = reference.get(f'{scene}-newton-off')
        control.append({
            'scene': scene,
            'exit_codes': [off['exit_code'], on['exit_code']],
            'frames': [off['frames'], on['frames']],
            'off_equals_on': off['frame_sha256'] == on['frame_sha256'] and on['frames'] > 0,
            'off_equals_reference': None if ref is None else off['frame_sha256'] == ref['frame_sha256'],
            'attempt_check': on.get('attempt_check')})
    record['controls'] = control
    results.write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(control, indent=1))
    ok = all(c['off_equals_on'] and c['off_equals_reference'] is not False
             and (c['attempt_check'] or {}).get('passed') for c in control)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
