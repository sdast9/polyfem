"""Contact convergence diagnostics for stage 4 of the BFGS convergence audit.

Runs the five public semi-implicit smokes under Newton and L-BFGS with
PolySolve's opt-in per-iteration diagnostics, then a bounded uninterrupted pass
and one-factor-at-a-time conditioning variants, and reduces every run to the
metrics the audit's stage 4 asks for: the final rescaled residual, the
termination criterion, the accepted displacement, alpha against the feasible
cap, the pair curvature and initial inverse-Hessian scale, objective versions,
skip/reset/fallback counts, and whether a restart came from alpha or from the
soft iteration budget.

Nothing here changes a production default. The diagnostics are observational
(one extra gradient evaluation per accepted iteration); the Newton pair of runs
with the option off and on exists to prove exactly that, by frame hash.

    python3 run_matrix.py --build /abs/polyfem/build --output /abs/fresh/dir
"""

import argparse
import hashlib
import json
import math
import re
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path

ASSETS = ('cube.mesh', 'slab.obj')
SMOKES = ('quasistatic-adaptive', 'quasistatic-semi', 'quasistatic-semi-alhess',
          'quasistatic-semi-friction', 'transient-semi')
# The audit's two scenes; the other three are the wider public set.
PAIR = ('quasistatic-semi', 'transient-semi')
TIMEOUT = 1800

STALL_RE = re.compile(r'Line-search stall detected \(trigger: ([^)]*)\)')
PERSISTED_RE = re.compile(r'Line-search stall persisted after (\d+) restart\(s\) \(([^)]*)\)')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quantiles(values):
    """min / median / max of the finite values, and how many there were."""
    finite = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    if not finite:
        return {'count': 0, 'min': None, 'median': None, 'max': None}
    return {'count': len(finite), 'min': min(finite), 'median': statistics.median(finite),
            'max': max(finite)}


def dig(row, *keys):
    """Nested lookup that tolerates a missing or null level."""
    for key in keys:
        if not isinstance(row, dict) or row.get(key) is None:
            return None
        row = row[key]
    return row


def scene_config(source, scene, method, diagnostics, overrides):
    config = json.loads((source / f'{scene}.json').read_text())
    if method is not None:
        config['solver']['nonlinear'] = {'solver': method}
        if method == 'BFGS':
            config['solver']['linear']['solver'] = 'Eigen::LDLT'
    if diagnostics:
        nonlinear = config['solver'].setdefault('nonlinear', {})
        nonlinear.setdefault('advanced', {})['iteration_diagnostics'] = True
        config['output']['physical_diagnostics'] = True
    for pointer, value in overrides.items():
        node = config
        parts = pointer.strip('/').split('/')
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = value
    return config


def run(binary, source, out, label, scene, method, diagnostics, overrides, note):
    directory = out / 'runs' / label
    directory.mkdir(parents=True)
    config = scene_config(source, scene, method, diagnostics, overrides)
    config['output']['directory'] = str(directory / 'output')
    for asset in ASSETS:
        if (source / asset).exists():
            shutil.copy2(source / asset, directory / asset)
    (directory / 'input.json').write_text(json.dumps(config, indent=2) + '\n')
    command = [str(binary), '--json', str(directory / 'input.json'),
               '--max_threads', '1', '--log_level', 'debug']
    row = {'label': label, 'scene': scene, 'method': method or 'scene default (Newton)',
           'iteration_diagnostics': diagnostics, 'overrides': overrides, 'note': note,
           'binary_sha256': sha(binary), 'input_sha256': sha(directory / 'input.json'),
           'command': command}
    start = time.monotonic()
    with (directory / 'run.log').open('w') as log:
        try:
            row['exit_code'] = subprocess.run(command, cwd=directory, stdout=log,
                                              stderr=subprocess.STDOUT,
                                              timeout=TIMEOUT).returncode
        except subprocess.TimeoutExpired:
            row['exit_code'] = None
            row['timeout_seconds'] = TIMEOUT
    row['wall_seconds'] = round(time.monotonic() - start, 2)
    row.update(summarize(directory))
    return row


def summarize(directory):
    """Reduce one run's log and diagnostic streams to the stage 4 metrics."""
    text = (directory / 'run.log').read_text(errors='replace')
    output = directory / 'output'
    frames = sorted(output.glob('step_*.vtu')) if output.exists() else []
    summary = {
        'error_lines': text.count('[error]'),
        'frames': len(frames),
        'frame_sha256': {p.name: sha(p) for p in frames},
        'restarts': len(STALL_RE.findall(text)),
        # The trigger the restart actually had, which the message named only
        # after this stage. "alpha" is the configured small-step patience;
        # "soft_iteration_limit" is the iteration budget, which the old
        # message reported as an alpha collapse.
        'restart_triggers': {},
        'stall_persisted': [{'restarts': int(m.group(1)), 'trigger': m.group(2)}
                            for m in PERSISTED_RE.finditer(text)],
        'objective_changes_logged': text.count('the objective changed'),
    }
    for match in STALL_RE.findall(text):
        kind = ('soft_iteration_limit' if match.startswith('soft iteration budget')
                else 'alpha_and_soft_iteration_limit' if 'and the soft iteration budget' in match
                else 'line_search_failed_on_all_strategies' if match.startswith('line search failed')
                else 'alpha')
        summary['restart_triggers'][kind] = summary['restart_triggers'].get(kind, 0) + 1

    attempts = output / 'solver-attempts.jsonl'
    if attempts.exists():
        summary.update(summarize_attempts(attempts))
    endpoints = output / 'physical-diagnostics.jsonl'
    if endpoints.exists():
        summary['endpoints'] = summarize_endpoints(endpoints)
    return summary


def summarize_attempts(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    accepted = [r for r in rows if r['kind'] == 'accepted' and isinstance(r.get('solver'), dict)]
    if not accepted:
        return {'accepted_rows': len([r for r in rows if r['kind'] == 'accepted']),
                'accepted_rows_with_diagnostics': 0}

    def field(*keys):
        return [dig(r['solver'], *keys) for r in accepted]

    sources, strategies, transitions = {}, {}, {}
    for r in accepted:
        source = dig(r['solver'], 'strategy_state', 'direction_source') or 'unreported'
        sources[source] = sources.get(source, 0) + 1
        strategy = r['solver'].get('strategy', 'unreported')
        strategies[strategy] = strategies.get(strategy, 0) + 1
        for event in r['solver'].get('strategy_transitions_since_previous_accept') or []:
            key = f"{event.get('from')}->{event.get('to')}:{event.get('reason')}"
            transitions[key] = transitions.get(key, 0) + 1

    alpha = field('accepted', 'alpha')
    over_feasible = field('line_search', 'accepted_over_feasible')
    ls_iterations = field('line_search', 'iterations')
    ratio = field('direction', 'norm_over_gradient_norm')
    slope_ratio = field('accepted', 'endpoint', 'slope_over_initial_slope')
    # Strong Wolfe curvature at the textbook c2 = 0.9, measured (not imposed):
    # |g(x+ap).p| <= c2 |g(x).p| with the accepted alpha.
    wolfe = [abs(v) <= 0.9 for v in slope_ratio
             if isinstance(v, (int, float)) and math.isfinite(v)]
    generations = {r['solver'].get('objective_generation') for r in accepted}

    return {
        'accepted_rows': len([r for r in rows if r['kind'] == 'accepted']),
        'accepted_rows_with_diagnostics': len(accepted),
        'direction_sources': sources,
        'accepted_by_strategy': strategies,
        'strategy_transitions': transitions,
        'alpha': quantiles(alpha),
        'alpha_equal_one': sum(1 for v in alpha if v == 1.0),
        'accepted_over_feasible': quantiles(over_feasible),
        'accepted_below_feasible_cap': sum(
            1 for v in over_feasible
            if isinstance(v, (int, float)) and math.isfinite(v) and v < 1 - 1e-12),
        'line_search_iterations': quantiles(ls_iterations),
        'line_search_no_backtracking': sum(1 for v in ls_iterations if v == 0),
        'direction_norm_over_gradient_norm': quantiles(ratio),
        'direction_norm': quantiles(field('direction', 'euclidean_norm')),
        'gradient_norm': quantiles(field('direction', 'gradient_euclidean_norm')),
        'gradient_problem_norm': quantiles(field('direction', 'gradient_problem_norm')),
        'accepted_displacement_norm': quantiles(field('accepted', 'euclidean_norm')),
        'accepted_displacement_problem_norm': quantiles(field('accepted', 'problem_norm')),
        'endpoint_slope_over_initial_slope': quantiles(slope_ratio),
        'strong_wolfe_c2_0p9_satisfied': sum(wolfe),
        'strong_wolfe_measured_on': len(wolfe),
        'inverse_hessian_initial_scale': quantiles(field('strategy_state', 'inverse_hessian_initial_scale')),
        'hessian_initial_scale': quantiles(field('strategy_state', 'hessian_initial_scale')),
        'history_corrections': quantiles(field('strategy_state', 'history_corrections')),
        'pair_relative_curvature': quantiles(field('strategy_state', 'secant_pair', 'relative_curvature')),
        'pair_s_dot_y': quantiles(field('strategy_state', 'secant_pair', 's_dot_y')),
        'pair_s_norm': quantiles(field('strategy_state', 'secant_pair', 's_norm')),
        'pair_y_norm': quantiles(field('strategy_state', 'secant_pair', 'y_norm')),
        'pair_verdicts': {v: sum(1 for r in accepted
                                 if dig(r['solver'], 'strategy_state', 'secant_pair', 'verdict') == v)
                          for v in {dig(r['solver'], 'strategy_state', 'secant_pair', 'verdict')
                                    for r in accepted} - {None}},
        'objective_generations_seen': sorted(g for g in generations if g is not None),
        'endpoint_gradient_unavailable': sum(
            1 for r in accepted if dig(r['solver'], 'accepted', 'endpoint', 'unavailable_reason')),
    }


def summarize_endpoints(path):
    """The per-step endpoint records: outcome, residual and the solver info of
    a failure, including the stall trigger this stage started recording."""
    out = []
    for line in path.read_text().splitlines():
        record = json.loads(line)
        entry = {'step': record.get('step'), 'outcome': record.get('outcome'),
                 'free_residual_norm': record.get('free_residual_norm'),
                 'solve_wall_seconds': record.get('solve_wall_seconds')}
        exception = dig(record, 'termination', 'exception')
        if isinstance(exception, str):
            entry['exception_head'] = exception.split('{', 1)[0].strip()
            start = exception.find('{')
            if start >= 0:
                try:
                    info = json.loads(exception[start:])
                except json.JSONDecodeError:
                    info = {}
                entry['solver'] = {
                    k: info.get(k) for k in
                    ('outcome', 'termination_reason', 'status', 'iterations', 'restarts',
                     'unchanged_restarts', 'stall_trigger', 'stall_iteration', 'stall_alpha',
                     'gradNorm', 'fDelta', 'xDelta', 'directional_derivative', 'energy',
                     'objective_changes', 'active_strategy', 'direction_sources',
                     'strategy_transition_counts', 'curvature_guard')}
                entry['final_iteration_diagnostics'] = info.get('iteration_diagnostics')
        out.append(entry)
    return out


def matrix():
    """label, scene, method, diagnostics, overrides, note."""
    plan = []
    # The control that makes every other row admissible: the same Newton solve
    # with the diagnostics off and on must produce identical frames.
    for scene in SMOKES:
        plan.append((f'{scene}-newton-off', scene, None, False, {},
                     'Newton control, diagnostics off (stage 2 baseline repeat)'))
        plan.append((f'{scene}-newton-on', scene, None, True, {},
                     'Newton control, diagnostics on; frames must match the row above'))
    # The audit's two scenes and the wider public set under L-BFGS.
    for scene in SMOKES:
        plan.append((f'{scene}-lbfgs', scene, 'L-BFGS', True, {},
                     'Public smoke under L-BFGS at production settings'))
    # Does the plateau survive when nothing interrupts it? Same trajectory up
    # to the first stall, so the comparison starts from the same state.
    plan.append(('quasistatic-semi-lbfgs-uninterrupted', 'quasistatic-semi', 'L-BFGS', True,
                 {'/solver/contact/semi_implicit/restart/enabled': False,
                  '/solver/nonlinear/max_iterations': 2000},
                 'Diagnostic only: one bounded uninterrupted pass, restart policy off'))
    plan.append(('quasistatic-semi-lbfgs-no-soft-budget', 'quasistatic-semi', 'L-BFGS', True,
                 {'/solver/contact/semi_implicit/restart/soft_iteration_limit': -1,
                  '/solver/nonlinear/max_iterations': 2000},
                 'Diagnostic only: alpha stalls still restart, iteration budget removed'))
    # One factor at a time, at unchanged tolerances.
    for size in (3, 12, 24):
        plan.append((f'quasistatic-semi-lbfgs-history-{size}', 'quasistatic-semi', 'L-BFGS', True,
                     {'/solver/nonlinear/L-BFGS/history_size': size},
                     f'Conditioning factor: L-BFGS history size {size} (default 6)'))
    for method in ('Armijo', 'Backtracking'):
        plan.append((f'quasistatic-semi-lbfgs-{method.lower()}', 'quasistatic-semi', 'L-BFGS', True,
                     {'/solver/nonlinear/line_search/method': method},
                     f'Line-search factor: {method} (default RobustArmijo)'))
    plan.append(('quasistatic-semi-bfgs', 'quasistatic-semi', 'BFGS', True, {},
                 'Dense BFGS for comparison; needs the dense linear solver'))
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--only', default=None, help='substring filter on the run label')
    args = parser.parse_args()

    build = args.build.resolve()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    binary = build / 'PolyFEM_bin'
    source = build.parent / 'scenes/semi-implicit'

    record = {'binary_sha256': sha(binary), 'scenes': str(source), 'runs': []}
    results = out / 'matrix-results.json'
    for label, scene, method, diagnostics, overrides, note in matrix():
        if args.only and args.only not in label:
            continue
        row = run(binary, source, out, label, scene, method, diagnostics, overrides, note)
        record['runs'].append(row)
        results.write_text(json.dumps(record, indent=2) + '\n')
        print(json.dumps({k: v for k, v in row.items()
                          if k in ('label', 'exit_code', 'frames', 'wall_seconds', 'restarts',
                                   'restart_triggers', 'accepted_rows_with_diagnostics')}),
              flush=True)

    # The observational check: diagnostics on and off must agree frame for frame.
    by_label = {r['label']: r for r in record['runs']}
    control = []
    for scene in SMOKES:
        off, on = by_label.get(f'{scene}-newton-off'), by_label.get(f'{scene}-newton-on')
        if off and on:
            control.append({'scene': scene, 'exit_codes': [off['exit_code'], on['exit_code']],
                            'frames': [off['frames'], on['frames']],
                            'identical': off['frame_sha256'] == on['frame_sha256'] and on['frames'] > 0})
    record['diagnostics_are_observational'] = control
    results.write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(control, indent=1))
    return 0 if all(c['identical'] for c in control) else 1


if __name__ == '__main__':
    sys.exit(main())
