"""RB-07: per-pass AL history from a PolyFEM debug log, and candidate stagnation rules evaluated on it.

python3 tools/rb07/analyze_passes.py run.log [run2.log ...] [--ceiling 1e8] [--json out.json]

For every log: the pass table (weight, subsolve termination and iterations,
stall events, last minimum contact distance, BC residual e = sqrt(sum of squared
constrained-DOF residuals) in length units, PF-07 relative progress eta) and,
for windows W in {3, 5, 10, 20, 40}, the first pass at which each candidate
rule would have ended the stage, so that a rule can be judged against a run
that is known to succeed (a false positive) and one known to be stuck:

  window-relative(tau)  the last W passes ran at the weight ceiling and
                        e_k > (1 - tau) * e_{k-W}: no relative BC progress
                        over the window
  running-min(W)        the last W passes ran at the ceiling and none of them
                        improved the best residual seen before the window
                        (an early-stopping patience rule)

These rules use the BC residual only; the plan warns against a
distance-to-feasibility metric built from it alone, which is exactly what this
script measures. Standard-library Python only.
"""
import argparse
import json
import math
from pathlib import Path
import re

PASS_RE = re.compile(r'Solving AL Problem with weight ([0-9.eE+-]+)')
FINISHED_RE = re.compile(r'Finished: (.*?) took .*?\(iters=(\d+)')
ERROR_RE = re.compile(r'Current error = ([0-9.eE+-]+|nan|inf)')
ETA_RE = re.compile(r'Current eta = ([0-9.eE+-]+|nan|-inf|inf)')
INITIAL_RE = re.compile(r'Initial error = ([0-9.eE+-]+)')
MIND_RE = re.compile(r'Minimum distance during solve: ([0-9.eE+-]+)')
STALL_RE = re.compile(r'Line-search stall detected|Hard stall persists|stall persisted|stall persists')
SNAP_RE = re.compile(r'Successfully applied constraints conditions')
STOP_RE = re.compile(r'PolyFEM stopped: (.*)')


def parse(log_path):
    passes, current, initial, snapped, stopped = [], None, None, False, None
    for line in Path(log_path).read_text(errors='replace').splitlines():
        m = INITIAL_RE.search(line)
        if m and initial is None:
            initial = float(m.group(1))
        m = STOP_RE.search(line)
        if m:
            stopped = m.group(1)
        if SNAP_RE.search(line) and passes:
            snapped = True
        m = PASS_RE.search(line)
        if m:
            current = dict(weight=float(m.group(1)), subsolves=[], stall_events=0, min_distance=None)
            passes.append(current)
            continue
        if current is None:
            continue
        if STALL_RE.search(line):
            current['stall_events'] += 1
        m = MIND_RE.search(line)
        if m:
            current['min_distance'] = float(m.group(1))
        m = FINISHED_RE.search(line)
        if m:
            current['subsolves'].append(dict(reason=m.group(1), iterations=int(m.group(2))))
        m = ERROR_RE.search(line)
        if m:
            current['bc_error_squared'] = float(m.group(1))
            current['bc_residual'] = math.sqrt(float(m.group(1))) if float(m.group(1)) >= 0 else float('nan')
        m = ETA_RE.search(line)
        if m:
            current['eta'] = float(m.group(1))
            current = None
    return dict(log=str(log_path), initial_error=initial, initial_residual=math.sqrt(initial) if initial else None,
                passes=passes, snapped_after_passes=snapped, stopped=stopped)


def evaluate(history, ceiling, windows=(3, 5, 10, 20, 40), tau=1e-2):
    es = [p.get('bc_residual', float('nan')) for p in history['passes']]
    at_ceiling = [p['weight'] >= ceiling for p in history['passes']]
    rules = {}
    for W in windows:
        rel = None
        for k in range(len(es)):
            if k + 1 <= W or not all(at_ceiling[k - W + 1:k + 1]):
                continue
            ref, now = es[k - W], es[k]
            if math.isnan(now) or (ref > 0 and now > (1 - tau) * ref) or (ref == 0 and now >= 0):
                rel = k + 1
                break
        patience = None
        for k in range(len(es)):
            if k + 1 <= W or not all(at_ceiling[k - W + 1:k + 1]):
                continue
            best = min(es[:k - W + 1])
            if min(es[k - W + 1:k + 1]) >= best * (1 - 1e-3):
                patience = k + 1
                break
        rules[f'W={W}'] = dict(window_relative_first_trigger=rel, running_min_first_trigger=patience)
    return rules


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('logs', nargs='+', type=Path)
    p.add_argument('--ceiling', type=float, default=1e8)
    p.add_argument('--tau', type=float, default=1e-2)
    p.add_argument('--json', type=Path)
    p.add_argument('--every', type=int, default=1, help='print every n-th pass (the first and last five always)')
    args = p.parse_args()
    out = []
    for log in args.logs:
        h = parse(log)
        h['candidate_rules'] = evaluate(h, args.ceiling, tau=args.tau)
        out.append(h)
        n = len(h['passes'])
        first_ceiling = next((i + 1 for i, q in enumerate(h['passes']) if q['weight'] >= args.ceiling), None)
        print(f"{log}: {n} passes, initial residual {h['initial_residual']}, first pass at the ceiling {first_ceiling}, snapped {h['snapped_after_passes']}, stopped: {h['stopped']}")
        for k, q in enumerate(h['passes']):
            if not (k < 5 or k >= n - 5 or k % args.every == 0):
                continue
            sub = q['subsolves'][-1] if q['subsolves'] else {}
            print(f"  pass {k + 1:3d}: weight {q['weight']:8.3g} subsolve '{sub.get('reason', '?')[:34]}' iters {sub.get('iterations', '?'):>3} stalls {q['stall_events']:2d} "
                  f"min_distance {q['min_distance'] if q['min_distance'] is not None else float('nan'):.3e} e {q.get('bc_residual', float('nan')):.5f} eta {q.get('eta', float('nan')):.4f}")
        for name, r in h['candidate_rules'].items():
            print(f"  {name}: window-relative({args.tau:g}) first trigger pass {r['window_relative_first_trigger']}, running-min first trigger pass {r['running_min_first_trigger']}")
    if args.json:
        args.json.write_text(json.dumps(out, indent=2) + '\n')


if __name__ == '__main__':
    main()
