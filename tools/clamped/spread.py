#!/usr/bin/env python3
"""Repeat-aware mode comparison: per scene, iterations of every run per mode,
keep's own run-to-run solution spread and each mode's difference to keep.

usage: spread.py EVIDENCE [--json OUT]
Relative L2 solution differences per step (ef01_reduce.solution_error); the
spread is the largest over step and pair. Missing steps are reported.
"""
import argparse, itertools, json, re, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ef01'))
import ef01_reduce as red  # noqa: E402

MODES = ('keep', 'exclude_statistics', 'exclude_collisions')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('evidence', type=Path)
    ap.add_argument('--json')
    a = ap.parse_args()
    groups = {}
    for run in sorted((a.evidence / 'runs').iterdir()):
        m = re.match(r'(keep|exclude_statistics|exclude_collisions)-(\w+)-(.+)-s(\d+)-r(\d+)$', run.name)
        if m:
            groups.setdefault((m[3], int(m[4])), {}).setdefault(m[1], []).append(run)
    out = []
    for (scene, steps), by_mode in sorted(groups.items()):
        e = dict(scene=scene, steps=steps, modes={})
        keep = by_mode.get('keep', [])
        for mode in MODES:
            runs = by_mode.get(mode, [])
            if not runs:
                continue
            rows = [red.reduce_run(r) for r in runs]
            s = dict(runs=[r.name for r in runs], exit=[x['exit_code'] for x in rows],
                     steps_completed=[x['steps_completed'] for x in rows], iterations=[x['iterations'] for x in rows],
                     stall_retunes=[sum(v.get('stall_retunes') or 0 for v in x['steps'].values()) for x in rows],
                     balance_true=[sum(1 for v in x['steps'].values() if v.get('physical_balance_pass')) for x in rows])
            pairs = itertools.combinations(keep, 2) if mode == 'keep' else itertools.product(runs, keep)
            diffs = []
            for x, y in pairs:
                d = red.solution_error(x, y)
                diffs.append(dict(a=x.name, b=y.name, compared_steps=len(d), max=max(d) if d else None, last=d[-1] if d else None))
            s['solution_difference'] = diffs
            e['modes'][mode] = s
        out.append(e)
    text = json.dumps(out, indent=1)
    if a.json:
        Path(a.json).write_text(text + '\n')
    for e in out:
        print(f"{e['scene']} s{e['steps']}")
        for mode, s in e['modes'].items():
            mx = [d['max'] for d in s['solution_difference'] if d['max'] is not None]
            print(f"   {mode:20s} its={s['iterations']} stalls={s['stall_retunes']} exit={s['exit']} balance_true={s['balance_true']} "
                  f"soldiff({'keep spread' if mode == 'keep' else 'to keep'}) max={max(mx) if mx else None:.3g}" if mx else
                  f"   {mode:20s} its={s['iterations']} stalls={s['stall_retunes']} exit={s['exit']} balance_true={s['balance_true']}")


if __name__ == '__main__':
    main()
