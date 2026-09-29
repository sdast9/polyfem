#!/usr/bin/env python3
"""Compare the clamped-contact modes run by sequence.py / bb_resume.py.

usage: compare.py EVIDENCE [--json OUT]

For every scene/steps/repeat: exit, accepted Newton iterations per step, stall
retunes, AL passes, physical_balance_pass flags, byte identity of the exported
VTU files and the relative L2 solution difference per step, each exclude mode
against keep. Ball-burst resumes (bb31-*) are compared on their log's Newton
iteration and restart counts (they export no VTU). Missing evidence is listed,
never counted as agreement.
"""
import argparse, hashlib, json, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ef01'))
import ef01_reduce as red  # noqa: E402

MODES = ('keep', 'exclude_statistics', 'exclude_collisions')


def vtu_hashes(run):
    return {f.name: hashlib.file_digest(f.open('rb'), 'sha256').hexdigest() for f in sorted((run / 'output').glob('*.vtu'))}


def bb_summary(run):
    row = json.loads((run / 'row.json').read_text()) if (run / 'row.json').exists() else {}
    preds = list(red.jsonl(run / 'output/trim-predictors.jsonl')) if (run / 'output/trim-predictors.jsonl').exists() else []
    per_step = {}
    for r in preds:
        if r.get('event') == 'iteration':
            per_step[r.get('step')] = per_step.get(r.get('step'), 0) + 1
    trims = {}
    for r in preds:
        trims.setdefault(r.get('step'), []).append(r.get('trim'))
    moves = {k: sum(1 for a, b in zip(v, v[1:]) if a != b) for k, v in trims.items()}
    return dict(label=run.name, exit_code=row.get('exit_code'), wall_seconds=row.get('wall_seconds'),
                accepted_iterations_per_step=[per_step[k] for k in sorted(per_step)],
                stall_retunes_per_step=[sum(1 for r in preds if r.get('step') == k and r.get('event') == 'stall_retune') for k in sorted(per_step)],
                trim_moves_per_step=[moves[k] for k in sorted(moves)],
                final_trim={k: v[-1] for k, v in trims.items()})


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('evidence', type=Path)
    ap.add_argument('--json')
    a = ap.parse_args()
    runs = a.evidence / 'runs'
    groups = {}
    for run in sorted(runs.iterdir()):
        name = run.name
        if name.startswith('bb31-'):
            mode, _, repeat = name[len('bb31-'):].partition('-rms')
            groups.setdefault(('ball-burst step 31', 'resume' + (repeat or '-r1')), {})[mode] = run
            continue
        for m in MODES:
            if name.startswith(m + '-'):
                groups.setdefault(tuple(name[len(m) + 1:].split('-', 1)), {})[m] = run
    out = []
    for key, by_mode in sorted(groups.items()):
        entry = dict(case='-'.join(key), modes={})
        base = by_mode.get('keep')
        for m, run in by_mode.items():
            if key[0].startswith('ball-burst'):
                entry['modes'][m] = bb_summary(run)
                continue
            r = red.reduce_run(run)
            s = dict(exit_code=r['exit_code'], timed_out=r['timed_out'], wall_seconds=r['wall_seconds'],
                     iterations=r['iterations'], iterations_per_step=r['iterations_per_step'],
                     stall_retunes=[v.get('stall_retunes') for v in r['steps'].values()],
                     al_passes=[v.get('al_passes') for v in r['steps'].values()],
                     balance=[v.get('physical_balance_pass') for v in r['steps'].values()])
            if base is not None and m != 'keep':
                hb, hm = vtu_hashes(base), vtu_hashes(run)
                s['vtu_identical_to_keep'] = bool(hb) and hb == hm
                s['relative_solution_difference_to_keep'] = red.solution_error(run, base)
            entry['modes'][m] = s
        out.append(entry)
    text = json.dumps(out, indent=1)
    if a.json:
        Path(a.json).write_text(text + '\n')
    for e in out:
        print(e['case'])
        for m, s in e['modes'].items():
            short = {k: v for k, v in s.items() if k != 'relative_solution_difference_to_keep'}
            d = s.get('relative_solution_difference_to_keep')
            if d:
                short['max_rel_diff'] = max(d)
            print('  ', m, json.dumps(short))


if __name__ == '__main__':
    main()
