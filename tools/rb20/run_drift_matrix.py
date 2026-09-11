"""RB-20: post-publication drift matrix.

Runs a public semi-implicit scene over a dt x trim-band matrix with optional
semi_implicit overrides (e.g. force_continuation, continuation_max_ratio) and
extracts, from output/coefficient-events.jsonl, every between-steps refresh
event's fixed-coordinate contact-force change: ||f_after - f_before|| at the
published endpoint, together with the before/after trim and active counts.
This is a fixed-coordinate force identity, not a physical balance.

Usage:
  python3 tools/rb20/run_drift_matrix.py --scene quasistatic-semi --output /abs/fresh \
      [--dt .25 .125 .0625] [--lower .1 .5 .8] [--steps N] \
      [--override force_continuation=false] [--override continuation_max_ratio=2]
"""
import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_override(text):
    key, _, value = text.partition('=')
    return key, json.loads(value)


def drift_events(output):
    path = output/'coefficient-events.jsonl'
    if not path.exists():
        return None
    rows = []
    for line in path.read_text().splitlines():
        e = json.loads(line)
        if e.get('phase') != 'between_steps_after_endpoint' or e.get('operation') != 'refresh':
            continue
        fb = np.asarray(e['before']['gradient_objective'])
        fa = np.asarray(e['after']['gradient_objective'])
        sb, sa = e['before']['evaluated_state'], e['after']['evaluated_state']
        rows.append(dict(step=e['step'], event_id=e['event_id'],
            active_before=sb['active_count'], active_after=sa['active_count'],
            trim_before=sb['trim_or_global_stiffness'], trim_after=sa['trim_or_global_stiffness'],
            continued=sa.get('continued_count'), fresh=sa.get('fresh_count'),
            force_norm_before=float(np.linalg.norm(fb)), force_norm_after=float(np.linalg.norm(fa)),
            force_change_norm=float(np.linalg.norm(fa-fb)),
            force_change_relative=float(np.linalg.norm(fa-fb)/np.linalg.norm(fb)) if np.linalg.norm(fb) > 0 else None,
            energy_before=e['before']['objective'], energy_after=e['after']['objective'],
            recorded_free_contact_force_change_norm=e.get('free_contact_force_change_norm')))
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--scene', required=True, choices=['quasistatic-semi', 'transient-semi', 'quasistatic-semi-friction'])
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--dt', nargs='+', type=float, default=[.25, .125, .0625])
    p.add_argument('--lower', nargs='+', type=float, default=[.1, .5, .8])
    p.add_argument('--upper', type=float, default=.9)
    p.add_argument('--steps', type=int, default=None, help='override time_steps (default: 1/dt)')
    p.add_argument('--override', action='append', default=[], help='semi_implicit key=json value')
    p.add_argument('--timeout', type=int, default=900)
    p.add_argument('--label', default='')
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    overrides = dict(parse_override(o) for o in args.override)
    binary = ROOT/'build/PolyFEM_bin'
    result = dict(binary_sha256=sha(binary), runner_sha256=sha(Path(__file__)),
                  source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                  scene=args.scene, label=args.label, overrides=overrides,
                  protocol=dict(dt=args.dt, lower=args.lower, upper=args.upper, steps=args.steps, timeout_seconds=args.timeout),
                  scope='Fixed-coordinate contact-force change at between-steps refresh events; numerical termination only, no physical acceptance criterion',
                  runs=[])
    save = lambda: (out/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    save()
    for lower in args.lower:
        for dt in args.dt:
            d = out/f'lower-{lower:g}-dt-{dt:g}'
            d.mkdir()
            cfg = json.loads((ROOT/'scenes/semi-implicit'/f'{args.scene}.json').read_text())
            cfg['time']['dt'] = dt
            if args.steps is not None:
                cfg['time'].pop('tend', None)
                cfg['time']['time_steps'] = args.steps
            semi = cfg['solver']['contact'].setdefault('semi_implicit', {})
            semi.update(trim_lower=lower, trim_upper=args.upper)
            semi.update(overrides)
            cfg['output'].update(directory=str(d/'output'), stats=True, physical_diagnostics=True)
            for asset in ('cube.mesh', 'slab.obj'):
                shutil.copy2(ROOT/'scenes/semi-implicit'/asset, d/asset)
            (d/'params.json').write_text(json.dumps(cfg, indent=2)+'\n')
            cmd = [str(binary), '--json', str(d/'params.json'), '--log_level', 'debug']
            row = dict(lower=lower, dt=dt, directory=d.name, command=cmd, input_sha256=sha(d/'params.json'), status='running')
            result['runs'].append(row)
            save()
            t0 = time.monotonic()
            with (d/'run.log').open('w') as log:
                try:
                    row['exit'] = subprocess.run(cmd, cwd=d, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout).returncode
                    row['status'] = 'exited'
                except subprocess.TimeoutExpired:
                    row.update(exit=None, status='timed_out')
            row['wall_seconds'] = time.monotonic()-t0
            text = (d/'run.log').read_text(errors='replace')
            row['error_lines'] = text.count('[error]')
            row['stall_restarts'] = text.count('retuning barrier stiffness and restarting')
            row['saved_steps'] = len(list((d/'output').glob('step_*.vtu'))) if (d/'output').exists() else 0
            row['newton_iterations'] = sum(int(m) for m in __import__('re').findall(r'Finished: [^(]*\(iters=(\d+)', text))
            ev = drift_events(d/'output')
            row['drift_events'] = ev
            if ev:
                unchanged = [e for e in ev if e['trim_before'] == e['trim_after'] and e['active_before'] == e['active_after']]
                row['summary'] = dict(events=len(ev), events_trim_and_count_unchanged=len(unchanged),
                    max_relative_change_trim_unchanged=max((e['force_change_relative'] or 0) for e in unchanged) if unchanged else None,
                    max_relative_change_all=max((e['force_change_relative'] or 0) for e in ev))
            save()
            print(json.dumps({k: row.get(k) for k in ('lower', 'dt', 'exit', 'saved_steps', 'stall_restarts', 'summary')}), flush=True)


if __name__ == '__main__':
    main()
