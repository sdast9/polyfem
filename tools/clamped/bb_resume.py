#!/usr/bin/env python3
"""Resume the EF-07 ball-burst step-31 state with a clamped-contact mode.

usage: bb_resume.py --template-dir D --state S --out E --binary B LABEL
                    [--clamped keep|exclude_statistics|exclude_collisions]
                    [--controller rms|force_weighted] [--steps 2] [--threads 0]

D holds EF-07's params-s31-rms.json / restart-s31-rms.json and the meshes
(ef07-work/scene); S is R0's state_30 (uncompressed). The run gets its own
params/restart copies in E/runs/LABEL with the output there, trim predictors
on, physical diagnostics off (their coefficient events exceed 10 GB per step).
"""
import argparse, json, os, subprocess, time, hashlib
from pathlib import Path


def sha(p):
    return hashlib.file_digest(open(p, 'rb'), 'sha256').hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('label')
    ap.add_argument('--template-dir', type=Path, required=True)
    ap.add_argument('--state', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--clamped', default='keep')
    ap.add_argument('--controller', default='rms')
    ap.add_argument('--steps', type=int, default=2)
    ap.add_argument('--threads', type=int, default=0)
    a = ap.parse_args()
    run = (a.out / 'runs' / a.label).resolve()
    if run.exists():
        raise SystemExit(f'{run} exists; choose a fresh label')
    (run / 'output').mkdir(parents=True)
    params = json.loads((a.template_dir / 'params-s31-rms.json').read_text())
    restart = json.loads((a.template_dir / 'restart-s31-rms.json').read_text())
    for f in a.template_dir.iterdir():
        if f.suffix in ('.msh', '.txt'):
            (run / f.name).symlink_to(f.resolve())
    si = params['solver']['contact']['semi_implicit']
    si['band_statistic'] = a.controller
    si['initial_trim_estimate'] = a.controller == 'force_weighted'
    if a.clamped != 'keep':
        si['clamped_contacts'] = a.clamped
    out = params['output']
    out['directory'] = str(run / 'output')
    out['paraview']['file_name'] = str(run / 'output/sim.pvd')
    out['log']['path'] = str(run / 'output/log.txt')
    out['trim_predictors'] = True
    out['physical_diagnostics'] = False
    (run / 'params.json').write_text(json.dumps(params, indent=1) + '\n')
    restart['common'] = str(run / 'params.json')
    restart['root_path'] = str(run / 'params.json')
    restart['input']['data']['state'] = str(a.state.resolve())
    restart['time']['time_steps'] = a.steps
    (run / 'restart.json').write_text(json.dumps(restart, indent=1) + '\n')
    cmd = ['/usr/bin/time', '-l', 'caffeinate', '-i', '-s', str(a.binary.resolve()), '-j', str(run / 'restart.json'),
           '--max_threads', str(a.threads), '--log_level', '0']
    row = dict(label=a.label, clamped=a.clamped, controller=a.controller, steps=a.steps, binary=str(a.binary),
               binary_sha256=sha(a.binary), state_sha256=sha(a.state), command=cmd,
               started=time.strftime('%Y-%m-%dT%H:%M:%S'))
    t = time.monotonic()
    with open(run / 'stdout.txt', 'w') as so, open(run / 'stderr.txt', 'w') as se:
        row['exit_code'] = subprocess.run(cmd, cwd=run, stdout=so, stderr=se).returncode
    row['wall_seconds'] = round(time.monotonic() - t, 1)
    (run / 'row.json').write_text(json.dumps(row, indent=1) + '\n')
    print(json.dumps({k: row[k] for k in ('label', 'exit_code', 'wall_seconds')}), flush=True)


if __name__ == '__main__':
    main()
