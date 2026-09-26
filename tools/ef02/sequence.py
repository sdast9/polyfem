#!/usr/bin/env python3
"""Sequential EF-02/03 matrix. Run after builds; binaries must be immutable copies.

Usage: sequence.py --workspace W --out E --binary B --mode pilot|production|candidate
Records each command BEFORE execution. Refuses existing runs; no silent cache reuse.
"""
import argparse, datetime, hashlib, json, subprocess, sys
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--workspace', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--mode', choices=['pilot', 'screen', 'production', 'candidate'], required=True)
    ap.add_argument('--prefix', default='')
    ap.add_argument('--only', action='append', help='Run only these scene names; preserve completed evidence separately')
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    if a.mode == 'pilot':
        jobs = [('R1', 3, 1), ('BBT', 20, 1), ('R4', 1, 1)]
    elif a.mode == 'screen':
        jobs = [('R1', 3, 1), ('BBT', 20, 1), ('IT', 200, 1), ('BB', 1, 1)]
        jobs += [(s, 4, 1) for s in ('smoke-qs','smoke-tr','smoke-adaptive','smoke-alhess','smoke-friction')]
    else:
        jobs = [('R4', 1, r) for r in (1, 2)] + [('R4', 5, r) for r in (1, 2)]
        jobs += [('R1', 3, 1), ('BBT', 20, 1), ('IT', 200, 1), ('BB', 1, 1)]
        jobs += [(s, 4, 1) for s in ('smoke-qs', 'smoke-tr', 'smoke-adaptive', 'smoke-alhess', 'smoke-friction')]
    if a.only:
        jobs = [job for job in jobs if job[0] in a.only]
        if not jobs: raise SystemExit('No scenes selected')
    digest = hashlib.file_digest(a.binary.open('rb'), 'sha256').hexdigest()
    ledger = a.out / f'sequence-{a.prefix}{a.mode}.jsonl'
    for scene, steps, repeat in jobs:
        label = f'{a.prefix}{a.mode}-{scene}-s{steps}-r{repeat}'
        cmd = [sys.executable, str(Path(__file__).with_name('run.py')), '--workspace', str(a.workspace), label,
               '--scene', scene, '--steps', str(steps), '--threads', '0' if scene in ('R4','BB') else '1',
               '--timeout', '2700' if scene == 'R4' else '3600', '--out', str(a.out), '--binary', str(a.binary)]
        if a.mode != 'production':
            cmd += ['--set', '/solver/contact/semi_implicit/band_statistic="force_weighted"',
                    '--set', '/solver/contact/semi_implicit/initial_trim_estimate=true']
        record = dict(label=label, command=cmd, binary_sha256=digest,
                      started=datetime.datetime.now(datetime.timezone.utc).isoformat())
        with ledger.open('a') as f: f.write(json.dumps(record)+'\n')
        subprocess.run(['pmset','-g','batt'], stdout=(a.out/f'{label}-power.txt').open('w'), check=False)
        result = subprocess.run(cmd)
        record.update(driver_exit=result.returncode, finished=datetime.datetime.now(datetime.timezone.utc).isoformat())
        with ledger.open('a') as f: f.write(json.dumps(record)+'\n')
        if result.returncode: raise SystemExit(result.returncode)
        row = json.loads((a.out/'runs'/label/'row.json').read_text())
        if row['exit_code'] != 0 or row['timed_out']:
            print(f'{label}: solver failure retained; continuing matrix', flush=True)

if __name__ == '__main__': main()
