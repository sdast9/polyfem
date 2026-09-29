#!/usr/bin/env python3
"""Clamped-contact matrix (docs/clamped-contacts-20260928.md) on the EF-01/EF-02 driver.

usage: sequence.py --workspace W --out E --binary B --clamped keep|exclude_statistics|exclude_collisions
                   [--controller rms|force_weighted] [--only SCENE ...] [--r4-steps 1 5]
                   [--balance-dofs all|free]

Every run states the controller explicitly: the R4 scene file was re-exported
on 2026-09-26 with band_statistic force_weighted + initial_trim_estimate, so a
run that relies on the scene file does not run the production controller.
rms = band_statistic rms, initial_trim_estimate false (production);
force_weighted = the experimental mode (force_weighted + estimate, pair guard).
--balance-dofs states semi_implicit.gradient_balance_dofs (docs/gradient-balance-
free-dofs-20260929.md) and prefixes the label with it; without it the key is absent.
Labels: [<balance>-]<clamped>-<controller>-<scene>-s<steps>-r<repeat>. Existing runs are
refused (run.py); the ledger is written before and after every run.
"""
import argparse, datetime, hashlib, json, subprocess, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SMOKES = ('smoke-qs', 'smoke-tr', 'smoke-adaptive', 'smoke-alhess', 'smoke-friction')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--workspace', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--clamped', choices=['keep', 'exclude_statistics', 'exclude_collisions'], required=True)
    ap.add_argument('--controller', choices=['rms', 'force_weighted'], default='rms')
    ap.add_argument('--only', action='append')
    ap.add_argument('--r4-steps', type=int, nargs='*', default=[1])
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--balance-dofs', choices=['all', 'free'])
    ap.add_argument('--timeout', type=float, help='per-run cap in seconds (default 2700 for R4, 3600 otherwise)')
    ap.add_argument('--repeat-start', type=int, default=1)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    jobs = [('R4', s, 1) for s in a.r4_steps]
    jobs += [('R1', 3, 1), ('BBT', 20, 1), ('IT', 200, 1), ('BB', 1, 1)]
    jobs += [(s, 4, 1) for s in SMOKES]
    if a.only:
        jobs = [j for j in jobs if j[0] in a.only]
    jobs = [(s, n, r) for s, n, _ in jobs for r in range(a.repeat_start, a.repeat_start + a.repeat)]
    digest = hashlib.file_digest(a.binary.open('rb'), 'sha256').hexdigest()
    ledger = a.out / f'sequence-{a.balance_dofs + "-" if a.balance_dofs else ""}{a.clamped}-{a.controller}.jsonl'
    si = '/solver/contact/semi_implicit/'
    sets = [si + f'band_statistic="{a.controller}"',
            si + f'initial_trim_estimate={"true" if a.controller == "force_weighted" else "false"}']
    # Always explicit: the default changed from keep to exclude_statistics on
    # 2026-09-29, and binaries before 8f8b51878 refuse the key altogether.
    sets.append(si + f'clamped_contacts="{a.clamped}"')
    if a.balance_dofs:
        sets.append(si + f'gradient_balance_dofs="{a.balance_dofs}"')
    prefix = f'{a.balance_dofs}-' if a.balance_dofs else ''
    for scene, steps, repeat in jobs:
        label = f'{prefix}{a.clamped}-{a.controller}-{scene}-s{steps}-r{repeat}'
        cmd = [sys.executable, str(HERE.parent / 'ef02/run.py'), '--workspace', str(a.workspace), label,
               '--scene', scene, '--steps', str(steps), '--threads', '0' if scene in ('R4', 'BB') else '1',
               '--timeout', str(a.timeout) if a.timeout else ('2700' if scene == 'R4' else '3600'), '--out', str(a.out), '--binary', str(a.binary)]
        for s in sets:
            cmd += ['--set', s]
        record = dict(label=label, command=cmd, binary_sha256=digest,
                      started=datetime.datetime.now(datetime.timezone.utc).isoformat())
        with ledger.open('a') as f:
            f.write(json.dumps(record) + '\n')
        result = subprocess.run(cmd)
        record.update(driver_exit=result.returncode, finished=datetime.datetime.now(datetime.timezone.utc).isoformat())
        with ledger.open('a') as f:
            f.write(json.dumps(record) + '\n')
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
