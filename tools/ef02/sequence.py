#!/usr/bin/env python3
"""Sequential EF-02/03 matrix. Run after builds; binaries must be immutable copies.

Usage: sequence.py --workspace W --out E --binary B --mode pilot|production|candidate
Records each command BEFORE execution. Refuses existing runs; no silent cache reuse.
Production mode resets any opt-in controller option a scene file selects to its
production value (PRODUCTION_CONTROLLER) and records the pins in the ledger.
"""
import argparse, datetime, hashlib, json, subprocess, sys
from pathlib import Path

SI = '/solver/contact/semi_implicit/'
# Production controller: the spec defaults of every opt-in trim-controller
# option that can act without the force-weighted band. A re-exported scene
# can select them (R4's 2026-09-26 export selected force_weighted + the
# estimate, and EF-07's R4 "production" runs silently ran v3:
# docs/r4-production-speedup-20260928.md). Production mode resets any of these
# the scene file sets to another value. Keys the scene leaves out are not
# written, so binaries that predate an option still accept the input.
PRODUCTION_CONTROLLER = {
    'band_statistic': 'rms',
    'initial_trim_estimate': False,
    'clamped_contacts': 'keep',
    'collapse_guard_partial': False,
    'collapse_exclude_born': False,
    'collapse_responsiveness_veto': False,
    'trim_step_excursion': 0,
    'trim_reversal_limit': 0,
}


def scene_config(workspace, scene):
    """The scene file run.py/ef01_run.py will start from (same tables)."""
    sys.path.insert(0, str(workspace / 'polyfem/tools/ef01'))
    import ef01_run
    table, files = ef01_run.qn_run.SCENES, ef01_run.qn_run.SCENE_FILE
    for key, name in [('smoke-adaptive', 'quasistatic-adaptive.json'),
                      ('smoke-alhess', 'quasistatic-semi-alhess.json'),
                      ('smoke-friction', 'quasistatic-semi-friction.json')]:
        table.setdefault(key, workspace / 'polyfem/scenes/semi-implicit')
        files.setdefault(key, name)
    return json.loads((table[scene] / files[scene]).read_text())


def production_pins(config):
    """--set arguments that return the scene's controller to production."""
    semi = config.get('solver', {}).get('contact', {}).get('semi_implicit', {})
    pins = []
    for key, value in PRODUCTION_CONTROLLER.items():
        if key in semi and semi[key] != value:
            pins.append(f'{SI}{key}={json.dumps(value)}')
    return pins


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--workspace', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--mode', choices=['pilot', 'screen', 'production', 'candidate'], required=True)
    ap.add_argument('--prefix', default='')
    ap.add_argument('--only', action='append', help='Run only these scene names; preserve completed evidence separately')
    ap.add_argument('--set', action='append', default=[], dest='extra_set',
                    help='Extra run.py --set POINTER=JSON for non-production modes (EF-07 options); recorded in the ledger')
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
        pinned = []
        if a.mode == 'production':
            pinned = production_pins(scene_config(a.workspace, scene))
            for pin in pinned:
                cmd += ['--set', pin]
            if pinned:
                print(f'{label}: scene selects experimental controller options; '
                      f'production pins {pinned}', flush=True)
        else:
            cmd += ['--set', '/solver/contact/semi_implicit/band_statistic="force_weighted"',
                    '--set', '/solver/contact/semi_implicit/initial_trim_estimate=true',
                    # EF-02/03 v3 as measured: the proxy guard (the default became pair
                    # on 2026-09-28, docs/ef-07-trim-loop.md); --set overrides it.
                    '--set', '/solver/contact/semi_implicit/collapse_guard_basis="proxy"']
            for extra in a.extra_set:
                cmd += ['--set', extra]
        record = dict(label=label, command=cmd, binary_sha256=digest, production_pins=pinned,
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
