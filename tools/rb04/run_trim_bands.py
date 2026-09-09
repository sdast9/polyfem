"""Bounded RB-04 trim-band pilot. Never overwrites an evidence directory."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/pf08'))
from measure_fem import measure

BANDS = [('lower_low', .1, .9), ('default', .5, .9),
         ('lower_high', .8, .9), ('upper_low', .5, .6), ('upper_high', .5, .99)]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    binary = ROOT / 'build/PolyFEM_bin'
    results = dict(binary_sha256=sha(binary), runner_sha256=sha(Path(__file__)),
                   source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                   scope='Endpoint band sensitivity; incomplete trajectory work accounting', runs=[])
    for scene in ('quasistatic-semi', 'transient-semi', 'quasistatic-semi-friction'):
        for label, lower, upper in BANDS:
            directory = out / f'{scene}-{label}'
            directory.mkdir()
            config = json.loads((ROOT / 'scenes/semi-implicit' / f'{scene}.json').read_text())
            config['solver']['contact'].setdefault('semi_implicit', {}).update(trim_lower=lower, trim_upper=upper)
            config['output'].update(directory=str(directory / 'output'), stats=True, physical_diagnostics=True)
            for asset in ('cube.mesh', 'slab.obj'):
                shutil.copy2(ROOT / 'scenes/semi-implicit' / asset, directory / asset)
            (directory / 'params.json').write_text(json.dumps(config, indent=2)+'\n')
            command = [str(binary), '--json', str(directory/'params.json'), '--log_level', 'debug']
            row = dict(scene=scene, band=label, lower=lower, upper=upper, command=command,
                       input_sha256={p.name: sha(p) for p in directory.iterdir()}, endpoints=[])
            results['runs'].append(row)
            (out/'results.json').write_text(json.dumps(results, indent=2)+'\n')
            start = time.monotonic()
            with (directory/'run.log').open('w') as log:
                try:
                    row['exit'] = subprocess.run(command, cwd=directory, stdout=log, stderr=subprocess.STDOUT, timeout=120).returncode
                    row['timed_out'] = False
                except subprocess.TimeoutExpired:
                    row.update(exit=None, timed_out=True)
            row['wall_seconds'] = time.monotonic()-start
            log = (directory/'run.log').read_text()
            row['observed_trim_log_events'] = log.count('Barrier stiffness trim:')
            row['observed_refresh_log_events'] = log.count('Refreshed semi-implicit barrier stiffness')
            records = directory/'output/physical-diagnostics.jsonl'
            for record in ([json.loads(s) for s in records.read_text().splitlines()] if records.exists() else []):
                endpoint = {k: record.get(k) for k in ('step', 'time', 'outcome', 'termination', 'lagging', 'contact',
                    'free_residual_norm', 'bc_error_inf', 'min_det_F', 'elastic_energy', 'barrier_energy', 'kinetic_energy', 'measurement_error')}
                if record['outcome'] == 'accepted':
                    step = record['step']
                    reference = measure(directory/'output'/f'step_{step}.vtu', 1e7, .45, step*.25)
                    endpoint['independent_elastic'] = reference
                    endpoint['displacement_norm'] = float(np.linalg.norm(record['endpoint']['value']))
                row['endpoints'].append(endpoint)
            row['complete'] = row['exit'] == 0 and [r['step'] for r in row['endpoints'] if r['outcome'] == 'accepted'] == [1, 2, 3, 4]
            (out/'results.json').write_text(json.dumps(results, indent=2)+'\n')
            print(scene, label, 'exit', row['exit'], 'complete', row['complete'], flush=True)


if __name__ == '__main__':
    main()
