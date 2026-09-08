"""Run isolated PF-08 public fixtures. No private scenes or golden regeneration.

Preserve tend and prescribed-motion law. Explicit resolution experiments vary
only mesh refinement and dt; the five original smokes retain their schedules.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--binary', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
source = Path(__file__).resolve().parents[2] / 'scenes/semi-implicit'
out = a.output.resolve()
out.mkdir(parents=True, exist_ok=True)
cases = [(n, n, 0, .25) for n in ('quasistatic-semi', 'quasistatic-adaptive', 'transient-semi', 'quasistatic-semi-friction', 'quasistatic-semi-alhess')]
cases += [(f'{n}-dt{dt}', n, 0, dt) for n in ('quasistatic-semi', 'transient-semi') for dt in (.125, .0625)]
cases += [('quasistatic-semi-ref1', 'quasistatic-semi', 1, .25)]
cases += [('quasistatic-semi-dt0.0625-repeat', 'quasistatic-semi', 0, .0625)]
records = []
for name, base, refs, dt in cases:
    run = out / name / 'current'
    inp, output = run / 'input', run / 'output'
    inp.mkdir(parents=True, exist_ok=False)
    output.mkdir()
    config = json.loads((source / (base + '.json')).read_text())
    for asset in ('cube.mesh', 'slab.obj'):
        shutil.copy2(source / asset, inp / asset)
    config['time']['dt'] = dt
    if refs:
        config['geometry'][0]['n_refs'] = refs
    config['output']['directory'] = str(output)
    (inp / 'params.json').write_text(json.dumps(config, indent=2) + '\n')
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in inp.iterdir()}
    cmd = [str(a.binary.resolve()), '--json', str(inp / 'params.json'), '--log_level', 'debug']
    start = time.monotonic()
    with (run / 'run.log').open('w') as log:
        completed = subprocess.run(cmd, cwd=run, stdout=log, stderr=subprocess.STDOUT)
    records.append(dict(scene=name, mode='current', exit_code=completed.returncode,
                        wall_seconds=time.monotonic()-start, command=cmd,
                        input_sha256=hashes, refinements=refs, dt=dt, tend=config['time']['tend']))
    (out / 'scene-results.json').write_text(json.dumps(records, indent=2) + '\n')
    print(name, completed.returncode, flush=True)

raise SystemExit(0 if all(r["exit_code"] == 0 for r in records) else 1)
