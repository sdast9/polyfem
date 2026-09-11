"""RB-19: rerun the RB-04 step-1 no-contact stall with different thread counts.

Scene: quasistatic-semi with dt=.0625 and trim band [.1,.9] (the RB-04
refinement configuration that failed at step 1 before contact). Each run uses
the current build/PolyFEM_bin at trace log level and records exit code, stall
restarts, saved steps, log hash and how often the line search accepted a step
on the gradient-norm fallback ("energy at roundoff"). Single-threaded runs of
the same input are expected to produce identical logs.

Usage: python3 tools/rb19/run_step1_threads.py /absolute/fresh/output [default default 1 ...]
The trailing arguments are thread counts ("default" = no --max_threads).
"""
import hashlib
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=False)
    threads = sys.argv[2:] or ['default', 'default', 'default', '1', '1', '1']
    binary = ROOT/'build/PolyFEM_bin'
    scene, lower, dt = 'quasistatic-semi', .1, .0625
    result = dict(binary_sha256=sha(binary), runner_sha256=sha(Path(__file__)),
                  source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                  scene=scene, dt=dt, trim_lower=lower, trim_upper=.9, runs=[])
    for i, th in enumerate(threads):
        d = out/f'run-{i:02d}-threads-{th}'
        d.mkdir()
        cfg = json.loads((ROOT/'scenes/semi-implicit'/f'{scene}.json').read_text())
        cfg['time']['dt'] = dt
        cfg['solver']['contact'].setdefault('semi_implicit', {}).update(trim_lower=lower, trim_upper=.9)
        cfg['output'].update(directory=str(d/'output'), stats=True)
        for asset in ('cube.mesh', 'slab.obj'):
            shutil.copy2(ROOT/'scenes/semi-implicit'/asset, d/asset)
        (d/'params.json').write_text(json.dumps(cfg, indent=2)+'\n')
        cmd = [str(binary), '--json', str(d/'params.json'), '--log_level', 'trace']
        if th != 'default':
            cmd += ['--max_threads', th]
        start = time.monotonic()
        with (d/'run.log').open('w') as log:
            try:
                rc = subprocess.run(cmd, cwd=d, stdout=log, stderr=subprocess.STDOUT, timeout=600).returncode
            except subprocess.TimeoutExpired:
                rc = None
        text = (d/'run.log').read_text(errors='replace')
        steps = sorted(p.name for p in (d/'output').glob('step_*.vtu')) if (d/'output').exists() else []
        row = dict(index=i, threads=th, command=cmd, exit=rc, wall_seconds=time.monotonic()-start,
                   restarts=text.count('retuning barrier stiffness and restarting'),
                   unchanged_restart_terminations=text.count('stall persisted with no retunable contact state'),
                   fallback_accepts=text.count('accepted on gradient norm'),
                   saved_steps=len(steps), log_sha256=sha(d/'run.log'), input_sha256=sha(d/'params.json'))
        result['runs'].append(row)
        (out/'threads-results.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps({k: row[k] for k in ('index', 'threads', 'exit', 'restarts', 'fallback_accepts', 'saved_steps')}), flush=True)


if __name__ == '__main__':
    main()
