"""RB-20/21: bounded ball-on-plate runs (user scene test_cases/input/params.json).

Copies the scene's inputs, bounds it to `--steps` steps at the scene's own dt
(tend rescaled), sets a local output directory with physical diagnostics, and
applies semi_implicit overrides. Reports completion, Newton iterations, stall
restarts, wall time, min gap/dhat and the between-steps drift metric.

Usage: python3 tools/rb20/run_ball_on_plate.py --output /abs/fresh --steps 8 [--override key=json ...]
"""
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_drift_matrix import drift_events, parse_override, sha  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SCENE = ROOT.parent/'test_cases/input'


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--steps', type=int, default=8)
    p.add_argument('--override', action='append', default=[])
    p.add_argument('--timeout', type=int, default=3600)
    p.add_argument('--label', default='')
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    cfg = json.loads((SCENE/'params.json').read_text())
    dt = cfg['time']['tend']/cfg['time']['time_steps']
    cfg['time'].update(tend=dt*args.steps, time_steps=args.steps)
    cfg['solver']['contact']['semi_implicit'].update(dict(parse_override(o) for o in args.override))
    cfg['output']['directory'] = str(out/'output')
    cfg['output']['paraview']['file_name'] = str(out/'output/sim.pvd')
    cfg['output']['physical_diagnostics'] = True
    cfg['output']['log']['level'] = 'debug'
    inputs = {}
    for name in sorted(x.name for x in SCENE.iterdir() if x.suffix in ('.msh', '.txt', '.obj')):
        shutil.copy2(SCENE/name, out/name)
        inputs[name] = sha(out/name)
    (out/'params.json').write_text(json.dumps(cfg, indent=2)+'\n')
    binary = ROOT/'build/PolyFEM_bin'
    cmd = [str(binary), '--json', str(out/'params.json'), '--log_level', 'debug']
    t0 = time.monotonic()
    with (out/'run.log').open('w') as log:
        try:
            rc = subprocess.run(cmd, cwd=out, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout).returncode
        except subprocess.TimeoutExpired:
            rc = None
    text = (out/'run.log').read_text(errors='replace')
    gaps = [float(m) for m in re.findall(r'Minimum distance during solve: ([0-9.e+-]+)', text)]
    result = dict(binary_sha256=sha(binary), source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                  label=args.label, scene='test_cases/input/params.json', steps=args.steps, dt=dt, overrides=dict(parse_override(o) for o in args.override),
                  input_sha256=inputs, params_sha256=sha(out/'params.json'), exit=rc, wall_seconds=time.monotonic()-t0,
                  error_lines=text.count('[error]'), stall_restarts=text.count('retuning barrier stiffness and restarting'),
                  saved_steps=len(list((out/'output').glob('step_*.vtu'))) if (out/'output').exists() else 0,
                  newton_iterations=sum(int(m) for m in re.findall(r'Finished: [^(]*\(iters=(\d+)', text)),
                  min_gap_over_dhat=(min(gaps)/cfg['contact']['dhat']) if gaps else None,
                  drift_events=drift_events(out/'output'),
                  scope='Bounded user scene; numerical termination only')
    ev = result['drift_events'] or []
    if ev:
        result['summary'] = dict(events=len(ev), max_relative_change_all=max((e['force_change_relative'] or 0) for e in ev),
                                 max_relative_change_trim_unchanged=max([(e['force_change_relative'] or 0) for e in ev if e['trim_before'] == e['trim_after']] or [0]),
                                 trim_moves=[(e['trim_before'], e['trim_after']) for e in ev if e['trim_before'] != e['trim_after']])
    (out/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: result.get(k) for k in ('exit', 'wall_seconds', 'saved_steps', 'stall_restarts', 'newton_iterations', 'min_gap_over_dhat', 'summary')}))


if __name__ == '__main__':
    main()
