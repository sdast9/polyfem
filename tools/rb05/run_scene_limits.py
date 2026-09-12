"""RB-05 end-to-end resource-limit checks on public scenes.

python3 tools/rb05/run_scene_limits.py --binary build/PolyFEM_bin --output /absolute/fresh/dir
        [--scene scenes/semi-implicit/quasistatic-semi.json ...] [--threads 1]

For each scene, isolated copies of the input are run with the physical
diagnostics on and:

  unlimited    no resource limits (the reference);
  generous     limits far above the scene's needs -- the solution must be
               bit-identical to `unlimited` (the counting paths run, the
               bounds never fire);
  tiny         max_cell_items = 1 -- the first static broad-phase build (the
               contact form's init) must fail with the named resource error
               before any allocation: no accepted step, a failed-attempt
               record, non-zero exit;
  sweep        max_cell_items just below the first trial sweep's item count
               (read from `generous`'s log) -- the failure must happen inside
               line_search_begin: the pre-build sweep diagnostics are logged,
               an `aborted` row with broad_phase_built=false ends the attempt
               stream, the RB-04 attempt-stream checker accepts the run;
  unsupported  limits with broad_phase "bvh" -- refused at startup with the
               named configuration error, before any solve.

Everything is written under --output; nothing is regenerated or relaxed.
"""
import argparse
import hashlib
import importlib.util
import json
import re
import struct
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools' / 'rb18'))
sys.path.insert(0, str(ROOT / 'tools' / 'rb04'))
summarize = importlib.import_module('summarize_smokes')
checker = importlib.import_module('check_solver_attempts')

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('--binary', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--scene', type=Path, action='append')
parser.add_argument('--threads', type=int, default=1)
parser.add_argument('--timeout', type=float, default=1800)
args = parser.parse_args()
binary = args.binary.resolve()
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)
scenes = args.scene or [ROOT / 'scenes/semi-implicit/quasistatic-semi.json', ROOT / 'scenes/semi-implicit/quasistatic-adaptive.json']


def last_solution(directory):
    vtus = sorted((directory / 'output').glob('step_*.vtu'), key=lambda p: int(re.search(r'step_(\d+)', p.name).group(1)))
    if not vtus:
        return None, 0
    ncomp, values = summarize.read_solution(vtus[-1])
    if values is None:
        return None, len(vtus)
    return hashlib.sha256(struct.pack('<%dd' % len(values), *values)).hexdigest(), len(vtus)


def run(scene, name, limits, broad_phase=None):
    directory = out / scene.stem / name
    (directory / 'output').mkdir(parents=True, exist_ok=True)
    config = json.loads(scene.read_text())
    config['root_path'] = str(scene.resolve())
    config.setdefault('output', {}).update(directory=str(directory / 'output'), stats=True, physical_diagnostics=True)
    ccd = config.setdefault('solver', {}).setdefault('contact', {}).setdefault('CCD', {})
    if limits:
        ccd['resource_limits'] = limits
    if broad_phase:
        ccd['broad_phase'] = broad_phase
    (directory / 'params.json').write_text(json.dumps(config, indent=2) + '\n')
    cmd = [str(binary), '--json', str(directory / 'params.json'), '--log_level', 'debug', '--max_threads', str(args.threads)]
    with open(directory / 'run.log', 'w') as log:
        try:
            proc = subprocess.run(cmd, cwd=directory, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
            exit_code = proc.returncode
        except subprocess.TimeoutExpired:
            exit_code = None
    text = (directory / 'run.log').read_text(errors='replace')
    sha, steps = last_solution(directory)
    record = {
        'name': name, 'command': cmd, 'exit': exit_code, 'limits': limits, 'broad_phase': broad_phase,
        'solution_sha256': sha, 'vtu_steps': steps,
        'budget_error_lines': [l for l in text.splitlines() if 'resource budget exceeded' in l or 'cannot be enforced' in l],
        'sweep_failure_lines': [l for l in text.splitlines() if 'Broad phase over trial step failed' in l],
        'pre_build_lines': len([l for l in text.splitlines() if 'Broad phase over trial step: trial Linf' in l]),
        'intermediate_lines': [l for l in text.splitlines() if 'cell items' in l and 'pre-filter pair emissions' in l][:3],
    }
    diagnostics = directory / 'output' / 'physical-diagnostics.jsonl'
    if diagnostics.exists():
        rows = [json.loads(l) for l in diagnostics.read_text().splitlines() if l.strip()]
        record['diagnostic_outcomes'] = [(r['step'], r['outcome']) for r in rows]
        record['failed_attempt_exception'] = next((r['termination'].get('exception') for r in rows if r['outcome'] != 'accepted'), None)
        record['attempt_summaries'] = [r.get('attempt_summary', {}) for r in rows]
    attempts = directory / 'output' / 'solver-attempts.jsonl'
    if attempts.exists():
        rows = [json.loads(l) for l in attempts.read_text().splitlines() if l.strip()]
        record['attempt_rows'] = len(rows)
        record['aborted_rows'] = [r for r in rows if r['kind'] == 'aborted']
    if diagnostics.exists() and attempts.exists() and record.get('attempt_rows', 0) == 0:
        record['attempt_stream_check'] = 'not applicable: the attempt failed before any PolySolve minimize started (no rows)'
    elif diagnostics.exists() and attempts.exists():
        try:
            record['attempt_stream_check'] = checker.check_run(directory)
            record['attempt_stream_check_passed'] = True
        except AssertionError as error:
            record['attempt_stream_check_passed'] = False
            record['attempt_stream_check_error'] = repr(error)
    print(name, 'exit', exit_code, 'steps', steps, 'sha', (sha or '')[:12], 'budget lines', len(record['budget_error_lines']), 'aborted', len(record.get('aborted_rows', [])), 'stream check', record.get('attempt_stream_check_passed'), flush=True)
    return record


results = {'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(), 'threads': args.threads, 'scenes': {}}
for scene in scenes:
    records = {}
    records['unlimited'] = run(scene, 'unlimited', None)
    records['generous'] = run(scene, 'generous', {'max_cell_items': 10**9, 'max_candidate_emissions': 10**9})
    records['tiny'] = run(scene, 'tiny', {'max_cell_items': 1})
    # The first trial sweep's item count, from the generous run's log.
    first = next((l for l in (out / scene.stem / 'generous' / 'run.log').read_text(errors='replace').splitlines() if 'cell items' in l and 'pre-filter pair emissions' in l), None)
    sweep_items = int(re.search(r'\((\d+) cell items', first).group(1)) if first else None
    records['sweep'] = run(scene, 'sweep', {'max_cell_items': sweep_items - 1}) if sweep_items else {'skipped': 'no sweep statistics in the generous log'}
    records['sweep_threshold_items'] = sweep_items
    records['unsupported'] = run(scene, 'unsupported', {'max_cell_items': 1000}, broad_phase='bvh')
    records['generous_identical_to_unlimited'] = records['generous']['solution_sha256'] == records['unlimited']['solution_sha256'] and records['unlimited']['solution_sha256'] is not None
    results['scenes'][scene.stem] = records
    print(scene.stem, 'generous == unlimited:', records['generous_identical_to_unlimited'], flush=True)
(out / 'results.json').write_text(json.dumps(results, indent=2, default=str) + '\n')
