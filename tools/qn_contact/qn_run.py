#!/usr/bin/env python3
"""Run one configuration of the real-scene quasi-Newton experiments.

usage: qn_run.py LABEL [--out DIR] --scene R1 [--binary PATH] [--method M] [--steps N]
                 [--timeout S] [--threads T] [--set /json/pointer=VALUE ...]

Writes runs/LABEL/{input.json,run.log,row.json,output/...}. VALUE is parsed as
JSON when possible, else kept as a string. Nothing here changes a default of
the solver; every deviation from the scene file is in row.json.
"""
import argparse, hashlib, json, math, shutil, subprocess, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]  # fable_polyfem (tools/qn_contact inside polyfem/)
SHARED_BIN = ROOT / 'polyfem/build/PolyFEM_bin'
SCENES = {
    # the user's current Houdini export: plate + ball head, dhat 1e-6
    'R1': ROOT / 'test_cases/input',
    'smoke-qs': ROOT / 'polyfem/scenes/semi-implicit',
    # inflation against an obstacle (Newton: 3-5 iterations/step)
    'R3': ROOT / 'test_cases/inflation/input',
    # the user's large uniaxial self-contact scene (their L-BFGS+Wolfe attempt)
    'R4': ROOT / 'test_cases/uniax_mesh_constraintfloor_zero_b/input',
}
SCENE_FILE = {'R1': 'params.json', 'smoke-qs': 'quasistatic-semi.json',
              'R3': 'params.json', 'R4': 'params.json'}


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def set_pointer(config, pointer, value):
    node = config
    parts = pointer.strip('/').split('/')
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = value


def build_config(scene, method, steps, overrides, directory):
    src = SCENES[scene]
    config = json.loads((src / SCENE_FILE[scene]).read_text())
    for f in src.iterdir():
        if f.suffix in ('.msh', '.txt', '.mesh', '.obj', '.vtk'):
            if f.stat().st_size < 60e6:
                (directory / f.name).symlink_to(f)
    if method:
        config['solver'].setdefault('nonlinear', {})['solver'] = method
        if method == 'BFGS':
            config['solver'].setdefault('linear', {})['solver'] = 'Eigen::LDLT'
    nl = config['solver'].setdefault('nonlinear', {})
    nl.setdefault('advanced', {})['iteration_diagnostics'] = True
    out = config.setdefault('output', {})
    out['directory'] = str(directory / 'output')
    out['physical_diagnostics'] = True
    # small visual output: the experiment reads the diagnostic streams
    out['paraview'] = {'file_name': 'sim.pvd', 'skip_frame': 1, 'volume': True,
                       'surface': False, 'vismesh_rel_area': 1.0,
                       'high_order_mesh': False,
                       'options': {'use_hdf5': False}}
    t = config['time']
    dt = t['dt'] if 'dt' in t else (t['tend'] - t.get('t0', 0)) / t['time_steps']
    if steps:
        t.pop('time_steps', None)
        t['dt'] = dt
        t['tend'] = t.get('t0', 0) + steps * dt
    for pointer, value in overrides.items():
        set_pointer(config, pointer, value)
    return config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('label')
    ap.add_argument('--scene', default='R1')
    ap.add_argument('--binary', default=str(SHARED_BIN))
    ap.add_argument('--method')
    ap.add_argument('--steps', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=1800)
    ap.add_argument('--threads', type=int, default=1)
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--note', default='')
    ap.add_argument('--out', default='.', help='evidence directory; runs go to OUT/runs/LABEL')
    a = ap.parse_args()

    overrides = {}
    for item in a.set:
        pointer, value = item.split('=', 1)
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            pass
        overrides[pointer] = value

    directory = Path(a.out).resolve() / 'runs' / a.label
    if directory.exists():
        raise SystemExit(f'{directory} exists; choose a fresh label')
    directory.mkdir(parents=True)
    config = build_config(a.scene, a.method, a.steps, overrides, directory)
    (directory / 'input.json').write_text(json.dumps(config, indent=2) + '\n')
    binary = Path(a.binary)
    command = [str(binary), '--json', str(directory / 'input.json'),
               '--max_threads', str(a.threads), '--log_level', 'debug']
    row = {'label': a.label, 'scene': a.scene, 'method': a.method or 'scene default',
           'steps_requested': a.steps, 'threads': a.threads, 'overrides': overrides,
           'note': a.note, 'binary': str(binary), 'binary_sha256': sha(binary),
           'command': command, 'started': time.strftime('%Y-%m-%dT%H:%M:%S')}
    start = time.monotonic()
    with (directory / 'run.log').open('w') as log:
        try:
            row['exit_code'] = subprocess.run(command, cwd=directory, stdout=log,
                                              stderr=subprocess.STDOUT,
                                              timeout=a.timeout).returncode
        except subprocess.TimeoutExpired:
            row['exit_code'] = None
            row['timeout_seconds'] = a.timeout
    row['wall_seconds'] = round(time.monotonic() - start, 2)
    (directory / 'row.json').write_text(json.dumps(row, indent=2) + '\n')
    print(json.dumps({k: row[k] for k in ('label', 'exit_code', 'wall_seconds')}))


if __name__ == '__main__':
    main()
