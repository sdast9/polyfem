#!/usr/bin/env python3
"""Run one EF-01 configuration (contact-efficiency plan, trim/AL survey).

usage: ef01_run.py LABEL --out DIR --scene R4 [--trim T] [--binary B] [--steps N]
                   [--threads T] [--timeout S] [--set /json/pointer=VALUE ...]

Builds on tools/qn_contact/qn_run.py (same scene table, iteration and physical
diagnostics on, small visual output) and adds, for every run:

* Newton with the scene's settings otherwise (R4's scene file asks for L-BFGS +
  Wolfe; its Newton runs use RobustArmijo as in the qn-contact investigation);
* `--trim T` pins the barrier trim: trim_min = trim_max = T, which every trim
  update is clamped to (the first-contact cap included);
* output/trim_predictors on (the observational EF-01 stream);
* coefficient-events.jsonl replaced by a link to /dev/null: on R4 the stream is
  2.6-17 GB per run (full coordinates per event) and EF-01 reads the trim from
  trim-predictors.jsonl instead. The records are still serialized, so the
  wall-time overhead of physical diagnostics is unchanged from the qn-contact
  evidence.

Writes OUT/runs/LABEL/{input.json,run.log,row.json,output/...}.
"""
import argparse, json, os, subprocess, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'qn_contact'))
import qn_run  # noqa: E402

ROOT = qn_run.ROOT
qn_run.SCENES.update({
    'smoke-tr': ROOT / 'polyfem/scenes/semi-implicit',
    # the user's thin-membrane ball burst, dhat 1e-5 (adaptive dhat)
    'BBT': ROOT / 'test_cases/ballburst_thin_membrane/input',
    # held out for EF-02/03 acceptance
    'IT': ROOT / 'test_cases/inertia_test/input',
    'BB': ROOT / 'test_cases/ball_burst/input',
})
qn_run.SCENE_FILE.update({'smoke-tr': 'transient-semi.json', 'BBT': 'params.json',
                          'IT': 'params.json', 'BB': 'params.json'})
SCENE_OVERRIDES = {
    'R4': {'/solver/nonlinear/line_search/method': 'RobustArmijo'},
}
# Keys of older Houdini exports that the current input spec refuses; removed
# from the run's input (the scene file is untouched) and listed in row.json.
SCENE_REMOVALS = {
    'BBT': ['/space/pressure_discr_order'],
}


def rewrite_file_selections(config):
    """[{"file": X}] and {"file": X} -> "X": the same FileSelection(resolve_path(X))
    in Selection.cpp, without Selection::build's const json operator[] read of
    the missing "id" key (undefined behaviour; three EF-01 runs segfaulted in
    mesh loading on 2026-09-23). Returns the rewritten pointers."""
    done = []
    for i, g in enumerate(config.get('geometry', [])):
        for key in ('surface_selection', 'volume_selection'):
            v = g.get(key)
            if isinstance(v, list) and len(v) == 1:
                v = v[0]
            if isinstance(v, dict) and set(v) == {'file'} and isinstance(v['file'], str):
                g[key] = v['file']
                done.append(f'/geometry/{i}/{key}')
    return done


def remove_pointer(config, pointer):
    node = config
    parts = pointer.strip('/').split('/')
    for part in parts[:-1]:
        node = node.get(part, {})
    return node.pop(parts[-1], None) is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('label')
    ap.add_argument('--scene', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--binary', default=str(qn_run.SHARED_BIN))
    ap.add_argument('--trim', type=float, help='pin the trim: trim_min = trim_max = TRIM')
    ap.add_argument('--steps', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=900)
    ap.add_argument('--threads', type=int, default=1)
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--note', default='')
    ap.add_argument('--no-predictors', action='store_true',
                    help='omit output/trim_predictors (binaries built before EF-01 refuse the key)')
    a = ap.parse_args()

    overrides = dict(SCENE_OVERRIDES.get(a.scene, {}))
    if not a.no_predictors:
        overrides['/output/trim_predictors'] = True
    overrides['/output/paraview/fields'] = ['solution']
    if a.trim is not None:
        overrides['/solver/contact/semi_implicit/trim_min'] = a.trim
        overrides['/solver/contact/semi_implicit/trim_max'] = a.trim
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
    config = qn_run.build_config(a.scene, 'Newton', a.steps, overrides, directory)
    removed = [p for p in SCENE_REMOVALS.get(a.scene, []) if remove_pointer(config, p)]
    rewritten = rewrite_file_selections(config)
    (directory / 'input.json').write_text(json.dumps(config, indent=2) + '\n')
    (directory / 'output').mkdir()
    os.symlink('/dev/null', directory / 'output' / 'coefficient-events.jsonl')

    binary = Path(a.binary).resolve()
    command = [str(binary), '--json', str(directory / 'input.json'),
               '--max_threads', str(a.threads), '--log_level', 'debug']
    row = {'label': a.label, 'scene': a.scene, 'method': 'Newton', 'trim_pin': a.trim,
           'steps_requested': a.steps, 'threads': a.threads, 'overrides': overrides,
           'note': a.note, 'binary': str(binary), 'binary_sha256': qn_run.sha(binary),
           'command': command, 'timeout_seconds': a.timeout,
           'coefficient_events': 'discarded (/dev/null)', 'removed_keys': removed,
           'selection_rewrites': rewritten,
           'started': time.strftime('%Y-%m-%dT%H:%M:%S')}
    start = time.monotonic()
    with (directory / 'run.log').open('w') as log:
        try:
            row['exit_code'] = subprocess.run(command, cwd=directory, stdout=log,
                                              stderr=subprocess.STDOUT,
                                              timeout=a.timeout).returncode
            row['timed_out'] = False
        except subprocess.TimeoutExpired:
            row['exit_code'] = None
            row['timed_out'] = True
    row['wall_seconds'] = round(time.monotonic() - start, 2)
    (directory / 'row.json').write_text(json.dumps(row, indent=2) + '\n')
    print(json.dumps({k: row[k] for k in ('label', 'exit_code', 'timed_out', 'wall_seconds')}), flush=True)


if __name__ == '__main__':
    main()
