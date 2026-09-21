"""CI-03 friction-policy A/B on the historical scene fixtures.

Usage:
  python3 tools/ci03/ab_runner.py --unit-tests build/tests/unit_tests --data data \
      --output /absolute/fresh/dir --label after \
      [--variants default,fi1,fi2] [--fixtures 2d-large-mass-ratio,...] [--jobs 4]

Every run is an isolated copy of one pinned polyfem-data fixture -- the fixture
file, its complete `common` chain and every mesh those files reference, at the
same relative paths under a private data root -- run through the real scene
harness (`unit_tests run_manifest_env`, i.e. tests/verify_run.cpp: the recorded
test duration, the harness's linear-solver selection, one thread) with the
run's outputs in its own directory. A variant is an overlay written into the
copied top-level fixture only:

  default       no key written (the copy is byte-identical to the pinned fixture)
  fiN           solver/contact/friction_iterations = N (fi1, fi2, fi4, ...; fiinf = -1)
  <v>-diag      the same plus output/physical_diagnostics and output/manifest
                (the RB-04 per-step record and the RB-12 run manifest with the
                fully resolved input)

The record of each run (`run.json`) holds the six harness metrics, their
relative error against the fixture's stored reference, the harness verdict, the
lag-loop summary read from the fixture's own `sim.json` (`solver_info` `rc`
rows, one per lag re-solve) and, for diag variants, the per-step lagging
states of `physical-diagnostics.jsonl`; `summary.json` / `summary.md` collect
them. Public data only; nothing is modified in place.
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

FIXTURES = {
    '2d-large-mass-ratio': 'contact/examples/2D/large-ratios/large-mass-ratio.json',
    '3d-large-mass-ratio': 'contact/examples/3D/large-ratios/large-mass-ratio.json',
    'ball-bounce-p4-dt0.01': 'contact/examples/3D/higher-order/ball-bounce/P4-dt=0.01.json',
}
METRICS = ['err_l2', 'err_h1', 'err_h1_semi', 'err_linf', 'err_linf_grad', 'err_lp']
MESH_PATCH = re.compile(r'^/geometry/\d+/mesh$')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def resolve_chain(data_root, relative):
    """Return [(relative path, json)] for the fixture and its common chain, top first."""
    chain = []
    rel = Path(relative)
    while True:
        path = data_root / rel
        doc = json.loads(path.read_text())
        chain.append((rel.as_posix(), doc))
        if 'common' not in doc:
            return chain
        rel = (rel.parent / doc['common']).as_posix()
        rel = Path(os.path.normpath(rel))


def referenced_meshes(data_root, chain):
    """Mesh files the chain references (geometry entries and /geometry/N/mesh patches).

    A relative mesh path is resolved against the declaring file's directory and,
    failing that, the top fixture's directory (PolyFEM's root_path). Both
    coincide for the CI-03 fixtures; a reference that resolves nowhere is reported."""
    top_dir = (data_root / chain[0][0]).parent
    found, missing = {}, []
    for rel, doc in chain:
        here = (data_root / rel).parent
        candidates = []
        geometry = doc.get('geometry', [])
        for entry in geometry if isinstance(geometry, list) else [geometry]:
            if isinstance(entry, dict) and isinstance(entry.get('mesh'), str):
                candidates.append(entry['mesh'])
        for op in doc.get('patch', []):
            if isinstance(op, dict) and MESH_PATCH.match(str(op.get('path', ''))) and isinstance(op.get('value'), str):
                candidates.append(op['value'])
        for mesh in candidates:
            for base in (here, top_dir):
                path = Path(os.path.normpath(base / mesh))
                if path.is_file():
                    found[path.relative_to(data_root).as_posix()] = path
                    break
            else:
                missing.append({'declared_in': rel, 'mesh': mesh})
    return found, missing


def overlay_for(variant):
    """The keys a variant writes into the copied top-level fixture (merge)."""
    base = variant[:-5] if variant.endswith('-diag') else variant
    overlay = {}
    if base == 'default':
        pass
    elif base.startswith('fi'):
        n = -1 if base == 'fiinf' else int(base[2:])
        overlay['solver'] = {'contact': {'friction_iterations': n}}
    else:
        raise ValueError(f'unknown variant {variant}')
    if variant.endswith('-diag'):
        overlay['output'] = {'physical_diagnostics': True, 'manifest': 'run-manifest.json'}
    return overlay


def deep_merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            deep_merge(dst[k], v)
        else:
            dst[k] = v


def prepare(run_dir, data_root, relative, variant):
    chain = resolve_chain(data_root, relative)
    meshes, missing = referenced_meshes(data_root, chain)
    copy_root = run_dir / 'data'
    copied = {}
    for rel, _ in chain:
        dst = copy_root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(data_root / rel, dst)
        copied[rel] = sha256(dst)
    for rel, src in meshes.items():
        dst = copy_root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dst)
        copied[rel] = sha256(dst)
    pinned_hash = copied[relative]
    overlay = overlay_for(variant)
    if overlay:
        top = copy_root / relative
        doc = json.loads(top.read_text())
        deep_merge(doc, overlay)
        top.write_text(json.dumps(doc, indent=4) + '\n')
        copied[relative] = sha256(top)
    (run_dir / 'manifest.txt').write_text(relative + '\n')
    reference = chain[0][1].get('tests', {})
    return {
        'fixture': relative,
        'chain': [rel for rel, _ in chain],
        'meshes': sorted(meshes),
        'unresolved_references': missing,
        'pinned_fixture_sha256': pinned_hash,
        'copied_fixture_sha256': copied[relative],
        'copy_identical_to_pinned': copied[relative] == pinned_hash,
        'overlay': overlay,
        'copied_files_sha256': copied,
        'reference': reference,
    }


def lag_summary(sim_json):
    """Lag re-solves per step from the fixture's own sim.json (`solver_info` `rc` rows)."""
    if not sim_json.is_file():
        return {'unavailable_reason': 'no sim.json'}
    try:
        info = json.loads(sim_json.read_text()).get('solver_info', [])
    except (ValueError, OSError) as e:
        return {'unavailable_reason': f'sim.json unreadable: {e}'}
    rc = [row for row in info if isinstance(row, dict) and row.get('type') == 'rc']
    # One `rc` row per minimize call of the reduced problem: the step's first
    # solve has no `lag_i`; every lag re-solve carries the lag iteration it follows.
    resolves = [row for row in rc if 'lag_i' in row]
    steps = {}
    for row in resolves:
        steps[row.get('t')] = max(steps.get(row.get('t'), 0), int(row['lag_i']))
    return {
        'rc_rows_total': len(rc),
        'first_solves': len(rc) - len(resolves),
        'lag_resolves_total': len(resolves),
        'steps_with_lag_resolve': len(steps),
        'max_lag_i_resolved': max(steps.values()) if steps else 0,
    }


def diagnostics_summary(jsonl):
    if not jsonl.is_file():
        return {'unavailable_reason': 'no physical-diagnostics.jsonl'}
    states, residuals, iterations, dissipation, free_residuals, balance = {}, [], [], [], [], {}
    for line in jsonl.read_text().splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        lag = row.get('lagging', {})
        if isinstance(lag, dict) and 'state' in lag:
            states[lag['state']] = states.get(lag['state'], 0) + 1
            if isinstance(lag.get('updated_lag_residual_norm_objective'), (int, float)):
                residuals.append(lag['updated_lag_residual_norm_objective'])
            if isinstance(lag.get('iteration'), int):
                iterations.append(lag['iteration'])
        d = row.get('frictional_dissipation_increment')
        if isinstance(d, dict):
            d = d.get('value')
        if isinstance(d, (int, float)):
            dissipation.append(d)
        f = row.get('free_residual_norm')
        if isinstance(f, dict):
            f = f.get('value')
        if isinstance(f, (int, float)):
            free_residuals.append(f)
        b = row.get('physical_balance_pass')
        if isinstance(b, dict) and isinstance(b.get('value'), bool):
            balance[b['value']] = balance.get(b['value'], 0) + 1
    return {
        'records': sum(states.values()),
        'lagging_states': states,
        'updated_lag_residual_max': max(residuals) if residuals else None,
        'updated_lag_residual_last': residuals[-1] if residuals else None,
        'final_lag_iteration_max': max(iterations) if iterations else None,
        'frictional_dissipation_increment_min': min(dissipation) if dissipation else None,
        'frictional_dissipation_cumulative': sum(dissipation) if dissipation else None,
        'free_residual_norm_max': max(free_residuals) if free_residuals else None,
        'physical_balance_pass_counts': {str(k): v for k, v in balance.items()},
    }


def run_one(args, label, slug, variant):
    relative = FIXTURES[slug]
    run_dir = args.output / 'runs' / label / slug / variant
    if run_dir.exists():
        shutil.rmtree(run_dir)
    run_dir.mkdir(parents=True)
    record = {'label': label, 'slug': slug, 'variant': variant, 'binary': str(args.unit_tests), 'binary_sha256': sha256(args.unit_tests)}
    record.update(prepare(run_dir, args.data, relative, variant))
    env = dict(os.environ)
    env['POLYFEM_RUN_MANIFEST'] = str(run_dir / 'manifest.txt')
    env['POLYFEM_RUN_DATA_DIR'] = str(run_dir / 'data')
    cmd = [str(args.unit_tests), 'run_manifest_env']
    record['command'] = cmd
    t0 = time.time()
    with open(run_dir / 'run.log', 'w') as log:
        try:
            proc = subprocess.run(cmd, cwd=run_dir, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
            record['exit'] = proc.returncode
        except subprocess.TimeoutExpired:
            record['exit'] = 'timeout'
    record['wall_seconds'] = time.time() - t0
    text = (run_dir / 'run.log').read_text(errors='replace')
    computed = None
    for m in re.finditer(r'Computed tests: (\{.*\})', text):
        computed = json.loads(m.group(1))
    record['computed'] = computed
    record['harness_verdict'] = ('authenticated' if 'Authenticated' in text
                                 else 'violated' if 'Violating Authenticate' in text
                                 else 'no verdict')
    record['harness_violations'] = re.findall(r'Violating Authenticate (\S+)', text)
    record['error_lines'] = len(re.findall(r'\[error\]', text))
    if computed:
        rel = {}
        for key in METRICS:
            prev = record['reference'].get(key)
            if prev is None:
                continue
            rel[key] = abs((computed[key] - prev) / max(abs(prev), 1e-5))
        record['relative_error_vs_reference'] = rel
        record['max_relative_error_vs_reference'] = max(rel.values()) if rel else None
    record['lag'] = lag_summary(run_dir / 'sim.json')
    if variant.endswith('-diag'):
        record['diagnostics'] = diagnostics_summary(run_dir / 'physical-diagnostics.jsonl')
        manifest = run_dir / 'run-manifest.json'
        if manifest.is_file():
            doc = json.loads(manifest.read_text())
            eff = doc.get('input', {}).get('effective', {})
            record['effective_friction_iterations'] = eff.get('solver', {}).get('contact', {}).get('friction_iterations')
            record['effective_barrier_stiffness'] = eff.get('solver', {}).get('contact', {}).get('barrier_stiffness')
            record['effective_friction_coefficient'] = eff.get('contact', {}).get('friction_coefficient')
            # In-library runs never finalize the manifest (PolyFEM_bin does that at exit).
            record['manifest_completion_status'] = doc.get('completion', {}).get('status')
    (run_dir / 'run.json').write_text(json.dumps(record, indent=2) + '\n')
    return record


def fmt(x):
    return '—' if x is None else f'{x:.3e}'


def write_summary(args, records):
    out = args.output
    summary = {'label': args.label, 'unit_tests': str(args.unit_tests), 'unit_tests_sha256': sha256(args.unit_tests), 'runs': records}
    (out / f'summary-{args.label}.json').write_text(json.dumps(summary, indent=2) + '\n')
    lines = [f'# CI-03 A/B — label `{args.label}`', '',
             f'`{args.unit_tests}` sha256 `{summary["unit_tests_sha256"][:16]}…`', '',
             '| fixture | variant | exit | wall s | verdict | max rel err vs reference | err_h1_semi | lag re-solves (steps) | copy = pinned |',
             '| --- | --- | ---: | ---: | --- | ---: | --- | ---: | --- |']
    for r in records:
        lag = r['lag']
        lag_s = f"{lag.get('lag_resolves_total', '—')} ({lag.get('steps_with_lag_resolve', '—')})" if 'lag_resolves_total' in lag else '—'
        h1s = r['computed']['err_h1_semi'] if r.get('computed') else None
        lines.append(f"| {r['slug']} | {r['variant']} | {r['exit']} | {r['wall_seconds']:.0f} | {r['harness_verdict']} | {fmt(r.get('max_relative_error_vs_reference'))} | {h1s!r} | {lag_s} | {r['copy_identical_to_pinned']} |")
    (out / f'summary-{args.label}.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--unit-tests', type=Path, required=True)
    parser.add_argument('--data', type=Path, required=True, help='the pinned polyfem-data checkout (read only)')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--label', required=True, help='run-set name, e.g. before / after')
    parser.add_argument('--variants', default='default,fi1,fi2')
    parser.add_argument('--fixtures', default=','.join(FIXTURES))
    parser.add_argument('--jobs', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=3600)
    args = parser.parse_args()
    args.unit_tests = args.unit_tests.resolve()
    args.data = args.data.resolve()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    jobs = [(slug, v) for slug in args.fixtures.split(',') for v in args.variants.split(',')]
    for slug, _ in jobs:
        if slug not in FIXTURES:
            sys.exit(f'unknown fixture {slug}; known: {", ".join(FIXTURES)}')
    records = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(run_one, args, args.label, slug, v): (slug, v) for slug, v in jobs}
        for fut in concurrent.futures.as_completed(futures):
            r = fut.result()
            print(f"done {r['slug']} {r['variant']}: exit {r['exit']}, {r['harness_verdict']}, max rel err {fmt(r.get('max_relative_error_vs_reference'))}", flush=True)
            records.append(r)
    records.sort(key=lambda r: (list(FIXTURES).index(r['slug']), r['variant']))
    write_summary(args, records)


if __name__ == '__main__':
    main()
