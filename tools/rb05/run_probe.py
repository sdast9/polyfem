"""Build and run the RB-05 broad-phase resource probe against an existing build.

Usage: python3 tools/rb05/run_probe.py --build build --output /absolute/fresh/dir
       [--max-items 2e8] [--configs default|quick] [--extra "--flag ..."]

Compiles tools/rb05/broad_phase_probe.cpp with the unit_tests flags/link line of
the Makefiles build (the effective IPC toolkit), then runs each configuration in
its own process (per-process maximum RSS) and writes results.json + summary.md.
Configurations whose estimated hash-grid items exceed --max-items are reported
with the estimate only: the probe never builds them (bounded by construction).
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--build', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--max-items', type=float, default=2e8)
parser.add_argument('--configs', default='default')
parser.add_argument('--extra', default='', help='extra probe arguments appended to every run')
parser.add_argument('--timeout', type=float, default=900)
args = parser.parse_args()
build = args.build.resolve() / 'tests'
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)

flags = {}
for line in (build / 'CMakeFiles/unit_tests.dir/flags.make').read_text().splitlines():
    if ' = ' in line:
        key, value = line.split(' = ', 1)
        flags[key] = shlex.split(value)
link = shlex.split((build / 'CMakeFiles/unit_tests.dir/link.txt').read_text())
source = Path(__file__).with_name('broad_phase_probe.cpp').resolve()
obj, binary = out / 'broad_phase_probe.o', out / 'broad_phase_probe'
compile_cmd = [link[0]] + flags['CXX_DEFINES'] + flags['CXX_INCLUDES'] + flags['CXX_FLAGS'] + ['-c', str(source), '-o', str(obj)]
subprocess.run(compile_cmd, cwd=build, check=True)
link = [part for part in link if not part.endswith('.o')]
link[link.index('-o') + 1] = str(binary)
link.insert(1, str(obj))
subprocess.run(link, cwd=build, check=True)
(out / 'build_commands.json').write_text(json.dumps([compile_cmd, link], indent=2) + '\n')

toolkit = build.parent / '_deps' / 'ipc-toolkit-src'
provenance = {
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'executable_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
    'polyfem_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=source.parent, text=True).strip(),
    'build_directory': str(build.parent),
    'ipc_toolkit_source': next((l.split('=', 1)[1] for l in (build.parent / 'CMakeCache.txt').read_text().splitlines() if l.startswith('IPCToolkit_SOURCE_DIR:')), None),
}
try:
    provenance['ipc_toolkit_head'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=provenance['ipc_toolkit_source'], text=True).strip()
except Exception as error:  # noqa: BLE001 - provenance only
    provenance['ipc_toolkit_head'] = f'unavailable: {error}'

n = 20
configs = []
if args.configs in ('default', 'quick'):
    disps = [0, .05, .5, 5, 20] if args.configs == 'default' else [0, .5, 5]
    for disp in disps:
        configs.append({'name': f'uniform-disp{disp:g}', 'args': ['--n', str(n), '--uniform', '--disp', str(disp)]})
    for disp in disps:
        configs.append({'name': f'one-mover-disp{disp:g}', 'args': ['--n', str(n), '--movers', '1', '--disp', str(disp)]})
    for disp in ([.5, 5, 20] if args.configs == 'default' else [5]):
        configs.append({'name': f'two-spread-movers-disp{disp:g}', 'args': ['--n', str(n), '--movers', '2', '--spread', '--disp', str(disp)]})
    # Growth law beyond what is built: estimates only.
    for disp in [50, 200, 1000, 10000]:
        configs.append({'name': f'one-mover-disp{disp:g}-estimate', 'args': ['--n', str(n), '--movers', '1', '--disp', str(disp), '--estimate-only']})
    for disp in [50, 200, 1000]:
        configs.append({'name': f'uniform-disp{disp:g}-estimate', 'args': ['--n', str(n), '--uniform', '--disp', str(disp), '--estimate-only']})
    if args.configs == 'default':
        for method in ['brute_force', 'bvh', 'spatial_hash', 'sweep_and_prune']:
            configs.append({'name': f'{method}-one-mover-disp5', 'args': ['--n', str(n), '--movers', '1', '--disp', '5', '--method', method]})
elif args.configs == 'budget':
    # The toolkit budget on the configurations that blew up above: the limit
    # is checked before the items / emissions are allocated, the candidate
    # set is empty after the failure, and a generous limit reproduces the
    # unlimited counts (the toolkit's own emission count is compared with the
    # probe's independent count).
    for name, extra in [('one-mover-disp20', ['--movers', '1', '--disp', '20']), ('two-spread-movers-disp20', ['--movers', '2', '--spread', '--disp', '20']), ('uniform-disp5', ['--uniform', '--disp', '5'])]:
        configs.append({'name': f'{name}-items-limit-1e6', 'args': ['--n', str(n), *extra, '--budget-cell-items', '1000000']})
        configs.append({'name': f'{name}-emissions-limit-1e6', 'args': ['--n', str(n), *extra, '--budget-cell-items', '1000000000', '--budget-emissions', '1000000']})
        configs.append({'name': f'{name}-generous-limits', 'args': ['--n', str(n), *extra, '--budget-cell-items', '1000000000', '--budget-emissions', '1000000000']})
    configs.append({'name': 'brute_force-uniform-disp5-emissions-limit-1e6', 'args': ['--n', str(n), '--uniform', '--disp', '5', '--method', 'brute_force', '--budget-emissions', '1000000']})
    configs.append({'name': 'bvh-one-mover-disp5-budget-refused', 'args': ['--n', str(n), '--movers', '1', '--disp', '5', '--method', 'bvh', '--budget-cell-items', '10']})
else:
    raise SystemExit(f'unknown --configs {args.configs}')

results = []
for config in configs:
    cmd = [str(binary), *config['args'], '--max-items', str(args.max_items), '--json', str(out / f"{config['name']}.json")] + shlex.split(args.extra)
    started = time.time()
    try:
        run = subprocess.run(cmd, cwd=out, text=True, capture_output=True, timeout=args.timeout)
        status = {'exit': run.returncode, 'stderr_tail': run.stderr[-2000:]}
    except subprocess.TimeoutExpired:
        status = {'exit': None, 'stderr_tail': 'timeout'}
    status['wall_seconds'] = time.time() - started
    report_path = out / f"{config['name']}.json"
    report = json.loads(report_path.read_text()) if report_path.exists() else None
    results.append({'name': config['name'], 'command': cmd, 'status': status, 'report': report})
    print(config['name'], status['exit'], (report or {}).get('estimate', {}).get('total_items'), (report or {}).get('measured', {}).get('items', {}).get('total'), (report or {}).get('candidates', {}).get('total'), (report or {}).get('exception', {}).get('type') if report and report.get('exception') else '', flush=True)

(out / 'results.json').write_text(json.dumps({'provenance': provenance, 'max_items': args.max_items, 'results': results}, indent=2) + '\n')

lines = ['| configuration | trial Linf | cell | grid cells | est. items | measured items | est == measured | emissions EE / FV (pre-filter) | candidates | item bytes | max RSS after (MB) | build s | exit | exception |', '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
for r in results:
    rep = r['report'] or {}
    est = rep.get('estimate', {})
    meas = rep.get('measured', {})
    em = meas.get('emissions', {})
    exc = rep.get('exception') or {}
    lines.append('| {name} | {linf:.3g} | {cell:.3g} | {cells:.3g} | {ei} | {mi} | {match} | {ee} / {fv} | {cand} | {bytes:.3g} | {rss} | {secs} | {exit} | {exc} |'.format(
        exc=('%s: %s %s > %s' % (exc.get('type'), exc.get('quantity', ''), exc.get('requested', ''), exc.get('limit', ''))) if exc else ('unsupported' if r['status']['exit'] not in (0,) and not rep else ''),
        name=r['name'], linf=rep.get('trial_linf', float('nan')), cell=est.get('cell_size', float('nan')), cells=est.get('cells', float('nan')),
        ei=est.get('total_items', '—'), mi=meas.get('items', {}).get('total', '—' if not rep.get('built') else '—'),
        match=rep.get('estimate_matches_items', '—'), ee=em.get('edge_edge', '—'), fv=em.get('face_vertex', '—'),
        cand=rep.get('candidates', {}).get('total', '—'), bytes=est.get('item_bytes', float('nan')),
        rss='%.0f' % (rep['max_rss_bytes_after'] / 2**20) if 'max_rss_bytes_after' in rep else '—',
        secs='%.3f' % rep['build_seconds'] if 'build_seconds' in rep else '—', exit=r['status']['exit']))
(out / 'summary.md').write_text('\n'.join(lines) + '\n')
print('\n'.join(lines))
