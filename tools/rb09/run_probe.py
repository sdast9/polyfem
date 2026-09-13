"""Build and run the RB-09 spring probe against an existing build.

Usage: python3 tools/rb09/run_probe.py --build build --output /absolute/fresh/dir [--source tools/rb09/spring_probe.cpp]

Compiles the probe source (default tools/rb09/spring_probe.cpp) with the unit_tests flags/link line of
the Makefiles build (the effective IPC toolkit / PolySolve), runs it once and
writes probe.json (its measurements) plus provenance.json.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--build', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--source', type=Path, default=Path(__file__).with_name('spring_probe.cpp'))
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
source = args.source.resolve()
obj, binary = out / (source.stem + '.o'), out / source.stem
compile_cmd = [link[0]] + flags['CXX_DEFINES'] + flags['CXX_INCLUDES'] + flags['CXX_FLAGS'] + ['-c', str(source), '-o', str(obj)]
subprocess.run(compile_cmd, cwd=build, check=True)
link = [part for part in link if not part.endswith('.o')]
link[link.index('-o') + 1] = str(binary)
link.insert(1, str(obj))
subprocess.run(link, cwd=build, check=True)
(out / 'build_commands.json').write_text(json.dumps([compile_cmd, link], indent=2) + '\n')

cache = (build.parent / 'CMakeCache.txt').read_text().splitlines()
def cache_value(prefix):
    return next((l.split('=', 1)[1] for l in cache if l.startswith(prefix)), None)
provenance = {
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'executable_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
    'polyfem_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=source.parent, text=True).strip(),
    'build_directory': str(build.parent),
    'ipc_toolkit_source': cache_value('IPCToolkit_SOURCE_DIR:'),
    'polysolve_source': cache_value('PolySolve_SOURCE_DIR:'),
}
for name in ('ipc_toolkit', 'polysolve'):
    try:
        provenance[f'{name}_head'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=provenance[f'{name}_source'], text=True).strip()
    except Exception as error:  # noqa: BLE001 - provenance only
        provenance[f'{name}_head'] = f'unavailable: {error}'
(out / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')

run = subprocess.run([str(binary)], cwd=out, capture_output=True, text=True)
(out / 'probe.stdout').write_text(run.stdout)
(out / 'probe.stderr').write_text(run.stderr)
try:
    result = json.loads(run.stdout)
except json.JSONDecodeError:
    result = {'passed': False, 'error': 'non-JSON output', 'stdout_tail': run.stdout[-2000:], 'stderr_tail': run.stderr[-2000:]}
result['exit_status'] = run.returncode
(out / 'probe.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({'exit_status': run.returncode, 'passed': result.get('passed'), 'checks': result.get('checks'), 'error': result.get('error')}, indent=2))
