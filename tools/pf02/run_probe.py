"""Build/run the PF-02 characterization against an existing Makefiles build.

Usage: python3 tools/pf02/run_probe.py --build build --output /absolute/evidence
Requires PolyFEM_bin and unit_tests to have already been built. No scenes run.
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
source = Path(__file__).with_name('contact_floor_probe.cpp').resolve()
obj, binary = out / 'contact_floor_probe.o', out / 'contact_floor_probe'
compile_cmd = [link[0]] + flags['CXX_DEFINES'] + flags['CXX_INCLUDES'] + flags['CXX_FLAGS'] + ['-c', str(source), '-o', str(obj)]
subprocess.run(compile_cmd, cwd=build, check=True)
link = [part for part in link if not part.endswith('.o')]
link[link.index('-o') + 1] = str(binary)
link.insert(1, str(obj))
subprocess.run(link, cwd=build, check=True)
(out / 'build_commands.json').write_text(json.dumps([compile_cmd, link], indent=2) + '\n')
provenance = {
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'executable_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
    'polyfem_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=source.parent, text=True).strip(),
    'build_directory': str(build.parent),
}
(out / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
results = []
for floor in (1e-4, 0):
    for snapshot in (.2, 1.2):
        start = time.monotonic()
        run = subprocess.run([str(binary), str(floor), str(snapshot)], cwd=out, text=True, capture_output=True, check=True)
        result = json.loads(run.stdout)
        result['wall_seconds'] = time.monotonic() - start
        results.append(result)
(out / 'probe-results.json').write_text(json.dumps(results, indent=2) + '\n')
print(json.dumps(results, indent=2))
