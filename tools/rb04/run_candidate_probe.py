"""Build/run the RB-04 fixed-coefficient candidate fixture against an existing Makefiles build.

Usage: python3 tools/rb04/run_candidate_probe.py --build build --output /absolute/evidence
Requires PolyFEM_bin and unit_tests to have already been built. No scenes run.
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
parser.add_argument('--source', type=Path, help='Alternate standalone probe source')
args = parser.parse_args()
build = args.build.resolve() / 'tests'
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=False)  # Preserve every earlier evidence run.
flags = {}
for line in (build / 'CMakeFiles/unit_tests.dir/flags.make').read_text().splitlines():
    if ' = ' in line:
        key, value = line.split(' = ', 1)
        flags[key] = shlex.split(value)
link = shlex.split((build / 'CMakeFiles/unit_tests.dir/link.txt').read_text())
source = (args.source or Path(__file__).with_name('candidate_probe.cpp')).resolve()
(out / source.name).write_bytes(source.read_bytes())
obj, binary = out / 'candidate_probe.o', out / 'candidate_probe'
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
run = subprocess.run([str(binary)], cwd=out, text=True, capture_output=True)
(out / 'probe-results.json').write_text(run.stdout)
(out / 'probe.stderr').write_text(run.stderr)
(out / 'exit.json').write_text(json.dumps({'probe_exit': run.returncode}) + '\n')
print(run.stdout)
raise SystemExit(run.returncode)
