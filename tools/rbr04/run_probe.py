"""Build/run the RBR-04 seam characterization against an existing Makefiles build.

Usage: python3 tools/rbr04/run_probe.py --build build --output /absolute/evidence [--expect-guard]
Requires unit_tests to have already been built (its compile/link recipe is
reused). No scenes run. Without --expect-guard the probe is the failing
control: the improved-max/semi-implicit configurations must construct and
show the finite seam jump. With --expect-guard they must be refused by name
and every control must keep its behaviour.
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
parser.add_argument('--expect-guard', action='store_true')
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
source = Path(__file__).with_name('seam_probe.cpp').resolve()
(out / source.name).write_bytes(source.read_bytes())
obj, binary = out / 'seam_probe.o', out / 'seam_probe'
compile_cmd = [link[0]] + flags['CXX_DEFINES'] + flags['CXX_INCLUDES'] + flags['CXX_FLAGS'] + ['-c', str(source), '-o', str(obj)]
subprocess.run(compile_cmd, cwd=build, check=True)
link = [part for part in link if not part.endswith('.o')]
link[link.index('-o') + 1] = str(binary)
link.insert(1, str(obj))
subprocess.run(link, cwd=build, check=True)
(out / 'build_commands.json').write_text(json.dumps([compile_cmd, link], indent=2) + '\n')
polyfem = source.parents[2]
provenance = {
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'executable_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
    'polyfem_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=polyfem, text=True).strip(),
    'polyfem_dirty': subprocess.check_output(['git', 'status', '--porcelain'], cwd=polyfem, text=True).splitlines(),
    'build_directory': str(build.parent),
    'expect_guard': args.expect_guard,
}
(out / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
command = [str(binary)] + (['--expect-guard'] if args.expect_guard else [])
run = subprocess.run(command, cwd=out, text=True, capture_output=True)
(out / 'probe-results.json').write_text(run.stdout)
(out / 'probe.stderr').write_text(run.stderr)
(out / 'exit.json').write_text(json.dumps({'command': command, 'probe_exit': run.returncode}) + '\n')
try:
    results = json.loads(run.stdout)
    for config in results.get('configurations', []):
        line = f"{config['name']}: constructed={config.get('constructed')}"
        if config.get('constructed'):
            for seam in ('seam_A', 'seam_B'):
                s = config[seam]
                line += f"  {seam}: {s['classification']} jump={s['measured_jump_at_smallest_offset']:.6g} predicted={s['predicted_limit_jump']:.6g}"
        else:
            line += f"  refused: {config.get('refusal')}"
        print(line)
    print(f"passed={results.get('passed')} checks={results.get('checks')} error={results.get('error')}")
except json.JSONDecodeError:
    print(run.stdout)
    print(run.stderr)
raise SystemExit(run.returncode)
