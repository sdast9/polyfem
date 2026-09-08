"""Build/run the PF-08 physical probe against an existing Makefiles build.

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
source = Path(__file__).with_name('physical_probe.cpp').resolve()
obj, binary = out / 'physical_probe.o', out / 'physical_probe'
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
for L in (1e-3, 1, 1e3):
    for S in (1e-2, 1, 1e2):
        for coupled in (0, 1):
            for moving in (0, 1):
                for steps in (2, 4, 8):
                    cmd = [str(binary), str(L), str(S), str(steps), str(moving), str(coupled), "0.0001"]
                    run = subprocess.run(cmd, cwd=out, text=True, capture_output=True)
                    name = f"L{L}-S{S}-c{coupled}-m{moving}-n{steps}"
                    (out / (name + ".stdout")).write_text(run.stdout)
                    (out / (name + ".stderr")).write_text(run.stderr)
                    try:
                        result = json.loads(run.stdout)
                    except ValueError:
                        result = {"error": run.stderr, "passed": False}
                    result.update(command=cmd, exit_code=run.returncode, length_scale=L,
                                  objective_scale=S, steps=steps, moving=bool(moving),
                                  coupled=bool(coupled))
                    results.append(result)
                    print(name, run.returncode, flush=True)
                    (out / 'probe-results.json').write_text(json.dumps(results, indent=2) + '\n')
raise SystemExit(0 if all(r.get("passed") and r["exit_code"] == 0 for r in results) else 1)
