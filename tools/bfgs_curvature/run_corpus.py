"""Build and run the BFGS curvature corpus against an existing PolyFEM build.

Compiles the corpus and the safeguarded strategies against the configured
headers, links them ahead of the libraries the build already produced, and
records the commands, source/library/executable hashes and the PolySolve
revision. The output directory must not exist yet.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=Path('build'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    build = args.build.resolve()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)

    flags = {}
    for line in (build/'_deps/polysolve-build/CMakeFiles/polysolve.dir/flags.make').read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1)
            flags[key] = shlex.split(value)
    cache = {}
    for line in (build/'CMakeCache.txt').read_text().splitlines():
        if '=' in line and not line.startswith(('#', '//')):
            key, value = line.split('=', 1)
            cache[key.split(':', 1)[0]] = value
    polysolve = Path(cache['CPM_PACKAGE_polysolve_SOURCE_DIR'])

    sources = [Path(__file__).with_name('corpus.cpp').resolve()]
    sources += [polysolve/'src/polysolve/nonlinear/descent_strategies'/p for p in
                ('CurvatureGuard.cpp', 'LBFGS.cpp', 'BFGS.cpp')]
    needed = {
        'libpolysolve.a', 'libpolysolve_linear.a', 'liblinear_spec.a',
        'libnon_linear_spec.a', 'libHYPRE.a', 'libumfpack.a', 'libspqr.a',
        'libcholmod.a', 'libamd.a', 'libcamd.a', 'libccolamd.a', 'libcolamd.a',
        'libsuitesparseconfig.a', 'libspdlog.a', 'libjse.a', 'libfinitediff_finitediff.a'}
    link = shlex.split((build/'tests/CMakeFiles/unit_tests.dir/link.txt').read_text())
    libs = list(dict.fromkeys((build/'tests'/p).resolve() for p in link if Path(p).name in needed))

    executable = out/'corpus'
    cmd = ['/usr/bin/c++', '-std=c++17', '-O2', '-DNDEBUG',
           *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'], *map(str, sources),
           *map(str, libs), '-framework', 'Accelerate', '-o', str(executable)]
    record = {
        'build_command': cmd,
        'run_command': [str(executable)],
        'source_sha256': {str(p): sha(p) for p in sources},
        'library_sha256': {str(p): sha(p) for p in libs},
        'polysolve_commit': subprocess.check_output(['git', '-C', str(polysolve), 'rev-parse', 'HEAD'], text=True).strip(),
        'polysolve_status': subprocess.check_output(['git', '-C', str(polysolve), 'status', '--short'], text=True),
    }
    record_file = out/'record.json'
    record_file.write_text(json.dumps(record, indent=2)+'\n')
    with (out/'build.log').open('w') as log:
        result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
    record['build_exit_code'] = result.returncode
    record_file.write_text(json.dumps(record, indent=2)+'\n')
    if result.returncode:
        raise SystemExit(f'Build failed; see {out / "build.log"}')
    record['executable_sha256'] = sha(executable)
    with (out/'results.jsonl').open('w') as log:
        try:
            result = subprocess.run([str(executable)], stdout=log, stderr=subprocess.STDOUT, timeout=900)
            record['run_exit_code'] = result.returncode
        except subprocess.TimeoutExpired:
            record['run_exit_code'] = None
            record['timeout_seconds'] = 900
    record_file.write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps({'record': str(record_file), 'run_exit_code': record['run_exit_code']}))
    if record['run_exit_code'] != 0:
        raise SystemExit('Corpus did not complete; execution evidence was retained.')


if __name__ == '__main__':
    main()
