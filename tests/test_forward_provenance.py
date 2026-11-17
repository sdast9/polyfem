"""Forward CLI source identities, independent of resource roots.
Run with existing h5py environment: python test_forward_provenance.py BINARY OUTPUT.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

binary = Path(sys.argv[1]).resolve()
root = Path(sys.argv[2]).resolve()
root.mkdir(exist_ok=False)
assets = root / 'assets'
assets.mkdir()
(assets / 'square.mesh').write_text('MeshVersionFormatted 1\nDimension 3\nVertices\n4\n0 0 0 0\n1 0 0 0\n1 1 0 0\n0 1 0 0\nTriangles\n2\n1 2 3 0\n1 3 4 0\nEnd\n')
config = {
    'root_path': str(assets),
    'geometry': {'mesh': 'square.mesh', 'surface_selection': 1},
    'materials': {'type': 'NeoHookean', 'E': 1000, 'nu': .3, 'rho': 1},
    'boundary_conditions': {'dirichlet_boundary': [{'id': 'all', 'value': [0, 0]}]},
    'time': {'dt': .1, 'time_steps': 2},
    'solver': {'linear': {'solver': 'Eigen::SimplicialLDLT'}},
    'output': {'checkpoint': {'path': 'step_{:d}.h5'}}}
source = root / 'input.json'
source.write_text(json.dumps(config))
yaml = root / 'input.yaml'
yaml.write_text(json.dumps(config))  # JSON is valid YAML; exercise YAML loader.
spec = importlib.util.spec_from_file_location('packager', Path(__file__).resolve().parents[1] / 'tools/package_json_hdf5.py')
packager = importlib.util.module_from_spec(spec)
spec.loader.exec_module(packager)
bundle = root / 'input.h5'
packager.package_scene(source, bundle)
results = []
def run(name, option, path):
    output = root / name
    output.mkdir()
    command = [str(binary), option, str(path), '-o', str(output), '--max_threads', '1', '--log_level', 'error']
    result = subprocess.run(command, cwd=root, capture_output=True, text=True, timeout=180,
                            env=dict(os.environ, OMP_NUM_THREADS='1', MKL_CBWR='COMPATIBLE'))
    (output / 'process.log').write_text(result.stdout + result.stderr)
    assert result.returncode == 0, (name, result.returncode, result.stdout, result.stderr)
    manifest = json.loads((output / 'run-manifest.json').read_text())
    expected = hashlib.sha256(path.read_bytes()).hexdigest()
    assert manifest['input']['file']['sha256'] == expected, manifest['input']['file']
    assert Path(manifest['input']['file']['path']).resolve() == path.resolve()
    assert 'manifest_input_file' not in json.dumps(manifest['input']), manifest['input']
    results.append({'name': name, 'sha256': expected, 'passed': True})
    return output
full = run('json', '--json', source)
run('yaml', '--yaml', yaml)
run('hdf5', '--hdf5', bundle)
checkpoint = full / 'step_1.h5'
run('direct', '--checkpoint', checkpoint)
wrapper = root / 'resume.json'
wrapper.write_text(json.dumps({'input': {'checkpoint': {'path': str(checkpoint), 'reorder': False}},
                              'time': {'dt': 99, 'time_steps': 99}}))
run('wrapper', '--json', wrapper)
(root / 'results.json').write_text(json.dumps(results, indent=2))
print('All five forward provenance routes passed')
