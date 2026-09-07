"""Read PF-02 scene logs and saved deformation gradients; never launch a scene.

Usage: python3 tools/pf02/summarize_scenes.py /absolute/evidence
Requires NumPy; supports uncompressed inline-binary VTU output (UInt64 headers).
"""
import base64
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

root = Path(sys.argv[1]).resolve()
summaries = []
for record in json.loads((root / 'scene-results.json').read_text()):
    run = root / record['scene'] / record['mode']
    log = (run / 'run.log').read_text()
    item = dict(record)
    distances = [(float(a), float(b)) for a, b in re.findall(r'Minimum distance during solve: ([\deE.+-]+), dhat: ([\deE.+-]+)', log)]
    item['min_logged_gap'] = min((a for a, b in distances), default=None)
    item['min_logged_gap_over_dhat'] = min((a / b for a, b in distances), default=None)
    item['floor_projection_log_count'] = log.count('Constraint floor: projected')
    item['restart_log_count'] = log.count('retuning barrier stiffness and restarting')
    item['configured_slope_exit_log_count'] = log.count('Finished: Search direction not a descent direction')
    item['final_reduced_failure_count'] = log.count('Final reduced solve did not converge')
    steps = re.findall(r'\[info\] (\d+)/(\d+)  t=([^\s]+)', log)
    item['last_completed_step'] = list(steps[-1]) if steps else None
    finishes = [line for line in log.splitlines() if 'Finished:' in line]
    item['last_solver_finish'] = finishes[-1] if finishes else None
    samples = []
    for path in sorted((run / 'output').glob('step_*.vtu'), key=lambda p: int(p.stem.split('_')[-1])):
        tensors = {}
        cells = {}
        for event, node in ET.iterparse(path, events=('start', 'end')):
            if event == 'start' and node.tag == 'VTKFile':
                if node.get('header_type') != 'UInt64' or node.get('compressor'):
                    raise ValueError(f'Unsupported VTU encoding: {path}')
            if event != 'end' or node.tag != 'DataArray':
                continue
            name = node.get('Name')
            if name in ('F_1', 'F_2', 'F_3', 'connectivity', 'offsets', 'types'):
                dtypes = {'Float64': '<f8', 'Int64': '<i8', 'UInt8': 'u1'}
                if node.get('format') != 'binary' or node.get('type') not in dtypes:
                    raise ValueError(f'Unsupported array encoding: {path}')
                payload = base64.b64decode(node.text.strip())
                size = int.from_bytes(payload[:8], 'little')
                if size != len(payload) - 8:
                    raise ValueError(f'Invalid array length: {path}')
                values = np.frombuffer(payload, dtype=dtypes[node.get('type')], offset=8)
                if name.startswith('F_'):
                    tensors[name] = values.reshape(-1, 3)
                else:
                    cells[name] = values
            node.clear()
        if tensors:
            deformation = np.stack([tensors[f'F_{i}'] for i in (1, 2, 3)], axis=1)
            determinant = np.linalg.det(deformation)
            # These fixtures contain tetrahedral FEM cells and optional rigid
            # obstacle triangles. Select by topology, never by determinant sign.
            if not set(cells['types']).issubset({5, 10}):
                raise ValueError(f'Only triangle/tetrahedron topology is supported: {path}')
            sizes = np.diff(np.r_[0, cells['offsets']])
            volume_ids = np.unique(cells['connectivity'][np.repeat(cells['types'] == 10, sizes)])
            if volume_ids.size == 0:
                raise ValueError(f'No tetrahedral volume samples: {path}')
            volume_determinant = determinant[volume_ids]
            samples.append(dict(file=path.name, min_det_F=float(determinant.min()), max_det_F=float(determinant.max()),
                                nonpositive=int(np.count_nonzero(determinant <= 0)), nonfinite=int(np.count_nonzero(~np.isfinite(determinant))),
                                min_tet_det_F=float(volume_determinant.min()),
                                nonpositive_tet=int(np.count_nonzero(volume_determinant <= 0)),
                                nonfinite_tet=int(np.count_nonzero(~np.isfinite(volume_determinant))),
                                nonvolume_samples=len(determinant)-len(volume_ids)))
    item['saved_F_frames_checked'] = len(samples)
    item['min_saved_det_F'] = min((s['min_det_F'] for s in samples), default=None)
    item['nonpositive_saved_F_samples'] = sum(s['nonpositive'] for s in samples)
    item['nonfinite_saved_F_samples'] = sum(s['nonfinite'] for s in samples)
    item['min_saved_tet_det_F'] = min((s['min_tet_det_F'] for s in samples), default=None)
    item['nonpositive_saved_tet_F_samples'] = sum(s['nonpositive_tet'] for s in samples)
    item['nonfinite_saved_tet_F_samples'] = sum(s['nonfinite_tet'] for s in samples)
    item['saved_nonvolume_F_samples'] = sum(s['nonvolume_samples'] for s in samples)
    (run / 'saved-F-check.json').write_text(json.dumps(samples, indent=2) + '\n')
    summaries.append(item)
    print(json.dumps(item), flush=True)
(root / 'scene-summary.json').write_text(json.dumps(summaries, indent=2) + '\n')
