"""Read PF-02 scene logs and saved deformation gradients; never launch a scene.

Usage: python3 tools/pf02/summarize_scenes.py /absolute/evidence
Requires NumPy and uncompressed binary VTU output with UInt64 headers.
"""
import json
from pathlib import Path
import re
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vtu_binary import DTYPES, read_arrays

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
        needed = {'F_1', 'F_2', 'F_3', 'connectivity', 'offsets', 'types'}
        for array, payload in read_arrays(path, needed):
            name = array['Name']
            values = np.frombuffer(payload, dtype=DTYPES[array['type']])
            if name.startswith('F_'):
                tensors[name] = values.reshape(-1, 3)
            else:
                cells[name] = values
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
