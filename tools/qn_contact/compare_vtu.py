#!/usr/bin/env python3
"""Compare the 'solution' point field of two runs step by step.

usage: compare_vtu.py RUN_A RUN_REF   (run directories under runs/)
Reads uncompressed binary output/step_K.vtu with UInt64 headers. Prints
the relative L2 error and maximum absolute error and reference per step.
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vtu_binary import DTYPES, read_arrays


def read_field(path, name):
    for array, payload in read_arrays(path, {name}):
        ncomp = int(array.get('NumberOfComponents', 1))
        data = np.frombuffer(payload, dtype=DTYPES[array['type']])
        return data.reshape(-1, ncomp)
    raise KeyError(f'{name} not in {path}')


def main():
    a, r = Path(sys.argv[1]), Path(sys.argv[2])
    k = 1
    while True:
        fa, fr = a / f'output/step_{k}.vtu', r / f'output/step_{k}.vtu'
        if not (fa.exists() and fr.exists()):
            break
        ua, ur = read_field(fa, 'solution'), read_field(fr, 'solution')
        if ua.shape != ur.shape:
            print(k, 'shape mismatch', ua.shape, ur.shape)
            break
        d = ua - ur
        print(f'step {k}: rel L2 {np.linalg.norm(d) / max(np.linalg.norm(ur), 1e-300):.3e}  '
              f'max|d| {np.abs(d).max():.3e}  max|u_ref| {np.abs(ur).max():.3e}')
        k += 1


if __name__ == '__main__':
    main()
