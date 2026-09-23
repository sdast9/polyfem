#!/usr/bin/env python3
"""Compare the 'solution' point field of two runs step by step.

usage: compare_vtu.py RUN_A RUN_REF   (run directories under runs/)
Reads output/step_K.vtu (PolyFEM binary-appended-free base64 VTU, UInt64
headers, uncompressed). Prints per step: ‖uA-uR‖/‖uR‖, max|uA-uR|, ‖uR‖∞.
"""
import base64, re, sys
from pathlib import Path
import numpy as np

DT = {'Float64': np.float64, 'Float32': np.float32, 'Int64': np.int64,
      'Int32': np.int32, 'UInt8': np.uint8, 'UInt64': np.uint64}


def read_field(path, name):
    text = Path(path).read_text()
    m = re.search(r'<DataArray[^>]*Name="%s"[^>]*>(.*?)</DataArray>' % re.escape(name), text, re.S)
    if not m:
        raise KeyError(f'{name} not in {path}')
    head = m.group(0)[:m.group(0).index('>')]
    dtype = DT[re.search(r'type="(\w+)"', head).group(1)]
    ncomp = int((re.search(r'NumberOfComponents="(\d+)"', head) or [None, 1])[1])
    raw = base64.b64decode(''.join(m.group(1).split()))
    nbytes = np.frombuffer(raw[:8], dtype=np.uint64)[0]
    data = np.frombuffer(raw[8:8 + nbytes], dtype=dtype)
    return data.reshape(-1, ncomp)


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
