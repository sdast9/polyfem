#!/usr/bin/env python3
"""Track the spike's tip in spike-in-crack runs (docs/esp-assessment-20261025.md, section 3).

usage: tip_track.py <run dir> [<run dir> ...]   (needs numpy and meshio)

A run dir is one run_matrix.py run (it holds out/step_*.vtu). The deformable body
is body_ids == 1; the tip is its vertex with the smallest rest y, the base its
vertices with rest y > 1.9. Prints, every 20 steps and at the last step, the tip
position, the base's mean x and the tip-to-base distance (rest length 3.0).
"""
import glob, re, sys
import numpy as np
import meshio


def step(f):
    return int(re.search(r"step_(\d+)\.vtu", f).group(1))


for d in sys.argv[1:]:
    files = sorted(glob.glob(f"{d}/out/step_*.vtu"), key=step)
    picks = files[::20] + ([files[-1]] if (len(files) - 1) % 20 else [])
    print(d)
    for f in picks:
        m = meshio.read(f)
        body = m.point_data["body_ids"].ravel() == 1
        rest = m.points[body]
        x = rest + m.point_data["solution"][body]
        tip, base = np.argmin(rest[:, 1]), rest[:, 1] > 1.9
        stretch = np.linalg.norm(x[tip] - x[base].mean(axis=0))
        print(f"  step {step(f):3d}: tip ({x[tip, 0]:+.3f}, {x[tip, 1]:+.3f})  base x {x[base, 0].mean():.3f}  tip-base {stretch:.3f}")
