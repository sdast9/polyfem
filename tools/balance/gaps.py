#!/usr/bin/env python3
"""Realized contact gaps per step (last iteration record of each step).

usage: gaps.py RUN [RUN ...]

Quantiles over steps with an active contact of the minimum gap and the band
rms (gap / dhat), from `clamped.excluding_fully` (the controller's view under
the default clamped_contacts), else the record's `gap` block; plus the
number of steps whose minimum gap is below the collapse pair threshold
(0.0707) and below 0.2.
"""
import json, sys
from pathlib import Path

import numpy as np


def step_last(path):
    last = {}
    for line in open(path):
        r = json.loads(line)
        if r.get('event') == 'iteration':
            last[r.get('step')] = r
    return last


def gaps(run):
    last = step_last(Path(run) / 'output/trim-predictors.jsonl')
    mins, bands, trims = [], [], []
    for r in last.values():
        c = (r.get('clamped') or {}).get('excluding_fully') or {}
        m = c.get('min_gap', (r.get('gap') or {}).get('min'))
        b = c.get('band_rms', (r.get('gap') or {}).get('rms'))
        if m is not None:
            mins.append(m)
            bands.append(b if b is not None else np.nan)
            trims.append(r.get('trim'))
    q = lambda v: np.nanquantile(v, [0, .1, .5, .9]).round(3).tolist() if len(v) else None
    return dict(run=Path(run).name, contact_steps=len(mins), min_gap_q0_10_50_90=q(mins), band_rms_q0_10_50_90=q(bands),
                trim_q0_10_50_90=q(trims), steps_below_collapse_pair=int(sum(m < 0.0707 for m in mins)),
                steps_below_0p2=int(sum(m < 0.2 for m in mins)))


if __name__ == '__main__':
    for run in sys.argv[1:]:
        print(json.dumps(gaps(run)))
