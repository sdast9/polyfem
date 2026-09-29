#!/usr/bin/env python3
"""Clamped-contact shares and controller-statistic shifts from trim-predictors.jsonl.

usage: reduce.py RUN_DIR [RUN_DIR ...] [--json OUT]

Reads the observational `clamped` block of every trim-predictor record
(docs/clamped-contacts-20260928.md): collision classes by stencil vertices
(free / partly / fully clamped), and the controller statistics over all
collisions and without the fully clamped ones. A run without the block is
reported as missing, never as zero.
"""
import argparse, json, math, statistics
from collections import Counter
from pathlib import Path


def rows(path):
    with open(path) as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def rel(a, b):
    if a is None or b is None or not math.isfinite(a) or not math.isfinite(b) or b == 0:
        return None
    return a / b - 1


def dist(values):
    v = sorted(x for x in values if x is not None)
    if not v:
        return None
    return dict(n=len(v), min=v[0], p50=v[len(v) // 2], max=v[-1], max_abs=max(abs(x) for x in v))


def reduce_run(run):
    path = Path(run) / 'output/trim-predictors.jsonl'
    out = dict(run=Path(run).name, records=0, with_block=0)
    if not path.exists():
        out['missing'] = 'trim-predictors.jsonl'
        return out
    events = Counter()
    active = {'free': [], 'partly': [], 'fully': []}
    in_set_fully = []
    fully_share = []
    min_pair = Counter()
    band_shift, min_shift, fw_shift, proxy_shift = [], [], [], []
    collapse_flip = Counter()
    batch_shift, keys_in_batch, gb_excl, gb_free, gate_flip_excl, gate_flip_free = [], [], [], [], Counter(), Counter()
    only_fully = Counter()
    steps_with_fully = set()
    clamped_vertices = None
    for r in rows(path):
        out['records'] += 1
        c = r.get('clamped')
        if c is None:
            continue
        out['with_block'] += 1
        ev = r.get('event')
        events[ev] += 1
        clamped_vertices = c.get('clamped_vertex_count')
        a = c['active']
        for k in active:
            active[k].append(a[k])
        in_set_fully.append(c['collisions']['fully'])
        n = a['free'] + a['partly'] + a['fully']
        if n:
            fully_share.append(a['fully'] / n)
        if a['fully'] > 0:
            steps_with_fully.add(r.get('step'))
            if a['free'] + a['partly'] == 0:
                only_fully[ev] += 1
        if c.get('min_pair_class') is not None:
            min_pair[c['min_pair_class']] += 1
        al, ex = c['all'], c['excluding_fully']
        if a['fully'] > 0:
            band_shift.append(rel(ex['band_rms'], al['band_rms']))
            min_shift.append(rel(ex['min_gap'], al['min_gap']))
            fw_shift.append(rel(ex['force_weighted_mean_gap'], al['force_weighted_mean_gap']))
            proxy_shift.append(rel(ex['collapse_proxy_gap'], al['collapse_proxy_gap']))
        if al['collapse'] != ex['collapse']:
            collapse_flip[ev] += 1
        b = c.get('batch')
        if b:
            keys_in_batch.append(b['fully_keys_in_batch'])
            if b['fully_keys_in_batch']:
                batch_shift.append(rel(b['excluding_fully']['median'], b['all']['median']))
        g = c.get('gradient_balance')
        if g:
            ka = g['all'].get('kappa_gb')
            gb_excl.append(rel(g['excluding_fully'].get('kappa_gb'), ka))
            if 'free_dofs' in g:
                gb_free.append(rel(g['free_dofs'].get('kappa_gb'), ka))
                if g['all'].get('gate_passes') != g['free_dofs'].get('gate_passes'):
                    gate_flip_free[ev] += 1
            if g['all'].get('gate_passes') != g['excluding_fully'].get('gate_passes'):
                gate_flip_excl[ev] += 1
    if not out['with_block']:
        out['missing'] = 'clamped block'
        return out
    out.update(
        events=dict(events), clamped_vertex_count=clamped_vertices,
        active={k: dict(max=max(v), p50=statistics.median(v), records_nonzero=sum(1 for x in v if x)) for k, v in active.items()},
        collisions_fully_max=max(in_set_fully),
        fully_share_of_active=dist(fully_share),
        steps_with_active_fully=sorted(s for s in steps_with_fully if s is not None),
        only_fully_active_records=dict(only_fully),
        min_pair_class={str(k): v for k, v in sorted(min_pair.items())},
        shift_when_fully_active=dict(band_rms=dist(band_shift), min_gap=dist(min_shift),
                                     force_weighted_mean_gap=dist(fw_shift), collapse_proxy_gap=dist(proxy_shift)),
        collapse_decision_flips=dict(collapse_flip),
        batch=dict(records=len(keys_in_batch), max_fully_keys=max(keys_in_batch) if keys_in_batch else None,
                   median_shift=dist(batch_shift)),
        gradient_balance=dict(kappa_gb_excluding_fully=dist(gb_excl), kappa_gb_free_dofs=dist(gb_free),
                              gate_flips_excluding_fully=dict(gate_flip_excl), gate_flips_free_dofs=dict(gate_flip_free)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--json')
    a = ap.parse_args()
    result = [reduce_run(r) for r in a.runs]
    text = json.dumps(result, indent=1)
    if a.json:
        Path(a.json).write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
