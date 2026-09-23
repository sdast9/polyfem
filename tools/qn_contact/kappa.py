#!/usr/bin/env python3
"""Lower bound on the Hessian condition number seen along a solve:
max Newton |H^-1 g|/|g| (<= 1/lambda_min) times max L-BFGS secant scale
y.y/s.y (<= lambda_max of the path-averaged Hessian)."""
import json, sys
def acc(run):
    return [json.loads(l) for l in open(run if run.endswith('.jsonl') else f'runs/{run}/output/solver-attempts.jsonl') if '"accepted"' in l]
def dig(d,*k):
    for x in k:
        if not isinstance(d,dict): return None
        d=d.get(x)
    return d
newton, lbfgs = sys.argv[1], sys.argv[2]
nr=[dig(r,'solver','direction','norm_over_gradient_norm') for r in acc(newton)]
nr=[x for x in nr if isinstance(x,(int,float)) and x>0]
th=[dig(r,'solver','strategy_state','hessian_initial_scale') for r in acc(lbfgs)]
th=[x for x in th if isinstance(x,(int,float)) and x>0 and x!=1.0]
import statistics as st
print(f'Newton |p|/|g|: min {min(nr):.2e} median {st.median(nr):.2e} max {max(nr):.2e}  (n={len(nr)})')
print(f'L-BFGS theta=y.y/s.y: min {min(th):.2e} median {st.median(th):.2e} max {max(th):.2e}  (n={len(th)})')
print(f'kappa lower bound  >= max(theta)*max(Newton ratio) = {max(th)*max(nr):.2e}')
