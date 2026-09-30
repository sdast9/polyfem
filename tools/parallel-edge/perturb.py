#!/usr/bin/env python3
"""Run the parallel-edge scene under 1-ulp geometric perturbations.

usage: perturb.py <PolyFEM_bin> <base data root (holding gcp-contact/parallel-edge/run.json)> <outdir> <label> [n=16] [jobs=2]
The dynamic tet's y translation is 0.5 + k*ulp(0.5) for k = 0..n-1. Prints one
step_metrics summary per k and a final line with the count of runs that hit the
Newton iteration limit and the distribution of the worst step's iterations.
"""
import sys, os, json, math, shutil, subprocess
from concurrent.futures import ThreadPoolExecutor
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
import step_metrics

binary, root, out, label = sys.argv[1:5]
n = int(sys.argv[5]) if len(sys.argv) > 5 else 16
jobs = int(sys.argv[6]) if len(sys.argv) > 6 else 2
base = json.load(open(os.path.join(root, "gcp-contact/parallel-edge/run.json")))
ulp = math.ulp(0.5)

def one(k):
    d = os.path.join(out, f"{label}-k{k}")
    os.makedirs(os.path.join(d, "gcp-contact/parallel-edge"), exist_ok=True)
    for name, target in (("contact", os.path.join(root, "contact")),):
        link = os.path.join(d, name)
        if not os.path.exists(link):
            os.symlink(os.path.realpath(target), link)
    shutil.copy(os.path.join(root, "gcp-contact/common.json"), os.path.join(d, "gcp-contact/common.json"))
    cfg = json.loads(json.dumps(base))
    cfg["geometry"][0]["transformation"]["translation"][1] = 0.5 + k * ulp
    scene = os.path.join(d, "gcp-contact/parallel-edge/run.json")
    json.dump(cfg, open(scene, "w"), indent=1)
    with open(os.path.join(d, "run.log"), "w") as log:
        subprocess.run([binary, "-j", scene, "-o", d, "--max_threads", "1", "--log_level", "debug"],
                       stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ, OMP_NUM_THREADS="1"))
    rows, hits = step_metrics.parse(os.path.join(d, "run.log"))
    its = [r[1] for r in rows]
    return k, len(rows), sum(its), max(its, default=0), hits

with ThreadPoolExecutor(jobs) as ex:
    res = list(ex.map(one, range(n)))
for k, solves, tot, mx, hits in res:
    print(f"{label} k={k:3d} solves={solves} total={tot} max={mx} limit_hits={hits}")
print(f"{label} SUMMARY runs={len(res)} with_limit_hit={sum(1 for r in res if r[4] or r[3] >= 500)} "
      f"complete={sum(1 for r in res if r[1] == 60)} max_iters={sorted(r[3] for r in res)}")
