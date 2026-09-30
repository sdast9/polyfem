#!/usr/bin/env python3
"""Run parallel-edge through the scene harness under a list of variants and
report the six harness metrics' largest relative deviation from the stored
reference (the harness's own normalisation), plus limit hits.

usage: harness_sweep.py <unit_tests> <data root> <outdir> <label> <variant> [<variant> ...]
variant = "ulp:K"            dynamic tet y translation 0.5 + K*ulp(0.5)
        | "E:K" / "rho:K"    Young's modulus / density plus K ulps
        | "env:NAME=VALUE[,NAME=VALUE]"   scene unchanged, environment set
"""
import sys, os, json, math, re, shutil, subprocess
from concurrent.futures import ThreadPoolExecutor

ut, root, out, label = sys.argv[1:5]
variants = sys.argv[5:]
KEYS = ["err_l2", "err_h1", "err_h1_semi", "err_linf", "err_linf_grad", "err_lp"]
src = os.path.join(root, "gcp-contact/parallel-edge/run.json")
base = json.load(open(src))
ref = base["tests"]

def run(v):
    name = re.sub(r"[^A-Za-z0-9]+", "_", v)
    d = os.path.join(out, f"{label}-{name}")
    os.makedirs(os.path.join(d, "data/gcp-contact/parallel-edge"), exist_ok=True)
    link = os.path.join(d, "data/contact")
    if not os.path.exists(link):
        os.symlink(os.path.realpath(os.path.join(root, "contact")), link)
    shutil.copy(os.path.join(root, "gcp-contact/common.json"), os.path.join(d, "data/gcp-contact/common.json"))
    cfg = json.loads(json.dumps(base))
    env = dict(os.environ, OMP_NUM_THREADS="1")
    kind, _, arg = v.partition(":")
    if kind == "ulp":
        cfg["geometry"][0]["transformation"]["translation"][1] = 0.5 + int(arg) * math.ulp(0.5)
    elif kind == "E":
        cfg["materials"][0]["E"] = 1e5 + int(arg) * math.ulp(1e5)
    elif kind == "rho":
        cfg["materials"][0]["rho"] = 1000 + int(arg) * math.ulp(1000.0)
    elif kind == "env":
        for kv in arg.split(","):
            k, _, val = kv.partition("=")
            env[k] = val
    json.dump(cfg, open(os.path.join(d, "data/gcp-contact/parallel-edge/run.json"), "w"))
    open(os.path.join(d, "manifest.txt"), "w").write("gcp-contact/parallel-edge/run.json\n")
    env["POLYFEM_RUN_MANIFEST"] = os.path.join(d, "manifest.txt")
    env["POLYFEM_RUN_DATA_DIR"] = os.path.join(d, "data")
    with open(os.path.join(d, "harness.log"), "w") as log:
        subprocess.run([ut, "run_manifest_env"], cwd=d, stdout=log, stderr=subprocess.STDOUT, env=env)
    txt = open(os.path.join(d, "harness.log"), errors="replace").read()
    m = re.search(r"Computed tests: (\{.*\})", txt)
    hits = txt.count("Reached iteration limit")
    if not m:
        return v, None, hits
    cur = json.loads(m.group(1))
    dev = max(abs(cur[k] - ref[k]) / max(abs(ref[k]), 1e-5) for k in KEYS)
    return v, dev, hits

with ThreadPoolExecutor(int(os.environ.get("JOBS", "2"))) as ex:
    for v, dev, hits in ex.map(run, variants):
        print(f"{label} {v:40s} max_rel_dev={'n/a' if dev is None else f'{dev:.3e}'} limit_hits={hits}", flush=True)
