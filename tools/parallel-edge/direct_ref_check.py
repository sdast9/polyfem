#!/usr/bin/env python3
"""Run a scene with PolyFEM_bin as the scene harness would (Eigen::SimplicialLDLT,
one thread) and compare the printed error metrics with the scene's stored
reference, using the harness's normalisation. Works on builds without the
fork's run_manifest_env hook (e.g. upstream PolyFEM).

usage: direct_ref_check.py <PolyFEM_bin> <scene run.json> <outdir> [--repeats N] [--E-ulps K1,K2,...] [--ydisp-ulps ...]
Each variant prints: label, max relative deviation, Newton limit hits, iterations of the worst solve.
"""
import argparse, json, math, os, re, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step_metrics

KEYS = {"err_l2": "L2 error", "err_lp": "Lp error", "err_h1": "H1 error",
        "err_h1_semi": "H1 semi error", "err_linf": "Linf error", "err_linf_grad": "grad max error"}
ap = argparse.ArgumentParser()
ap.add_argument("binary"); ap.add_argument("scene"); ap.add_argument("out")
ap.add_argument("--repeats", type=int, default=1)
ap.add_argument("--E-ulps", default="")
ap.add_argument("--ydisp-ulps", default="", help="ulps added to geometry[0] translation[1]")
ap.add_argument("--env", default="", help="NAME=VAL,... added to the environment")
ap.add_argument("--wrap", default="", help="command prefix, e.g. 'qemu-x86_64 -cpu EPYC-Milan -L /'")
ap.add_argument("--debug-log", action="store_true")
a = ap.parse_args()
base = json.load(open(a.scene))
ref = base["tests"]
variants = [("rep%d" % i, None) for i in range(a.repeats)]
variants += [("E%+d" % int(k), ("E", int(k))) for k in a.E_ulps.split(",") if k]
variants += [("y%+d" % int(k), ("y", int(k))) for k in a.ydisp_ulps.split(",") if k]
env = dict(os.environ, OMP_NUM_THREADS="1")
for kv in filter(None, a.env.split(",")):
    k, _, v = kv.partition("="); env[k] = v
os.makedirs(a.out, exist_ok=True)
for label, var in variants:
    c = json.loads(json.dumps(base)); c.pop("tests", None)
    c.setdefault("solver", {}).setdefault("linear", {})["solver"] = "Eigen::SimplicialLDLT"
    c["root_path"] = os.path.abspath(a.scene)
    if var and var[0] == "E":
        e0 = c["materials"][0]["E"]; c["materials"][0]["E"] = e0 + var[1] * math.ulp(e0)
    if var and var[0] == "y":
        t = c["geometry"][0]["transformation"]["translation"]; t[1] = t[1] + var[1] * math.ulp(t[1])
    d = os.path.join(a.out, label); os.makedirs(d, exist_ok=True)
    sj = os.path.join(d, "scene.json"); json.dump(c, open(sj, "w"))
    cmd = (a.wrap.split() if a.wrap else []) + [a.binary, "-j", sj, "-o", d, "--max_threads", "1",
                                              "--log_level", "debug" if a.debug_log else "info"]
    with open(os.path.join(d, "run.log"), "w") as log:
        subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, env=env)
    txt = open(os.path.join(d, "run.log"), errors="replace").read()
    vals = {}
    for k, name in KEYS.items():
        m = re.findall(r"-- %s: ([0-9.eE+-]+)" % re.escape(name), txt)
        if m: vals[k] = float(m[-1])
    hits = txt.count("Reached iteration limit")
    rows, _ = step_metrics.parse(os.path.join(d, "run.log")) if a.debug_log else ([], 0)
    worst = max((r[1] for r in rows), default=None)
    if len(vals) < 6:
        print(f"{label:8s} max_rel_dev=n/a limit_hits={hits} worst_iters={worst}", flush=True); continue
    dev = max(abs(vals[k] - ref[k]) / max(abs(ref[k]), 1e-5) for k in KEYS)
    print(f"{label:8s} max_rel_dev={dev:.3e} limit_hits={hits} worst_iters={worst} (margin {ref.get('margin', 1e-5)})", flush=True)
