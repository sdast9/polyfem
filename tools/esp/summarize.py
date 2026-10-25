#!/usr/bin/env python3
"""Summarize a run_matrix.py output directory (docs/esp-assessment-20261025.md).

usage: summarize.py <output dir>   (needs numpy and meshio)
"""
import glob, json, re, sys
from pathlib import Path
import numpy as np
import meshio

STEP = re.compile(r"\[info\]\s+(\d+)/(\d+)\s+t=")
ITERS = re.compile(r"Finished:.*?\(iters=(\d+)")
MIND = re.compile(r"Minimum distance during solve: ([-0-9.eE+]+)")


def final_solution(out):
    files = glob.glob(str(out / "step_*.vtu"))
    if not files:
        return None, None
    files.sort(key=lambda f: int(re.search(r"step_(\d+)\.vtu", f).group(1)))
    m = meshio.read(files[-1])
    return m.points, m.point_data.get("solution")


def summarize(rd):
    run = json.loads((rd / "run.json").read_text())
    log = (rd / "log.txt").read_text(errors="replace")
    steps = STEP.findall(log)
    iters = [int(x) for x in ITERS.findall(log)]
    mind = [float(x) for x in MIND.findall(log)]
    res = dict(run)
    res.pop("cmd", None)
    res["steps"] = f"{steps[-1][0]}/{steps[-1][1]}" if steps else "0/?"
    res["completed"] = bool(steps) and steps[-1][0] == steps[-1][1]
    res["minimize_calls"] = len(iters)
    res["newton_iters"] = sum(iters)
    res["error_lines"] = len(re.findall(r"\] \[(?:error|critical)\]", log))
    res["stall_restarts"] = len(re.findall(r"Line-search stall detected", log))
    res["ls_failures"] = len(re.findall(r"Line search failed", log))
    res["min_distance"] = min(mind) if mind else None
    res["last_error"] = next((l[l.find("]", 30) + 2:][:200] for l in reversed(log.splitlines()) if "[error]" in l or "[critical]" in l), None)
    pts, u = final_solution(rd / "out")
    if u is not None:
        res["n_points"] = int(len(u))
        res["max_abs_u"] = float(np.abs(u).max())
        res["mean_u"] = [round(float(x), 6) for x in u.mean(axis=0)]
        np.save(rd / "final_u.npy", u)
    return res


def main(root):
    root = Path(root)
    rows = [summarize(p.parent) for p in sorted(root.glob("**/run.json"))]
    # pairwise: compare each run's final displacement with the scene's reference variant
    by_scene = {}
    for r in rows:
        by_scene.setdefault(r["label"].rsplit("/", 1)[0], []).append(r)
    for scene, rs in by_scene.items():
        ref = next((r for r in rs if r["label"].endswith("/U-ESP")), None) or next((r for r in rs if r["label"].endswith("/F-SEMI")), None)
        if ref is None:
            continue
        ru = root / ref["label"] / "final_u.npy"
        if not ru.exists():
            continue
        U = np.load(ru)
        for r in rs:
            p = root / r["label"] / "final_u.npy"
            if p.exists():
                V = np.load(p)
                if V.shape == U.shape:
                    r["max_du_vs_" + ref["label"].rsplit("/", 1)[1]] = float(np.abs(V - U).max())
    (root / "summary.json").write_text(json.dumps(rows, indent=2))
    hdr = f"{'run':48} {'exit':>5} {'steps':>9} {'wall s':>8} {'calls':>6} {'newton':>7} {'stall':>5} {'err':>4} {'min dist':>10} {'max|u|':>9} {'max du':>9}"
    print(hdr); print("-" * len(hdr))
    for r in rows:
        du = next((v for k, v in r.items() if k.startswith("max_du_vs_")), None)
        md = r["min_distance"]
        md_s = f"{md:.2e}" if md is not None else "-"
        mu_s = f"{r.get('max_abs_u', float('nan')):.3g}"
        du_s = f"{du:.2e}" if du is not None else "-"
        print(f"{r['label']:48} {str(r['exit']):>5} {r['steps']:>9} {r['wall_s']:>8.1f} {r['minimize_calls']:>6} {r['newton_iters']:>7} {r['stall_restarts']:>5} {r['error_lines']:>4} "
              f"{md_s:>10} {mu_s:>9} {du_s:>9}")


if __name__ == "__main__":
    main(sys.argv[1])
