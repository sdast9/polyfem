#!/usr/bin/env python3
"""RB-22 scene matrix: high-order hexahedral collision surfaces.

Runs every scene in ``tools/rb22/scenes`` (or the selected ones) with a
given ``PolyFEM_bin`` into ``<output>/<scene>/`` single-threaded, dumping the
FE collision proxy (``POLYFEM_DUMP_COLLISION_PROXY``), and writes
``<output>/summary.json`` with, per scene: exit status, the first error line,
the builder's collision-mesh diagnostic (vertices, exact selector vs
interpolated rows, faces), proxy statistics from the dumped OBJ (used
vertices, faces, closedness, edge-manifoldness, Euler characteristic,
zero-area faces, SHA-256 of the file), contact counts, Newton iterations,
minimum distance, wall time, and the final solution's SHA-256 / max |u|
(``tools/rb18/summarize_smokes.py`` convention).

``--compare other/summary.json`` adds per-scene identity flags (proxy OBJ
hash, solution hash, max abs solution difference) against another run, e.g.
the pre-change baseline binary. Bounded evidence extraction, not a physical
acceptance criterion. The output directory must not already exist.
"""
import argparse
import collections
import hashlib
import json
import os
import re
import struct
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "rb18"))
from summarize_smokes import read_solution  # noqa: E402


def proxy_stats(obj: Path):
    if not obj.exists():
        return None
    V, F = [], []
    for line in obj.read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "v":
            V.append(tuple(float(x) for x in parts[1:4]))
        elif parts[0] == "f":
            F.append(tuple(int(x.split("/")[0]) - 1 for x in parts[1:4]))
    used = set(i for f in F for i in f)
    edge_count = collections.Counter()
    for a, b, c in F:
        for u, v in ((a, b), (b, c), (c, a)):
            edge_count[(min(u, v), max(u, v))] += 1
    zero_area = 0
    for a, b, c in F:
        ab = [V[b][k] - V[a][k] for k in range(3)]
        ac = [V[c][k] - V[a][k] for k in range(3)]
        cross = [ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]]
        area2 = sum(x * x for x in cross) ** 0.5
        scale = max(sum(x * x for x in ab), sum(x * x for x in ac))
        if not area2 > 1e-12 * scale:
            zero_area += 1
    return {
        "vertex_rows": len(V),
        "used_vertices": len(used),
        "faces": len(F),
        "edges": len(edge_count),
        "closed": all(n == 2 for n in edge_count.values()),
        "edge_manifold": all(n <= 2 for n in edge_count.values()),
        "euler": len(used) - len(edge_count) + len(F),
        "zero_area_faces": zero_area,
        "sha256": hashlib.sha256(obj.read_bytes()).hexdigest(),
    }


def parse_log(text: str):
    entry = {"error_lines": text.count("[error]")}
    m = re.search(r"\[error\] (.*)", text)
    if m:
        entry["first_error"] = m.group(1)[:400]
    m = re.search(r"Collision mesh from (.*?): (\d+) FE surface vertices \((\d+) exact selector rows, (\d+) interpolated rows\), (\d+) faces, (\d+) edges", text)
    if m:
        entry["collision_mesh"] = {
            "extraction": m.group(1),
            "surface_vertices": int(m.group(2)),
            "selector_rows": int(m.group(3)),
            "interpolated_rows": int(m.group(4)),
            "faces": int(m.group(5)),
            "edges": int(m.group(6)),
        }
    stamps = re.findall(r"\[[0-9-]+ (\d\d:\d\d:\d\d\.\d+)\] \[polyfem\] \[info\] (Building collision mesh\.\.\.|Done!)", text)
    for i in range(len(stamps) - 1):
        if stamps[i][1].startswith("Building") and stamps[i + 1][1] == "Done!":
            def secs(s):
                h, m_, s_ = s.split(":")
                return int(h) * 3600 + int(m_) * 60 + float(s_)
            entry["collision_mesh_build_seconds"] = round(secs(stamps[i + 1][0]) - secs(stamps[i][0]), 4)
            break
    contacts = [int(x) for x in re.findall(r"Refreshed semi-implicit barrier stiffness over (\d+) contacts", text)]
    if contacts:
        entry["contacts_refresh"] = {"first": contacts[0], "max": max(contacts), "last": contacts[-1]}
    iters = [int(x) for x in re.findall(r"Finished: .*?\(iters=(\d+)", text)]
    if iters:
        entry["newton_solves"] = len(iters)
        entry["newton_iterations"] = sum(iters)
    dists = [float(x) for x in re.findall(r"Minimum distance during solve: ([0-9.eE+-]+)", text)]
    if dists:
        entry["min_distance"] = min(dists)
    m = re.findall(r"total time: ([0-9.]+)s", text)
    if m:
        entry["total_time_s"] = float(m[-1])
    skipped = re.search(r"skipped (\d+) of (\d+) boundary faces", text)
    if skipped:
        entry["skipped_faces"] = [int(skipped.group(1)), int(skipped.group(2))]
    return entry


def run_scene(binary: Path, scene: Path, out: Path, threads: int, timeout: float):
    out.mkdir(parents=True)
    obj = out / "proxy.obj"
    env = dict(os.environ, POLYFEM_DUMP_COLLISION_PROXY=str(obj))
    cmd = [str(binary), "--json", scene.name, "-o", str(out), "--log_level", "debug", "--max_threads", str(threads)]
    t0 = time.time()
    returncode = "timeout"
    with open(out / "run.log", "w") as log:
        try:
            returncode = subprocess.run(cmd, cwd=scene.parent, stdout=log, stderr=subprocess.STDOUT, env=env, timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            log.write(f"\n[run_matrix] killed after {timeout}s timeout\n")
    wall = time.time() - t0
    (out / "exit.txt").write_text(f"{returncode}\n")
    (out / "command.txt").write_text(" ".join(cmd) + f"\ncwd={scene.parent}\ntimeout={timeout}\n")
    entry = {"exit": returncode, "wall_seconds": round(wall, 3), "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest()}
    entry.update(parse_log((out / "run.log").read_text(errors="replace")))
    entry["proxy"] = proxy_stats(obj)
    steps = sorted(out.glob("step_*.vtu"), key=lambda p: int(p.stem.split("_")[1]))
    entry["steps"] = len(steps)
    if steps:
        ncomp, vals = read_solution(steps[-1])
        if vals is not None:
            entry["solution"] = {
                "n_values": len(vals),
                "max_abs_u": max(abs(v) for v in vals),
                "sha256": hashlib.sha256(struct.pack("<%dd" % len(vals), *vals)).hexdigest(),
            }
            entry["_values"] = vals
    return entry


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--binary", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True, help="fresh directory")
    ap.add_argument("--scenes", nargs="*", help="scene names without .json (default: all)")
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--compare", type=Path, help="summary.json of another run")
    ap.add_argument("--timeout", type=float, default=600.0, help="seconds per scene; a killed run records exit \"timeout\"")
    args = ap.parse_args()
    args.output = args.output.resolve() # the solver runs from the scene directory
    args.binary = args.binary.resolve()
    if args.output.exists():
        sys.exit(f"refusing to overwrite {args.output}")
    args.output.mkdir(parents=True)
    scenes = sorted((HERE / "scenes").glob("*.json"))
    if args.scenes:
        scenes = [HERE / "scenes" / (s + ".json") for s in args.scenes]
    summary = {"binary": str(args.binary), "binary_sha256": hashlib.sha256(args.binary.read_bytes()).hexdigest(), "threads": args.threads, "scenes": {}}
    for scene in scenes:
        print("===", scene.stem, flush=True)
        entry = run_scene(args.binary, scene, args.output / scene.stem, args.threads, args.timeout)
        summary["scenes"][scene.stem] = entry
        print({k: v for k, v in entry.items() if k != "_values"}, flush=True)
    if args.compare:
        other = json.loads(args.compare.read_text())["scenes"]
        for name, e in summary["scenes"].items():
            o = other.get(name)
            if o is None:
                continue
            cmp = {"exit_same": e["exit"] == o["exit"]}
            if e.get("proxy") and o.get("proxy"):
                cmp["proxy_identical"] = e["proxy"]["sha256"] == o["proxy"]["sha256"]
            if "solution" in e and "solution" in o:
                cmp["solution_identical"] = e["solution"]["sha256"] == o["solution"]["sha256"]
                a, b = e["_values"], o["_values"]
                cmp["max_abs_solution_diff"] = max(abs(x - y) for x, y in zip(a, b)) if len(a) == len(b) else None
            e["compare"] = cmp
    (args.output / "summary.json").write_text(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
