#!/usr/bin/env python3
"""RB-12 stage 2: controlled repeatability of public fixtures.

Runs each selected fixture ``--repeats`` times per thread setting from its own
isolated input directory (a copy of the scene and its meshes, ``output/json``
and ``output/physical_diagnostics`` enabled so that energies, reactions and
candidate counts are recorded), and compares the repeats on what the plan
asks for -- convergence (iterations, termination), restarts and stall
retunes, reactions, energies, candidate counts and peak memory -- not just
VTU byte hashes. Every outcome is kept; a failed run stays in the matrix.

Fixtures are named by their scene in ``scenes/semi-implicit`` with an
optional override, e.g. ``quasistatic-semi@dt=0.0625`` (the PF-08 fine-load
case that failed once and passed on its repeat). Thread settings are
``default`` (no ``--max_threads``) or an integer.

Declared comparison rule (docs/rb-12-validation.md): repeats with an
identical solver path (same subsolve iteration counts, termination reasons,
restarts, retunes, lagging state, trim and refresh sequence) must agree on
the last-step solution to ``--roundoff`` (default 1e-12, |du|_inf, internal
length) and on every endpoint scalar to the same amount relative to the
run-level magnitude of that quantity; repeats whose paths differ are a
*branch divergence* and are reported with their endpoint difference, never
silently averaged; the roundoff-sensitive contact state is a per-step
spread. ``--verify`` exits 1 when a same-path pair violates the rule or any
run did not complete; ``--analyze-only`` recomputes an existing directory.

    python3 tools/rb12/repeat.py --binary build/PolyFEM_bin --output /abs/fresh \
        [--fixtures quasistatic-semi quasistatic-semi-friction quasistatic-semi@dt=0.0625] \
        [--threads default 1] [--repeats 5] [--timeout 3600] [--roundoff 1e-12] [--verify]
"""
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
POLYFEM = HERE.parents[1]
WORKSPACE = POLYFEM.parent
sys.path.insert(0, str(WORKSPACE / "houdini_HDAs" / "src" / "common"))
try:
    import vtu_parser  # noqa: E402
except Exception:  # pragma: no cover - the HDA tree is optional
    vtu_parser = None

SCENES = POLYFEM / "scenes" / "semi-implicit"
ASSETS = ("cube.mesh", "slab.obj")


def parse_fixture(spec):
    name, _, rest = spec.partition("@")
    overrides = {}
    for item in filter(None, rest.split(",")):
        key, _, value = item.partition("=")
        overrides[key] = float(value)
    return name, overrides


def fixture_tag(name, overrides):
    return name + "".join(f"-{k}{v:g}" for k, v in sorted(overrides.items()))


def prepare_input(name, overrides, directory):
    directory.mkdir(parents=True, exist_ok=False)
    config = json.loads((SCENES / f"{name}.json").read_text())
    for asset in ASSETS:
        shutil.copy2(SCENES / asset, directory / asset)
    for key, value in overrides.items():
        if key == "dt":
            config["time"]["dt"] = value
        else:
            raise SystemExit(f"unknown override {key}")
    config.setdefault("output", {})
    config["output"]["json"] = "output.json"
    config["output"]["physical_diagnostics"] = True
    config["output"]["log"] = {"level": "debug"}
    path = directory / "params.json"
    path.write_text(json.dumps(config, indent=2) + "\n")
    return path


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def last_solution(out_dir):
    if vtu_parser is None:
        return None
    vtus = sorted(out_dir.glob("step_*.vtu"), key=lambda p: int(re.search(r"_(\d+)\.vtu", p.name).group(1)))
    vtus = [v for v in vtus if "surf" not in v.name]
    if not vtus:
        return None
    return np.asarray(vtu_parser.read_vtu(str(vtus[-1]))["point_data"]["solution"], dtype=float)


def value(record, key):
    v = record.get(key)
    if isinstance(v, dict):
        return v.get("value")
    return v


def summarize_run(run_dir, out_dir, returncode, wall, timed_out):
    result = {"exit": returncode, "wall_seconds": wall, "timed_out": timed_out}
    manifest_path = out_dir / "run-manifest.json"
    if manifest_path.is_file():
        m = json.loads(manifest_path.read_text())
        result["run_id"] = m["run_id"]
        result["executable_sha256"] = m["process"]["executable"].get("sha256")
        result["effective_sha256"] = m["input"]["effective_sha256"]
        result["effective_sha256_without_paths"] = m["input"].get("effective_sha256_without_paths")
        result["threads_effective"] = m["process"]["threads"]["effective"]
        result["status"] = m["completion"]["status"]
        result["peak_rss_mb"] = m["completion"]["peak_rss_mb"]
        result["sources"] = {k: {"commit": v.get("commit"), "dirty": v.get("dirty"), "patch_sha256": v.get("patch_sha256")} for k, v in m["build"]["sources"].items()}
        history = []
        for s in m["steps"]:
            contact = s.get("contact") or {}
            candidates = contact.get("candidate_count") or {}
            history.append({
                "step": s["step"], "outcome": s["outcome"],
                "iterations": [x.get("iterations") for x in s["subsolves"]],
                "types": [x.get("type") + (f"/lag{x['lag_i']}" if "lag_i" in x else "") for x in s["subsolves"]],
                "reasons": [x.get("termination_reason") for x in s["subsolves"]],
                "restarts": s["termination"].get("restarts"), "unchanged_restarts": s["termination"].get("unchanged_restarts"),
                "stall_retunes": s["stall_retunes"], "lagging": s["lagging"].get("state"),
                "trim": contact.get("trim_or_global_stiffness"), "refresh_id": contact.get("refresh_id"),
                "active_count": contact.get("active_count"), "batch_median": contact.get("batch_median"),
                "fallbacks": [contact.get(k) for k in ("curvature_fallback_count", "curvature_abs_fallback_count", "curvature_global_fallback_count", "interpolated_direction_count")],
                "continued": contact.get("continued_count"), "fresh": contact.get("fresh_count"),
                "candidates_last": candidates.get("value"), "candidates_max": candidates.get("max"), "candidate_builds": candidates.get("builds"),
            })
        result["history"] = history
    else:
        result["status"] = "no manifest"
    output_json = out_dir / "output.json"
    num_vertices = None
    if output_json.is_file():
        o = json.loads(output_json.read_text())
        num_vertices = o.get("num_vertices")
        result["output_json"] = {"num_threads": o.get("num_threads"), "peak_memory_mb": o.get("peak_memory"), "time_solving": o.get("time_solving"),
                                 "num_dofs": o.get("num_dofs"), "num_bases": o.get("num_bases"), "num_vertices": num_vertices}
    diag = out_dir / "physical-diagnostics.jsonl"
    if diag.is_file():
        records = [json.loads(line) for line in diag.read_text().splitlines() if line.strip()]
        result["physical"] = []
        for r in records:
            reaction, reaction_fe = None, None
            for entry in r.get("reactions", []):
                vec = entry.get("full_dof_vector", {}).get("value")
                if vec is not None:
                    v = np.asarray(vec, dtype=float)
                    dim = 3 if v.size % 3 == 0 else 2
                    rows = v.reshape(-1, dim)
                    reaction = [float(x) for x in rows.sum(axis=0)]
                    # The support force on the FE body alone: obstacle vertices
                    # are appended after the FE nodes, and on the P1 public
                    # fixtures the FE node count is the mesh vertex count. The
                    # whole-vector sum cancels the obstacle's reaction against
                    # the support's and is kept only as a balance check.
                    if num_vertices is not None and num_vertices <= rows.shape[0]:
                        reaction_fe = [float(x) for x in rows[:num_vertices].sum(axis=0)]
            pbp = r.get("physical_balance_pass") or {}
            result["physical"].append({
                "step": r["step"], "outcome": r["outcome"],
                "elastic_energy": value(r, "elastic_energy"), "barrier_energy": value(r, "barrier_energy"),
                "kinetic_energy": value(r, "kinetic_energy"), "free_residual_norm": value(r, "free_residual_norm"),
                "min_det_F": value(r, "min_det_F"), "support_work_increment": value(r, "support_work_increment"),
                "external_work_increment": value(r, "external_work_increment"),
                "frictional_dissipation_increment": value(r, "frictional_dissipation_increment"),
                "reaction_sum": reaction, "reaction_fe_sum": reaction_fe, "physical_balance_pass": pbp.get("value"),
                "free_residual_ratio": pbp.get("free_residual_ratio"),
                "peak_external_force": value(pbp, "peak_external_force") if isinstance(pbp.get("peak_external_force"), dict) else pbp.get("peak_external_force"),
                "peak_rss_bytes": value(r, "peak_rss_bytes"),
            })
    u = last_solution(out_dir)
    if u is not None:
        result["solution"] = {"n": int(u.size), "sum": float(u.sum()), "max_abs": float(np.abs(u).max()), "sha256": hashlib.sha256(u.tobytes()).hexdigest()}
        np.save(run_dir / "last_solution.npy", u)
    log = out_dir / "run.log"
    if log.is_file():
        text = log.read_text()
        result["error_lines"] = len(re.findall(r"\[error\]", text))
        result["stopped"] = next((line.strip() for line in text.splitlines() if "PolyFEM stopped:" in line), None)
    return result


def scalar_differences(a, b):
    """Largest difference per endpoint scalar over the steps of two runs, each
    scaled by the run-level magnitude of that quantity (the largest |value|
    over both runs' steps) so a step where the quantity is near zero cannot
    inflate a roundoff difference into a relative one. The residual norm is
    scaled by the peak external force (its RB-09 normalization); the reaction
    is the FE body's summed support force."""
    keys = ("elastic_energy", "barrier_energy", "kinetic_energy", "support_work_increment", "external_work_increment", "frictional_dissipation_increment")
    pa_all, pb_all = a.get("physical", []), b.get("physical", [])
    out = {}
    for key in keys:
        values = [p.get(key) for p in pa_all + pb_all if isinstance(p.get(key), (int, float))]
        if not values:
            continue
        scale = max(max(abs(v) for v in values), 1e-300)
        for pa, pb in zip(pa_all, pb_all):
            va, vb = pa.get(key), pb.get(key)
            if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
                out[key] = max(out.get(key, 0.0), abs(va - vb) / scale)
    forces = [p.get("peak_external_force") for p in pa_all + pb_all if isinstance(p.get("peak_external_force"), (int, float))]
    if forces:
        scale = max(max(forces), 1e-300)
        for pa, pb in zip(pa_all, pb_all):
            va, vb = pa.get("free_residual_norm"), pb.get("free_residual_norm")
            if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
                out["free_residual_norm_over_peak_force"] = max(out.get("free_residual_norm_over_peak_force", 0.0), abs(va - vb) / scale)
    reactions = [np.asarray(p["reaction_fe_sum"]) for p in pa_all + pb_all if p.get("reaction_fe_sum")]
    if reactions:
        scale = max(max(float(np.abs(r).max()) for r in reactions), 1e-300)
        for pa, pb in zip(pa_all, pb_all):
            if pa.get("reaction_fe_sum") and pb.get("reaction_fe_sum"):
                ra, rb = np.asarray(pa["reaction_fe_sum"]), np.asarray(pb["reaction_fe_sum"])
                out["reaction_fe_sum"] = max(out.get("reaction_fe_sum", 0.0), float(np.abs(ra - rb).max()) / scale)
    return out


HISTORY_FIELDS = ("outcome", "iterations", "types", "reasons", "restarts", "stall_retunes", "lagging", "trim", "refresh_id")


def history_key(result):
    """The solver path: what the solver decided, not the roundoff-sensitive
    contact state (active count, continued count, batch median), which is
    summarised separately as the discrete spread of a cell."""
    return json.dumps([{k: h[k] for k in HISTORY_FIELDS} for h in result.get("history", [])], sort_keys=True)


def discrete_spread(runs):
    """Per step, the range over the completed repeats of the discrete contact
    state (active collisions, continued / fresh coefficients, candidates) and
    of the iteration count: which decisions roundoff can move."""
    completed = [r for r in runs if r.get("status") == "completed" and r.get("history")]
    if not completed:
        return []
    spread = []
    for step in range(min(len(r["history"]) for r in completed)):
        entry = {"step": completed[0]["history"][step]["step"]}
        for key in ("active_count", "continued", "fresh", "candidates_last", "candidate_builds", "trim"):
            values = [r["history"][step].get(key) for r in completed]
            values = [v for v in values if isinstance(v, (int, float))]
            if values:
                entry[key] = [min(values), max(values)]
        iterations = [sum(v for v in r["history"][step]["iterations"] if isinstance(v, int)) for r in completed]
        entry["iterations_total"] = [min(iterations), max(iterations)]
        spread.append(entry)
    return spread


def compare_group(runs, run_dirs, roundoff):
    """Pairwise comparison of the repeats of one (fixture, threads) cell."""
    completed = [r for r in runs if r.get("status") == "completed"]
    out = {"runs": len(runs), "completed": len(completed), "distinct_histories": len({history_key(r) for r in completed}),
           "identical_executable": len({r.get("executable_sha256") for r in runs}) == 1,
           "identical_effective_input": len({r.get("effective_sha256_without_paths") for r in runs}) == 1,
           "wall_seconds": [r["wall_seconds"] for r in runs], "peak_rss_mb": [r.get("peak_rss_mb") for r in runs],
           "same_history_max_abs_du": 0.0, "branch_divergences": [], "violations": [],
           "discrete_spread": discrete_spread(runs)}
    sols = {}
    for r, d in zip(runs, run_dirs):
        p = d / "last_solution.npy"
        if p.is_file():
            sols[d.name] = np.load(p)
    names = [d.name for d in run_dirs]
    for i in range(len(runs)):
        for j in range(i + 1, len(runs)):
            a, b = runs[i], runs[j]
            if a.get("status") != "completed" or b.get("status") != "completed":
                continue
            if names[i] not in sols or names[j] not in sols or sols[names[i]].shape != sols[names[j]].shape:
                continue
            du = float(np.abs(sols[names[i]] - sols[names[j]]).max())
            same = history_key(a) == history_key(b)
            scalars = scalar_differences(a, b)
            pair = {"pair": [names[i], names[j]], "same_history": same, "max_abs_du": du, "max_rel_scalar_diff": scalars}
            if same:
                out["same_history_max_abs_du"] = max(out["same_history_max_abs_du"], du)
                if du > roundoff or any(v > roundoff for v in scalars.values()):
                    out["violations"].append(pair)
            else:
                first = next((k for k, (ha, hb) in enumerate(zip(a["history"], b["history"]))
                              if any(ha.get(f) != hb.get(f) for f in HISTORY_FIELDS)), None)
                pair["first_differing_step"] = a["history"][first]["step"] if first is not None else None
                out["branch_divergences"].append(pair)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="fresh evidence directory (must not exist)")
    parser.add_argument("--fixtures", nargs="+", default=["quasistatic-semi", "quasistatic-semi-friction", "quasistatic-semi@dt=0.0625"])
    parser.add_argument("--threads", nargs="+", default=["default", "1"])
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=3600.0)
    parser.add_argument("--roundoff", type=float, default=1e-12)
    parser.add_argument("--verify", action="store_true")
    parser.add_argument("--analyze-only", action="store_true", help="recompute the summary of an existing output directory without running the solver")
    args = parser.parse_args()

    binary = args.binary.resolve()
    out = args.output.resolve()
    if args.analyze_only:
        if not (out / "summary.json").is_file():
            raise SystemExit(f"{out} has no summary.json to re-analyze")
        previous = json.loads((out / "summary.json").read_text())
        args.fixtures, args.threads, args.repeats = previous["fixtures"], previous["threads"], previous["repeats"]
        binary_sha = previous["binary_sha256"]
    else:
        if out.exists():
            raise SystemExit(f"{out} exists; use a fresh directory")
        out.mkdir(parents=True)
        binary_sha = sha256(binary)
    summary = {"binary": str(binary), "binary_sha256": binary_sha, "repeats": args.repeats, "roundoff": args.roundoff,
               "threads": args.threads, "fixtures": args.fixtures, "cells": {}}
    failures = 0
    for spec in args.fixtures:
        name, overrides = parse_fixture(spec)
        tag = fixture_tag(name, overrides)
        for threads in args.threads:
            cell = f"{tag}--threads-{threads}"
            runs, run_dirs = [], []
            for i in range(args.repeats):
                run_dir = out / cell / f"repeat-{i + 1}"
                inp, out_dir = run_dir / "input", run_dir / "output"
                if args.analyze_only:
                    previous_result = json.loads((run_dir / "result.json").read_text())
                    rc, wall, timed_out = previous_result["exit"], previous_result["wall_seconds"], previous_result["timed_out"]
                else:
                    params = prepare_input(name, overrides, inp)
                    out_dir.mkdir()
                    cmd = [str(binary), "--json", "params.json", "-o", str(out_dir), "--log_level", "debug"]
                    if threads != "default":
                        cmd += ["--max_threads", str(threads)]
                    (run_dir / "command.txt").write_text(" ".join(cmd) + "\n")
                    start = time.monotonic()
                    timed_out = False
                    with (out_dir / "run.log").open("w") as log:
                        try:
                            completed = subprocess.run(cmd, cwd=inp, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
                            rc = completed.returncode
                        except subprocess.TimeoutExpired:
                            rc, timed_out = None, True
                    wall = time.monotonic() - start
                result = summarize_run(run_dir, out_dir, rc, wall, timed_out)
                result["input_sha256"] = {p.name: sha256(p) for p in inp.iterdir()}
                runs.append(result)
                run_dirs.append(run_dir)
                (run_dir / "result.json").write_text(json.dumps(result, indent=1) + "\n")
                print(f"{cell} repeat {i + 1}: exit {rc} status {result.get('status')} wall {wall:.1f}s "
                      f"peak {result.get('peak_rss_mb')} MB steps {len(result.get('history', []))}", flush=True)
                if result.get("status") != "completed":
                    failures += 1
            comparison = compare_group(runs, run_dirs, args.roundoff)
            summary["cells"][cell] = {"fixture": tag, "threads": threads, "runs": runs, "comparison": comparison}
            print(f"  -> {comparison['completed']}/{comparison['runs']} completed, {comparison['distinct_histories']} distinct histories, "
                  f"same-history max ‖du‖ {comparison['same_history_max_abs_du']:.3e}, {len(comparison['branch_divergences'])} branch divergences, "
                  f"{len(comparison['violations'])} violations", flush=True)
            failures += len(comparison["violations"])
            (out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")

    # Serial versus threaded: compare the cells of one fixture across thread settings.
    summary["cross_thread"] = {}
    for spec in args.fixtures:
        name, overrides = parse_fixture(spec)
        tag = fixture_tag(name, overrides)
        cells = [(t, summary["cells"][f"{tag}--threads-{t}"]) for t in args.threads if f"{tag}--threads-{t}" in summary["cells"]]
        for i in range(len(cells)):
            for j in range(i + 1, len(cells)):
                (ta, ca), (tb, cb) = cells[i], cells[j]
                pairs = []
                for ra in ca["runs"]:
                    for rb in cb["runs"]:
                        if ra.get("status") != "completed" or rb.get("status") != "completed":
                            continue
                        pa = out / f"{tag}--threads-{ta}" / f"repeat-{ca['runs'].index(ra) + 1}" / "last_solution.npy"
                        pb = out / f"{tag}--threads-{tb}" / f"repeat-{cb['runs'].index(rb) + 1}" / "last_solution.npy"
                        if not (pa.is_file() and pb.is_file()):
                            continue
                        ua, ub = np.load(pa), np.load(pb)
                        if ua.shape != ub.shape:
                            continue
                        pairs.append({"same_history": history_key(ra) == history_key(rb), "max_abs_du": float(np.abs(ua - ub).max())})
                summary["cross_thread"][f"{tag}: threads {ta} vs {tb}"] = {
                    "pairs": len(pairs), "same_history_pairs": sum(p["same_history"] for p in pairs),
                    "max_abs_du_same_history": max([p["max_abs_du"] for p in pairs if p["same_history"]], default=None),
                    "max_abs_du_different_history": max([p["max_abs_du"] for p in pairs if not p["same_history"]], default=None),
                    "peak_rss_mb": {ta: ca["comparison"]["peak_rss_mb"], tb: cb["comparison"]["peak_rss_mb"]},
                    "wall_seconds": {ta: ca["comparison"]["wall_seconds"], tb: cb["comparison"]["wall_seconds"]}}
    (out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    write_markdown(summary, out / "summary.md")
    print(f"summary: {out / 'summary.json'}; failures/violations: {failures}")
    return 1 if (args.verify and failures) else 0


def write_markdown(summary, path):
    lines = ["# RB-12 repeatability matrix", "", f"binary `{summary['binary']}` sha256 `{summary['binary_sha256']}`; repeats {summary['repeats']}; roundoff rule {summary['roundoff']}", "",
             "| cell | completed | distinct histories | same-history max ‖du‖ | branch divergences (first differing step, max ‖du‖) | violations | wall s (min–max) | peak RSS MB (min–max) |", "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for cell, data in summary["cells"].items():
        c = data["comparison"]
        walls = [w for w in c["wall_seconds"] if w is not None]
        rss = [r for r in c["peak_rss_mb"] if r is not None]
        div = "; ".join(f"step {d['first_differing_step']}, {d['max_abs_du']:.2e}" for d in c["branch_divergences"]) or "none"
        lines.append(f"| {cell} | {c['completed']}/{c['runs']} | {c['distinct_histories']} | {c['same_history_max_abs_du']:.2e} | {div} | {len(c['violations'])} | {min(walls):.1f}–{max(walls):.1f} | {min(rss):.0f}–{max(rss):.0f} |" if walls and rss else f"| {cell} | {c['completed']}/{c['runs']} | – | – | – | – | – | – |")
    lines += ["", "## Discrete state spread across repeats (min–max per step)", "", "| cell | step | active | continued | candidates (last) | iterations | trim |", "| --- | --- | --- | --- | --- | --- | --- |"]
    for cell, data in summary["cells"].items():
        for e in data["comparison"].get("discrete_spread", []):
            rng = lambda k: "–" if k not in e else (f"{e[k][0]:g}" if e[k][0] == e[k][1] else f"{e[k][0]:g}–{e[k][1]:g}")
            if any(k in e and e[k][0] != e[k][1] for k in ("active_count", "continued", "candidates_last", "iterations_total", "trim")):
                lines.append(f"| {cell} | {e['step']} | {rng('active_count')} | {rng('continued')} | {rng('candidates_last')} | {rng('iterations_total')} | {rng('trim')} |")
    lines += ["", "(only steps where some repeat differs are listed)", "", "## Serial versus threaded", "", "| fixture | pairs | same-history pairs | max ‖du‖ same history | max ‖du‖ different history |", "| --- | --- | --- | --- | --- |"]
    for key, data in summary.get("cross_thread", {}).items():
        f = lambda v: "–" if v is None else f"{v:.2e}"
        lines.append(f"| {key} | {data['pairs']} | {data['same_history_pairs']} | {f(data['max_abs_du_same_history'])} | {f(data['max_abs_du_different_history'])} |")
    path.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
