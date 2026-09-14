#!/usr/bin/env python3
"""RB-11 input-validation matrix runner.

Builds every case of ``cases.py`` (or the selected ones) under
``<output>/cases/<name>/``, runs ``PolyFEM_bin`` on it single-threaded from
inside that directory, and classifies the outcome:

* ``named_failure`` -- exit status 1 with a ``PolyFEM stopped:`` line;
* ``resource_failure`` -- exit status 3;
* ``crash`` -- a signal, an abort or any exit status >= 128;
* ``completed`` -- exit 0 and the final ``total time`` line;
* ``timeout`` / ``other_exit`` -- everything else.

Per case the summary records the first ``[error]``/``[critical]`` line, the
``PolyFEM stopped`` line, warnings, the last log phase reached (input parsing,
mesh, basis, collision mesh, solve), whether any VTU output was written, the
wall time and -- for the per-element material cases -- whether the exported
field at the top/bottom vertices matches the source file (``check.json``).
``expect_match`` compares the run with the case's declared contract, not
just its classification: a ``named_failure`` must exit 1 with ``PolyFEM
stopped:``, write no VTU and carry the case's ``expect_error`` phrase in its
log; an ``accepted`` run must complete, save the intended number of steps and
pass its ``check.json`` (per-element transfer, top/bottom field, obstacle
incidence, required log lines); an ``accepted_with_notice`` run additionally
needs its notice in the log. A case may declare ``expected_status: fail`` for
its check to document that the oracle discriminates (the permuted-file case).

Modes: the default is investigation (report every mismatch, exit 0);
``--verify`` exits 1 on any mismatch or when no case was selected;
``--self-test`` feeds deliberately hollow records to the oracle (accepted
without steps, notice without the notice, a named failure with an unrelated
error) and exits 1 if any of them is accepted. The output directory must not
already exist.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cases  # noqa: E402
import fixtures  # noqa: E402

# the VTU reader lives in the Houdini tree beside the polyfem checkout; a
# worktree elsewhere finds it through the ancestors or POLYFEM_HDA_COMMON
import os  # noqa: E402
_HDA_COMMON = [Path(os.environ["POLYFEM_HDA_COMMON"])] if os.environ.get("POLYFEM_HDA_COMMON") else []
_HDA_COMMON += [parent / "houdini_HDAs" / "src" / "common" for parent in HERE.parents]
for _candidate in _HDA_COMMON:
    if (_candidate / "vtu_parser.py").exists():
        sys.path.insert(0, str(_candidate))
        break
try:
    import vtu_parser  # noqa: E402
except Exception:  # pragma: no cover - the HDA tree is optional
    vtu_parser = None

PHASES = [
    ("solve", re.compile(r"Solving|Timestep|time step|Newton|Starting nonlinear")),
    ("collision_mesh", re.compile(r"Building collision mesh")),
    ("basis", re.compile(r"Building .*basis")),
    ("mesh", re.compile(r"Loading mesh")),
    ("input", re.compile(r"Saving output to|Using variational formulation")),
]


def classify(returncode, log, timed_out):
    if timed_out:
        return "timeout"
    if returncode < 0 or returncode >= 128:
        return "crash"
    if returncode == 3:
        return "resource_failure"
    if returncode == 1 and "PolyFEM stopped:" in log:
        return "named_failure"
    if returncode == 0 and "total time:" in log:
        return "completed"
    return "other_exit"


def last_phase(log):
    for name, rx in PHASES:
        if rx.search(log):
            return name
    return "none"


def first_match(log, rx):
    m = re.search(rx, log)
    return m.group(0)[:400] if m else None


def _last_volume_vtu(case_dir):
    out = Path(case_dir) / "out"
    vtus = sorted(out.glob("*.vtu"))
    vtus = [v for v in vtus if "surf" not in v.name and "col" not in v.name]
    return vtus[-1] if vtus else None


def check_per_element(case_dir, spec):
    """Match every output element to the fixture element with the same
    centroid and compare its field value/vector with the source row of that
    element. Unique source values make this sensitive to any permutation of
    the file rows (the permuted-file case documents that)."""
    import numpy as np
    if vtu_parser is None:
        return {"status": "not run", "reason": "vtu_parser unavailable"}
    vtu = _last_volume_vtu(case_dir)
    if vtu is None:
        return {"status": "fail", "reason": "no vtu"}
    mesh = vtu_parser.read_vtu(str(vtu))
    pts, pdata = mesh["points"], mesh["point_data"]
    cells = mesh["cells"].get("tet")
    if cells is None:
        cells = mesh["cells"].get("tri")
    if cells is None:
        return {"status": "fail", "reason": "no volume cells", "cells": sorted(mesh["cells"])}
    cells = np.asarray(cells)
    src_cent = np.asarray(spec["centroids"], dtype=float)
    dim = src_cent.shape[1]
    out_cent = pts[cells][:, :, :dim].mean(axis=1)
    # nearest fixture centroid for every output cell; must be unique and exact
    d2 = ((out_cent[:, None, :] - src_cent[None, :, :]) ** 2).sum(axis=2)
    match = d2.argmin(axis=1)
    scale = 1e-8 * max(1.0, float(np.abs(src_cent).max()))
    if (np.sqrt(d2[np.arange(len(match)), match]) > scale).any():
        return {"status": "fail", "reason": "output cells do not match the fixture centroids"}
    if len(set(match.tolist())) != len(src_cent) or len(cells) != len(src_cent):
        return {"status": "fail", "reason": f"{len(cells)} output cells vs {len(src_cent)} fixture elements, {len(set(match.tolist()))} matched"}
    body = np.asarray(spec["body_ids"])
    bodies = spec.get("bodies")
    keep = np.ones(len(cells), bool) if bodies is None else np.isin(body[match], bodies)
    field = spec["field"]
    if "values" in spec:
        keys = [k for k in pdata if k == field or k.endswith("/" + field)]
        if not keys:
            return {"status": "fail", "reason": f"no {field} field", "fields": sorted(pdata)}
        vals = pdata[keys[0]].ravel()
        expected = np.asarray(spec["values"], dtype=float)[match]
        worst = 0.0
        for c in np.flatnonzero(keep):
            got = vals[cells[c]]
            # the output is per element: every point of the cell carries the element value
            err = float(np.abs(got - expected[c]).max() / max(abs(expected[c]), 1e-300))
            worst = max(worst, err)
        return {"status": "pass" if worst <= 1e-9 else "fail", "max_rel_error": worst, "cells_checked": int(keep.sum())}
    keys = [k for k in pdata if k.endswith(field + "_x")]
    if not keys:
        return {"status": "fail", "reason": f"no {field} vector field", "fields": sorted(pdata)}
    prefix = keys[0][:-len(field + "_x")]
    comps = [prefix + f"{field}_{a}" for a in "xyz"[:dim]]
    vec = np.stack([pdata[c].ravel() for c in comps], axis=1)
    expected = np.asarray(spec["vectors"], dtype=float)[match][:, :dim]
    worst_dot = 1.0
    for c in np.flatnonzero(keep):
        got = vec[cells[c]]
        norms = np.linalg.norm(got, axis=1)
        if (norms < 1e-12).any():
            worst_dot = 0.0
            continue
        dots = (got / norms[:, None]) @ (expected[c] / np.linalg.norm(expected[c]))
        worst_dot = min(worst_dot, float(dots.min()))
    return {"status": "pass" if worst_dot >= 1 - 1e-9 else "fail", "min_dot": worst_dot, "cells_checked": int(keep.sum())}


def check_field(case_dir, spec):
    """Evaluate a case's check.json: per-element transfer, obstacle incidence,
    top/bottom field values, or required / forbidden log lines."""
    if "per_element" in spec:
        return check_per_element(case_dir, spec["per_element"])
    if "obstacle_max_edge_degree" in spec:
        faces = fixtures.read_obj_faces(str(Path(case_dir) / spec["obstacle"]))
        degree = max(fixtures.edge_degrees(faces).values())
        return {"status": "pass" if degree == spec["obstacle_max_edge_degree"] else "fail",
                "max_edge_degree": degree}
    if "field" not in spec:
        log = (Path(case_dir) / "run.log").read_text()
        missing = [m for m in spec.get("log_contains", []) if m not in log]
        present = [m for m in spec.get("log_excludes", []) if m in log]
        return {"status": "pass" if not missing and not present else "fail",
                "missing": missing, "forbidden_present": present}
    if vtu_parser is None:
        return {"status": "not run", "reason": "vtu_parser unavailable"}
    vtu = _last_volume_vtu(case_dir)
    if vtu is None:
        return {"status": "not run", "reason": "no vtu"}
    mesh = vtu_parser.read_vtu(str(vtu))
    pts = mesh["points"]
    pdata = mesh["point_data"]
    field = spec["field"]
    if field == "fiber_direction":
        keys = [k for k in pdata if k.endswith("fiber_direction_x")]
        if not keys:
            return {"status": "fail", "reason": "no fiber_direction field", "fields": sorted(pdata)}
        prefix = keys[0][:-len("fiber_direction_x")]
        vals = {a: pdata[prefix + f"fiber_direction_{a}"].ravel() for a in "xyz"}
        import numpy as np
        vec = np.stack([vals["x"], vals["y"], vals["z"]], axis=1)
        if spec.get("normalize"):
            norms = np.linalg.norm(vec, axis=1)
            vec = vec / np.where(norms > 0, norms, 1.0)[:, None]
        zmax = pts[:, 2].max()
        top = np.abs(pts[:, 2] - zmax) < 1e-9
        expect = np.asarray(spec["top_expect"], dtype=float)
        dots = vec[top] @ expect
        return {"status": "pass" if (dots > 0.99).all() else "fail",
                "top_points": int(top.sum()), "min_dot_top": float(dots.min()),
                "max_dot_top": float(dots.max())}
    keys = [k for k in pdata if k == field or k.endswith("/" + field)]
    if not keys:
        return {"status": "fail", "reason": f"no {field} field", "fields": sorted(pdata)}
    vals = pdata[keys[0]].ravel()
    zmax, zmin = pts[:, 2].max(), pts[:, 2].min()
    top = abs(pts[:, 2] - zmax) < 1e-9
    bottom = abs(pts[:, 2] - zmin) < 1e-9
    res = {"top_points": int(top.sum()), "top_min": float(vals[top].min()), "top_max": float(vals[top].max())}
    ok = abs(vals[top] - spec["top_expect"]).max() <= 1e-6 * abs(spec["top_expect"])
    if "bottom_expect" in spec:
        res.update({"bottom_min": float(vals[bottom].min()), "bottom_max": float(vals[bottom].max())})
        ok = ok and abs(vals[bottom] - spec["bottom_expect"]).max() <= 1e-6 * abs(spec["bottom_expect"])
    res["status"] = "pass" if ok else "fail"
    return res


def run_case(binary, case_dir, timeout, log_level, meta):
    out = Path(case_dir) / "out"
    out.mkdir()
    cmd = [str(binary), "--json", "scene.json", "-o", "out", "--log_level", log_level, "--max_threads", "1"]
    t0 = time.time()
    timed_out = False
    try:
        proc = subprocess.run(cmd, cwd=case_dir, capture_output=True, text=True, timeout=timeout)
        rc, log = proc.returncode, proc.stdout + proc.stderr
    except subprocess.TimeoutExpired as e:
        timed_out = True
        rc = None
        log = (e.stdout or b"").decode(errors="replace") if isinstance(e.stdout, bytes) else (e.stdout or "")
        log += (e.stderr or b"").decode(errors="replace") if isinstance(e.stderr, bytes) else (e.stderr or "")
    wall = time.time() - t0
    (Path(case_dir) / "run.log").write_text(log)
    (Path(case_dir) / "command.txt").write_text(" ".join(cmd) + f"\n# cwd {case_dir}\n# exit {rc}\n")
    vtus = list(out.glob("*.vtu"))
    entry = {
        "exit": rc,
        "classification": classify(rc if rc is not None else 0, log, timed_out),
        "wall_s": round(wall, 2),
        "last_phase": last_phase(log),
        "first_error": first_match(log, r"\[(?:error|critical)\] (.*)"),
        "stopped": first_match(log, r"PolyFEM stopped: (.*)"),
        "warnings": len(re.findall(r"\[warning\]", log)),
        "warning_lines": [m[:300] for m in re.findall(r"\[warning\] (.*)", log)][:12],
        "vtu_files": len(vtus),
        "steps_saved": len([v for v in vtus if re.search(r"_(\d+)\.vtu$", v.name) and "surf" not in v.name]),
        "total_time": first_match(log, r"total time: [0-9.e+-]+s"),
    }
    check = Path(case_dir) / "check.json"
    entry["has_check"] = check.exists()
    if check.exists():
        spec = json.loads(check.read_text())
        entry["expected_check_status"] = spec.get("expected_status", "pass")
        if entry["classification"] == "completed":
            entry["field_check"] = check_field(case_dir, spec)
    entry["log_has_expected_error"] = bool(meta.get("expect_error")) and meta["expect_error"] in log
    return entry


def expect_match(meta, entry):
    """The oracle: the run must satisfy the case's contract, not just its class."""
    e, c = meta["expect"], entry["classification"]
    if e == "named_failure":
        return (c == "named_failure" and entry.get("vtu_files", 1) == 0
                and bool(meta.get("expect_error")) and bool(entry.get("log_has_expected_error")))
    if e in ("accepted", "accepted_with_notice"):
        if c != "completed":
            return False
        if entry.get("steps_saved") != meta.get("steps", 3):
            return False
        if e == "accepted_with_notice" and not entry.get("has_check"):
            return False  # the notice is a required log line declared in check.json
        if entry.get("has_check"):
            fc = entry.get("field_check")
            if not fc:
                return False
            return fc.get("status") == entry.get("expected_check_status", "pass")
        return True
    return False


def self_test():
    """The oracle must reject hollow records."""
    failures = []
    named = {"expect": "named_failure", "expect_error": "must be finite", "steps": 3}
    if expect_match(named, {"classification": "named_failure", "vtu_files": 0, "log_has_expected_error": False}):
        failures.append("named failure with an unrelated error accepted")
    if expect_match(named, {"classification": "named_failure", "vtu_files": 1, "log_has_expected_error": True}):
        failures.append("named failure that wrote output accepted")
    if not expect_match(named, {"classification": "named_failure", "vtu_files": 0, "log_has_expected_error": True}):
        failures.append("a proper named failure was rejected")
    accepted = {"expect": "accepted", "steps": 3}
    if expect_match(accepted, {"classification": "completed", "steps_saved": 0, "has_check": False}):
        failures.append("accepted run without saved steps accepted")
    if expect_match(accepted, {"classification": "completed", "steps_saved": 3, "has_check": True, "field_check": {"status": "fail"}, "expected_check_status": "pass"}):
        failures.append("accepted run with a failed check accepted")
    if not expect_match(accepted, {"classification": "completed", "steps_saved": 3, "has_check": True, "field_check": {"status": "fail"}, "expected_check_status": "fail"}):
        failures.append("a declared-to-fail check was not honoured")
    notice = {"expect": "accepted_with_notice", "steps": 3}
    if expect_match(notice, {"classification": "completed", "steps_saved": 3, "has_check": False}):
        failures.append("notice case without any notice check accepted")
    if expect_match(notice, {"classification": "completed", "steps_saved": 3, "has_check": True, "field_check": {"status": "fail", "missing": ["x"]}, "expected_check_status": "pass"}):
        failures.append("notice case with the notice missing accepted")
    if not expect_match(notice, {"classification": "completed", "steps_saved": 3, "has_check": True, "field_check": {"status": "pass"}, "expected_check_status": "pass"}):
        failures.append("a complete notice case was rejected")
    for f in failures:
        print("SELF-TEST FAILURE:", f)
    print("oracle self-test:", "pass" if not failures else "FAIL")
    return 0 if not failures else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--binary", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--cases", nargs="*")
    ap.add_argument("--groups", nargs="*")
    ap.add_argument("--timeout", type=float, default=300)
    ap.add_argument("--log-level", default="debug")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--verify", action="store_true", help="exit 1 on any mismatch or when no case is selected")
    ap.add_argument("--self-test", action="store_true", help="check that the oracle rejects hollow records, then exit")
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    if args.list:
        for name, c in cases.CASES.items():
            print(f"{name:40s} {c['group']:10s} {c['expect']:22s} {c['description']}")
        return 0

    binary = Path(args.binary).resolve()
    output = Path(args.output)
    if output.exists():
        sys.exit(f"refusing to write into an existing directory: {output}")
    output.mkdir(parents=True)
    names = set(args.cases or [])
    if args.groups:
        names |= {n for n, c in cases.CASES.items() if c["group"] in args.groups}
    metas = cases.build_all(str(output / "cases"), names or None)
    if not metas:
        print("no case selected")
        return 1

    summary = {"binary": str(binary), "cases": {}}
    try:
        import hashlib
        summary["binary_sha256"] = hashlib.sha256(binary.read_bytes()).hexdigest()
    except Exception:
        pass
    for meta in metas:
        case_dir = output / "cases" / meta["name"]
        entry = run_case(binary, str(case_dir), args.timeout, args.log_level, meta)
        entry.update({k: meta[k] for k in ("group", "expect", "description", "control", "expect_error", "steps")})
        entry["expect_match"] = expect_match(meta, entry)
        summary["cases"][meta["name"]] = entry
        flag = "ok " if entry["expect_match"] else "!! "
        print(f"{flag}{meta['name']:40s} {entry['classification']:16s} exit={entry['exit']!s:5s} "
              f"{entry['wall_s']:6.1f}s phase={entry['last_phase']:14s} {(entry['stopped'] or entry['first_error'] or '')[:90]}")
        sys.stdout.flush()
    (output / "summary.json").write_text(json.dumps(summary, indent=2))

    rows = ["| case | group | expected | observed | exit | phase | output files | message |", "|---|---|---|---|---|---|---|---|"]
    for name, e in summary["cases"].items():
        msg = (e["stopped"] or e["first_error"] or "").replace("|", "\\|")
        fc = e.get("field_check", {}).get("status")
        obs = e["classification"] + (f" (field {fc})" if fc else "")
        rows.append(f"| `{name}` | {e['group']} | {e['expect']} | {obs}{'' if e['expect_match'] else ' **≠**'} | {e['exit']} | {e['last_phase']} | {e['vtu_files']} | {msg[:160]} |")
    (output / "summary.md").write_text("\n".join(rows) + "\n")
    n_ok = sum(1 for e in summary["cases"].values() if e["expect_match"])
    print(f"\n{n_ok}/{len(summary['cases'])} cases behave as the plan requires")
    if args.verify and n_ok != len(summary["cases"]):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
