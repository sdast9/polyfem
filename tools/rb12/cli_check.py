#!/usr/bin/env python3
"""RB-12 stage 3: the executable's contract, end to end, standard library only.

Registered as the CTest ``cli_contract`` so every CI platform exercises the
actual ``PolyFEM_bin`` (not the library): ``--build_info`` prints the
compiled-in identity; a public smoke completes with exit 0, its saved steps
and a ``run-manifest.json`` that is ``completed``, names this very binary
(SHA-256), the input file and the meshes it read; a resource limit reached
before allocation exits 3 with a ``resource_failure`` manifest and the
``PolyFEM stopped:`` line (RB-05); an input refused at init exits 1 with the
same line and leaves no accepted output and no manifest; ``output/manifest``
set to ``""`` writes none. Exits 1 with the first failed check.

    python3 tools/rb12/cli_check.py --binary build/PolyFEM_bin [--scene scenes/semi-implicit/quasistatic-semi.json] \
        [--output /fresh/dir] [--keep]
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
POLYFEM = HERE.parents[1]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Failure(Exception):
    pass


def check(condition, message):
    if not condition:
        raise Failure(message)


def run(cmd, cwd, log_path, timeout):
    with open(log_path, "w") as log:
        try:
            completed = subprocess.run(cmd, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
        except subprocess.TimeoutExpired:
            raise Failure(f"timeout after {timeout}s: {' '.join(map(str, cmd))}")
    return completed.returncode, Path(log_path).read_text(errors="replace")


def prepare(scene, directory, edit=None):
    directory.mkdir(parents=True)
    config = json.loads(scene.read_text())
    for asset in scene.parent.iterdir():
        if asset.suffix in (".mesh", ".obj", ".msh"):
            shutil.copy2(asset, directory / asset.name)
    if edit:
        edit(config)
    path = directory / "params.json"
    path.write_text(json.dumps(config, indent=2) + "\n")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--scene", type=Path, default=POLYFEM / "scenes" / "semi-implicit" / "quasistatic-semi.json")
    parser.add_argument("--output", type=Path, default=None, help="working directory; recreated (default: a temporary directory)")
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--keep", action="store_true", help="keep the working directory")
    args = parser.parse_args()

    binary = args.binary.resolve()
    check(binary.is_file(), f"binary {binary} does not exist")
    scene = args.scene.resolve()
    check(scene.is_file(), f"scene {scene} does not exist")
    if args.output is None:
        root = Path(tempfile.mkdtemp(prefix="polyfem-cli-contract-"))
    else:
        root = args.output.resolve()
        if root.exists():
            shutil.rmtree(root)
        root.mkdir(parents=True)
    binary_sha = sha256(binary)
    print(f"binary {binary} sha256 {binary_sha}")
    checks = 0
    try:
        # 1. --build_info
        rc, text = run([str(binary), "--build_info"], root, root / "build-info.log", args.timeout)
        check(rc == 0, f"--build_info exited {rc}")
        info = json.loads(text[text.index("{"):])
        check(info.get("schema") == "polyfem.build-info", "build info schema")
        for name in ("polyfem", "ipc_toolkit", "polysolve"):
            check(name in info["sources"], f"build info names {name}")
        check(isinstance(info["build"].get("compiler"), str) and info["build"]["compiler"], "build info compiler")
        print("build identity:", {k: (v.get("commit") or v.get("state"))[:12] for k, v in info["sources"].items()}, info["build"]["compiler"], info["build"]["configuration"])
        checks += 1

        # 2. a public smoke, single-threaded
        inp = root / "smoke" / "input"
        out = root / "smoke" / "output"
        params = prepare(scene, inp)
        out.mkdir()
        rc, text = run([str(binary), "--json", "params.json", "-o", str(out), "--log_level", "info", "--max_threads", "1"], inp, out / "run.log", args.timeout)
        check(rc == 0, f"smoke exited {rc}")
        check("total time:" in text, "smoke did not reach 'total time'")
        vtus = [p for p in out.glob("step_*.vtu") if "surf" not in p.name]
        config = json.loads(params.read_text())
        intended = config["time"]["time_steps"] if "time_steps" in config.get("time", {}) else round(config["time"]["tend"] / config["time"]["dt"])
        check(len(vtus) == intended + 1, f"smoke saved {len(vtus) - 1} steps, intended {intended}")
        manifest_path = out / "run-manifest.json"
        check(manifest_path.is_file(), "smoke wrote no run-manifest.json")
        m = json.loads(manifest_path.read_text())
        check(m.get("schema") == "polyfem.run-manifest" and m.get("version") == 1, "manifest schema/version")
        check(m["completion"]["status"] == "completed" and m["completion"]["exit_status"] == 0, f"manifest completion {m['completion']}")
        check(m["completion"]["steps_recorded"] == intended and len(m["steps"]) == intended, "manifest step count")
        check(all(s["outcome"] == "accepted" for s in m["steps"]), "every manifest step accepted")
        check(m["process"]["executable"].get("sha256") == binary_sha, "manifest executable hash is this binary")
        check(m["input"]["file"].get("sha256") == sha256(params), "manifest input file hash")
        referenced = {Path(f["path"]).name: f["sha256"] for f in m["input"]["referenced_files"]}
        for asset in inp.iterdir():
            if asset.suffix in (".mesh", ".obj", ".msh"):
                check(referenced.get(asset.name) == sha256(asset), f"manifest references {asset.name} with its hash")
        check(m["build"] == info, "manifest build identity equals --build_info")
        check(m["process"]["threads"]["effective"] == 1, "manifest effective threads")
        check(m["solver"]["model"].get("form") == "barrier-contact", "manifest model description")
        check(m["run_id"] and all(v for v in (m["started_at"], m["completion"]["finished_at"])), "manifest timestamps")
        print(f"smoke: {intended} steps, manifest completed in {m['completion']['wall_seconds']:.2f}s, peak {m['completion']['peak_rss_mb']:.0f} MB")
        checks += 1

        # 3. a resource limit reached before allocation: exit 3, resource_failure
        inp = root / "resource" / "input"
        out = root / "resource" / "output"

        def limit(config):
            ccd = config.setdefault("solver", {}).setdefault("contact", {}).setdefault("CCD", {})
            ccd["resource_limits"] = {"max_cell_items": 0, "max_candidate_emissions": 10}

        prepare(scene, inp, limit)
        out.mkdir()
        rc, text = run([str(binary), "--json", "params.json", "-o", str(out), "--log_level", "info", "--max_threads", "1"], inp, out / "run.log", args.timeout)
        check(rc == 3, f"resource failure exited {rc}, expected 3")
        check("PolyFEM stopped:" in text and "Exit status 3" in text, "resource failure log lines")
        m = json.loads((out / "run-manifest.json").read_text())
        check(m["completion"]["status"] == "resource_failure" and m["completion"]["exit_status"] == 3, f"resource manifest completion {m['completion']}")
        check("candidate_emissions" in (m["completion"]["message"] or ""), "resource manifest message names the limit")
        check(not [p for p in out.glob("step_*.vtu") if int(p.stem.split("_")[1]) > 0], "resource failure left no accepted step")
        print("resource failure: exit 3, manifest resource_failure")
        checks += 1

        # 4. an input refused at init: exit 1, PolyFEM stopped, no output, no manifest
        inp = root / "refused" / "input"
        out = root / "refused" / "output"

        def refuse(config):
            config.setdefault("contact", {})["dhat"] = 0.0

        prepare(scene, inp, refuse)
        out.mkdir()
        rc, text = run([str(binary), "--json", "params.json", "-o", str(out), "--log_level", "info", "--max_threads", "1"], inp, out / "run.log", args.timeout)
        check(rc == 1, f"refused input exited {rc}, expected 1")
        check("PolyFEM stopped:" in text and "contact.dhat must be a positive" in text, "refused input log lines")
        check(not list(out.glob("*.vtu")) and not (out / "run-manifest.json").exists(), "refused input left no output and no manifest")
        print("refused input: exit 1, no output")
        checks += 1

        # 5. output/manifest = "" disables the manifest
        inp = root / "nomanifest" / "input"
        out = root / "nomanifest" / "output"

        def disable(config):
            config.setdefault("output", {})["manifest"] = ""
            config["time"] = {"dt": config["time"]["dt"], "time_steps": 1}

        prepare(scene, inp, disable)
        out.mkdir()
        rc, text = run([str(binary), "--json", "params.json", "-o", str(out), "--log_level", "info", "--max_threads", "1"], inp, out / "run.log", args.timeout)
        check(rc == 0, f"no-manifest run exited {rc}")
        check(not (out / "run-manifest.json").exists(), "output/manifest = \"\" still wrote a manifest")
        print("output/manifest \"\": no manifest, exit 0")
        checks += 1
    except Failure as e:
        print(f"FAILED: {e}", file=sys.stderr)
        print(f"working directory kept at {root}", file=sys.stderr)
        return 1
    print(f"cli_contract: {checks} checks passed")
    if not args.keep:
        shutil.rmtree(root, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
