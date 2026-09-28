#!/usr/bin/env python3
"""CI-09: a portable smoke runner for the semi-implicit scenes.

Standard library only, so it runs unmodified on any Python 3 that can launch
the target ``PolyFEM_bin`` -- a developer workstation, a packaged CLI on a
clean machine, or a CI runner. It replaces the developer-only zsh
``run-smoke.sh`` referenced by ``docs/ci-portability-plan.md`` (CI-09): the
same executable, explicit input/output directories and aggregate failure
status, without an assumed build layout or shell.

For each scene it copies that scene's directory (json plus mesh/obj/msh
assets) into its own input directory, runs

    <binary> --json <scene>.json -o <outdir> --log_level debug [--max_threads N]

as an argument list (no shell), captures the log, and reports the exit code,
the number of ``[error]`` log lines, the last few lines describing
semi-implicit/AL solver behaviour, the worst (smallest) minimum-distance
readings, and wall time. Every scene runs from its own directory so paths
with spaces or non-ASCII characters work like any other path.

    python3 tools/smoke/run_smoke.py --binary build-cloud/PolyFEM_bin \\
        --output outputs/smoke/20260928T000000Z
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
POLYFEM = HERE.parent.parent
DEFAULT_SCENES = POLYFEM / "scenes" / "semi-implicit"
ASSET_SUFFIXES = (".mesh", ".obj", ".msh")

LOG_LINE_PATTERN = re.compile(
    r"Semi-implicit barrier stiffness|Refreshed semi-implicit|hessian-scaled initial AL|stall",
    re.IGNORECASE,
)
MIN_DISTANCE_PATTERN = re.compile(r"Minimum distance during solve:\s*([0-9.eE+-]+)")
ERROR_LINE_PATTERN = re.compile(r"\[error\]")


def discover_scenes(scenes_dir, names):
    if names:
        result = []
        for name in names:
            stem = name[:-5] if name.endswith(".json") else name
            path = scenes_dir / f"{stem}.json"
            if not path.is_file():
                raise SystemExit(f"scene not found: {path}")
            result.append((stem, path))
        return result
    return sorted(
        ((path.stem, path) for path in scenes_dir.glob("*.json")),
        key=lambda item: item[0],
    )


def prepare_input(scene_path, scenes_dir, input_dir):
    input_dir.mkdir(parents=True)
    shutil.copy2(scene_path, input_dir / scene_path.name)
    for asset in scenes_dir.iterdir():
        if asset.suffix in ASSET_SUFFIXES:
            shutil.copy2(asset, input_dir / asset.name)


def run_scene(binary, scene_name, scene_path, scenes_dir, scene_root, threads, timeout):
    input_dir = scene_root / "input"
    output_dir = scene_root / "output"
    prepare_input(scene_path, scenes_dir, input_dir)
    output_dir.mkdir(parents=True)

    cmd = [str(binary), "--json", scene_path.name, "-o", str(output_dir), "--log_level", "debug"]
    if threads is not None:
        cmd += ["--max_threads", str(threads)]

    log_path = scene_root / "run.log"
    start = time.monotonic()
    with open(log_path, "w", encoding="utf-8", errors="replace") as log:
        try:
            completed = subprocess.run(cmd, cwd=input_dir, stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
            exit_code = completed.returncode
            timed_out = False
        except subprocess.TimeoutExpired:
            exit_code = None
            timed_out = True
    wall_seconds = time.monotonic() - start

    text = log_path.read_text(encoding="utf-8", errors="replace")
    error_lines = len(ERROR_LINE_PATTERN.findall(text))
    matching_lines = [line.strip() for line in text.splitlines() if LOG_LINE_PATTERN.search(line)]
    last_matching_lines = matching_lines[-5:]
    min_distances = [float(m) for m in MIN_DISTANCE_PATTERN.findall(text)]
    worst_min_distances = sorted(min_distances)[:3]

    return {
        "scene": scene_name,
        "command": cmd,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "wall_seconds": wall_seconds,
        "error_lines": error_lines,
        "last_matching_lines": last_matching_lines,
        "worst_min_distances": worst_min_distances,
        "log": str(log_path),
    }


def write_markdown(summary, path):
    lines = [
        "# Portable smoke run",
        "",
        f"binary `{summary['binary']}`; scenes `{summary['scenes_dir']}`; threads `{summary['threads']}`",
        "",
        "| scene | exit | error lines | worst min distances | wall (s) |",
        "| --- | --- | --- | --- | --- |",
    ]
    for r in summary["results"]:
        worst = ", ".join(f"{v:.3e}" for v in r["worst_min_distances"]) or "-"
        exit_code = "timeout" if r["timed_out"] else r["exit_code"]
        lines.append(f"| {r['scene']} | {exit_code} | {r['error_lines']} | {worst} | {r['wall_seconds']:.2f} |")
    lines += ["", "## Last matching solver log lines", ""]
    for r in summary["results"]:
        lines.append(f"### {r['scene']}")
        lines.append("")
        if r["last_matching_lines"]:
            for line in r["last_matching_lines"]:
                lines.append(f"    {line}")
        else:
            lines.append("    (none)")
        lines.append("")
    path.write_text("\n".join(lines) + "\n")


def print_table(results):
    header = f"{'scene':<28} {'exit':>6} {'errors':>7} {'worst min distances':>34} {'wall (s)':>9}"
    print(header)
    print("-" * len(header))
    for r in results:
        worst = ", ".join(f"{v:.3e}" for v in r["worst_min_distances"]) or "-"
        exit_code = "timeout" if r["timed_out"] else r["exit_code"]
        print(f"{r['scene']:<28} {str(exit_code):>6} {r['error_lines']:>7} {worst:>34} {r['wall_seconds']:>9.2f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--scenes", type=Path, default=DEFAULT_SCENES, help="directory of scene JSON files and their assets (default: scenes/semi-implicit)")
    parser.add_argument("--output", type=Path, required=True, help="fresh directory for isolated inputs, outputs and the summary (refused if non-empty)")
    parser.add_argument("--threads", type=int, default=None, help="passed to the binary as --max_threads; omitted means the binary's own default")
    parser.add_argument("--timeout", type=float, default=1800.0)
    parser.add_argument("scene_names", nargs="*", help="scene names to run (default: every *.json in --scenes)")
    args = parser.parse_args()

    binary = args.binary.resolve()
    if not binary.is_file():
        raise SystemExit(f"binary not found: {binary}")
    scenes_dir = args.scenes.resolve()
    if not scenes_dir.is_dir():
        raise SystemExit(f"scenes directory not found: {scenes_dir}")

    output = args.output.resolve()
    if output.exists():
        if any(output.iterdir()):
            raise SystemExit(f"--output {output} already exists and is not empty; use a fresh directory")
    else:
        output.mkdir(parents=True)

    scenes = discover_scenes(scenes_dir, args.scene_names)
    if not scenes:
        raise SystemExit(f"no scenes found in {scenes_dir}")

    results = []
    for scene_name, scene_path in scenes:
        result = run_scene(binary, scene_name, scene_path, scenes_dir, output / scene_name, args.threads, args.timeout)
        results.append(result)
        exit_code = "timeout" if result["timed_out"] else result["exit_code"]
        print(f"{scene_name}: exit {exit_code}, {result['error_lines']} error line(s), {result['wall_seconds']:.2f}s", flush=True)

    summary = {
        "binary": str(binary),
        "scenes_dir": str(scenes_dir),
        "threads": args.threads,
        "results": results,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    write_markdown(summary, output / "summary.md")

    print()
    print_table(results)

    failed = any(r["timed_out"] or r["exit_code"] != 0 or r["error_lines"] > 0 for r in results)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
