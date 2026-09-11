#!/usr/bin/env python3
"""Summarize RB-18 smoke endpoints for before/after comparison.

For each scene directory under the given smoke output root, read the last
``step_N.vtu`` and extract the ``solution`` point array (base64 Float64,
UInt64 header, uncompressed as PolyFEM writes it). Emit per-scene: number of
steps, exit code (from run.log presence of the finish line), max |u|, sum of u,
and a SHA-256 of the raw solution bytes. With ``--compare`` also print the max
absolute per-component difference against a previous summary.

This is bounded evidence extraction, not a physical acceptance criterion.
"""
import argparse
import base64
import hashlib
import json
import re
import struct
import sys
from pathlib import Path


def read_solution(vtu: Path):
    text = vtu.read_text()
    m = re.search(
        r'<DataArray[^>]*Name="solution"[^>]*NumberOfComponents="(\d+)"[^>]*format="binary"[^>]*>\s*([A-Za-z0-9+/=]+)',
        text)
    if m is None:
        return None, None
    ncomp = int(m.group(1))
    raw = base64.b64decode(m.group(2))
    (nbytes,) = struct.unpack("<Q", raw[:8])
    payload = raw[8:8 + nbytes]
    vals = struct.unpack("<%dd" % (nbytes // 8), payload)
    return ncomp, vals


def summarize(root: Path):
    out = {}
    for scene_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        steps = sorted(scene_dir.glob("step_*.vtu"),
                       key=lambda p: int(p.stem.split("_")[1]))
        entry = {"steps": len(steps)}
        log = scene_dir / "run.log"
        if log.exists():
            txt = log.read_text(errors="replace")
            entry["error_lines"] = txt.count("[error]")
            m = re.findall(r"Linf error: ([0-9.eE+-]+)", txt)
            if m:
                entry["linf_error_last"] = m[-1]
            entry["conditioning_cap_events"] = txt.count("Conditioning cap on first contact")
            entry["stall_restarts"] = txt.count("retuning barrier stiffness and restarting")
            trims = re.findall(r"Refreshed semi-implicit barrier stiffness over \d+ contacts: [^\n]*\(trim=([0-9.eE+-]+)\)", txt)
            if trims:
                entry["refresh_trims"] = trims
        if steps:
            ncomp, vals = read_solution(steps[-1])
            if vals is not None:
                entry["ncomp"] = ncomp
                entry["n_values"] = len(vals)
                entry["max_abs_u"] = max(abs(v) for v in vals)
                entry["sum_u"] = sum(vals)
                entry["solution_sha256"] = hashlib.sha256(
                    struct.pack("<%dd" % len(vals), *vals)).hexdigest()
                entry["_values"] = vals
        out[scene_dir.name] = entry
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--compare", type=Path)
    args = ap.parse_args()
    summary = summarize(args.root)
    if args.compare:
        prev = json.loads(args.compare.read_text())
        for name, e in summary.items():
            p = prev.get(name)
            if p is None or "_values" not in e or "_values" not in p:
                e["max_abs_diff_vs_compare"] = None
                continue
            a, b = e["_values"], p["_values"]
            e["max_abs_diff_vs_compare"] = (
                max(abs(x - y) for x, y in zip(a, b)) if len(a) == len(b) else None)
            e["identical_vs_compare"] = e.get("solution_sha256") == p.get("solution_sha256")
    args.output.write_text(json.dumps(summary, indent=1))
    for name, e in summary.items():
        print(name, {k: v for k, v in e.items() if k != "_values"})
    return 0


if __name__ == "__main__":
    sys.exit(main())
