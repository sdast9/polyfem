#!/usr/bin/env python3
"""EF-07 reducer: attribute every trim move of a run to its controller branch.

Usage: reduce.py <run output dir> [--steps a-b] [--json out.json]

Reads trim-predictors.jsonl (output/trim_predictors) and log.txt from the run
output directory. Per step: accepted Newton iterations, stall restarts and
whether the restart budget ran out, trim moves by source and direction,
direction reversals, trim range and the maximum excursion from the step's
first trim, collapse-proxy firing, and the minimum gap. Records written by the
EF-07 binary also carry the minimum-gap pair (collapse_pairs), which gives the
pair's birth status and whether the same pair stays the minimum across trim
changes.

A move is the change of trim between consecutive records. Its source is the
record's controller_decision when that decision produced the new trim
(collapse, force band iteration/refresh/stall, initial estimate); a change
without such a decision is 'collapse?' on an iteration record (the pre-EF-07
binary emitted no collapse decision) or the record's event otherwise.
"""
import argparse
import collections
import json
import math
import os
import re
import sys


def load_records(path):
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                yield json.loads(line)


def attribute(prev_trim, rec):
    d = rec.get("controller_decision") or {}
    trim = rec["trim"]
    if d and d.get("after") is not None and math.isclose(d["after"], trim, rel_tol=1e-12) \
            and not math.isclose(d.get("before", trim), trim, rel_tol=1e-12):
        src = d.get("source", "?")
        if src == "collapse":
            return "collapse:" + str(d.get("context"))
        if src in ("iteration", "refresh", "refresh_endpoint", "stall"):
            return "band:" + src
        return src
    if rec["event"] == "iteration":
        return "collapse?"
    return rec["event"]


def log_steps(log_path):
    """Per step from log.txt: stall restarts, budget exhaustion, final status."""
    out = collections.defaultdict(lambda: {"stalls": 0, "exhausted": False, "finish": None, "ls_failed_residual": 0})
    step = None
    pat_step = re.compile(r"Rollback point of step (\d+)")
    if not os.path.exists(log_path):
        return out
    with open(log_path, errors="replace") as f:
        for line in f:
            m = pat_step.search(line)
            if m:
                step = int(m.group(1))
                continue
            if step is None:
                continue
            if "Line-search stall detected" in line:
                out[step]["stalls"] += 1
            elif "Line-search stall persisted" in line or "subsolve interrupted" in line:
                out[step]["exhausted"] = True
            elif "large (or nan) linear solve" in line:
                out[step]["ls_failed_residual"] += 1
            elif "Finished:" in line and "[error]" in line:
                out[step]["finish"] = line.split("Finished: ", 1)[1].strip()[:60]
    return out


def reduce(run, steps=None):
    recs = list(load_records(os.path.join(run, "trim-predictors.jsonl")))
    per = collections.OrderedDict()
    prev_trim = None
    prev_pair = None
    for r in recs:
        s = r.get("step")
        if steps and not (steps[0] <= s <= steps[1]):
            prev_trim = r["trim"]
            continue
        st = per.setdefault(s, {
            "records": 0, "iterations": 0, "moves": collections.Counter(), "up": 0, "down": 0,
            "reversals": 0, "last_dir": 0, "trim_first": prev_trim if prev_trim is not None else r["trim"],
            "trim_min": math.inf, "trim_max": 0, "min_gap_min": math.inf, "min_gap_max": 0,
            "proxy_fired": 0, "iter_records": 0, "min_pair_born": 0, "min_pair_seen": 0,
            "min_pair_same_across_move": 0, "min_pair_moves_checked": 0, "min_pair_gap_response": [],
            "below_pair_threshold_max": 0, "active_max": 0, "active_min": math.inf, "estimates_accepted": 0,
            "estimates_rejected": 0, "collapse_vetoed": 0, "band_guarded": 0})
        st["records"] += 1
        trim = r["trim"]
        st["trim_min"] = min(st["trim_min"], trim)
        st["trim_max"] = max(st["trim_max"], trim)
        st["active_max"] = max(st["active_max"], r.get("active_count", 0))
        st["active_min"] = min(st["active_min"], r.get("active_count", 0))
        g = r.get("gap") or {}
        if "min" in g:
            st["min_gap_min"] = min(st["min_gap_min"], g["min"])
            st["min_gap_max"] = max(st["min_gap_max"], g["min"])
        d = r.get("controller_decision") or {}
        if r["event"] == "initial_estimate":
            st["estimates_accepted" if d.get("accepted") else "estimates_rejected"] += 1
        if d.get("responsiveness_veto"):
            st["collapse_vetoed"] += 1
        if d.get("collapse_guard") and d.get("source") != "initial_estimate":
            st["band_guarded"] += 1
        if r["event"] == "iteration":
            st["iter_records"] += 1
            if "min" in g and 100 * g["min"] ** 2 < 0.5:
                st["proxy_fired"] += 1
        cp = r.get("collapse_pairs") or {}
        mp = cp.get("min_pair")
        if mp:
            st["min_pair_seen"] += 1
            st["min_pair_born"] += bool(mp.get("born_since_previous_iterate"))
            st["below_pair_threshold_max"] = max(st["below_pair_threshold_max"], cp.get("below_collapse_pair_gap", 0))
        if prev_trim is not None and trim != prev_trim:
            src = attribute(prev_trim, r)
            direction = 1 if trim > prev_trim else -1
            st["moves"][src + (":up" if direction > 0 else ":down")] += 1
            st["up" if direction > 0 else "down"] += 1
            if st["last_dir"] and direction != st["last_dir"]:
                st["reversals"] += 1
            st["last_dir"] = direction
            if mp and prev_pair:
                st["min_pair_moves_checked"] += 1
                if mp["full_vertex_ids"] == prev_pair["full_vertex_ids"]:
                    st["min_pair_same_across_move"] += 1
        if r["event"] == "iteration" and r.get("iteration") is not None:
            st["iterations"] = max(st["iterations"], 0) + 1
        prev_trim = trim
        prev_pair = mp
    logs = log_steps(os.path.join(run, "log.txt"))
    rows = []
    for s, st in per.items():
        lg = logs.get(s, {"stalls": 0, "exhausted": False, "finish": None, "ls_failed_residual": 0})
        excursion = max(st["trim_max"] / st["trim_first"], st["trim_first"] / st["trim_min"]) if st["trim_first"] > 0 else None
        rows.append({
            "step": s, "iteration_records": st["iter_records"], "stall_restarts": lg["stalls"],
            "restart_budget_exhausted": lg["exhausted"], "finish": lg["finish"],
            "linear_solve_residual_failures": lg["ls_failed_residual"],
            "trim_moves": st["up"] + st["down"], "up": st["up"], "down": st["down"], "reversals": st["reversals"],
            "moves_by_source": dict(st["moves"]), "trim_first": st["trim_first"], "trim_min": st["trim_min"],
            "trim_max": st["trim_max"], "max_excursion_from_step_start": excursion,
            "proxy_fired_fraction": st["proxy_fired"] / st["iter_records"] if st["iter_records"] else None,
            "min_gap_range": [st["min_gap_min"], st["min_gap_max"]],
            "active_range": [st["active_min"], st["active_max"]],
            "min_pair_born_fraction": st["min_pair_born"] / st["min_pair_seen"] if st["min_pair_seen"] else None,
            "min_pair_same_across_moves": [st["min_pair_same_across_move"], st["min_pair_moves_checked"]],
            "below_pair_threshold_max": st["below_pair_threshold_max"],
            "estimates": [st["estimates_accepted"], st["estimates_rejected"]],
            "collapse_vetoed": st["collapse_vetoed"], "band_guarded": st["band_guarded"]})
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--steps")
    ap.add_argument("--json")
    a = ap.parse_args()
    steps = tuple(int(x) for x in a.steps.split("-")) if a.steps else None
    rows = reduce(a.run, steps)
    print("step iters stalls exh moves up/down rev trim[first..min..max] excursion proxy% mingap[min,max] active born% samepair")
    for r in rows:
        print("%4d %5d %4d %3s %5d %4d/%-4d %3d %9.3g %9.3g %9.3g %9.3g %5s [%.3f,%.3f] [%d,%d] %s %s" % (
            r["step"], r["iteration_records"], r["stall_restarts"], "Y" if r["restart_budget_exhausted"] else "-",
            r["trim_moves"], r["up"], r["down"], r["reversals"], r["trim_first"], r["trim_min"], r["trim_max"],
            r["max_excursion_from_step_start"] or float("nan"),
            "%.0f" % (100 * r["proxy_fired_fraction"]) if r["proxy_fired_fraction"] is not None else "-",
            r["min_gap_range"][0], r["min_gap_range"][1], r["active_range"][0], r["active_range"][1],
            "%.2f" % r["min_pair_born_fraction"] if r["min_pair_born_fraction"] is not None else "-",
            "%d/%d" % tuple(r["min_pair_same_across_moves"])))
        print("       moves:", r["moves_by_source"], "estimates acc/rej:", r["estimates"],
              "vetoed:", r["collapse_vetoed"], "guarded:", r["band_guarded"])
    if a.json:
        json.dump(rows, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
