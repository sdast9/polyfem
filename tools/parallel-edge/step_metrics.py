#!/usr/bin/env python3
"""Per-time-step Newton summary from a PolyFEM --log_level debug log.

usage: step_metrics.py run.log [--table]
Prints one JSON summary line (steps, total/max iterations, worst final
||grad||, limit hits, step index of the worst) and with --table one row per
step: step, iterations, final ||grad||, stopping reason.
"""
import re, sys, json

FIN = re.compile(r"Finished: (.*?) took [0-9.e+-]+s \(iters=(\d+) .*?‖∇f‖=([0-9.e+-]+|nan)")
def parse(path):
    rows = []
    limit_hits = 0
    for line in open(path, errors="replace"):
        if "Reached iteration limit" in line:
            limit_hits += 1
        m = FIN.search(line)
        if m and "[SparseNewton]" in line or (m and "Newton" in line):
            rows.append((len(rows), int(m.group(2)), float(m.group(3)), m.group(1)))
    return rows, limit_hits

if __name__ == "__main__":
    rows, limit_hits = parse(sys.argv[1])
    its = [r[1] for r in rows]
    worst = max(rows, key=lambda r: r[2]) if rows else None
    print(json.dumps({"solves": len(rows), "total_iters": sum(its), "max_iters": max(its, default=0),
                      "limit_hits": limit_hits,
                      "worst_final_grad": worst[2] if worst else None,
                      "worst_step": worst[0] if worst else None}))
    if "--table" in sys.argv:
        for r in rows:
            print(f"{r[0]:4d} {r[1]:4d} {r[2]:.3e} {r[3]}")
