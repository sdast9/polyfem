#!/usr/bin/env python3
"""usage: summarize_sweep.py sweep.txt [...]  -- fail count, >1.2e-5 count, quantiles."""
import re, sys
for f in sys.argv[1:]:
    lines = [l for l in open(f).read().splitlines() if "max_rel_dev" in l]
    d = sorted(float(m.group(1)) for l in lines for m in [re.search(r"max_rel_dev=([0-9.e+-]+)", l)] if m)
    fail = sum("n/a" in l for l in lines)
    n = len(d)
    print(f.split("/")[-1], "runs", len(lines), "fail(n/a)", fail, "dev>1.2e-5:", sum(x > 1.2e-5 for x in d),
          "median %.2e p90 %.2e max %.2e" % (d[n // 2], d[int(.9 * n)], d[-1]))
