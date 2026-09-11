# RB-20 / RB-21 drift matrix and ball-on-plate runners

`run_drift_matrix.py` runs a public semi-implicit scene over a dt × trim-band
matrix with `semi_implicit` overrides and extracts, from each run's
`coefficient-events.jsonl`, the fixed-coordinate contact-force change at every
between-steps refresh (the RB-04 post-publication drift). `run_ball_on_plate.py`
does the same for the user's ball-on-plate scene (`test_cases/input/params.json`,
bounded to a few steps). Read the records first:
[RB-20](../../docs/rb-20-force-continuation.md),
[RB-21](../../docs/rb-21-parent-keyed-kappa.md).

From `polyfem/`, into fresh output paths:

```bash
python3 tools/rb20/run_drift_matrix.py --scene quasistatic-semi --output /abs/fresh/on
python3 tools/rb20/run_drift_matrix.py --scene quasistatic-semi --output /abs/fresh/off --override force_continuation=false
python3 tools/rb20/run_ball_on_plate.py --output /abs/fresh/ball --steps 8
```

`--override key=json` sets `solver.contact.semi_implicit.<key>` (e.g.
`coefficient_identity='"stencil"'`, `continuation_max_ratio=2`). The
[compact results](results-20260911.json) hold the 2026-09-11 matrices
(quasistatic 3×3 on/off/stencil, transient, friction, max-ratio), the two
ball-on-plate runs and the smoke endpoint comparisons against RB-19. These are
measurements, not goldens; drift is a fixed-coordinate force identity, not a
physical balance, and a completed run is numerical termination only.
