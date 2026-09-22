# BFGS curvature corpus

The fixed corpus of [audit stage 1](../../docs/bfgs-curvature-safeguard-20260922.md):
it chose the curvature policy, the relative threshold and the restart bound,
and it is what any later change to those should be re-run against.

`corpus.cpp` builds the real PolySolve strategies, line searches and factory
and emits one JSON object per row on stdout. It runs

- the two bounded polynomials of the audit, Rosenbrock from `(-1.2, 1)` and
  positive quadratics at curvature `1e-8 … 1e8` in three variables,
- as a lone strategy and as the public string form, which appends gradient
  descent as a fallback,
- across both curvature policies and all three energy line searches (192 rows),
  a `curvature_restart` sweep (256), a `curvature_tolerance` sweep (64) and
  eight per-iteration traces.

Each row carries the status, iteration count, final gradient norm, the guard's
accepted / damped / refused counts with their reasons, and the discarded
approximations. **Exit zero means the corpus completed, not that every solve
converged**; a row's `converged` field and `error` say what happened, and rows
that do not converge are part of the recorded comparison.

From the PolyFEM repository, with a rebuilt macOS Makefiles `build`:

```sh
python3 tools/bfgs_curvature/run_corpus.py --build build --output /absolute/fresh/evidence
```

The runner requires a fresh output directory. It compiles the corpus and the
three safeguarded translation units against the configured headers, links them
ahead of the libraries the build already produced, and records the commands,
the source/library/executable hashes and the PolySolve revision and dirty
status. It targets this workspace's macOS Accelerate/Makefiles build; it is not
a cross-platform test runner, and it loads no geometry, user scene or Teseo.

The committed results of the run that decided the defaults, together with the
before/after scene runs and the test records, are in the parent workspace's
`outputs/bfgs-stage1/20260922T113940Z/`.
