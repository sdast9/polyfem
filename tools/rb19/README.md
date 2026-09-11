# RB-19 step-1 stall reproduction

Reruns the RB-04 refinement case that stalled at step 1 before contact
(`quasistatic-semi`, `dt=.0625`, trim band `[.1,.9]`) with different thread
counts, so the energy's summation-order noise and the line search's roundoff
fallback can be measured directly. Read the
[validation record](../../docs/rb-19-line-search-roundoff.md) first.

From `polyfem/`, into a fresh output path:

```bash
python3 tools/rb19/run_step1_threads.py /absolute/fresh/output default default default 1 1 1
```

Each trailing argument is one run (`default` = no `--max_threads`). Runs use
the current `build/PolyFEM_bin` at trace level; the summary counts stall
restarts, saved steps and `accepted on gradient norm` fallback lines. The
[compact results](results-20260911-step1-threads.json) hold the pre-fix
(PolySolve `4d372fa8`) and post-fix (`5afe3b5d`) runs from 2026-09-11 with
timing-free iteration-log hashes: single-threaded runs are bit-identical,
threaded runs never are. Full logs stay under `outputs/rb-19/` in the parent
workspace. These are measurements, not goldens.
