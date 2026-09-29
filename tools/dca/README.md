# Default-controller assessment tools (2026-09-29)

These tools produced
[docs/default-controller-assessment-20260929.md](../../docs/default-controller-assessment-20260929.md).
They change no solver default or scene file. Paths default to the parent workspace
`/Users/stevenabramowitch/Downloads/fable_polyfem` and its evidence folder
`default-controller-work/`. Edit `W` and the default arguments to reuse them elsewhere.

| file | purpose |
|---|---|
| `dca_run.py` | Runs one arm on one scene through `tools/ef02/run.py` (the EF-01 driver). The arms are P (rms, no estimate), E (estimate), Er (estimate, scope run), F (force_weighted + pair), B (both) and Br. Every arm sets the controller explicitly, whatever the scene file says. `--realization acc1/cholmod/simplicial` switches the linear solver for deterministic single-threaded ensembles. It adds the scenes PP (`pup_push`), FB (`fibers`), MT (`mesh_tissue`) and ITF (IT with friction 0.3). |
| `dca_reduce.py` | Writes `RUN/qoi.json`: EF-01/EF-02 metrics, force-band decisions and vetoes, estimate acceptances and factors, trim excursion, and per-step observables (kinetic/elastic/barrier energy, net AL reaction, contact-force norm, gaps, trim, balance flag). |
| `dca_compare.py` | Ensemble comparison for runs named `ARM-GROUP-REAL`. For each arm it reports ρ = Σ D_X / Σ E_P (cross distance over production's own realization spread, realization-matched pairs excluded) and observable deviations. `--ref` gives the error against a pinned-trim reference; `--offset` handles resumed runs. |
| `queue2.sh` | Runs a job file (`label scene arm steps threads realization [timeout] [extra args]`) with N jobs in parallel and skips finished runs. Needs bash 4.3+ (`wait -n`). |
| `bb_run.sh`, `bb_summarize.py` | Resume the ball-burst from EF-07 R0's `state_30` for steps 31–32 with one arm, then give a per-step summary (iterations, trim moves, reversals, excursion, minimum gap). |

A resume example is IT from `state_20`:

```sh
python3 dca_run.py rt-B-IT-resume20 --scene IT --arm B --steps 10 \
  --set '/input/data/state="…/state_20.hdf5"' --set /time/t0=0.5 \
  --set /time/tend=0.75 --set /output/data/file_index_offset=20
```

Physical-diagnostics step numbers are local to the resumed run; VTU names use the offset.
