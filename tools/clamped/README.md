# Clamped-contact measurement tools

Record: [docs/clamped-contacts-20260928.md](../../docs/clamped-contacts-20260928.md).
Python 3.11+ with NumPy. They reuse EF-01's scene table and driver
(`tools/ef01`, `tools/ef02/run.py`) and change no default or scene file.

```sh
W=/path/to/fable_polyfem; E=/fresh/evidence; B=$E/bin/PolyFEM_bin
python3 tools/clamped/sequence.py --workspace "$W" --out "$E" --binary "$B" --clamped keep
python3 tools/clamped/sequence.py --workspace "$W" --out "$E" --binary "$B" --clamped exclude_statistics
python3 tools/clamped/bb_resume.py --template-dir "$W/ef07-work/scene" --state state_30.hdf5 \
  --out "$E" --binary "$B" --clamped keep bb31-keep-rms
python3 tools/clamped/reduce.py "$E"/runs/* --json "$E/clamped.json"
python3 tools/clamped/compare.py "$E" --json "$E/compare.json"
```

* `sequence.py` always states the controller (`--controller rms` by default:
  `band_statistic rms`, `initial_trim_estimate false`). The R4 scene file was
  re-exported on 2026-09-26 with the force-weighted mode, so a run that relies
  on the scene file does not run the production controller.
* `bb_resume.py` resumes EF-07's ball-burst step-31 state (R0's `state_30`),
  with trim predictors on and physical diagnostics off (their coefficient
  events exceed 10 GB per step).
* `reduce.py` reads the observational `clamped` block of every trim-predictor
  record: collision classes (free / partly / fully clamped by stencil
  vertices), and the controller statistics with and without the fully clamped
  collisions (band statistic, minimum gap, collapse proxy and decision,
  force-weighted gap, coefficient batch median, gradient-balance trim over all
  DOFs, without fully clamped collisions and on free DOFs only). A run without
  the block is reported as missing, not as zero.
* `compare.py` compares each exclude mode with `keep`: iterations, stall
  retunes, AL passes, balance flags, VTU byte identity and relative solution
  difference per step. R1, BBT, IT, R4 and BB are not reproducible run to run
  on this host (see the record), so single-run differences need the repeats.
