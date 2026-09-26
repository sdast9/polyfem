# EF-02/03 measurement tools

Use Python 3.11 or newer with NumPy (the measured host used Python 3.14.7).

These tools reuse EF-01's scene preparation, solver driver, and solution reader.
They do not change any production defaults or scene files. Set `W` to the parent
workspace (containing `polyfem/` and `test_cases/`), and `E` to a fresh evidence
folder. Copy binaries into `E/bin/` and save their hashes before running. Build
first; do not run a build or another solver while a sequence is running.

```sh
python3 tools/ef02/sequence.py --workspace "$W" --out "$E" \
  --binary "$E/bin/PolyFEM_bin-production" --mode production
python3 tools/ef02/sequence.py --workspace "$W" --out "$E" \
  --binary "$E/bin/PolyFEM_bin-candidate" --mode candidate
python3 tools/ef02/reduce.py "$E" --ef01 "$W/outputs/ef-01/20260923T145726Z"
python3 tools/ef02/compare.py "$E" --candidate-prefix ""
python3 tools/ef04/ef04_tables.py "$E" --ef01 "$W/outputs/ef-01/20260923T145726Z" \
  --ref IT=production-IT-s200-r1 --ref BB=production-BB-s1-r1
python3 tools/ef02/smoke_identity.py --workspace "$W" --out "$E/identity" \
  --production "$E/bin/PolyFEM_bin-production" --candidate "$E/bin/PolyFEM_bin-candidate"
```

`pilot` runs R1, BBT and one R4 step. `screen` runs R1, BBT, held-out IT/BB,
and the five smokes. `production`/`candidate` run the complete matrix, including
two one-step R4 runs and two five-step R4 runs. `--prefix` retains distinct
exploratory configurations. Every run gets a unique label; existing directories
are refused rather than silently reused. The sequence ledger is written before
execution, in addition to EF-01's per-run `row.json`. Solver failures remain in
the matrix; they do not terminate the sequence or become successful evidence.

R4 and BB use all available threads; other scenes use one. R4 has a 45-minute
run cap. `pmset` captures power status on this macOS host. Inspect sleep logs
separately before trusting wall times. The option-off identity check compares
all exported VTU bytes, not timing/provenance fields in manifests.

`reduce.py` reports the original [.35,.50] target occupancy and expanded
[.325,.525] occupancy separately. Only accepted-iteration observations enter
those denominators. Trim direction reversals reset at step boundaries. The
explicit `initial_estimate` and `force_band` events identify new decisions;
other events retain their own attribution even when they include a previous
decision as context. Missing output is not a pass. These summaries do not
replace the acceptance-by-acceptance assessment in the validation record.

`compare.py` requires `ef02-metrics.json` from the reducer and compares every
candidate/production pair within each scene and requested step count. Use
`--candidate-prefix v3-` for retained versioned runs. Non-R4 `screen` runs
can supply the candidate matrix. It reports missing solution/balance steps,
per-step balance changes, both repeat spreads, and the conservative cost
check (largest candidate count <= smallest production count). A single
production run does not establish a repeat envelope; accuracy interpretation
remains explicit in the validation record. R3 is excluded from acceptance.
