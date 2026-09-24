# EF-01 trim / AL-weight survey tools

Runners and reducers for [ef-01-trim-survey.md](../../docs/ef-01-trim-survey.md)
(contact-efficiency plan, item EF-01). They build on
[`../qn_contact/`](../qn_contact/README.md) (same scene table and input builder)
and change no solver default; every deviation from a scene file is in the run's
`row.json`.

| script | what it does |
| --- | --- |
| `ef01_run.py LABEL --out DIR --scene S [--trim T] [--steps N] [--threads T] [--timeout S] [--set /ptr=value ...] [--no-predictors]` | one Newton run into `DIR/runs/LABEL`; `--trim` pins `trim_min = trim_max = T`; turns on `output/trim_predictors` (omit with `--no-predictors` for binaries that predate it), iteration and physical diagnostics; discards `coefficient-events.jsonl` to `/dev/null` (17 GB per production R4 run); removes the spec-refused `space/pressure_discr_order` of the BBT export and rewrites a lone `{"file": X}` selection to `"X"` (same FileSelection, avoids the const `json::operator[]` crash in `Selection::build`) |
| `ef01_reduce.py [--ref RUN] [--json OUT] RUN...` | per run: iterations per step (all sub-solves), restarts by trigger, trial-cap / CCD binding, AL weights, assembly/solve time, per-step trim trajectory, endpoint gap statistics, `physical_balance_pass`, relative L2 error against `--ref` |
| `ef01_tables.py DIR [--scene S ...]` | per-scene tables (pins, production, repeats, AL runs) with accuracy against `ref-SCENE`; writes `DIR/tables.md` and `DIR/metrics.json` |
| `ef01_predictors.py RUN` | every full `trim-predictors.jsonl` record (refreshes and stall retunes): κ_gb and cosine, Hessian-ratio trims, conditioning-cap trim, rms / force-weighted gap, multiplicity |
| `ef01_best.py DIR SCENE` | the trim of the scene's cheapest completed pinned run |

Scenes (in addition to `qn_contact`'s R1 / R3 / R4 / smoke-qs): `smoke-tr`
(`scenes/semi-implicit/transient-semi.json`), `BBT`
(`test_cases/ballburst_thin_membrane/input`), `IT` (`test_cases/inertia_test`)
and `BB` (`test_cases/ball_burst`) — the last two held out for EF-02/03
acceptance. The `test_cases` inputs are the user's Houdini exports and are not
in this repository.
