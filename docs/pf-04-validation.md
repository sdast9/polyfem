# PF-04: material input snapshots — 2026-09-07

## Ownership and scope

Scalar matrix files (`ExpressionValue`) and normalized per-element fiber fields
now use an explicitly owned `MaterialFileCache`. VarForm and legacy State create
one snapshot at input initialization, retain it during material rebuilds and
remeshing, and pass it to every assembler material setup. Coupled thermal/fluid
assemblers and the nested FSI solid formulation share their simulation's snapshot.
Reinitializing input creates a fresh snapshot. Existing values remain valid.

The snapshot is lazy: each resolved path (or path plus fiber field) is captured
at its first successful read. Files must be finalized before material setup;
this is not an atomic filesystem snapshot of all files at the instant of `init`.
There are no timestamp checks or global cache clears. Relative paths retain the
existing resolver policy; cache keys normalize absolute paths lexically.

`Assembler::set_materials` accepts the snapshot explicitly and establishes a
thread-local RAII binding for its synchronous `add_multimaterial` calls, including
composite models. This avoids changing every material-model virtual interface.
Worker threads must each establish their own scope with the shared snapshot;
thread-local bindings are not inherited. A mutex serializes cache lookup and
first load, and exceptions restore outer scopes without caching failed loads.
A standalone load outside a scope is a fresh read; callers that initialize
multiple elements directly should establish a scope explicitly. Separate
`set_materials` calls without an explicit snapshot are separate input snapshots.

Matrices and fiber vectors are shared as const data. `set_mat` replaces the
calling expression's matrix, leaving cached data and copies unchanged. Reloading
an existing fiber object under a new snapshot rereads its file; conflicting
file/field declarations for one fiber object retain their existing error.
No constitutive equations, fiber normalization, solver settings, contact policy,
or dependency pins are changed.

## Reproduction and validation

Before changing production code, the independent-consumer regression failed two
of four assertions: after scalar value 1 became 9, the new expression still read
1; after fiber (1,1,0) became (9,1,0), the new fiber still read the original unit
direction. The old consumers correctly retained their original values.

After rebuilding `PolyFEM_bin` and `unit_tests`, the focused suite passed:
**31 cases, 659 assertions**. This includes the three new PF-04 cases, the
existing utility tests, BC metric and AL termination cases, and semi-implicit
contact/friction derivative cases. New coverage includes:

- Scalar 1 -> 9 and fiber (1,1,0) -> (9,1,0), with old readers still alive.
- Actual VarForm and legacy State initialization, repeated material/mesh loading
  within a snapshot, and fresh input initialization on an existing State.
- Sixteen concurrent readers sharing two snapshots; unchanged scalar timestamp;
  equal matrix storage addresses and shared fiber storage within a snapshot.
- Copy-on-write, nested scope restoration after an exception, distinct fiber
  fields, failed-load retry, cached reads after file deletion, and payload
  survival after destruction of the cache owner.
- Reusing a material assembler with the same and then a fresh snapshot.

The first immutable-storage build exposed a numeric-array construction path that
still wrote through the published matrix pointer. Construction now fills a local
mutable matrix before publishing it as const. The first simulation fixture also
called the output-space debug helper before FE-space preparation and crashed;
a test-only accessor now inspects the initialized material assembler directly.
The passing rerun includes that simulation regression.

All five solver smokes exited zero with no error log lines. The scripts direct
results to `outputs/pf-04/smokes/` without overwriting prior smoke evidence.
The real-binary Houdini `test_polyfem_materials.py` suite passed, including the
exported per-element material solve, output field checks, and round-trip import.
Formatting and `git diff --check` passed.

The broader `[assembler],[fsi],thermo` selection passed **40 cases and
4,768,563 assertions**, including all three FSI tests and the six thermoelastic
reference scenes. These use the existing test harness's configured short-run
settings; they are not claims of full-duration scene validation. No golden
references or tolerances were changed. No full unit-suite, Ballburst, or Teseo
run is claimed for this session.

## Provenance and limits

Started on main at `1796cc917`, with no tracked incoming edits. Existing untracked
simulation outputs are preserved. Effective IPC is
`9da3094a46bcc054cc19024a5c748557c5bb6b9e`; the clean PolySolve override is
`012658e5d95b086f20c8e0e5f247473a2a229825`. Local evidence is in the parent
workspace's `outputs/pf-04/` directory.

This session addresses cache lifetime only. It does not validate physical
accuracy or complete PF-02's unresolved contact-floor work. No Teseo run is
permitted without a new explicit user request.

Reproduce the focused check from the workspace root:

```sh
cmake --build polyfem/build --target PolyFEM_bin unit_tests -j 6
polyfem/build/tests/unit_tests '[material_cache],[utils],[bc_metric],[al_solver],semi-implicit barrier contact form derivatives,semi-implicit friction form derivatives'
```
