# RB-11 geometry, material and input validation: probe matrix

Fixture generator and runner behind [docs/rb-11-validation.md](../../docs/rb-11-validation.md).
Everything is synthetic and public: a 2×2×2 tetrahedralised unit cube (48 tets,
Gmsh 4.1 ASCII or MEDIT), an open two-triangle slab obstacle, and scene JSON
built by `fixtures.py`. `cases.py` perturbs them into the invalid variants the
plan requires (invalid indices, degenerate/inverted/duplicate rest elements,
duplicate/nonmanifold/degenerate collision topology, initial intersections,
unit mismatches, nonfinite and out-of-range material parameters, misbound
per-element scalar and fibre files, missing materials, conflicting prescribed
motion, aliasing solver settings) and keeps a valid control next to each.

```sh
python3 tools/rb11/run_matrix.py --binary build/PolyFEM_bin --output /absolute/fresh/dir \
    [--cases g1-missing-mesh-file ...] [--groups geometry material ...] [--timeout 120] [--verify]
python3 tools/rb11/run_matrix.py --binary x --output y --list
python3 tools/rb11/run_matrix.py --binary x --output y --self-test
```

`--verify` makes the runner exit 1 on any mismatch (or when no case is
selected); without it the run is an investigation that reports every
mismatch and exits 0. `--self-test` feeds hollow records to the oracle
(an accepted run without saved steps, a notice case without its notice, a
named failure with an unrelated error) and fails if any is accepted.

Every case runs single-threaded from its own directory and is classified as
`named_failure` (exit 1, `PolyFEM stopped:`), `resource_failure` (exit 3),
`crash` (signal / exit ≥ 128), `completed`, `timeout` or `other_exit`.
`summary.json` / `summary.md` record per case the first error line, the
`PolyFEM stopped` line, warnings, the last log phase reached, whether any VTU
was written (an invalid input must not leave accepted output), the wall time
and, where `check.json` asks for it, whether the exported material field at
the top/bottom vertices matches the source file (`E`, `fiber_direction`) or
whether the log carries required/forbidden lines. `expect_match` compares the
run with the case's declared contract: a `named_failure` must exit 1 with a
`PolyFEM stopped:` line, write no VTU and carry the case's `expect_error`
phrase in its log; an `accepted` run must complete with the intended number
of saved steps and pass its `check.json` (per-element transfer by centroid
matching, top/bottom field values, obstacle edge incidence, required log
lines); an `accepted_with_notice` run also needs its notice. A case may
declare `expected_status: fail` for its check — `g3-E-file-permuted-oracle`
does, to document that the transfer oracle detects a permuted file.

Expectations encode the plan's acceptance: genuinely invalid inputs fail early
with element/material/path context and no accepted output; valid inputs
(controls, open and codimensional obstacles, a T-junction shelf, a pinched FE
mesh, a massless quasistatic body, auxetic ν, per-body value files) stay
accepted; envelope-limited inputs (declared millimetre units, zero density in a
transient run, row-sum lumping of quadratic elements) complete with the
documented notice.

Per-element value contract exercised by `g3-E-file-two-bodies-*` and
`g4-fiber-file-two-bodies-global`: a list/file with one row per element of
the whole FE mesh binds by the global element id (the Houdini export
contract); one with one row per element of the body it is given for binds by
the body-local index (upstream `#333`); any other length is refused at load.
The Gmsh writer emits one element block per body, so the global order is body
1's elements followed by body 2's.

Review additions (2026-09-13): 2D `HGODispersion` cases at `kappa` 0.4 / 0.5
(valid, the law's domain is `[0, 1/d]`) and 0.6 (refused), plain and
composite; fibre representations (constant zero, expression zero, non-unit
under the normalising `HGODispersion`, unit-length expression, wrong
dimension); per-element transfer with unique values on one body, unequal
bodies (8 + 40 tets), two geometry meshes and body-local files, plus the
permuted-file oracle control; a real three-face edge for the T-junction
obstacle, asserted from the written OBJ.

The Catch regression `[input_validation]` (`tests/test_input_validation.cpp`)
covers the same checks at the API level.

## Envelope stage (`envelope.py`)

```sh
python3 tools/rb11/envelope.py --list [--stage locking thin distorted homogeneous anisotropic refine]
python3 tools/rb11/envelope.py --binary <bin> --output /abs/candidate-dir [--reuse /abs/earlier-candidate ...] \
    [--reuse-binary-sha <sha>] [--stage ...] [--only name ...] [--jobs 6] [--timeout 1800] [--verify]
python3 tools/rb11/envelope.py --output /abs/candidate-dir --analyze-only   # no solver: parse + verify what exists
python3 tools/rb11/envelope.py --self-test
```

The physical-envelope characterisation behind the record's envelope section
and [docs/rb-11-envelope-contract.md](../../docs/rb-11-envelope-contract.md):
Kuhn-tetrahedralised cantilevers (volumetric locking vs ν, thin sections,
jittered/stretched cells, fibre-reinforced bending), the compressed cube
(homogeneous near-incompressibility against the exact RB-09 state) and the
affine cube (MaterialSum(NeoHookean, HGODispersion) against the law's stress
by central differences), each with `LinearElasticity` twins whose reduced
stiffness gives κ(K); the `refine` stage adds one P3 refinement of four
references (contract amendment 1).

Execution, parsing, analysis and verification are separate: `run.json`
(version 2: command, binary sha, input hashes, exit/signal/timeout, wall)
is written before any parsing; parsing never raises (nonlinear step lists,
the linear `solver_info` dict, missing/malformed output are recorded per
case); verification needs an explicit manifest and reports every check as
pass / fail / not evaluated with value, threshold and cases
(`verify.json`, `summary.md`). A record from a `--reuse` directory is
reused only with exit 0, byte-identical regenerated inputs and a stated
binary identity (`--reuse-binary-sha` for legacy records); every report row
names its candidate. `--self-test` feeds the parser and the oracle their
known failure modes, including synthetic case directories with a real
endpoint VTU driven through parse → analyse → verify (missing / null /
invalid / zero Newton counts, malformed step records, absent `F` arrays,
an unconverged run): a nonlinear case without a recorded count or without
determinant evidence fails C-R and leaves C-N not evaluated.
`--report-dir` writes the report elsewhere than `--output`, keeping an
earlier report intact.

RBR-05 addition (2026-09-20): `g5-fully-prescribed-body` — a unit cube of six
tets whose eight vertices are all on the boundary, with the whole boundary
prescribed (zero free DOFs), transient. Expected `accepted`: the empty reduced
problem is a valid trivial solve whose solution is the prescribed values
(`check.json` requires the "No free degrees of freedom" info line). On the
binaries before the repair the case is a `crash` (SIGSEGV after `step_0`).
