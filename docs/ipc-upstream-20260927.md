# IPC upstream integration — 2026-09-27

## Change

Advance IPC Toolkit from `7560095572cd62792dc773c2690437d57412761c` to
`cf99893be74fe296e6b771b8e22ba4562942e77e`, published on
`sdast9/ipc-toolkit:semi-implicit-stiffness`. It merges upstream `869e489e`:
MeshFEMSparse/full-DOF assembly, templated/SIMD geometry and shared CPU/CUDA
LBVH code. The fork retains per-contact stiffness, parent contributions,
exact displacement mapping and checked resource budgets. The new CUDA LBVH
override explicitly refuses unsupported budgets before allocating device
storage; its conditional test is included but unexecuted on this Mac.

PolyFEM's custom clamped-log barrier now names `ipc::ClampedLogBarrier<>` to
match upstream's templated API. Its double-precision formula is unchanged.
The shared CPU build uses upstream's MeshFEMSparse ON default, SIMD ON and
CUDA OFF. PolySolve stays at `6099b9cd`. No solver tolerance, trim rule,
friction policy, broad-phase selection, resource limit or HDA menu changes.

## Validation and numerical difference

Native macOS arm64, AppleClang 21:

| Check | Result |
| --- | --- |
| IPC optimized assertion-enabled CPU suite | 339 cases pass, 5 existing assertion-build skips; 4,975,065 assertions pass |
| PolyFEM RelWithDebInfo build | Builds after the barrier template adaptation |
| RB-02 coefficient probe | 270 checks pass |
| RB-05 budget probe | All 11 outcomes, exception types and candidate counts match the accepted RBR-02 record |
| RB-05 public scene limits | All expected exits and default/generous versus unlimited identities pass |
| RB-12 repeat matrix | All 30 runs complete, zero same-history violations; one threaded cell has two histories; single-threaded repeats identical |
| Affected PolyFEM unit selection | 87 cases / 8,247 assertions pass on the restored-default build (`tests-polyfem-affected.log`) |
| Houdini end-to-end | Export/solve/import passes, including Hypre nodal coarsening and nonlinear method round-trips |

Five public smokes were compared single-threaded against the saved clean
PolyFEM `3c40ae557` binary, before the IPC change. Every run completes, but the
VTUs are **not byte-identical**. Four scenes retain the same solver history:
maximum displacement difference `4.44e-16`, maximum normalized endpoint
scalar difference `5.53e-13`, within the existing `1e-12` roundoff threshold.

**The friction scene changes materially at step 4.** Its first three steps
agree within `5.60e-15`; step 4 takes absolute-gradient rather than relative-
gradient termination, leaves trim at 8 rather than 16, and changes the final
displacement by at most `1.7047621e-4` in internal length units. Normalized
barrier-energy difference is 0.336; frictional-dissipation difference is
0.00350. The old binary reproduces trim 16 and identical final displacement
on two additional runs. Both binaries already report `physical_balance_pass`
false on this friction fixture; this integration does not certify it.

Disabling MeshFEMSparse and using the triplet backend reproduces the same
step-4 discrepancy, so the change is not specific to the new backend. The
weighted derivative suite passes. The tiny differences before the stopping
decision are consistent with changed arithmetic affecting the existing
controller branch; no missing stiffness factor was found. The upstream
backend default was restored after this experiment. No tolerance or trim
logic was changed to force agreement. RB-12 classifies different solver
histories as **branch divergences**, separately from same-history roundoff
violations; the strict cross-binary roundoff comparison still fails for
friction and is retained as such. Exact friction-output preservation is an
unresolved integration limit.

**Correction (2026-09-27, later).** The explanation above is incomplete. The
step-4 decision is taken at Newton iteration 1 by the trim band, whose
count-based rms gap straddled the band edge (0.70537 vs 0.70796 `dhat`, edge
0.7071) because five cube vertices sit exactly over the slab's shared diagonal
and upstream's rewritten `point_triangle_distance_type` resolves that tie into
a different number of merged collisions (energy unchanged, count changed).
Old/new binaries took trim 16 / 8 in 8 of 8 threaded repeats each. The band
statistic is now weighted by collision weight and no longer depends on the tie
resolution (0.7142 `dhat` at that decision); see
[band-statistic-weighting-20260927.md](band-statistic-weighting-20260927.md).
The full-suite cap of 1,800 s was also too short: complete suites on this
machine take about 60-65 minutes. A complete suite on `61a7a4507` found one
regression of this integration: `shape-transient-friction` (`[opt_gradient]`)
fails its finite-difference tolerance (4.3e-5 vs 1.1e-5) because the finite
difference moved while the adjoint derivative did not; bisected to
`6570e0410` (see the band-statistic record).

The full PolyFEM unit suite reached its 1,800-second cap while processing
`contact/examples/2D/unit-tests/5-squares.json`. No assertion failure was
reported before the cap, but this is **not a complete suite pass**. The
affected selection is recorded separately. CUDA execution is untested;
native IPC CI run [36339194721](https://github.com/sdast9/ipc-toolkit/actions/runs/36339194721)
on `cf99893b` completed with all six jobs passing (Linux, macOS and Windows,
Debug and Release; `ci-completed-check.json`). No Teseo or private scene ran.

The shared `build/PolyFEM_bin` (built 2026-09-27 14:10) was built from this
change before it was committed: a user run started from it records
working-tree patch SHA-256
`07352050d7b0ba5569154efef3c6b1ec856748156bb6a47ddfcbc06dcc8e7f4b`, which
matches that uncommitted diff (the pin, the barrier adaptation and the earlier
PolySolve record wording). Only documentation changed after that build.

## Evidence

Parent workspace `outputs/ipc-upstream/20260927/`: `PROGRESS.md`,
`tests-all-with-data.log` (IPC), `consumer-validation-results.json`,
`rb02/probe-results.json`, `rb05-budget-audit.json`, `rb05-scenes/results.json`,
`repeat/summary.json`, `unit-suite.log`, `tests-polyfem-affected.log`,
`hda-e2e.log`, `smoke-identity/identity.json`,
`smoke-numerical-comparison.json`, `baseline-repeat/`, `triplet-experiment/`,
`ci-completed-check.json`.
The initial fixture-less IPC run and initial PolyFEM API compile errors are
retained; neither is counted as a passing check. Saved baseline and candidate
binaries, build logs and build-info JSONs establish source provenance.
The companion record is `ipc-toolkit-fork/docs/upstream-integration-20260927.md`.
