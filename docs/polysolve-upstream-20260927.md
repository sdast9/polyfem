# PolySolve upstream adoption — 2026-09-27

## Change

Advance PolySolve from `448f1b8e0da72101d17dcbf1f49da5f647fe0315` to
`6099b9cddbab7d856c57de95e2642b87a1348b7b`, published on
`sdast9/polysolve:iteration-callback`. It merges upstream `da4e7fe` and adds
CPUHybrid/GPUHybrid/cuDSS linear solvers, nano-MPI integration and large-index
build/CI support. The fork's nonlinear source tree is unchanged, preserving
callbacks, objective-generation resets, BFGS safeguards, Wolfe and the
Newton-only slope/step-length stopping contract.

The shared native build adopts upstream's non-Windows nano-MPI default (ON),
using rank threads without an external MPI launcher. Its CUDA and large-index
options remain OFF, and the selected production solvers/controller settings
are unchanged. IPC stays at `75600955`. No HDA source/menu change is included.

The companion merge declares the BFGS tests' LBFGSpp header dependency, removes
upstream trailing whitespace, corrects a CI comment and pins Hypre's moving
thread-mpi-backend to `ba9c640318c5f96aeb9d2afc95ddc9243b2cd40e`.
nano-MPI v0.1.1 resolves to `456871cfa71744702889f476d372c0b53fd6dca2`.

## Verification

Native macOS arm64 / AppleClang 21:

| Check | Result |
| --- | --- |
| Standalone PolySolve Release default build and enabled suite | 61 cases / 2,962 assertions pass |
| Separate large-index Release build and enabled suite | 61 cases / 2,848 assertions pass |
| CPUHybrid, 32-cubed grid, 1/2/4 ranks | Relative residuals 3.668e-11 / 2.281e-11 / 4.927e-11; all below 1e-8 |
| Shared PolyFEM RelWithDebInfo build | PolyFEM_bin and unit_tests rebuilt successfully |
| Affected PolyFEM solver/form tests | 87 cases / 9,224 assertions pass |
| RB-02 coefficient probe | 270 checks pass |
| Five public smokes versus saved ad7f41622 binary | All runs exit 0; 25 VTUs byte-identical |
| Houdini end-to-end export/run/import | Pass, including Hypre nodal coarsening and nonlinear method round-trips |

Initial standalone test-file failures came from two build directories sharing
one ExternalProject fixture checkout while it was being recreated. Separating
the fixture roots and running the suites sequentially produced the passes
above. Initial missing-header and fixture-loading failures remain in the logs.

The rank checks establish residual accuracy, not performance scaling. CUDA
execution is untested on this Mac. Native Linux/Windows CI is a separate
result. In [PolySolve CI run 36295825495](https://github.com/sdast9/polysolve/actions/runs/36295825495),
all seven Release jobs passed (Linux/macOS/Windows with both index widths,
plus Hybrid Release). All seven Debug jobs failed in `nonlinear-easier`: six
retrieved job logs show the `Armijo.cpp:24` assertion `armijo_criteria <= 0`;
the Hybrid Debug log shows the same test aborting without the assertion text. Linux Debug with large indices OFF passed the other 64/65 tests.
The nonlinear source tree and that test are identical to the baseline; this
does not prove the failure is pre-existing. Baseline CI run 35871632819 stopped
at the missing LBFGSpp header repaired here, so it supplies no comparable
Debug runtime result. The assertion remains an open CI issue; this integration
does not alter nonlinear behavior or weaken the assertion to obtain a pass.

**Resolved later on 2026-09-27.** The assertion was the Armijo line search receiving
an uphill direction (ADAM directions are unscreened). PolySolve `6a8c2cc9` makes
Armijo/RobustArmijo/Wolfe's fallback fail the search on an uphill direction instead
of asserting, and PolySolve CI then passed 14/14 jobs, the seven Debug jobs
included (recorded at `43ca2e66`, the pin PolyFEM adopted in `61a7a4507`; the workspace
README's 2026-09-27 band-statistic entry carries the result).
The "open CI issue" above is closed.
The complete PolyFEM unit suite and all 13 HDA scripts were not rerun;
the affected selection and the HDA end-to-end script are the consumer checks.
No Teseo or private scene was run. General physical accuracy is not established
by this dependency integration.

## Evidence

Parent workspace: `outputs/polysolve-upstream/20260926/` (session began September
26). `PROGRESS.md` tracks completion; `suite-results.json`, `hybrid-results.json`
and raw logs retain standalone evidence. `baseline-build-info.json` accompanies
`PolyFEM_bin-baseline-ad7f41622`. The consumer build log is `build-polyfem.log`.
Consumer evidence: `tests-polyfem.log`, `rb02/probe-results.json`,
`smoke-identity/identity.json`, `hda-e2e.log` and
`polyfem-build-info-tested.json`. `polyfem-build-info-final.json` records the
subsequent metadata-only rebuild at the clean published source revision.
CI evidence: `polysolve-ci-later-check.json` (completed run),
`ci-linux-debug-off-api.log`, `ci-job-*.log` and `ci-baseline-linux-debug.log`.
The companion's detailed record is `docs/upstream-integration-20260926.md`
in the PolySolve repository.
