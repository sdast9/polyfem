# Review of updated RBR-01 and CI-01–CI-03 — 2026-09-21

**The RBR-01 follow-up, CI-01, CI-02, and CI-03's fixture policy are correct
within the reviewed scope. Two P2 defects remain in CI-03's new evidence
tools.** Neither changes solver results or invalidates the independently
authenticated fixtures. They do prevent the tools from reliably retaining
the evidence needed for later reference changes.

This is a review and an implementation handoff. No production code, tests,
fixtures, dependency pins, defaults, or tolerances were changed. The plans
below remain unimplemented. The review is published on a separate branch,
`codex/rbr-ci-review-20260921`, to preserve the running main-branch CI job.

## Source and evidence

Reviewed PolyFEM `baac15c5f3812aef983d577ef721b89b1b617ca0`, with a clean
working tree and these clean companion/data checkouts:

| Repository | Revision |
| --- | --- |
| IPC Toolkit | `482b9eab2f81bbc5ee59586f80ffb041bcd488dd` |
| PolySolve | `bce32a39a2c8f0a64cb8ffa85b89f0ee773df0ec` |
| `sdast9/polyfem-data` | `e6ed5cf2d6514ef2595d28a23400521e2b3c717c` |

The review covers the changes since the previous RBR review at `ad109b58b`,
plus the CI-01/CI-02 implementation at `a327e2932`. The relevant RBR fix is
`924a8597f`, its clarification `11cdf476f`, and the GCC brace correction
`0c129dcfb`. The CI-03 implementation is `bf6ea5c58` and the data commit
above. Instructions, PF invariants, plans, validation records, source,
tests, saved A/B evidence, and current native job logs were inspected.

Local evidence is in the parent workspace:
`outputs/rbr-ci-review/20260921T190659Z/`. It contains source/build identity,
build and test logs, independent RBR probe source and compiler commands,
saved VTUs, synthetic evidence-tool probes, and downloaded native CI logs.
These evidence files are not part of the documentation commit. No Teseo or
private scene ran.

## Findings and coding handoffs

### F1 / P2 — Fixture generation overwrites the 2D run with the 3D run

**Location:** [make_fixtures.py](../tools/ci03/make_fixtures.py),
`generate_tests`, lines 87–90 at the reviewed revision.

The directory key is `Path(twin_rel).stem`. Both mass-ratio fixtures map to
`generate/large-mass-ratio-friction-defaults`. The normal three-fixture loop
runs 2D first, then recursively deletes that directory before running 3D.
This removes the 2D log, isolated input, and per-run ledger even in a fresh
top-level output directory.

This happened in the actual implementation evidence:
`outputs/ci-03/20260921T142215Z/fixtures/make_fixtures.json` records three
generated fixtures, but `fixtures/generate/` retains only two `run.json`
files. The mass-ratio survivor identifies the **3D** parent. A fresh probe
of the unchanged `generate_tests` with a synthetic harness reproduced the
same three-to-two loss (`evidence-collision-result.json`). The synthetic
harness only wrote identifying markers; it did not calculate physics or
produce replacement references.

The aggregate report retains the 2D reference values and hash, and both
native Release lanes independently authenticate that twin. Therefore this
finding is loss of generation provenance, not evidence that the committed
2D reference is numerically incorrect.

**Implementation plan, one bounded tools repair:**

1. Pass the existing unique `FIXTURES` slug from `main` to `generate_tests`
   and use `output/generate/<slug>` as the run directory. Precompute and
   validate all directory identities before changing any parent fixture.
   Use the complete relative fixture path in the ledger as well as the slug.
2. Remove implicit deletion of an existing run directory. The CLI promises
   a fresh evidence directory: reject a collision with a clear error before
   mutating data, or create an explicitly separate run attempt. Do not add
   automatic reuse or overwrite behavior. Apply the same preservation rule
   to `ab_runner.run_one` when touching shared run-directory helpers.
3. Give every fixture its own durable record and aggregate entry, including
   parent/twin paths, run directory, exact command, binary hash, copied
   inputs, input twin hash before generation, exit status, and generated
   twin hash. Keep parent identity distinct from the generated twin's
   identity. Retain partial evidence on failure; use F2's execution record
   contract if that repair lands first.
4. Add a small Python test using temporary directories and a synthetic
   harness. Generate the **actual two equal-basename relative paths** plus
   the ball path. Assert three distinct surviving ledgers/logs/input trees,
   correct fixture identities, unchanged first-run bytes after the second
   run, and aggregate links to all three. A second invocation against the
   same destination must preserve the old bytes and fail before data writes.
5. Update the tools README and CI-03 record. Preserve the original incomplete
   evidence directory. No production fixture regeneration, golden change,
   or data-pin update is needed to fix this directory bug. If replacement
   provenance is desired later, generate only the affected evidence in a
   new directory and compare it to the already committed references.

Acceptance: all three evidence bundles survive; collisions cannot delete
earlier evidence; synthetic tests pass; existing successful input/reference
semantics are unchanged. Commit and push the tools/test/doc repair with its
new evidence location.

### F2 / P2 — Parsing partial output discards the execution result

**Locations:** [ab_runner.py](../tools/ci03/ab_runner.py), `run_one`, lines
245–281, and the future-result loop at 328–333. The analogous order occurs
in [make_fixtures.py](../tools/ci03/make_fixtures.py), lines 97–100.

`run_one` catches a subprocess timeout and stores `exit = "timeout"` only
in memory. It parses computed metrics, diagnostics, and the manifest before
writing `run.json`. A truncated JSONL tail raises from
`diagnostics_summary`; `future.result()` then aborts the matrix before its
summary is written. The raw files remain, but there is no durable run ledger
with the process outcome, command, elapsed time, and provenance. Generation
has the same ordering problem if the subprocess leaves malformed twin JSON.

A subprocess probe wrote a partial `physical-diagnostics.jsonl` and then
exceeded the supported `--timeout`. The actual runner raised
`JSONDecodeError`, exited 1, and produced neither `run.json` nor the matrix
summary. The raw log and partial diagnostics remained.
See `partial-output-result.json` and `partial-output-probe.log`. This is a
controlled failure-handling probe, not a solver timeout claim. All 39 saved
A/B run ledgers from the completed CI-03 investigation were present and
matched the published compact results; this defect did not occur there.

**Implementation plan, separate bounded tools repair:**

1. Persist an initial ledger before launching a child. Include fixture,
   variant, binary/hash, copied inputs/hashes, command, working directory,
   and start time. Immediately after the child exits or times out, persist
   its return code or timeout outcome and elapsed time **before parsing any
   output**. Record launch failures too. Use a temporary JSON file followed
   by an atomic replacement within the same run directory.
2. Separate execution status from analysis status. Parse computed metrics,
   `sim.json`, JSONL diagnostics, and the manifest independently. Attach a
   structured error with source path and, for JSONL, line number when a
   parser fails. Preserve any other successfully parsed component. Do not
   silently treat a partial diagnostic record as a complete successful run.
3. Keep incomplete and non-finite metrics out of numeric comparisons; record
   why a comparison is unavailable. A parser error must not overwrite the
   original exit status or erase input/binary provenance. A nonzero harness
   result can be an intentional negative A/B result, so preserve the
   distinction between an authentication mismatch, a solver/launch failure,
   and an analysis failure.
4. Convert each worker exception into a recorded per-run failure and continue
   collecting other workers. Always write a matrix summary containing every
   requested run, including failed/incomplete entries. Return a nonzero CLI
   result for launch, timeout, or analysis failure after preserving the
   summary; document how expected authentication mismatches affect the CLI
   result. Update the summary formatter to handle missing computed/lag data.
5. Apply the same record-before-parse ordering to `generate_tests`. A malformed
   generated JSON file or nonzero generator exit must retain its ledger and
   must never publish a candidate twin to the data checkout. Preserve an
   incremental aggregate ledger even if a later fixture fails.
6. Add lightweight Python tests with synthetic child processes for: a
   successful run; an ordinary nonzero harness exit; timeout with a partial
   final JSONL line; malformed manifest; incomplete/non-finite computed
   metrics; malformed generated twin; and a two-run matrix where one fails
   but the other completes. Assert raw evidence preservation, durable actual
   exit status, explicit analysis errors, complete matrix membership, and
   no invalid fixture publication. Check a successful result still produces
   the same metrics and overlays as before.

Acceptance: the reproduced timeout retains its `timeout` ledger and matrix
entry; no parser failure loses execution evidence or another run's result;
successful numerical behavior is unchanged. No solver change, reference
regeneration, or tolerance relaxation belongs to this repair.

**Reproduction assets for F1/F2:** `probe_ci03_tools.py` in this review's
evidence directory executes the unchanged tools against isolated copies of
the public inputs and two synthetic harness scripts. It writes both result
JSON files above. Copy it to a new evidence directory when reproducing, so
the original review evidence remains intact. For committed coverage, move
the scenarios into small temporary-directory Python tests rather than
depending on this machine's absolute paths or a full solver build.

## Correctness assessment by item

| Item | Verdict and reason |
| --- | --- |
| RBR-01 follow-up | Correct. The specialized differentiable solve now states `CurrentStepBeforeAdvance` before lag initialization and every subsolve export. Initialization and history advancement retain `HistoryHead`. The false-mode delegation remains intact. This repairs the missing owner transition without changing integration, displacements, or derivatives. |
| CI-01 | Correct. Nested aggregate braces preserve values and the warning policy. Renaming the test-local `near` identifier avoids the Windows macro without changing SDK definitions or test behavior. The later RBR test arrays use the same portable spelling. Native GCC and MSVC builds succeed. |
| CI-02 | Correct. Exact vector cardinality remains checked; only four analytical-value assertions gain the narrow relative bound `4 * double epsilon`. That covers the measured two-ULP discrepancy near 100 without weakening continuity/equality contracts or scene tolerances. The other edits are formatting. |
| CI-03 solver/fixtures | Correct within its stated regression scope. The original three references, durations, margins, and other input fields are unchanged. Each parent explicitly requests the historical friction budget 1; each twin inherits its parent and removes the pin before schema defaulting. Native execution confirms that both policies authenticate. The two tool findings above remain open. |

The new RBR regression uses the public differentiable entry point with both
mode values, initial velocity, move/hold/hold/return motion, callback output,
saved VTUs, final post-advance output, and the last subsolve. Its oracle uses
saved displacements and independent Implicit Euler arithmetic. The
differentiable path's previously documented lack of rollback remains a
separate limitation; this change does not add a retry or a rollback policy.

For CI-03, the 39 original A/B records were checked against the published
results. All six metrics satisfy both recorded exact equalities for each
fixture: before-default = before-explicit-1 = after-explicit-1, and
before-explicit-2 = after-explicit-2 = after-default. Twelve runs authenticated;
27 intentionally failed the historical references with exit 42. Those are
negative comparison evidence, not successful authentication runs.

The current twins are **regression references for the approved policy**.
The documented 3D budget-2 overshoot relative to converged lagging remains:
roughly 33% in the reported `err_h1_semi` metric. The fixtures are not physical
accuracy certification. This review does not reopen the approved default.

One reproduction detail matters for later models: the A/B tool defines
`default` as an unchanged fixture copy. The saved causal experiment used
data `e0efb6b`, before these pins existed. Against today's `e6ed5cf` data,
that variant inherits the explicit historical budget 1. Repeating the old
experiment requires the old data snapshot; testing the **current solver
default** requires the new twin. This is the tool's documented copy
semantics, not a third solver defect. Make this distinction explicit when
updating its reproduction instructions.

## Fresh validation and remaining CI failures

The existing AppleClang 21 / arm64 / TBB / RelWithDebInfo build was rebuilt
at the reviewed clean revision. `build_info.json` confirms both companion
pins; Triangle remains off in this local build.

- `[output_kinematics],[kappa_continuity],[semi_implicit_coefficients],[rb22],[data],[rollback]`,
  seed 1: **38 cases, 6,978 assertions pass**, exit 0, 273 seconds.
- Recompiled the previous independent RBR review probe against the current
  library: **572 assertions pass**, exit 0. Callback velocity/acceleration
  errors are exactly zero on both modes at every endpoint.
- Independently decoded all ten saved VTUs from that probe. Maximum errors
  against saved-displacement Implicit Euler arithmetic are `8.68e-18` for
  velocity and `3.48e-17` for acceleration. All 31 Float64 point fields agree
  exactly across the two modes at each of the five saved endpoints.
- Static data comparison confirms that each historical file differs from
  the old data revision only by the explicit budget-1 block; stored references,
  margins, and durations are identical. The twins have the intended removal
  patch and unchanged duration/margin. Dataset classification passes above.

Native evidence was read from the **current source**,
[Build run 35627410442](https://github.com/sdast9/polyfem/actions/runs/35627410442),
at `baac15c5f`, rather than treating a cancelled workflow as a green matrix:

| Lane | Verified result at review time |
| --- | --- |
| [Linux Release](https://github.com/sdast9/polyfem/actions/runs/35627410442/job/106425422451) | 378/381; only CI-04 `standard`, CI-05 `triangle_data`, CI-06 `contact_2d` fail. |
| [macOS Release](https://github.com/sdast9/polyfem/actions/runs/35627410442/job/106425422634) | 377/380; the same three failures. |
| [Windows Release](https://github.com/sdast9/polyfem/actions/runs/35627410442/job/106425422637) | 365/365. The hidden scene groups are not included on Windows. |
| [Linux DebugNoSymbols](https://github.com/sdast9/polyfem/actions/runs/35627410442/job/106425422381) | 345/349; four rollback scene cases die with SIGSEGV. |
| macOS / Windows DebugNoSymbols | Still running when this review snapshot was taken; no completion claimed. |
| [Pre-commit](https://github.com/sdast9/polyfem/actions/runs/35627410169) | Pass. |

All six CI-03 fixtures explicitly authenticate on **both** Release Unix
lanes; the maximum relative discrepancy from their stored values is below
`7.8e-10`, well inside the unchanged `1e-5` margin. `contact_3d` is green on
both. The RBR output regressions and focused portability checks pass in the
completed lanes. No local P4 scene run is claimed with Triangle off.

The Linux Debug failures are not introduced by this follow-up: the same four
named cases fail in the pre-update
[baseline Linux Debug job](https://github.com/sdast9/polyfem/actions/runs/35584793121/job/106285598924)
at `303b54cc0`. They concern failed-step restoration, stall retune plus
failure, AL-budget termination, and publication failure. Native logs name
`test_step_rollback.cpp:407` as the last Catch assertion; that location is
not a stack trace and does not establish the crash cause. Local rollback
success does not resolve them. Keep their Linux Debug investigation
separate: reproduce one case in the native configuration, obtain a stack
trace, and only then select a bounded repair. No cause or fix is invented
by this review.

## Applicability upstream and by stiffness context

- **RBR-01:** the explicit output phase belongs to the common transient
  exporter/owner contract and is appropriate wherever the matching upstream
  path exists, for fixed, adaptive, and variable stiffness alike. Preserve
  coverage of every owner, including differentiable dispatch and rollback.
- **CI-01/CI-02:** portable initialization, avoiding SDK macro names, and
  narrowly justified analytical floating-point assertions are suitable
  upstream in corresponding code. Coefficient-specific tests only apply
  where that coefficient implementation exists. There is no reason to weaken
  production tolerances or warning policy.
- **CI-03:** explicit historical friction budgets plus separate tests of an
  approved current default are a generally useful fixture policy. The three
  measured scenes use **adaptive** barrier stiffness, so their passing
  results do not separately validate fixed or semi-implicit stiffness.
  The friction-budget issue is broader than variable stiffness. Upstream
  reference values must match upstream's own approved policy and evidence;
  the fork's data URL and budget-2 goldens are not automatic upstream changes.
- **F1/F2:** these are evidence-tool repairs, with no reason to change either
  barrier stiffness implementation. Carry the evidence-preservation contract
  into any upstream contribution that also ships these tools.

The existing [upstream plan](rb-upstream-plan-20260920.md),
[RBR handoff](rb-review-repair-plan-20260920.md),
[RB-04 record](rb-04-validation.md), and
[CI-03 record](ci-03-validation.md) retain their model and acceptance limits.
