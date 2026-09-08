# RB-01 — Contact-cache ownership and invalidation

Date: 2026-09-08
Status: **validated within stated scope**
Selected stage: all four RB-01 stages; ownership/invalidation repair only.

## Contract and authorization

The user selected RB-01 from [the robustness plan](robustness-plan.md).
A form must not skip its required collision-set rebuild because another form
used equal coordinates, or because candidates/configuration changed at fixed x.
Read the PF invariants and [floor retirement](pf-02-floor-removal.md), together
with workspace/HDA instructions. No prerequisite blocks this item.

This repair changes no coefficient law, stiffness default, CCD behavior, trial
cap, friction policy, solver stopping tolerance, input model or recovery policy.
The constraint floor remains retired. RB-02 coefficient/lifecycle questions,
RB-03 nonidentity mapping, and physical certification remain separate work.

## Baseline and reproduction

Started on PolyFEM `main` at `bab5501a6`, with no tracked edits and existing
untracked simulation/VTK artifacts. Those artifacts were preserved. No running
build or simulation was found at inspection. There was no earlier RB-01 record.

Effective dependencies were verified, including source overrides:

- IPC recipe and effective CPM checkout: `9da3094a46bcc054cc19024a5c748557c5bb6b9e`,
  `~/.cache/CPM/ipc-toolkit/0c20`, clean detached checkout.
- PolySolve recipe and effective local `polysolve-merged` override:
  `4d372fa8a73f42bc224e31d464f1a308e1159ba8`, clean `iteration-callback` branch.
- No dependency source, recipe, or configuration was changed.

The existing macOS arm64 build uses Apple Clang, `RelWithDebInfo`, TBB,
Accelerate enabled, and the public smoke inputs' `Eigen::SimplicialLDLT` solver.
Builds used six jobs. Full compiler version, CMake cache, source states, input
and binary SHA-256 hashes are recorded in the local evidence manifest.

Evidence directory (relative to the parent workspace):
`outputs/rb-01/20260908-contact-cache/`. It contains `commands.md`,
`baseline-manifest.json`, `CMakeCache.txt`, `tested-sha256.json`, baseline/fixed
build and test logs, process exits, the original baseline test source, smoke
script/results/VTK outputs, and the HDA end-to-end log. Original scenes and
historical PF measurement files were not overwritten.

Before modifying production code, the first three new `[contact_cache]` cases
were compiled and run against the baseline: **461 assertions, 169 failed,
3 cases failed, exit 42**. The preserved log shows the second equal-position
fixed-stiffness form has no stencils instead of edge-vertex stencil
`(type=1, vertex IDs=2,0,1,-1)`, and zero energy instead of
`2.96651596019853336`. Its gradient and Hessian errors have norms
`14.3149364082069237` and `76.52425023029397266`. The fresh direct-build reference
has the contact. Both fixed and semi-implicit modes are exercised.

The finite-difference case's baseline failure was its nonzero-energy precondition
following other forms; it is not evidence of a separate analytical-derivative
bug. The concurrent case was added only after removing the shared cache, to
avoid intentionally running a known unsynchronized static-matrix mutation.
This reproduces a real-form ownership defect, not a historical scene failure.

## Findings and changes

| Finding | Evidence | Outcome |
| --- | --- | --- |
| A function-static position matrix lets A suppress B's first rebuild | A/B/A and B/A/B against direct fresh references | reproduced and fixed |
| Coordinate equality omits topology/support configuration | Separate meshes with edge (0,1) versus (0,2); support 1 versus .1 | reproduced and fixed within fixture scope |
| Equal-position filter/candidate changes can leave stale contacts | Enable/disable `can_collide` around candidate begin/end and full rebuilds | reproduced and fixed |
| Independent forms no longer write a shared position matrix | Two asynchronous workers, separate meshes/forms/callbacks, 16 samples each | fixed; concurrent fixture passes |

Implementation: `3abfcfe25` in `sdast9/polyfem:main`.
`BarrierContactForm::update_collision_set` now rebuilds on every notification.
A position-only shortcut was removed instead of adding an incomplete instance
key. The mesh's mutable `std::function` collision filter can also change captured
state without an observable generation number, so a cheap complete key is not
available in the current API. No process-global clearing workaround was added.

The existing candidate and stiffness caches remain owned by each form:

- `init`, `solution_changed`, and `update_quantities` request the rebuild.
- `line_search_begin` creates the form's candidates for its trial interval;
  subsequent notifications rebuild collisions from those candidates.
- `line_search_end` clears candidates and disables candidate reuse; the next
  notification performs a full broad-phase collision build.
- Per-stencil stiffness assignment still follows every collision rebuild, with
  the same frozen snapshot/cache/cap and refresh policy as before.

The effective CollisionMesh API exposes const topology/rest-geometry accessors
and no supported in-place topology mutation protocol for a live form. Tests use
separate meshes/forms for topology changes. `can_collide` is public and mutable;
its tested changes occur outside an active candidate interval or before a new
`line_search_begin`. Changing filtering during an already-built candidate
interval still requires ending/rebuilding that interval; this repair does not
introduce an implicit mid-line-search filter-mutation protocol.

The concurrent fixture uses independent forms, collision meshes, broad phases,
and thread-safe constant Hessian callbacks. It does not claim same-form mutation,
concurrent external mesh/filter mutation, or copying a form's shared broad-phase
pointer is safe. ThreadSanitizer and other platforms were not run.

Removing the shortcut permits more rebuild work at unchanged positions. The
smokes establish completion for the small public fixtures; a controlled large
scene performance comparison was not measured.

## Validation

The regression source is [test_contact_cache.cpp](../tests/test_contact_cache.cpp).
Its helper inherits the real form without overriding production lifecycle or
energy/derivative methods. Only the reference path directly calls IPC's collision
build, bypassing the production cache; references are newly constructed and use
a constant positive system Hessian `100 I`, unit masses and trim 1.

Coordinates are `(-1,0), (1,0), (0,.2)`, with a benign finite gap and identity
collision/FEM mapping. Exact sorted stencil identities are compared. Energy,
gradient and Hessian comparisons use absolute `1e-10` plus relative `1e-10`
(norms for derivatives). Central differences on all six DOFs use step `1e-6`
and absolute `1e-6` plus relative `1e-6` norm tolerance. These thresholds were
chosen before execution: the finite gap avoids singular conditioning, while
central differences need room for truncation and cancellation. No threshold was
relaxed after observing results. Hessians are unprojected.

| Check | Measured result | Exit / outcome |
| --- | --- | --- |
| Baseline ownership/lifecycle reproduction | 169/461 assertions failed | 42; expected failure |
| Rebuilt `PolyFEM_bin` and `unit_tests` | Both targets completed | 0 |
| `[contact_cache]`, seed 1 | 703 assertions / 4 cases, including both orders, unchanged positions, fresh initialization, configuration/candidate transitions, finite differences and concurrent independent forms | 0; pass |
| Plan's affected contact/AL/BC/filter/derivative selection, seed 1 | 1,189 assertions / 22 cases | 0; pass |
| `quasistatic-adaptive.json` | Four steps to t=1, zero error lines, reported total 2.7930 s | 0; pass |
| `quasistatic-semi-alhess.json` | Four steps to t=1, zero error lines, reported total .8707 s | 0; pass |
| `quasistatic-semi-friction.json` | Four steps to t=1, zero error lines, reported total .9628 s | 0; pass |
| `quasistatic-semi.json` | Four steps to t=1, zero error lines, reported total .8655 s | 0; pass |
| `transient-semi.json` | Four steps to t=1, zero error lines, reported total .9522 s | 0; pass |
| Houdini 22.0.429 `test_polyfem_hda.py` against rebuilt binary | End-to-end pass, including retired-floor compatibility/round-trip | 0; pass |
| clang-format, staged diff whitespace, local Markdown links | Checked changed files | pass |

The copied smoke script changed only `OUT`. Every per-scene process exit was
checked, and PVD time entries were `[0,.25,.5,.75,1]`; wrapper exit alone was not
used as proof. Reported times are solver log totals, not a baseline speed ratio.
Expected negative-test error logs in the affected suite did not fail assertions.
Build warnings about deprecated IPC APIs and duplicate link libraries remain.

New tests measure contact energy and its derivatives, not final physical
residuals, reaction balance, continuum accuracy, inversion fields, or work
balance. These latter quantities were **not measured** in this RB-01 session.
Successful smoke exits retain their configured numerical meaning; they do not
certify physical accuracy. No whole-suite, PF-08 sweep, PF-02 probe rerun, other
HDA tests, private scene, Ballburst or Teseo run is claimed. The known historical
`contact_2d` cube-on-floor mismatch was not investigated or regenerated.

## Publication and reproducibility

The implementation commit contains only the collision-cache repair, registered
regressions and the corrected PF-02 probe comment. This record and the RB status
update are published as a following documentation commit to the same remote and
branch; its hash is reported in the task completion. No HDA asset/source change,
companion pin change or dependency publication is needed.

The built binaries are tied to implementation `3abfcfe25` by saved source/binary
hashes; the following documentation changes do not change compiled sources.
Incoming untracked outputs remain untouched. Evidence and the refreshed parent
workspace README are local; no private inputs or large generated outputs are
included in the PolyFEM commits.

To reproduce the fixed regression in a fresh output directory, build the two
targets, then run the absolute path to `build/tests/unit_tests` with
`'[contact_cache]' --rng-seed 1`. For the historical baseline, add the new test
source/CMake registration to `bab5501a6` in an isolated checkout and run the three
non-concurrent cases (exclude `[contact_cache_parallel]`). Do not revert a shared
working checkout merely to reproduce this record.

## Next session handoff

All RB-01 stages are completed and validated within the stated scope. The
[plan status](robustness-plan.md) and parent README were updated. RB-02 is eligible
for selection; its coefficient/lifecycle inventory and model-decision boundaries
remain in force. No downstream implementation or new physical model is selected
by this record.
