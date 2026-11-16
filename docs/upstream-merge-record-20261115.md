# Upstream merge execution record

## 2026-11-16 — PF-1: conforming tetrahedral refinement

Merged upstream `d7525f554162f95307d587ac012bd870a14fc40c` (#547) into the isolated `upstream-merge` branch from fork `7aa161108daa87e53c2a0c2d93a3cd4a2bf3c8d2`.

Test data merged first: upstream `b125252765d8b3bae72e876743d57263f2ba705e` into fork `aed03ab9abf9eb54ca21dcbc3df3682dc8176e7f`, published as `feba1481682285a7febf896211a980a45a5c5e3b` on `sdast9/polyfem-data:fable-fixtures-upstream-merge`. All nine fork fixture files remained byte-identical; the isolated data checkout remains clean after testing.

### Resolution and preservation

Only expected conflict: `cmake/recipes/polyfem_data.cmake`. Kept the fork repository/recipe, changed only its data pin to the published data merge above. No files deleted. Upstream mesh-refinement changes and new tests retained; no fork changes were dropped. Upstream also adjusts the fixed-corotational derivative test's finite-difference step (not a production tolerance). No solver tolerance, controller default, CCD safeguard or trial-displacement cap was altered.

The plan's global configuration note says to point at the IPC integration worktree once available, while IPC-1/PF-4 specifically place PolyFEM consumption in PF-4. To isolate PF-1 and avoid adopting IPC behavior prematurely, this stage uses an isolated detached `ipc-pre-pf4` worktree at the unchanged fork pin `7b8fcac7`. PF-4 will switch to accepted IPC-1. PolySolve remains `f175cabe11deea2d3cafb74a6dec21a9e680ef69`. ITR is OFF because RM-0 has not passed; MISO remains OFF. No remeshing functionality is claimed, removed, or replaced by this task.

The old integration binary had dirty/older source provenance and was not used as an oracle. Built a clean pre-merge baseline at `7aa161108d` first, preserving its binary, build info and hash. All comparisons below use that clean baseline and identical dependency commits/settings.

### Ladder results

- **V1:** `PolyFEM_bin` and `unit_tests` compile successfully (AppleClang 21, macOS arm64, RelWithDebInfo, four build jobs).
- Additional focused refinement check: **2 cases, 83 assertions passed**.
- **V3:** all five one-thread smoke scenes exited 0 with no error lines. `quasistatic-semi`, `quasistatic-semi-friction`, `transient-semi`, `quasistatic-adaptive`, and `quasistatic-semi-alhess` each produced **five byte-identical VTUs**, 25/25 total, versus the clean baseline.
- **V5:** entire default `unit_tests` invocation with seed 1, no tag filter, under `caffeinate -s` and a 7,200-second watchdog: **443 cases, 5,179,481 assertions passed**, exit 0, 1,155.54 seconds. No shortened test selection, timeout, accepted failure or tolerance adjustment. This native result does not validate other platforms or CUDA.

Evidence is in sibling `PF-1/`: configure/build logs and exit receipts, `fork-fixtures-preserved.json`, focused-refinement log/receipt, both smoke directories and receipts, `smoke-comparison.json`, and full-suite log/start/receipt. Baseline and tested binaries/build-info snapshots are under `bin/PF-1-pre/` and `bin/PF-1/`.

PF-1 meets its specified native gate. As already stated in the plan, #547 changes refined tetrahedral results for `n_refs > 0`; no smoke uses refinement. Keep this behavior change on the REL-1 disclosure/confirmation list. No release, shared-checkout change, main-branch push or Houdini publication is authorized by this result.

### Related independent IPC-1 status

IPC-1 completed separately: native toolkit default suite passed 453 cases / 5,802,589 assertions, new interaction checks and Python flag bindings passed. Integration tip is `eec0ff82d2b7a0a9ce27fff0afd7de2c18e57d3a` after whitespace-only CI fixes. Remote formatting workflow 43910976196 succeeded. Its consumer validation remains at PF-4. RM-0 evidence/experimental branch is unchanged and still separate.
