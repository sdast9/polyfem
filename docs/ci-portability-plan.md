# GitHub CI failures and cross-platform repair plan

Audit date: **2026-09-13**. Source snapshot: **`3eecd190396ff07cb5ba4857beee23f88503fba2`**, `sdast9/polyfem:main`.

**The pushes reach GitHub successfully. The checks fail for several independent reasons:** a GCC compilation error, a Windows macro collision, overly strict floating-point assertions, formatting violations, and scene regressions or changed scene requirements. Fixing compilation will expose tests that currently cannot run. A successful local macOS build does not validate the Linux or Windows configurations.

This document records the investigation and work still required. **No solver, test, dependency, workflow, or production-default fix was applied by the original audit.** The [evidence record](ci-portability-evidence-2026-09-13.md) contains dated run/job links, measurements, verification scope, and the local evidence location. Source line numbers below refer to the audited revision.

**Implementation update, 2026-09-13:** the bounded CI-01 and CI-02 source/test/formatting repairs have now been implemented without changing solver behavior or production defaults. An isolated AppleClang build and the three affected CTest cases pass, and the pinned clang-format 21.1.8 check passes over the full tracked C/C++/CUDA-family tree. Native GCC/MSVC and all-platform focused-test acceptance remain pending the GitHub run triggered by publication.

**Implementation update, 2026-09-15 (RB-12 stage 3, [record](rb-12-validation.md#stage-3--ci-and-release-integration-2026-09-14)):** the native acceptance of CI-01 and CI-02 is read from [Build run 34903453744](https://github.com/sdast9/polyfem/actions/runs/34903453744) at `fffc722b9` — Linux GCC 13 Debug 312/312 and Release 340/344, Windows MSVC 19.44 Debug 309/309 and Release 328/328, macOS AppleClang Debug 311/311 and Release 339/343, with [pre-commit run 34903453923](https://github.com/sdast9/polyfem/actions/runs/34903453923) green; the only failures on any lane are the four CI-03–CI-06 scene groups (`standard`, `contact_2d`, `contact_3d`, `triangle_data`) on the two Release lanes that run them. **CI-01 and CI-02 are complete.** Of CI-07, RB-12 delivered the dependency-fork triggers (item 1), the `SCCACHE_CACHE_SIZE` fix and the always-on `LastTest.log` / `LastTestsFailed.log` / CLI-contract uploads (item 6, in part), the compiled-in build identity printed on every lane by the new CTest `cli_contract` (`PolyFEM_bin --build_info`: compiler, configuration, options, effective PolyFEM/IPC/PolySolve SHAs against the declared pins — item 2, in part) and one public integration run through the real executable on all three OSes (`quasistatic-semi`, single-threaded — a first step toward item 3). Two findings recorded by RB-12 are now CI-07 work items (see its 2026-09-15 update): the IPC Toolkit fork's two Windows lane failures on upstream code, and the cross-platform repeatability matrix. The user's decision of 2026-09-14 sets the required lanes to pre-commit, Linux Release and macOS Release; the four scene groups stay tracked exceptions with the owners below, and the Windows lanes are informational (green since `fffc722b9`).

## 1. Verified state

The latest completed [Build run 34757133039](https://github.com/sdast9/polyfem/actions/runs/34757133039), started September 13 at 12:27 UTC, reports:

| Job | Result | First blocking problem |
| --- | --- | --- |
| Linux / GCC 13.3 / DebugNoSymbols / TBB | Build fails; tests skipped | `BarrierContactForm.cpp:804`, array initializer rejected by `-Werror=missing-braces` |
| Linux / GCC 13.3 / Release / TBB | Build fails; tests skipped | Same initializer |
| Windows / MSVC 19.44 / DebugNoSymbols / CPP | Build fails; tests skipped | `test_hex_collision_surface.cpp:392`, identifier `near` conflicts with a Windows SDK macro |
| Windows / MSVC 19.44 / Release / CPP | Build fails; tests skipped | Same test compilation error |
| macOS / AppleClang 21 / arm64 / DebugNoSymbols / TBB | 274 tests pass | Does not include the 32 Release-only scene groups |
| macOS / AppleClang 21 / arm64 / Release / TBB | 302 pass, 4 fail, out of 306 tests | `triangle_data`, `contact_2d`, `standard`, `contact_3d`; six failing scene inputs in total |
| [pre-commit run 34757133029](https://github.com/sdast9/polyfem/actions/runs/34757133029) | Fails | Four files differ from clang-format 21.1.8 output |

The runners were Ubuntu 24.04 x64, Windows Server 2022 x64, and macOS 26 arm64. Earlier completed runs at `91c7ff8a` and `382c4c21` have the same compilation failures and three failing macOS Release groups. The older September 11 run at `ca5a7965` reached Linux/Windows tests and exposed the floating-point assertions described below.

At the 13:54 UTC metadata snapshot, the most recent 100 PolyFEM workflow runs contain 22 failed Build runs, 28 cancelled Build runs, 41 failed pre-commit runs, and 9 successful pre-commit runs. This is a bounded sample, not all repository history. The Build workflow cancels an in-progress run when another push arrives on the same branch; cancellations are not additional test failures.

## 2. Recommended work order

CI-01 and CI-02 are **complete** (native validation read on 2026-09-15, see the implementation update above). CI-03 through CI-09 remain **open**; CI-07 carries two items handed over by RB-12. IDs are specific to this CI plan and do not rename the existing RB work.

| Order / ID | Work | Completion evidence |
| --- | --- | --- |
| 1 / CI-01 | Repair GCC initialization and Windows test identifier | Both configurations compile on native GCC and MSVC |
| 1 / CI-02 | Correct analytical floating-point expectations; apply pinned formatting | Focused tests pass on all three platforms; repository formatting check passes |
| 2 / CI-03 | Make historical friction fixtures specify their intended policy | A/B comparison explains the new mismatches; historical and current-default coverage both pass |
| 2 / CI-04 | Repair mixed P1/P2 tetrahedral collision-surface extraction | Complete, valid surface and unchanged intended FE mapping; `standard` passes |
| 2 / CI-05 | Reconcile the microstructure workload with resource protection | Measured resource profile, successful supported run, and preserved budget-refusal coverage |
| 2 / CI-06 | Resolve the GCP cube-on-floor reference contract | Repeated, controlled comparison with justified reference policy |
| 3 / CI-07 | Close workflow and platform-coverage gaps | Active fork branches tested; common public integration tests run on each OS |
| 4 / CI-08 | Package and test the actual solver executable | Relocatable packages run on clean target machines |
| 4 / CI-09 | Validate the Houdini front-end on each OS | Export, launch, output import, failure handling, and paths with spaces pass |

CI-01 and CI-02 are one mechanical repair changeset. The scene items should remain separate, bounded changes with their own evidence and applicable RB records. Reaching green CI is an intermediate milestone; it does not establish a usable downloadable package or general physical accuracy.

## 3. Immediate compilation and test repairs

### CI-01 — GCC initializer and Windows macro collision

**Implemented:** the `std::array` now uses explicit nested braces, and the test-local `near` identifier is now `near_selector_count` everywhere in that test case. Local warning-policy and empty-`near`-macro syntax probes pass. Native GCC and MSVC builds remain the completion evidence.

**Complete (2026-09-15):** both configurations compile and their suites run on native GCC 13.3 (Linux) and MSVC 19.44 (Windows) at `fffc722b9`, Build run 34903453744; the Debug and Windows lanes are green there.

**Linux: confirmed compiler failure.** In [BarrierContactForm.cpp](../src/polyfem/solver/forms/BarrierContactForm.cpp), `interpolated_stiffness()` initializes:

```cpp
std::array<int, 4> slot = {-1, -1, -1, -1};
```

[polyfem_warnings.cmake](../cmake/polyfem/polyfem_warnings.cmake) enables `-Werror=missing-braces`. GCC rejects this spelling of the nested aggregate initializer. The code was introduced by `1ea71d93` on September 11. Use explicit nested braces, for example `std::array<int, 4> slot{{-1, -1, -1, -1}};`, or value-initialize and call `slot.fill(-1)`. Keep the warning policy. Rebuild both the library and tests under the existing Linux Release/DebugNoSymbols settings; later diagnostics may appear once this first blocker is removed.

**Windows: confirmed failure with a reproduced macro mechanism.** [test_hex_collision_surface.cpp](../tests/test_hex_collision_surface.cpp):392 declares `int exact = 0, near = 0;`. The Windows SDK defines `near` as an empty macro in [minwindef.h](https://github.com/microsoft/win32metadata/blob/main/generation/WinSDK/RecompiledIdlHeaders/shared/minwindef.h). Expansion makes the declaration and subsequent expressions invalid, matching the MSVC C2059/C2143 errors. `NOMINMAX` does not suppress this separate macro. Rename the local variable and its uses to `near_selector_count`; do not remove the regression test or globally undefine SDK macros. This test arrived in `8f76fef6` on September 12.

A small isolated Apple-Clang syntax probe with the SDK's empty `near` macro fails with the old identifier and passes with the proposed identifier. This verifies the mechanism, not the full Windows build. Native MSVC compilation and the `[rb22]` tests remain the acceptance check.

### CI-02 — Floating-point expectations and formatting

**Implemented:** the three vector assertions retain exact cardinality checks and compare the single analytical value using `Approx(100.).epsilon(4 * std::numeric_limits<double>::epsilon())`. The JSON median is extracted as `double` and uses the same bound. This relative tolerance is about `8.88e-16`, covers the observed `2.84e-16` relative discrepancy, and remains specific to these well-conditioned synthetic calculations. clang-format 21.1.8 was applied to all four files identified by the audit and rechecked over the tracked tree.

**Complete (2026-09-15):** the focused tests pass on all three platforms in Build run 34903453744 (every lane is green apart from the CI-03–CI-06 scene groups), and the repository formatting check passes (pre-commit run 34903453923; green since `1f6f826fa`, run 34854316144).

**Confirmed on earlier native Linux and Windows runs; currently masked by CI-01.** At `ca5a7965`, both platforms compute `99.99999999999997158` where these tests demand literal `100.0`:

- [test_kappa_continuity.cpp](../tests/test_kappa_continuity.cpp):146,179,227 compares whole vectors using exact `==`.
- [test_semi_implicit_coefficients.cpp](../tests/test_semi_implicit_coefficients.cpp):103 compares the JSON `batch_median` directly with `100.`.

Those expectations are still present at the audited head. The discrepancy is two double-precision representable steps below 100, about `2.84e-16` relative error. Check the vector size exactly, then compare each computed analytical value with a narrowly justified `WithinRel`, `WithinAbs`, or explicitly configured `Approx`. Extract the JSON value as `double` before comparison. A bound tied to machine epsilon is appropriate for these small, well-conditioned synthetic calculations; document it and verify it on each compiler.

Retain exact equality where the contract really requires the same stored coefficient or unchanged force across a refresh. Do not convert all tests to approximate comparisons, change the coefficient formula, or loosen scene goldens to repair these four assertions. Run the affected tests again after the compilation repair; the current full result on Linux/Windows is still unknown.

**Formatting: reproduced locally against the audited head.** A read-only clang-format **21.1.8** check of 639 tracked C/C++/CUDA-family files identifies the same four files as GitHub:

- `src/polyfem/io/OutData.cpp`
- `tests/test_kappa_continuity.cpp`
- `tools/rb04/candidate_probe.cpp`
- `tools/rb04/contact_path_probe.cpp`

Use the version pinned by [.pre-commit-config.yaml](../.pre-commit-config.yaml), apply its changes, review the diff, and run `pre-commit run --all-files`. Checking only the modified solver files misses the committed probe sources. The audit's dry run changed no source files.

## 4. Scene failures requiring distinct treatment

`verify_run.cpp` reports `3 == 0` for its internal `SOLVE_FAILED` enum and `4 == 0` for its internal authentication/reference mismatch enum. These numbers are **not** the standalone executable's exit-status contract. An exception printed by a negative regression test is also not automatically a failing CTest: use the test conclusion and captured scene path.

### CI-03 — Pin the intended friction policy in historical fixtures

**Confirmed new mismatches; exact causal contribution still requires an A/B run.** At `3eecd190`, these three fixtures fail their historical reference comparisons but passed in the inspected `91c7ff8a` macOS Release run:

| Fixture | Largest reported relative discrepancy among its six metrics |
| --- | ---: |
| `contact/examples/3D/higher-order/ball-bounce/P4-dt=0.01.json` | `0.44076346` (44.1%, `err_h1_semi`) |
| `contact/examples/2D/large-ratios/large-mass-ratio.json` | `0.00622455` (0.622%, `err_h1_semi`) |
| `contact/examples/3D/large-ratios/large-mass-ratio.json` | `0.31665698` (31.7%, `err_h1_semi`) |

The intervening approved RB-10 change `756070f4` changes the **general** `/solver/contact/friction_iterations` default from 1 to 2 and the **semi-implicit-specific** `friction_lag` default to `realized_force`. The complete `common` chains of all three fixtures were inspected: none pins the iteration count. This provides a concrete policy-change hypothesis, not proof that every numerical discrepancy has that cause. The ordinary adaptive-IPC scenes do not automatically exercise the semi-implicit force-lag setting.

Required work:

1. Reproduce these exact fixtures on isolated copies at the before/after revisions, keeping their recorded test duration, material/contact settings, output paths, and solver selection.
2. At the current revision, compare omitted `friction_iterations`, explicit `1`, and explicit `2`; record the fully resolved input and friction work/residual diagnostics, including the complete `common` chain. Do not treat these differences as floating-point noise.
3. If explicit historical settings recover the old results, encode those settings in the historical fixtures (a versioned fixture overlay is an option) and add separate tests for the approved current defaults. If they do not, bisect the remaining behavior change before altering references.
4. Preserve the user-approved production defaults. If intentionally generating new current-policy reference data, require independent numerical checks and a reviewed reference change; never bulk-regenerate just to obtain green checks.

Coordinate the explanation and validation with [RB-10](rb-10-validation.md). This audit does not reopen its approved default decision.

### CI-04 — Mixed P1/P2 tetrahedral contact boundary

**Confirmed extraction rejection; repair not implemented.** `standard` fails on `multi-material/stretch-cubes.json`. The log reports **437 of 7,610 boundary faces skipped**, with a P2 tetrahedron having **five owned face nodes**. The fixture assigns orders 1, 2, and 1 to its three material regions. This is a mixed-order tetrahedral problem, distinct from the documented Q3 hexahedral defect.

In [OutData.cpp](../src/polyfem/io/OutData.cpp), the simplex extraction path omits basis entries whose `global().size() != 1`, then only triangulates 3, 6, 10, or 15 owned nodes. The new completeness check correctly refuses the resulting partial surface. Earlier success with missing collision faces is not an acceptable reference implementation.

Required work: reproduce one affected P1/P2 interface, trace the constrained basis weights, and retain the complete geometric boundary and collision-to-FE displacement map. Either extend extraction for these weighted boundary nodes or validate and explicitly route this supported case through an appropriate sampled proxy. Check surface coverage, nondegenerate faces, expected topology, mapping dimensions, partition of unity, displacement agreement, and the actual contact solve. Keep RB-22's named-error guards. A proposed proxy/model change must follow the existing RB decision boundaries. Passing by disabling contact, lowering the FE order, or suppressing the completeness error does not fix this fixture.

### CI-05 — Resource limit rejects the microstructure fixture

**Confirmed workload/default-budget incompatibility.** `triangle_data` fails on `contact/examples/3D/higher-order/microstructure.json`. HashGrid estimates **65,153,532 pre-filter candidate emissions**, exceeding the automatic **50,000,000** limit before allocation. The estimate concerns raw pair emissions before filtering/deduplication; it is not a count of actual contacts or bytes. The log gives 19,240 collision vertices, 57,768 edges, and 38,512 faces.

The refusal follows the approved RB-05 default protection. Measure peak memory, raw and surviving candidates, and runtime on the target runners before selecting a supported resource profile. Then either reduce the conservative enumeration cost without losing collision pairs, or provide a documented fixture-specific budget on a runner able to execute it. Keep a separate small test that proves automatic limits reject oversized demand cleanly. If the full case needs a resource-intensive test job, retain it there with an explicit support policy; do not silently remove it from coverage.

Preserve CCD and the separate trial-displacement cap. A larger global allocation budget or disabled protection is not part of this investigation's authorization. Record the resulting decision and scope in [RB-05](rb-05-validation.md).

### CI-06 — GCP cube-on-floor reference mismatch

**Confirmed recurring reference mismatch; reference policy unresolved.** `gcp-contact/cube-on-floor/run.json` already fails before RB-10. At `91c7ff8a`, the largest reported relative discrepancy is about `0.003881`; at `3eecd190`, about `0.006565`, while the fixture's margin is `1e-5`. This scene uses GCP/SmoothContact, so a semi-implicit coefficient change is not automatically its cause.

The pinned `polyfem-data` history was checked directly: `f52db28` changed this scene's margin to `0.01`; bulk update `6f8f569` restored `1e-5`. The current [verify_run.cpp](../tests/verify_run.cpp) also unconditionally emits `out["margin"] = 1e-5` when generating reference data. This explains a concrete fragility in reference maintenance, but does not independently validate `0.01` for the current solver.

Required work: establish explicit policy settings, repeat the exact scene across fresh processes and platforms, inspect the numerical residuals and solution differences, and compare with the relevant upstream/data revisions. Separate the old mismatch from new default-policy effects. Fix reference generation to preserve an existing scene-specific margin, with focused coverage. Change the stored tolerance or golden values only when justified by this evidence. Publish approved data changes to a writable, versioned home and update [polyfem_data.cmake](../cmake/recipes/polyfem_data.cmake); a local edit to `data/` does not reach clean CI clones.

## 5. Make the checks represent the supported product

### CI-07 — Workflow triggers, test selection, and reproducible builds

**Verified gaps:**

- The active IPC branch is `semi-implicit-stiffness`; its Build workflow's push trigger covers only `main`. GitHub reports active workflows but no runs in the returned history. PolySolve's active branch is `iteration-callback`; its Build workflow likewise triggers pushes only on `main`, and its only returned Build result is from July 18 on `main`. PolyFEM linking these dependencies does not run their standalone test suites.
- [tests/verify_run.cpp](../tests/verify_run.cpp):250 assigns hidden `[.][run]` tags unless `NDEBUG` is defined and `WIN32` is absent. Debug and Windows therefore do not share the Unix Release scene coverage. Its Pardiso-related comment should be re-audited against the current Eigen fallback and actual backend requirements.
- PolyFEM tests use TBB on macOS/Linux and CPP on Windows. CPP still links TBB for IPC internals; it is not a TBB-free product. The current matrix does not distinguish an OS issue from a threading-backend issue.
- The shared developer build uses RelWithDebInfo and local IPC/PolySolve source overrides. Clean GitHub builds use committed recipe pins and different feature/toolchain settings. Do not use the shared build as evidence of a clean-clone pass.
- At this snapshot, only PolyFEM Build and pre-commit are enabled. Artifacts, Nightly, Coverage, Docs, and Docker are manually disabled. Their absence is not success. Re-enable only workflows selected for the fork's intended use; publishing workflows require fork-specific destinations and credentials to be reviewed first.

**Update 2026-09-15 (RB-12 stage 3):**

- Item 1 is done for the triggers: `sdast9/ipc-toolkit@c24d803e` and `sdast9/polysolve@bce32a39` (workflow files only) run `Build` on `semi-implicit-stiffness` / `iteration-callback` and by `workflow_dispatch`; PolyFEM's recipe pins follow at `4f5acc773`. First native outcomes: PolySolve [run 34889533898](https://github.com/sdast9/polysolve/actions/runs/34889533898) **6/6 green**; IPC Toolkit [run 34889554146](https://github.com/sdast9/ipc-toolkit/actions/runs/34889554146) **4/6** — Linux and macOS Debug/Release green, both Windows lanes failing on upstream code under the upstream workflow's Windows toolchain (MinGW GCC 15.2.0, not the MSVC that builds PolyFEM): Debug stops in the oneTBB 2022.3.0 dependency build (`oneapi/tbb/profiling.h:148: cannot convert 'const char*' to 'const wchar_t*'`, before any toolkit source compiles); Release builds and passes 284/285, the upstream `Smooth barrier potential real sim 2D C^2` test segfaulting (a GCP/smooth-contact test the fork does not touch). Logs: `outputs/rb-12/20260914T142106Z-repeat/ci-stage3/ipc/`. **New work item (8 below).**
- Item 6 is done for `LastTest.log`, `LastTestsFailed.log` and the CLI contract's manifests/logs (`if: always()`, all three OSes, `f5f59db26`); JUnit results, configure metadata and failing-scene diagnostics remain.
- The sccache paragraph below is resolved: `SCCACHE_CACHE_SIZE: "1G"` is the job environment (`f5f59db26`).
- Item 2 is partly covered: `PolyFEM_bin --build_info` (compiled in before every build) prints compiler, configuration, options and the effective PolyFEM/IPC/PolySolve SHAs against the declared pins, and CTest `cli_contract` prints it on every lane; presets, a pinned runner OS/toolchain and the cold-cache build remain.
- Item 3 has a first public integration run on all three OSes through the real executable (`cli_contract`: `quasistatic-semi`, single-threaded, exit statuses 0/1/3 checked); the named subset, the expected counts and the `[.][run]` gating remain.
- RB-12's controlled repeatability matrix (`tools/rb12/repeat.py`, rule in `tools/rb12/README.md`) ran on one host only — macOS arm64, AppleClang, RelWithDebInfo; the plan's cross-platform/compiler comparison is pending and needs another host. **New work item (9 below).**

Required changes:

1. Trigger IPC/PolySolve checks on their maintained fork branches and PRs, with manual dispatch for diagnosis. Record the exact dependency SHAs; publish a companion fix before advancing PolyFEM's pin.
2. Provide checked-in CMake presets for the tested configurations, pin the primary runner OS/toolchain, and print compiler, architecture, dependency/data SHAs, options, and effective solver backends. Add a periodic cold-cache configure/build to catch dependency and cache drift.
3. Keep native Release and Debug coverage. Register a named common public integration subset on all three OSes; require nonzero expected test counts and report exclusions. Enable optional backend tests according to capability instead of hiding every Windows scene.
4. Add a common portable Eigen-backed lane and separate acceleration/backend lanes. Decide and document supported TBB/CPP combinations; include at least enough common threading coverage to isolate platform-specific failures.
5. Use explicit bounded test/build concurrency. `CTEST_PARALLEL_LEVEL` counts test processes, not each solver's internal threads. The existing Linux `OMP_NUM_THREADS=1` is useful but does not limit all TBB work.
6. Upload `LastTest.log`, `LastTestsFailed.log`, JUnit results, configure metadata, and failing scene diagnostics with `if: always()`. Preserve per-test output directories and original fixtures. Never turn a failure into a warning with `continue-on-error` on a required lane.
7. Run the pinned formatter before publishing. Reduce redundant expensive runs through reviewed path filters or a cheap-check/build split, while preserving checks required by branch protection. Keep cancelled runs separate from completed results.
8. **IPC Toolkit fork, Windows lanes (from RB-12).** Reproduce the two failures against upstream `ipc-toolkit` at the fork's base commit to confirm they are upstream; then either make the lane match the consumer's toolchain (PolyFEM builds IPC with MSVC 19.44, where the same sources compile and pass inside PolyFEM's Windows lanes) or pin a oneTBB revision that compiles under MinGW GCC 15, and report the segfaulting upstream test with its reproduction rather than deleting it. Until then the IPC Windows lanes are informational; no fork source is involved.
9. **Cross-platform repeatability (from RB-12).** Run `python3 tools/rb12/repeat.py --binary <PolyFEM_bin> --output <fresh> --fixtures quasistatic-semi --threads 1 --repeats 3 --verify` (serial: the rule requires bit-identical endpoints) and a threaded pair on the Linux and Windows runners as an informational job or on another host, upload `summary.json` / `summary.md`, and append the outcome to the RB-12 record's stage-2 section as measured evidence. Thread-count divergence on friction fixtures is a documented limit (1.7e-4, RB-12), not a failure of this item.

**Resolved 2026-09-14 (`f5f59db26`).** The Windows setup also ran `sccache --max-size=1.0G`, which sccache 0.17.0 rejects. The setup step nevertheless succeeds after later commands; this is **not** the reported build blocker. Set `SCCACHE_CACHE_SIZE=1G`, as documented by [sccache](https://github.com/mozilla/sccache/blob/main/docs/Local.md), and make command failures propagate. Avoid copying ccache-only options into sccache commands.

### CI-08 — Installable solver packages

The disabled [artifacts workflow](../.github/workflows/artifacts.yml) builds the GUI with tests and Python disabled. [app/CMakeLists.txt](../app/CMakeLists.txt) packages `polyfem_app`; it does not install the `PolyFEM_bin` CLI required by the Houdini workflow. A successful GUI artifact build alone would leave this distribution requirement unmet.

Create a release path for the CLI and its required runtime libraries, with the GUI separately tested if it is offered. Test the extracted/installed package in a clean runner job, without the source tree, CPM cache, or build-directory library paths. Launch it from a directory containing spaces, run the public integration scenes, and verify exit status, JSON/VTU/PVD output, and unsupported-solver diagnostics. Check runtime library resolution, architecture, and redistribution notices.

| Target | Required release decision and validation |
| --- | --- |
| macOS arm64 | Native build/package, explicit minimum macOS/deployment target, relocatable dylibs; test downloaded application behavior if distributing a GUI |
| macOS x86_64 | Add a native Intel runner and package if Intel Macs are supported; current arm64 success says nothing about Intel binaries |
| Linux x86_64 | Choose oldest supported distribution/glibc baseline; build against that contract, use portable CPU flags, and test runtime dependencies on a clean supported distribution |
| Windows x86_64 | MSVC build, documented VC runtime and bundled DLL policy, `.exe` discovery, working paths with spaces, and native solver execution |

GitHub's [runner reference](https://docs.github.com/en/actions/reference/runners/github-hosted-runners) lists arm64 and Intel macOS labels separately. Do not infer both architectures from `macos-latest`. Windows/Linux ARM support is not established by this audit and requires separate lanes if selected.

Linux continuous CI enables `POLYFEM_PORTABLE_BUILD`, but the artifact workflow omits it. Apply a consistent declared CPU baseline to distributed binaries and dependencies; a build that works only on the compilation runner is not portable. Python expressions are a separate product capability: the present artifact workflow disables them, so a Python-free package must state that limit and a package offering expressions must validate its Python runtime explicitly.

### CI-09 — Houdini integration and launch paths

This is a source-level portability review, **not a recorded GitHub failure**: `sdast9/houdini-plugins` has no workflows/runs in the queried metadata.

`houdini_HDAs/src/polyfem/PythonModule.py::write_params()` joins arguments with spaces without quoting the executable path before passing them through Terminal/AppleScript, xterm/bash, or cmd. An installation path such as `C:\Program Files\...` therefore needs repair and native testing. The Linux path assumes xterm is installed. The background path already uses an argument list plus `cwd`; retain that pattern for direct execution and implement/test each terminal launcher with platform-appropriate quoting.

The HDA test helpers assume a sibling `polyfem/build/PolyFEM_bin`, without a Windows `.exe` suffix or a build/package-location override. The parent-workspace `run-smoke.sh` is a zsh script with absolute `/Users/...` binary, scene, and output paths, and does not reliably return the aggregate solver failure status. Replace this development-only assumption with a portable runner accepting explicit executable, scene, and fresh output directories, and a nonzero status if any required run fails.

Add tests for each supported Houdini/Python version and OS: load the published HDA, export strict JSON, locate the packaged solver, launch foreground/background with spaces and non-ASCII paths, surface resource/solver errors, and read the produced outputs. Pure parsers can run without Houdini; HDA integration requires a Houdini-capable licensed environment or a documented native manual gate. Publish HDA fixes through the separate `houdini-plugins` procedure.

## 6. Acceptance and next session

The first implementation pass covers **CI-01 and CI-02** in an isolated checkout/build. Native matrix validation remains the final acceptance gate. The focused local selection was:

```sh
ctest --test-dir build --output-on-failure -R '^(RB-20 force continuation carries endpoint coefficients|Semi-implicit batch median ignores zeros and applies a relative floor|max_order lattice proxy of Q2/Q3/serendipity hexahedra is closed)$'
```

For each scene repair, reproduce the named failure first with the same data pin and effective policy. Save failures as well as successful runs. Then run the affected test group and the common public integration subset; finish with the relevant full supported matrix. The test data includes cases that intentionally exercise failure, so assess expected outcomes rather than searching logs for the word `error` alone.

Cross-platform usability is ready to claim only when:

- A fresh clone resolves the published dependency/data pins and builds with documented commands on the supported native targets.
- Required formatting, unit, and public integration checks complete successfully, with all omissions explicit and the known blockers above resolved.
- The packages' actual CLI/GUI capabilities match their documentation and run after relocation on clean machines.
- The Houdini workflow passes its native integration checks if offered as part of the supported product.
- The support table states minimum OS, CPU architecture, compiler/runtime, solver backends, Python/Houdini versions, and known numerical/model limitations.

This work must retain CCD, the trial-displacement cap, the retired status of the constraint floor, and approved RB defaults unless a separate documented decision changes them. Use public fixtures; do not run Teseo. The current audit ran no solver scenes and made no numerical/model changes. Documentation-only verification consists of evidence reconciliation, source inspection, the formatter dry run, the isolated macro probe, and Markdown link/diff checks.
