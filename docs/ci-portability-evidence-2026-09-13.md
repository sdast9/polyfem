# CI portability audit: evidence record

Date: 2026-09-13. Main report: [repair plan](ci-portability-plan.md).

## Scope and identity

GitHub metadata and job logs were read with the authenticated GitHub CLI. The final workflow-history snapshot was captured at **2026-09-13 13:54:20 UTC**. Detailed logs were examined for the September 11 run at `ca5a7965`, September 12 run at `382c4c21`, and September 13 runs at `91c7ff8a` and `3eecd190`. This is a diagnosis/documentation task; it did not rerun the solver scenes or change production sources, tests, workflows, dependencies, or defaults.

| Component | Audited/pinned revision |
| --- | --- |
| PolyFEM | `3eecd190396ff07cb5ba4857beee23f88503fba2` |
| IPC Toolkit recipe | `bb795446812a3d3c7b5358c4cdbf26c6bbe48a16` |
| PolySolve recipe | `ee5b296a690ce225fb34f6f375d27f9ea2a16027` |
| polyfem-data recipe and inspected local data | `e0efb6ba291e3acfc8a5e12e66486a7246849b30` |
| Houdini published head checked during the audit | `f10f9c850f7ab740fdb31246a72ccefa83624236` |

Other work continued publishing while this report was being written. The results below belong to the named revisions; publishing this document on a later branch head does not test that later implementation. The shared developer checkouts/build were not reset, updated, reformatted, or rebuilt by this task. Documentation was prepared in a separate clone.

## Current snapshot: exact GitHub jobs

Build run: [34757133039](https://github.com/sdast9/polyfem/actions/runs/34757133039), `3eecd190`, push on `main`, created 2026-09-13 12:27:19 UTC.

| Job | Link | Observed outcome |
| --- | --- | --- |
| Linux DebugNoSymbols / TBB | [103723329836](https://github.com/sdast9/polyfem/actions/runs/34757133039/job/103723329836) | Configure passes; build fails at `BarrierContactForm.cpp:804`; tests skipped |
| Linux Release / TBB | [103723329886](https://github.com/sdast9/polyfem/actions/runs/34757133039/job/103723329886) | Same failure |
| Windows DebugNoSymbols / CPP | [103723329680](https://github.com/sdast9/polyfem/actions/runs/34757133039/job/103723329680) | Configure passes; test source compilation fails; tests skipped |
| Windows Release / CPP | [103723329877](https://github.com/sdast9/polyfem/actions/runs/34757133039/job/103723329877) | Same failure |
| macOS DebugNoSymbols / TBB | [103723329872](https://github.com/sdast9/polyfem/actions/runs/34757133039/job/103723329872) | Build passes; 274/274 tests pass |
| macOS Release / TBB | [103723329772](https://github.com/sdast9/polyfem/actions/runs/34757133039/job/103723329772) | Build passes; 302/306 tests pass, four groups fail |
| pre-commit | [103723329449](https://github.com/sdast9/polyfem/actions/runs/34757133029/job/103723329449) | clang-format modifies four files, so the hook fails |

Linux compiler: GNU 13.3.0, Ubuntu 24.04 x64. Windows compiler: MSVC 19.44.35228.0, Windows Server 2022 x64. macOS compiler: AppleClang 21.0.0.21000101, macOS 26 arm64. These are the recorded runner configurations, not minimum supported product versions.

### Compiler diagnostics

Linux, `BarrierContactForm.cpp:804`:

```text
error: missing braces around initializer for 'std::__array_traits<int, 4>::_Type'
[-Werror=missing-braces]
std::array<int, 4> slot = {-1, -1, -1, -1};
```

Windows, `test_hex_collision_surface.cpp`:

```text
(392): error C2059: syntax error: '='
(396): error C2059: syntax error: '+='
(398): error C2059: syntax error: '<<'
(399): error C2059: syntax error: '<='
```

The source declares and uses `near` on precisely those lines. Microsoft's [SDK header](https://github.com/microsoft/win32metadata/blob/main/generation/WinSDK/RecompiledIdlHeaders/shared/minwindef.h) contains an empty `#define near`. A local syntax probe using that macro and Apple Clang exits 1 with the original identifier and 0 after renaming it. A native MSVC rebuild of the proposed fix was not performed.

### Six failing macOS Release scene inputs

The four failing CTest groups contain six failing fixtures. This table excludes intentional error messages in passing negative tests.

| CTest group | Captured scene input | Failure evidence |
| --- | --- | --- |
| `triangle_data` | `contact/examples/3D/higher-order/ball-bounce/P4-dt=0.01.json` | Reference mismatch; `err_h1_semi` 0.0013129920252841353 → 0.0007342731213253285; relative difference 0.4407634569 |
| `triangle_data` | `contact/examples/3D/higher-order/microstructure.json` | HashGrid requires 65,153,532 candidate emissions against a limit of 50,000,000; solve refused before allocation |
| `contact_2d` | `contact/examples/2D/large-ratios/large-mass-ratio.json` | Reference mismatch; `err_h1_semi` 0.1463522823796857 → 0.14544130541023184; relative difference 0.0062245491 |
| `contact_2d` | `gcp-contact/cube-on-floor/run.json` | Reference mismatch; `err_h1_semi` 0.0983232623925236 → 0.09767777258669319; relative difference 0.0065649755 |
| `standard` | `multi-material/stretch-cubes.json` | Incomplete collision surface: 437/7,610 boundary faces skipped; P2 tetrahedral face has five owned nodes |
| `contact_3d` | `contact/examples/3D/large-ratios/large-mass-ratio.json` | Reference mismatch; `err_h1_semi` 0.051938087521556535 → 0.03549152967293719; relative difference 0.3166569782 |

The reference-comparison margin in these numerical-mismatch records is `1e-5`. A relative difference in an aggregate test metric is not itself a measured percentage error in the physical solution.

The same resource refusal, incomplete-surface refusal, and cube-on-floor mismatch are present at `91c7ff8a`. Its [macOS Release job](https://github.com/sdast9/polyfem/actions/runs/34730919919/job/103653427421) reports 299/302 passing tests, with `triangle_data`, `standard`, and `contact_2d` failing. The P4 ball and both large-mass-ratio fixtures authenticate successfully in that earlier log. The [debug job](https://github.com/sdast9/polyfem/actions/runs/34730919919/job/103653427492) passes 270/270.

The intervening [RB-10 defaults commit](https://github.com/sdast9/polyfem/commit/756070f44da15b2565334ec9ec3ac70868a3f3a0) changes friction iterations from 1 to 2. The complete `common` chains of all three newly failing fixtures were read from the pinned dataset; none supplies `solver/contact/friction_iterations`. P4 resolves through `P4.json`, `P1.json`, `3D/higher-order/common.json`, then `contact/examples/common.json`. This is evidence for the plan's default-policy hypothesis. No isolated before/after settings comparison was run here, so the precise cause of every new mismatch remains open.

### Hidden scene coverage

[verify_run.cpp at the audited revision](https://github.com/sdast9/polyfem/blob/3eecd190396ff07cb5ba4857beee23f88503fba2/tests/verify_run.cpp#L250) selects `[run]` only for `NDEBUG && !WIN32`; other configurations receive `[.][run]`. That explains why a green debug job does not establish a passing Release scene suite. Windows also has capability-related feature differences; counts should be compared by registered test name and supported capability, not just totals.

## Earlier native runs expose additional assertions

Run [34646789405](https://github.com/sdast9/polyfem/actions/runs/34646789405), `ca5a79655023f206b3ef11ea692b57cf6a985762`, created 2026-09-11 20:55:58 UTC, completed compilation on all inspected platforms.

| Job | Failing tests |
| --- | --- |
| [Linux DebugNoSymbols 103419415015](https://github.com/sdast9/polyfem/actions/runs/34646789405/job/103419415015) | `RB-20 force continuation carries endpoint coefficients`; `Semi-implicit batch median ignores zeros and applies a relative floor` |
| [Linux Release 103419414963](https://github.com/sdast9/polyfem/actions/runs/34646789405/job/103419414963) | Same two, plus `contact_2d` |
| [Windows DebugNoSymbols 103419414921](https://github.com/sdast9/polyfem/actions/runs/34646789405/job/103419414921) | Same two synthetic coefficient tests |
| [Windows Release 103419415223](https://github.com/sdast9/polyfem/actions/runs/34646789405/job/103419415223) | Same two synthetic coefficient tests |
| [macOS Release 103419415008](https://github.com/sdast9/polyfem/actions/runs/34646789405/job/103419415008) | `contact_2d` |

Representative Linux/Windows expansions:

```text
REQUIRE( f.scales() == std::vector<double>{100.} )
{ 99.99999999999997158 } == { 100.0 }

CHECK( f.diagnostic_state()["batch_median"] == 100. )
99.99999999999997 == 100.0
```

The strict analytical comparisons remain at `3eecd190`. Historical failures establish the portability defect in those assertions; a current repaired native run is still required, especially because subsequent solver changes can affect other assertions.

## Formatting and configuration verification

Local command mechanism: clang-format **21.1.8**, `--dry-run --Werror`, over 639 tracked C/C++/CUDA-family files in the isolated clone at `3eecd190`. Exit **1**; no formatting edits made. The violating-file list agrees with GitHub:

```text
src/polyfem/io/OutData.cpp
tests/test_kappa_continuity.cpp
tools/rb04/candidate_probe.cpp
tools/rb04/contact_path_probe.cpp
```

The GitHub API reports PolyFEM Build and pre-commit active. Artifacts, Coverage, Docker, Docs, and Nightly are `disabled_manually`. IPC workflows are active but their maintained branch is outside the Build push filter. PolySolve likewise excludes `iteration-callback` from its Build push filter; its returned Build/Coverage runs are successful July 18 `main` runs. Houdini has no returned workflows or runs. No workflow was enabled, disabled, rerun, or edited by this audit.

The Windows job also prints `error: unexpected argument '--max-size' found` for `sccache --max-size=1.0G`. It continues successfully through setup/configure and fails later during compilation. This setup defect is secondary and distinct from the build error.

## Data-history verification

The local `polyfem/data` checkout was verified at the same `e0efb6ba...` pin used by CI. Its history directly shows:

- [f52db28](https://github.com/polyfem/polyfem-data/commit/f52db28): cube-on-floor margin `1e-2`.
- [6f8f569](https://github.com/polyfem/polyfem-data/commit/6f8f569): cube-on-floor margin `1e-5`.

`verify_run.cpp` still writes `out["margin"] = 1e-5` during reference generation. The audit did not alter test data, regenerate goldens, or establish a replacement tolerance.

## Evidence files and reproducibility

Local parent-workspace evidence directory:

```text
outputs/ci-portability/20260913T0145Z/
```

The directory name identifies the initial audit session; collection continued after the user's September 13 continuation. It contains `github-state.json`, per-run job JSON, `heads-at-report.json`, `friction-input-chains.json`, formatter version/results/log, the isolated syntax probes, compressed ANSI-cleaned job logs, and `log-manifest.json` with job IDs, line counts, sizes, and SHA-256 hashes of the cleaned logs. Full build logs are retained locally rather than committed as large repository artifacts. The diagnostic facts and durable run links needed to use the plan are preserved in this document.

Example read-only retrieval commands:

```sh
gh run view 34757133039 --repo sdast9/polyfem --json jobs,headSha,conclusion,url
gh api repos/sdast9/polyfem/actions/workflows
gh api --allow-escape-sequences repos/sdast9/polyfem/actions/jobs/103723329836/logs > linux-debug.log
```

The log-output option is needed by the installed GitHub CLI because compiler logs contain terminal formatting. Original job conclusions, not search hits for `Failed` in CMake capability probes, determine whether configure/build/test stages failed.

Documentation validation: reconcile every reported failing fixture with its job output; compare culprit code against the pinned revision; verify relative Markdown links and the scoped Git diff. No new claim of full native build success, numerical correctness, or package/Houdini usability follows from this audit.
