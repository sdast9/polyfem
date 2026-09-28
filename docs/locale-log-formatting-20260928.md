# Locale-dependent debug log formatting crashes without en_US.UTF-8 (2026-09-28)

Date: 2026-09-28. Status: **fixed and validated within the scope below**
(cloud session, branch `cloud/locale-fix`). Source finding:
[docs/ci-portability-plan.md](ci-portability-plan.md) section 1 (cloud Linux
full-suite run, 2026-09-28).

## Contract and authorization

- Scheduled task: remove a hard-coded locale that crashes PolyFEM on
  machines without `en_US.UTF-8` generated, and pin the CI-05 data-fork
  fixture change. In scope: the log-formatting fix and a search for the same
  pattern elsewhere in `src/`, plus a regression test. Not authorized and
  not done: any change to solver behavior, defaults, stored golden values or
  tolerances.
- This is a portability/log-formatting repair, not a named RB item; no
  RB/CI plan record besides the CI portability plan's finding tracks it.

## Reproduction

`src/polyfem/varforms/NonlinearElasticVarForm.cpp:1043-1047` and
`src/polyfem/legacy/State.cpp:1378-1382` each built a
`std::locale("en_US.UTF-8")` to format a debug log line printed right after
building a `max_edge_length` collision proxy:

```cpp
logger().debug(fmt::format(
    std::locale("en_US.UTF-8"),
    "Done (took {:g}s, {:L} vertices, {:L} triangles)",
    timer.getElapsedTime(),
    collision_vertices.rows(), collision_triangles.rows()));
```

`fmt::format(...)` is a plain function-call argument to `logger().debug()`,
so it is evaluated unconditionally before the call, regardless of whether
debug-level logging is enabled. On a machine without `en_US.UTF-8`
generated, `std::locale`'s constructor throws `std::runtime_error`
(`locale::facet::_S_create_c_locale name not valid`) — every scene that
builds a `max_edge_length` collision proxy fails before solving.

This cloud container does not have `en_US.UTF-8` generated
(`locale -a` lists only `C`, `C.utf8`, `POSIX`) — matching the reported
reproduction environment; the locale was **not** generated to fix this, per
the task's instruction.

**Reproduced** on an isolated build of the pre-fix sources (same commit,
`src/polyfem/legacy/State.cpp` and
`src/polyfem/varforms/NonlinearElasticVarForm.cpp` reverted to their
pre-fix content, `PolyFEM_bin` rebuilt, then the source restored before
building the fix): a copy of
`data/contact/examples/3D/higher-order/ball-bounce/P1.json` (`tend` reduced
from 1 to 0.003, i.e. 3 of the original 1000 steps, `tests` block removed —
this run is a reproduction, not a golden check) at
`outputs/locale-fix/20260928T205618Z/repro-pre-fix/`:

```
[polyfem] [debug] Building collision proxy with max edge length=0.01 ...
[polyfem] [critical] PolyFEM stopped: locale::facet::_S_create_c_locale name not valid
[polyfem] [critical] Exit status 1 (named failure; not a crash).
```

Exit status 1, matching the finding. Full log:
`outputs/locale-fix/20260928T205618Z/repro-pre-fix/repro.log`.

## Fix

Format the counts with plain `{}` instead of locale-dependent `{:L}`, and
let `logger().debug(fmt, args...)` decide whether to format at all (the
pattern already used by the neighboring line, `"Building collision proxy
with max edge length={} ..."`), instead of eagerly pre-formatting with
`fmt::format`:

```cpp
logger().debug(
    "Done (took {:g}s, {} vertices, {} triangles)",
    timer.getElapsedTime(),
    collision_vertices.rows(), collision_triangles.rows());
```

No other behavior changes — the log text loses thousands separators on the
vertex/triangle counts, nothing else. `grep -rn 'std::locale(|\{:L\}|setlocale' src/`
found no other occurrence in `src/` (dependency sources under
`build-cloud/_deps/` were not searched or edited, per the task's scope).

Same run on the fixed binary (`outputs/locale-fix/20260928T205618Z/repro-pre-fix/repro-fixed.log`):

```
[polyfem] [debug] Building collision proxy with max edge length=0.01 ...
[polyfem] [debug] Done (took 0.005287s, 2601 vertices, 5198 triangles)
```

Exit status 0.

## Regression test

`tests/test_collision_proxy.cpp`, `[build_collision_proxy]`: "build
collision proxy via max_edge_length does not require a locale" — drives
`NonlinearElasticVarForm::build_collision_mesh`'s `max_edge_length` branch
directly (the exact call site of the bug) with `logger().set_level(spdlog::level::debug)`,
and asserts `CHECK_NOTHROW`.

| Build | Result |
| --- | --- |
| Pre-fix `unit_tests` (isolated rebuild, sources reverted then restored, binary saved before restoring) | **fails**: `CHECK_NOTHROW(...)` — "due to unexpected exception with message: locale::facet::_S_create_c_locale name not valid"; 1 of 5 assertions in the case fail |
| Fixed `unit_tests` (this branch) | passes |

## Verification

Host: Ubuntu 24.04.4, GCC 13.3.0, CMake 3.28.3, Ninja, 4 vCPU, `en_US.UTF-8`
not generated (`locale -a`: `C`, `C.utf8`, `POSIX`). Build:
`tools/cloud/compile.sh` (Release, `POLYFEM_PORTABLE_BUILD=ON`,
`POLYFEM_WITH_TRIANGLE=ON`, `POLYFEM_WITH_MISO=OFF`); data pin
`8e8861309d6318c0dcfdc20f78fb2f38cffb7a16` (see the CI-05 section below).

| Check | Criterion | Result | Status |
| --- | --- | --- | --- |
| `[build_collision_proxy],[upsample_mesh]`, single-threaded | pass | 2192 assertions / 5 test cases, all pass | pass |
| Five `scenes/semi-implicit` smokes, `tools/smoke/run_smoke.py`, single-threaded | exit 0, 0 error lines | `quasistatic-adaptive`, `quasistatic-semi`, `quasistatic-semi-alhess`, `quasistatic-semi-friction`, `transient-semi`: all exit 0, 0 error lines (`outputs/locale-fix/20260928T205618Z/smoke/`) | pass |

Not performed: the whole unit suite (usage-limited; read excerpts per the
task's instruction); private scenes; Teseo; other platforms. The smoke
scenes were not compared VTU-by-VTU against a pre-fix baseline beyond exit
status and error-line count, since this change touches only a debug log
line on a code path (`max_edge_length` collision proxies) that none of the
five `scenes/semi-implicit` fixtures exercise — their `0 error line(s)` and
unchanged worst-min-distance readings are the applicable check.

## Publication

PolyFEM commits on `cloud/locale-fix`:
- `54fa859` — CI-05 data pin (see below).
- `89f9bad` — the locale fix and regression test.

Evidence: `outputs/locale-fix/20260928T205618Z/` (`repro-pre-fix/`
reproduction logs and the isolated copied scene, `smoke/` run). Not
committed (per CLAUDE.md: evidence isolated per task, not committed unless
a record needs it).
