# CI-08 — Installable Linux x86_64 CLI package

Date: 2026-09-28. Status: **Linux x86_64 target done and validated within the
scope below** (cloud session, branch `cloud/ci-08-linux`). macOS and Windows
targets remain open (see the [CI portability
plan](ci-portability-plan.md#ci-08--installable-solver-packages)). This
record also covers the CI-09 portable smoke runner
(`tools/smoke/run_smoke.py`, [README](../tools/smoke/README.md)), built and
verified in the same session.

## Contract and authorization

- Scheduled task: "the Linux x86_64 part of CI-08 (installable solver
  package) and the portable smoke runner from CI-09", branch
  `cloud/ci-08-linux`. In scope: install rules + CPack packaging for
  `PolyFEM_bin`, without changing how developers build; a stdlib-only
  Python smoke runner; measuring and documenting (not deciding) the glibc/CPU
  baseline and redistribution notices; the oldest-supported-distribution
  choice is left to the user.
- Not authorized and not done: changing solver defaults, model behavior, test
  tolerances or golden values; enabling or editing GitHub workflow publishing
  steps; macOS/Windows packaging.
- Read: CLAUDE.md, `docs/ci-portability-plan.md` (CI-07 items 2/3/6, CI-08,
  CI-09), `scenes/semi-implicit/README.md`, `app/CMakeLists.txt` (the
  existing GUI install/CPack pattern this follows), `tools/rb12/cli_check.py`
  (the existing CLI exit-status contract), `tools/rb12/repeat.py` (style
  reference for the smoke runner).

## Host and build

Cloud container: `nproc` 4, Ubuntu 24.04, GCC 13.3.0, CMake 3.28.3, Ninja,
glibc 2.39 (`ldd --version`). `ccache` had to be installed (missing from the
image; `tools/cloud/setup-environment.sh` installs it but wasn't re-run since
the toolchain was otherwise already present).

Build: `tools/cloud/compile.sh PolyFEM_bin` (Release,
`POLYFEM_PORTABLE_BUILD=ON`, `POLYFEM_WITH_TRIANGLE=ON`, all other options at
their defaults: `POLYFEM_WITH_PYTHON=ON`, `POLYFEM_WITH_MMG=ON`,
`POLYFEM_WITH_OPTIMIZATION=ON`, `POLYFEM_THREADING=TBB`,
`POLYFEM_WITH_MISO=OFF`). `PolyFEM_bin --build_info`: `polyfem` at
`afaa3848407e` (the first branch commit that changed the build; later commits
on this branch are docs/CMake-only besides this one), `ipc_toolkit`
`cf99893be74f`, `polysolve` `43ca2e661069` — all matching their declared pins.

## What changed

- `cmake/polyfem/polyfem_cli_package.cmake` (new): install rules for
  `PolyFEM_bin` plus a CPack TGZ (Linux/macOS) / ZIP (Windows) configuration,
  included from `CMakeLists.txt` right after the `PolyFEM_bin` target is
  defined, guarded by `POLYFEM_TOPLEVEL_PROJECT` (same guard as the target
  itself) and skipped when `POLYFEM_WITH_APP` is also on (the GUI already
  configures its own `include(CPack)` in `app/CMakeLists.txt`; packaging both
  components in one configure is out of scope for this item — build them
  separately). It mirrors `app/CMakeLists.txt`'s existing Unix pattern
  exactly: `install(TARGETS ... RUNTIME_DEPENDENCY_SET ...)` plus
  `install(RUNTIME_DEPENDENCY_SET ...)` excluding base-system directories
  (`^/lib/.*`, `^/usr/lib/.*`, `^/usr/lib64/.*`), `$ORIGIN/../lib` RPATH.
  A `BUILD_RPATH` is also forced (see below) so install-time relocation
  always has an RPATH section to rewrite.
- `tools/smoke/run_smoke.py` + `tools/smoke/README.md` (new): the CI-09
  portable smoke runner.
- `CLAUDE.md`: points at the smoke runner from the build-and-test section.
- `.gitignore`: `/outputs/` (this fork's per-task evidence convention was
  previously enforced only by discipline, not tooling).
- Nothing in the ordinary developer build changed: `cmake --build . --target
  PolyFEM_bin` produces the same target it always did; packaging only
  happens on explicit `cmake --install` / `cpack`, which nothing runs
  implicitly.

### The RPATH fix

The first `cmake --install --component cli` failed:

```
CMake Error ... file RPATH_CHANGE could not write new RPATH ... No valid ELF
RPATH or RUNPATH entry exists in the file
```

`PolyFEM_bin` links no third-party shared library outside the default system
search path in this configuration (TBB and MKL are both linked **statically**
here — see below), so CMake's automatic RPATH computation embeds no
RPATH/RUNPATH section at build time at all, and `file(RPATH_CHANGE)` at
install time has nothing to rewrite. Setting `BUILD_RPATH` to `$ORIGIN`
(a harmless no-op today) forces a section to exist, so relocation still works
if a future configuration — a different platform, `POLYFEM_THREADING`, or a
shared TBB — does bundle a runtime library into `lib/`.

## Verification

### Build, install, package

```
tools/cloud/compile.sh PolyFEM_bin
cmake --install build-cloud --prefix build-cloud/install-test --component cli
cd build-cloud && cpack
```

`cmake --install` places `bin/PolyFEM_bin`, `README.txt`, `LICENSE` (and this
document, renamed `PACKAGE-NOTES.md`, once it exists) under the prefix; no
files land under `lib/` in this configuration (nothing needed bundling — see
below). `cpack` produces `PolyFEM-CLI-Linux-x86_64.tar.gz`, **49 MB**
compressed (the installed `PolyFEM_bin` itself is 142 MB, mostly static
MKL/SuiteSparse/HYPRE/geogram/HDF5 object code).

### Relocation: space path, scrubbed environment, renamed source/build trees

The package was extracted to `/tmp/ci08 pkg extract/dest dir/` (a path with a
space), and then, with `/home/user/polyfem` (source **and** build tree, since
`build-cloud` lives under it) temporarily renamed so no absolute path to
either could resolve, run from a plain `/tmp` working directory with
`env -i PATH=/usr/bin:/bin`:

```
$ env -i PATH=/usr/bin:/bin "<extracted>/bin/PolyFEM_bin" --build_info
{...}   # exit 0, full build identity printed
```

The `tools/rb12/cli_check.py` contract (this fork's existing end-to-end exit-
status check) ran against the extracted binary the same way, with its own
copy of `scenes/semi-implicit/quasistatic-semi.json` staged outside the
renamed tree:

```
build identity: {'ipc_toolkit': 'cf99893be74f', 'polyfem': 'afaa3848407e', 'polysolve': '43ca2e661069'} GNU 13.3.0 Release
smoke: 4 steps, manifest completed in 3.38s, peak 158 MB
resource failure: exit 3, manifest resource_failure
refused input: exit 1, no output
output/manifest "": no manifest, exit 0
cli_contract: 5 checks passed
```

All three documented exit statuses are confirmed on the relocated,
environment-scrubbed binary: **0** (a completed public smoke, with a
`run-manifest.json` naming this exact binary by SHA-256), **1** (an input
refused at init — `contact.dhat` non-positive — with no output and no
manifest), **3** (a resource limit reached before allocation, with a
`resource_failure` manifest). The source/build tree was restored immediately
after (`mv` back).

The CI-09 smoke runner ran against the same relocated binary and a copy of
`scenes/semi-implicit` staged outside the renamed tree, also under
`env -i PATH=/usr/bin:/bin`:

```
scene                          exit  errors                worst min distances  wall (s)
quasistatic-semi                  0       0    1.001e-04, 1.002e-04, 2.026e-04      9.06
quasistatic-semi-friction         0       0    1.001e-04, 1.002e-04, 2.026e-04      9.47
quasistatic-semi-alhess            0       0    1.001e-04, 1.002e-04, 2.026e-04      9.00
transient-semi                    0       0    1.001e-04, 1.001e-04, 2.024e-04      9.30
quasistatic-adaptive               0       0    4.392e-08, 4.663e-08, 4.683e-08     12.49
```

(the same run, unrelocated, against `build-cloud/PolyFEM_bin` directly, gives
identical results — see `tools/smoke/README.md`). The runner itself was also
exercised once with both `--binary` and `--output` containing spaces and
non-ASCII characters (`/tmp/pöly fem tëst/PolyFEM bin (ünïcödé).bin`,
`.../out püt dïr`), all five scenes exit 0 with 0 error lines, and once
confirmed to refuse a non-empty `--output` directory as specified.

### Python-expression capability under relocation

`POLYFEM_WITH_PYTHON=ON` embeds the build's Python **program path** into the
binary (`ExpressionValue.cpp`'s `Py_SetProgramName` call, compiled in as
`POLYFEM_PYTHON_EXECUTABLE`); here that path is `/usr/local/bin/python3` (a
system symlink to `/usr/bin/python3.11`). A `dirichlet_boundary` value
routed through `{"file_name": "expr.py", "function_name": "dz"}` (the
`init_python` path in `ExpressionValue.cpp`) was run with the source/build
tree renamed away and `env -i PATH=/usr/bin:/bin` — it completed
successfully (exit 0), because the exact `/usr/local/bin/python3` path,
`libpython3.11.so.1.0`, and the full Python 3.11 standard library are all
**system-installed** on this host and unaffected by scrubbing `PATH` or
hiding the source tree.

This confirms the capability survives relocation **on the machine it was
built on**, but does not establish portability to a different machine: see
the "Python expressions" note under Redistribution below.

### glibc baseline, CPU baseline, package contents

```
$ ldd build-cloud/PolyFEM_bin
        libgomp.so.1 => /lib/x86_64-linux-gnu/libgomp.so.1
        libpython3.11.so.1.0 => /lib/x86_64-linux-gnu/libpython3.11.so.1.0
        libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6
        libstdc++.so.6 => /lib/x86_64-linux-gnu/libstdc++.so.6
        libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1
        libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6
        libz.so.1 => /lib/x86_64-linux-gnu/libz.so.1
        libexpat.so.1 => /lib/x86_64-linux-gnu/libexpat.so.1
```

Every dynamic dependency is a base-system library; **nothing non-system is
linked dynamically** in this configuration (TBB — `TBB_PREFER_STATIC=ON` in
`cmake/recipes/onetbb.cmake` — and MKL are both statically linked; confirmed
in the link line: `gnu_13.3_cxx17_64_release/libtbb.a`,
`.../mkl-static/.../libmkl_{core,intel_lp64,tbb_thread}.a`). This is why
`cmake --install`'s `RUNTIME_DEPENDENCY_SET` step copies nothing into `lib/`
— the package is `bin/PolyFEM_bin` plus `README.txt`/`LICENSE` only, and the
`$ORIGIN/../lib` RPATH above is currently unused but harmless.

```
$ objdump -T build-cloud/PolyFEM_bin | grep -oE 'GLIBC_[0-9.]+' | sort -u -V | tail -3
GLIBC_2.34
GLIBC_2.38
$ ldd --version | head -1
ldd (Ubuntu GLIBC 2.39-0ubuntu8.7) 2.39
```

**GLIBC_2.38 is the binding requirement.** It comes from `__isoc23_fscanf`,
`__isoc23_sscanf`, `__isoc23_strtol`/`strtoll`/`strtoull`, and a versioned
`fmod` — the ISO-C23 `scanf`/`strtol` symbol variants that glibc ≥ 2.38
introduces and that GCC 13 on a ≥ 2.38 host requests automatically (this is
a toolchain/host effect, not a PolyFEM source choice; nothing in this
codebase calls these directly). **Measured recommendation:** with this build
host (Ubuntu 24.04, glibc 2.39), the oldest distributions that satisfy
glibc ≥ 2.38 are Ubuntu 24.04 LTS itself, Debian 13 ("trixie"), and Fedora
39+ — **not** Ubuntu 22.04 (glibc 2.35) or Debian 12 (glibc 2.36). Choosing a
wider baseline (e.g. Ubuntu 22.04) is possible but requires building on a
host with an older glibc (or an old-sysroot/manylinux-style toolchain); that
rebuild was not done here and the choice of oldest supported distribution is
the user's decision, per CLAUDE.md.

CPU baseline: `POLYFEM_PORTABLE_BUILD=ON` sets `-msse4.2` for dependencies
that support it (`CMakeLists.txt`, portable-build block) and disables AVX
autodetection; this was not independently re-verified against a non-AVX host
in this session, only inherited from the existing cloud build convention.

## Redistribution notices

**Everything below is measured (what's actually linked, per the link line
`build-cloud/CMakeFiles/PolyFEM_bin.dir/link.txt`, and the LICENSE files CPM
fetched into `/root/.cache/CPM/*/`), not a legal opinion.** A license/policy
decision on any of this remains the user's.

| Component | License | Linkage | Note |
| --- | --- | --- | --- |
| PolyFEM | MIT | — | this project |
| polysolve, IPC Toolkit | MIT (`LICENSE` in each CPM checkout) | static | this fork's pinned forks |
| Eigen | MPL-2.0 (with BSD/LGPL-licensed subcomponents PolyFEM doesn't use) | static (header-only) | |
| **UMFPACK, SPQR** | **GPL-2.0-or-later** | **static** | `SuiteSparse/{UMFPACK,SPQR}/Doc/License.txt`; commercial licenses available from the author. **Statically linking GPL-licensed code into a single distributed executable is a real redistribution constraint** (source-availability / GPL terms for the combined binary), not merely an attribution notice — flagged for the user's decision, not resolved here. |
| CHOLMOD | Mixed per module (LGPL-2.1+ for `Check`/`Core`-type modules; other modules GPL-2.0+) | static | see `SuiteSparse/CHOLMOD/Doc/License.txt` for the exact module breakdown; not independently itemized per module here |
| AMD, CAMD, COLAMD, CCOLAMD, SuiteSparseConfig | BSD-3-Clause | static | |
| **Intel MKL** (`libmkl_core`, `libmkl_intel_lp64`, `libmkl_tbb_thread`) | **Intel Simplified Software License** (Aug 2021) | static | permits redistribution "without modification" provided the notice is reproduced; include `info/licenses/mkl-static/info/licenses/license.txt` (fetched by CPM) with any distributed package |
| TBB (oneTBB) | Apache-2.0 | static | `TBB_PREFER_STATIC=ON` |
| HYPRE | Apache-2.0 / MIT (dual) | static | |
| Python 3.11 (`libpython3.11.so.1.0`) | PSF License | dynamic, **system-provided, not bundled** | see the Python-expressions note below |
| HDF5, geogram, spdlog, nlohmann_json, yaml-cpp, tinyxml2, CLI11, Catch2 (tests only), libigl, mshio, glob, natsort, units, tinyexpr, MeshFEMSparse, scalable-ccd, tight-inclusion, TinyAD, finite-diff, abseil-cpp, filib, Clipper, MMG, Boost (header use) | each permissive (BSD/MIT/Apache/zlib; MMG is LGPL-3.0, statically linked) | static | see each package's `LICENSE*` under `/root/.cache/CPM/<name>/*/` for exact text |
| glibc, libstdc++, libgcc_s, libgomp, libm, libz, libexpat | GPL/LGPL with the GCC runtime exception / permissive | dynamic, system-provided | assumed present on the target distribution, not redistributed |

**Python expressions.** `POLYFEM_WITH_PYTHON=ON` is the only capability this
package's redistribution can't fully self-certify: the interpreter is
dynamically linked against the **build host's** `libpython3.11.so.1.0` (a
system library, excluded from bundling like glibc/libstdc++), and the
compiled-in program path (`/usr/local/bin/python3` here) is used to locate
the standard library at process start. A target machine that lacks a
matching Python 3.11 installation at that path cannot use Python-expression
scenes (`{"file_name": ..., "function_name": ...}` values); everything else
(the FEM/contact solver itself) is unaffected, since nothing else in the
link line depends on Python. This was verified to survive relocation **on
the build host** (see above) but not across machines, which this sandbox
cannot provide. Recommendation for a wider release: either document the
exact required Python version/path as a package prerequisite, or reconfigure
with an explicit `-DPython3_EXECUTABLE=/usr/bin/python3` (or similar
distro-standard path) before packaging so the baked-in path matches a
documented, stable location; a Python-free package
(`-DPOLYFEM_WITH_PYTHON=OFF`) removes the dependency and the capability
entirely. Neither change was made here (a build-option default change is the
user's decision).

## What's not done

- macOS and Windows packaging (CI-08's other two targets) — untouched.
- The GPL-licensed SuiteSparse components (UMFPACK, SPQR, parts of CHOLMOD)
  and the Intel MKL redistribution notice need an explicit decision from the
  user before this package is distributed outside this fork's own testing;
  this record only measures and surfaces that.
- No independent non-AVX-host verification of the `-msse4.2` CPU baseline.
- No native (non-cloud-container) test of the oldest recommended distribution
  (Ubuntu 24.04 / Debian 13 / Fedora 39+); the glibc-symbol analysis is the
  evidence, not a boot of that exact distribution.
