#!/usr/bin/env bash
# Configure and build PolyFEM in a cloud session, matching the fork's Linux
# Release CI lane. Dependencies come from the SHA pins in cmake/recipes
# (sdast9/polysolve, sdast9/ipc-toolkit, sdast9/polyfem-data).
#   tools/cloud/compile.sh              # PolyFEM_bin + unit_tests
#   tools/cloud/compile.sh PolyFEM_bin  # one target
set -euo pipefail
cd "$(dirname "$0")/../.."

BUILD_DIR=${BUILD_DIR:-build-cloud}
if [ ! -f "$BUILD_DIR/build.ninja" ]; then
    # PORTABLE_BUILD: no -march=native in dependencies (cached objects from
    # another CPU SIGILL; see docs/ci-portability-plan.md). MISO stays at the
    # fork's OFF default.
    cmake -G Ninja -S . -B "$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DPOLYFEM_WITH_TRIANGLE=ON \
        -DPOLYFEM_PORTABLE_BUILD=ON
fi

if [ $# -eq 0 ]; then set -- PolyFEM_bin unit_tests; fi
cmake --build "$BUILD_DIR" --target "$@"
