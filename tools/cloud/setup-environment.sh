#!/usr/bin/env bash
# Toolchain for a Claude Code cloud environment (Ubuntu). Paste this into the
# environment's setup script, or run it from the repository root. It installs
# packages only and does not depend on the checkout.
set -euo pipefail

SUDO=""
if [ "$(id -u)" -ne 0 ] && command -v sudo >/dev/null; then SUDO=sudo; fi

$SUDO apt-get update
$SUDO apt-get -o Acquire::Retries=3 install -y \
    build-essential ninja-build ccache git python3-dev python3-pip

# PolyFEM needs CMake >= 3.25 (REQUIRED_CMAKE_VERSION in CMakeLists.txt).
need=3.25
have=$(cmake --version 2>/dev/null | head -1 | awk '{print $3}' || true)
if [ -z "$have" ] || [ "$(printf '%s\n%s\n' "$need" "$have" | sort -V | head -1)" != "$need" ]; then
    python3 -m pip install --break-system-packages --upgrade cmake \
        || python3 -m pip install --upgrade cmake
fi
cmake --version | head -1

# Formatting: the CI pre-commit check pins clang-format 21.1.8. Install the
# pre-commit tool and its git hook so commits from this clone are formatted
# with that exact version (see CLAUDE.md, Working rules).
python3 -m pip install --break-system-packages --quiet pre-commit \
    || python3 -m pip install --quiet pre-commit
(cd "$(git rev-parse --show-toplevel 2>/dev/null || pwd)" && pre-commit install) || true
