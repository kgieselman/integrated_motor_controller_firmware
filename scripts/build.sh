#!/usr/bin/env bash
# =============================================================================
# scripts/build.sh — Configure and compile the firmware
#
# Intended to run INSIDE the dev container (or any host with the toolchain).
# The Makefile `make build` target calls this automatically via docker run.
#
# Usage:
#   scripts/build.sh [options]
#
# Options:
#   -t, --target <name>     CMake target to build (default: all)
#                           Valid targets: imc_bringup, imc_tactical, all
#   -T, --type <type>       CMake build type (default: Debug)
#                           Valid types: Debug, Release, MinSizeRel, RelWithDebInfo
#   -j, --jobs <n>          Parallel jobs (default: nproc)
#   -c, --clean             Wipe the build directory before building
#   -h, --help              Print this message
#
# Examples:
#   scripts/build.sh
#   scripts/build.sh --target imc_bringup
#   scripts/build.sh --target imc_tactical --type Release
#   scripts/build.sh --clean
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
TARGET="all"
BUILD_TYPE="Debug"
JOBS=$(nproc 2>/dev/null || sysctl -n hw.logicalcpu 2>/dev/null || echo 4)
CLEAN=false
BUILD_DIR="build"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -t|--target)   TARGET="$2";     shift 2 ;;
        -T|--type)     BUILD_TYPE="$2"; shift 2 ;;
        -j|--jobs)     JOBS="$2";       shift 2 ;;
        -c|--clean)    CLEAN=true;      shift   ;;
        -h|--help)
            sed -n '3,30p' "$0"  # print the header comment as help
            exit 0
            ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

cd "$REPO_ROOT"

# ---------------------------------------------------------------------------
# Clean
# ---------------------------------------------------------------------------
if [[ "$CLEAN" == true ]]; then
    echo ">>> Removing ${BUILD_DIR}/"
    rm -rf "$BUILD_DIR"
fi

# ---------------------------------------------------------------------------
# Configure (only if build.ninja is missing or CMakeLists.txt is newer)
# ---------------------------------------------------------------------------
if [[ ! -f "${BUILD_DIR}/build.ninja" ]] || \
   [[ CMakeLists.txt -nt "${BUILD_DIR}/build.ninja" ]]; then
    echo ">>> Configuring (${BUILD_TYPE})"
    cmake -S . -B "$BUILD_DIR" -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
if [[ "$TARGET" == "all" ]]; then
    echo ">>> Building all targets (${BUILD_TYPE}, -j${JOBS})"
    cmake --build "$BUILD_DIR" --parallel "$JOBS"
else
    echo ">>> Building ${TARGET} (${BUILD_TYPE}, -j${JOBS})"
    cmake --build "$BUILD_DIR" --target "$TARGET" --parallel "$JOBS"
fi

echo ""
echo ">>> Build complete. Artifacts:"
find "$BUILD_DIR" -maxdepth 1 -name "*.elf" | sort | while read -r f; do
    arm-none-eabi-size "$f" 2>/dev/null || ls -lh "$f"
    bin="${f%.elf}.bin"
    arm-none-eabi-objcopy -O binary "$f" "$bin"
    echo "    -> ${bin}"
done
