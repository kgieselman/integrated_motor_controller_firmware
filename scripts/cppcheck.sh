#!/usr/bin/env bash
# =============================================================================
# scripts/cppcheck.sh — Run cppcheck static analysis on hand-written source
#
# Uses compile_commands.json so cppcheck automatically picks up all compiler
# defines (STM32H563xx, USE_HAL_DRIVER, etc.) and include paths. Only files
# under app/ and drivers/ are analysed; generated cubemx/ code is skipped.
#
# Intended to run INSIDE the dev container (cppcheck is installed there).
# The Makefile `make cppcheck` and `make cppcheck-ci` targets call this.
#
# Usage:
#   scripts/cppcheck.sh [options]
#
# Options:
#   --ci        Exit non-zero if any issues are found (for CI pipelines).
#               Without this flag the script always exits 0 so a developer
#               can review findings without breaking their local workflow.
#   -h, --help  Print this message
#
# Output:
#   Findings are printed to stdout and also written to build/cppcheck.txt
#   for review or archiving.
#
# Examples:
#   scripts/cppcheck.sh           # developer run — always exits 0
#   scripts/cppcheck.sh --ci      # CI run — exits non-zero on any finding
# =============================================================================

set -euo pipefail

CI_MODE=false
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"
COMPILE_COMMANDS="${BUILD_DIR}/compile_commands.json"
SUPPRESSIONS_FILE="${REPO_ROOT}/.cppcheck-suppressions"
REPORT_FILE="${BUILD_DIR}/cppcheck.txt"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ci)      CI_MODE=true; shift ;;
        -h|--help)
            sed -n '3,33p' "$0"
            exit 0
            ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

cd "$REPO_ROOT"

# ---------------------------------------------------------------------------
# Require compile_commands.json — run 'make build' or 'make configure' first
# ---------------------------------------------------------------------------
if [[ ! -f "$COMPILE_COMMANDS" ]]; then
    echo "ERROR: ${COMPILE_COMMANDS} not found." >&2
    echo "       Run 'make build' (or 'make configure') first to generate it." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Build the cppcheck command
# ---------------------------------------------------------------------------
CPPCHECK_ARGS=(
    # Use compile_commands.json for defines and include paths
    --project="$COMPILE_COMMANDS"

    # Only analyse our own code — skip cubemx/ vendor / generated files
    --file-filter='*/app/*'

    # ARM Cortex-M platform: 32-bit, 2-byte wchar_t
    --platform=arm32-wchar_t2

    # Enable checks beyond the default error-only set.
    # "warning"     — likely bugs (e.g. null-pointer dereference)
    # "style"       — readability / maintenance issues
    # "performance" — inefficient constructs
    # "portability" — constructs that may behave differently across platforms
    # "information" — informational messages (e.g. about configuration)
    # Excluded: "unusedFunction" — too noisy for embedded (IRQ/HAL callbacks
    #   are never called from user code but are referenced by the linker)
    --enable=warning,style,performance,portability,information

    # Suppress known false positives — see .cppcheck-suppressions for details
    "--suppressions-list=${SUPPRESSIONS_FILE}"

    # Human-readable output (file:line: [severity] message)
    --template='{file}:{line}: [{severity}] {message} ({id})'

    # Show progress on large codebases
    --showtime=summary

    # Use all available cores
    -j "$(nproc 2>/dev/null || echo 4)"
)

# In CI mode, turn any finding into a non-zero exit
if [[ "$CI_MODE" == true ]]; then
    CPPCHECK_ARGS+=(--error-exitcode=1)
fi

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
echo ">>> cppcheck (platform: arm32-wchar_t2, CI mode: ${CI_MODE})"
echo "    compile_commands: ${COMPILE_COMMANDS}"
echo "    report:           ${REPORT_FILE}"
echo ""

# Tee output to both terminal and report file
cppcheck "${CPPCHECK_ARGS[@]}" 2>&1 | tee "$REPORT_FILE"
EXIT_CODE=${PIPESTATUS[0]}

echo ""
if [[ $EXIT_CODE -eq 0 ]]; then
    echo ">>> cppcheck passed — no issues found."
else
    echo ">>> cppcheck found issues. See ${REPORT_FILE} for the full report."
    if [[ "$CI_MODE" == false ]]; then
        # Allow developer to review findings without blocking their workflow
        echo "    (Exiting 0 — run with --ci to enforce a non-zero exit code.)"
        EXIT_CODE=0
    fi
fi

exit $EXIT_CODE
