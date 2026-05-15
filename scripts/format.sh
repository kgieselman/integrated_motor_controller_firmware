#!/usr/bin/env bash
# =============================================================================
# scripts/format.sh — Run clang-format on all hand-written source files
#
# Formats everything under app/ and drivers/. Skips cubemx/ (generated code).
# Reads style from .clang-format in the repo root; falls back to Google style.
#
# Intended to run INSIDE the dev container (or any host with clang-format).
# The Makefile `make format` and `make format-check` targets call this.
#
# Usage:
#   scripts/format.sh [options]
#
# Options:
#   --check     Dry-run mode: exit non-zero if any file would be reformatted.
#               Use this in CI to enforce formatting without modifying files.
#   -h, --help  Print this message
#
# Examples:
#   scripts/format.sh             # format in place
#   scripts/format.sh --check     # CI check — fails if anything is unformatted
# =============================================================================

set -euo pipefail

CHECK_ONLY=false
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --check)   CHECK_ONLY=true; shift ;;
        -h|--help)
            sed -n '3,30p' "$0"
            exit 0
            ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

cd "$REPO_ROOT"

# ---------------------------------------------------------------------------
# Collect source files (hand-written code only — skip cubemx/)
# ---------------------------------------------------------------------------
mapfile -t FILES < <(
    find app drivers \
        \( -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" \) \
        | sort
)

if [[ ${#FILES[@]} -eq 0 ]]; then
    echo "No source files found under app/ or drivers/."
    exit 0
fi

echo ">>> clang-format: ${#FILES[@]} files"

# ---------------------------------------------------------------------------
# Format or check
# ---------------------------------------------------------------------------
if [[ "$CHECK_ONLY" == true ]]; then
    UNFORMATTED=()
    for f in "${FILES[@]}"; do
        # --output-replacements-xml is empty when the file is already formatted
        RESULT=$(clang-format --dry-run --Werror "$f" 2>&1 || true)
        if [[ -n "$RESULT" ]]; then
            UNFORMATTED+=("$f")
            echo "  NEEDS FORMAT: $f"
        fi
    done

    if [[ ${#UNFORMATTED[@]} -gt 0 ]]; then
        echo ""
        echo ">>> ${#UNFORMATTED[@]} file(s) would be reformatted."
        echo "    Run 'make format' (or 'scripts/format.sh') to fix."
        exit 1
    else
        echo ">>> All files are correctly formatted."
    fi
else
    for f in "${FILES[@]}"; do
        clang-format -i "$f"
        echo "  formatted: $f"
    done
    echo ""
    echo ">>> Done."
fi
