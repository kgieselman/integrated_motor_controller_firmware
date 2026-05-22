#!/usr/bin/env bash
# =============================================================================
# scripts/flash.sh — Flash firmware to the board via OpenOCD or STM32CubeProgrammer
#
# !! Run this on the HOST, not inside the container !!
# The container does not have USB/ST-Link access by default. If you want to
# flash from inside the container you'd need --privileged + --device flags,
# which is generally not worth the complexity for a dev workflow.
#
# Tool selection (--programmer flag or auto-detection):
#   openocd     — uses OpenOCD + stm32h5x.cfg (requires OpenOCD >= 0.12.0 with H5 support)
#   cubeprog    — uses STM32_Programmer_CLI (always supports ST MCUs; installed with
#                 STM32CubeIDE or STM32CubeProgrammer)
#   auto        — tries OpenOCD first; falls back to cubeprog if stm32h5x.cfg is missing
#
# Prerequisites (host):
#   OpenOCD path:    openocd must be on PATH (sudo apt install openocd / brew install openocd /
#                    xPack: https://github.com/xpack-binaries/openocd/releases)
#   CubeProg path:   STM32_Programmer_CLI on PATH, or installed at the default Windows location:
#                    C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin
#
# Usage:
#   scripts/flash.sh [options]
#
# Options:
#   -t, --target <name>          Firmware target to flash (default: imc_bringup)
#                                Valid targets: imc_bringup, imc_tactical
#   -b, --build-dir <dir>        Build output directory (default: build)
#   -p, --programmer <tool>      Force programmer: openocd | cubeprog | auto (default: auto)
#   --no-verify                  Skip flash verification
#   --no-reset                   Do not reset the MCU after flashing
#   -h, --help                   Print this message
#
# Examples:
#   scripts/flash.sh
#   scripts/flash.sh --target imc_tactical
#   scripts/flash.sh --programmer cubeprog
#   scripts/flash.sh --programmer openocd --no-verify
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
TARGET="imc_bringup"
BUILD_DIR="build"
VERIFY="1"
RESET="1"
PROGRAMMER="auto"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -t|--target)      TARGET="$2";     shift 2 ;;
        -b|--build-dir)   BUILD_DIR="$2";  shift 2 ;;
        -p|--programmer)  PROGRAMMER="$2"; shift 2 ;;
        --no-verify)      VERIFY="";       shift   ;;
        --no-reset)       RESET="";        shift   ;;
        -h|--help)
            sed -n '3,46p' "$0"
            exit 0
            ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

cd "$REPO_ROOT"

# ---------------------------------------------------------------------------
# Validate programmer selection
# ---------------------------------------------------------------------------
case "$PROGRAMMER" in
    auto|openocd|cubeprog) ;;
    *) echo "Error: unknown --programmer value '${PROGRAMMER}' (openocd | cubeprog | auto)" >&2; exit 1 ;;
esac

# ---------------------------------------------------------------------------
# Resolve the ELF path
# ---------------------------------------------------------------------------
ELF="${BUILD_DIR}/${TARGET}.elf"

if [[ ! -f "$ELF" ]]; then
    echo "Error: ELF not found at ${ELF}" >&2
    echo "       Run 'make build' (or 'make build-${TARGET#imc_}') first." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Tool discovery helpers
# ---------------------------------------------------------------------------

# Returns 0 and prints the openocd path if openocd is usable (on PATH and
# stm32h5x.cfg is findable), non-zero otherwise.
find_openocd() {
    command -v openocd &>/dev/null || return 1
    # Probe for the H5 target config without actually opening a debug session.
    openocd -f interface/stlink.cfg -f target/stm32h5x.cfg -c exit 2>/dev/null || return 1
    command -v openocd
}

# Returns 0 and prints the STM32_Programmer_CLI path if the tool is found,
# non-zero otherwise.  Searches PATH first, then the default Windows install
# directory (works transparently under Git Bash / MSYS2).
find_cubeprog() {
    if command -v STM32_Programmer_CLI &>/dev/null; then
        command -v STM32_Programmer_CLI
        return 0
    fi

    # Default Windows install path (Git Bash translates /c/... to C:\...)
    local win_default="/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
    if [[ -x "$win_default" ]]; then
        echo "$win_default"
        return 0
    fi
    # Also try without .exe extension omitted (Git Bash resolves it)
    if [[ -f "${win_default}.exe" ]]; then
        echo "${win_default}.exe"
        return 0
    fi

    return 1
}

# ---------------------------------------------------------------------------
# Select programmer
# ---------------------------------------------------------------------------
TOOL=""
TOOL_NAME=""

if [[ "$PROGRAMMER" == "openocd" || "$PROGRAMMER" == "auto" ]]; then
    if TOOL=$(find_openocd 2>/dev/null); then
        TOOL_NAME="openocd"
    elif [[ "$PROGRAMMER" == "openocd" ]]; then
        echo "Error: openocd not found on PATH, or stm32h5x.cfg is missing from your OpenOCD install." >&2
        echo "       OpenOCD >= 0.12.0 with H5 support is required." >&2
        echo "       xPack builds: https://github.com/xpack-binaries/openocd/releases" >&2
        exit 1
    fi
fi

if [[ -z "$TOOL_NAME" && ( "$PROGRAMMER" == "cubeprog" || "$PROGRAMMER" == "auto" ) ]]; then
    if TOOL=$(find_cubeprog 2>/dev/null); then
        TOOL_NAME="cubeprog"
    elif [[ "$PROGRAMMER" == "cubeprog" ]]; then
        echo "Error: STM32_Programmer_CLI not found." >&2
        echo "       Install STM32CubeProgrammer: https://www.st.com/en/development-tools/stm32cubeprog.html" >&2
        exit 1
    fi
fi

if [[ -z "$TOOL_NAME" ]]; then
    echo "Error: no suitable flash tool found." >&2
    echo "" >&2
    echo "  Option A — OpenOCD with H5 support (>= 0.12.0 + stm32h5x.cfg):" >&2
    echo "    Linux:  sudo apt install openocd" >&2
    echo "    macOS:  brew install openocd" >&2
    echo "    xPack:  https://github.com/xpack-binaries/openocd/releases" >&2
    echo "" >&2
    echo "  Option B — STM32CubeProgrammer (always supports ST MCUs):" >&2
    echo "    https://www.st.com/en/development-tools/stm32cubeprog.html" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Flash
# ---------------------------------------------------------------------------
echo ">>> Flashing ${ELF} to STM32H563 via ST-Link (${TOOL_NAME})"

if [[ "$TOOL_NAME" == "openocd" ]]; then
    OCD_CMDS="program ${ELF}"
    [[ -n "$VERIFY" ]] && OCD_CMDS+=" verify"
    [[ -n "$RESET"  ]] && OCD_CMDS+=" reset"
    OCD_CMDS+=" exit"

    echo "    openocd -f interface/stlink.cfg -f target/stm32h5x.cfg -c \"${OCD_CMDS}\""
    echo ""
    openocd \
        -f interface/stlink.cfg \
        -f target/stm32h5x.cfg \
        -c "$OCD_CMDS"

else  # cubeprog
    CP_ARGS=(-c port=SWD -w "$ELF")
    [[ -n "$VERIFY" ]] && CP_ARGS+=(-v)
    [[ -n "$RESET"  ]] && CP_ARGS+=(-rst)

    echo "    STM32_Programmer_CLI ${CP_ARGS[*]}"
    echo ""
    "$TOOL" "${CP_ARGS[@]}"
fi

echo ""
echo ">>> Flash complete."

# ---------------------------------------------------------------------------
# Helpful hint for the bringup console
# ---------------------------------------------------------------------------
if [[ "$TARGET" == "imc_bringup" ]]; then
    echo ""
    echo ">>> Connect to the USB-CDC console:"
    echo "      Linux:   screen /dev/ttyACM0 115200"
    echo "      macOS:   screen /dev/tty.usbmodem* 115200"
    echo "      Windows: PuTTY on the enumerated COM port, 115200 baud"
fi
