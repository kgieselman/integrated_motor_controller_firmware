#!/usr/bin/env bash
# =============================================================================
# scripts/flash.sh — Flash firmware to the board via OpenOCD + ST-Link
#
# !! Run this on the HOST, not inside the container !!
# The container does not have USB/ST-Link access by default. If you want to
# flash from inside the container you'd need --privileged + --device flags,
# which is generally not worth the complexity for a dev workflow.
#
# Prerequisites (host):
#   openocd 0.12.0+     (sudo apt install openocd  /  brew install openocd)
#   ST-Link drivers     (Linux: see README udev rules  /  Windows: ST-LINK driver)
#
# Usage:
#   scripts/flash.sh [options]
#
# Options:
#   -t, --target <name>     Firmware target to flash (default: imc_bringup)
#                           Valid targets: imc_bringup, imc_tactical
#   -b, --build-dir <dir>   Build output directory (default: build)
#   -v, --verify            Verify flash contents after programming (default: on)
#   -r, --reset             Reset the MCU after flashing (default: on)
#   -h, --help              Print this message
#
# Examples:
#   scripts/flash.sh
#   scripts/flash.sh --target imc_tactical
#   scripts/flash.sh --target imc_bringup --build-dir build
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
TARGET="imc_bringup"
BUILD_DIR="build"
VERIFY="verify"
RESET="reset"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -t|--target)      TARGET="$2";    shift 2 ;;
        -b|--build-dir)   BUILD_DIR="$2"; shift 2 ;;
        --no-verify)      VERIFY="";      shift   ;;
        --no-reset)       RESET="";       shift   ;;
        -h|--help)
            sed -n '3,35p' "$0"
            exit 0
            ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

cd "$REPO_ROOT"

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
# Check that openocd is available on the host
# ---------------------------------------------------------------------------
if ! command -v openocd &>/dev/null; then
    echo "Error: openocd not found on PATH." >&2
    echo "       Install it on your host machine:" >&2
    echo "         Linux:  sudo apt install openocd" >&2
    echo "         macOS:  brew install openocd" >&2
    echo "         Windows: https://openocd.org/pages/getting-openocd.html" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Flash
# ---------------------------------------------------------------------------
# Build the OpenOCD -c command string
OCD_CMDS="program ${ELF}"
[[ -n "$VERIFY" ]] && OCD_CMDS+=" verify"
[[ -n "$RESET"  ]] && OCD_CMDS+=" reset"
OCD_CMDS+=" exit"

echo ">>> Flashing ${ELF} to STM32H563 via ST-Link"
echo "    openocd -f interface/stlink.cfg -f target/stm32h5x.cfg -c \"${OCD_CMDS}\""
echo ""

openocd \
    -f interface/stlink.cfg \
    -f target/stm32h5x.cfg \
    -c "$OCD_CMDS"

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
