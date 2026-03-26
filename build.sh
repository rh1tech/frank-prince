#!/usr/bin/env bash
set -euo pipefail

# Build frank-prince using CMake + Make.
# Usage:
#   ./build.sh                      # Build with defaults (M1, 378 MHz)
#   ./build.sh clean                # Clean rebuild
#   ./build.sh --board M2           # Build for M2 board
#   ./build.sh --cpu 504            # Build with 504 MHz CPU
#   ./build.sh --board M2 --cpu 504 # Combine options
#
# Options:
#   --board M1|M2        Board variant (default: M1)
#   --cpu 252|378|504    CPU speed in MHz (default: 378)
#   --psram 100|133|166  PSRAM speed in MHz (auto-matched to CPU by default)
#   clean                Delete build directory and rebuild

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT_DIR/build-make"

# Defaults
BOARD_VAR="${BOARD_VARIANT:-M1}"
CPU_VAR="${CPU_SPEED:-378}"
PSRAM_VAR="${PSRAM_SPEED:-}"
DO_CLEAN=0

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        clean)
            DO_CLEAN=1
            shift
            ;;
        --board|-b)
            BOARD_VAR="$2"
            shift 2
            ;;
        --cpu|-c)
            CPU_VAR="$2"
            shift 2
            ;;
        --psram|-p)
            PSRAM_VAR="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [options] [clean]"
            echo ""
            echo "Options:"
            echo "  --board M1|M2        Board variant (default: M1)"
            echo "  --cpu 252|378|504    CPU speed in MHz (default: 378)"
            echo "  --psram 100|133|166  PSRAM speed in MHz (auto-selected if not set)"
            echo "  clean                Delete build directory and rebuild"
            echo ""
            echo "Speed presets:"
            echo "  252 MHz CPU → 100 MHz PSRAM (no overclock)"
            echo "  378 MHz CPU → 133 MHz PSRAM (medium overclock)"
            echo "  504 MHz CPU → 166 MHz PSRAM (max overclock)"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Auto-select PSRAM speed based on CPU speed if not specified
if [[ -z "$PSRAM_VAR" ]]; then
    case "$CPU_VAR" in
        252) PSRAM_VAR=100 ;;
        378) PSRAM_VAR=133 ;;
        504) PSRAM_VAR=166 ;;
        *)   PSRAM_VAR=133 ;;
    esac
fi

# Clean if requested
if [[ $DO_CLEAN -eq 1 ]]; then
    rm -rf "$BUILD_DIR"
fi

cmake_args=(
    "-DBOARD_VARIANT=${BOARD_VAR}"
    "-DCPU_SPEED=${CPU_VAR}"
    "-DPSRAM_SPEED=${PSRAM_VAR}"
)

# Diagnostics toggles (legacy env var support)
if [[ -n "${RP2350_FORCE_TEST_PATTERN:-}" ]]; then
  cmake_args+=("-DRP2350_FORCE_TEST_PATTERN=${RP2350_FORCE_TEST_PATTERN}")
fi
if [[ -n "${RP2350_DUMP_FIRST_FRAME_BYTES:-}" ]]; then
  cmake_args+=("-DRP2350_DUMP_FIRST_FRAME_BYTES=${RP2350_DUMP_FIRST_FRAME_BYTES}")
fi
if [[ -n "${RP2350_DEBUG_INDEX_BAR:-}" ]]; then
  cmake_args+=("-DRP2350_DEBUG_INDEX_BAR=${RP2350_DEBUG_INDEX_BAR}")
fi
if [[ -n "${RP2350_BOOT_TEST_PATTERN:-}" ]]; then
  cmake_args+=("-DRP2350_BOOT_TEST_PATTERN=${RP2350_BOOT_TEST_PATTERN}")
fi
if [[ -n "${RP2350_BOOT_TEST_PATTERN_HALT:-}" ]]; then
  cmake_args+=("-DRP2350_BOOT_TEST_PATTERN_HALT=${RP2350_BOOT_TEST_PATTERN_HALT}")
fi
if [[ -n "${RP2350_BOOT_TEST_PATTERN_MODE:-}" ]]; then
  cmake_args+=("-DRP2350_BOOT_TEST_PATTERN_MODE=${RP2350_BOOT_TEST_PATTERN_MODE}")
fi

echo "Building: Board=${BOARD_VAR}, CPU=${CPU_VAR} MHz, PSRAM=${PSRAM_VAR} MHz"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G "Unix Makefiles" "${cmake_args[@]}"
cmake --build "$BUILD_DIR" -- -j

echo "Built: $BUILD_DIR/frank-prince.elf"
