# =============================================================================
# IMC Firmware — Docker wrapper Makefile
#
# Container targets (run inside Docker):
#   make image           Build (or rebuild) the dev container image
#   make configure       Run CMake configure
#   make build           Configure (if needed) + compile all targets
#   make build-bringup   Build imc_bringup only
#   make build-tactical  Build imc_tactical only
#   make rebuild         Wipe build dir + full recompile
#   make shell           Interactive shell in the container
#   make format          Run clang-format on app/ and drivers/ (in-place)
#   make format-check    Check formatting without modifying files (for CI)
#   make clean           Remove the build/ directory
#
# Host targets (run on your machine — require openocd + ST-Link):
#   make flash           Flash imc_bringup via OpenOCD
#   make flash-bringup   Flash imc_bringup via OpenOCD
#   make flash-tactical  Flash imc_tactical via OpenOCD
#
# Pass extra CMake args via:  make build CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release"
# =============================================================================

IMAGE_NAME  := imc-firmware-dev
BUILD_DIR   := build
CMAKE_ARGS  ?=

# Mount the project root into the container as /workspace
DOCKER_RUN := docker run --rm \
    -v "$(CURDIR):/workspace" \
    -w /workspace \
    $(IMAGE_NAME)

# ---------------------------------------------------------------------------
# Image management
# ---------------------------------------------------------------------------
.PHONY: image
image:
	docker build -t $(IMAGE_NAME) .

# ---------------------------------------------------------------------------
# Configure — runs CMake to generate the build system
# ---------------------------------------------------------------------------
.PHONY: configure
configure:
	$(DOCKER_RUN) cmake -S . -B $(BUILD_DIR) -G Ninja \
	    -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
	    -DCMAKE_BUILD_TYPE=Debug \
	    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
	    $(CMAKE_ARGS)

# ---------------------------------------------------------------------------
# Build targets — all delegate to scripts/build.sh inside the container
# ---------------------------------------------------------------------------
.PHONY: build
build:
	$(DOCKER_RUN) scripts/build.sh

.PHONY: build-bringup
build-bringup:
	$(DOCKER_RUN) scripts/build.sh --target imc_bringup

.PHONY: build-tactical
build-tactical:
	$(DOCKER_RUN) scripts/build.sh --target imc_tactical

# Full rebuild — delete build dir first
.PHONY: rebuild
rebuild: clean build

# ---------------------------------------------------------------------------
# Format (runs inside container — clang-format is installed there)
# ---------------------------------------------------------------------------
.PHONY: format
format:
	$(DOCKER_RUN) scripts/format.sh

.PHONY: format-check
format-check:
	$(DOCKER_RUN) scripts/format.sh --check

# ---------------------------------------------------------------------------
# Interactive shell inside the container
# ---------------------------------------------------------------------------
.PHONY: shell
shell:
	docker run --rm -it \
	    -v "$(CURDIR):/workspace" \
	    -w /workspace \
	    $(IMAGE_NAME) /bin/bash

# ---------------------------------------------------------------------------
# Flash — runs on the HOST (openocd needs USB access to the ST-Link)
# ---------------------------------------------------------------------------
.PHONY: flash flash-bringup flash-tactical
flash: flash-bringup

flash-bringup:
	scripts/flash.sh --target imc_bringup

flash-tactical:
	scripts/flash.sh --target imc_tactical

# ---------------------------------------------------------------------------
# Clean
# ---------------------------------------------------------------------------
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

# ---------------------------------------------------------------------------
# Help
# ---------------------------------------------------------------------------
.PHONY: help
help:
	@echo ""
	@echo "IMC Firmware — Makefile targets"
	@echo "================================"
	@echo ""
	@echo "Container targets (run inside Docker):"
	@echo "  make image           Build the dev container image"
	@echo "  make configure       Run CMake configure"
	@echo "  make build           Configure (if needed) + compile all"
	@echo "  make build-bringup   Build imc_bringup only"
	@echo "  make build-tactical  Build imc_tactical only"
	@echo "  make rebuild         Clean + full recompile"
	@echo "  make format          Run clang-format in-place"
	@echo "  make format-check    Check formatting (CI-safe, no file changes)"
	@echo "  make shell           Interactive shell in the container"
	@echo "  make clean           Delete build/"
	@echo ""
	@echo "Host targets (require openocd + ST-Link on your machine):"
	@echo "  make flash           Flash imc_bringup (default)"
	@echo "  make flash-bringup   Flash imc_bringup"
	@echo "  make flash-tactical  Flash imc_tactical"
	@echo ""
	@echo "Override CMake options:  make build CMAKE_ARGS=\"-DCMAKE_BUILD_TYPE=Release\""
	@echo ""
