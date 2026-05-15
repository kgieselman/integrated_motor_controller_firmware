# =============================================================================
# IMC Firmware — Development Container
#
# Includes:
#   - arm-none-eabi-gcc (latest stable via Ubuntu repos)
#   - CMake + Ninja
#   - clangd  (for VS Code / Neovim IntelliSense via the Dev Containers extension)
#   - clang-format + clang-tidy
#   - Python 3  (flash scripts, code-gen helpers, etc.)
#   - git, make, and common dev utilities
# =============================================================================
FROM ubuntu:24.04

LABEL maintainer="integrated-motor-controller"
LABEL description="Embedded dev environment for STM32H563 firmware"

# Prevent apt from trying to interact with stdin during installs
ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# System packages
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ARM cross-compiler toolchain
    gcc-arm-none-eabi \
    binutils-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    # Build system
    cmake \
    ninja-build \
    make \
    # Clang tooling (clangd, clang-format, clang-tidy)
    clangd \
    clang-format \
    clang-tidy \
    # Python (scripts, code-gen)
    python3 \
    python3-pip \
    python3-venv \
    # Utilities
    git \
    curl \
    wget \
    ca-certificates \
    file \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Sanity-check that the toolchain is present and print versions at build time
# ---------------------------------------------------------------------------
RUN arm-none-eabi-gcc --version \
    && cmake --version \
    && ninja --version \
    && clangd --version

# ---------------------------------------------------------------------------
# Working directory — the repo root is mounted here at runtime
# ---------------------------------------------------------------------------
WORKDIR /workspace

# Default to an interactive shell; docker run / exec override this
CMD ["/bin/bash"]
