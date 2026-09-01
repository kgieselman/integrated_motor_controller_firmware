# Integrated Motor Controller Firmware

Modular embedded C++ firmware for the Integrated Motor Controller board, targeting the **STM32H563RIT6** (Cortex-M33, 250 MHz, 2 MB flash, 640 KB SRAM).

The repo contains two applications that share a common driver library:

- **Bringup** — interactive USB-CDC console for hardware validation (11-stage checklist)
- **Tactical** — competition/mission control logic (stub, in progress)

---

## Hardware

| Subsystem | Part | Interface |
|-----------|------|-----------|
| MCU | STM32H563RIT6 @ 250 MHz, 2 MB flash, 640 KB SRAM | — |
| IMU | ICM-42688-P (±16 g / ±2000 °/s) | SPI1 (PB3/PB4/PB5), CS=PC12 |
| Motor drivers (×2) | DRV8874 H-bridge | TIM3 CH1–4 PWM + ADC1 current sense |
| Encoders (×2) | Single-channel input capture | TIM4 CH4 (PC2, motor 0), TIM8 CH1 (PC6, motor 1) |
| EEPROM | M24C64 8 KB | I²C1 (PB6/PB7), WC=PA9 |
| Radio | ELRS receiver (CRSF, 16 ch) | UART4 @ 420 kbaud (PC10/PC11) |
| Servo outputs | 3-channel PWM (50 Hz) | TIM2 CH1/CH2/CH3 (PA0/PA1/A2) |
| Battery monitor | ADC voltage divider | ADC1 INP4 (PC4) |
| Debug console | USB-CDC virtual serial | PA11/PA12 |

---

## Repository Structure

```
integrated_motor_controller_firmware/
├── app/
│   ├── bringup/        # USB-CDC console + hardware validation app
│   └── tactical/       # Competition firmware (stub)
├── cmake/              # Toolchain file (gcc-arm-none-eabi)
├── cubemx/
│   ├── integrated_motor_controller.ioc     # CubeMX project (pin/clock config)
│   ├── Core/           # CubeMX HAL output — do not edit by hand
│   ├── Drivers/        # STM32Cube HAL + CMSIS
│   └── ...             # Other CubeMX-generated files
├── docs/
│   ├── readme.md           # Toolchain setup, bringup checklist, pin map
│   ├── integrated_motor_controller_firmware_arch.md  # Software architecture reference
│   └── style_guide.md      # C++ conventions and Doxygen requirements
├── drivers/            # Hardware abstraction layer (8 driver classes)
├── scripts/
│   ├── build.sh        # Configure + compile (runs inside container)
│   ├── connect.sh      # Attach to the board's serial console
│   ├── cppcheck.sh     # Static analysis over app/ and drivers/
│   └── flash.sh        # Flash via OpenOCD (runs on host)
├── .devcontainer/
│   └── devcontainer.json   # VS Code Dev Containers configuration
├── .clang-format       # Formatting rules derived from style_guide.md
├── Dockerfile          # Dev container image definition
└── Makefile            # docker run wrappers for common tasks
```

---

## Driver Library

| Class | Hardware | Responsibility |
|-------|----------|----------------|
| `Motor` | DRV8874 + TIM3 PWM + ADC1 DMA | Forward/reverse/brake/coast; individual ENABLE + MODE pins per channel; current readback via shared ADC DMA buffer |
| `Encoder` | TIM4 CH4 / TIM8 CH1 input capture | Position accumulation, RPM velocity (single-channel IC, not quadrature) |
| `Drivetrain` | Motor + Encoder pairs | Differential drive, arcade/tank steering |
| `IMU` | ICM-42688-P via SPI1 | Accel/gyro burst read, float conversion |
| `EEPROM` | M24C64 via I²C1 | Byte/page read-write with ACK polling |
| `CRSFReceiver` | UART4 DMA + IDLE IRQ | ELRS frame parse, 16-channel RC decode |
| `ServoChannel` | TIM2 CH1/CH2 | 50 Hz PWM, normalized [-1, 1] input |
| `Battery` | ADC1 DMA voltage divider | Battery voltage in millivolts via shared ADC DMA buffer |
| `AdcDma` | ADC1 DMA circular scan | Shared DMA buffer for motor 0 IPROPI, motor 1 IPROPI, and VBAT_SENSE |

All drivers are constructed with injected HAL handles, use no dynamic allocation, and are documented with Doxygen.

---

## Building

The recommended workflow uses a Docker container — no local toolchain installation required.

### Prerequisites

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (Windows/macOS) or Docker Engine (Linux)
- For VS Code: the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension

STM32CubeMX 6.12+ is still needed on the host if you need to regenerate HAL code from the `.ioc` file.

### Option A — VS Code Dev Containers (recommended)

1. Open the repo folder in VS Code
2. Click **"Reopen in Container"** when prompted (or run **Dev Containers: Reopen in Container** from the Command Palette)
3. VS Code builds the image and configures CMake automatically — clangd IntelliSense is ready when it finishes
4. Build from the integrated terminal:

```bash
scripts/build.sh                    # all targets
scripts/build.sh -t imc_bringup    # bringup only
scripts/build.sh -t imc_tactical   # tactical only
```

### Option B — Command line

```bash
# Build the image once
docker build -t imc-firmware-dev .

# Build all targets
docker run --rm -v "$(pwd):/workspace" -w /workspace imc-firmware-dev scripts/build.sh
```

See `scripts/build.sh --help` for available options (`--type Release`, `--clean`, etc.).

### Flash

Flashing runs on the **host** (the container does not have USB access to the ST-Link). Connect an ST-Link to the SWD header (J1), then from a host terminal:

```bash
scripts/flash.sh                    # flash imc_bringup (default)
scripts/flash.sh -t imc_tactical   # flash imc_tactical
```

Requires [OpenOCD 0.12.0+](https://openocd.org) installed on your host machine.

---

## Bringup Checklist

The bringup app exposes a USB-CDC console with commands for each subsystem. Validation proceeds in 11 stages:

1. Power-on / LED smoke test
2. USB-CDC console
3. User button
4. EEPROM read/write
5. IMU WHO_AM_I + live data
6. Left motor + encoder
7. Right motor + encoder
8. Drivetrain (combined)
9. Servo outputs
10. Battery voltage
11. ELRS radio (CRSF frames)

Each stage prints `PASS` or `FAIL`. See [`docs/readme.md`](docs/readme.md) for expected console output and pass criteria.

---

## Growth Options

Deferred improvements that are architecturally clean to add later without breaking existing drivers:

| Option | Benefit | Prerequisite |
|--------|---------|--------------|
| ADC1/ADC2 dual simultaneous sampling | Captures both motor current readings at the same PWM phase point — useful for torque balancing or accurate stall detection under load. PA2 (motor 0) and PC5 (motor 1) are the current IPROPI pins. | Current-mode motor control in tactical firmware |

---

## Documentation

- **[Architecture](docs/integrated_motor_controller_firmware_arch.md)** — layer diagram, module catalogue, boot sequence, hardware–software mapping
- **[Style Guide](docs/style_guide.md)** — C++17 conventions, naming, Doxygen requirements, embedded rules
- **[Detailed README](docs/readme.md)** — full toolchain setup, complete pin map, bringup procedures

---

## License

GNU General Public License v3.0 — see [LICENSE](LICENSE).
