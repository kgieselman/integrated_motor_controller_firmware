# Integrated Motor Controller Firmware

Firmware for the Integrated Motor Controller board (STM32H563RIT6). Organized as two application targets — **bringup** (hardware validation) and **tactical** (competition firmware) — sharing a common set of C++ hardware drivers.

**Hardware:** STM32H563RIT6 @ 250 MHz · ICM-42688-P IMU · 2× DRV8874 motor drivers · M24C64 EEPROM · ELRS (ESP32 + SX1280) on UART4

---

## Current State

| Layer | Status |
|---|---|
| `cubemx/integrated_motor_controller.ioc` | ✅ Committed — pin map, clock tree, peripheral config complete |
| `cubemx/` (HAL output) | ✅ Generated — CubeMX output in `cubemx/` alongside `.ioc` |
| `drivers/` | ✅ All hardware drivers written (C++), including `AdcDma` for shared ADC1 DMA buffer |
| `app/bringup/` | 🔄 In progress — `Console` class implemented; test commands not yet wired in `main_bringup.cpp` |
| `app/tactical/` | ⬜ Stub only |
| CMakeLists.txt | ✅ Created — repo builds `imc_bringup` and `imc_tactical` targets |

---

## Toolchain

**STM32CubeMX** (pin/clock config + HAL codegen) → **CMake + Ninja** → **arm-none-eabi-gcc** → **OpenOCD** (flash/debug via SWD header J1)

The same toolchain carries from bringup through tactical firmware — no migration later.

### Prerequisites

The repo ships a Docker-based dev environment — no local toolchain installation needed for building.

| Tool | Where needed | Notes |
|---|---|---|
| Docker Desktop | Host (always) | Provides the build container |
| OpenOCD 0.12.0+ | Host (for flashing) | Talks to the ST-Link; runs outside the container |
| ST-Link drivers | Host (for flashing) | Windows: [ST-LINK driver](https://www.st.com/en/development-tools/stsw-link009.html) · Linux: udev rules below |
| STM32CubeMX 6.12+ | Host (optional) | Only needed when editing pin/clock config in the `.ioc` |
| VS Code + Dev Containers extension | Host (optional) | Recommended editor setup; see `README.md` |

The container provides: `arm-none-eabi-gcc 13`, CMake 3.28+, Ninja, clangd, clang-format, clang-tidy, Python 3.

**udev rules (Linux, once per machine — required for ST-Link access):**
```bash
sudo curl -fsSL -o /etc/udev/rules.d/49-stlinkv3.rules \
  https://raw.githubusercontent.com/stlink-org/stlink/develop/config/udev/rules.d/49-stlinkv3.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## Repo Layout

```
integrated_motor_controller_firmware/
├── cubemx/
│   ├── integrated_motor_controller.ioc                ← Authoritative pin/clock config — edit here
│   ├── Core/                      ← CubeMX HAL output — DO NOT edit by hand (MX_* functions, IRQ handlers)
│   ├── Drivers/                   ← STM32Cube HAL + CMSIS (CubeMX-generated)
│   └── USB_Device/                ← USB CDC middleware (CubeMX-generated)
├── cmake/
│   └── gcc-arm-none-eabi.cmake    ← CMake toolchain file (Cortex-M33 flags, compiler paths)
├── drivers/                       ← Hand-written C++ hardware drivers
│   ├── include/
│   │   ├── AdcDma.hpp             ← Shared ADC1 DMA buffer (g_adcBuf[]) + slot constants
│   │   ├── Motor.hpp              ← DRV8874 IN1/IN2 mode + IPROPI via shared ADC DMA slot
│   │   ├── Encoder.hpp            ← TIM2 (left) / TIM8 (right) quadrature
│   │   ├── IMU.hpp                ← ICM-42688-P over SPI1
│   │   ├── EEPROM.hpp             ← M24C64 over I²C2
│   │   ├── CRSFReceiver.hpp       ← CRSF framing over UART4 (ELRS), DMA+IDLE receive
│   │   ├── Battery.hpp            ← VBAT_SENSE via shared ADC DMA slot + divider math
│   │   ├── ServoChannel.hpp       ← TIM1 CH1/CH2 PWM servo output
│   │   └── Drivetrain.hpp         ← Differential drive abstraction
│   └── src/
│       └── *.cpp
├── app/
│   ├── bringup/
│   │   ├── include/Console.hpp    ← USB-CDC command dispatcher
│   │   └── src/
│   │       ├── Console.cpp
│   │       └── main_bringup.cpp   ← menu loop over USB-CDC
│   └── tactical/
│       └── src/main_tactical.cpp  ← Placeholder
├── scripts/
│   ├── build.sh                   ← Configure + compile (runs inside the container)
│   ├── flash.sh                   ← Flash via OpenOCD (runs on the host)
│   └── format.sh                  ← clang-format on app/ and drivers/
├── .devcontainer/
│   └── devcontainer.json          ← VS Code Dev Containers config
├── docs/
│   └── readme.md                  ← This file
├── .clang-format                  ← Formatting rules (derived from style_guide.md)
├── Dockerfile                     ← Dev container image
└── Makefile                       ← docker run wrappers for common tasks
```

### CMake targets

- **`STM32_Drivers`** (object lib) — CubeMX HAL + CMSIS; regenerated from `.ioc`, never edited by hand.
- **`imc_bringup`** (executable) — USB-CDC hardware validation console; produces `build/imc_bringup.elf`.
- **`imc_tactical`** (executable) — competition firmware; produces `build/imc_tactical.elf`.

### CubeMX workflow

1. Open `cubemx/integrated_motor_controller.ioc` in CubeMX 6.12+.
2. Make pin/clock changes, then **Project Manager → Toolchain/IDE = CMake**. Output path defaults to the `.ioc` directory (`cubemx/`) — leave it as-is.
3. Generate. Commit everything under `cubemx/` — custom driver code in `drivers/` and `app/` is never touched by the regenerator.

---

## Build and Flash

### Build (inside the container)

From the VS Code integrated terminal (when opened in the Dev Container), or via `docker run`:

```bash
scripts/build.sh                      # configure + build all targets
scripts/build.sh -t imc_bringup      # bringup only
scripts/build.sh -t imc_tactical     # tactical only
scripts/build.sh -t imc_bringup -T Release   # release build
scripts/build.sh --clean             # wipe build/ and rebuild
```

Run `scripts/build.sh --help` for full options. Build artifacts land in `build/`.

### Flash (on the host — outside the container)

The container does not have USB access to the ST-Link. Run flash commands from a terminal on your host machine with the board connected via SWD header J1:

```bash
scripts/flash.sh                      # flash imc_bringup (default)
scripts/flash.sh -t imc_tactical     # flash imc_tactical
```

Requires OpenOCD 0.12.0+ on the host. Run `scripts/flash.sh --help` for options.

### USB-CDC console after flashing bringup

```bash
screen /dev/ttyACM0 115200          # Linux
screen /dev/tty.usbmodem* 115200    # macOS
# Windows: PuTTY on the enumerated COM port, 115200 baud
```

---

## Pin Map (STM32H563RIT6, from `cubemx/integrated_motor_controller.ioc`)

> **Authoritative source:** `cubemx/integrated_motor_controller.ioc`. Update this table whenever the `.ioc` changes.

| Pin | GPIO | Net / Label | Peripheral | Notes |
|---|---|---|---|---|
| PA0 | TIM2 CH1 | TIM2_CH1_SERVO_0 | TIM2 PWM | Servo ch 0; 50 Hz, prescaler 249 → 1 µs/tick, ARR 19999 |
| PA1 | TIM2 CH2 | TIM2_CH2_SERVO_1 | TIM2 PWM | Servo ch 1 |
| PA2 | ADC1 INP14 | ADC1_INP14_MOTOR_0_IPROPI | ADC1 | Motor 0 current sense (DRV8874 IPROPI) |
| PA3 | GPIO in / EXTI3 | EXTI3_MOTOR_0_FAULT | EXTI3 | DRV8874 motor 0 ~FAULT (active low) |
| PA4 | GPIO out | MOTOR_0_MODE | — | DRV8874 motor 0 PMODE — drive high for IN1/IN2 mode |
| PA5 | GPIO out | MOTOR_0_ENABLE | — | DRV8874 motor 0 nSLEEP |
| PA6 | TIM3 CH1 | TIM3_CH1_MOTOR_0_IN_2 | TIM3 PWM | Motor 0 IN2 (PH / direction pin) |
| PA7 | TIM3 CH2 | TIM3_CH2_MOTOR_0_IN_1 | TIM3 PWM | Motor 0 IN1 (EN / speed pin) |
| PA8 | I2C3 SCL | GROWTH_I2C3_SCL | I2C3 | Growth header I²C |
| PA9 | GPIO out | EEPROM_WRITE_PROTECT | — | M24C64 ~WC — drive low to enable writes |
| PA11 | USB D− | USB_DM | USB_DRD_FS | USB-CDC console |
| PA12 | USB D+ | USB_DP | USB_DRD_FS | USB-CDC console |
| PA13 | SWD | SWDIO | SWD | J2 debug header |
| PA14 | SWD | SWCLK | SWD | J2 debug header |
| PB0 | TIM3 CH3 | TIM3_CH3_MOTOR_1_IN_2 | TIM3 PWM | Motor 1 IN2 (PH / direction pin) |
| PB1 | TIM3 CH4 | TIM3_CH4_MOTOR_1_IN_1 | TIM3 PWM | Motor 1 IN1 (EN / speed pin) |
| PB2 | GPIO out | MOTOR_1_ENABLE | — | DRV8874 motor 1 nSLEEP |
| PB3 | SPI1 SCK | IMU_SPI1_SCK | SPI1 | ICM-42688-P |
| PB4 | SPI1 MISO | IMU_SPI1_MISO | SPI1 | ICM-42688-P |
| PB5 | SPI1 MOSI | IMU_SPI1_MOSI | SPI1 | ICM-42688-P |
| PB6 | I2C1 SCL | EEPROM_I2C1_SCL | I2C1 | M24C64 SCL |
| PB7 | I2C1 SDA | EEPROM_I2C1_SDA | I2C1 | M24C64 SDA |
| PB8 | GPIO in / EXTI8 | EXTI8_IMU_INT2 | EXTI8 | ICM-42688-P INT2 |
| PB10 | GPIO out | MOTOR_1_MODE | — | DRV8874 motor 1 PMODE — drive high for IN1/IN2 mode |
| PB12 | UART5 RX | UART5_RX | UART5 | Growth header UART (aux) |
| PB13 | UART5 TX | UART5_TX | UART5 | Growth header UART (aux) |
| PB14 | USART1 TX | USART1_MCU_TO_GROWTH | USART1 | Growth header UART |
| PB15 | USART1 RX | USART1_GROWTH_TO_MCU | USART1 | Growth header UART |
| PC0 | GPIO out | DEBUG_LED_0 | — | Debug LED 0 |
| PC1 | GPIO out | DEBUG_LED_1 | — | Debug LED 1 |
| PC2 | TIM4 CH4 IC | TIM4_CH4_ENCODER_0 | TIM4 input capture | Motor 0 encoder — single-channel input capture |
| PC3 | GPIO out | DEBUG_LED_2 | — | Debug LED 2 |
| PC4 | ADC1 INP4 | ADC1_INP4_VBATT | ADC1 | Battery voltage divider |
| PC5 | ADC1 INP8 | ADC1_INP8_MOTOR_1_IPROPI | ADC1 | Motor 1 current sense (DRV8874 IPROPI) |
| PC6 | TIM8 CH1 IC | TIM8_CH1_ENCODER_1 | TIM8 input capture | Motor 1 encoder — single-channel input capture |
| PC7 | GPIO in / EXTI7 | EXTI7_MOTOR_1_FAULT | EXTI7 | DRV8874 motor 1 ~FAULT (active low) |
| PC9 | I2C3 SDA | GROWTH_I2C3_SDA | I2C3 | Growth header I²C |
| PC10 | UART4 TX | UART4_MCU_TO_CRSF | UART4 | CRSF to ELRS module |
| PC11 | UART4 RX | UART4_CRSF_TO_MCU | UART4 | CRSF from ELRS module @ 420 kbaud |
| PC12 | GPIO out | IMU_SPI_CS | SPI1 CS | ICM-42688-P chip select (software-controlled) |
| PC13 | GPIO in / EXTI13 | EXTI13_DEBUG_BUTTON | EXTI13 | User debug button |
| PD2 | GPIO in / EXTI2 | EXTI2_IMU_INT_1 | EXTI2 | ICM-42688-P INT1 |

---

## Bringup Checklist

Perform these in order on a freshly assembled board. Each stage logs `PASS`/`FAIL` to the USB-CDC console.

**Stage 0 — Power-on smoke test (no firmware)**
Bench-power at nominal voltage. Verify 5V and 3.3V rails; measure idle current <50 mA with motors disabled.

**Stage 1 — SWD + blinky**
Flash bringup image. Confirm DEBUG_LED_0 (PC0) blinks at 1 Hz (validates HSE, PLL, GPIO).

**Stage 2 — USB-CDC console**
Plug USB-C (J6); board enumerates as a CDC serial device. Expect `Integrated Motor Controller bring-up vX.Y — type 'help'`.

**Stage 3 — LEDs + button + buzzer**
`test leds` — cycle all three debug LEDs (PC15=LED0, PC0=LED1, PC1=LED2). `test button` — confirm PC13 EXTI13 interrupt fires on press. `test buzzer` — toggle PC14 (DEBUG_BUZZER) a few times; confirm audible output.

**Stage 4 — EEPROM (I²C1)**
`test eeprom read 0` → `0xFF`. `test eeprom write 0 42` / `test eeprom read 0` → round-trip. EEPROM address `0x50` on I²C1 (PB6/PB7); solder jumpers A0/A1/A2 all bridged to GND by default.

**Stage 5 — IMU (SPI1)**
`test imu whoami` → `0x47`. `test imu stream` — 100 Hz gyro+accel; with board flat and level verify
accelZ ≈ +9.81 m/s², accelX ≈ 0, accelY ≈ 0, all gyro axes ≈ 0. Then rotate the board and confirm
accel vector magnitude stays ≈ 9.81 m/s² and axis signs track the motion.

**Stage 6 — ADC**
`test adc vbat` — compare to calibrated DC supply (not just a bench meter). `test adc iprop left/right` — near zero with motors off.

**VBAT calibration (required per board):** The VBAT_SENSE divider uses JLC resistors; each board must be trimmed in software. With the calibrated supply set to a known voltage in the 6–9 V range, run the ADC calibration routine (trigger TBD — options include a `cal vbat` console command, button-hold at boot, or EEPROM flag). The routine computes and stores a per-board scale factor in EEPROM; `Battery.hpp` must apply it at runtime. Verify linearity at a second setpoint before proceeding.

**Stage 7 — Motor drivers**
ENABLE_MOTORS (PA4) high to wake. `test motor left forward 25` / `reverse` / `brake`, then right. Verify FAULT pins stay high.

**Stage 8 — Encoders**
`test encoder left` / `right` — spin wheel by hand, confirm counter direction matches motor forward.

**Stage 9 — Servo PWM**
`test servo 0 1500` / `test servo 1 1500` — plug in a servo, sweep 1000–2000 µs.

**Stage 10 — ELRS/CRSF**
`test crsf sniff` — expect sync byte `0xC8` on UART4 at 420 kbaud.

**Stage 11 — Aux UARTs**
Loopback jumper on USART1 (PB14↔PB15) and UART5 (PC12↔PD2). `test uart aux loopback` / `test uart config loopback`.

Once all stages pass, tag `v0.1.0-bringup` and record board serial + date.
