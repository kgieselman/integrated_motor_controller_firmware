# Integrated Motor Controller Firmware Software Architecture

**Bringup + Tactical Firmware · Rev A · STM32H563RIT6**

| Field      | Value                                                                    |
|------------|--------------------------------------------------------------------------|
| Status     | Bringup: drivers written, console operational. Tactical: FreeRTOS baseline integrated. |
| Board      | Integrated Motor Controller board Rev A                                  |
| MCU        | STM32H563RIT6 (Cortex-M33, up to 250 MHz, 640 KB SRAM, 2 MB Flash)    |
| Toolchain  | STM32CubeMX → CMake (Ninja) → arm-none-eabi-gcc 13.2/14.2               |
| Source     | `integrated_motor_controller_firmware/` repository                       |

---

## 1. Purpose and Scope

This document describes the software architecture for both firmware images produced by this repository:

- **`imc_bringup`** — bare-metal, no RTOS. Proves every interface on the Rev A PCB through an interactive USB-CDC console.
- **`imc_tactical`** — FreeRTOS-based competition/mission-control firmware. Shares all drivers with bringup; replaces the application layer with a preemptive task structure.

The driver layer is the reuse boundary. Each driver exposes the same interface regardless of which firmware image links it.

### 1.1 What is in scope

- USB-CDC console with line-oriented command parser (bringup)
- C++ peripheral drivers for every device on the board (LED, button, EEPROM, IMU, motor drivers, encoders, servos, ADC, UARTs)
- Bring-up test routines invoked from the console (one test module per peripheral)
- FreeRTOS V11.1.0 task architecture for the tactical target
- CMake build system (single repo, two ELF targets, Ninja)

### 1.2 What is out of scope

- ELRS ESP32 firmware — the module runs stock ELRS; the STM32 treats it as a CRSF UART peer
- Competition-specific control loops and state machines (application logic above the driver layer)

---

## 2. High-Level Architecture

Both firmware images share the same three-layer stack. Dependencies flow strictly downward; no layer calls upward.

| Layer       | Bringup contents                                     | Tactical contents                                      | Location         |
|-------------|------------------------------------------------------|--------------------------------------------------------|------------------|
| Application | Console loop, command dispatch, test routines        | FreeRTOS tasks (heartbeat, control, telemetry, …)      | `app/bringup/` · `app/tactical/` |
| Driver      | C++ peripheral drivers — one `.hpp/.cpp` pair per device, shared unchanged between both targets    | ←                                                      | `drivers/`       |
| HAL / BSP   | STM32Cube HAL (CubeMX-generated), CMSIS              | ← (same HAL; IRQ table forked in `stm32h5xx_it_tactical.c`) | `cubemx/`  |

The Driver layer is the primary reuse boundary. Each driver exposes a thin C++ interface (init, read, write, enable/disable). The tactical firmware links the same driver library and replaces only the Application layer with FreeRTOS tasks.

### 2.1 Repository tree

```
integrated_motor_controller_firmware/
├── cubemx/
│   ├── integrated_motor_controller.ioc                ← Authoritative pin/clock config — edit here
│   ├── Core/                      ← CubeMX HAL output — DO NOT edit by hand (MX_* functions, IRQ handlers)
│   ├── Drivers/                   ← STM32Cube HAL + CMSIS (CubeMX-generated)
│   └── USB_Device/                ← USB CDC middleware (CubeMX-generated)
├── drivers/                       ← Hand-written C++ hardware drivers (shared by both targets)
│   ├── include/
│   │   ├── AdcDma.hpp             ← Shared ADC1 DMA buffer (g_adcBuf[]) + slot constants
│   │   ├── Motor.hpp              ← DRV8874 IN1/IN2 mode + IPROPI via AdcDma slot
│   │   ├── Encoder.hpp            ← TIM4 CH4 / TIM8 CH1 input capture
│   │   ├── IMU.hpp                ← ICM-42688-P over SPI1
│   │   ├── EEPROM.hpp             ← M24C64 over I²C1
│   │   ├── CRSFReceiver.hpp       ← CRSF framing over UART4 (ELRS), DMA+IDLE receive
│   │   ├── Battery.hpp            ← VBAT_SENSE via AdcDma slot + voltage divider math
│   │   ├── ServoChannel.hpp       ← TIM2 CH1/CH2/CH3 PWM servo output
│   │   └── Drivetrain.hpp         ← Differential drive abstraction
│   └── src/
│       └── *.cpp
├── app/
│   ├── bringup/
│   │   ├── include/Console.hpp    ← USB-CDC command dispatcher
│   │   └── src/
│   │       ├── Console.cpp
│   │       └── main_bringup.cpp   ← Bare-metal, no RTOS
│   └── tactical/
│       ├── include/
│       │   └── FreeRTOSConfig.h   ← FreeRTOS tuning for STM32H563 @ 250 MHz
│       └── src/
│           ├── main_tactical.cpp  ← Creates tasks, starts vTaskStartScheduler()
│           ├── freertos_hooks.c   ← vApplicationTickHook (HAL_IncTick), idle, stack-overflow, static task memory
│           └── stm32h5xx_it_tactical.c  ← IRQ table — omits SVC/PendSV/SysTick (FreeRTOS port owns those)
└── docs/
    ├── readme.md
    ├── integrated_motor_controller_firmware_arch.md   ← This file
    └── style_guide.md
```

---

## 3. Tactical Firmware — FreeRTOS Architecture

### 3.1 Kernel configuration

| Parameter | Value | Notes |
|---|---|---|
| Kernel version | FreeRTOS V11.1.0 | Fetched via CMake `FetchContent` at configure time; pinned by `GIT_TAG` |
| Port | `GCC_ARM_CM33_NTZ_NONSECURE` | Cortex-M33, no TrustZone, FPU enabled (`configENABLE_FPU=1`) |
| Heap implementation | `heap_4` | Coalescing allocator; 64 KB pool in BSS (`configTOTAL_HEAP_SIZE`) |
| Tick rate | 1 kHz | `configTICK_RATE_HZ = 1000`; one SysTick IRQ per millisecond |
| `configCPU_CLOCK_HZ` | `SystemCoreClock` | Evaluated at runtime; automatically correct after `SystemClock_Config()` |
| Max task priorities | 7 | `configMAX_PRIORITIES`; priority 0 = idle, 6 = highest |
| Static allocation | Enabled | Idle and timer-daemon tasks use statically allocated TCB + stack in `freertos_hooks.c` |
| Stack overflow check | Level 2 | Runtime watermark check on every context switch |
| Config file | `app/tactical/include/FreeRTOSConfig.h` | CMake `freertos_config` interface target points the kernel here |

### 3.2 Interrupt priority scheme

The STM32H563 NVIC is configured with 4 priority bits (NVIC_PRIORITYGROUP_4 → 16 preemption levels, 0 sub-priority levels).

| Priority level | Use |
|---|---|
| 0–4 | Reserved (FreeRTOS kernel cannot mask these — never call FreeRTOS APIs from here) |
| 5 (`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`) | Highest priority allowed to call `FromISR` FreeRTOS APIs (e.g. motor fault EXTI) |
| 6–14 | Normal ISRs that use `FromISR` APIs (DMA, UART, USB, IMU INT) |
| 15 (`configLIBRARY_LOWEST_INTERRUPT_PRIORITY`) | FreeRTOS SysTick and PendSV (kernel internal) |

Any ISR that calls a `xQueueSendFromISR`, `xTaskNotifyFromISR`, or similar FreeRTOS API **must** be assigned a priority ≥ 5 (numerically). ISRs at priority 0–4 must never call FreeRTOS APIs.

### 3.3 SysTick / HAL tick co-existence

FreeRTOS owns `SysTick_Handler` (aliased via `#define xPortSysTickHandler SysTick_Handler` in `FreeRTOSConfig.h`). `HAL_IncTick()` is kept functional through the tick hook:

```c
// freertos_hooks.c
void vApplicationTickHook(void)
{
    HAL_IncTick();   // keeps HAL_GetTick() / HAL_Delay() accurate
}
```

`stm32h5xx_it_tactical.c` intentionally omits `SVC_Handler`, `PendSV_Handler`, and `SysTick_Handler`; the FreeRTOS port object provides all three.

### 3.4 Task inventory

| Task | Priority | Stack (words) | Purpose |
|---|---|---|---|
| Idle (FreeRTOS built-in) | 0 | `configMINIMAL_STACK_SIZE` (128) | Runs `__WFI()` via `vApplicationIdleHook`; statically allocated |
| Timer daemon (FreeRTOS built-in) | 6 (highest) | 256 | Software timer callbacks |
| `Heartbeat` | 1 | 256 | Toggles `DEBUG_LED_0` (PC15) at 500 ms; proves scheduler is running |

Additional application tasks (control loop, telemetry, CRSF receive) will be added here as the tactical firmware develops.

### 3.5 Tactical boot sequence

1. `HAL_Init()` — SysTick, Flash latency, power domain.
2. `SystemClock_Config()` — 250 MHz SYSCLK (same as bringup).
3. `MX_GPIO_Init()` — all GPIO clocks and EXTI lines.
4. Peripheral inits as needed (`MX_DMA_Init`, `MX_UART4_Init`, `MX_SPI1_Init`, etc.).
5. `xTaskCreate(...)` — create application tasks.
6. `vTaskStartScheduler()` — transfers control to FreeRTOS; never returns.
7. FreeRTOS runs idle task → `__WFI()` until first tick or higher-priority task unblocks.

`HAL_Delay()` is safe to call after `vTaskStartScheduler()` because `vApplicationTickHook` keeps `uwTick` incrementing. However, prefer `vTaskDelay(pdMS_TO_TICKS(n))` inside tasks so the CPU can context-switch while waiting.

---

## 4. Module Catalogue

Each driver is a `.hpp/.cpp` pair with a defined public C++ interface. Drivers depend only on the STM32 HAL — never on each other or on application code. The `.ioc` and KiCad netlist are the authoritative pin references.

| Class            | Header              | Responsibility                                                                                         | Key peripherals                       |
|------------------|---------------------|--------------------------------------------------------------------------------------------------------|---------------------------------------|
| `AdcDma`         | `AdcDma.hpp`        | ADC1 continuous scan with DMA circular transfer; shared `g_adcBuf[]` read by Motor and Battery        | ADC1, DMA                             |
| `IMU`            | `IMU.hpp`           | ICM-42688-P: SPI init, WHO_AM_I, accel/gyro burst read, interrupt config (INT1 EXTI2/PD2)             | SPI1 (PB3/PB4/PB5), CS=PC12, EXTI    |
| `EEPROM`         | `EEPROM.hpp`        | M24C64: I²C byte/page read-write, write-protect GPIO (~WC = PA9); address via solder jumpers (default 0x50) | I2C1 (PB6 SCL / PB7 SDA)      |
| `Motor`          | `Motor.hpp`         | DRV8874×2: IN1/IN2 PWM, individual PMODE + ENABLE per channel, fault EXTI, IPROPI from `g_adcBuf`    | TIM3, GPIO, EXTI                      |
| `Encoder`        | `Encoder.hpp`       | TIM4 CH4 (PC2, motor 0) + TIM8 CH1 (PC6, motor 1) single-channel input capture; count, velocity      | TIM4, TIM8 ⚠ driver needs rework to match IC mode |
| `ServoChannel`   | `ServoChannel.hpp`  | TIM2 CH1/CH2 50 Hz PWM servo output (PA0 = Servo 0, PA1 = Servo 1); Servo 2 added to PCB — ioc pin TBD | TIM2                                |
| `Battery`        | `Battery.hpp`       | VBAT_SENSE (PC4/ADC1_INP4) voltage with divider math; reads `g_adcBuf[kSlotVbat]`                    | ADC1 (via AdcDma)                     |
| `CRSFReceiver`   | `CRSFReceiver.hpp`  | CRSF framing over UART4 (PC10 TX / PC11 RX) at 420 kbaud; DMA+IDLE receive; RC_CHANNELS_PACKED decode | UART4, DMA                            |
| `Drivetrain`     | `Drivetrain.hpp`    | Differential drive: arcade / tank mixing on top of two `Motor` instances                               | —                                     |
| `Console`        | `Console.hpp`       | USB-CDC line accumulator, tokeniser, command dispatch table; class-based with `registerCommand()`      | USB_DRD_FS (PA11/PA12)               |

---

## 5. Module Interfaces

### 5.1 `Motor` — DRV8874, IN1/IN2 mode

Both motor channels operate in IN1/IN2 (independent half-bridge) mode with PMODE driven high. Current sense (IPROPI) is read from the shared `g_adcBuf[]` populated by the ADC1 DMA circular scan — Motor holds a slot index, not an ADC handle.

```cpp
Motor(TIM_HandleTypeDef* pwmTimer,
      uint32_t in1Channel, uint32_t in2Channel,
      GPIO_TypeDef* pmodePort, uint16_t pmodePin,
      GPIO_TypeDef* faultPort, uint16_t faultPin,
      uint8_t ipropSlot);   // kSlotLeftIpropi or kSlotRightIpropi from AdcDma.hpp

bool    init();
void    set(float duty, Direction direction);   // duty in [0.0, 1.0]
void    setSigned(float duty);                  // signed [-1.0, 1.0]
void    brake();                                // IN1=100%, IN2=100%
void    coast();                                // IN1=0, IN2=0
bool    isFaulted() const;
int32_t readCurrentMilliamps();                 // reads g_adcBuf[m_ipropSlot]
```

**Pin assignments (from `cubemx/integrated_motor_controller.ioc`):**
- Motor 0: IN1=PA7 (TIM3_CH2), IN2=PA6 (TIM3_CH1), MODE=PA3, ENABLE=PA5, FAULT=PC3 (EXTI3), IPROPI=PA4 (ADC1_INP18, slot 0)
- Motor 1: IN1=PB1 (TIM3_CH4), IN2=PB0 (TIM3_CH3), MODE=PB10, ENABLE=PB2, FAULT=PC7 (EXTI7), IPROPI=PC5 (ADC1_INP8, slot 1)


IN1/IN2 truth table (PMODE=high on DRV8874):

| IN1  | IN2  | Action  |
|------|------|---------|
| PWM  | 0    | Forward |
| 0    | PWM  | Reverse |
| 100% | 100% | Brake   |
| 0    | 0    | Coast   |

### 5.1a `AdcDma` — Shared ADC1 DMA Buffer

`AdcDma.hpp` is not a class — it declares a shared `volatile uint16_t g_adcBuf[3]` populated continuously by the ADC1 DMA circular scan, plus slot index constants used by `Motor` and `Battery`.

```cpp
// Slot constants
static constexpr uint8_t kSlotLeftIpropi  = 0U;  // PA4, ADC1 INP18, LEFT_MOTOR_IPROPI
static constexpr uint8_t kSlotRightIpropi = 1U;  // PC5, ADC1 INP8,  RIGHT_MOTOR_IPROPI
static constexpr uint8_t kSlotVbat        = 2U;  // PC4, ADC1 INP4

extern volatile uint16_t g_adcBuf[ADC_DMA_NUM_CHANNELS];
```


The caller (`main`) must issue after `MX_ADC1_Init()`:
```cpp
HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcBuf, ADC_DMA_NUM_CHANNELS);
```

Drivers snapshot their slot into a local variable before arithmetic; no mutex is required since naturally-aligned 16-bit reads are atomic on Cortex-M33.

### 5.2 `Encoder` — Input Capture

> **⚠ Architecture change:** Encoders were previously configured in TIM encoder mode (quadrature, two channels per timer). The current `.ioc` uses single-channel input capture: TIM4 CH4 on PC2 (motor 0) and TIM8 CH1 on PC6 (motor 1). The `Encoder` driver class was written for quadrature mode and must be rewritten to match input capture before any encoder testing.

```cpp
Encoder(TIM_HandleTypeDef* htim, uint32_t countsPerRev, float updateRateHz);

bool    init();
void    update();              // call at fixed rate; updates velocity
int64_t count() const;         // accumulated position (software, 64-bit)
float   velocityRpm() const;
void    resetCount();
```

**Pin assignments (current `.ioc`):**
- Motor 0: PC2 (TIM4 CH4, input capture from TI4) — single channel, direction unknown without second channel
- Motor 1: PC6 (TIM8 CH1, input capture from TI1) — single channel

### 5.3 `ServoChannel` — 50 Hz PWM

```cpp
ServoChannel(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t ticksPerUs = 1U);

bool    init();
void    setPulseUs(uint16_t pulseUs);    // clamped to [1000, 2000]
void    setNormalised(float position);   // [-1.0, 1.0] → [1000, 2000] µs
uint16_t currentPulseUs() const;
```

**Pin assignments:**
- Servo 0: PA0 (TIM2 CH1)
- Servo 1: PA1 (TIM2 CH2)
- Servo 2: added to PCB schematic — ioc and driver not yet updated

At 250 MHz SYSCLK, prescaler 249 → 1 MHz timer clock → 1 µs/tick, ARR = 19999 → 50 Hz. (TIM2 config confirmed in `.ioc`: `Prescaler=249`, `PeriodNoDither=19999`.)

### 5.4 `CRSFReceiver` — ELRS / CRSF UART

Receive is DMA-driven with IDLE-line detection (`HAL_UARTEx_ReceiveToIdle_DMA`). The ISR callback feeds data into an internal buffer; `update()` processes complete frames from the main loop.

```cpp
explicit CRSFReceiver(UART_HandleTypeDef* huart);   // UART4, 420 000 baud

bool     init();                              // call after MX_UART4_UART_Init() + MX_DMA_Init()
void     onDmaRxEvent(uint16_t size);         // wire to HAL_UARTEx_RxEventCallback; ISR-safe
bool     update();                            // call from main loop; returns true on new frame
uint16_t channel(uint8_t ch) const;           // CRSF units [172, 1811]
uint16_t channelUs(uint8_t ch) const;         // PWM µs [1000, 2000]
float    channelNorm(uint8_t ch) const;       // normalised [-1.0, 1.0]
bool     hasNewData();
uint32_t lastFrameAgeMs() const;
UART_HandleTypeDef* uartHandle() const;       // for identifying the handle in the ISR callback
```

Wiring the callback:
```cpp
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* h, uint16_t size)
{
    if (h == crsfReceiver.uartHandle())
        crsfReceiver.onDmaRxEvent(size);
}
```

**Pin assignments:** PC10 (UART4 TX → ELRS module), PC11 (UART4 RX ← ELRS module)

CRSF frame format: `[SYNC 0xC8] [LEN] [TYPE] [PAYLOAD…] [CRC8/DVB-S2]`

### 5.5 `Console` — USB-CDC Command Dispatcher

`Console` is a class, not a set of free functions. One instance is constructed in `main_bringup.cpp` and shared with test modules via pointer or reference.

```cpp
Console console;

// Register commands (one call per test module, before the main loop)
bool registerCommand(const char* name, const char* help, CommandHandler handler);

// Called from CDC_Receive_FS() callback with the raw received bytes
void feed(const uint8_t* buf, uint32_t len);

// Call from the main loop; non-blocking
void poll();

// Output
void print(const char* str);
void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
void println(const char* str);
```

`CommandHandler` signature: `void (*)(int argc, char* argv[])`.

USB_DRD_FS on PA11 (D−) / PA12 (D+). `feed()` accumulates bytes into a line buffer; `poll()` tokenises complete lines and dispatches to the registered command table.

---

## 6. Boot Sequence

### 6.1 Bringup (`imc_bringup`) — bare-metal

1. `HAL_Init()` — SysTick, Flash latency, power domain.
2. `SystemClock_Config()` — HSE 25 MHz → PLL → 250 MHz SYSCLK, 48 MHz USB from HSI48+CRS.
3. `MX_GPIO_Init()` — all GPIO clocks and EXTI lines.
4. `MX_DMA_Init()` — DMA channels before any peripheral using DMA.
5. `MX_USB_Device_Init()` — USB_DRD_FS CDC class; waits for host enumeration (non-blocking).
6. `MX_UART4_Init()` — CRSF UART at 420 kbaud.
7. `MX_USART1_Init()`, `MX_UART5_Init()` — growth header UARTs (USART1 on PB14/PB15, UART5 on PB12/PB13).
8. `MX_SPI1_Init()` — IMU SPI bus.
9. `MX_I2C1_Init()` — EEPROM I²C bus (PB6/PB7).
10. `MX_I2C3_Init()` — Growth header I²C bus (PA8/PC9).
11. `MX_TIM2/3/4/8_Init()` — PWM (TIM2 servo, TIM3 motor), input capture (TIM4 encoder 0, TIM8 encoder 1).
12. `MX_ADC1_Init()` — ADC1 scan configuration. Follow immediately with `HAL_ADCEx_Calibration_Start` + `HAL_ADC_Start_DMA` to start the `g_adcBuf[]` circular scan.
13. Driver `init()` calls; LED heartbeat started.
14. `console.printf(banner)` — greet once host connects.
15. Main loop: `console.poll()`.

**Interrupt priority policy (bringup):** USB IRQ at priority 5, UART DMA at priority 6, EXTI fault lines (motor fault, IMU INT1) at priority 4. SysTick at priority 15. Lower numeric value = higher urgency.

### 6.2 Tactical (`imc_tactical`) — FreeRTOS

1. `HAL_Init()` — SysTick at lowest priority (FreeRTOS will reconfigure it before the scheduler starts).
2. `SystemClock_Config()` — identical 250 MHz clock tree; `SystemCoreClock` updated so `configCPU_CLOCK_HZ` is accurate.
3. `MX_GPIO_Init()` and required peripheral inits.
4. `xTaskCreate(...)` — register application tasks (currently: `Heartbeat`).
5. `vTaskStartScheduler()` — FreeRTOS takes over. Reconfigures SysTick at 1 kHz; never returns.
6. FreeRTOS first tick fires `SysTick_Handler` → `vApplicationTickHook()` → `HAL_IncTick()`.
7. Idle task executes `__WFI()` between ticks.

**Interrupt priority policy (tactical):** ISRs that call FreeRTOS `FromISR` APIs must use priority 5–14. Priorities 0–4 are non-maskable by the kernel and must never call FreeRTOS APIs. SysTick and PendSV are forced to priority 15 by the port. See Section 3.2 for the full table.

---

## 7. Hardware–Software Mapping

Cross-reference of every MCU signal to its driver class and bring-up test. Derived from `cubemx/integrated_motor_controller.ioc`.

| GPIO | Net / Label                    | Peripheral         | Driver class    | Bring-up test              |
|------|--------------------------------|--------------------|-----------------|----------------------------|
| PC15 | DEBUG_LED_0                    | GPIO out           | LED (GPIO)      | `test leds` / heartbeat    |
| PC0  | DEBUG_LED_1                    | GPIO out           | LED (GPIO)      | `test leds`                |
| PC1  | DEBUG_LED_2                    | GPIO out           | LED (GPIO)      | `test leds`                |
| PC14 | DEBUG_BUZZER                   | GPIO out           | Buzzer (GPIO)   | `test buzzer`              |
| PC13 | EXTI13_DEBUG_BUTTON            | EXTI13             | (GPIO)          | `test button`              |
| PD2  | EXTI2_IMU_INT_1                | EXTI2              | `IMU`           | `test imu int`             |
| PC12 | IMU_SPI_CS                     | GPIO out (SPI CS)  | `IMU`           | `test imu whoami`          |
| PB3  | IMU_SPI1_SCK                   | SPI1 SCK           | `IMU`           | `test imu stream`          |
| PB4  | IMU_SPI1_MISO                  | SPI1 MISO          | `IMU`           | `test imu stream`          |
| PB5  | IMU_SPI1_MOSI                  | SPI1 MOSI          | `IMU`           | `test imu stream`          |
| PB8  | EXTI8_IMU_INT2                 | EXTI8              | `IMU`           | `test imu int`             |
| PB6  | EEPROM_I2C1_SCL                | I2C1 SCL           | `EEPROM`        | `test eeprom`              |
| PB7  | EEPROM_I2C1_SDA                | I2C1 SDA           | `EEPROM`        | `test eeprom`              |
| PA9  | EEPROM_WRITE_PROTECT           | GPIO out           | `EEPROM`        | `test eeprom write`        |
| PA15 | USB_DETECTED                   | GPIO in            | —               | USB presence sense         |
| PA3  | MOTOR_0_MODE (PMODE)           | GPIO out           | `Motor`         | `test motors`              |
| PA5  | MOTOR_0_ENABLE (nSLEEP)        | GPIO out           | `Motor`         | `test motors`              |
| PA7  | TIM3_CH2_MOTOR_0_IN_1 (EN)    | TIM3 CH2 PWM       | `Motor`         | `test motors left`         |
| PA6  | TIM3_CH1_MOTOR_0_IN_2 (PH)    | TIM3 CH1 PWM       | `Motor`         | `test motors left`         |
| PC3  | EXTI3_MOTOR_0_FAULT            | EXTI3              | `Motor`         | `test motors fault`        |
| PA4  | ADC1_INP18_MOTOR_0_IPROPI      | ADC1 INP18         | `Motor`         | `test adc` / `test motors` |
| PB10 | MOTOR_1_MODE (PMODE)           | GPIO out           | `Motor`         | `test motors`              |
| PB2  | MOTOR_1_ENABLE (nSLEEP)        | GPIO out           | `Motor`         | `test motors`              |
| PB1  | TIM3_CH4_MOTOR_1_IN_1 (EN)    | TIM3 CH4 PWM       | `Motor`         | `test motors right`        |
| PB0  | TIM3_CH3_MOTOR_1_IN_2 (PH)    | TIM3 CH3 PWM       | `Motor`         | `test motors right`        |
| PC7  | EXTI7_MOTOR_1_FAULT            | EXTI7              | `Motor`         | `test motors fault`        |
| PC5  | ADC1_INP8_MOTOR_1_IPROPI       | ADC1 INP8          | `Motor`         | `test adc` / `test motors` |
| PC2  | TIM4_CH4_ENCODER_0             | TIM4 CH4 IC        | `Encoder`       | `test encoder left`        |
| PC6  | TIM8_CH1_ENCODER_1             | TIM8 CH1 IC        | `Encoder`       | `test encoder right`       |
| PA0  | TIM2_CH1_SERVO_0               | TIM2 CH1 PWM       | `ServoChannel`  | `test servo`               |
| PA1  | TIM2_CH2_SERVO_1               | TIM2 CH2 PWM       | `ServoChannel`  | `test servo`               |
| PA2  | TIM2_CH3_SERVO_2               | TIM2 CH3 PWM       | `ServoChannel`  | `test servo`               |
| PC4  | ADC1_INP4_VBATT                | ADC1 INP4          | `Battery`       | `test adc vbat`            |
| PC10 | UART4_MCU_TO_CRSF              | UART4 TX           | `CRSFReceiver`  | `test crsf sniff`          |
| PC11 | UART4_CRSF_TO_MCU              | UART4 RX           | `CRSFReceiver`  | `test crsf sniff`          |
| PB14 | USART1_MCU_TO_GROWTH           | USART1 TX          | (uart wrapper)  | `test uart growth loopback`|
| PB15 | USART1_GROWTH_TO_MCU           | USART1 RX          | (uart wrapper)  | `test uart growth loopback`|
| PB13 | UART5_TX                       | UART5 TX           | (uart wrapper)  | `test uart aux loopback`   |
| PB12 | UART5_RX                       | UART5 RX           | (uart wrapper)  | `test uart aux loopback`   |
| PA8  | GROWTH_I2C3_SCL                | I2C3 SCL           | —               | growth header              |
| PC9  | GROWTH_I2C3_SDA                | I2C3 SDA           | —               | growth header              |
| PA11 | USB_D−                         | USB_DRD_FS         | `Console` (CDC) | all stages                 |
| PA12 | USB_D+                         | USB_DRD_FS         | `Console` (CDC) | all stages                 |

---

## 8. Design Decisions

| Decision | Rationale |
|---|---|
| C++ class-per-device | Clean encapsulation; the tactical firmware instantiates the same classes without modification. |
| Constructor injection of HAL handles | Enables unit-testable drivers (swap real handle for a stub) without touching driver code. |
| Driver state statically allocated | All driver objects in statically allocated storage. No `malloc` in drivers; predictable memory layout; safe under FreeRTOS. |
| `bool` return from `init()` | Consistent; callers decide whether to retry, halt, or log. No exceptions (`-fno-exceptions`). |
| `cubemx/` is CubeMX-owned | Custom code lives entirely in `drivers/` and `app/` — regenerating the `.ioc` never clobbers driver code. |
| Individual ENABLE/MODE per motor | Each DRV8874 channel has its own nSLEEP (ENABLE) and PMODE pin. Allows independent sleep/wake and mode control. |
| ADC1 DMA circular scan for all three channels | All three ADC signals run on a single circular DMA scan into `g_adcBuf[3]`. Drivers read their slot without locking — 16-bit aligned reads are atomic on Cortex-M33. |
| USB-CDC decoupled from console | `CDC_Receive_FS()` calls `console.feed()`; `console.poll()` drains the line buffer. Easy to replace transport in tactical. |
| FreeRTOS via CMake `FetchContent` | Kernel is fetched at configure time from a pinned `GIT_TAG`; no vendored source in the repo. Re-run configure to pick up a new version. |
| `freertos_config` interface target | Defined before `FetchContent_MakeAvailable(freertos_kernel)` so FreeRTOS CMake finds `FreeRTOSConfig.h` without the deprecated `FREERTOS_CONFIG_FILE_DIRECTORY` variable. |
| `stm32h5xx_it_tactical.c` forks the IRQ table | `SVC_Handler`, `PendSV_Handler`, and `SysTick_Handler` are owned by the FreeRTOS port (via `#define` aliases in `FreeRTOSConfig.h`). The tactical IT file omits them; `HAL_IncTick()` is restored through `vApplicationTickHook`. |
| `heap_4` (coalescing, 64 KB) | Good general-purpose choice for a mixed static/dynamic allocation pattern. 64 KB is conservative given 640 KB SRAM; shrink once task watermarks are profiled. |
| Static memory for idle + timer tasks | `vApplicationGetIdleTaskMemory` / `vApplicationGetTimerTaskMemory` in `freertos_hooks.c` provide statically allocated TCB + stack. Required because `configSUPPORT_STATIC_ALLOCATION = 1`. |

---

## 9. Open Items

See [`ISSUES.md`](../../ISSUES.md) for the authoritative cross-repo tracker. Architecture-relevant items:

| # | Item | Status | Notes |
|---|------|--------|-------|
| — | Driver headers have stale pin references — `Encoder.hpp` (TIM2 quadrature, now TIM4/TIM8 input capture) | Open | `Encoder.hpp` must be rewritten for IC mode before any encoder testing |
| N2 | Integration wiring absent — ADC DMA never starts, CRSF/Console never receive | Open | Wire `HAL_ADCEx_Calibration_Start` + `HAL_ADC_Start_DMA`, `CRSFReceiver::init()`, `HAL_UARTEx_RxEventCallback`, `CDC_Receive_FS` → `Console::feed()` |
| — | Tactical task inventory is a stub — only `Heartbeat` task exists | Open | Define and create control loop, CRSF receive, and telemetry tasks; assign priorities using Section 3.2 policy |
| — | Tactical peripheral init incomplete — only GPIO initialised in `main_tactical.cpp` | Open | Add `MX_DMA_Init`, `MX_UART4_Init`, `MX_SPI1_Init`, `MX_ADC1_Init`, timer inits before `vTaskStartScheduler()` |
| — | FreeRTOS ISR priority not yet set for shared peripherals | Open | Any ISR calling FreeRTOS `FromISR` APIs must be assigned priority 5–14 via `HAL_NVIC_SetPriority` before `vTaskStartScheduler()` |
| — | `configTOTAL_HEAP_SIZE` (64 KB) is unvalidated | Open | Profile task stack watermarks with `uxTaskGetStackHighWaterMark()`; right-size heap |
| — | `Encoder` driver must be rewritten for input capture mode | Open | Hardware uses TIM4 CH4 IC (PC2) and TIM8 CH1 IC (PC6) — quadrature driver is wrong |
| N1 | CRSF DMA re-arm collides with circular DMA config | Open | Revert UART4 RX DMA to NORMAL mode or use a `m_dmaTail` ring index |
| F7 | 64-bit encoder accumulator not atomic on Cortex-M33 | Open | Protect with `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` in tactical, or use 32-bit counter with overflow tracking |
| F18 | CRSF DMA re-arm after byte-copy loop | Open | Address together with N1 |
| F4 | `IMU::readBurst` fixed 12-byte dummy buffer ignores `len` param | Open | Size the buffer to `len` or add assertion |
| F2/F8 | Console USB re-entrancy and blocking `HAL_Delay` in transmit path | Open | Move transmit to non-blocking path (moot for tactical which does not use `Console`) |
| F3 | Console ISR/main shared state | Open | `m_lineBuf`, `m_lineIdx`, `m_lineReady` unprotected between `feed()` (ISR) and `poll()` (main) |
| F19–F21 | CRSF `uint8_t` arithmetic, `m_dmaRxBuf` not `volatile`, PRIMASK not saved | Open | See ISSUES.md |
| P10 | VBAT divider ratio unconfirmed | Open | Measure on bench; encode confirmed ratio in `Battery.hpp::kDividerRatio` |
| — | CI (GitHub Actions) not set up | Open | Matrix build ubuntu/windows via `carlosperate/arm-none-eabi-gcc-action@v1` |
| — | Buzzer driver absent — PC14 (DEBUG_BUZZER) has no driver class or bringup test | Open | Add simple GPIO toggle wrapper and `test buzzer` console command |
| — | FreeRTOS kernel integrated for tactical target | ✅ Done | V11.1.0, port `GCC_ARM_CM33_NTZ_NONSECURE`, heap_4; both targets build clean — 2026-05-17 |
| — | `stm32h5xx_it.c` split per-target — tactical uses `stm32h5xx_it_tactical.c` | ✅ Done | FreeRTOS port owns SVC/PendSV/SysTick; `HAL_IncTick` restored via tick hook — 2026-05-17 |
| — | SERVO_2 ioc updated — PA2 (TIM2 CH3) | ✅ Done | — |
| F1 | LMOT_FAULT pin conflict | ✅ Done | Resolved 2026-04-23 |
| F5 | ADC1 shared without coordination | ✅ Done | `AdcDma.hpp/cpp` circular DMA scan — resolved 2026-04-23 |
| — | CubeMX HAL output generated into `cubemx/` | ✅ Done | — |
| — | CRSF ISR/main shared state (`volatile` + critical sections) | ✅ Done | Resolved 2026-04-22 |
| F14 | Top-level CMakeLists.txt and both `main_*.cpp` files absent | ✅ Done | Both targets compile; Docker + Ninja build working |

---

## 10. Success Criteria

### Bringup (`imc_bringup`)

- Every stage in the bring-up checklist (`docs/readme.md`) produces `PASS` on the USB-CDC console.
- All driver class interfaces are stable — the tactical firmware links the same library without modification.
- The validated board is tagged `v0.1.0-bringup` and the checklist is archived with board serial and date.

### Tactical (`imc_tactical`)

- `imc_bringup` and `imc_tactical` both compile without warnings from a single `cmake --build` invocation.
- FreeRTOS scheduler starts and `Heartbeat` task toggles `DEBUG_LED_0` at 500 ms as observed on hardware.
- No stack overflow hooks fire; `uxTaskGetStackHighWaterMark()` shows ≥ 20% headroom on all tasks.
- All ISRs calling FreeRTOS APIs operate at priority 5–14; no `configASSERT` fires in the kernel.
- CI passes on ubuntu-latest for both targets.

---

*Pin assignments are derived from `cubemx/integrated_motor_controller.ioc` and `integrated_motor_controller_pcb/output/integrated_motor_controller.net`. Update this document when the hardware rev changes.*
