# GitHub Issues — IMC Bringup Load Plan

Paste each block below into **New Issue** on `kgieselman/integrated_motor_controller_firmware`.
Suggested labels are noted in parentheses for each issue.

---

## Prerequisites — must land before any stage testing

---

### Issue: Wire bringup console commands into `main_bringup.cpp`
**Labels:** `bringup` `infrastructure`

**Description:**

`Console` is instantiated and `poll()` is called in the main loop, but no test commands have been registered. The bringup cannot exercise any peripheral until each test module is connected via `console.registerCommand(...)`.

**Work required:**
- Create one test function (or a small test module file) per peripheral: LEDs/button/buzzer, EEPROM, IMU, ADC, motors, encoders, servos, CRSF, aux UARTs.
- Call `console.registerCommand("test leds", ...)`, `"test eeprom"`, `"test imu whoami"`, `"test imu stream"`, `"test adc"`, `"test motor left"`, `"test motor right"`, `"test encoder left"`, `"test encoder right"`, `"test servo"`, `"test crsf sniff"`, `"test uart aux loopback"`, `"test uart config loopback"` from `main_bringup.cpp` before the main loop.
- Ensure each handler prints `PASS` or `FAIL` with enough diagnostic detail to triage failures.

**Acceptance criteria:**
- `help` on the USB-CDC console lists all expected commands.
- Each command runs without crashing when invoked with valid arguments.

---

### Issue: Wire ADC DMA start and USB-CDC `Console::feed()` integration (N2)
**Labels:** `bringup` `bug` `infrastructure`

**Description:**

Two integration wires are missing from `main_bringup.cpp` that prevent the ADC and console from functioning:

1. **ADC DMA never starts.** `MX_ADC1_Init()` is called, but neither `HAL_ADCEx_Calibration_Start` nor `HAL_ADC_Start_DMA` is issued. `g_adcBuf[]` stays at zero, so motor current sense and battery voltage read garbage.
2. **USB-CDC receive is not routed to `Console`.** `CDC_Receive_FS()` (in `usbd_cdc_if.c`) does not call `console.feed()`. The console never receives any user input over USB.
3. **`CRSFReceiver::init()` is not called**, so the CRSF DMA receive never arms.
4. **`HAL_UARTEx_RxEventCallback` is not wired** to `crsfReceiver.onDmaRxEvent()`.

**Work required:**
- After `MX_ADC1_Init()` in `main_bringup.cpp`:
  ```cpp
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(g_adcBuf), ADC_DMA_NUM_CHANNELS);
  ```
- In `usbd_cdc_if.c`, route received data: `console.feed(Buf, *Len);`
- Instantiate `CRSFReceiver` and call `crsfReceiver.init()` after `MX_UART4_UART_Init()`.
- Add `HAL_UARTEx_RxEventCallback` that dispatches to `crsfReceiver.onDmaRxEvent(size)`.

**Acceptance criteria:**
- `test adc vbat` returns a plausible voltage when a battery is connected.
- Characters typed in a serial terminal appear in the USB-CDC console and commands execute.

---

### Issue: Add `Buzzer` driver class and `test buzzer` console command
**Labels:** `bringup` `driver` `missing-feature`

**Description:**

PC14 (`DEBUG_BUZZER`) is present on the schematic and in the `.ioc` but has no driver class and no bringup test command. Stage 3 of the bringup checklist requires audible buzzer validation.

**Work required:**
- Add a minimal `Buzzer` class (or free-function wrapper) in `drivers/` that toggles PC14 at a configurable frequency using `HAL_GPIO_TogglePin` + `HAL_Delay` or a TIM-based approach.
- Register `test buzzer` in the console: pulse the buzzer 3–5 times; print `PASS` if no fault.
- Add Doxygen comment block consistent with `style_guide.md`.

**Acceptance criteria:**
- `test buzzer` produces audible output on a board with a buzzer populated on PC14.
- Driver compiles cleanly; `imc_bringup` ELF size delta is reasonable.

---

### Issue: Rewrite `Encoder` driver for single-channel input capture mode
**Labels:** `bringup` `driver` `bug`

**Description:**

The `Encoder` driver class was written for TIM quadrature encoder mode (two channels, hardware count). The current `.ioc` configures both encoder channels as **single-channel input capture**:
- Motor 0: TIM4 CH4 input capture from TI4, pin PC2
- Motor 1: TIM8 CH1 input capture from TI1, pin PC6

The quadrature driver is functionally wrong for this hardware and must be replaced before encoder testing (Stage 8).

**Work required:**
- Rewrite `Encoder.hpp` / `Encoder.cpp` to:
  - Start input capture with `HAL_TIM_IC_Start_IT()`.
  - Accumulate edge counts in the `HAL_TIM_IC_CaptureCallback` ISR (software 32/64-bit counter).
  - Compute velocity from inter-capture timestamps or periodic count snapshots.
  - Expose the same public interface: `init()`, `update()`, `count()`, `velocityRpm()`, `resetCount()`.
- Note: direction is unknown without a second channel; document this limitation in the header.
- Update stale pin references in `Encoder.hpp` comments (currently references TIM2 quadrature).

**Acceptance criteria:**
- `test encoder left` and `test encoder right` print a non-zero count when a wheel is spun by hand.
- Counter direction is consistent (monotonically increasing or decreasing for a given spin direction).
- Both `imc_bringup` and `imc_tactical` build without warnings.

---

## Driver bug fixes — needed before specific stages

---

### Issue: Fix Console ISR/main shared state race condition (F3)
**Labels:** `bug` `bringup`

**Description:**

`Console::feed()` is called from the UART RX interrupt context and writes to `m_lineBuf`, `m_lineIdx`, and `m_lineReady`. `Console::poll()` reads and clears those same fields from the main loop. There is no memory barrier or critical section protecting this shared state, which can cause corrupted commands or missed input under load.

**Work required:**
- Wrap the shared fields in a critical section (`__disable_irq()` / `__enable_irq()`, or `taskENTER_CRITICAL()` when building tactical) inside both `feed()` and `poll()`.
- Alternatively, use an atomic flag for `m_lineReady` and only copy `m_lineBuf` + reset `m_lineIdx` under a brief critical section.

**Acceptance criteria:**
- No corrupted commands observed during rapid input (e.g., repeated `help` invocations at maximum baud rate).
- No regression in normal single-command usage.

---

### Issue: Fix Console USB transmit path — remove blocking `HAL_Delay` (F2 / F8)
**Labels:** `bug` `bringup`

**Description:**

The `Console` transmit path calls `HAL_Delay` to wait for USB CDC to become ready, blocking the main loop and preventing `console.poll()` from processing incoming bytes during long output sequences. On a busy console (e.g., IMU stream), this causes input loss and command misses.

**Work required:**
- Replace the blocking `HAL_Delay` loop in `Console::print` / `Console::printf` with a non-blocking approach: check `CDC_Transmit_FS` return code and drop or buffer if busy, or use a small outbound ring buffer drained from the main loop.
- Ensure `console.poll()` is never starved by its own output.

**Acceptance criteria:**
- IMU stream (`test imu stream`) runs at 100 Hz without dropping input commands typed concurrently.
- No `HAL_Delay` calls remain in the Console transmit path.

---

### Issue: Fix CRSF DMA re-arm collision with circular DMA config (N1 / F18)
**Labels:** `bug` `bringup`

**Description:**

`CRSFReceiver` arms DMA receive with `HAL_UARTEx_ReceiveToIdle_DMA`, but the UART4 RX DMA channel in the `.ioc` may be configured in circular mode. The re-arm logic in `onDmaRxEvent` unconditionally restarts the transfer as if it were a one-shot (NORMAL mode) transfer, which causes a collision: either the DMA descriptor is overwritten mid-transfer or the re-arm fails silently.

**Work required:**
- **Option A (preferred):** Reconfigure UART4 RX DMA to NORMAL mode in the `.ioc`. Re-arm by calling `HAL_UARTEx_ReceiveToIdle_DMA` again at the end of `onDmaRxEvent` after copying the received bytes.
- **Option B:** Keep circular DMA and implement a `m_dmaTail` ring index in `CRSFReceiver` to track how much of the circular buffer has been consumed, never re-arming manually.

Address F18 (byte-copy loop after re-arm) in the same PR.

**Acceptance criteria:**
- `test crsf sniff` reliably prints sync bytes continuously without hangs or buffer corruption.
- No DMA error callbacks fire during a 60-second CRSF stream.

---

### Issue: Fix CRSF `uint8_t` arithmetic overflow, missing `volatile`, and unsaved PRIMASK (F19–F21)
**Labels:** `bug` `bringup`

**Description:**

Three related bugs in `CRSFReceiver`:

- **F19 — `uint8_t` arithmetic overflow:** Channel value reconstruction performs arithmetic in `uint8_t` before widening, silently truncating intermediate results. Promote operands to `uint16_t` or `uint32_t` before shifting/ORing.
- **F20 — `m_dmaRxBuf` not `volatile`:** The DMA RX buffer is shared between the DMA hardware and the main loop but is not declared `volatile`, allowing the compiler to cache stale values in registers.
- **F21 — PRIMASK not saved on critical section exit:** If the critical section in `onDmaRxEvent` enters when interrupts are already disabled (nested call), re-enabling them unconditionally with `__enable_irq()` breaks the outer critical section. Use `__get_PRIMASK()` / `__set_PRIMASK()` save-and-restore pattern.

**Acceptance criteria:**
- All 16 CRSF channels decode to correct `[172, 1811]` values for a known transmitter stick position.
- No compiler warnings about `volatile` or implicit conversions in `CRSFReceiver.cpp`.

---

### Issue: Fix `IMU::readBurst` ignoring `len` parameter (F4)
**Labels:** `bug` `bringup`

**Description:**

`IMU::readBurst` allocates a fixed 12-byte dummy transmit buffer for the SPI full-duplex transfer regardless of the `len` argument passed by the caller. If `len > 12`, the SPI peripheral reads garbage TX bytes; if `len < 12`, extra bytes are clocked out unnecessarily.

**Work required:**
- Size the local dummy Tx buffer to `len` (stack-allocated VLA or a fixed max-size array with an assertion `len <= kMaxBurstBytes`).
- Add a `static_assert` or runtime assert guarding the maximum burst size.

**Acceptance criteria:**
- `test imu stream` reads correct accel/gyro values at any supported burst length.
- No buffer overrun or SPI framing errors under ASAN/static analysis.

---

### Issue: Confirm VBAT divider ratio and implement per-board calibration routine (P10)
**Labels:** `bringup` `hardware` `driver`

**Description:**

`Battery.hpp::kDividerRatio` is set to a nominal value, but JLC resistors (5% tolerance) cause meaningful variation per board. The bringup checklist (Stage 6) requires comparing the ADC reading against a calibrated DC supply and storing a per-board correction factor.

**Work required:**
- Measure the actual divider ratio on the bench with a known supply in the 6–9 V range.
- Update `kDividerRatio` to the measured nominal or add a software-trim field.
- Implement a `cal vbat <known_mv>` console command that: reads the raw ADC, computes a scale factor, and writes it to EEPROM (page 0 or a dedicated calibration page).
- `Battery::readMillivolts()` must apply the stored scale factor on startup (read from EEPROM in `Battery::init()` or at first read).
- Verify linearity at a second voltage setpoint before marking Stage 6 as `PASS`.

**Acceptance criteria:**
- `test adc vbat` agrees with a calibrated bench meter to within ±50 mV across the 6–12 V operating range.
- Scale factor survives a power cycle (persisted in EEPROM).

---

### Issue: Add Servo 2 (PA2 / TIM2 CH3) to `.ioc` and `ServoChannel` driver
**Labels:** `bringup` `driver` `missing-feature`

**Description:**

Servo 2 was added to the PCB schematic on PA2 (TIM2 CH3), but the `.ioc` still assigns PA2 to `ADC1_INP14` and the `ServoChannel` driver is not instantiated for channel 3. Stage 9 (servo validation) requires all three servo outputs.

**Work required:**
- In CubeMX, reassign PA2 from `ADC1_INP14` to `TIM2_CH3`; regenerate HAL output into `cubemx/`.
- Instantiate a third `ServoChannel` for `TIM2_CH3` in `main_bringup.cpp`.
- Update `test servo` command to accept a channel argument `0`/`1`/`2` and sweep all three.
- Update the pin map in `docs/readme.md` and `docs/integrated_motor_controller_firmware_arch.md`.
- **Note:** PA2 was previously `ADC1_INP14_MOTOR_0_IPROPI` in older docs — confirm motor 0 current sense has moved to PA4 (`ADC1_INP18`) in the current `.ioc` before removing the ADC assignment.

**Acceptance criteria:**
- `test servo 2 1500` moves a servo connected to PA2.
- Both `imc_bringup` and `imc_tactical` build without warnings after the `.ioc` regeneration.

---

### Issue: Protect 64-bit encoder accumulator from non-atomic reads on Cortex-M33 (F7)
**Labels:** `bug` `tactical`

**Description:**

`Encoder::count()` returns an `int64_t` that is incremented in the input capture ISR. On Cortex-M33, a 64-bit read/write is not atomic — a context switch between the high and low word loads can return a torn value. This is benign in bare-metal bringup (ISR and main loop share the bus without preemption from another task), but will cause intermittent position errors in the FreeRTOS tactical firmware.

**Work required:**
- For **tactical**: wrap all 64-bit counter accesses in `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` inside `Encoder::count()` and `Encoder::resetCount()`.
- For **bringup**: use `__disable_irq()` / `__enable_irq()` around the 64-bit read, or switch to a 32-bit counter with overflow tracking (document the decision).
- Alternatively, expose `count()` via a `uint32_t` snapshot that is updated atomically each `update()` call.

**Acceptance criteria:**
- No torn encoder reads observed under stress (rapid encoder pulses + concurrent console output).
- `uxTaskGetStackHighWaterMark()` shows no unexpected stack consumption from the critical section overhead.

---

## Bringup stages — one issue per stage

---

### Issue: Bringup Stage 0 — Power-on smoke test (no firmware)
**Labels:** `bringup` `hardware`

**Description:**

Validate the bare board before any firmware is flashed.

**Checklist:**
- [ ] Apply bench power at nominal operating voltage.
- [ ] Verify 5 V rail within ±5%.
- [ ] Verify 3.3 V rail within ±5%.
- [ ] Measure idle current < 50 mA with all motor drivers disabled.
- [ ] No components visually hot after 30 seconds at idle.

**Pass criteria:** All rail voltages nominal; idle current within spec.
Record measured values in a comment before closing.

---

### Issue: Bringup Stage 1 — SWD connection and blinky
**Labels:** `bringup`

**Description:**

Validate that the toolchain, ST-Link, and clock tree are all functional.

**Prerequisites:** Board passed Stage 0.

**Checklist:**
- [ ] Connect ST-Link to SWD header J1.
- [ ] `scripts/flash.sh` completes without error (OpenOCD reports `verified`).
- [ ] `DEBUG_LED_0` (PC15) blinks at ~1 Hz.
- [ ] Confirm blink rate with a stopwatch or oscilloscope (validates 250 MHz PLL, HAL tick at 1 ms).

**Pass criteria:** LED blinks at 1 Hz ± 10%.

---

### Issue: Bringup Stage 2 — USB-CDC console enumeration
**Labels:** `bringup`

**Description:**

Validate USB-CDC enumeration and console communication.

**Prerequisites:** Stage 1 passed. Issue "Wire ADC DMA start and USB-CDC Console integration" resolved.

**Checklist:**
- [ ] Connect USB-C to J6 with bringup firmware running.
- [ ] Board enumerates as a CDC serial device (appears as `COMx` / `ttyACM0` / `tty.usbmodem*`).
- [ ] Open a terminal at 115200 baud; confirm banner: `IMC bring-up console ready. Type 'help' for commands.`
- [ ] `help` lists all registered commands.

**Pass criteria:** Console responds to `help` with a full command list.

---

### Issue: Bringup Stage 3 — LEDs, user button, and buzzer
**Labels:** `bringup`

**Description:**

Validate all debug GPIO outputs and the user button interrupt.

**Prerequisites:** Stage 2 passed. Buzzer driver issue resolved.

**Checklist:**
- [ ] `test leds` — observe all three debug LEDs cycling: PC15 (LED0), PC0 (LED1), PC1 (LED2). Console prints `PASS`.
- [ ] `test button` — press PC13 button; console prints interrupt confirmation and `PASS`.
- [ ] `test buzzer` — audible pulses from PC14; console prints `PASS`.

**Pass criteria:** All three commands print `PASS`.

---

### Issue: Bringup Stage 4 — EEPROM read/write (I²C1)
**Labels:** `bringup`

**Description:**

Validate M24C64 EEPROM on I²C1 (PB6/PB7), address `0x50`.

**Prerequisites:** Stage 2 passed.

**Checklist:**
- [ ] `test eeprom read 0` → `0xFF` (erased byte).
- [ ] `test eeprom write 0 42` → `OK`.
- [ ] `test eeprom read 0` → `42` (round-trip).
- [ ] Power cycle; `test eeprom read 0` → `42` (persistence confirmed).
- [ ] `test eeprom write 0 255` → restore erased state.

**Pass criteria:** Round-trip read/write succeeds; data persists across power cycle.

---

### Issue: Bringup Stage 5 — IMU WHO\_AM\_I and live data (SPI1)
**Labels:** `bringup`

**Description:**

Validate ICM-42688-P on SPI1 (PB3/PB4/PB5, CS=PC12).

**Prerequisites:** Stage 2 passed. F4 (`IMU::readBurst` buffer fix) resolved.

**Checklist:**
- [ ] `test imu whoami` → `0x47`. Console prints `PASS`.
- [ ] `test imu stream` — 100 Hz output. With board flat and level:
  - [ ] `accelZ ≈ +9.81 m/s²`
  - [ ] `accelX ≈ 0`, `accelY ≈ 0`
  - [ ] All gyro axes ≈ 0 rad/s (< 0.05 rad/s noise)
- [ ] Rotate the board; confirm accel vector magnitude stays ≈ 9.81 m/s² and axis signs track motion.

**Pass criteria:** WHO_AM_I matches; accel/gyro values physically plausible across orientations.

---

### Issue: Bringup Stage 6 — ADC and VBAT calibration
**Labels:** `bringup`

**Description:**

Validate ADC1 DMA scan and perform per-board VBAT calibration.

**Prerequisites:** Stage 2 passed. "Wire ADC DMA start" and "VBAT calibration" issues resolved.

**Checklist:**
- [ ] `test adc vbat` — compare to calibrated DC supply; initial reading within ±500 mV (pre-cal).
- [ ] `cal vbat <known_mv>` — store calibration scale factor in EEPROM.
- [ ] `test adc vbat` — post-cal reading within ±50 mV of bench meter.
- [ ] Verify linearity at a second setpoint (e.g., 8.0 V if calibrated at 7.4 V).
- [ ] `test adc iprop left` and `test adc iprop right` — near zero with motors off (< 50 mA equivalent).

**Pass criteria:** VBAT calibrated and accurate; IPROPI reads near zero at idle.

---

### Issue: Bringup Stage 7 — Motor drivers (left and right)
**Labels:** `bringup`

**Description:**

Validate both DRV8874 motor channels using PWM (TIM3) and current sense (ADC1 DMA).

**Prerequisites:** Stages 2 and 6 passed. Console commands wired.

**Checklist:**
- [ ] Wake motor 0: `MOTOR_0_ENABLE` (PA5) high.
- [ ] `test motor left forward 25` — motor spins; FAULT pin (PC3) stays high.
- [ ] `test motor left reverse 25` — motor reverses.
- [ ] `test motor left brake` — motor stops quickly.
- [ ] `test motor left coast` — motor freewheels.
- [ ] Repeat all four commands for right motor (FAULT = PC7).
- [ ] `test adc iprop left` / `test adc iprop right` — current reads > 0 mA under load.

**Pass criteria:** Both motors obey all four commands; no fault asserted; current sense reads nonzero under load.

---

### Issue: Bringup Stage 8 — Encoder validation (left and right)
**Labels:** `bringup`

**Description:**

Validate single-channel input capture encoders on TIM4 CH4 (PC2, motor 0) and TIM8 CH1 (PC6, motor 1).

**Prerequisites:** Stage 7 passed. Encoder driver rewrite issue resolved.

**Checklist:**
- [ ] `test encoder left` — spin left wheel by hand; confirm counter changes.
- [ ] `test encoder right` — spin right wheel by hand; confirm counter changes.
- [ ] Spin each wheel forward (matching motor forward direction); confirm count increases.
- [ ] `test encoder left` after resetting — count returns to 0 on reset command.
- [ ] Run motor at known duty cycle; compare encoder count rate to expected RPM (rough sanity check).

**Pass criteria:** Both encoders count edges; direction is consistent with motor forward command.
Document single-channel limitation (no direction sensing without second channel) in the test log.

---

### Issue: Bringup Stage 9 — Servo PWM outputs (all three channels)
**Labels:** `bringup`

**Description:**

Validate 50 Hz servo PWM on TIM2 CH1/CH2/CH3 (PA0, PA1, PA2).

**Prerequisites:** Stage 2 passed. Servo 2 ioc/driver issue resolved.

**Checklist:**
- [ ] Connect a servo to Servo 0 (PA0).
- [ ] `test servo 0 1500` — servo centers.
- [ ] `test servo 0 1000` — servo moves to one extreme.
- [ ] `test servo 0 2000` — servo moves to other extreme.
- [ ] Repeat for Servo 1 (PA1) and Servo 2 (PA2).
- [ ] Measure pulse width with oscilloscope or servo tester; verify 1000/1500/2000 µs ± 5 µs.

**Pass criteria:** All three channels produce correct 50 Hz PWM across the full [1000, 2000] µs range.

---

### Issue: Bringup Stage 10 — ELRS/CRSF radio receive
**Labels:** `bringup`

**Description:**

Validate CRSF framing over UART4 (PC10/PC11) at 420 kbaud from an ELRS receiver.

**Prerequisites:** Stage 2 passed. CRSF DMA re-arm and F19–F21 fixes resolved.

**Checklist:**
- [ ] Bind ELRS receiver to a transmitter and power it.
- [ ] `test crsf sniff` — console prints `0xC8` sync bytes continuously.
- [ ] With transmitter on, verify `channelUs(0)` (roll) tracks stick movement from ~1000 to ~2000 µs.
- [ ] Verify `lastFrameAgeMs()` stays < 100 ms with link active.
- [ ] Power off transmitter; verify `lastFrameAgeMs()` climbs (failsafe detection).

**Pass criteria:** CRSF frames decode with correct channel values; failsafe age counter increments when link is lost.

---

### Issue: Bringup Stage 11 — Aux UART loopback (USART1 and UART5)
**Labels:** `bringup`

**Description:**

Validate growth header UARTs with a loopback jumper.

**Prerequisites:** Stage 2 passed.

**Checklist:**
- [ ] Install loopback jumper: USART1 TX (PB14) → USART1 RX (PB15).
- [ ] `test uart config loopback` — TX byte echoed back; console prints `PASS`.
- [ ] Remove USART1 jumper; install UART5 loopback: TX (PB13) → RX (PB12).
- [ ] `test uart aux loopback` — TX byte echoed back; console prints `PASS`.

**Pass criteria:** Both UART loopback tests print `PASS`.

---

### Issue: Tag `v0.1.0-bringup` and archive checklist results
**Labels:** `bringup` `release`

**Description:**

Once all 12 bringup stages (0–11) pass on hardware, create the release tag and archive the results.

**Checklist:**
- [ ] All Stage 0–11 issues are closed with `PASS` recorded in comments.
- [ ] Board serial number and test date noted.
- [ ] Create annotated git tag: `git tag -a v0.1.0-bringup -m "Rev A board SN-<serial> bringup complete <date>"`.
- [ ] Push tag: `git push origin v0.1.0-bringup`.
- [ ] Create a GitHub Release from the tag; attach the bringup console log (copied from serial terminal).

**Pass criteria:** Tag pushed; Release created with serial number and date.

---

## Tactical firmware

---

### Issue: Complete `main_tactical.cpp` peripheral initialization
**Labels:** `tactical` `infrastructure`

**Description:**

`main_tactical.cpp` currently only calls `MX_GPIO_Init()` before `vTaskStartScheduler()`. All peripherals used by tactical tasks must be initialized before the scheduler starts, since tasks may access drivers immediately on their first tick.

**Work required (before `vTaskStartScheduler()`):**
- `MX_GPDMA1_Init()`
- `MX_ADC1_Init()` + `HAL_ADCEx_Calibration_Start` + `HAL_ADC_Start_DMA`
- `MX_UART4_UART_Init()` + `CRSFReceiver::init()`
- `MX_SPI1_Init()`
- `MX_I2C1_Init()`
- `MX_TIM2/3/4/8_Init()`
- `MX_USB_PCD_Init()` (if USB console is used in tactical)

**Acceptance criteria:**
- All peripheral init calls present in `main_tactical.cpp` in the same order as `main_bringup.cpp`.
- `imc_tactical` boots to scheduler without Hard Fault.
- `Heartbeat` task toggles `DEBUG_LED_0` at 500 ms as observed on hardware.

---

### Issue: Define tactical task inventory — control loop, CRSF receive, telemetry
**Labels:** `tactical` `infrastructure`

**Description:**

The tactical firmware currently runs only the `Heartbeat` task (priority 1). The full task inventory needs to be designed and implemented before competition use.

**Proposed tasks:**

| Task | Priority | Period | Responsibility |
|---|---|---|---|
| `Heartbeat` | 1 | 500 ms | LED toggle (already exists) |
| `CRSFReceive` | 4 | Event-driven | Arm DMA; process CRSF frames; post channel data to a queue |
| `ControlLoop` | 3 | 10 ms (100 Hz) | Read CRSF channels; compute motor commands; call `drivetrain.arcade()` |
| `Telemetry` | 2 | 100 ms | Log IMU + encoder + battery data over USB-CDC or USART1 |

**Work required:**
- Define task functions and stacks in `main_tactical.cpp`.
- Assign priorities per the Section 3.2 interrupt priority policy.
- Pass shared driver references via task parameter structs (no global state).
- Add stack size + watermark constants; validate with `uxTaskGetStackHighWaterMark()`.

**Acceptance criteria:**
- All four tasks start and run without stack overflow or priority inversion.
- `uxTaskGetStackHighWaterMark()` shows ≥ 20% headroom on each task.

---

### Issue: Set FreeRTOS ISR priorities for all shared peripherals
**Labels:** `tactical` `bug`

**Description:**

ISRs that call FreeRTOS `FromISR` APIs must use NVIC priority 5–14 (per `FreeRTOSConfig.h` and Section 3.2). This is not yet enforced for peripherals shared between bringup and tactical. Incorrect priority assignment causes `configASSERT` to fire in `xQueueSendFromISR` / `xTaskNotifyFromISR`.

**Work required:**
- Audit all `HAL_NVIC_SetPriority()` calls in CubeMX-generated `gpio.c`, `gpdma.c`, `usart.c`, `spi.c`, and `adc.c`.
- For tactical, override any priority < 5 for ISRs that will call FreeRTOS APIs (DMA complete, UART idle, IMU INT1 EXTI).
- Add a comment beside each `HAL_NVIC_SetPriority` call documenting whether it calls FreeRTOS APIs.
- USB IRQ: priority 5. UART4 DMA: priority 6. IMU INT1 (EXTI2/PD2): priority 6. Motor FAULT EXTIs: priority 5 (fastest response).

**Acceptance criteria:**
- No `configASSERT` fires during a 60-second run with all peripherals active.
- `stm32h5xx_it_tactical.c` priorities documented and reviewed.

---

### Issue: Validate and right-size FreeRTOS heap — profile stack watermarks
**Labels:** `tactical` `performance`

**Description:**

`configTOTAL_HEAP_SIZE` is set to 64 KB (conservative). Once all tactical tasks exist, actual heap and stack usage must be measured to confirm headroom and avoid over-allocating from the 640 KB SRAM budget.

**Work required:**
- Add a `stats` console command (or periodic `Telemetry` log entry) that calls `xPortGetFreeHeapSize()` and `uxTaskGetStackHighWaterMark()` for each task.
- Run a 60-second stress test with all tasks active.
- If any task watermark < 20%, increase its stack size.
- If heap free > 2× peak usage, consider reducing `configTOTAL_HEAP_SIZE` to free SRAM for driver buffers.
- Document final values in a comment in `FreeRTOSConfig.h`.

**Acceptance criteria:**
- All tasks show ≥ 20% stack headroom after a 60-second run.
- Heap never reaches < 4 KB free.
- Final stack sizes and heap size committed with a comment justifying the values.

---

## CI

---

### Issue: Set up GitHub Actions CI — matrix build for both targets
**Labels:** `ci` `infrastructure`

**Description:**

No CI pipeline exists. Every push should verify that both `imc_bringup` and `imc_tactical` compile cleanly.

**Work required:**
- Add `.github/workflows/build.yml` with a matrix job:
  - `ubuntu-latest`
  - Toolchain: `carlosperate/arm-none-eabi-gcc-action@v1` (pinned to gcc 13.x)
  - Steps: checkout → install toolchain → `cmake -B build -G Ninja` → `cmake --build build`
  - Build both targets in the matrix or sequentially in one job.
- Add a status badge to `README.md`.
- (Optional) Add `cppcheck` as a second job using the existing `build/cppcheck.txt` config.

**Acceptance criteria:**
- Green CI badge on `master`.
- PR checks block merge on build failure.
- Both `imc_bringup.elf` and `imc_tactical.elf` appear as build artifacts (uploaded via `actions/upload-artifact`).
