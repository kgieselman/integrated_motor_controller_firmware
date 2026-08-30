# Integrated Motor Controller — Tactical Firmware Architecture

**`imc_tactical` · Rev A · STM32H563RIT6**

| Field | Value |
|---|---|
| Status | Foundation complete and compiling; application layers (3-7) not yet written. |
| Target | `imc_tactical` — the only application target. `app/bringup/` was removed 2026-08-30 and will be reworked later. |
| Kernel | FreeRTOS V11.1.0, `GCC_ARM_CM33_NTZ_NONSECURE`, 1 kHz tick |
| Control rate | 200 Hz (`dt = 0.005 f`) |
| Language | C++20, `-fno-exceptions -fno-rtti`, no heap allocation after boot (§2.2) |
| Companion | [`integrated_motor_controller_firmware_arch.md`](integrated_motor_controller_firmware_arch.md) — hardware map, driver interfaces, boot sequence |

---

## 1. Purpose

The tactical firmware runs an employee-competition robot: it drives a track and completes objectives that
change every year. Last year that meant collecting ping-pong balls at one end of the track, depositing
them at the other, and launching balls at a target.

The architecture is therefore optimised for one property above all others: **a new mechanism must be
additive**. Adding a launcher, a scoop, or whatever next year's objective demands should be a new class
plus one line in a configuration file — never a change to the scheduler, the failsafe logic, or the
drivetrain.

A second, explicit priority: **readability over efficiency**. The board is a 250 MHz Cortex-M33 with an
FPU and 640 KB of SRAM running a control loop at 200 Hz. There is no cycle budget worth defending at the
cost of clarity. Virtual dispatch, whole-struct copies under critical sections, and floating-point
everywhere are all deliberate choices in favour of code that reads plainly.

### 1.1 What this document adds

`integrated_motor_controller_firmware_arch.md` describes layers 1–2 (HAL and drivers), which are built
and compiling. This document specifies layers 3–7, which are not yet written.

---

## 2. Layer Model

Dependencies flow strictly downward. No layer calls upward.

| # | Layer | Location | Contents | Status |
|---|---|---|---|---|
| 7 | Robot config | `app/tactical/config/` | One file per machine per year. Declares which subsystems exist, constructs them with their drivers, lists their gains. | New |
| 6 | Behavior | `app/tactical/behavior/` | Teleop stick mapping; autonomous routines as ordered step lists. | New |
| 5 | Subsystem | `app/tactical/subsystems/` | One class per mechanism, all implementing the same five methods. Each owns its actuators exclusively. | New |
| 4 | Control | `app/tactical/control/` | PID, slew limiting, expo, filters, velocity and heading estimation. Pure math, no HAL types. | New |
| 3 | Platform | `app/tactical/platform/` | Task set, `Snapshot<T>`, `SafetyMonitor`, `InputSource`, `ParamStore`, `Telemetry`. | New |
| 2 | Driver | `drivers/` | `Motor`, `Encoder`, `IMU`, `EEPROM`, `CRSFReceiver`, `ServoChannel`, `Battery`, `AdcDma`. | Exists |
| 1 | HAL / BSP | `cubemx/` | STM32Cube HAL and CMSIS, CubeMX-generated. | Exists |

### 2.1 Invariants

These six rules are what make the design modular. A change that breaks one of them is a design error,
not a reason to relax the rule.

1. **Dependencies flow one way.** A subsystem may use `control/` and `drivers/`. Only `behavior/` and the
   robot config may name a subsystem. Nothing calls upward, ever.
2. **One writer per actuator.** A `Motor` or `ServoChannel` belongs to exactly one subsystem for the life
   of the program. Two pieces of code commanding one actuator is the bug class this rule deletes.
3. **No HAL types above layer 2.** Control and subsystem code takes `float` and plain structs, never
   `TIM_HandleTypeDef*`. This is what makes the interesting half of the codebase compile and unit-test on
   a host.
4. **Every subsystem is the same shape.** Five methods, no exceptions. Adding a mechanism never edits the
   scheduler, the safety monitor, or the telemetry formatter.
5. **Config picks; it never computes.** The robot config constructs objects and lists them. An `if`
   statement in that file means logic that belongs in a subsystem or a behavior.
6. **Spend cycles on clarity.** The style guide bans virtual functions in *performance-critical paths*.
   The subsystem dispatch loop — eight indirect calls per 5 ms cycle — is not one.

### 2.2 Language standard — C++20

The repo builds as **C++20** (`CMakeLists.txt`, and `style_guide.md` §1 carries the full rule),
adopted 2026-08-30 for two features and nothing else. `Dockerfile` is `ubuntu:24.04`, which ships
arm-none-eabi-gcc 13.2 — complete C++20 core language support and effectively complete library support.

**Adopted:**

| Feature | Why it earns its place here |
|---|---|
| **Designated initializers** | The `config/` layer is nothing but construction, and a config file's entire job is to be readable. `PidGains{.kp = 1.2f, .ki = 0.0f, .kd = 0.05f}` versus `PidGains{1.2f, 0.0f, 0.05f}` is the difference between a gain table you can review and one you have to count positions in. Not available in C++17 with `CMAKE_CXX_EXTENSIONS OFF`. |
| **`std::span`** | Replaces every `(pointer, length)` pair in the driver layer. `bool readBurst(std::span<uint8_t> out)` cannot silently ignore its length argument, which is exactly open issue **F4** on `IMU::readBurst`. Zero overhead, no allocation, works directly with the fixed arrays the style guide already requires. |

**Also fine to use where it reads better:** `<bit>` (`std::bit_cast`, `std::countl_zero`) in place of the
`reinterpret_cast` and `memcpy` patterns in CRSF frame decoding; `constinit` on statically allocated
objects; `using enum` in switch-heavy state machines.

**Deliberately avoided, standard notwithstanding:**

| Feature | Why not |
|---|---|
| **Coroutines** | Frames heap-allocate by default, which is straight against the no-heap-after-boot rule. The custom-allocator escape hatch costs more clarity than the plain state machine it would replace. |
| **`<ranges>`, `std::format`** | Real code size on newlib-nano. The telemetry path wants a `snprintf`-style formatter, not `std::format`. |
| **Modules** | Not realistically usable with this CMake and toolchain setup. |
| **C++23** | `std::expected` would genuinely improve the `bool init()` convention — a failure that carries a reason. But GCC 13 does not have it, so adopting C++23 means pinning to a newer toolchain than the base image provides, for one feature. Not worth the CI fragility. Revisit when the base image moves. |

**Concepts** are available and worth a `requires std::is_trivially_copyable_v<T>` on `Snapshot<T>`, but a
`static_assert` reads about as well — not a reason to move on its own.

---

## 3. Runtime Model

### 3.1 Tasks

`configMAX_PRIORITIES = 7`, so priorities run 0–6. The FreeRTOS timer daemon already holds 6.

| Task | Prio | Trigger | Stack | Responsibility |
|---|---|---|---|---|
| Timer daemon | 6 | kernel | 256 w | FreeRTOS built-in. Buzzer chirps, LED patterns. |
| `Comms` | 5 | task notification from UART4 DMA/IDLE ISR | 512 w | Parse CRSF frames, decode 16 channels, apply deadband and expo, publish a `DriverInput` snapshot with a timestamp. |
| `Control` | 4 | `vTaskDelayUntil`, 5 ms | 1024 w | The control cycle (§4). Owns every actuator on the board. |
| `Telemetry` | 2 | `vTaskDelay`, 50 ms | 768 w | Drain the event ring and state snapshot to USB-CDC; host the parameter console. |
| `Heartbeat` | 1 | `vTaskDelay`, 100 ms | 256 w | Mode-coded LED blink, buzzer, and IWDG refresh gated on the control task's liveness counter. |
| Idle | 0 | kernel | 128 w | `__WFI()`, statically allocated. |

The entire robot runs inside one 200 Hz task. Everything else exists to feed it or to watch it.

### 3.2 Cross-task data

Exactly two mechanisms, and nothing else:

| Mechanism | Used for |
|---|---|
| `Snapshot<T>` | A plain struct copied in and out inside `taskENTER_CRITICAL()`. `DriverInput` (Comms → Control) and `RobotState` (Control → Telemetry, Heartbeat). |
| `EventRing` | Fixed-size single-producer/single-consumer ring of small event records. Control pushes, Telemetry drains, neither ever blocks. |

```cpp
/*******************************************************************************
 * @brief Single-writer, multiple-reader value shared between FreeRTOS tasks.
 *
 * The value is copied whole under a critical section, so a reader never
 * observes a partially updated struct. Intended for small POD types
 * (tens of bytes); copying 64 bytes at 200 Hz costs a few hundred nanoseconds.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/
template <typename T>
class Snapshot final
{
public:
  Snapshot()
      : m_value()
  {}

  /// @brief Overwrite the stored value. Called by the owning task only.
  /// @param value New value to store.
  void write(const T& value);

  /// @brief Copy out the stored value.
  /// @return The most recently written value.
  T read() const;

private:
  T m_value;  ///< The shared value; guarded by a critical section.
};
```

Some data needs no snapshot at all. `g_adcBuf[]` is written by the ADC1 circular DMA and read directly
by the control task — naturally aligned 16-bit reads are atomic on Cortex-M33. The encoder counters are
read only by the control task, in-cycle, which is also what dissolves open issue **F7** (the 64-bit
accumulator is never touched concurrently). That reasoning should be written down in `Encoder.hpp`, so
that nobody later adds a second reader without noticing what it costs.

### 3.3 Interrupt priorities

Lower numeric value = higher urgency. The kernel cannot mask 0–4, so nothing at those levels may call a
FreeRTOS API. All of these must be set explicitly with `HAL_NVIC_SetPriority()` before
`vTaskStartScheduler()`.

| Prio | Source | Calls FreeRTOS API |
|---|---|---|
| 5 | Motor fault EXTI3 / EXTI7 | Yes — sets a latch, notifies Control |
| 6 | UART4 DMA + IDLE (CRSF) | Yes — `vTaskNotifyGiveFromISR` |
| 7 | IMU INT1 EXTI2 (optional) | Yes |
| 8 | USB_DRD_FS | Yes |
| 15 | SysTick, PendSV | Owned by the FreeRTOS port |

---

## 4. The Control Cycle

One task, one fixed order, every 5 ms. Reading this list tells you what the robot does.

| # | Step | Detail |
|---|---|---|
| 1 | `sense()` | Read every input exactly once into a single `SensorFrame`: encoder counts and velocities, IMU accel and gyro, battery millivolts, both motor currents, both fault pins. Nothing else in the cycle reads hardware. |
| 2 | `input = m_driverInput.read()` | Copy the latest decoded CRSF frame and its age out of its snapshot. Already deadbanded, expo-curved and switch-decoded — the control task sees intent, not channel counts. |
| 3 | `verdict = m_safety.evaluate(...)` | The gate (§5). Returns a mode plus the reason for it, so telemetry can report `"Disabled: link age 312 ms"` rather than going quiet. |
| 4 | `if (verdict != Run)` | `m_manager.disableAll(); publish(); return;` |
| 5 | `m_behavior->update(ctx)` | The active behavior expresses intent on subsystems. It never touches an actuator. |
| 6 | `m_manager.periodic(ctx)` | Walk the subsystem array in the fixed order declared by the robot config. The only place in the firmware where a PWM register changes. |
| 7 | `publish()` | Copy `RobotState` into its snapshot, push events to the ring, record cycle time from the DWT counter, increment the liveness counter. |

```cpp
/*******************************************************************************
 * @brief Everything a behavior or subsystem may know about the current cycle.
 ******************************************************************************/
struct RobotContext
{
  float       dt;       ///< Seconds since the previous cycle. Nominally 0.005f.
  uint32_t    nowMs;    ///< HAL_GetTick() sampled once, at cycle start.
  ControlMode mode;     ///< Disabled / Teleop / Auto / Fault.
  SensorFrame sensors;  ///< Every sensor value read this cycle.
  DriverInput input;    ///< Last decoded CRSF frame, plus its age in ms.
};
```

### 4.1 Why one task rather than one task per subsystem

- **Determinism.** Fixed order and a fixed `dt` mean a PID's derivative term is honest. Nothing preempts
  between sensing and actuating.
- **No locks.** The cycle is single-threaded end to end. Only two snapshots cross a task boundary.
- **Debuggability.** One breakpoint, one stack, the whole robot state for that tick.
- **Headroom.** At 250 MHz a 5 ms slot is 1.25 M cycles. Eight subsystems each running a PID, a filter
  and some float multiplies will not use one percent of it.

Measure the cycle anyway with the DWT cycle counter and publish the number in telemetry — not because an
overrun is expected, but because when the figure starts creeping you want to know which phase caused it.

---

## 5. Modes and Failsafe

Built in phase 0, before anything moves.

### 5.1 State machine

```
Boot ──▶ SelfTest ──(every onInit() true)──▶ Disabled
                   └─(any onInit() false)──▶ Fault

Disabled ──(link OK + enable switch + no fault)──▶ Teleop
Disabled ──(auto switch, from Disabled only)─────▶ Auto

Teleop / Auto ──(enable released, or link lost)──▶ Disabled     [unlatched]
any state     ──(hard fault trigger)────────────▶ Fault         [latched]
Fault         ──(console clear, or power cycle)─▶ Disabled
```

Two tiers, deliberately. **Disabled** is the normal resting state and recovers by itself — the radio
drops for 300 ms behind an obstacle, the robot coasts, the link returns, you keep driving. **Fault** is
latched, because a motor driver reporting overcurrent is something a human should look at before the
robot moves again.

### 5.2 Failsafe triggers

Every one of these is evaluated at step 3 of every control cycle, 200 times per second, before any
behavior code runs.

| Trigger | Detected by | Threshold | Result |
|---|---|---|---|
| RC link loss | `DriverInput::ageMs` | > 250 ms | Disabled. Two CRSF frames at 50 Hz is 40 ms; 250 ms tolerates a burst of dropouts without tolerating an unplugged receiver. |
| Driver disable | CRSF channel 5 low | — | Disabled. A held switch on the transmitter — the driver's own kill switch. |
| Motor driver fault | EXTI3 / EXTI7 latch (DRV8874 `nFAULT`) | any edge | **Fault**, latched. Names the channel in the message. |
| Battery sag | `Battery::readMillivolts()` | below cutoff for 200 ms | **Fault**, motors coast. Debounced — a hard launch dips the rail for milliseconds and that is not a flat pack. Cutoff is a stored parameter. |
| Control overrun | DWT cycle counter | > 4 ms, 10 consecutive | **Fault**. The cycle is no longer keeping its schedule, so nothing derived from `dt` can be trusted. |
| Control task stall | liveness counter, checked by `Heartbeat` | no advance in 50 ms | IWDG is not refreshed → hardware reset. The one path that survives the control task itself wedging. |
| Stack overflow / malloc failure | FreeRTOS hooks | — | Record the task name to EEPROM, then reset. The reason survives and prints on the next boot. |

### 5.3 Safe is a state, not a check

There is no `if (m_enabled)` anywhere in a subsystem. The manager either calls `onPeriodic()` or it calls
`onDisable()`. A subsystem that forgets to check a flag is a robot that keeps driving; a subsystem that
never has to check one cannot make that mistake.

The corollary: `onDisable()` must genuinely command hardware every time it is called, not early-return on
an "already disabled" flag. It runs 200 times a second while the robot sits still, which is fine — and it
is also what makes the state correct again after a brownout scrambles a PWM register.

### 5.4 Indicators

| Signal | Pin | Meaning |
|---|---|---|
| `DEBUG_LED_0` | PC15 | Heartbeat; the blink pattern encodes the mode. Slow = Disabled, fast = Teleop, double = Auto, solid = Fault. |
| `DEBUG_LED_1` | PC0 | Radio link healthy (last frame < 250 ms old). The single most useful light on the board. |
| `DEBUG_LED_2` | PC1 | Fault latched. |
| `DEBUG_BUZZER` | PC14 | One chirp on enable, two on disable, a repeating pattern on fault. |

Driven by the heartbeat task from the state snapshot, so they keep telling the truth with the USB console
unplugged — which, on a competition floor, it always is.

---

## 6. The Subsystem Contract

```cpp
/*******************************************************************************
 * @file Subsystem.hpp
 * @brief Base class for every mechanism on the robot.
 *
 * Every mechanism — the drivetrain, a launcher, a scoop — implements these five
 * methods and nothing else is required of it. The control task, the safety
 * monitor and the telemetry formatter know only this interface.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/
class Subsystem
{
public:
  virtual ~Subsystem() = default;

  /// @brief Name used in telemetry, console output and fault messages.
  /// @return Pointer to a string literal with static storage duration.
  virtual const char* name() const = 0;

  /// @brief One-time hardware setup, run during boot self-test.
  /// @return true on success; false fails the self-test and boots into Fault.
  virtual bool onInit() { return true; }

  /// @brief Run this subsystem's loop for one cycle and write its actuators.
  ///        Called only while enabled. Must never block or allocate.
  /// @param ctx Timing, mode, sensors and driver input for this cycle.
  virtual void onPeriodic(const RobotContext& ctx) = 0;

  /// @brief Put the hardware in a known-safe state.
  /// @note Called on every disable, fault and link loss. Must be safe to call
  ///       repeatedly, and must command hardware rather than set a flag.
  virtual void onDisable() = 0;

  /// @brief Append this subsystem's values to the outgoing telemetry frame.
  /// @param sink Formatter to append named values to.
  virtual void publishTelemetry(TelemetrySink& sink) const { (void)sink; }
};
```

### 6.1 Intent, not actuation

Behaviors call named intent methods — `drive.arcade(fwd, turn)`, `launcher.spinUp(2400.0f)`,
`intake.collect()`. Those methods set targets on member variables and return immediately. The subsystem
acts on those targets during `onPeriodic()`, and it is the only code in the firmware that touches its
motors.

This is what makes teleop and autonomous interchangeable: both speak the same vocabulary, so an auto
routine is a different caller rather than a different control path. It is also what stops a half-written
behavior from leaving a flywheel spinning — the subsystem always gets its cycle, and always gets its
disable.

### 6.2 What a subsystem may and may not do

| May | May not |
|---|---|
| Own drivers, hold state, run PIDs and state machines | Block, delay, or allocate |
| Read `ctx`, read parameters, publish telemetry | Read the radio directly |
| Time-limit its own actions and log a jam | Touch another subsystem's hardware |
| Report readiness through a query method | Decide the robot's mode |

A subsystem that needs to know whether the match has started is asking for something the behavior layer
should have told it.

### 6.3 Same subsystem, open or closed loop

`DriveBase` holds a control mode. `Open` multiplies stick input straight into duty cycle; `Velocity` runs
a PID per side against encoder RPM. Phase 1 ships `Open`; phase 3 flips a stored parameter and the same
teleop code, unchanged, drives a closed loop. The upgrade is a config change, not a rewrite — and you can
flip back in the pits when an encoder comes loose.

### 6.4 Adding a mechanism — the complete diff

Adding a ping-pong-ball launcher:

1. **`subsystems/Launcher.hpp` / `.cpp`** — a class implementing the five methods. Owns one
   `ServoChannel` driving a brushless ESC, one `ServoChannel` driving the feeder, one `PidController`,
   and a four-state machine (`Idle → SpinUp → AtSpeed → Feed`). Its `onDisable()` commands zero throttle
   and retracts the feeder.
2. **`config/Robot2026.hpp`** — construct it as a member, add `&m_launcher` to the subsystem array,
   register its gains and target RPM in the parameter table. Three lines.
3. **`behavior/TeleopBehavior.cpp`** — map a radio channel to `m_launcher.requestShot()`. One line.

Nothing else changes. The safety monitor disables the launcher correctly on link loss without knowing
what a launcher is, because disabling is a method on the contract rather than a case in a switch
statement.

---

## 7. Configuration and Tuning

### 7.1 Compile-time robot composition

Everything that differs between machines lives in one file per machine per year.

```cpp
/*******************************************************************************
 * @file Robot2026.hpp
 * @brief Machine definition for the 2026 competition robot.
 *
 * Construction and a list. No branching, no computation, no control logic.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/
class Robot2026 final
{
public:
  static constexpr uint8_t kSubsystemCount = 3U;

  /// @brief Initialise drivers, then subsystems, in declaration order.
  /// @return true if every driver and subsystem initialised successfully.
  bool build();

  /// @brief The subsystem array, in the order they run each cycle.
  Subsystem* const* subsystems() const { return m_subsystems; }

  /// @brief The teleop behavior bound to this robot's subsystems.
  TeleopBehavior& teleop() { return m_teleop; }

private:
  // Drivers — HAL handles injected exactly as bringup constructs them.
  Motor        m_leftMotor   { &htim3, TIM_CHANNEL_2, TIM_CHANNEL_1, /* … */ kSlotLeftIpropi  };
  Motor        m_rightMotor  { &htim3, TIM_CHANNEL_4, TIM_CHANNEL_3, /* … */ kSlotRightIpropi };
  Encoder      m_leftEnc     { &htim4, kCountsPerRev, kControlRateHz };
  Encoder      m_rightEnc    { &htim8, kCountsPerRev, kControlRateHz };
  ServoChannel m_flywheelEsc { &htim2, TIM_CHANNEL_1 };
  ServoChannel m_feeder      { &htim2, TIM_CHANNEL_2 };
  ServoChannel m_scoop       { &htim2, TIM_CHANNEL_3 };

  // Subsystems — each owns its drivers; nothing else touches them.
  DriveBase    m_drive       { m_leftMotor, m_rightMotor, m_leftEnc, m_rightEnc };
  Intake       m_intake      { m_scoop };
  Launcher     m_launcher    { m_flywheelEsc, m_feeder };

  Subsystem* m_subsystems[kSubsystemCount] = { &m_drive, &m_intake, &m_launcher };

  TeleopBehavior m_teleop { m_drive, m_intake, m_launcher };
};
```

Two CMake knobs:

| Option | Effect |
|---|---|
| `-DIMC_ROBOT=2026` | Selects the active machine. Last year's config keeps building, so the day someone breaks the subsystem contract, `Robot2025` fails to compile and says so — a free regression test for the price of one variable. |
| `-DIMC_HOST_TESTS=ON` | Builds `control/` and subsystem logic for the host against a stub HAL. A PID's anti-windup behaviour deserves a unit test; proving it on a bench with a real battery does not. |

### 7.2 Runtime parameters

`ParamStore` is a static table of named tunables. Each entry carries a name, a type, a pointer to the
live value, min/max clamps, a default, and an offset into the EEPROM block.

- Console commands over USB-CDC: `param list`, `param get <name>`, `param set <name> <value>`,
  `param save`, `param load`, `param default`.
- Persisted to the M24C64 as a versioned, CRC'd block. A version mismatch or bad CRC loads defaults and
  says so on the console rather than applying garbage gains.
- The control task reads parameters at the top of each cycle. Writes come from the telemetry task; the
  values are small scalars, so a torn read is not possible in practice, and the clamps are applied at
  write time.

This is what phases 3 onward depend on. Tuning a PID by editing a `constexpr` and reflashing works for
about the first four iterations.

---

## 8. Source Layout

### 8.1 Convention: headers beside their sources

The repo uses **co-located headers and sources** everywhere hand-written code lives — `drivers/` and
`app/`. The `include/` + `src/` split was removed on 2026-08-30. `cubemx/` is CubeMX-owned and untouched.

The reasoning:

- **`include/` + `src/` encodes a public-vs-private boundary.** In `drivers/` there is no private half:
  all nine headers are public API, consumed by both applications. The split is separating public from
  private where nothing is private — ceremony with an empty second half.
- **In `app/tactical/` the directory names are the architecture.** `platform/ → control/ → subsystems/ →
  behavior/ → config/` is the design story. Co-located, `subsystems/` *is* the subsystem layer. Split, the
  architecture appears twice, mirrored, and neither half is the layer.
- **Much of the new code is header-only anyway.** `Snapshot.hpp` is a template and must be; `Subsystem.hpp`,
  `RobotContext.hpp` and `Behavior.hpp` are pure interface; roughly five of the eight files in `control/`
  are pure functions with no `.cpp` at all. A mirrored tree two-thirds empty on one side expresses a
  boundary that is not there.
- **The split never enforced the boundary that matters.** Invariant 1 (§2.1, dependencies flow one way) is
  not helped by `include/` — a subsystem including a behavior header is equally legal either way. Real
  enforcement comes from one CMake library per layer with explicit `target_link_libraries`, which works
  identically under any file layout. See §8.3.
- **The build does not care.** Sources are listed explicitly rather than globbed, and headers are never
  listed at all, so the layout only affects `target_include_directories` — one line either way.

`FreeRTOSConfig.h` is the deliberate exception and stays at `app/tactical/include/FreeRTOSConfig.h`. It is
not an application header; it is a build-system artifact that the kernel's `freertos_config` interface
target points at, and giving it its own directory keeps that distinction visible.

### 8.2 Repository layout

```
drivers/                         # flat since 2026-08-30 (was include/ + src/)
├── AdcDma.hpp / .cpp            # shared ADC1 DMA buffer + adcDmaStart()
├── Battery.hpp / .cpp
├── Buzzer.hpp / .cpp            # AT-0927-TT-6-R passive transducer, 2730 Hz
├── Console.hpp / .cpp           # line editor + command dispatch (was app/bringup/)
├── CRSFReceiver.hpp / .cpp      # circular DMA ring, parsed in task context
├── EEPROM.hpp / .cpp
├── Encoder.hpp / .cpp           # input capture, interval-based velocity
├── IMU.hpp / .cpp
├── Led.hpp / .cpp
├── Motor.hpp / .cpp
└── ServoChannel.hpp / .cpp

app/tactical/
├── include/FreeRTOSConfig.h     # build artifact, not an application header
├── main_tactical.cpp            # HAL + peripheral init -> build robot -> tasks -> scheduler
├── freertos_hooks.c             # tick / idle / stack-overflow hooks, static task memory
├── stm32h5xx_it_tactical.c      # IRQ table; FreeRTOS port owns SVC/PendSV/SysTick
├── tasks/
│   ├── ControlTask.cpp          # the 200 Hz cycle
│   ├── CommsTask.cpp            # CRSF frames -> DriverInput snapshot
│   ├── TelemetryTask.cpp        # console out, parameter console in
│   └── HeartbeatTask.cpp        # LEDs, buzzer, IWDG refresh
├── platform/
│   ├── Snapshot.hpp             # template<T>, copy in/out under a critical section
│   ├── RobotContext.hpp         # dt, nowMs, mode, SensorFrame, DriverInput
│   ├── SafetyMonitor.hpp/.cpp   # the failsafe table, evaluated every cycle
│   ├── InputSource.hpp/.cpp     # CRSF channels -> deadband, expo, switch decode
│   ├── ParamStore.hpp/.cpp      # named tunables, EEPROM persist, CRC + version
│   └── Telemetry.hpp/.cpp       # sink, SPSC event ring, text formatter
├── control/                     # pure math - no HAL, builds and tests on a host
│   ├── PidController.hpp/.cpp
│   ├── SlewRateLimiter.hpp
│   ├── ExpoCurve.hpp
│   ├── ArcadeMixer.hpp          # throttle + steering -> left/right, ratio-preserving
│   ├── LowPassFilter.hpp
│   ├── Debouncer.hpp
│   ├── VelocityEstimator.hpp/.cpp
│   └── HeadingEstimator.hpp/.cpp
├── subsystems/
│   ├── Subsystem.hpp            # the contract - five methods
│   ├── SubsystemManager.hpp/.cpp
│   ├── DriveBase.hpp/.cpp       # phase 1 open loop, phase 3 velocity PID
│   ├── Intake.hpp/.cpp          # phase 5
│   └── Launcher.hpp/.cpp        # phase 5
├── behavior/
│   ├── Behavior.hpp             # update(ctx) / isFinished()
│   ├── TeleopBehavior.hpp/.cpp  # the only place stick -> intent mapping lives
│   ├── RoutineRunner.hpp/.cpp   # runs a static array of steps in order
│   └── steps/                   # DriveDistance, TurnToAngle, RunLauncher, Wait
└── config/
    ├── RobotConfig.hpp          # selects the active year from -DIMC_ROBOT
    ├── Robot2026.hpp/.cpp       # this year's machine
    └── Robot2025.hpp/.cpp       # last year's, kept building as a contract check

tests/                           # host build: control/ and subsystem logic, no HAL, no board
```

`app/bringup/` follows the same convention when it is reworked; it is a leaf application too, so the same
reasoning applies. That rework is deliberately deferred until the tactical foundation is settled.

### 8.3 Make `drivers/` a real library target

Worth doing at the same time, because the current arrangement is not a library at all.

Driver `.cpp` files are pasted directly into `APP_Bringup_Src` in `CMakeLists.txt`, and **only three of the
nine are listed** — `EEPROM.cpp`, `IMU.cpp`, `ServoChannel.cpp`. `AdcDma.cpp`, `Battery.cpp`,
`CRSFReceiver.cpp`, `Drivetrain.cpp`, `Encoder.cpp` and `Motor.cpp` exist on disk and are never compiled by
any target, despite `MotorTest.cpp`, `EncoderTest.cpp`, `BatteryTest.cpp` and `CrsfTest.cpp` being in the
build. This is almost certainly why the stale pin references in `Encoder.hpp` and `Drivetrain.hpp` went
unnoticed: **those files have never been through a compiler.**

The fix travels with the move:

```cmake
add_library(imc_drivers STATIC
    ${CMAKE_SOURCE_DIR}/drivers/AdcDma.cpp
    ${CMAKE_SOURCE_DIR}/drivers/Battery.cpp
    # … all nine …
)
target_include_directories(imc_drivers PUBLIC ${CMAKE_SOURCE_DIR}/drivers)
target_link_libraries(imc_drivers PUBLIC STM32_Drivers)

target_link_libraries(imc_bringup  PRIVATE imc_drivers)
target_link_libraries(imc_tactical PRIVATE imc_drivers)   # currently links no drivers at all
```

`target_include_directories(... PUBLIC drivers)` gives consumers exactly the same surface the old
`drivers/include` did, enforced by the target rather than by directory layout. And once every layer is its
own library, invariant 1 becomes a link-time property rather than a review convention:

```cmake
add_library(imc_control  STATIC …)   # links nothing above it
add_library(imc_platform STATIC …)   # links imc_drivers
add_library(imc_subsys   STATIC …)   # links imc_control, imc_platform, imc_drivers
add_library(imc_behavior STATIC …)   # links imc_subsys
```

A subsystem that reaches upward into `behavior/` then fails to link, rather than merely failing review.

---

## 9. Actuator and I/O Budget

The architecture will happily host ten subsystems. The board has two H-bridges and three PWM outputs,
and the drivetrain wants both bridges. This is the constraint that decides what a mechanism can
physically be, and it is worth settling before designing next year's scoop rather than after.

| Resource | Pins | Status | Notes |
|---|---|---|---|
| `MOTOR_0` / `MOTOR_1` (DRV8874 ×2) | TIM3 CH1–4; PA6/PA7, PB0/PB1 | Drivetrain | Brushed DC, bidirectional, per-channel current sense via `g_adcBuf`, independent nSLEEP and PMODE, fault line on EXTI. The only outputs that can tell you a motor is stalling — spend them on the wheels. |
| `SERVO_0/1/2` | TIM2 CH1–3; PA0, PA1, PA2 | Mechanisms | 50 Hz, 1000–2000 µs — a hobby servo *or* a brushless ESC, identical signal, so `ServoChannel` is already an ESC driver. No current sense and no fault line, so a stalled servo is invisible to firmware; time-limit anything that can jam. |
| `ENCODER_0` / `ENCODER_1` | TIM4 CH4 (PC2), TIM8 CH1 (PC6) | Drivetrain | Single-channel input capture: speed only, no direction. `VelocityEstimator` signs the reading from the last commanded direction — correct except during reversal and when the robot is pushed. Enough for velocity PID; not enough to trust for odometry. |
| IMU (ICM-42688-P) | SPI1, INT1 on PD2 | Phase 4 | Yaw rate for heading hold and turn-to-angle. Polling once per cycle at 200 Hz is ample; the INT line stays available. |
| EEPROM (M24C64) | I2C1, `~WC` on PA9 | Phase 2 | 8 KB. Parameter block plus a small crash log — task name and reason from the last fault reset, printed on the next boot. |
| Growth header | I2C3 (PA8/PC9), USART1 (PB14/15), UART5 (PB12/13) | Unclaimed | Where a fourth and fifth actuator comes from: a second IMC board, a smart ESC, or a sensor pod. From firmware it becomes a driver — a `RemoteMotor` speaking a small framed protocol — and a subsystem holding one cannot tell the difference. |

### 9.1 Consequences

A launcher flywheel and a scoop both come off the servo header, as an ESC-driven brushless motor or a
servo. If a mechanism genuinely needs a current-sensed, fault-monitored H-bridge — an intake that will
jam on a ball, an arm that will stall against a hard stop — that is a second board on the growth UART,
not a firmware problem. Either way the subsystem contract is unchanged, which is the payoff for keeping
HAL types out of layer 5.

### 9.2 Two decisions to make before phase 5

- **Does the launcher need closed-loop RPM?** If it shoots at one fixed distance, an open-loop throttle
  number found on the bench is honest and simple. If the shot must stay repeatable as the battery sags
  over a match, it needs feedback — and a single-channel encoder on the flywheel is one more input-capture
  timer that is not currently free. An ESC reporting RPM over the growth UART solves it more cheaply than
  more pins.
- **How does a mechanism report that it is stuck?** Servos cannot. The pattern that works without current
  sense is a timeout inside the subsystem's own state machine: `Feed` must reach `Idle` within N
  milliseconds or it retracts and logs a jam event. Cheap, and it keeps a jammed feeder from cooking a
  servo for the rest of the match.

---

## 10. Build Order

Each phase ends with something demonstrable on hardware, and each adds code without editing the phase
before it.

| Phase | What lands | Done when |
|---|---|---|
| **0 — Spine** | Peripheral init in `main_tactical.cpp`, the four tasks, `Snapshot<T>`, `SafetyMonitor`, `InputSource`, and a `SubsystemManager` with zero subsystems. Nothing moves. | LED_1 tracks the radio link, the mode reaches Disabled within 250 ms of unplugging the receiver, and the cycle holds 200 Hz with zero overruns for ten minutes. |
| **1 — Raw drive (MVP)** | `DriveBase` in open-loop mode owning the two `Motor` and two `Encoder` instances directly (§10.2), `TeleopBehavior` mapping two sticks to arcade drive, `Robot2026` wiring them up. Deadband, expo, slew limit. | Sticks drive the robot, and it coasts to a stop within 250 ms of the transmitter being switched off — tested deliberately, at speed, more than once. |
| **2 — Telemetry and tuning** | `TelemetryTask` over USB-CDC, `ParamStore` with console commands and EEPROM persistence. | You change the drive slew limit from the console, feel the difference, save it, power-cycle, and it is still there. |
| **3 — Closed-loop drive** | Rewrite `Encoder` for input-capture mode. `VelocityEstimator`, `PidController`, `DriveBase` velocity mode selected by a stored parameter. | Commanded RPM holds within 5% while the robot is pushed against a wall, both sides match on a straight line, and the phase-1 teleop code is unchanged. |
| **4 — Heading** | `HeadingEstimator` (complementary filter over gyro and accel), heading hold on straight drive, a `TurnToAngle` primitive. | The robot tracks a straight line across the track with the turn stick centred, and turns 90° to within a few degrees. |
| **5 — Mechanisms** | `Intake` and `Launcher` on the servo/ESC outputs, each with its own state machine and jam timeout. | Adding each is a new class plus one line in the robot config. If it is not, the contract needs fixing before phase 6. |
| **6 — Autonomy** | `RoutineRunner` walking a static array of steps, each with `update(ctx)` and `isFinished()`. A routine is a table, not a function. | One switch runs a collect-and-deposit routine end to end, and the disable switch stops it instantly. |

### 10.1 Foundation work — status

The reset that preceded phase 0 is complete: `app/bringup/` removed, `drivers/` made a real library
target, every driver compiled for the first time, the layout co-located and the build moved to C++20.
`imc_tactical` compiles clean.

**Closed 2026-08-30:**

| Was | Resolution |
|---|---|
| Six of nine drivers in no build target | `add_library(imc_drivers STATIC …)`, all listed, linked into `imc_tactical` (§8.3). This was the root cause of most of the stale headers below. |
| Co-located restructure and C++20 | Done — §8.1 and §2.2. |
| Tactical peripheral init missing | `main_tactical.cpp` now brings up GPIO, GPDMA1, ICACHE, ADC1, SPI1, I²C1/3, TIM2/3/4/8, UART4, USART1 and USB, and starts the ADC scan through `adcDmaStart()`. |
| **N1 / F18** — CRSF DMA re-arm vs. circular config | Confirmed circular in the `.ioc`. Receive path reworked: the ISR records the DMA head position only, `update()` drains the ring from its own tail in task context, and the transfer is never re-armed. |
| **F7** — 64-bit encoder accumulator not atomic | `Encoder` position is now `int32_t`, accumulated and read only in task context. |
| **F19–F21** — CRSF `uint8_t` arithmetic, missing `volatile`, PRIMASK not saved | The accumulator is single-context, so the races are gone; the remaining critical sections save and restore PRIMASK. |
| `Encoder` written for quadrature | Rewritten for single-channel input capture, with velocity measured by edge interval rather than count-per-window. |
| `Motor` does not own its ENABLE pin | Constructor now takes the per-channel nSLEEP; `enable()` / `disable()` added. Fault is latched from EXTI, and `brake()` scales to ARR + 1 for a true low-side short. |
| `Drivetrain` documents ENABLE as PA4 | Removed from the build (§10.2). PA4 is `LEFT_MOTOR_IPROPI` — following that example would have configured an analog input as a push-pull output. |
| IPROPI sense resistor contradiction | Resolved: **1.6 kΩ**, confirmed against the schematic, so `kDefaultRIpropi` was right and the Doxygen was wrong. Scale is 0.8 V/A, so the sense saturates at ≈4.125 A (`Motor::kCurrentFullScaleMa`) — stall detection must read full scale as "at least this much", never as a measurement. |
| Battery reports flat at boot | `adcDmaIsRunning()` now gates it; `isLow()` returns false until there is a real measurement. |

**Still open:**

| Phase | Issue |
|---|---|
| 0 | **ISR priorities.** The policy is applied to USART1 and the pattern is set in `configureInterruptPriorities()`. Extend it to the CRSF UART4 DMA, the motor fault EXTI lines and the IMU INT line as each is wired. |
| 0 | **Nothing has run on hardware.** The FreeRTOS baseline, `Led`, `Buzzer` and the relocated `Console` compile but have never executed. Flash and confirm the heartbeat, the boot chirp and the console banner before building on top of them. |
| 0 | **Heap sizing.** `configTOTAL_HEAP_SIZE` is still an unvalidated 64 KB. Right-size it from `uxTaskGetStackHighWaterMark()` once the task set is real. |
| 2 | **P10** — the VBAT divider ratio (4.333) is unconfirmed. Measure it before the brownout threshold means anything. |
| 2 | **F3** — `Console`'s line buffer is shared between the RX ISR and the console task with no protection. Acceptable while it is diagnostics-only; must be fixed before `ParamStore` trusts it. |
| 3 | **Encoder calibration unknown.** `pulsesPerRev` and `timerClockHz` are caller-supplied, and nothing has verified which encoders are actually fitted. |
| — | **CI exists** (`.github/workflows/ci.yml`: Docker build, Debug all-targets, Release, cppcheck, ELF/map artifacts). Its `imc_bringup` release step was removed 2026-08-30 when that target went away. Worth adding a host-side unit-test job when `control/` lands in phase 3. |

### 10.2 `Drivetrain` and the tactical path

`Drivetrain` currently does three things at once: it owns the motors and encoders, it performs arcade
mixing, and it manages a shared ENABLE pin that this board does not have. The third of those is wrong for
Rev A, and the first conflicts with what `DriveBase` needs in phase 3 — a per-side velocity PID wants the
`Motor` and `Encoder` instances directly, not through a wrapper that only exposes combined commands.

Recommendation: **`DriveBase` owns the two `Motor` and two `Encoder` instances directly**, and the arcade
and tank mixing math moves into `control/ArcadeMixer.hpp` as pure functions — testable on a host, with no
hardware ownership at all. That leaves `Drivetrain` used only by `imc_bringup`, where its combined
commands are exactly what a bring-up test wants; fixing its ENABLE model and stale comments then becomes
a bringup-scoped task rather than a phase-1 blocker.

### 10.3 Deliberately out of scope

- **A full command scheduler** (requiring subsystems, interruptible commands, default commands). Powerful,
  and a lot of machinery to read. Intent methods plus a step list cover a track robot; if a genuine
  arbitration problem appears, it appears at the behavior layer only.
- **Runtime subsystem enable/disable.** Compile-time composition means what runs is what the config says,
  and a call stack is the whole truth.
- **Field-relative odometry.** Single-channel encoders cannot report direction. Distance along a commanded
  heading is honest; a full field pose is not, and pretending otherwise produces autonomous routines that
  fail mysteriously.

---

## 11. Success Criteria

- `imc_bringup` and `imc_tactical` both build without warnings from a single `cmake --build`, and
  `drivers/` is byte-identical between them.
- Adding a mechanism touches exactly three files: the new subsystem pair, the robot config, and the teleop
  mapping.
- Every failsafe trigger in §5.2 has been provoked deliberately on hardware and observed to do what this
  document says.
- `uxTaskGetStackHighWaterMark()` shows ≥ 20% headroom on every task; `configTOTAL_HEAP_SIZE` is
  right-sized from measured watermarks rather than left at the initial 64 KB.
- The control cycle's worst-case duration is published in telemetry and is under 2 ms.
- `control/` and subsystem logic build and pass their tests on a host with no board attached.

---

*Companion to [`integrated_motor_controller_firmware_arch.md`](integrated_motor_controller_firmware_arch.md).
Pin assignments are authoritative in `cubemx/integrated_motor_controller.ioc`. Code conventions are
governed by [`style_guide.md`](style_guide.md).*
