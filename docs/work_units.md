# Tactical Firmware — Work Units

Companion to [`tactical_architecture.md`](tactical_architecture.md), which is the design. This file is
the plan of attack: each unit is sized for one fresh session, with a named set of files to read and a
verification step the agent can run itself.

**To start a unit, the whole prompt is:**

```
Execute work unit U0.3 from docs/work_units.md.
```

Everything the agent needs is below. Pick the model from the dispatch table in §1 before you start.

---

## 0. Agent protocol — read this first

You have been asked to execute a single work unit. Follow this exactly.

**1. Load context, in this order.**
   - This section (§0) and your unit's entry in §3 or §4.
   - The sections of `docs/tactical_architecture.md` your unit names.
   - `docs/style_guide.md`.
   - The files in your unit's **Read** list — **and no others.** Do not explore the codebase. If you
     believe you need a file that is not listed, stop and say which file and why.

**2. Do the work.** Implement exactly what the unit's **Task** describes. Nothing more. Do not add
   features, helpers, or "while I'm here" improvements — a later unit may own that ground.

**3. Stay inside the blast radius.** You may create or modify only the files in your unit's
   **Creates / modifies** list, plus these three standing permissions:
   - `CMakeLists.txt` — to register any new `.cpp` you created. Every new source must go in the
     matching target's source list in the same change; a file in no target is the bug this project
     was restructured to prevent.
   - `app/tactical/main_tactical.cpp` — **one `#include` line only**, if your unit produces headers
     with no `.cpp` and needs them included somewhere to prove they compile.
   - `docs/work_units.md` — your own unit's status cell in §1, set to `Done`, as your last step.

   If you find a bug outside the blast radius, **report it, do not fix it.** Another unit may own that
   file, and a helpful cross-unit edit is the most expensive mistake available here.

**4. Verify.** Run the unit's **Done when** command. If it fails, fix your own work and run it again.
   If it needs hardware you cannot reach, say so plainly in your report — never imply it passed.

**5. Do not commit.** Leave the working tree dirty. The human reviews `git status` against the blast
   radius, then commits.

**6. Report, briefly:**
   - What you created or changed, one line each.
   - The verification command and its actual result.
   - Anything you found outside the blast radius that needs attention.
   - Any decision the unit left ambiguous and how you resolved it.

**Conventions that apply to every unit:** C++20, `-fno-exceptions -fno-rtti`, no heap allocation after
boot, no `std::function`, no heap-backed containers, fixed-size arrays only. Doxygen on every header,
class, public method and member — `docs/style_guide.md` is the authority and it is not optional.

---

## 1. Dispatch table

Pick the model from this column. The rationale is in §2.

| Unit | Model | Status | Depends on | Goal |
|---|---|---|---|---|
| **U0.1** | Stronger | ✅ Done | — | Core types: `RobotContext`, `SensorFrame`, `DriverInput`, `ControlMode` |
| **U0.2** | Stronger | ✅ Done | U0.1 | `Snapshot<T>` — the cross-task value holder |
| **U0.3** | Stronger | ✅ Done | U0.1 | `Subsystem` contract + `SubsystemManager` |
| **U0.4** | Cheaper | Ready | U0.1, U0.2 | `InputSource` — CRSF → `DriverInput` |
| **U0.5** | Stronger | Blocked on U0.4 | U0.1, U0.2, U0.4 | `SafetyMonitor` — the failsafe table |
| **U0.6** | Stronger | Blocked on U0.5 | U0.1–U0.5 | Task set and integration |
| **U0.7** | Cheaper | Blocked on U0.6 | U0.6 | Indicators and watchdog |
| **U1.1** | Stronger | Ready *(parallel with phase 0)* | — | Host test harness |
| **U1.2a** | Cheaper | Blocked on U1.1 | U1.1 | `ExpoCurve` |
| **U1.2b** | Cheaper | Blocked on U1.1 | U1.1 | `SlewRateLimiter` |
| **U1.2c** | Cheaper | Blocked on U1.1 | U1.1 | `Debouncer` |
| **U1.2d** | Cheaper | Blocked on U1.1 | U1.1 | `ArcadeMixer` |
| **U1.3** | Cheaper | Blocked | U0.3, U1.2 | `DriveBase` |
| **U1.4** | Cheaper | Blocked | U1.3 | Teleop behavior + `Robot2026` config |
| **U1.5** | Stronger | Blocked | U1.4 | Integration and bench verification |

Phase 2–6 units are written when the phase starts, so they can take account of what the previous phase
actually taught us.

---

## 2. Choosing a model

**Stronger model** when *any* of these holds:

- The unit **defines an interface** other units consume — a mistake propagates into every consumer.
- It reasons about **concurrency or interrupt context**. These failures are intermittent, and a passing
  test proves nothing.
- Its **failure mode is silent or physical** — a failsafe that doesn't fire, a pin driven the wrong way.
- The spec needs **judgment**: choosing thresholds, resolving a contradiction, picking an abstraction.

**Cheaper model** when *all* of these hold: the interface it works against is frozen and committed, the
spec is unambiguous, and a build or host test catches errors in seconds.

The `control/` primitives are the archetype of the second case. Contracts and safety logic are the first.

---

## 3. Phase 0 — Spine

Nothing moves at the end of this phase. The milestone is that the radio link is visible and the failsafe
demonstrably fires.

### U0.1 — Core types ✅ Done

| | |
|---|---|
| **Model** | Stronger — pure contract work; every later unit consumes these types. |
| **Read** | `docs/tactical_architecture.md` §4; `drivers/Motor.hpp`, `drivers/Encoder.hpp`, `drivers/Battery.hpp` |
| **Creates** | `app/tactical/platform/RobotContext.hpp` |
| **Freezes** | `ControlMode`, `SensorFrame`, `DriverInput`, `RobotContext` |
| **Done when** | `scripts/build.sh --target imc_tactical` succeeds. |

### U0.2 — `Snapshot<T>` ✅ Done

| | |
|---|---|
| **Model** | Stronger — small, but it is the concurrency primitive the whole design rests on, and a torn read fails once a week rather than once a run. |
| **Read** | `docs/tactical_architecture.md` §3.2; `app/tactical/platform/RobotContext.hpp`; `drivers/CRSFReceiver.cpp` (for the PRIMASK pattern in `hasNewData()`) |
| **Creates** | `app/tactical/platform/Snapshot.hpp` |
| **Freezes** | `Snapshot<T>::read()`, `Snapshot<T>::write()` |
| **Done when** | Builds, and a `static_assert` rejects a non-trivially-copyable instantiation. |

**Task.** Header-only template holding one value of type `T`, copied whole in and out under a critical
section so a reader never observes a partial update. Single writer, multiple readers.

Save and restore PRIMASK — `uint32_t p = __get_PRIMASK(); __disable_irq(); … __set_PRIMASK(p);`. Do **not**
call `__enable_irq()` on exit: that unconditionally enables interrupts and silently breaks any outer
critical section the caller may be inside. `CRSFReceiver::hasNewData()` is the reference.

Guard with `static_assert(std::is_trivially_copyable_v<T>)`. Intended for small POD structs of tens of
bytes; say so in the Doxygen along with the reason the copy cost is acceptable.

### U0.3 — Subsystem contract and manager

| | |
|---|---|
| **Model** | Stronger — *the* interface of the whole design; every mechanism, this year and every year after, implements it. |
| **Read** | `docs/tactical_architecture.md` §6; `app/tactical/platform/RobotContext.hpp` |
| **Creates** | `app/tactical/subsystems/Subsystem.hpp`, `app/tactical/subsystems/SubsystemManager.hpp/.cpp` |
| **Freezes** | The five-method `Subsystem` contract |
| **Done when** | Builds. Add a throwaway subsystem in the manager's own test path if you need one to prove it compiles, then remove it. |

**Task.** `Subsystem` is an abstract base with exactly five methods — `name()`, `onInit()`,
`onPeriodic(const RobotContext&)`, `onDisable()`, `publishTelemetry()`. Signatures are given verbatim in
§6 of the architecture doc; use them as written. Virtual dispatch is correct here and is a deliberate
choice — see invariant 6.

`SubsystemManager` holds `Subsystem* const*` plus a count. No ownership, no allocation, no container.
`periodic(ctx)` walks the array in order; `disableAll()` calls `onDisable()` on **every** subsystem
unconditionally — never track which ones you think are enabled and skip the rest.

`TelemetrySink` does not exist yet (phase 2). Forward-declare it; do not invent it.

### U0.4 — `InputSource`

| | |
|---|---|
| **Model** | Cheaper — mechanical mapping against two frozen interfaces, fully specified. |
| **Read** | `drivers/CRSFReceiver.hpp`; `app/tactical/platform/RobotContext.hpp`; `app/tactical/platform/Snapshot.hpp` |
| **Creates** | `app/tactical/platform/InputSource.hpp/.cpp` |
| **Done when** | `scripts/build.sh --target imc_tactical` succeeds. |

**Task.** Turn raw CRSF channels into a `DriverInput` and publish it into a `Snapshot<DriverInput>`.

- Deadband around centre, then an expo curve on the stick axes, so the control task sees intent rather
  than channel counts.
- **CRSF channel 5 is the enable switch** (agreed default).
- Stamp `ageMs` from `CRSFReceiver::lastFrameAgeMs()` at publish time.
- Deadband width and expo factor are `static constexpr` for now; phase 2 moves them into `ParamStore`.

Take the `CRSFReceiver` by reference in the constructor. Do not call HAL directly.

### U0.5 — `SafetyMonitor`

| | |
|---|---|
| **Model** | Stronger — safety logic with judgment calls, and the failure mode is a robot that does not stop. |
| **Read** | `docs/tactical_architecture.md` §5; `app/tactical/platform/RobotContext.hpp`; `drivers/Battery.hpp`, `drivers/Motor.hpp` |
| **Creates** | `app/tactical/platform/SafetyMonitor.hpp/.cpp` |
| **Freezes** | `SafetyVerdict { ControlMode mode; Reason reason; }` |
| **Done when** | Builds, and every trigger in §5.2 is reachable from synthetic inputs. |

**Task.** Implement the full §5.2 trigger table — link age > 250 ms, driver disable switch, latched motor
fault, debounced battery sag, control overrun, and the mode state machine of §5.1.

Two tiers, and the distinction is the point: **Disabled** is unlatched and recovers by itself when the
cause clears; **Fault** latches until explicitly cleared. Return the *reason* alongside the mode so
telemetry can report `"Disabled: link age 312 ms"` rather than going quiet.

`Battery::isLow()` already suppresses its own boot case via `adcDmaIsRunning()` — do not duplicate that
check. `Motor::isFaulted()` already ORs the EXTI latch with the pin level.

### U0.6 — Task set and integration

| | |
|---|---|
| **Model** | Stronger — interrupt priorities, task interaction and watchdog gating all meet here. |
| **Read** | `docs/tactical_architecture.md` §3 and §4; `app/tactical/main_tactical.cpp`; `drivers/CRSFReceiver.hpp` |
| **Creates / modifies** | `app/tactical/tasks/*`; `app/tactical/main_tactical.cpp` |
| **Done when** | Builds. On hardware: mode reaches Disabled within 250 ms of pulling the receiver, and the cycle holds 200 Hz with zero overruns for ten minutes. |

**Task.** Create Comms (priority 5), Control (4), Telemetry stub (2) and Heartbeat (1), per §3.1. The
control cycle runs the seven steps of §4 in order, at 200 Hz via `vTaskDelayUntil`.

Wire `HAL_UARTEx_RxEventCallback` to `CRSFReceiver::onDmaRxEvent`, and add the UART4 DMA IRQ to
`configureInterruptPriorities()` at priority 6 — it calls a FreeRTOS API, so it must sit in the 5–14 band.

Measure cycle time with the DWT cycle counter, which `Buzzer::init()` already enables. Publish it; fault
on ten consecutive overruns.

The hardware half of the verification cannot be run from a session — report the build result and state
plainly that the on-hardware criteria are outstanding.

### U0.7 — Indicators and watchdog

| | |
|---|---|
| **Model** | Cheaper — small, fully specified, no shared interfaces. |
| **Read** | `docs/tactical_architecture.md` §5.4; `drivers/Led.hpp`, `drivers/Buzzer.hpp` |
| **Creates** | `app/tactical/tasks/HeartbeatTask.cpp` |
| **Done when** | Builds. On hardware, LED_1 tracks the link and LED_0's blink pattern changes with mode. |

**Task.** LED_0 blink pattern encodes the mode; LED_1 shows link health; LED_2 shows a latched fault;
buzzer chirps on mode change.

Refresh the IWDG **only** if the control task's liveness counter advanced since the last tick — that is
the entire point of putting the watchdog here rather than in the control task. `Buzzer::beep()` blocks
and busy-waits; priority 1 is the only place that is acceptable, so call it from here and nowhere else.

---

## 4. Phase 1 — Raw drive (the MVP)

### U1.1 — Host test harness

| | |
|---|---|
| **Model** | Stronger — build-system work against a cross-compiling project; obscure failure modes. |
| **Read** | `CMakeLists.txt`; `cmake/gcc-arm-none-eabi.cmake` |
| **Creates** | `tests/CMakeLists.txt`, `tests/example_test.cpp`; modifies root `CMakeLists.txt` |
| **Done when** | `cmake -S . -B build-host -DIMC_HOST_TESTS=ON && cmake --build build-host && ctest --test-dir build-host` passes. |

**Task.** Build `app/tactical/control/` for the host with the native compiler — no toolchain file, no
HAL. Gate the whole thing behind `option(IMC_HOST_TESTS)` so the firmware build is untouched when it
is off.

**Glob the test sources deliberately:**

```cmake
file(GLOB IMC_TEST_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp)
```

A glob is the wrong default in this repo and right here: U1.2a–d run in parallel and would otherwise all
edit this one file. Comment the reason in place so nobody "fixes" it later.

Keep the framework minimal — a hand-rolled assert macro that counts failures and returns non-zero is
enough. Do not add a dependency.

### U1.2a–d — Control primitives *(four independent units)*

| | |
|---|---|
| **Model** | Cheaper, all four. Pure functions, unambiguous specs, instant host-test feedback. |
| **Read** | `docs/style_guide.md`; `tests/CMakeLists.txt` |
| **Creates** | one header plus one test each, under `app/tactical/control/` and `tests/` |
| **Done when** | `ctest --test-dir build-host` passes, including the edge cases named below. |

Header-only, no HAL include, no global state, no dependency on each other.

- **U1.2a `ExpoCurve.hpp`** — maps `[-1, 1]` to `[-1, 1]` with an adjustable expo factor; `0.0f` is
  linear. Must be exactly symmetric about zero and preserve the endpoints. Edge cases: 0, ±1, factor 0.
- **U1.2b `SlewRateLimiter.hpp`** — limits change per second given a `dt`. Edge cases: `dt` of zero
  (must not divide by it), a step larger than the limit, and a sign change across zero.
- **U1.2c `Debouncer.hpp`** — a boolean must hold its new state for N milliseconds before the output
  follows. Edge cases: chatter shorter than the window, and the very first sample.
- **U1.2d `ArcadeMixer.hpp`** — throttle and steering to left/right. When the result clips, **scale both
  sides to preserve the ratio** rather than clamping each independently; clamping turns a hard forward
  turn into an unintended straight line. Edge cases: full throttle plus full steering, and both zero.

### U1.3 — `DriveBase`

| | |
|---|---|
| **Model** | Cheaper — consumes three frozen interfaces; the open-loop path is straightforward. |
| **Read** | `app/tactical/subsystems/Subsystem.hpp`; `drivers/Motor.hpp`, `drivers/Encoder.hpp`; `app/tactical/control/ArcadeMixer.hpp` |
| **Creates** | `app/tactical/subsystems/DriveBase.hpp/.cpp` |
| **Done when** | Builds; `onDisable()` calls `Motor::disable()` on both channels. |

**Task.** A `Subsystem` owning two `Motor` and two `Encoder` instances **directly** — not `Drivetrain`,
which was removed and must not come back.

Intent methods (`arcade(throttle, steering)`, `tank(left, right)`) set targets and return.
`onPeriodic()` applies slew limiting and writes the motors. Carry a `ControlMode { Open, Velocity }` enum
now even though only `Open` is implemented, so phase 3 fills in a branch rather than changing signatures.

Call `Encoder::setDirection()` with the sign of the commanded duty each cycle, before `Encoder::update()`
— a single-channel encoder cannot determine direction on its own.

### U1.4 — Teleop behavior and robot config

| | |
|---|---|
| **Model** | Cheaper — construction and mapping against frozen interfaces. |
| **Read** | `docs/tactical_architecture.md` §6.1 and §7.1; `app/tactical/subsystems/DriveBase.hpp` |
| **Creates** | `app/tactical/behavior/Behavior.hpp`, `behavior/TeleopBehavior.hpp/.cpp`, `config/RobotConfig.hpp`, `config/Robot2026.hpp/.cpp` |
| **Done when** | Builds with `-DIMC_ROBOT=2026`. |

**Task.** `Behavior` is an interface with `update(const RobotContext&)` and `isFinished()`.
`TeleopBehavior` maps sticks to `DriveBase` intent methods and touches no actuator directly.

`Robot2026` constructs the drivers and subsystems as members, lists them in a `Subsystem*` array, and
does nothing else. **If you write an `if` in that file, the logic belongs in a subsystem or a behavior.**
Use designated initializers for anything with more than two fields — this layer is why C++20 was adopted.

### U1.5 — Integration and bench verification

| | |
|---|---|
| **Model** | Stronger — first time the whole stack drives real motors. |
| **Read** | `app/tactical/main_tactical.cpp`; `app/tactical/tasks/ControlTask.cpp` |
| **Modifies** | `app/tactical/main_tactical.cpp`; `app/tactical/tasks/ControlTask.cpp` |
| **Done when** | Sticks drive the robot, **and it coasts to a stop within 250 ms of the transmitter being switched off — tested deliberately, at speed, more than once.** |

---

## 5. The human's loop

Before reading any code, three checks:

```bash
git status --short          # only the declared blast radius? if not, that is the finding
scripts/build.sh --target imc_tactical
git add -A && git commit -m "U0.3: subsystem contract and manager"
```

The `git status` check is mechanical and catches the most expensive failure — an agent that edited a
header another unit owns. Unit-prefixed commit messages make `git log --oneline` a progress report.

If verification fails, **stay in that session**. The context is loaded; restarting pays the read cost
twice. If the agent starts asking clarifying questions about scope, the unit was too big — split it.

---

## 6. Why this shape rather than sprints

The useful part of agile here is the vertical slice: every phase ends in something demonstrable rather
than in a completed layer. The phases in `tactical_architecture.md` §10 already have that shape.

The rest does not transfer. Story points measure nothing about an agent, sprints are a time box and an
agent has no cadence, and "as a user, I want…" produces fiction when applied to `Snapshot<T>`.

What actually constrains a session is **context budget**. So the unit here is not a story — it is a
change with a closed context boundary and a machine-checkable done condition. Within each phase the
order is always **contract → leaves → integration**, and it is not negotiable: interfaces get frozen
before anything consumes them, leaves depend only on committed interfaces, and integration is
sequential because merging two agents' wiring costs more than doing it once.

That ordering works because of the architecture's one-way dependency rule. Interfaces that only ever
point downward are interfaces that can be frozen, and a frozen interface is what makes a unit closed.

---

*Mark units Done in §1 as they land. Add the next phase's units before starting that phase.*
