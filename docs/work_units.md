# Tactical Firmware — Work Units

**How this codebase is broken up for AI-assisted development.**

Companion to [`tactical_architecture.md`](tactical_architecture.md), which is the design. This file is
the *plan of attack*: each unit below is sized to be picked up in one fresh session, by one agent, with a
named set of files to read and a verification step that agent can run without hardware or a human.

---

## 1. Why not sprints and stories

The useful part of agile here is the vertical slice — every phase ends in something demonstrable rather
than in a completed layer. The phases in `tactical_architecture.md` §10 already have that shape.

The rest of the ceremony does not transfer. Story points and velocity measure nothing about an agent.
Sprints are a time box, and an agent has no cadence. "As a user, I want…" produces fiction when applied
to `Snapshot<T>`, which serves no user story and never will.

What actually constrains a session is **context budget** — how much must be read to do the work
correctly. So the unit of work here is not a story. It is *a change with a closed context boundary and a
machine-checkable done condition*.

### 1.1 Contract, then leaves, then integration

Within each phase the ordering is always the same, and it is not negotiable:

1. **Contract units** — define the headers and types that other units consume. Done alone, first, and
   committed before anything else starts. Header-only where possible.
2. **Leaf units** — consume only committed interfaces, and nothing from each other. These are the ones
   that can run in parallel if you ever want to.
3. **Integration units** — wiring, task creation, configuration. Inherently sequential. Do not try to
   parallelise these; the cost of merging two agents' integration work exceeds the saving.

The reason this works is the architecture's one-way dependency rule (§2.1). Interfaces that only ever
point downward are interfaces that can be frozen, and a frozen interface is what makes a unit closed.

### 1.2 What makes a unit the right size

Every unit below states four things. If a unit is missing any of them, it is the wrong shape and should
be split:

| | |
|---|---|
| **Read** | The exact files to load. If this list exceeds five files, the unit is too big. |
| **Creates / modifies** | The blast radius, declared up front. |
| **Freezes / consumes** | Which interfaces it defines, and which it depends on already existing. |
| **Done when** | A command the agent can run, or an observable it can check. Not "looks right". |

---

## 2. Choosing a model

Do not spend a frontier model on mechanical work, and do not save money on work whose mistakes are
silent.

**Use the stronger model when *any* of these is true:**

- The unit **defines an interface** other units consume. A mistake here propagates into every consumer
  and is expensive to unwind after they exist.
- It reasons about **concurrency or interrupt context** — ISR/task boundaries, critical sections,
  atomicity. These failures are intermittent, and a test that passes proves nothing.
- Its **failure mode is silent or physical** — a failsafe that doesn't fire, a pin driven the wrong way,
  a motor that keeps running.
- The **spec requires judgment**: choosing thresholds, resolving a contradiction between documents,
  deciding what the right abstraction is.

**Use the cheaper model when *all* of these are true:**

- The interface it works against is **already frozen and committed**.
- The spec is **unambiguous** — the behaviour is fully described, nothing is left to taste.
- A **build or host test catches errors immediately**, so a wrong answer surfaces in seconds.

Pure functions in `control/` are the archetype of the second category: fully specified, no hardware, no
concurrency, host-testable. Contracts and safety logic are the archetype of the first.

### 2.1 Briefing template

```
Read docs/tactical_architecture.md, then these files and no others:
  <the unit's Read list>

Task: <the unit's one-sentence goal>

You may create or modify only:
  <the unit's blast radius>

Follow docs/style_guide.md. Do not change any interface outside your blast
radius — if one looks wrong, stop and say so rather than editing it.

Done when: <the unit's verification>
```

The instruction not to edit outside the blast radius matters more than it looks. Most cross-unit damage
comes from an agent "helpfully" fixing something in a header another unit owns.

---

## 3. Phase 0 — Spine

Nothing moves at the end of this phase. The milestone is that the radio link is visible and the failsafe
demonstrably fires.

### U0.1 — Core types

| | |
|---|---|
| **Model** | **Stronger.** Pure contract work: every later unit consumes these types, so a poor shape here is paid for six times over. |
| **Depends on** | — |
| **Blocks** | U0.2, U0.3, U0.4, U0.5, U0.6 |
| **Read** | `docs/tactical_architecture.md` §4, `drivers/Motor.hpp`, `drivers/Encoder.hpp`, `drivers/Battery.hpp` |
| **Creates** | `app/tactical/platform/RobotContext.hpp` |
| **Freezes** | `ControlMode`, `SensorFrame`, `DriverInput`, `RobotContext` |
| **Done when** | `scripts/build.sh --target imc_tactical` still succeeds (header compiles when included from `main_tactical.cpp`). |

Header-only, no `.cpp`. `SensorFrame` holds one cycle's readings as plain values — no HAL types, per
invariant 3. `DriverInput` carries the decoded axes, a switch bitmask and `ageMs`.

### U0.2 — `Snapshot<T>`

| | |
|---|---|
| **Model** | **Stronger.** Sixty lines, but it is the concurrency primitive the whole design rests on, and a torn read fails once a week rather than once a run. |
| **Depends on** | U0.1 |
| **Blocks** | U0.4, U0.5, U0.6 |
| **Read** | `docs/tactical_architecture.md` §3.2, `app/tactical/platform/RobotContext.hpp` |
| **Creates** | `app/tactical/platform/Snapshot.hpp` |
| **Freezes** | `Snapshot<T>::read()`, `Snapshot<T>::write()` |
| **Done when** | Builds, and a `static_assert(std::is_trivially_copyable_v<T>)` rejects a non-POD instantiation. |

Save and restore PRIMASK rather than calling `__enable_irq()` — see `CRSFReceiver::hasNewData()` for the
pattern and the reason.

### U0.3 — Subsystem contract and manager

| | |
|---|---|
| **Model** | **Stronger.** This is *the* interface of the whole design — every mechanism, this year and every year after, implements it. |
| **Depends on** | U0.1 |
| **Blocks** | every subsystem, in every later phase |
| **Read** | `docs/tactical_architecture.md` §6, `app/tactical/platform/RobotContext.hpp` |
| **Creates** | `app/tactical/subsystems/Subsystem.hpp`, `app/tactical/subsystems/SubsystemManager.hpp/.cpp` |
| **Freezes** | The five-method `Subsystem` contract |
| **Done when** | Builds with a throwaway test subsystem registered in a zero-length array. |

The manager holds `Subsystem* const*` and a count — no ownership, no allocation. `disableAll()` must call
`onDisable()` on every subsystem unconditionally, not only on ones it believes are enabled.

### U0.4 — `InputSource`

| | |
|---|---|
| **Model** | **Cheaper.** Mechanical mapping against two frozen interfaces, fully specified, verifiable on a host. |
| **Depends on** | U0.1, U0.2 |
| **Blocks** | U0.5, U0.6 |
| **Read** | `drivers/CRSFReceiver.hpp`, `app/tactical/platform/RobotContext.hpp`, `app/tactical/platform/Snapshot.hpp` |
| **Creates** | `app/tactical/platform/InputSource.hpp/.cpp` |
| **Consumes** | `CRSFReceiver`, `Snapshot<DriverInput>` |
| **Done when** | Builds; the CRSF→normalised mapping is exercised by a table of known channel values. |

Deadband, expo curve and switch decode happen here, so the control task sees intent rather than channel
counts. **CRSF channel 5 is the enable switch** (agreed default). Stamp `ageMs` from `lastFrameAgeMs()`.

### U0.5 — `SafetyMonitor`

| | |
|---|---|
| **Model** | **Stronger.** Safety logic with judgment calls, and its failure mode is a robot that does not stop. |
| **Depends on** | U0.1, U0.2, U0.4 |
| **Blocks** | U0.6 |
| **Read** | `docs/tactical_architecture.md` §5, `app/tactical/platform/RobotContext.hpp`, `drivers/Battery.hpp`, `drivers/Motor.hpp` |
| **Creates** | `app/tactical/platform/SafetyMonitor.hpp/.cpp` |
| **Freezes** | `SafetyVerdict { ControlMode mode; Reason reason; }` |
| **Done when** | Builds; each trigger in §5.2 is reachable in a host-side table test with synthetic inputs. |

Implement the full §5.2 table. Two tiers: **Disabled** recovers by itself, **Fault** latches. Return the
*reason* alongside the mode — telemetry saying `"Disabled: link age 312 ms"` is worth the extra field.
`Battery::isLow()` already guards its own boot case; do not duplicate that check here.

### U0.6 — Task set and integration

| | |
|---|---|
| **Model** | **Stronger.** Interrupt priorities, task interaction, watchdog gating — many constraints meeting at once, and this is where they collide. |
| **Depends on** | U0.1 – U0.5 |
| **Blocks** | phase 1 |
| **Read** | `docs/tactical_architecture.md` §3–§4, `app/tactical/main_tactical.cpp`, `drivers/CRSFReceiver.hpp` |
| **Modifies** | `app/tactical/main_tactical.cpp`, `CMakeLists.txt`; creates `app/tactical/tasks/*` |
| **Done when** | Builds; on hardware the mode reaches Disabled within 250 ms of pulling the receiver, and the cycle holds 200 Hz with zero overruns for ten minutes. |

Create Comms (5), Control (4), Telemetry stub (2), Heartbeat (1). Wire `HAL_UARTEx_RxEventCallback` to
`CRSFReceiver::onDmaRxEvent`, and set the UART4 DMA IRQ to priority 6 — it calls a FreeRTOS API, so
`configureInterruptPriorities()` must cover it. Measure cycle time with the DWT counter that
`Buzzer::init()` already enables.

### U0.7 — Indicators and watchdog

| | |
|---|---|
| **Model** | **Cheaper.** Small, fully specified, no shared interfaces. |
| **Depends on** | U0.6 |
| **Blocks** | — |
| **Read** | `docs/tactical_architecture.md` §5.4, `drivers/Led.hpp`, `drivers/Buzzer.hpp` |
| **Creates** | `app/tactical/tasks/HeartbeatTask.cpp` |
| **Done when** | Builds; LED_1 tracks the link and LED_0's blink pattern changes with mode, observed on hardware. |

Refresh IWDG **only** if the control task's liveness counter advanced. Call `Buzzer::beep()` from here and
nowhere else — it busy-waits, and priority 1 is the only place that is acceptable.

---

## 4. Phase 1 — Raw drive (the MVP)

### U1.1 — Host test harness

| | |
|---|---|
| **Model** | **Stronger.** Build-system work against a cross-compiling project; the failure modes are obscure. |
| **Depends on** | — (can run in parallel with phase 0) |
| **Blocks** | U1.2a–d |
| **Read** | `CMakeLists.txt`, `cmake/gcc-arm-none-eabi.cmake` |
| **Creates** | `tests/CMakeLists.txt`, `tests/main.cpp`; modifies root `CMakeLists.txt` |
| **Done when** | `cmake -B build-host -DIMC_HOST_TESTS=ON && ctest` runs and passes with one trivial test. |

Builds `control/` for the host with the native compiler — no toolchain file, no HAL. Keep the test
framework minimal; a hand-rolled assert macro is enough and adds no dependency.

### U1.2a–d — Control primitives *(four parallel units)*

| | |
|---|---|
| **Model** | **Cheaper**, all four. Pure functions, unambiguous specs, instant host-test feedback. The archetype. |
| **Depends on** | U1.1 |
| **Blocks** | U1.3 |
| **Read** | `docs/style_guide.md`, `tests/CMakeLists.txt` |
| **Creates** | one pair each: **a)** `ExpoCurve.hpp` · **b)** `SlewRateLimiter.hpp` · **c)** `Debouncer.hpp` · **d)** `ArcadeMixer.hpp` under `app/tactical/control/` |
| **Done when** | `ctest` passes, including edge cases: zero `dt`, saturated input, sign changes across zero. |

No HAL types, no `#include "stm32h5xx_hal.h"`, no global state. `ArcadeMixer` must preserve the
throttle/steering ratio when scaling a clipped result rather than clamping each side independently.

### U1.3 — `DriveBase`

| | |
|---|---|
| **Model** | **Cheaper.** Consumes three frozen interfaces; the open-loop path is straightforward. |
| **Depends on** | U0.3, U1.2 |
| **Blocks** | U1.4 |
| **Read** | `app/tactical/subsystems/Subsystem.hpp`, `drivers/Motor.hpp`, `drivers/Encoder.hpp`, `app/tactical/control/ArcadeMixer.hpp` |
| **Creates** | `app/tactical/subsystems/DriveBase.hpp/.cpp` |
| **Done when** | Builds; `onDisable()` verifiably calls `Motor::disable()` on both channels. |

Owns two `Motor` and two `Encoder` instances directly — *not* `Drivetrain`, which was removed. Carry the
`Open` / `Velocity` mode enum now even though only `Open` is implemented, so phase 3 is a fill-in rather
than a signature change. Call `Encoder::setDirection()` with the commanded sign each cycle.

### U1.4 — Teleop and robot config

| | |
|---|---|
| **Model** | **Cheaper.** Construction and mapping against frozen interfaces. |
| **Depends on** | U1.3 |
| **Blocks** | U1.5 |
| **Read** | `docs/tactical_architecture.md` §6.1 and §7.1, `app/tactical/subsystems/DriveBase.hpp` |
| **Creates** | `app/tactical/behavior/Behavior.hpp`, `behavior/TeleopBehavior.hpp/.cpp`, `config/RobotConfig.hpp`, `config/Robot2026.hpp/.cpp` |
| **Done when** | Builds with `-DIMC_ROBOT=2026`. |

Use designated initializers for anything with more than two fields — this layer is the reason C++20 was
adopted. `Robot2026` constructs and lists; if you write an `if` in it, the logic belongs elsewhere.

### U1.5 — Integration and bench verification

| | |
|---|---|
| **Model** | **Stronger.** First time the whole stack drives real motors; judgment needed on what "drivable" means. |
| **Depends on** | U1.4 |
| **Blocks** | phase 2 |
| **Read** | `app/tactical/main_tactical.cpp`, `app/tactical/tasks/ControlTask.cpp` |
| **Modifies** | `app/tactical/main_tactical.cpp`, `app/tactical/tasks/ControlTask.cpp` |
| **Done when** | Sticks drive the robot, **and it coasts to a stop within 250 ms of the transmitter being switched off — tested deliberately, at speed, more than once.** |

---

## 5. Session hygiene

- **One unit per session.** Start fresh. Context does not need to carry over, because every interface a
  unit depends on is already committed to a file.
- **Read the architecture doc, not the codebase.** That is what it is for — one read instead of fifteen
  files of reverse-engineering. If the doc is wrong, fix the doc as part of the unit.
- **Commit per unit.** A unit that cannot be committed on its own was scoped wrong.
- **The blast radius is a hard boundary.** An agent that finds a problem outside it should report it, not
  fix it. Cross-unit edits are where parallel work goes wrong.
- **When a unit's verification cannot be run** — anything needing hardware — say so explicitly in the
  handover rather than implying it passed.

---

*Update this file when a phase completes, and add the next phase's units before starting it.*
