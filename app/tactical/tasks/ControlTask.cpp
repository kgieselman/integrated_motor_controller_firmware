/*******************************************************************************
 * @file ControlTask.cpp
 * @brief The seven steps of §4, every 5 ms, at priority 4.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "tasks/ControlTask.hpp"

#include "tasks/CommsTask.hpp"
#include "tasks/Tasks.hpp"

#include "Battery.hpp"
#include "platform/RobotContext.hpp"
#include "platform/SafetyMonitor.hpp"
#include "subsystems/SubsystemManager.hpp"

extern "C"
{
#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
}

#include <span>

/* File-local constants ------------------------------------------------------*/

/// Control period. 5 ms at configTICK_RATE_HZ 1000 is exactly 5 ticks, so
/// vTaskDelayUntil() has no rounding error to accumulate.
static constexpr TickType_t kControlPeriodTicks = pdMS_TO_TICKS(5U);

/**
 * @brief The dt handed to subsystems, in seconds.
 *
 * DELIBERATELY THE NOMINAL PERIOD, NOT THE MEASURED ONE. §4.1 asks for a fixed
 * dt so that a PID's derivative term is honest; feeding it the jitter between
 * two vTaskDelayUntil() wake-ups would put scheduler noise straight into the
 * derivative, where it is amplified. vTaskDelayUntil() holds the period to the
 * tick, so the nominal value is also the true one to well within what any of
 * this is measuring. The cycle's real execution time is not discarded - it is
 * measured separately and published as RobotState::cycleTimeUs, which is where
 * an overrun becomes visible.
 */
static constexpr float kControlPeriodSeconds = 0.005f;

/* Task-owned objects --------------------------------------------------------*/

/// VBAT divider reader. Owned here rather than by a subsystem because it is
/// board-level: no mechanism owns the battery, and sense() is the only code in
/// the cycle permitted to read hardware (§4 step 1).
static Battery s_battery;

/// The failsafe gate consulted at step 3 of every cycle.
static SafetyMonitor s_safety;

/**
 * @brief The subsystem array, empty in phase 0.
 *
 * §10's phase 0 lands "a SubsystemManager with zero subsystems. Nothing moves."
 * SubsystemManager documents an empty span as legal and as making every method
 * a no-op, which is exactly the phase-0 robot. Phase 1 replaces this with the
 * array a config/Robot<year>.hpp owns.
 */
static SubsystemManager s_manager{std::span<Subsystem* const>{}};

/// Published at step 7; read by the heartbeat and telemetry tasks.
static Snapshot<RobotState> s_robotState;

/* Cycle statistics ----------------------------------------------------------*/

/// Execution time of the PREVIOUS cycle, µs. Fed to SafetyMonitor::evaluate(),
/// which is the only causal option: the current cycle is not over when the
/// verdict is needed.
static uint32_t s_cycleTimeUs = 0U;

/// Largest cycle time seen since boot, µs.
static uint32_t s_maxCycleTimeUs = 0U;

/// Cumulative overrunning cycles. Telemetry only - SafetyMonitor counts the
/// CONSECUTIVE ones and owns the fault.
static uint32_t s_overrunTotal = 0U;

/// Incremented once per completed cycle. The heartbeat task's liveness gate.
static uint32_t s_liveness = 0U;

/* Private helpers -----------------------------------------------------------*/

/**
 * @brief Convert a DWT cycle count to microseconds.
 *
 * @param cycles Elapsed core cycles.
 * @return Elapsed microseconds, truncated.
 *
 * @note Reads SystemCoreClock rather than hard-coding 250, so this stays
 *       correct if the clock tree changes. At 250 MHz one microsecond is 250
 *       cycles, so the truncation is worth well under a percent of a 4 ms
 *       overrun threshold.
 */
static uint32_t cyclesToMicroseconds(uint32_t cycles)
{
  const uint32_t cyclesPerUs = SystemCoreClock / 1000000U;

  if (cyclesPerUs == 0U)
  {
    return 0U;
  }

  return cycles / cyclesPerUs;
}

/**
 * @brief Step 1 - read every input exactly once into this cycle's SensorFrame.
 *
 * @param frame Frame to fill. Every field this firmware currently measures is
 *              overwritten; the rest keep RobotContext.hpp's defaults.
 *
 * @note WHAT IS NOT READ HERE YET, and why, because a reader will ask.
 *
 *       - encoderCounts, encoderVelocityRpm, motorCurrentMa and motorFaulted
 *         all come from the Motor and Encoder instances that DriveBase owns
 *         (§10.2). Phase 0 has no subsystems at all, so those objects do not
 *         exist yet, and constructing a second set here would put two owners on
 *         one H-bridge - the exact thing invariant 2 forbids. The unit that
 *         adds DriveBase gives this function access to them; until then these
 *         fields keep their defaults, which RobotContext.hpp warns are
 *         placeholders and not safe states. Nothing consumes them in phase 0:
 *         SafetyMonitor's motor-fault trigger reads motorFaulted and will
 *         simply never fire until this is wired.
 *       - accelG and gyroDps come from the IMU on SPI1. It is board-level like
 *         the battery, so it could be read here, but its first consumer is
 *         HeadingEstimator in phase 4 and reading it now would add a blocking
 *         SPI transaction to the only cycle phase 0 has to prove holds 200 Hz.
 *         Deliberately deferred rather than forgotten.
 *
 * @todo Phase 1 (U1.3 / U1.4): fill the four per-channel arrays from the
 *       DriveBase-owned Motor and Encoder instances.
 * @todo Phase 4: fill accelG and gyroDps from the IMU.
 */
static void sense(SensorFrame& frame)
{
  frame.batteryValid      = s_battery.isValid();
  frame.batteryMillivolts = s_battery.readMillivolts();
}

/**
 * @brief Step 7 - measure the cycle, update the statistics and publish.
 *
 * @param ctx         This cycle's context, already carrying the decided mode.
 * @param verdict     What SafetyMonitor returned at step 3.
 * @param startCycles DWT->CYCCNT sampled at the top of this cycle.
 *
 * @note The measurement covers steps 1-6 and excludes this function's own
 *       snapshot write, because a cycle time cannot include the act of
 *       reporting it. The excluded part is one struct copy under a critical
 *       section - a few hundred nanoseconds against a 4000 µs threshold.
 *
 * @note DWT->CYCCNT wraps every ~17 s at 250 MHz. The unsigned subtraction
 *       below is correct across that wrap for any interval shorter than the
 *       wrap period, which a 5 ms cycle always is.
 */
static void publish(const RobotContext&                 ctx,
                    const SafetyMonitor::SafetyVerdict& verdict,
                    uint32_t                            startCycles)
{
  s_cycleTimeUs = cyclesToMicroseconds(DWT->CYCCNT - startCycles);

  if (s_cycleTimeUs > s_maxCycleTimeUs)
  {
    s_maxCycleTimeUs = s_cycleTimeUs;
  }

  if (s_cycleTimeUs > SafetyMonitor::kCycleOverrunUs)
  {
    ++s_overrunTotal;
  }

  ++s_liveness;

  const RobotState state{.nowMs             = ctx.nowMs,
                         .mode              = verdict.mode,
                         .reason            = verdict.reason,
                         .faultLatched      = s_safety.isFaultLatched(),
                         .linkAgeMs         = ctx.input.ageMs,
                         .batteryMillivolts = ctx.sensors.batteryMillivolts,
                         .batteryValid      = ctx.sensors.batteryValid,
                         .cycleTimeUs       = s_cycleTimeUs,
                         .maxCycleTimeUs    = s_maxCycleTimeUs,
                         .overrunTotal      = s_overrunTotal,
                         .liveness          = s_liveness};

  s_robotState.write(state);
}

/* Task ----------------------------------------------------------------------*/

/**
 * @brief The control cycle. Seven steps, fixed order, 200 Hz.
 *
 * @param pvParameters Unused.
 */
static void controlTaskEntry(void* pvParameters)
{
  (void)pvParameters;

  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&lastWakeTime, kControlPeriodTicks);

    const uint32_t startCycles = DWT->CYCCNT;

    RobotContext ctx;
    ctx.dt    = kControlPeriodSeconds;
    ctx.nowMs = HAL_GetTick();

    /* 1 - sense */
    sense(ctx.sensors);

    /* 2 - the driver's last decoded frame */
    ctx.input = commsTaskDriverInput().read();

    /* 3 - the gate */
    const SafetyMonitor::SafetyVerdict verdict =
        s_safety.evaluate(ctx.nowMs, ctx.input, ctx.sensors, s_cycleTimeUs);

    ctx.mode = verdict.mode;

    if ((verdict.mode == ControlMode::Disabled) || (verdict.mode == ControlMode::Fault))
    {
      /* 4 - not permitted to run */
      s_manager.disableAll();
    }
    else
    {
      /* 5 - the active behavior expresses intent.
       *
       * Phase 0 has no behavior layer: behavior/Behavior.hpp and
       * TeleopBehavior arrive with phase 1, and the robot config that chooses
       * between them arrives with them. The step is left visible rather than
       * silently absent so that the seven steps of §4 can be read off this
       * loop, which is the whole reason §4 is a numbered list.
       *
       * @todo Phase 1 (U1.4): m_behavior->update(ctx).
       */

      /* 6 - subsystems run and write their actuators */
      s_manager.periodic(ctx);
    }

    /* 7 - publish.
     *
     * On BOTH paths, deliberately. §4 step 4 publishes before returning for
     * exactly this reason: a robot that goes quiet the moment it is disabled is
     * a robot whose telemetry is useless at the only time you need it.
     */
    publish(ctx, verdict, startCycles);
  }
}

/* Public interface ----------------------------------------------------------*/

bool controlTaskInit()
{
  const bool selfTestPassed = s_manager.initAll();
  s_safety.reportSelfTest(selfTestPassed);

  return selfTestPassed;
}

bool controlTaskCreate()
{
  return (xTaskCreate(controlTaskEntry,
                      "Control",
                      Tasks::kControlStackWords,
                      nullptr,
                      Tasks::kControlPriority,
                      nullptr) == pdPASS);
}

const Snapshot<RobotState>& controlTaskRobotState()
{
  return s_robotState;
}

/* EOF -----------------------------------------------------------------------*/
