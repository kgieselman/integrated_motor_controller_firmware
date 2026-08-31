/*******************************************************************************
 * @file RobotState.hpp
 * @brief The Control -> Telemetry / Heartbeat snapshot payload.
 *
 * The second of the two Snapshot<T> payloads named in
 * docs/tactical_architecture.md §3.2. DriverInput travels Comms -> Control;
 * this type travels the other way, out of the control cycle to every task that
 * needs to report on the robot without touching the cycle's own state.
 *
 * It is written exactly once per cycle, at step 7 of §4, by the control task
 * and by nothing else. Everything in it is a value the control task already
 * held that cycle - nothing here is recomputed, and no consumer reads hardware
 * to obtain any of it.
 *
 * WHY THIS LIVES IN tasks/ RATHER THAN platform/. §8.2 puts Telemetry's own
 * types in platform/, and this type will very likely move there when phase 2
 * builds the telemetry layer. Unit U0.6's blast radius is everything under
 * app/tactical/tasks/ plus main_tactical.cpp, so it is created here rather
 * than by editing a directory another unit owns. The move is a rename; the
 * contract below is what matters.
 *
 * @note Trivially copyable, and checked below - Snapshot<T> copies it whole
 *       inside a critical section, so a constructor or a virtual method added
 *       later would break publication silently.
 *
 * @see docs/tactical_architecture.md §3.2 for the mechanism, §4 step 7 for the
 *      publish step that fills this.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include "platform/RobotContext.hpp"
#include "platform/SafetyMonitor.hpp"

#include <cstdint>
#include <type_traits>

/*******************************************************************************
 * @brief Everything the control cycle reports about itself, once per cycle.
 *
 * Read by the heartbeat task to drive the indicators (§5.4) and by the
 * telemetry task to format an outgoing frame. Both are lower priority than the
 * control task and neither may block it, which is the whole reason this
 * crosses a Snapshot rather than being read from the control task's members.
 *
 * The defaults describe a robot that has not completed a cycle yet: Disabled,
 * no link, no measurement. A consumer that runs before the first publish -
 * which both the heartbeat and telemetry tasks will, since they are created at
 * the same time - therefore sees the safe story rather than a plausible one.
 ******************************************************************************/
struct RobotState
{
  /// HAL_GetTick() at the start of the cycle that published this.
  uint32_t nowMs = 0U;

  /// The mode this cycle ran in, as SafetyMonitor decided it at step 3.
  ControlMode mode = ControlMode::Disabled;

  /// Why that mode. Render with SafetyMonitor::reasonToString().
  SafetyMonitor::Reason reason = SafetyMonitor::Reason::LinkNeverEstablished;

  /// True while a latching fault is held (§5.1). Survives its cause going away.
  bool faultLatched = false;

  /// Age of the last decoded CRSF frame in ms, or DriverInput::kAgeNeverReceived.
  /// Carried so an indicator can show link health without reading the comms
  /// task's snapshot for itself.
  uint32_t linkAgeMs = DriverInput::kAgeNeverReceived;

  /// Pack voltage in millivolts as sense() read it this cycle.
  uint32_t batteryMillivolts = 0U;

  /// True once the battery reading can be trusted. Mirrors Battery::isValid().
  bool batteryValid = false;

  /// Execution time of this cycle in microseconds, from the DWT cycle counter.
  /// Measured across steps 1-6; see ControlTask.cpp on what it excludes.
  uint32_t cycleTimeUs = 0U;

  /// Largest cycleTimeUs seen since boot. The number worth watching - a mean
  /// hides exactly the outlier that trips SafetyMonitor::kCycleOverrunUs.
  uint32_t maxCycleTimeUs = 0U;

  /// Cumulative count of cycles that exceeded SafetyMonitor::kCycleOverrunUs.
  /// Telemetry only. The fault is decided by SafetyMonitor from CONSECUTIVE
  /// overruns; this total is the softer signal that something is creeping.
  uint32_t overrunTotal = 0U;

  /**
   * @brief Incremented once per completed cycle. The control task's liveness.
   *
   * The heartbeat task refreshes the IWDG only when this has advanced since its
   * last tick (U0.7). That is the entire reason the watchdog lives there and
   * not in the control task: a task cannot notice itself wedging.
   *
   * @note Wraps at 2^32, which at 200 Hz is about 248 days. A consumer must
   *       compare with != or unsigned subtraction, never with <.
   */
  uint32_t liveness = 0U;
};

/* Contract checks -----------------------------------------------------------*/

static_assert(std::is_trivially_copyable_v<RobotState>,
              "RobotState must stay trivially copyable - it lives in a Snapshot<T>");
static_assert(std::is_aggregate_v<RobotState>,
              "RobotState must stay an aggregate - the publish step builds one by hand");

/* EOF -----------------------------------------------------------------------*/
