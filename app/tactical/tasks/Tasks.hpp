/*******************************************************************************
 * @file Tasks.hpp
 * @brief The task set of docs/tactical_architecture.md §3.1, in one table.
 *
 * Four application tasks and nothing else. Their priorities and stack depths
 * are declared here rather than beside each task's body so that the whole
 * schedule is one screen: a priority is only meaningful relative to the others,
 * and a table split across four files is a table nobody checks.
 *
 * @code
 *   // main_tactical.cpp, before the scheduler starts:
 *   commsTaskInit();
 *   telemetryTaskInit();
 *   heartbeatTaskInit(ledMode, ledLink, ledFault, buzzer);
 *   controlTaskInit();
 *   tasksCreateAll();
 *   vTaskStartScheduler();
 * @endcode
 *
 * Each task owns its own hardware and its own snapshots inside its translation
 * unit; main_tactical.cpp brings up peripherals and creates tasks, and knows
 * nothing about what any of them holds.
 *
 * @note configMAX_PRIORITIES is 7, so priorities run 0-6. The FreeRTOS timer
 *       daemon holds 6 (configTIMER_TASK_PRIORITY) and the idle task holds 0.
 *       Application tasks therefore live in 1-5, which is exactly the range
 *       used below.
 *
 * @see docs/tactical_architecture.md §3.1 for the table this file implements.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "FreeRTOS.h"
#include "task.h"
}

#include <cstdint>

/*******************************************************************************
 * @brief Priorities and stack depths for the four application tasks.
 *
 * Stack depths are in WORDS, which is what xTaskCreate() takes, and are §3.1's
 * figures verbatim. They are first estimates and are meant to be replaced by
 * measurements: uxTaskGetStackHighWaterMark() is compiled in
 * (INCLUDE_uxTaskGetStackHighWaterMark), and right-sizing these along with
 * configTOTAL_HEAP_SIZE is a known open item in §10.1.
 ******************************************************************************/
namespace Tasks
{
  /// Comms: CRSF frames -> DriverInput. Highest, so a frame is never queued
  /// behind the cycle that wants to read it.
  static constexpr UBaseType_t kCommsPriority   = 5U;
  static constexpr uint16_t    kCommsStackWords = 512U;

  /// Control: the 200 Hz cycle. Below Comms so that its input is fresh, above
  /// everything that only reports on it.
  static constexpr UBaseType_t kControlPriority   = 4U;
  static constexpr uint16_t    kControlStackWords = 1024U;

  /// Telemetry: console and, in phase 2, the outgoing frame. Deliberately far
  /// below Control - formatting text must never delay a cycle.
  static constexpr UBaseType_t kTelemetryPriority   = 2U;
  static constexpr uint16_t    kTelemetryStackWords = 768U;

  /// Heartbeat: indicators and, in U0.7, the IWDG refresh. Lowest of the four
  /// on purpose - see HeartbeatTask.hpp on why Buzzer::beep() may only be
  /// called from here.
  static constexpr UBaseType_t kHeartbeatPriority   = 1U;
  static constexpr uint16_t    kHeartbeatStackWords = 256U;
} // namespace Tasks

/**
 * @brief Create all four application tasks.
 *
 * Call once, after every task's own init function has run and before
 * vTaskStartScheduler(). Creation order does not matter - nothing runs until
 * the scheduler starts - but each task's init DOES have to precede it, because
 * a task may be notified from an ISR the moment its peripheral is armed.
 *
 * @return true only if all four tasks were created. A false result means
 *         configTOTAL_HEAP_SIZE is too small for the stacks above, or that a
 *         task's init function was not called.
 *
 * @note Does not roll back on partial failure. There is nothing useful to roll
 *       back to: a robot that could not create its control task is not going to
 *       run, and the caller's only sane response is to stop before the
 *       scheduler starts.
 */
bool tasksCreateAll();

/* EOF -----------------------------------------------------------------------*/
