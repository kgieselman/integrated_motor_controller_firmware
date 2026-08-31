/*******************************************************************************
 * @file HeartbeatTask.hpp
 * @brief The Heartbeat task - indicators and, from U0.7, the watchdog.
 *
 * Priority 1, the lowest application task, on a 100 ms vTaskDelay (§3.1). It
 * drives the three debug LEDs and the buzzer, and it is where the IWDG refresh
 * belongs.
 *
 * WHY THE LOWEST PRIORITY, AND WHY THAT MATTERS TWICE:
 *
 *  - Buzzer::beep() blocks and busy-waits on the DWT cycle counter. Priority 1
 *    is the only place in this firmware where that is acceptable, so the buzzer
 *    is called from here and from nowhere else (U0.7).
 *  - The IWDG refresh is gated on the control task's liveness counter
 *    advancing, not on this task running. A watchdog that a task refreshes just
 *    because it is scheduled proves only that the scheduler is alive; gating it
 *    on RobotState::liveness is what makes it prove that the CONTROL task is.
 *    That is the entire reason the refresh lives here rather than in the cycle
 *    it is watching.
 *
 * PHASE 0 SCOPE. This unit (U0.6) creates the task, its period and its hold on
 * the indicator hardware, and blinks LED_0 so that the scheduler is visibly
 * running. The mode-coded blink pattern, the link and fault indicators, the
 * mode-change chirp and the IWDG refresh are unit U0.7, whose blast radius is
 * HeartbeatTask.cpp alone - which is why heartbeatTaskInit() below already
 * takes every piece of hardware U0.7 needs. That signature is not provisional.
 *
 * @see docs/tactical_architecture.md §5.4 for the indicator scheme, §3.1 for
 *      the task.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include "Buzzer.hpp"
#include "Led.hpp"

/**
 * @brief Bind the task to the board's indicator hardware.
 *
 * Call once, after each object's own init() and before tasksCreateAll(). None
 * of the four is owned or copied; all are constructed in main_tactical.cpp and
 * live for the whole run.
 *
 * @param ledMode  DEBUG_LED_0 - blink pattern encodes the mode.
 * @param ledLink  DEBUG_LED_1 - radio link health.
 * @param ledFault DEBUG_LED_2 - latched fault.
 * @param buzzer   The passive transducer. Blocks while sounding; see the file
 *                 note on why only this task may call it.
 */
void heartbeatTaskInit(Led& ledMode, Led& ledLink, Led& ledFault, Buzzer& buzzer);

/**
 * @brief Create the Heartbeat task at Tasks::kHeartbeatPriority.
 *
 * @return true if the task was created. False also if heartbeatTaskInit() was
 *         not called first - a heartbeat task with no hardware bound would run
 *         happily and indicate nothing, which is the worst possible failure for
 *         an indicator.
 */
bool heartbeatTaskCreate();

/* EOF -----------------------------------------------------------------------*/
