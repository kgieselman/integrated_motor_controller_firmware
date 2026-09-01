/*******************************************************************************
 * @file ControlTask.hpp
 * @brief The 200 Hz control cycle - the whole robot, in one task.
 *
 * Priority 4, woken by vTaskDelayUntil() every 5 ms (§3.1). It runs the seven
 * steps of §4 in a fixed order and owns every actuator on the board through
 * SubsystemManager. Everything else in the firmware exists to feed this task or
 * to watch it.
 *
 * It owns the SafetyMonitor, the SubsystemManager and the Snapshot<RobotState>
 * it publishes at step 7. It reads the comms task's DriverInput snapshot and
 * writes nothing that another task can see except that one snapshot.
 *
 * @see docs/tactical_architecture.md §4 for the seven steps, §4.1 for why this
 *      is one task rather than one per subsystem, §5 for the failsafe it
 *      consults at step 3.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include "platform/Snapshot.hpp"
#include "tasks/RobotState.hpp"

/**
 * @brief Start the cycle counter, run the boot self-test, report both.
 *
 * Starts and verifies this task's DWT cycle counter, calls
 * SubsystemManager::initAll(), and hands the AND of the two to
 * SafetyMonitor::reportSelfTest() - the §5.1 edge
 * `SelfTest --(any onInit() false)--> Fault`. Call once, before
 * tasksCreateAll() and therefore before the scheduler starts, which is the only
 * window in which a subsystem's onInit() may block.
 *
 * @return true if the cycle counter runs and every subsystem initialised.
 *
 * @note A cycle counter that will not start is a self-test FAILURE, not a
 *       degraded mode. It is what RobotState::cycleTimeUs is measured with, and
 *       therefore what the §5.2 control-overrun trigger is measured with; with
 *       it stopped the cycle time reads zero forever and the overrun failsafe
 *       silently cannot fire. Latching Fault with a readable reason is strictly
 *       better than running a robot whose failsafe is decorative.
 *
 * @note The subsystem half returns true in phase 0, where the manager holds
 *       zero subsystems - an empty walk trivially succeeds. Wiring it now
 *       rather than in the unit that adds the first subsystem is what stops
 *       that unit having to edit this task.
 */
bool controlTaskInit();

/**
 * @brief Create the Control task at Tasks::kControlPriority.
 *
 * @return true if the task was created.
 */
bool controlTaskCreate();

/**
 * @brief The RobotState published at step 7 of every cycle.
 *
 * @return Reference to the control task's RobotState snapshot. Const for the
 *         same reason as commsTaskDriverInput(): one writer, enforced by the
 *         type rather than by convention.
 */
const Snapshot<RobotState>& controlTaskRobotState();

/* EOF -----------------------------------------------------------------------*/
