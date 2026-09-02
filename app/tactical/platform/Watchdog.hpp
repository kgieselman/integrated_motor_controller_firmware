/*******************************************************************************
 * @file Watchdog.hpp
 * @brief Independent watchdog bring-up and refresh (RM0481 §41).
 *
 * The IWDG is the only failsafe in this firmware that survives the control task
 * itself wedging: it is clocked by the LSI, not by the system clock, and no
 * amount of software going wrong can stop it. Every row of
 * docs/tactical_architecture.md §5.2 above "control task stall" is evaluated by
 * software that a stall would take down with it; this row is not.
 *
 * Two free functions and a namespace of constants rather than a class, matching
 * Tasks.hpp. There is exactly one IWDG, it holds no per-instance state, and it
 * can never be constructed twice - a class here would be a singleton in a
 * costume.
 *
 * Written against the registers directly. HAL_IWDG_MODULE_ENABLED is commented
 * out in cubemx/Core/Inc/stm32h5xx_hal_conf.h, which is CubeMX-owned and would
 * lose the edit on the next regeneration; the CMSIS IWDG definition and the
 * RCC / DBGMCU macros used in the implementation all arrive with
 * stm32h5xx_hal.h regardless.
 *
 * Usage - watchdogStart() is called by main() as the very last thing before
 * vTaskStartScheduler(), and watchdogRefresh() only from the heartbeat task's
 * liveness gate:
 *
 * @code
 *   if (!watchdogStart())     // last statement before the scheduler starts
 *   {
 *     s_ledFault.on();
 *   }
 *
 *   vTaskStartScheduler();
 * @endcode
 *
 * @warning Once started, the IWDG cannot be stopped by software. A build that
 *          calls watchdogStart() and then stops refreshing resets in
 *          Watchdog::kReloadMs - which is the entire point.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include <cstdint>

/*******************************************************************************
 * @brief Timing the independent watchdog is configured for.
 *
 * Public because the heartbeat task's period and the buzzer's blocking chirp
 * are what size it, and anyone changing either needs to see this number.
 ******************************************************************************/
namespace Watchdog
{
  /// Reload period in milliseconds - the longest a refresh may be missed before
  /// the board resets.
  ///
  /// The floor is the worst case the heartbeat task can legitimately produce:
  /// two of its 100 ms ticks (docs/tactical_architecture.md §3.1, and §5.2's
  /// stall row is detected at one-to-two-tick granularity) plus one blocking
  /// Buzzer::kChirpMs of 60 ms landing inside one of them - 260 ms. 500 ms is
  /// that with a shade under 2x margin, which absorbs the LSI's own tolerance
  /// (roughly ±3% over temperature and supply) and leaves room for the task to
  /// be preempted by the three higher-priority tasks without a spurious reset.
  ///
  /// Not made larger, because this number is also how long a wedged robot holds
  /// its last commanded duty cycle on the H-bridges before the reset releases
  /// them - and from phase 1 on, that duty cycle can be full throttle.
  static constexpr uint32_t kReloadMs = 500U;
} // namespace Watchdog

/**
 * @brief Start the independent watchdog and freeze it under debug.
 *
 * Enables the LSI and waits for it to be ready, configures the prescaler and
 * reload for Watchdog::kReloadMs, starts the counter, and sets the DBGMCU
 * freeze bit so that halting at a breakpoint does not reset the board.
 *
 * @return true if the watchdog is running; false if the LSI never became ready
 *         or the prescaler / reload write was never accepted.
 *
 * @note Call this LAST, immediately before vTaskStartScheduler(). Started any
 *       earlier, every remaining line of peripheral init has to complete inside
 *       one reload period - and it would buy nothing, because nothing before
 *       the scheduler loops, so nothing before the scheduler can wedge in the
 *       way a watchdog exists to catch.
 *
 * @note Uses HAL_GetTick() for its timeouts, so it must run after HAL_Init()
 *       and before the FreeRTOS port takes SysTick over.
 *
 * @warning A false return means the stall failsafe of §5.2 is not armed. That
 *          is not a reason to refuse to boot - a robot that reports the gap is
 *          more useful than one sitting dark in a while(1) - but it is a reason
 *          to light LED_2.
 */
bool watchdogStart();

/**
 * @brief Reload the watchdog counter.
 *
 * @note Called only from HeartbeatTask's refreshWatchdogIfControlAlive(), and
 *       only when the control task's liveness counter has advanced. Refreshing
 *       from anywhere else would downgrade a statement about the 200 Hz control
 *       cycle into a statement about the scheduler, which is a far weaker thing
 *       to promise.
 *
 * @note Safe to call before watchdogStart(): the key register ignores a reload
 *       key while the counter is stopped.
 */
void watchdogRefresh();

/* EOF -----------------------------------------------------------------------*/
