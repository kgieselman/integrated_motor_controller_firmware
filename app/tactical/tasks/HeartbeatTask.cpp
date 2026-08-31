/*******************************************************************************
 * @file HeartbeatTask.cpp
 * @brief Indicator drive at priority 1. Phase 0 blink only; see the header.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "tasks/HeartbeatTask.hpp"

#include "tasks/Tasks.hpp"

extern "C"
{
#include "FreeRTOS.h"
#include "task.h"
}

#include <cstdint>

/* File-local constants ------------------------------------------------------*/

/// Task period. §3.1's 100 ms - the resolution every blink pattern is built on.
static constexpr TickType_t kHeartbeatPeriodTicks = pdMS_TO_TICKS(100U);

/// Ticks of this task per LED_0 toggle. Five gives 500 ms on, 500 ms off.
static constexpr uint32_t kToggleTicks = 5U;

/* Task-owned objects --------------------------------------------------------*/

// Non-owning. Constructed and initialised in main_tactical.cpp, which keeps
// board-level hardware in one readable place; bound here by heartbeatTaskInit().
static Led*    s_pLedMode  = nullptr; ///< DEBUG_LED_0 - mode.
static Led*    s_pLedLink  = nullptr; ///< DEBUG_LED_1 - link health.
static Led*    s_pLedFault = nullptr; ///< DEBUG_LED_2 - latched fault.
static Buzzer* s_pBuzzer   = nullptr; ///< Passive transducer.

/* Task ----------------------------------------------------------------------*/

/**
 * @brief Blink LED_0 to prove the scheduler and this task are running.
 *
 * @param pvParameters Unused.
 *
 * @todo U0.7: replace the fixed toggle with a mode-coded blink pattern on
 *       LED_0, drive LED_1 from RobotState::linkAgeMs and LED_2 from
 *       RobotState::faultLatched, chirp on a mode change, and refresh the IWDG
 *       only when RobotState::liveness has advanced since the previous tick.
 *       Everything that needs is already bound above and readable through
 *       controlTaskRobotState().
 */
static void heartbeatTaskEntry(void* pvParameters)
{
  (void)pvParameters;

  uint32_t tick = 0U;

  for (;;)
  {
    if ((tick % kToggleTicks) == 0U)
    {
      s_pLedMode->toggle();
    }

    ++tick;
    vTaskDelay(kHeartbeatPeriodTicks);
  }
}

/* Public interface ----------------------------------------------------------*/

void heartbeatTaskInit(Led& ledMode, Led& ledLink, Led& ledFault, Buzzer& buzzer)
{
  s_pLedMode  = &ledMode;
  s_pLedLink  = &ledLink;
  s_pLedFault = &ledFault;
  s_pBuzzer   = &buzzer;
}

bool heartbeatTaskCreate()
{
  // Guard clause rather than a null check inside the loop: the binding cannot
  // change once the task is running, so the question is worth asking exactly
  // once, here, where the answer can still stop the scheduler from starting.
  if ((s_pLedMode == nullptr) || (s_pLedLink == nullptr) || (s_pLedFault == nullptr)
      || (s_pBuzzer == nullptr))
  {
    return false;
  }

  return (xTaskCreate(heartbeatTaskEntry,
                      "Heartbeat",
                      Tasks::kHeartbeatStackWords,
                      nullptr,
                      Tasks::kHeartbeatPriority,
                      nullptr) == pdPASS);
}

/* EOF -----------------------------------------------------------------------*/
