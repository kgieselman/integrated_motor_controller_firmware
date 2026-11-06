/*******************************************************************************
 * @file TelemetryTask.cpp
 * @brief Console host and outgoing-telemetry stub, at priority 2.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "tasks/TelemetryTask.hpp"

#include "tasks/ControlTask.hpp"
#include "tasks/RobotState.hpp"
#include "tasks/Tasks.hpp"

#include "Console.hpp"

extern "C"
{
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
}

#include <cstdint>

/* File-local constants ------------------------------------------------------*/

/// Task period. §3.1's 50 ms - fast enough that a terminal feels responsive and
/// slow enough that formatting never competes with the control cycle.
static constexpr TickType_t kTelemetryPeriodTicks = pdMS_TO_TICKS(50U);

/* Task-owned objects --------------------------------------------------------*/

/**
 * @brief The console, on USART1.
 *
 * @note Transport is USART1 rather than USB-CDC because of a Rev A connector
 *       part shortage. Swapping it back is this one line plus the receive
 *       wiring below.
 */
static Console s_console(&huart1);

/// Single-byte staging buffer for interrupt-driven receive.
static uint8_t s_rxByte = 0U;

/* Console commands ----------------------------------------------------------*/

/**
 * @brief "status": print what the control cycle last published.
 *
 * Read-only. Uses the same RobotState snapshot the heartbeat task uses, so it
 * shows exactly what the indicators are acting on. The raw USB_DETECTED pin
 * level is read here directly, as a diagnostic, because RobotState does not
 * carry it and its polarity is still to be confirmed on the bench.
 */
static void cmdStatus(Console& console, int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  const RobotState st = controlTaskRobotState().read();

  const char* modeText = "unknown";
  switch (st.mode)
  {
    case ControlMode::Disabled: modeText = "Disabled"; break;
    case ControlMode::Teleop:   modeText = "Teleop";   break;
    case ControlMode::Auto:     modeText = "Auto";     break;
    case ControlMode::Fault:    modeText = "Fault";    break;
    default:                    break;
  }

  console.printf("mode      : %s\r\n", modeText);
  console.printf("reason    : %s%s\r\n",
                 SafetyMonitor::reasonToString(st.reason),
                 st.faultLatched ? " (latched)" : "");
  console.printf("battery   : %lu mV (%s)\r\n",
                 static_cast<unsigned long>(st.batteryMillivolts),
                 st.batteryValid ? "valid" : "not valid");
  console.printf("usb pin   : %d (PA15 raw level)\r\n",
                 static_cast<int>(HAL_GPIO_ReadPin(USB_DETECTED_GPIO_Port, USB_DETECTED_Pin)));
  console.printf("link age  : %lu ms\r\n", static_cast<unsigned long>(st.linkAgeMs));
  console.printf("cycle     : %lu us (max %lu us, overruns %lu)\r\n",
                 static_cast<unsigned long>(st.cycleTimeUs),
                 static_cast<unsigned long>(st.maxCycleTimeUs),
                 static_cast<unsigned long>(st.overrunTotal));
  console.printf("liveness  : %lu\r\n", static_cast<unsigned long>(st.liveness));
}

/* Task ----------------------------------------------------------------------*/

/**
 * @brief Drain complete console lines and dispatch them.
 *
 * @param pvParameters Unused.
 *
 * @todo Console::feed() runs in ISR context while poll() runs here, and the
 *       line buffer is unprotected between them (open issue F3). Acceptable
 *       while the console is diagnostics-only; it must be fixed before phase
 *       2's ParamStore trusts a command to change a stored parameter.
 *
 * @todo Phase 2: drain the EventRing and format a RobotState frame to the
 *       transport. The snapshot read below is the whole of the outgoing path
 *       today, and exists so that the wiring is exercised rather than merely
 *       described.
 */
static void telemetryTaskEntry(void* pvParameters)
{
  (void)pvParameters;

  static_cast<void>(s_console.registerCommand("status",
                                              "Mode, fault reason, battery, cycle stats",
                                              cmdStatus));

  s_console.printAbout();
  s_console.printHelp();

  for (;;)
  {
    s_console.poll();

    const RobotState state = controlTaskRobotState().read();
    static_cast<void>(state);

    vTaskDelay(kTelemetryPeriodTicks);
  }
}

/* Public interface ----------------------------------------------------------*/

bool telemetryTaskInit()
{
  return (HAL_UART_Receive_IT(&huart1, &s_rxByte, 1U) == HAL_OK);
}

bool telemetryTaskCreate()
{
  return (xTaskCreate(telemetryTaskEntry,
                      "Telemetry",
                      Tasks::kTelemetryStackWords,
                      nullptr,
                      Tasks::kTelemetryPriority,
                      nullptr) == pdPASS);
}

void telemetryTaskOnRxComplete(UART_HandleTypeDef* huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  s_console.feed(&s_rxByte, 1U);
  static_cast<void>(HAL_UART_Receive_IT(&huart1, &s_rxByte, 1U));
}

/* EOF -----------------------------------------------------------------------*/
