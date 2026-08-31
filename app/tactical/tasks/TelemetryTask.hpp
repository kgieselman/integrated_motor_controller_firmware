/*******************************************************************************
 * @file TelemetryTask.hpp
 * @brief The Telemetry task - console in, robot state out. Stub in phase 0.
 *
 * Priority 2, a plain 50 ms vTaskDelay (§3.1). It owns the console transport
 * and, from phase 2, the outgoing telemetry frame and the parameter console.
 *
 * WHAT IS A STUB AND WHAT IS NOT. The task, its period, its priority and the
 * console it hosts are real - this file replaces the standalone console task
 * that main_tactical.cpp carried through the phase-0 skeleton. What is not
 * here is the outgoing half: §3.2's EventRing, the TelemetrySink formatter and
 * ParamStore are all phase 2, and this task reads RobotState without yet doing
 * anything with it. That is the "Telemetry stub" of unit U0.6.
 *
 * Console transport is USART1 on the growth header, not USB-CDC - a Rev A
 * connector workaround documented in main_tactical.cpp.
 *
 * @see docs/tactical_architecture.md §3.1 for the task, §7.2 for the parameter
 *      console this grows into.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "stm32h5xx_hal.h"
}

/**
 * @brief Arm the console receiver.
 *
 * Call once, after MX_USART1_UART_Init() and before tasksCreateAll().
 *
 * @return true if the first receive interrupt was armed.
 */
bool telemetryTaskInit();

/**
 * @brief Create the Telemetry task at Tasks::kTelemetryPriority.
 *
 * @return true if the task was created.
 */
bool telemetryTaskCreate();

/**
 * @brief Feed a received console byte and re-arm the receiver.
 *
 * Wire this to HAL_UART_RxCpltCallback.
 *
 * @param huart UART that raised the interrupt. Ignored unless it is the console
 *              transport, so the application callback can forward every UART.
 *
 * @note ISR context. Calls no FreeRTOS API, so the kernel places no constraint
 *       on its NVIC priority - it is nonetheless set with the others so the
 *       policy of §3.3 holds uniformly.
 */
void telemetryTaskOnRxComplete(UART_HandleTypeDef* huart);

/* EOF -----------------------------------------------------------------------*/
