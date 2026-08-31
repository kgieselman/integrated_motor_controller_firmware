/*******************************************************************************
 * @file CommsTask.cpp
 * @brief CRSF receive, decode and publish, at priority 5.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "tasks/CommsTask.hpp"

#include "tasks/Tasks.hpp"

#include "CRSFReceiver.hpp"
#include "platform/InputSource.hpp"

extern "C"
{
#include "usart.h"
}

/* File-local constants ------------------------------------------------------*/

/**
 * @brief How long to wait for a frame notification before publishing anyway.
 *
 * Chosen against SafetyMonitor::kLinkTimeoutMs (250 ms), not against the frame
 * rate: it only has to be short enough that the published ageMs is never stale
 * by a margin that matters at that threshold. 20 ms is an order of magnitude
 * inside it and costs 50 wake-ups a second on a dead link.
 */
static constexpr TickType_t kNotifyTimeoutTicks = pdMS_TO_TICKS(20U);

/* Task-owned objects --------------------------------------------------------*/

/// UART4 at 420 000 baud, circular DMA. Owned by this task alone.
static CRSFReceiver s_receiver(&huart4);

/// Published once per update(); read by the control task at step 2.
static Snapshot<DriverInput> s_driverInput;

/// Channel mapping, deadband, expo and switch decode. See InputSource.hpp.
static InputSource s_inputSource(s_receiver, s_driverInput);

/// Set by commsTaskCreate(); read by the ISR hook. nullptr until then.
static TaskHandle_t s_taskHandle = nullptr;

/* Task ----------------------------------------------------------------------*/

/**
 * @brief Wake on a frame or on a timeout, then decode and publish.
 *
 * @param pvParameters Unused.
 */
static void commsTaskEntry(void* pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    // The return value is deliberately ignored: this task does the same work
    // whether it was woken by a frame or by the timeout. InputSource::update()
    // drains whatever has arrived and always republishes, so a timeout wake-up
    // is what keeps ageMs growing while the link is down.
    (void)ulTaskNotifyTake(pdTRUE, kNotifyTimeoutTicks);

    s_inputSource.update();
  }
}

/* Public interface ----------------------------------------------------------*/

bool commsTaskInit()
{
  return s_receiver.init();
}

bool commsTaskCreate()
{
  return (xTaskCreate(commsTaskEntry,
                      "Comms",
                      Tasks::kCommsStackWords,
                      nullptr,
                      Tasks::kCommsPriority,
                      &s_taskHandle) == pdPASS);
}

void commsTaskOnUartRxEvent(UART_HandleTypeDef* huart, uint16_t size)
{
  if (huart != s_receiver.uartHandle())
  {
    return;
  }

  s_receiver.onDmaRxEvent(size);

  // The DMA is armed before the task is created, so this window is real rather
  // than theoretical. The head position above is already recorded; skipping the
  // notification costs at most one timeout period of latency.
  if (s_taskHandle == nullptr)
  {
    return;
  }

  BaseType_t higherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(s_taskHandle, &higherPriorityTaskWoken);
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

const Snapshot<DriverInput>& commsTaskDriverInput()
{
  return s_driverInput;
}

/* EOF -----------------------------------------------------------------------*/
