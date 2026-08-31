/*******************************************************************************
 * @file CommsTask.hpp
 * @brief The Comms task: CRSF frames in, a DriverInput snapshot out.
 *
 * Priority 5, the highest application task (§3.1). It owns the CRSFReceiver on
 * UART4, the InputSource that decodes it, and the Snapshot<DriverInput> the
 * control task reads at step 2 of every cycle. Nothing else in the firmware
 * touches a CRSFReceiver.
 *
 * TRIGGERED BY THE ISR, BUT NOT ONLY BY IT. §3.1 has this task woken by a task
 * notification from the UART4 DMA / IDLE interrupt, which is what makes a fresh
 * frame visible to the next cycle almost immediately. It also wakes on a short
 * timeout, because InputSource::update() must keep publishing when frames STOP
 * arriving: DriverInput::ageMs is what the failsafe reads, and an age that
 * stops being refreshed is an age that stops growing. A purely notification-
 * driven comms task goes silent at exactly the moment the failsafe needs it.
 *
 * @see docs/tactical_architecture.md §3.1 for the task, §3.3 for the interrupt
 *      priority the wake-up ISR must sit at.
 * @see drivers/CRSFReceiver.hpp for the receive model - the ISR records the DMA
 *      head position and nothing else; all parsing happens in this task.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "stm32h5xx_hal.h"
}

#include "platform/RobotContext.hpp"
#include "platform/Snapshot.hpp"

#include <cstdint>

/**
 * @brief Start the CRSF circular DMA receive.
 *
 * Call once, after MX_UART4_Init() and MX_GPDMA1_Init() and before
 * tasksCreateAll(). The transfer then runs for the life of the program and is
 * never re-armed.
 *
 * @return true if the DMA receive started.
 *
 * @note After this returns true the Rx event ISR can fire at any time, which is
 *       why it must run before the task is created rather than inside it: the
 *       ISR hook below is written to tolerate arriving first.
 */
bool commsTaskInit();

/**
 * @brief Create the Comms task at Tasks::kCommsPriority.
 *
 * @return true if the task was created.
 */
bool commsTaskCreate();

/**
 * @brief Record a UART Rx event and wake the Comms task.
 *
 * Wire this to HAL_UARTEx_RxEventCallback. It is the only path from the
 * interrupt into this task.
 *
 * @param huart UART that raised the event. Ignored unless it is UART4, so the
 *              application callback can forward every UART unconditionally.
 * @param size  Current DMA write position in bytes from the start of the ring -
 *              a position, not a count, because the transfer is circular.
 *
 * @note ISR context. Calls vTaskNotifyGiveFromISR(), so its NVIC priority must
 *       sit in the 5-14 band the kernel can mask (§3.3, priority 6). It does no
 *       parsing and no copying: one 16-bit store, one notification.
 *
 * @note Safe before the task exists and before the scheduler starts. The
 *       receiver's head position is still recorded in that window; only the
 *       notification is skipped, and the task's timeout wake-up covers it.
 */
void commsTaskOnUartRxEvent(UART_HandleTypeDef* huart, uint16_t size);

/**
 * @brief The Snapshot the control task reads at step 2 of every cycle.
 *
 * @return Reference to the Comms task's DriverInput snapshot. Read-only to
 *         everyone but this task - single writer is Snapshot<T>'s one
 *         unenforceable rule (invariant 2), and const is how it is enforced
 *         here.
 */
const Snapshot<DriverInput>& commsTaskDriverInput();

/* EOF -----------------------------------------------------------------------*/
