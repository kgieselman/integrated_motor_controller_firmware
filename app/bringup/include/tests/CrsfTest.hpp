#pragma once
#include "Console.hpp"
#include <cstdint>

/// Register the "crsf" command with the console.
void registerCrsfTests(Console& c);

/**
 * @brief Route a UART4 DMA Rx event to the file-local CRSFReceiver.
 *
 * Call from HAL_UARTEx_RxEventCallback in main_bringup.cpp:
 * @code
 *   if (huart->Instance == UART4) crsfOnDmaRxEvent(size);
 * @endcode
 */
void crsfOnDmaRxEvent(uint16_t size);
