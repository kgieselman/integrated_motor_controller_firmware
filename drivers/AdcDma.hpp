/*******************************************************************************
 * @file AdcDma.hpp
 * @brief ADC1 DMA circular scan buffer — shared by Motor (IPROPI) and Battery (VBAT).
 *
 * ADC1 runs in continuous scan mode with DMA circular transfer.  The DMA
 * writes each completed scan into g_adcBuf[] indefinitely; drivers read their
 * slot directly without polling or locking.
 *
 * CubeMX requirements (integrated_motor_controller.ioc):
 *   - ADC1: Continuous Conversion Mode = Enabled, Scan Conversion Mode = Enabled
 *   - DMA request: ADC1, circular, half-word (16-bit), memory increment
 *   - Channel ranks (must match slot indices below):
 *       Rank 1 — PA4,  ADC1 INP18, LEFT_MOTOR_IPROPI  → g_adcBuf[kSlotLeftIpropi]
 *       Rank 2 — PC5,  ADC1 INP8,  RIGHT_MOTOR_IPROPI → g_adcBuf[kSlotRightIpropi]
 *       Rank 3 — PC4,  ADC1 INP4,  VBAT_SENSE         → g_adcBuf[kSlotVbat]
 *   - Sampling times:
 *       IPROPI (INP18, INP8): 47.5 cycles  (low-impedance DRV8874 current-sense output)
 *       VBAT   (INP4):        92.5 cycles  (higher-impedance resistor divider)
 *
 * Start the scan once, after MX_ADC1_Init(), with adcDmaStart(), which wraps:
 * @code
 *   HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
 *   HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcBuf, ADC_DMA_NUM_CHANNELS);
 * @endcode
 * (Note the two-argument calibration call — the three-argument form taking
 * ADC_CALIB_OFFSET belongs to the STM32H7 HAL, not H5.) Until it has
 * been called, g_adcBuf[] is all zeroes — which is a *valid-looking* reading of
 * 0 mV and 0 mA. Drivers must therefore gate on adcDmaIsRunning() rather than
 * trusting the buffer, or a battery monitor reports a flat pack on every boot.
 *
 * Atomicity note: on Cortex-M33, naturally aligned 16-bit reads are single-bus-cycle
 * atomic operations.  A driver reading its slot once into a local variable is safe
 * without a mutex, provided no other code writes to the same slot.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "stm32h5xx_hal.h"
}

#include <cstdint>

/// Total number of channels in the ADC1 scan sequence.
static constexpr uint8_t ADC_DMA_NUM_CHANNELS = 3U;

/// Slot 0 — PA4, ADC1 INP18, LEFT_MOTOR_IPROPI
static constexpr uint8_t kSlotLeftIpropi  = 0U;
/// Slot 1 — PC5, ADC1 INP8,  RIGHT_MOTOR_IPROPI
static constexpr uint8_t kSlotRightIpropi = 1U;
/// Slot 2 — PC4, ADC1 INP4,  VBAT_SENSE
static constexpr uint8_t kSlotVbat        = 2U;

/**
 * @brief DMA destination buffer — updated continuously in circular mode.
 *
 * Declared volatile because the DMA controller writes it asynchronously.
 * Drivers must copy a slot to a local variable before performing arithmetic.
 * This array is defined in AdcDma.cpp and must be linked into every build
 * that uses Motor or Battery.
 */
extern volatile uint16_t g_adcBuf[ADC_DMA_NUM_CHANNELS];

/**
 * @brief Calibrate ADC1 and start the circular DMA scan.
 *
 * Call once, after MX_ADC1_Init(), before any Motor or Battery read is
 * trusted. Wrapping calibration and start together means a caller cannot start
 * the scan while leaving the running flag unset.
 *
 * @param hadc Handle for ADC1.
 * @return true if calibration and the DMA start both succeeded.
 */
bool adcDmaStart(ADC_HandleTypeDef* hadc);

/**
 * @brief Report whether the DMA scan has been started successfully.
 *
 * @return true once adcDmaStart() has succeeded. Until then every slot in
 *         g_adcBuf[] reads zero, which is indistinguishable from a genuine
 *         zero measurement — treat readings as invalid while this is false.
 */
bool adcDmaIsRunning();

/* EOF -----------------------------------------------------------------------*/
