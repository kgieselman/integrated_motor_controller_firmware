/*******************************************************************************
 * @file Battery.hpp
 * @brief Battery voltage monitor via VBAT_SENSE ADC channel.
 *
 * The Integrated Motor Controller board divides +BATT down to the STM32 ADC range through a
 * resistor divider on PC4 (ADC1_IN4 / VBAT_SENSE, also TP2). The divider
 * ratio defaults to kDefaultDividerRatio and can be tuned via the constructor.
 *
 * Pin assignment:
 *  - PC4 → ADC1 channel IN4 (VBAT_SENSE)
 *
 * Nominal battery: 3S LiPo, ~11.1 V nominal, 12.6 V full, 9.9 V cutoff.
 *
 * Voltage is read from g_adcBuf[kSlotVbat], which is populated continuously
 * by the ADC1 DMA circular scan.  No HAL polling is performed at read time.
 * See AdcDma.hpp for scan configuration requirements.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include "AdcDma.hpp"
#include <cstdint>

/**
 * @brief Reads battery voltage via the VBAT_SENSE ADC divider.
 */
class Battery
{
public:
  /**
   * @brief Default voltage divider ratio: V_battery / V_adc.
   *
   * Schematic: R_top = 100 kΩ (R7/R9), R_bot = 30 kΩ (R12).
   * Ratio = (100 kΩ + 30 kΩ) / 30 kΩ = 4.333.
   *
   * Pass a different value to the constructor to tune for real-world resistor
   * tolerances without recompiling.
   */
  static constexpr float kDefaultDividerRatio = 4.333f; ///< (100 kΩ + 30 kΩ) / 30 kΩ

  /// Low-battery warning threshold (mV). 3S LiPo ≈ 3.5 V/cell = 10 500 mV.
  static constexpr uint32_t kLowBatteryThresholdMv = 10500U;

  /**
   * @brief Construct a Battery monitor.
   *
   * @param vbatSlot      Slot index into g_adcBuf[] for the VBAT_SENSE channel.
   *                      Defaults to kSlotVbat (PC4, ADC1 INP4).
   * @param dividerRatio  Voltage divider ratio V_battery / V_adc.
   *                      Defaults to kDefaultDividerRatio.  Adjust to compensate
   *                      for real-world resistor tolerances without recompiling.
   */
  explicit Battery(uint8_t vbatSlot    = kSlotVbat,
                   float   dividerRatio = kDefaultDividerRatio);

  /**
   * @brief Read the battery voltage.
   *
   * Reads g_adcBuf[m_vbatSlot] (populated by the ADC1 DMA circular scan),
   * applies the voltage-divider ratio, and returns the result in millivolts.
   *
   * @return Battery voltage in millivolts.
   */
  uint32_t readMillivolts();

  /**
   * @brief Check whether the battery is below the low-voltage threshold.
   *
   * @return true if voltage < kLowBatteryThresholdMv.
   */
  bool isLow();

private:
  uint8_t m_vbatSlot;     ///< Slot index into g_adcBuf[] for VBAT_SENSE.
  float   m_dividerRatio; ///< Voltage divider ratio (V_battery / V_adc).

  static constexpr float kVdda    = 3300.0f; ///< ADC reference (mV)
  static constexpr float kAdcMaxF = 4095.0f; ///< 12-bit full-scale (float)
};

/* EOF -----------------------------------------------------------------------*/
