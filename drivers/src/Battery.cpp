/*******************************************************************************
 * @file Battery.cpp
 * @brief Battery voltage monitor implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Battery.hpp"

#include "AdcDma.hpp"

Battery::Battery(uint8_t vbatSlot)
  : m_vbatSlot(vbatSlot)
{}

uint32_t Battery::readMillivolts()
{
  // Snapshot the volatile slot into a local variable before arithmetic.
  // g_adcBuf[m_vbatSlot] is written continuously by the ADC1 DMA circular scan.
  uint16_t raw = g_adcBuf[m_vbatSlot];

  // V_adc (mV) = (raw / 4095) × 3300
  float vAdcMv = (static_cast<float>(raw) / kAdcMaxF) * kVdda;

  // V_battery = V_adc × kDividerRatio
  return static_cast<uint32_t>(vAdcMv * kDividerRatio);
}

bool Battery::isLow()
{
  return (readMillivolts() < kLowBatteryThresholdMv);
}

/* EOF -----------------------------------------------------------------------*/
