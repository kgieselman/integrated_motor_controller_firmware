/*******************************************************************************
 * @file Battery.cpp
 * @brief Battery voltage monitor implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Battery.hpp"

#include "AdcDma.hpp"

Battery::Battery(uint8_t vbatSlot, float dividerRatio)
  : m_vbatSlot((vbatSlot < ADC_DMA_NUM_CHANNELS) ? vbatSlot : kSlotVbat)
  , m_dividerRatio(dividerRatio)
{
  // An out-of-range slot would index past g_adcBuf[]; fall back to the real
  // VBAT slot rather than reading whatever follows the array in memory.
}

bool Battery::isValid() const
{
  return adcDmaIsRunning();
}

uint32_t Battery::readMillivolts()
{
  // Snapshot the volatile slot into a local variable before arithmetic.
  // g_adcBuf[m_vbatSlot] is written continuously by the ADC1 DMA circular scan.
  const uint16_t raw = g_adcBuf[m_vbatSlot];

  // V_adc (mV) = (raw / 4095) × 3300
  const float vAdcMv = (static_cast<float>(raw) / kAdcMaxF) * kVdda;

  // V_battery = V_adc × m_dividerRatio
  return static_cast<uint32_t>(vAdcMv * m_dividerRatio);
}

bool Battery::isLow()
{
  // Before the DMA scan starts, every slot reads zero — which would look like a
  // completely flat pack and latch a brownout fault on every power-up. Report
  // "not low" until there is a real measurement to judge.
  if (!isValid())
  {
    return false;
  }

  return (readMillivolts() < kLowBatteryThresholdMv);
}

/* EOF -----------------------------------------------------------------------*/
