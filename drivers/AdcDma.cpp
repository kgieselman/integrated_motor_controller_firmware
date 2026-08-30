/*******************************************************************************
 * @file AdcDma.cpp
 * @brief ADC1 DMA circular scan buffer definition and start-up.
 *
 * Zero-initialised at startup.  The DMA controller populates each slot on
 * every completed ADC1 scan; reads are performed by Motor and Battery without
 * locking.  See AdcDma.hpp for the full usage contract.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "AdcDma.hpp"

volatile uint16_t g_adcBuf[ADC_DMA_NUM_CHANNELS] = {};

namespace
{
  /// Set once adcDmaStart() succeeds. Read from task context only.
  volatile bool s_running = false;
}

bool adcDmaStart(ADC_HandleTypeDef* hadc)
{
  if (hadc == nullptr)
  {
    return false;
  }

  // STM32H5's HAL_ADCEx_Calibration_Start takes only the single/differential
  // selector. The three-argument form with ADC_CALIB_OFFSET is the H7 API.
  if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    return false;
  }

  // The DMA writes 16-bit samples; the HAL signature takes a uint32_t* address.
  if (HAL_ADC_Start_DMA(hadc,
                        reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(g_adcBuf)),
                        ADC_DMA_NUM_CHANNELS) != HAL_OK)
  {
    return false;
  }

  s_running = true;
  return true;
}

bool adcDmaIsRunning()
{
  return s_running;
}

/* EOF -----------------------------------------------------------------------*/
