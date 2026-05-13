/*******************************************************************************
 * @file AdcDma.cpp
 * @brief ADC1 DMA circular scan buffer definition.
 *
 * Zero-initialised at startup.  The DMA controller populates each slot on
 * every completed ADC1 scan; reads are performed by Motor and Battery without
 * locking.  See AdcDma.hpp for the full usage contract.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "AdcDma.hpp"

volatile uint16_t g_adcBuf[ADC_DMA_NUM_CHANNELS] = {};

/* EOF -----------------------------------------------------------------------*/
