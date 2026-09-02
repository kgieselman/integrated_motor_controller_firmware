/*******************************************************************************
 * @file Buzzer.cpp
 * @brief Driver for the DEBUG_BUZZER transducer on PC14.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Buzzer.hpp"

/* Construction --------------------------------------------------------------*/

Buzzer::Buzzer(GPIO_TypeDef* port, uint16_t pin)
    : m_port(port)
    , m_pin(pin)
    , m_ready(false)
{}

/* Public methods ------------------------------------------------------------*/

bool Buzzer::init()
{
  if (m_port == nullptr)
  {
    return false;
  }

  silence();

  // Enable the DWT cycle counter. Idempotent — if a debugger or another driver
  // already enabled it, these writes change nothing.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT       = 0U;
  DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

  // Confirm the counter actually advances before trusting a busy-wait against
  // it. On a part where DWT is unavailable, waitCycles() would never return.
  //
  // The counter is deliberately NOT volatile: incrementing a volatile is
  // deprecated under C++20 and draws -Wvolatile. __NOP() is an asm volatile, so
  // the loop body cannot be optimised away and the loop survives without it.
  const uint32_t first = DWT->CYCCNT;
  for (uint32_t i = 0U; i < 16U; ++i)
  {
    __NOP();
  }
  m_ready = (DWT->CYCCNT != first);

  return m_ready;
}

void Buzzer::beep(uint32_t freqHz, uint32_t durMs)
{
  if (!m_ready)
  {
    return;
  }

  if (freqHz < kMinFreqHz)
  {
    freqHz = kMinFreqHz;
  }

  const uint32_t halfPeriodCycles = SystemCoreClock / (2U * freqHz);
  const uint32_t endTick          = HAL_GetTick() + durMs;

  while (HAL_GetTick() < endTick)
  {
    HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_SET);
    waitCycles(halfPeriodCycles);
    HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_RESET);
    waitCycles(halfPeriodCycles);
  }

  silence();
}

void Buzzer::chirp()
{
  beep(kResonantFreqHz, kChirpMs);
}

void Buzzer::silence()
{
  if (m_port != nullptr)
  {
    HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_RESET);
  }
}

/* Private helpers -----------------------------------------------------------*/

void Buzzer::waitCycles(uint32_t cycles)
{
  const uint32_t start = DWT->CYCCNT;

  // Unsigned subtraction wraps correctly, so this is safe across the counter's
  // 32-bit rollover.
  while ((DWT->CYCCNT - start) < cycles)
  {
  }
}

/* EOF -----------------------------------------------------------------------*/
