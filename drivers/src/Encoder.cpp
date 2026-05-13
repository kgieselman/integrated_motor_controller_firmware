/*******************************************************************************
 * @file Encoder.cpp
 * @brief Quadrature encoder implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Encoder.hpp"

Encoder::Encoder(TIM_HandleTypeDef* htim,
                 uint32_t           countsPerRev,
                 float              updateRateHz)
  : m_htim(htim)
  , m_countsPerRev(countsPerRev)
  , m_updateRateHz(updateRateHz)
  , m_accumulator(0)
  , m_lastRaw(0U)
  , m_velocityRpm(0.0f)
{}

bool Encoder::init()
{
  if (HAL_TIM_Encoder_Start(m_htim, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return false;
  }
  m_lastRaw = readCounter();
  return true;
}

void Encoder::update()
{
  uint32_t raw    = readCounter();
  uint32_t period = m_htim->Instance->ARR + 1U; // counter period (e.g. 65536 for 16-bit)

  // Compute signed delta with overflow handling.
  int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(m_lastRaw);

  // Correct for 16-bit wraparound (TIM4 / TIM8 / TIM12 are 16-bit).
  if (delta >  static_cast<int32_t>(period / 2U)) { delta -= static_cast<int32_t>(period); }
  if (delta < -static_cast<int32_t>(period / 2U)) { delta += static_cast<int32_t>(period); }

  m_accumulator += static_cast<int64_t>(delta);
  m_lastRaw      = raw;

  // Velocity: counts/update → rev/min
  // velocity(RPM) = (delta_counts / countsPerRev) * updateRateHz * 60
  if (m_countsPerRev > 0U && m_updateRateHz > 0.0f)
  {
    m_velocityRpm = (static_cast<float>(delta) / static_cast<float>(m_countsPerRev))
                    * m_updateRateHz * 60.0f;
  }
}

int64_t Encoder::count() const
{
  return m_accumulator;
}

float Encoder::velocityRpm() const
{
  return m_velocityRpm;
}

void Encoder::resetCount()
{
  m_accumulator = 0;
}

uint32_t Encoder::readCounter() const
{
  return __HAL_TIM_GET_COUNTER(m_htim);
}

/* EOF -----------------------------------------------------------------------*/
