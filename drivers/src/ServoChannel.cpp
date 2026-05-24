/*******************************************************************************
 * @file ServoChannel.cpp
 * @brief RC servo PWM channel implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "ServoChannel.hpp"

#include <algorithm>

ServoChannel::ServoChannel(TIM_HandleTypeDef* htim,
                           uint32_t           channel,
                           uint32_t           ticksPerUs)
  : m_htim(htim)
  , m_channel(channel)
  , m_ticksPerUs(ticksPerUs)
  , m_currentPulseUs(kPulseMidUs)
{}

bool ServoChannel::init()
{
  // Stop first so re-initialisation is safe — HAL_TIM_PWM_Start fails if the
  // channel is already running (state != READY).
  HAL_TIM_PWM_Stop(m_htim, m_channel);

  if (HAL_TIM_PWM_Start(m_htim, m_channel) != HAL_OK)
  {
    return false;
  }
  setPulseUs(kPulseMidUs);
  return true;
}

void ServoChannel::setPulseUs(uint16_t pulseUs)
{
  pulseUs = static_cast<uint16_t>(
      std::max(static_cast<uint32_t>(kPulseMinUs),
               std::min(static_cast<uint32_t>(kPulseMaxUs),
                        static_cast<uint32_t>(pulseUs))));

  m_currentPulseUs = pulseUs;

  uint32_t compare = static_cast<uint32_t>(pulseUs) * m_ticksPerUs;
  __HAL_TIM_SET_COMPARE(m_htim, m_channel, compare);
}

void ServoChannel::setNormalised(float position)
{
  position = std::max(-1.0f, std::min(1.0f, position));

  // Map [-1, 1] → [kPulseMinUs, kPulseMaxUs]
  float rangeHalf = static_cast<float>(kPulseMaxUs - kPulseMinUs) / 2.0f;
  float pulse     = static_cast<float>(kPulseMidUs) + position * rangeHalf;

  setPulseUs(static_cast<uint16_t>(pulse));
}

uint16_t ServoChannel::currentPulseUs() const
{
  return m_currentPulseUs;
}

/* EOF -----------------------------------------------------------------------*/
