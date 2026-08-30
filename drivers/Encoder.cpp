/*******************************************************************************
 * @file Encoder.cpp
 * @brief Single-channel input-capture encoder driver implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Encoder.hpp"

/// Seconds-to-minutes conversion for the RPM calculation.
static constexpr float kSecondsPerMinute = 60.0f;

Encoder::Encoder(TIM_HandleTypeDef* htim,
                 uint32_t           channel,
                 uint32_t           pulsesPerRev,
                 uint32_t           timerClockHz)
  : m_htim(htim)
  , m_channel(channel)
  , m_pulsesPerRev((pulsesPerRev > 0U) ? pulsesPerRev : 1U)
  , m_timerClockHz(timerClockHz)
  , m_counterMask(0xFFFFU)
  , m_lastCapture(0U)
  , m_interval(0U)
  , m_pulseCount(0U)
  , m_lastCaptureMs(0U)
  , m_haveFirstCapture(false)
  , m_position(0)
  , m_lastPulseCount(0U)
  , m_velocityRpm(0.0f)
  , m_direction(1)
{
  // A zero pulsesPerRev would divide by zero in update(); clamp rather than
  // trusting every future caller to get it right.
}

bool Encoder::init()
{
  if (m_htim == nullptr)
  {
    return false;
  }

  // Capture deltas must be masked to the counter width or every rollover reads
  // as an enormous interval. TIM4 and TIM8 are both 16-bit on this part, but
  // derive it rather than assuming, so moving a channel to TIM2 or TIM5 does
  // not silently break the arithmetic.
#ifdef IS_TIM_32B_COUNTER_INSTANCE
  m_counterMask = IS_TIM_32B_COUNTER_INSTANCE(m_htim->Instance) ? 0xFFFFFFFFU : 0xFFFFU;
#else
  m_counterMask = 0xFFFFU;
#endif

  return (HAL_TIM_IC_Start_IT(m_htim, m_channel) == HAL_OK);
}

void Encoder::onCapture()
{
  // ISR context.
  const uint32_t now = HAL_TIM_ReadCapturedValue(m_htim, m_channel);

  if (m_haveFirstCapture)
  {
    // Unsigned subtraction then mask handles counter rollover correctly for
    // both 16-bit and 32-bit timers.
    m_interval = (now - m_lastCapture) & m_counterMask;
  }
  else
  {
    // No previous edge to measure against; record the timestamp and wait.
    m_haveFirstCapture = true;
    m_interval         = 0U;
  }

  m_lastCapture   = now;
  m_lastCaptureMs = HAL_GetTick();
  ++m_pulseCount;
}

void Encoder::setDirection(int8_t sign)
{
  if (sign > 0)
  {
    m_direction = 1;
  }
  else if (sign < 0)
  {
    m_direction = -1;
  }
  // sign == 0 deliberately holds the previous direction: a momentarily zero
  // command does not mean the shaft has stopped or reversed.
}

void Encoder::update()
{
  // Snapshot everything the ISR owns in one short critical section. Save and
  // restore PRIMASK rather than calling __enable_irq(), so this stays safe when
  // called from inside an outer critical section.
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  const uint32_t pulses    = m_pulseCount;
  const uint32_t interval  = m_interval;
  const uint32_t captureMs = m_lastCaptureMs;
  const bool     haveEdge  = m_haveFirstCapture;

  __set_PRIMASK(primask);

  // Accumulate position from the edges seen since the last call, signed by the
  // caller's direction. Unsigned subtraction is rollover-safe.
  const uint32_t delta = pulses - m_lastPulseCount;
  m_lastPulseCount     = pulses;
  m_position += static_cast<int32_t>(delta) * static_cast<int32_t>(m_direction);

  // No edge yet, or no edge recently: the shaft is stopped as far as we can
  // tell. Reporting the last known velocity forever would make a stalled wheel
  // look like it is still turning.
  if (!haveEdge || (interval == 0U) ||
      ((HAL_GetTick() - captureMs) > kStaleTimeoutMs))
  {
    m_velocityRpm = 0.0f;
    return;
  }

  // rev/s  = timerClockHz / (interval × pulsesPerRev)
  // rev/min = rev/s × 60
  const float ticksPerRev = static_cast<float>(interval)
                          * static_cast<float>(m_pulsesPerRev);
  const float rpm = (static_cast<float>(m_timerClockHz) * kSecondsPerMinute)
                  / ticksPerRev;

  m_velocityRpm = rpm * static_cast<float>(m_direction);
}

int32_t Encoder::count() const
{
  return m_position;
}

float Encoder::velocityRpm() const
{
  return m_velocityRpm;
}

void Encoder::resetCount()
{
  m_position = 0;
}

/* EOF -----------------------------------------------------------------------*/
