/*******************************************************************************
 * @file InputSource.cpp
 * @brief Turns raw CRSF channels into a DriverInput and publishes it.
 ******************************************************************************/

#include "InputSource.hpp"

InputSource::InputSource(CRSFReceiver& receiver, Snapshot<DriverInput>& sink)
    : m_receiver(receiver)
    , m_sink(sink)
    , m_everReceived(false)
{}

void InputSource::update()
{
  // Drain the DMA ring and parse whatever complete frames are sitting in it.
  // hasNewData() below reflects frames decoded by this call, not a stale flag
  // from before it.
  m_receiver.update();

  if (m_receiver.hasNewData())
  {
    m_everReceived = true;
  }

  DriverInput input{};

  input.throttle = applyDeadbandExpo(m_receiver.channelNorm(kChannelThrottle));
  input.steering = applyDeadbandExpo(m_receiver.channelNorm(kChannelSteering));

  for (uint8_t i = 0U; i < DriverInput::kAuxAxisCount; ++i)
  {
    input.aux[i] = applyDeadbandExpo(m_receiver.channelNorm(kChannelAux[i]));
  }

  uint16_t switches = 0U;
  if (isChannelHigh(kChannelEnable))
  {
    switches |= DriverInput::kSwitchEnable;
  }
  // kSwitchAuto is left unset - no channel is assigned to it yet, see the
  // class-level @note in InputSource.hpp.
  input.switches = switches;

  // Before the first frame ever arrives, CRSFReceiver::lastFrameAgeMs() reads
  // as uptime, not "never received" - stamp the sentinel ourselves until
  // m_everReceived flips, per the @note on DriverInput::kAgeNeverReceived.
  input.ageMs = m_everReceived ? m_receiver.lastFrameAgeMs() : DriverInput::kAgeNeverReceived;

  m_sink.write(input);
}

float InputSource::applyDeadbandExpo(float raw)
{
  const float magnitude = (raw < 0.0f) ? -raw : raw;

  if (magnitude < kDeadband)
  {
    return 0.0f;
  }

  const float sign     = (raw < 0.0f) ? -1.0f : 1.0f;
  const float rescaled = (magnitude - kDeadband) / (1.0f - kDeadband);
  const float clamped  = (rescaled > 1.0f) ? 1.0f : rescaled;

  // Standard cubic expo blend: preserves both endpoints (0 -> 0, 1 -> 1) and
  // is symmetric about zero by construction, since the sign was split out
  // above and only the magnitude is shaped here.
  const float shaped = (kExpoFactor * clamped * clamped * clamped) + ((1.0f - kExpoFactor) * clamped);

  return sign * shaped;
}

bool InputSource::isChannelHigh(uint8_t channel) const
{
  return m_receiver.channelUs(channel) > kSwitchThresholdUs;
}

/* EOF -----------------------------------------------------------------------*/
