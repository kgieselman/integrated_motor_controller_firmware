/*******************************************************************************
 * @file SafetyMonitor.cpp
 * @brief The failsafe gate — decides the robot's mode once per control cycle.
 *
 * Implements the §5.2 trigger table and the §5.1 state machine. The order the
 * triggers are tested in is the whole of the safety policy, so it is written out
 * once, in evaluate(), rather than spread across helpers.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "platform/SafetyMonitor.hpp"

/* Construction --------------------------------------------------------------*/

SafetyMonitor::SafetyMonitor(uint32_t batteryCutoffMv)
    : m_batteryCutoffMv(batteryCutoffMv)
    , m_mode(ControlMode::Disabled)
    , m_faultReason(Reason::None)
    , m_faultLatched(false)
    , m_batterySagging(false)
    , m_batterySagStartMs(0U)
    , m_overrunCount(0U)
{
}

/* Operations ----------------------------------------------------------------*/

void SafetyMonitor::reportSelfTest(bool passed)
{
  if (!passed)
  {
    latchFault(Reason::SelfTestFailed);
  }
}

SafetyMonitor::SafetyVerdict SafetyMonitor::evaluate(uint32_t           nowMs,
                                                     const DriverInput& input,
                                                     const SensorFrame& sensors,
                                                     uint32_t           cycleTimeUs)
{
  // A latched fault is the end of the conversation. Returning here also leaves
  // the debounce and overrun state frozen where it stood, which is why
  // clearFault() resets both - a cleared fault re-earns itself from scratch.
  if (m_faultLatched)
  {
    m_mode = ControlMode::Fault;
    return SafetyVerdict{.mode = ControlMode::Fault, .reason = m_faultReason};
  }

  // Both helpers carry per-cycle state, so both run every cycle regardless of
  // which one fires first. Short-circuiting them inside the `if` below would
  // stop the sag timer advancing whenever a motor fault was pending.
  const bool batterySagFault = updateBatterySag(nowMs, sensors);
  const bool overrunFault    = updateOverrun(cycleTimeUs);

  /* Latching triggers - §5.2, hard faults ---------------------------------- */

  // Physical causes first: a motor driver reporting overcurrent is the most
  // specific thing that can be wrong, and it is the one a human must look at.
  // Motor::isFaulted() has already OR'd the EXTI latch with the pin level, so
  // sensors.motorFaulted[] is true for a transient the poll would have missed.
  if (sensors.motorFaulted[SensorFrame::kChannelLeft])
  {
    latchFault(Reason::MotorFaultLeft);
  }
  else if (sensors.motorFaulted[SensorFrame::kChannelRight])
  {
    latchFault(Reason::MotorFaultRight);
  }
  else if (batterySagFault)
  {
    latchFault(Reason::BatterySag);
  }
  else if (overrunFault)
  {
    latchFault(Reason::ControlOverrun);
  }

  if (m_faultLatched)
  {
    m_mode = ControlMode::Fault;
    return SafetyVerdict{.mode = ControlMode::Fault, .reason = m_faultReason};
  }

  /* Unlatched triggers - §5.2, ordinary Disabled --------------------------- */

  // Link before switch: with no radio there is no switch reading to believe.
  if (input.ageMs == DriverInput::kAgeNeverReceived)
  {
    m_mode = ControlMode::Disabled;
    return SafetyVerdict{.mode   = ControlMode::Disabled,
                         .reason = Reason::LinkNeverEstablished};
  }

  if (input.ageMs > kLinkTimeoutMs)
  {
    m_mode = ControlMode::Disabled;
    return SafetyVerdict{.mode = ControlMode::Disabled, .reason = Reason::LinkLost};
  }

  if (!input.isSwitchSet(DriverInput::kSwitchEnable))
  {
    m_mode = ControlMode::Disabled;
    return SafetyVerdict{.mode = ControlMode::Disabled, .reason = Reason::DriverDisabled};
  }

  /* Permissive - §5.1 state machine ---------------------------------------- */

  // Auto is entered from Disabled only. Once running it is not left by releasing
  // the auto switch: a routine mid-arc would then hand a bumped switch straight
  // control of the sticks. The way out of Auto is the way out of everything -
  // release enable, which is the driver's kill switch, and Disabled recovers by
  // itself. §5.1 draws exactly these edges.
  if (m_mode == ControlMode::Auto)
  {
    return SafetyVerdict{.mode = ControlMode::Auto, .reason = Reason::None};
  }

  if ((m_mode == ControlMode::Disabled) && input.isSwitchSet(DriverInput::kSwitchAuto))
  {
    m_mode = ControlMode::Auto;
    return SafetyVerdict{.mode = ControlMode::Auto, .reason = Reason::None};
  }

  m_mode = ControlMode::Teleop;
  return SafetyVerdict{.mode = ControlMode::Teleop, .reason = Reason::None};
}

void SafetyMonitor::clearFault()
{
  m_faultLatched      = false;
  m_faultReason       = Reason::None;
  m_batterySagging    = false;
  m_batterySagStartMs = 0U;
  m_overrunCount      = 0U;
  m_mode              = ControlMode::Disabled;
}

/* Queries -------------------------------------------------------------------*/

bool SafetyMonitor::isFaultLatched() const
{
  return m_faultLatched;
}

const char* SafetyMonitor::reasonToString(Reason reason)
{
  switch (reason)
  {
    case Reason::None:                 return "ok";
    case Reason::SelfTestFailed:       return "self-test failed";
    case Reason::MotorFaultLeft:       return "motor fault (left)";
    case Reason::MotorFaultRight:      return "motor fault (right)";
    case Reason::BatterySag:           return "battery sag";
    case Reason::ControlOverrun:       return "control overrun";
    case Reason::LinkNeverEstablished: return "no receiver";
    case Reason::LinkLost:             return "link lost";
    case Reason::DriverDisabled:       return "driver disabled";
    default:                           return "unknown";
  }
}

/* Private helpers -----------------------------------------------------------*/

bool SafetyMonitor::updateBatterySag(uint32_t nowMs, const SensorFrame& sensors)
{
  // batteryValid is the SensorFrame mirror of Battery::isValid(). Until the ADC1
  // DMA scan is running the buffer reads a plausible 0 mV, so a failsafe that
  // ignores this flag latches a brownout on every power-up.
  const bool belowCutoff = sensors.batteryValid
                        && (sensors.batteryMillivolts < m_batteryCutoffMv);

  if (!belowCutoff)
  {
    m_batterySagging = false;
    return false;
  }

  if (!m_batterySagging)
  {
    m_batterySagging    = true;
    m_batterySagStartMs = nowMs;
    return false;
  }

  // Unsigned subtraction, so this stays correct across the 49-day tick wrap.
  return (nowMs - m_batterySagStartMs) >= kBatterySagDebounceMs;
}

bool SafetyMonitor::updateOverrun(uint32_t cycleTimeUs)
{
  if (cycleTimeUs <= kCycleOverrunUs)
  {
    m_overrunCount = 0U;
    return false;
  }

  // Saturate rather than wrap: an 8-bit counter that rolls over would clear the
  // fault condition every 256 overrunning cycles.
  if (m_overrunCount < kOverrunFaultCount)
  {
    ++m_overrunCount;
  }

  return m_overrunCount >= kOverrunFaultCount;
}

void SafetyMonitor::latchFault(Reason reason)
{
  // First cause wins. The reported reason is the one that actually stopped the
  // robot, not whatever else went wrong in the cycles after it stopped.
  if (!m_faultLatched)
  {
    m_faultLatched = true;
    m_faultReason  = reason;
  }

  m_mode = ControlMode::Fault;
}

/* EOF -----------------------------------------------------------------------*/
