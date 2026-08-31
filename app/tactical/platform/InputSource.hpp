/*******************************************************************************
 * @file InputSource.hpp
 * @brief Turns raw CRSF channels into a DriverInput and publishes it.
 *
 * Owns the CRSF channel -> DriverInput mapping: deadband, expo shaping and
 * switch decode. Runs from the Comms task (docs/tactical_architecture.md
 * §3.1), which calls update() at a steady rate; the Control task never talks
 * to a CRSFReceiver directly, only to the Snapshot<DriverInput> this class
 * writes into.
 *
 * CHANNEL MAPPING — this unit's own decision, not fixed elsewhere. Channel 5
 * is the agreed enable switch (docs/work_units.md U0.4); nothing else pins a
 * channel to a purpose, so this class assigns channel 0 to throttle, channel
 * 1 to steering, and the remaining unassigned channels (2, 3, 4, 6) to the
 * four DriverInput::aux axes in index order, skipping channel 5. No channel
 * is assigned to DriverInput::kSwitchAuto yet — Auto mode has no consumer
 * before phase 2, and guessing a channel for it now risks colliding with
 * whatever a later unit picks. Revisit both choices in config/Robot<year>.hpp
 * once real hardware and a real radio profile exist to check them against.
 *
 * FIRST-FRAME SENTINEL. CRSFReceiver::lastFrameAgeMs() reports uptime before
 * the first frame has ever arrived, not DriverInput::kAgeNeverReceived — see
 * the @note on that constant in RobotContext.hpp. This class tracks "at least
 * one frame received" itself, using CRSFReceiver::hasNewData(), and stamps
 * the sentinel until that has happened once.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include "CRSFReceiver.hpp"
#include "RobotContext.hpp"
#include "Snapshot.hpp"

#include <cstdint>

/**
 * @brief Decodes CRSF channels into a DriverInput and publishes it into a Snapshot.
 *
 * Construct once, against the Comms task's CRSFReceiver and DriverInput
 * Snapshot, then call update() every Comms task cycle:
 *
 * @code
 *   static Snapshot<DriverInput> g_driverInput;
 *   CRSFReceiver crsfReceiver(&huart4);
 *   InputSource  inputSource(crsfReceiver, g_driverInput);
 *
 *   // Comms task body:
 *   inputSource.update();
 * @endcode
 */
class InputSource final
{
public:
  /// Deadband half-width around centre-stick, as a fraction of full travel.
  /// A raw axis inside [-kDeadband, kDeadband] reports exactly zero; the
  /// remaining travel is rescaled to still reach ±1 at full stick.
  static constexpr float kDeadband = 0.05f;

  /// Cubic expo blend factor: 0.0 is linear, 1.0 is a pure cube. Applied after
  /// the deadband rescale, so it always sees the full [-1, 1] range.
  static constexpr float kExpoFactor = 0.35f;

  /// CRSF channel index carrying throttle intent.
  static constexpr uint8_t kChannelThrottle = 0U;

  /// CRSF channel index carrying steering intent.
  static constexpr uint8_t kChannelSteering = 1U;

  /// CRSF channel indices carrying the four DriverInput aux axes, in order.
  /// Channel 5 (the enable switch) is skipped.
  static constexpr uint8_t kChannelAux[DriverInput::kAuxAxisCount] = {2U, 3U, 4U, 6U};

  /// CRSF channel index carrying the driver enable switch (docs/work_units.md U0.4).
  static constexpr uint8_t kChannelEnable = 5U;

  /// PWM-microsecond threshold above which a two-position switch channel
  /// reads as "high". Midpoint of CRSFReceiver's [1000, 2000] µs range.
  static constexpr uint16_t kSwitchThresholdUs = 1500U;

  /**
   * @brief Construct an InputSource over a receiver and its output snapshot.
   *
   * @param receiver Decoded CRSF channel source. update() drains and reads
   *                  it; InputSource does not touch HAL or own the UART.
   * @param sink      Snapshot to publish the decoded DriverInput into, once
   *                  per update(). Owned by the caller, not by InputSource.
   */
  explicit InputSource(CRSFReceiver& receiver, Snapshot<DriverInput>& sink);

  /**
   * @brief Drain the receiver, decode one DriverInput, and publish it.
   *
   * Always writes, even when no new frame arrived this call: ageMs still
   * needs to grow so a stale link is visible to whoever reads the snapshot.
   * Call from the Comms task at a steady rate, per
   * docs/tactical_architecture.md §3.1.
   */
  void update();

private:
  /**
   * @brief Apply the deadband rescale then the expo curve to one raw axis.
   *
   * @param raw Normalised axis value, [-1, 1], centred at zero.
   * @return Shaped value, [-1, 1], centred at zero.
   */
  static float applyDeadbandExpo(float raw);

  /**
   * @brief Read one channel as a two-position switch.
   *
   * @param channel CRSF channel index (0-15).
   * @return true if the channel's PWM-microsecond value is above kSwitchThresholdUs.
   */
  bool isChannelHigh(uint8_t channel) const;

  CRSFReceiver&          m_receiver;     ///< Decoded CRSF channel source; not owned.
  Snapshot<DriverInput>& m_sink;         ///< Publish target; not owned.
  bool                   m_everReceived; ///< True once at least one CRSF frame has arrived.
};

/* EOF -----------------------------------------------------------------------*/
