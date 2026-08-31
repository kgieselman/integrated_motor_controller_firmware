/*******************************************************************************
 * @file RobotContext.hpp
 * @brief Core value types shared by every layer above the driver layer.
 *
 * Defines the four types the tactical application is built on:
 *
 *  - ControlMode  - which of the four operating states the robot is in.
 *  - SensorFrame  - every sensor reading taken during one control cycle.
 *  - DriverInput  - the last decoded CRSF frame, as intent rather than counts.
 *  - RobotContext - the bundle handed to each behavior and subsystem per cycle.
 *
 * These are frozen interfaces: unit U0.1 in docs/work_units.md defines them and
 * every later unit consumes them. See docs/tactical_architecture.md §4 for the
 * control cycle that fills a RobotContext, and §5 for the mode state machine.
 *
 * NO HAL TYPES APPEAR HERE, DELIBERATELY. Invariant 3 (§2.1) keeps layer 3 and
 * above free of stm32h5xx_hal.h, which is what lets control and subsystem code
 * compile and unit-test on a host. Everything below is a plain value. Exactly
 * two places convert a driver reading into one: the sense() step fills
 * SensorFrame (§4 step 1), and InputSource fills DriverInput from the comms
 * task (§3.1). Nothing else in the cycle converts anything.
 *
 * Every type here is trivially copyable so it can live in a Snapshot<T> (§3.2)
 * and be copied whole inside a critical section. The static_asserts at the foot
 * of the file enforce that - do not add a user-declared constructor, a virtual
 * method or a non-trivial member to any of them.
 *
 * @note DriverInput's default member initialisers are load-bearing, not
 *       cosmetic. A default-constructed DriverInput reports a link that has
 *       never been heard from, sticks centred and every switch off. Snapshot<T>
 *       value-initialises its storage, so a reader that runs before the comms
 *       task has ever written sees "no radio" rather than a plausible-looking
 *       fresh frame with an age of 0 ms.
 *
 * @note SensorFrame's defaults make no such claim. sense() overwrites every
 *       field before anything reads it, so they are placeholders rather than
 *       safe states - motorFaulted in particular defaults to the permissive
 *       false, not to a fault nobody has detected yet.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include <cstdint>
#include <type_traits>

/*******************************************************************************
 * @brief The robot's operating state for one control cycle.
 *
 * Decided by SafetyMonitor at step 3 of every cycle and carried in
 * RobotContext::mode. Two tiers, per docs/tactical_architecture.md §5.1:
 * Disabled is the normal resting state and recovers by itself, Fault is latched
 * until a console clear or a power cycle.
 *
 * @note The Boot and SelfTest states drawn in §5.1 are not listed here. They
 *       exist only before the scheduler starts, so no control cycle - and
 *       therefore no RobotContext - is ever produced while the robot is in one.
 *
 * @note Disabled is deliberately zero, so that a zero-initialised or otherwise
 *       uninitialised mode field reads as the safe state rather than as Teleop.
 ******************************************************************************/
enum class ControlMode : uint8_t
{
  Disabled = 0U, ///< Resting state. Subsystems get onDisable() every cycle. Unlatched.
  Teleop   = 1U, ///< Driver in control through the active teleop behavior.
  Auto     = 2U, ///< An autonomous routine is driving the subsystems.
  Fault    = 3U  ///< Latched fault. Motors coast until cleared or power-cycled.
};

/*******************************************************************************
 * @brief Every input read during one control cycle, as plain values.
 *
 * Filled by the sense() step (§4, step 1), which is the only code in the
 * firmware permitted to read sensor hardware during a cycle. Reading each input
 * exactly once per cycle is what makes the rest of the cycle self-consistent:
 * two subsystems asking the same encoder for its velocity get the same answer.
 *
 * Units are part of this contract. sense() converts; nothing downstream does.
 *
 * ONE INDEX MEANS ONE DRIVE CHANNEL. Element i of encoderCounts,
 * encoderVelocityRpm, motorCurrentMa and motorFaulted all describe the same
 * side of the robot: that motor, the wheel it drives, and its current sense.
 * Channel 0 is the left side and channel 1 the right - not a choice made here,
 * but one the driver layer already fixed, since AdcDma.hpp defines
 * kSlotLeftIpropi as slot 0 and Motor.hpp binds motor 0 to that slot.
 * SafetyMonitor names a faulted channel by this index and DriveBase pairs a
 * Motor with an Encoder by it, so the pairing is contractual, not incidental.
 ******************************************************************************/
struct SensorFrame
{
  /// Drive channel indices, shared by every per-channel array below.
  static constexpr uint8_t kChannelLeft  = 0U;
  static constexpr uint8_t kChannelRight = 1U;

  /// H-bridge motor channels on the board. See drivers/Motor.hpp.
  static constexpr uint8_t kMotorCount = 2U;

  /// Encoder input-capture channels, one per wheel. See drivers/Encoder.hpp.
  static constexpr uint8_t kEncoderCount = 2U;

  /**
   * @brief Current reading with no measurement behind it yet.
   *
   * Mirrors Motor::kCurrentUnavailable. Duplicated rather than included because
   * Motor.hpp pulls in the HAL, which invariant 3 forbids at this layer. A
   * reading of 0 mA is entirely plausible, so the distinction matters.
   */
  static constexpr int32_t kCurrentUnavailableMa = -1;

  /**
   * @brief Highest current the sense chain can represent, in milliamps.
   *
   * Mirrors Motor::kCurrentFullScaleMa, duplicated for the same reason as
   * kCurrentUnavailableMa. Mirrored rather than left dangling because the
   * consumer that needs it - stall detection - is forbidden from including
   * Motor.hpp to reach the original.
   */
  static constexpr int32_t kCurrentFullScaleMa = 4125;

  /**
   * @brief A three-axis measurement in the IMU's frame.
   *
   * Nested rather than global: it is a field shape, not a vector-maths type,
   * and it deliberately carries no operators.
   */
  struct Vec3
  {
    float x = 0.0f; ///< X axis.
    float y = 0.0f; ///< Y axis.
    float z = 0.0f; ///< Z axis.
  };

  /// Accumulated signed encoder position, in capture edges, per channel.
  /// Signed by the last commanded direction - see Encoder.hpp on why that is
  /// good enough to close a velocity loop and not good enough for odometry.
  int32_t encoderCounts[kEncoderCount] = {0, 0};

  /// Shaft velocity per channel, in revolutions per minute, signed as above.
  /// Reads exactly zero when no edge has arrived for Encoder::kStaleTimeoutMs,
  /// which a stopped wheel and a disconnected encoder both produce. No validity
  /// flag is carried for this yet; a closed-loop consumer that needs to tell
  /// them apart must add one here rather than guess downstream.
  float encoderVelocityRpm[kEncoderCount] = {0.0f, 0.0f};

  Vec3 accelG{};   ///< Linear acceleration, g.
  Vec3 gyroDps{};  ///< Angular rate, degrees per second.

  /// Battery voltage in millivolts. Zero, and batteryValid false, until the
  /// ADC1 DMA scan is running. Nominal pack is 3S LiPo - see Battery.hpp.
  uint32_t batteryMillivolts = 0U;

  /// True once the battery reading can be trusted. Mirrors Battery::isValid().
  /// A failsafe that ignores this latches a brownout on every power-up.
  bool batteryValid = false;

  /// Per-channel motor current in milliamps, or kCurrentUnavailableMa.
  /// MAGNITUDE ONLY - IPROPI cannot report direction, so this is never negative
  /// except for the sentinel; take the sign from the commanded duty instead.
  /// Saturates at kCurrentFullScaleMa (about 4.1 A): a reading at full scale
  /// means "at least this much", not a measurement.
  int32_t motorCurrentMa[kMotorCount] = {kCurrentUnavailableMa, kCurrentUnavailableMa};

  /// Per-channel DRV8874 fault, true if asserted now or latched since the last
  /// clear. Any true entry is a latching Fault trigger (§5.2).
  bool motorFaulted[kMotorCount] = {false, false};
};

/*******************************************************************************
 * @brief The driver's last decoded command, plus how old it is.
 *
 * Written by InputSource (U0.4) into a Snapshot<DriverInput> from the comms
 * task, and copied out once per control cycle at step 2. Deadband, expo curve
 * and switch decode have already been applied, so the control task sees intent
 * rather than channel counts.
 *
 * Axes are normalised to [-1, 1] and centred at zero. A frame that has aged
 * past the radio-loss timeout (§5.2) is stale, whatever its axes say - check
 * ageMs before believing any of them.
 ******************************************************************************/
struct DriverInput
{
  /// Auxiliary analogue axes beyond throttle and steering, for mechanisms.
  /// Mechanisms are additive year over year; spare axes cost four floats.
  static constexpr uint8_t kAuxAxisCount = 4U;

  /**
   * @brief ageMs value meaning no frame has ever been received.
   *
   * The default, so that a DriverInput read before the comms task has ever
   * written one fails every freshness test rather than passing them all.
   *
   * @note InputSource has to produce this value itself; it cannot come from
   *       CRSFReceiver. That driver returns HAL_GetTick() - m_lastFrameTick
   *       with the tick zero-initialised, so before the first frame it reports
   *       uptime - a large number, but never this one. U0.4 must therefore
   *       track "at least one frame has arrived" and stamp this sentinel until
   *       it has. The failsafe fires either way, since uptime passes the 250 ms
   *       threshold almost at once; what is lost without the sentinel is
   *       telemetry's ability to say "no receiver fitted" rather than
   *       "link lost".
   */
  static constexpr uint32_t kAgeNeverReceived = 0xFFFFFFFFU;

  /// Driver enable switch, held on to permit motion. The driver's kill switch.
  static constexpr uint16_t kSwitchEnable = 1U << 0U;

  /// Autonomous request switch. Entered from Disabled only (§5.1).
  static constexpr uint16_t kSwitchAuto = 1U << 1U;

  /**
   * @brief Test one or more switch bits.
   *
   * @param mask One of the kSwitch* constants, or several OR'd together.
   * @return true if ANY bit in @p mask is set - never all of them. Passing
   *         `kSwitchEnable | kSwitchAuto` asks "is either held", not "are
   *         both"; test the two separately when you mean both.
   *
   * @note The one piece of implementation in this header, which is header-only
   *       by mandate (U0.1 creates no .cpp), so there is nowhere else for it to
   *       live. It exists so that no consumer open-codes a mask test against a
   *       frozen bit layout.
   */
  constexpr bool isSwitchSet(uint16_t mask) const
  {
    return (switches & mask) != 0U;
  }

  float throttle = 0.0f; ///< Forward / reverse intent, [-1, 1]. Positive is forward.
  float steering = 0.0f; ///< Turn intent, [-1, 1]. Positive is clockwise from above.

  /// Auxiliary axes, [-1, 1]. Meaning is assigned by the robot config, not here.
  float aux[kAuxAxisCount] = {};

  /// Decoded switch positions, one bit each. Test with the kSwitch* masks.
  /// InputSource owns the CRSF channel to bit mapping; channel 5 is the enable
  /// switch (docs/work_units.md U0.4).
  uint16_t switches = 0U;

  /// Milliseconds since the frame these values came from was received, or
  /// kAgeNeverReceived before any frame has arrived. Derived from
  /// CRSFReceiver::lastFrameAgeMs() once one has - see kAgeNeverReceived for
  /// why it is not simply that call's return value.
  uint32_t ageMs = kAgeNeverReceived;
};

/*******************************************************************************
 * @brief Everything a behavior or subsystem may know about the current cycle.
 *
 * Constructed once per cycle by the control task and passed by const reference
 * down to behaviors and subsystems. It is the whole of their world: a subsystem
 * that needs something not in here is reaching for hardware it does not own.
 *
 * @see docs/tactical_architecture.md §4 for the seven steps that fill it.
 ******************************************************************************/
struct RobotContext
{
  float       dt    = 0.0f;                  ///< Seconds since the last cycle; nominally 0.005f.
  uint32_t    nowMs = 0U;                    ///< HAL_GetTick() sampled once, at cycle start.
  ControlMode mode  = ControlMode::Disabled; ///< Disabled / Teleop / Auto / Fault.
  SensorFrame sensors{};                     ///< Every sensor value read this cycle.
  DriverInput input{};                       ///< Last decoded CRSF frame, plus its age in ms.
};

/* Contract checks -----------------------------------------------------------*/

// Snapshot<T> copies its payload whole inside a critical section, and the
// control task copies a RobotContext by value every cycle. Both require these
// types to stay trivially copyable; a constructor or a virtual method added
// later would break that silently, so it is checked here instead.
static_assert(std::is_trivially_copyable_v<SensorFrame>,
              "SensorFrame must stay trivially copyable - see Snapshot<T>, §3.2");
static_assert(std::is_trivially_copyable_v<DriverInput>,
              "DriverInput must stay trivially copyable - it lives in a Snapshot<T>");
static_assert(std::is_trivially_copyable_v<RobotContext>,
              "RobotContext must stay trivially copyable - it is copied per cycle");

// Aggregate initialisation, and designated initialisers in particular, are what
// the config and test layers use to build these by hand (§2.2).
static_assert(std::is_aggregate_v<SensorFrame>, "SensorFrame must stay an aggregate");
static_assert(std::is_aggregate_v<DriverInput>, "DriverInput must stay an aggregate");
static_assert(std::is_aggregate_v<RobotContext>, "RobotContext must stay an aggregate");

/* EOF -----------------------------------------------------------------------*/
