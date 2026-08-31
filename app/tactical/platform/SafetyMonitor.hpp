/*******************************************************************************
 * @file SafetyMonitor.hpp
 * @brief The failsafe gate — decides the robot's mode once per control cycle.
 *
 * SafetyMonitor is step 3 of the control cycle (§4). It runs 200 times a second,
 * before any behavior code, and its answer decides whether the cycle goes on to
 * run behaviors and subsystems or whether it calls SubsystemManager::disableAll()
 * and returns. It is the only place in the firmware that decides that question -
 * per §5.3 there is no second `if (m_enabled)` inside a subsystem.
 *
 * Two tiers, and the distinction is the whole design (§5.1):
 *
 *  - **Disabled** is unlatched. It is the normal resting state and it recovers by
 *    itself: the radio drops for 300 ms behind an obstacle, the robot coasts, the
 *    link returns, you keep driving. Nothing has to be cleared.
 *  - **Fault** is latched. It survives the cause going away and is released only
 *    by clearFault() (the console command) or a power cycle, because a motor
 *    driver reporting overcurrent is something a human should look at before the
 *    robot moves again.
 *
 * Every verdict carries the reason for it alongside the mode, so telemetry can
 * report "Disabled: link lost" rather than going quiet. reasonToString() gives
 * the text; the numbers that complete the sentence (`input.ageMs`,
 * `sensors.batteryMillivolts`) are in the same RobotContext the caller already
 * has, which is why SafetyVerdict carries no detail field of its own.
 *
 * NO HAL, NO DRIVER OBJECTS, NO HARDWARE READS. Invariant 3 (§2.1) keeps this
 * layer free of stm32h5xx_hal.h; §4 makes sense() the only code in the cycle that
 * reads hardware. So this class is given the SensorFrame and DriverInput that
 * steps 1 and 2 already filled, plus the cycle's own timing, and it reads nothing
 * for itself - not even HAL_GetTick(). That is also what lets it be exercised on
 * a host from synthetic inputs, which is how the §5.2 trigger table is verified.
 *
 * Two rows of the §5.2 table are deliberately NOT implemented here, because they
 * exist precisely to survive this code failing:
 *
 *  - **Control task stall** is the heartbeat task's liveness check and the IWDG
 *    (U0.7). A monitor called from the control task cannot detect that task
 *    wedging.
 *  - **Stack overflow / malloc failure** are FreeRTOS hooks in freertos_hooks.c;
 *    they record and reset rather than produce a verdict.
 *
 * @see docs/tactical_architecture.md §5 for the state machine and trigger table.
 * @see docs/tactical_architecture.md §4 for the control cycle that calls this.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include "RobotContext.hpp"

#include <cstdint>

/*******************************************************************************
 * @brief Evaluates the §5.2 failsafe table and drives the §5.1 mode machine.
 *
 * One instance, owned by the control task. Single-threaded by construction: only
 * the control task calls evaluate(), so no critical section is needed inside.
 * clearFault() is the one method another task (the console) calls - see its note.
 *
 * Usage, at step 3 of the cycle:
 *
 * @code
 * const SafetyVerdict verdict = m_safety.evaluate(ctx.nowMs, ctx.input,
 *                                                 ctx.sensors, lastCycleUs);
 * ctx.mode = verdict.mode;
 *
 * if (verdict.mode == ControlMode::Disabled || verdict.mode == ControlMode::Fault)
 * {
 *   m_manager.disableAll();
 *   publish();
 *   return;
 * }
 * @endcode
 ******************************************************************************/
class SafetyMonitor final
{
public:
  /* Thresholds --------------------------------------------------------------*/

  /**
   * @brief Age past which the radio link counts as lost, in milliseconds.
   *
   * Two CRSF frames at 50 Hz is 40 ms; 250 ms tolerates a burst of dropouts
   * without tolerating an unplugged receiver (§5.2).
   */
  static constexpr uint32_t kLinkTimeoutMs = 250U;

  /**
   * @brief Default pack voltage below which the robot faults, in millivolts.
   *
   * 9.9 V is the 3S LiPo cutoff quoted in drivers/Battery.hpp - 3.3 V/cell.
   *
   * @note Deliberately NOT Battery::kLowBatteryThresholdMv (10 500 mV). That is
   *       a 3.5 V/cell *warning*, and a hard launch drags a healthy pack below
   *       it routinely; faulting there would stop the robot mid-match on a good
   *       battery. Warning and cutoff are two different numbers.
   *
   * @todo Phase 2 moves this into ParamStore; §5.2 calls the cutoff a stored
   *       parameter. Until then it is a constructor argument.
   */
  static constexpr uint32_t kDefaultBatteryCutoffMv = 9900U;

  /**
   * @brief How long the pack must stay below cutoff before it is a fault, in ms.
   *
   * Debounced because a hard launch dips the rail for milliseconds and that is
   * not a flat pack (§5.2).
   */
  static constexpr uint32_t kBatterySagDebounceMs = 200U;

  /**
   * @brief Control cycle execution time that counts as an overrun, in µs.
   *
   * The cycle period is 5 ms; 4 ms of work in it means the schedule is nearly
   * gone (§5.2).
   */
  static constexpr uint32_t kCycleOverrunUs = 4000U;

  /**
   * @brief Consecutive overruns that latch a fault.
   *
   * One long cycle is a hiccup. Ten in a row means nothing derived from `dt` can
   * be trusted any more, which is the actual reason this is a fault rather than
   * a telemetry counter.
   */
  static constexpr uint8_t kOverrunFaultCount = 10U;

  /* Nested types ------------------------------------------------------------*/

  /**
   * @brief Why the robot is in the mode it is in.
   *
   * Reported alongside every verdict, including the permissive ones. Telemetry
   * turns it into text with reasonToString().
   *
   * @note The motor fault is split per channel rather than carried as an index
   *       in SafetyVerdict, because §5.2 requires the channel to be named and
   *       SafetyVerdict is frozen at two fields. If SensorFrame::kMotorCount
   *       ever grows past two, this enum grows with it.
   */
  enum class Reason : uint8_t
  {
    None = 0U,             ///< No restriction. Paired with Teleop or Auto.
    SelfTestFailed,        ///< A subsystem's onInit() returned false at boot. Latched.
    MotorFaultLeft,        ///< DRV8874 nFAULT on drive channel 0 (left). Latched.
    MotorFaultRight,       ///< DRV8874 nFAULT on drive channel 1 (right). Latched.
    BatterySag,            ///< Pack below cutoff for kBatterySagDebounceMs. Latched.
    ControlOverrun,        ///< kOverrunFaultCount cycles over kCycleOverrunUs. Latched.
    LinkNeverEstablished,  ///< No CRSF frame has ever arrived. Unlatched.
    LinkLost,              ///< Last frame older than kLinkTimeoutMs. Unlatched.
    DriverDisabled         ///< The driver's enable switch is not held. Unlatched.
  };

  /**
   * @brief The gate's answer for one control cycle.
   *
   * Frozen by unit U0.5: exactly a mode and a reason. Anything a message needs
   * beyond the reason - an age in milliseconds, a pack voltage - is already in
   * the RobotContext the caller holds, so it is not duplicated here.
   */
  struct SafetyVerdict
  {
    ControlMode mode   = ControlMode::Disabled; ///< The mode this cycle runs in.
    Reason      reason = Reason::LinkNeverEstablished; ///< Why that mode, for telemetry.
  };

  /* Construction ------------------------------------------------------------*/

  /**
   * @brief Construct the monitor in its resting state.
   *
   * Starts Disabled with Reason::LinkNeverEstablished - no frame has arrived and
   * no self-test has been reported, so the first verdict before either happens
   * is the safe one rather than a plausible-looking Teleop.
   *
   * @param batteryCutoffMv Pack voltage below which the robot faults, in mV.
   *                        Defaults to kDefaultBatteryCutoffMv.
   */
  explicit SafetyMonitor(uint32_t batteryCutoffMv = kDefaultBatteryCutoffMv);

  /* Operations --------------------------------------------------------------*/

  /**
   * @brief Record the boot self-test result (§5.1, Boot → SelfTest → …).
   *
   * Called once, after SubsystemManager::initAll(), before the control task
   * starts cycling. A false result latches Fault with Reason::SelfTestFailed, so
   * a subsystem that failed to come up cannot be driven by a later enable.
   *
   * @param passed The value SubsystemManager::initAll() returned.
   *
   * @note Passing true does nothing. It is not an arming step - the monitor is
   *       already in its resting state - and calling it is optional for a caller
   *       that has no subsystems to test.
   */
  void reportSelfTest(bool passed);

  /**
   * @brief Evaluate the whole §5.2 trigger table and return this cycle's mode.
   *
   * Latching triggers are tested first, then the unlatched ones, then the §5.1
   * state machine decides between Teleop and Auto. Once a latching trigger has
   * fired, every later call returns that same verdict until clearFault().
   *
   * @param nowMs       Milliseconds since boot, sampled once at cycle start
   *                    (RobotContext::nowMs). Passed in rather than read here so
   *                    this class touches no HAL - see the file note.
   * @param input       The driver's last decoded frame and its age (step 2).
   * @param sensors     Every sensor value read this cycle (step 1).
   * @param cycleTimeUs Execution time of the *previous* control cycle in
   *                    microseconds, from the DWT cycle counter. Pass 0 on the
   *                    first cycle, before a measurement exists.
   *
   * @return The mode this cycle must run in, and the reason for it.
   *
   * @note Battery sag is judged from sensors.batteryMillivolts and gated on
   *       sensors.batteryValid, which is the SensorFrame mirror of
   *       Battery::isValid(). That is the same boot suppression
   *       Battery::isLow() applies internally, consumed rather than duplicated:
   *       ignoring the flag latches a brownout on every power-up, because the
   *       DMA buffer reads a perfectly plausible 0 mV until the scan runs.
   */
  SafetyVerdict evaluate(uint32_t           nowMs,
                         const DriverInput& input,
                         const SensorFrame& sensors,
                         uint32_t           cycleTimeUs);

  /**
   * @brief Release the latched fault and return to Disabled (§5.1).
   *
   * Also resets the battery-sag debounce and the overrun counter, so a cleared
   * fault has to be re-earned from scratch rather than re-firing on one stale
   * sample.
   *
   * @note This clears *this* monitor's latch only. A DRV8874 fault is also
   *       latched inside Motor, and Motor::clearFault() does not clear a fault
   *       that is still physically asserted - so a console clear that does not
   *       also clear the motor re-latches on the very next cycle. That is the
   *       intended behaviour: the cause has to go away, not just the report.
   *
   * @note Called from the console task, not the control task. It is two stores
   *       and cannot leave a half-cleared state that changes a verdict: the
   *       worst interleaving costs one extra cycle of Fault.
   */
  void clearFault();

  /* Queries -----------------------------------------------------------------*/

  /**
   * @brief Report whether a fault is latched.
   *
   * @return true if evaluate() will return Fault until clearFault() is called.
   */
  bool isFaultLatched() const;

  /**
   * @brief Render a reason as fixed text for telemetry and the console.
   *
   * @param reason Any Reason value.
   * @return A static string, never nullptr. An unknown value returns "unknown".
   *
   * @note Deliberately not a full sentence. The caller adds the measurement that
   *       completes it - "Disabled: link lost (312 ms)".
   */
  static const char* reasonToString(Reason reason);

private:
  /**
   * @brief Advance the battery sag debounce for this cycle.
   *
   * @param nowMs   Cycle timestamp in milliseconds.
   * @param sensors This cycle's sensor frame.
   * @return true once the pack has been below cutoff for kBatterySagDebounceMs.
   */
  bool updateBatterySag(uint32_t nowMs, const SensorFrame& sensors);

  /**
   * @brief Advance the consecutive-overrun counter for this cycle.
   *
   * @param cycleTimeUs Previous cycle's execution time in microseconds.
   * @return true once kOverrunFaultCount consecutive cycles have overrun.
   */
  bool updateOverrun(uint32_t cycleTimeUs);

  /**
   * @brief Latch a fault, unless one is already latched.
   *
   * First cause wins, so the reported reason is the one that actually stopped
   * the robot rather than whatever fired last.
   *
   * @param reason The trigger that fired.
   */
  void latchFault(Reason reason);

  uint32_t    m_batteryCutoffMv;   ///< Pack voltage below which the robot faults (mV).
  ControlMode m_mode;              ///< Mode the previous evaluate() returned.
  Reason      m_faultReason;       ///< Reason the latch holds; None when unlatched.
  bool        m_faultLatched;      ///< True once a latching trigger has fired.
  bool        m_batterySagging;    ///< True while the pack is below cutoff.
  uint32_t    m_batterySagStartMs; ///< nowMs at which the current sag began.
  uint8_t     m_overrunCount;      ///< Consecutive overrunning cycles, capped.
};

/* EOF -----------------------------------------------------------------------*/
