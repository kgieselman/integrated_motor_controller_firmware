/*******************************************************************************
 * @file Subsystem.hpp
 * @brief Base class for every mechanism on the robot.
 *
 * Every mechanism - the drivetrain, a launcher, a scoop - implements these five
 * methods, and nothing else is required of it. The control task, the safety
 * monitor and the telemetry formatter know only this interface, which is what
 * makes adding a mechanism a change to three files rather than to the
 * scheduler (docs/tactical_architecture.md §6.4, invariant 4 of §2.1).
 *
 * This is the frozen contract of unit U0.3 in docs/work_units.md; the
 * signatures below are §6's verbatim. See §6.1 for why intent methods rather
 * than actuation, §6.2 for what a subsystem may and may not do, and §6.3 for
 * how the same subsystem carries an open- and a closed-loop path.
 *
 * NO HAL TYPES APPEAR HERE, DELIBERATELY. Invariant 3 keeps this base class and
 * SubsystemManager free of stm32h5xx_hal.h. A concrete subsystem owns drivers
 * and so does see HAL types in its own header; the contract they are dispatched
 * through does not, which is what lets the manager and the control cycle
 * compile and be tested on a host.
 *
 * @note Virtual dispatch here is a deliberate choice, not an oversight of the
 *       style guide's ban on virtual functions in performance-critical paths.
 *       A handful of indirect calls per 5 ms cycle is not one of those paths,
 *       and invariant 6 spends the cycles on clarity on purpose.
 *
 * @see docs/tactical_architecture.md §6
 * @see SubsystemManager.hpp for the array walk that drives these methods.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include "platform/RobotContext.hpp"

/*******************************************************************************
 * @brief Formatter that outgoing telemetry values are appended to.
 *
 * Forward-declared, not defined: the telemetry layer is phase 2 work and this
 * unit deliberately does not invent it. A reference to an incomplete type is
 * all publishTelemetry() needs, so nothing here is blocked on that design.
 ******************************************************************************/
class TelemetrySink;

/*******************************************************************************
 * @brief The contract every mechanism on the robot implements.
 *
 * Five methods, no exceptions - that sameness is the whole point. The safety
 * monitor disables a launcher correctly on link loss without knowing what a
 * launcher is, because disabling is a method on this contract rather than a
 * case in a switch statement.
 *
 * Lifecycle, per cycle: the control task builds a RobotContext, asks
 * SafetyMonitor for a mode, and then calls SubsystemManager::periodic() or
 * SubsystemManager::disableAll(). A subsystem therefore gets exactly one of
 * onPeriodic() and onDisable() every cycle, and never both. It must not decide
 * for itself which of the two applies: "safe is a state, not a check" (§5.3)
 * means there is no `if (enabled)` inside a subsystem, because the caller has
 * already made that decision for every subsystem at once.
 *
 * A subsystem owns its actuators exclusively (invariant 2) and is the only code
 * in the firmware that writes them. Behaviors call named intent methods on it -
 * `arcade(fwd, turn)`, `spinUp(2400.0f)` - which set targets and return; the
 * work happens in onPeriodic(). That is what makes teleop and autonomous
 * interchangeable: both are callers speaking the same vocabulary rather than
 * two control paths.
 *
 * Concrete subsystems are constructed as members of the robot config
 * (app/tactical/config/Robot<year>.hpp) and live for the whole run. There is no
 * heap here and nothing is ever deleted.
 ******************************************************************************/
class Subsystem
{
public:
  /**
   * @brief Virtual destructor, so deletion through a Subsystem* would be well
   *        defined.
   *
   * @note Nothing in this firmware ever destroys a subsystem: they are members
   *       of the robot config with static storage duration, and the no-heap
   *       rule means there is nowhere to return storage to. This is
   *       correctness insurance against a future holder, not a lifecycle hook -
   *       do not put shutdown behaviour here. Commanding hardware safe is
   *       onDisable()'s job, and it happens while the robot is still running.
   */
  virtual ~Subsystem() = default;

  /**
   * @brief Name used in telemetry, console output and fault messages.
   *
   * @return Pointer to a string literal with static storage duration.
   *
   * @note Must be callable at any time, including from a fault path before the
   *       scheduler is running. Return a literal; never format a name.
   */
  virtual const char* name() const = 0;

  /**
   * @brief One-time hardware setup, run during boot self-test.
   *
   * Called once, before the scheduler starts, while it is still safe to take
   * as long as a driver's init needs. This is the only place a subsystem may
   * block.
   *
   * @return true on success; false fails the self-test and boots into Fault.
   *
   * @note Defaulted to true for the subsystem that has nothing to bring up.
   *       A subsystem owning drivers should override it and return the AND of
   *       its drivers' init results, so that a failure is reported rather than
   *       discovered when the mechanism does not move.
   */
  virtual bool onInit() { return true; }

  /**
   * @brief Run this subsystem's loop for one cycle and write its actuators.
   *
   * Called only while enabled, once per control cycle, in the fixed order the
   * robot config lists its subsystems.
   *
   * @param ctx Timing, mode, sensors and driver input for this cycle. It is the
   *            whole of a subsystem's world; needing something not in here is
   *            reaching for hardware this subsystem does not own.
   *
   * @note Must never block, delay or allocate. The cycle has a 5 ms budget at
   *       200 Hz and overruns are a fault trigger (§5.2), so a subsystem that
   *       waits on hardware stops the whole robot rather than just itself.
   */
  virtual void onPeriodic(const RobotContext& ctx) = 0;

  /**
   * @brief Put the hardware in a known-safe state.
   *
   * @note Called on every disable, fault and link loss - every cycle the robot
   *       is not enabled, not once on the transition. It must therefore be
   *       cheap and safe to call repeatedly, and it must COMMAND HARDWARE
   *       rather than set a flag: a subsystem that only clears an internal
   *       target leaves the last duty cycle on the motor, which is exactly the
   *       failure this contract exists to make impossible.
   *
   * @note Also called after onInit() fails and while the robot is in Fault, so
   *       it may run when the subsystem's own state is not fully initialised.
   *       Do not assume onPeriodic() has ever run.
   */
  virtual void onDisable() = 0;

  /**
   * @brief Append this subsystem's values to the outgoing telemetry frame.
   *
   * @param sink Formatter to append named values to.
   *
   * @note const, and deliberately so: publishing reports state, it never
   *       computes or commands. Defaulted to a no-op so that a subsystem with
   *       nothing to say costs nothing, and so that phase 0 and phase 1
   *       subsystems compile before TelemetrySink exists.
   */
  virtual void publishTelemetry(TelemetrySink& sink) const { (void)sink; }
};

/* EOF -----------------------------------------------------------------------*/
