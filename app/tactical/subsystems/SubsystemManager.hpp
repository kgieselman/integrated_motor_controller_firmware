/*******************************************************************************
 * @file SubsystemManager.hpp
 * @brief Walks the robot's subsystem array and calls the Subsystem contract.
 *
 * The whole of the dispatch layer. It holds a non-owning view of the array the
 * robot config built, and turns "the robot is enabled" or "the robot is not"
 * into the matching call on every subsystem, in a fixed order, once per cycle.
 *
 * It knows nothing about any particular mechanism, and that is the point: the
 * array it walks is the only place the set of subsystems appears, so adding a
 * launcher never edits this file (invariant 4, §2.1).
 *
 * Unit U0.3 in docs/work_units.md creates this. See
 * docs/tactical_architecture.md §6 for the contract it dispatches, §4 for
 * where in the control cycle it is called, and §5.3 for why the enabled/
 * disabled decision lives here and not inside each subsystem.
 *
 * NO HAL TYPES APPEAR HERE, DELIBERATELY - invariant 3.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include "subsystems/Subsystem.hpp"

#include <cstddef>
#include <span>

/*******************************************************************************
 * @brief Non-owning dispatcher over a fixed array of subsystems.
 *
 * Ownership stays with the robot config, which constructs every subsystem as a
 * member and lists their addresses in a `Subsystem*` array. This class holds a
 * pointer to that array and a count, nothing more: no ownership, no allocation,
 * no container. The array and everything in it must outlive the manager, which
 * on this robot they do trivially - both live for the whole run.
 *
 * Order is contractual. periodic() walks the array front to back, so the order
 * the config lists its subsystems in is the order they run in every cycle, and
 * it is stable and reviewable in one place.
 *
 * THE CALLER CHOOSES BETWEEN periodic() AND disableAll(), EVERY CYCLE. This
 * class does not look at ctx.mode, deliberately. The control task has already
 * asked SafetyMonitor for a verdict by the time it gets here, and a second
 * enabled/disabled decision made in a second place is precisely the bug class
 * "safe is a state, not a check" (§5.3) deletes. Exactly one of the two is
 * called per cycle, never both and never neither.
 *
 * Usage, from the robot config and the control task:
 *
 * @code
 * // config/Robot2026.hpp - ownership and order live here.
 * Subsystem* m_subsystems[2] = {&m_drive, &m_launcher};
 * SubsystemManager m_manager{m_subsystems};
 *
 * // tasks/ControlTask.cpp - one of these two, once per cycle.
 * if (ctx.mode == ControlMode::Teleop || ctx.mode == ControlMode::Auto)
 * {
 *   manager.periodic(ctx);
 * }
 * else
 * {
 *   manager.disableAll();
 * }
 * @endcode
 ******************************************************************************/
class SubsystemManager final
{
public:
  /**
   * @brief Bind the manager to the robot config's subsystem array.
   *
   * @param subsystems View of the array of subsystem pointers, in the order
   *                   they should run. Neither the array nor any subsystem in
   *                   it is copied or owned; both must outlive this manager.
   *                   No entry may be nullptr.
   *
   * @note Takes a std::span rather than a (pointer, count) pair because the
   *       style guide §1 requires it, and because it earns its keep here: a
   *       raw C array converts implicitly, so the config layer writes
   *       `SubsystemManager m_manager{m_subsystems};` and cannot get the count
   *       wrong. The members below are still just a pointer and a count - the
   *       span is the parameter, not the storage.
   *
   * @note A nullptr entry is not checked for. The array is a hand-written list
   *       of the addresses of members in one file whose entire job is to be
   *       read (invariant 5), so a runtime check would cost every cycle to
   *       guard against a typo that does not survive a first run. An empty
   *       span is legal and makes every method a no-op, which is what a robot
   *       config with no mechanisms yet should do.
   */
  explicit SubsystemManager(std::span<Subsystem* const> subsystems);

  /**
   * @brief Call onInit() on every subsystem, in array order, during boot.
   *
   * Runs once, before the scheduler starts.
   *
   * @return true only if every subsystem returned true.
   *
   * @note Every subsystem is initialised even after one has failed - the walk
   *       does not short-circuit. A subsystem that never gets its onInit() is
   *       a subsystem whose hardware is in whatever state reset left it in,
   *       and the boot self-test should report all the failures it can see
   *       rather than only the first.
   */
  bool initAll();

  /**
   * @brief Run one control cycle: onPeriodic() on every subsystem, in order.
   *
   * @param ctx This cycle's timing, mode, sensors and driver input, passed
   *            unchanged to every subsystem so they all see the same world.
   *
   * @note Call this only when the robot is enabled. It does not check the mode
   *       - see the class note above.
   */
  void periodic(const RobotContext& ctx);

  /**
   * @brief Command every subsystem into its safe state.
   *
   * @note UNCONDITIONAL, BY DESIGN. Every subsystem gets onDisable() on every
   *       call, in array order. This class never tracks which subsystems it
   *       believes are enabled in order to skip the rest: that bookkeeping is
   *       one stale bool away from a mechanism that keeps running through a
   *       link loss, and the calls are cheap enough that the question is not
   *       worth asking.
   *
   * @note Safe to call at any time, including before initAll() and repeatedly
   *       every cycle while the robot rests in Disabled.
   */
  void disableAll();

private:
  /// Non-owning pointer to the robot config's array of subsystem pointers.
  Subsystem* const* m_subsystems;

  /// Number of entries in that array; may be zero.
  std::size_t m_count;
};

/* EOF -----------------------------------------------------------------------*/
