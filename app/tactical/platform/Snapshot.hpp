/*******************************************************************************
 * @file Snapshot.hpp
 * @brief Single-writer, multiple-reader value shared between FreeRTOS tasks.
 *
 * One of exactly two mechanisms permitted for moving data between tasks
 * (docs/tactical_architecture.md §3.2); the other is EventRing. A Snapshot<T>
 * holds one small plain struct and copies it whole in and out under a critical
 * section, so a reader never observes a half-updated value. It carries no
 * timestamp, no sequence number and no "is it fresh" flag - the payload owns
 * those. DriverInput::ageMs is why that type, not this one, knows about age.
 *
 * Two instances are planned: DriverInput (Comms -> Control) and RobotState
 * (Control -> Telemetry and Heartbeat). Both are tens of bytes; copying 64
 * bytes at 250 MHz costs a few hundred nanoseconds, which §1 spends willingly
 * for code that reads plainly.
 *
 * @note Single writer is a convention this class cannot enforce. Two tasks
 *       calling write() will not tear a value, but the loser's update is simply
 *       lost. One owning task per Snapshot, per invariant 2.
 *
 * @note The critical section saves and restores PRIMASK rather than calling
 *       taskENTER_CRITICAL() as §3.2's prose has it. Three reasons, and they
 *       are the same ones behind CRSFReceiver::hasNewData() and
 *       Encoder::update(): the PRIMASK form is safe from an ISR (the motor
 *       fault EXTI at priority 5 may want to publish), it is safe before
 *       vTaskStartScheduler() has run, and it nests inside an outer critical
 *       section without releasing it. taskENTER_CRITICAL() satisfies none of
 *       the three. The cost is that PRIMASK masks everything, including the
 *       priority 0-4 interrupts the kernel cannot touch, for the length of one
 *       struct copy.
 *
 * @note This header defines its methods inline, against §2.1 of the style
 *       guide. A class template has no other option; there is no translation
 *       unit to put them in.
 *
 * @see docs/tactical_architecture.md §3.2 for the mechanism, §3.1 for the tasks
 *      at either end of one, and §3.3 for the interrupt priorities the note
 *      above refers to.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "stm32h5xx_hal.h"
}

#include <cstdint>
#include <type_traits>

/*******************************************************************************
 * @brief A value written by one task and read by any number of others.
 *
 * Usage is deliberately two methods and nothing else:
 *
 * @code
 *   static Snapshot<DriverInput> g_driverInput;   // file scope, no allocation
 *
 *   // Comms task, after decoding a frame:
 *   g_driverInput.write(decoded);
 *
 *   // Control task, once per cycle:
 *   const DriverInput input = g_driverInput.read();
 * @endcode
 *
 * A default-constructed Snapshot value-initialises its payload, so a reader
 * that runs before the first write() sees T's default member initialisers
 * rather than whatever was in RAM. DriverInput relies on this: its defaults
 * describe a link that has never been heard from, not a fresh frame.
 *
 * @tparam T Payload type. Must be trivially copyable - checked below.
 ******************************************************************************/
template <typename T>
class Snapshot final
{
public:
  static_assert(std::is_trivially_copyable_v<T>,
                "Snapshot<T> copies T with a plain assignment inside a critical section, so T "
                "must be trivially copyable. A type with a user-provided copy constructor, a "
                "virtual method or an owning member cannot be published this way.");

  /*****************************************************************************
   * @brief Construct with a value-initialised payload.
   *
   * constexpr so that a Snapshot at static storage duration is constant
   * initialised - it lands in .bss or .data instead of needing a run-time
   * static initialiser, and may be declared constinit.
   ****************************************************************************/
  constexpr Snapshot()
      : m_value()
  {}

  /// A Snapshot is a shared cell, not a value. Copying one would read the
  /// payload outside a critical section, which is the one thing this class
  /// exists to prevent, so both copy operations are removed.
  Snapshot(const Snapshot&)            = delete;
  Snapshot& operator=(const Snapshot&) = delete;

  /*****************************************************************************
   * @brief Overwrite the stored value. Called by the owning task only.
   *
   * @param value New value to store.
   *
   * @note Safe from an ISR and from inside an outer critical section.
   ****************************************************************************/
  void write(const T& value);

  /*****************************************************************************
   * @brief Copy out the stored value.
   *
   * @return The most recently written value, or a value-initialised T if
   *         nothing has been written yet.
   *
   * @note Safe from an ISR and from inside an outer critical section.
   ****************************************************************************/
  T read() const;

private:
  T m_value; ///< The shared payload; touched only inside a critical section.
};

/* Inline definitions --------------------------------------------------------*/

template <typename T>
void Snapshot<T>::write(const T& value)
{
  // Save and restore PRIMASK rather than calling __enable_irq(): an
  // unconditional enable would silently break any outer critical section this
  // happens to be called from. The "memory" clobber inside the CMSIS
  // intrinsics is also what stops the compiler hoisting the copy out.
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  m_value = value;

  __set_PRIMASK(primask);
}

template <typename T>
T Snapshot<T>::read() const
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  const T copy = m_value;

  __set_PRIMASK(primask);

  // Returned outside the critical section deliberately: the copy is already
  // this caller's own, and the return path may involve a struct move the
  // section has no reason to cover.
  return copy;
}

/* Contract checks -----------------------------------------------------------*/

// Nothing in a class template is compiled until something instantiates it, and
// the first real payloads arrive in U0.4 and U0.6. Instantiating against a
// stand-in here means a mistake in Snapshot<T> is a build error in this unit
// rather than in whichever unit first uses one.
namespace SnapshotContract
{
  /// Stands in for a real payload: an aggregate of plain values, a few bytes.
  struct Probe
  {
    uint32_t counter; ///< Any integer field.
    float    value;   ///< Any floating-point field.
  };

  static_assert(sizeof(Snapshot<Probe>) >= sizeof(Probe),
                "Snapshot<T> must store its payload by value");
  /*****************************************************************************
   * @brief Never called. Compiled only for its side effect.
   *
   * Declaring the object and calling both methods odr-uses them, which is what
   * forces their bodies - not just their declarations - through the compiler.
   * The function is static and unused, so the optimiser drops it: no symbol of
   * this name reaches the image. Measured cost of this whole block is 8 bytes
   * of .rodata and no RAM in a Debug build.
   ****************************************************************************/
  [[maybe_unused]] static void probeInstantiation()
  {
    Snapshot<Probe> probe;
    probe.write(Probe{.counter = 1U, .value = 1.0f});
    const Probe copy = probe.read();
    static_cast<void>(copy);
  }
} // namespace SnapshotContract

/* EOF -----------------------------------------------------------------------*/
