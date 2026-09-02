/*******************************************************************************
 * @file HeartbeatTask.cpp
 * @brief Indicators and the liveness-gated watchdog refresh, at priority 1.
 *
 * Implements the four signals of docs/tactical_architecture.md §5.4 and the
 * "control task stall" row of §5.2, driven entirely from the RobotState
 * snapshot the control task publishes at step 7 of every cycle. This task reads
 * no hardware and holds no state the control cycle depends on: it is a pure
 * consumer, which is why it can sit at the bottom of the priority table and
 * block in the buzzer without endangering anything.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "tasks/HeartbeatTask.hpp"

#include "tasks/ControlTask.hpp"
#include "tasks/RobotState.hpp"
#include "tasks/Tasks.hpp"

#include "platform/SafetyMonitor.hpp"
#include "platform/Watchdog.hpp"

extern "C"
{
#include "FreeRTOS.h"
#include "task.h"
}

#include <cstdint>

/* File-local constants ------------------------------------------------------*/

/// Task period. §3.1's 100 ms - the resolution every blink pattern is built on.
static constexpr TickType_t kHeartbeatPeriodTicks = pdMS_TO_TICKS(100U);

/// Slots in one blink frame. Ten slots of 100 ms makes the frame exactly one
/// second, which is long enough to hold a double-flash and a legible pause.
static constexpr uint32_t kBlinkSlots = 10U;

/*******************************************************************************
 * @brief LED_0 blink patterns, one bit per 100 ms slot, bit 0 first.
 *
 * §5.4 asks for "slow = Disabled, fast = Teleop, double = Auto, solid = Fault".
 * Encoding each as a ten-bit mask rather than as four counters means the whole
 * indicator vocabulary is one table a human can read, and adding a fifth
 * pattern later is a constant rather than another branch in the loop.
 ******************************************************************************/
namespace BlinkPattern
{
  /// Slow: 500 ms on, 500 ms off. The resting state, and unmistakably calm.
  static constexpr uint16_t kDisabled = 0b0000011111U;

  /// Fast: 100 ms on, 100 ms off - 5 Hz. Reads as "live" from across a field.
  static constexpr uint16_t kTeleop = 0b0101010101U;

  /// Double: two 100 ms flashes, then a 700 ms pause.
  static constexpr uint16_t kAuto = 0b0000000101U;

  /// Solid. A latched fault is not a state worth animating.
  static constexpr uint16_t kFault = 0b1111111111U;
} // namespace BlinkPattern

/// Chirps queued when the robot becomes enabled. §5.4: "one chirp on enable".
static constexpr uint8_t kChirpsOnEnable = 1U;

/// Chirps queued when the robot returns to Disabled. §5.4: "two on disable".
static constexpr uint8_t kChirpsOnDisable = 2U;

/// Chirps in one repetition of the fault pattern (§5.4: "a repeating pattern").
static constexpr uint8_t kChirpsOnFault = 2U;

/// Ticks between repetitions of the fault pattern. Ten is one second - often
/// enough that nobody misses it, rarely enough that nobody unplugs the board.
static constexpr uint32_t kFaultChirpPeriodTicks = 10U;

/* Task-owned objects --------------------------------------------------------*/

// Non-owning. Constructed and initialised in main_tactical.cpp, which keeps
// board-level hardware in one readable place; bound here by heartbeatTaskInit().
static Led*    s_pLedMode  = nullptr; ///< DEBUG_LED_0 - mode.
static Led*    s_pLedLink  = nullptr; ///< DEBUG_LED_1 - link health.
static Led*    s_pLedFault = nullptr; ///< DEBUG_LED_2 - latched fault.
static Buzzer* s_pBuzzer   = nullptr; ///< Passive transducer.

/* Task-owned state ----------------------------------------------------------*/

/// Mode seen on the previous tick, so a change can be detected without the
/// control task having to announce one. Starts at the mode RobotState itself
/// defaults to, so the first tick after boot is not read as a transition.
static ControlMode s_prevMode = ControlMode::Disabled;

/// Chirps still owed to the buzzer. Emitted at most one per tick; see
/// updateBuzzer() for why the queue exists at all.
static uint8_t s_chirpsPending = 0U;

/// Tick at which the current fault chirp pattern was last queued.
static uint32_t s_faultChirpTick = 0U;

/// RobotState::liveness as of the last IWDG refresh. Starts at RobotState's own
/// default, so a control task that never publishes never earns a refresh.
static uint32_t s_lastLiveness = 0U;

/* Private helpers -----------------------------------------------------------*/

/**
 * @brief Select the LED_0 blink pattern for a mode.
 *
 * @param mode Mode the control cycle last ran in.
 *
 * @return Ten-bit slot mask; bit n is the LED state during slot n of the frame.
 */
static uint16_t blinkPatternFor(ControlMode mode)
{
  switch (mode)
  {
    case ControlMode::Teleop:
      return BlinkPattern::kTeleop;

    case ControlMode::Auto:
      return BlinkPattern::kAuto;

    case ControlMode::Fault:
      return BlinkPattern::kFault;

    case ControlMode::Disabled:
    default:
      // Default and Disabled share a branch deliberately: an unrecognised mode
      // should look like the safe one, not like a lit indicator nobody can read.
      return BlinkPattern::kDisabled;
  }
}

/**
 * @brief Drive all three debug LEDs from one snapshot.
 *
 * @param state Snapshot published by the control task at step 7 of its cycle.
 * @param tick  Monotonic count of heartbeat ticks; selects the blink slot.
 *
 * @note The frame phase is not reset on a mode change. Restarting it would
 *       delay the first flash of the new pattern by up to a second; letting the
 *       pattern change mid-frame makes the indicator respond immediately, which
 *       is the only property that matters when someone is watching it to find
 *       out what the robot just did.
 */
static void updateLeds(const RobotState& state, uint32_t tick)
{
  const uint16_t pattern = blinkPatternFor(state.mode);
  const uint32_t slot    = tick % kBlinkSlots;

  s_pLedMode->set(((pattern >> slot) & 1U) != 0U);

  // §5.4: LED_1 is "radio link healthy (last frame < 250 ms old)". The
  // threshold is SafetyMonitor's own kLinkTimeoutMs rather than a second copy
  // of 250, so the light and the failsafe can never disagree about what healthy
  // means. DriverInput::kAgeNeverReceived is 0xFFFFFFFF, so a robot that has
  // never seen a frame fails this comparison without a special case.
  s_pLedLink->set(state.linkAgeMs < SafetyMonitor::kLinkTimeoutMs);

  s_pLedFault->set(state.faultLatched);
}

/**
 * @brief Queue and emit the buzzer patterns of §5.4.
 *
 * One chirp on enable, two on disable, and a repeating two-chirp pattern for as
 * long as a fault is held.
 *
 * @param state Snapshot published by the control task.
 * @param tick  Monotonic count of heartbeat ticks.
 *
 * @note At most ONE chirp is emitted per tick, which is why a pending count
 *       exists rather than a loop. Buzzer::chirp() busy-waits for kChirpMs
 *       (60 ms); two back to back would overrun this task's 100 ms period and
 *       stretch every blink slot behind them. Spreading them one per tick makes
 *       a "double chirp" 100 ms apart, which is what it sounds like anyway.
 *
 * @note Priority 1 is the only place Buzzer::beep() may be called from, so this
 *       function is the only caller in the running firmware. main_tactical.cpp
 *       chirps once more, before the scheduler starts, where nothing preempts.
 */
static void updateBuzzer(const RobotState& state, uint32_t tick)
{
  if (state.mode != s_prevMode)
  {
    if (state.mode == ControlMode::Fault)
    {
      // A fault entered from Teleop is also a disable, but it gets the fault
      // pattern rather than both: the repeating chirp is the more urgent of the
      // two messages and layering them would just sound like noise.
      s_chirpsPending  = kChirpsOnFault;
      s_faultChirpTick = tick;
    }
    else if ((state.mode == ControlMode::Teleop) || (state.mode == ControlMode::Auto))
    {
      s_chirpsPending = kChirpsOnEnable;
    }
    else
    {
      s_chirpsPending = kChirpsOnDisable;
    }

    s_prevMode = state.mode;
  }
  else if (state.mode == ControlMode::Fault)
  {
    // Unsigned subtraction, so the comparison stays correct across the tick
    // counter's 2^32 wrap.
    if ((tick - s_faultChirpTick) >= kFaultChirpPeriodTicks)
    {
      s_chirpsPending  = kChirpsOnFault;
      s_faultChirpTick = tick;
    }
  }
  else
  {
    // Steady in a non-fault mode: nothing to queue.
  }

  if (s_chirpsPending > 0U)
  {
    --s_chirpsPending;
    s_pBuzzer->chirp();
  }
}

/**
 * @brief Refresh the independent watchdog, but only if the control task moved.
 *
 * This is the "control task stall" row of §5.2 and the entire reason the
 * watchdog refresh lives in the heartbeat task. Refreshing unconditionally
 * would prove only that the scheduler still runs this task; gating it on
 * RobotState::liveness makes the refresh a statement about the 200 Hz cycle,
 * which is the thing whose stall nothing else in the firmware can detect - a
 * task cannot notice itself wedging.
 *
 * @param state Snapshot published by the control task.
 *
 * @note Compared with != rather than <. liveness wraps at 2^32 (about 248 days
 *       at 200 Hz) and RobotState.hpp asks consumers not to order it.
 *
 * @note §5.2 specifies the stall threshold as "no advance in 50 ms". This task
 *       runs at 100 ms (§3.1), so the real detection granularity is one to two
 *       heartbeat ticks. The IWDG reload period is what sets the actual reset
 *       delay: Watchdog::kReloadMs is 500 ms, sized for exactly that worst case
 *       plus the blocking chirp that can land inside one of those ticks.
 *
 * @note The counter is started by watchdogStart() in main(), immediately before
 *       the scheduler. If that call failed, LED_2 is lit at boot and this
 *       refresh is a write to a counter that is not running - the gate is still
 *       correct, but no stall resets the board.
 */
static void refreshWatchdogIfControlAlive(const RobotState& state)
{
  if (state.liveness == s_lastLiveness)
  {
    return;
  }

  s_lastLiveness = state.liveness;
  watchdogRefresh();
}

/* Task ----------------------------------------------------------------------*/

/**
 * @brief Drive the indicators and the watchdog from the RobotState snapshot.
 *
 * @param pvParameters Unused.
 *
 * @note Order inside the tick is deliberate. The snapshot is read exactly once
 *       so that the LEDs, the buzzer and the watchdog all describe the same
 *       cycle. The watchdog refresh precedes the buzzer because chirp() blocks
 *       for up to 60 ms, and a refresh must never be delayed by a sound.
 */
static void heartbeatTaskEntry(void* pvParameters)
{
  (void)pvParameters;

  uint32_t tick = 0U;

  for (;;)
  {
    const RobotState state = controlTaskRobotState().read();

    updateLeds(state, tick);
    refreshWatchdogIfControlAlive(state);
    updateBuzzer(state, tick);

    ++tick;
    vTaskDelay(kHeartbeatPeriodTicks);
  }
}

/* Public interface ----------------------------------------------------------*/

void heartbeatTaskInit(Led& ledMode, Led& ledLink, Led& ledFault, Buzzer& buzzer)
{
  s_pLedMode  = &ledMode;
  s_pLedLink  = &ledLink;
  s_pLedFault = &ledFault;
  s_pBuzzer   = &buzzer;
}

bool heartbeatTaskCreate()
{
  // Guard clause rather than a null check inside the loop: the binding cannot
  // change once the task is running, so the question is worth asking exactly
  // once, here, where the answer can still stop the scheduler from starting.
  if ((s_pLedMode == nullptr) || (s_pLedLink == nullptr) || (s_pLedFault == nullptr)
      || (s_pBuzzer == nullptr))
  {
    return false;
  }

  return (xTaskCreate(heartbeatTaskEntry,
                      "Heartbeat",
                      Tasks::kHeartbeatStackWords,
                      nullptr,
                      Tasks::kHeartbeatPriority,
                      nullptr) == pdPASS);
}

/* EOF -----------------------------------------------------------------------*/
