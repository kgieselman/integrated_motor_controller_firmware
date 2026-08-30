/*******************************************************************************
 * @file Encoder.hpp
 * @brief Single-channel input-capture encoder driver.
 *
 * The Rev A board brings one encoder channel per wheel to a timer input:
 *
 *   - Encoder 0 — PC2, TIM4 CH4 (input capture)
 *   - Encoder 1 — PC6, TIM8 CH1 (input capture)
 *
 * ONE CHANNEL MEANS NO DIRECTION. A quadrature pair tells you which way the
 * shaft turned; a single channel only tells you that it turned. This driver
 * therefore takes the sign from the caller via setDirection() — normally the
 * direction the motor was last commanded to run. That is correct except during
 * a reversal, and while the robot is being pushed. It is good enough to close a
 * velocity loop; it is NOT good enough to trust for odometry.
 *
 * VELOCITY IS MEASURED BY INTERVAL, NOT BY COUNTING PER WINDOW. Counting edges
 * in a fixed window makes resolution collapse as the window shrinks — a
 * low-count-per-revolution encoder at a 200 Hz update rate can yield well under
 * one edge per window, at which point the velocity signal is quantisation
 * noise. Input capture timestamps every edge in hardware, so velocity comes
 * from the reciprocal of the measured period and its resolution is set by the
 * timer clock rather than by how often update() runs.
 *
 * Wiring (application side):
 * @code
 *   Encoder encoder0(&htim4, TIM_CHANNEL_4, kPulsesPerRev, kTimerClockHz);
 *   encoder0.init();
 *
 *   extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
 *   {
 *     if (htim->Instance == TIM4 &&
 *         htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
 *     {
 *       encoder0.onCapture();
 *     }
 *   }
 *
 *   // Each control cycle, in order:
 *   encoder0.setDirection(commandedDutySign);
 *   encoder0.update();
 *   const float rpm = encoder0.velocityRpm();
 * @endcode
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "stm32h5xx_hal.h"
}

#include <cstdint>

/**
 * @brief Wheel encoder on one timer input-capture channel.
 */
class Encoder final
{
public:
  /// Age beyond which the last capture is considered stale and velocity reads
  /// as zero. Below roughly 1/kStaleTimeoutMs revolutions per second the shaft
  /// is indistinguishable from stopped.
  static constexpr uint32_t kStaleTimeoutMs = 100U;

  /**
   * @brief Construct an encoder bound to one timer capture channel.
   *
   * @param htim         HAL timer handle configured for input capture.
   * @param channel      TIM channel constant (e.g. TIM_CHANNEL_4).
   * @param pulsesPerRev Capture edges per full shaft revolution. This is the
   *                     RAW single-channel edge count — do NOT pre-multiply by
   *                     four as you would for quadrature.
   * @param timerClockHz Timer counter frequency after its prescaler, in hertz.
   *                     Velocity is derived from this, so an incorrect value
   *                     scales every reading.
   */
  Encoder(TIM_HandleTypeDef* htim,
          uint32_t           channel,
          uint32_t           pulsesPerRev,
          uint32_t           timerClockHz);

  /**
   * @brief Start input capture with interrupts.
   *
   * @return true if HAL_TIM_IC_Start_IT() succeeded.
   */
  bool init();

  /**
   * @brief Record one capture event.
   *
   * Call from HAL_TIM_IC_CaptureCallback for this timer and channel.
   *
   * @note ISR context. Performs one capture read, one subtraction and three
   *       stores; calls no FreeRTOS API and never blocks.
   */
  void onCapture();

  /**
   * @brief Supply the direction the shaft is believed to be turning.
   *
   * @param sign Positive for forward, negative for reverse, zero to hold the
   *             previous sign. Normally the sign of the commanded motor duty.
   */
  void setDirection(int8_t sign);

  /**
   * @brief Recompute velocity and accumulate position.
   *
   * Call once per control cycle, after setDirection(). Snapshots the ISR-owned
   * state under a short critical section so a capture landing mid-read cannot
   * produce a torn value.
   */
  void update();

  /**
   * @brief Accumulated signed position in capture edges.
   *
   * Signed using whatever was last passed to setDirection(), so it inherits
   * that value's limitations. 32-bit deliberately: a 64-bit counter cannot be
   * read atomically on Cortex-M33 and would need a critical section at every
   * call site. At 2000 pulses/rev and 5000 rpm this wraps after several hours
   * of continuous running — far longer than a match.
   *
   * @return Accumulated edge count.
   */
  int32_t count() const;

  /**
   * @brief Most recently computed shaft velocity.
   *
   * @return Revolutions per minute, signed by the commanded direction. Reads
   *         exactly zero when no edge has arrived for kStaleTimeoutMs.
   */
  float velocityRpm() const;

  /**
   * @brief Reset the accumulated position to zero. Does not affect velocity.
   */
  void resetCount();

private:
  TIM_HandleTypeDef* m_htim;         ///< HAL timer handle.
  uint32_t           m_channel;      ///< TIM channel constant.
  uint32_t           m_pulsesPerRev; ///< Capture edges per shaft revolution.
  uint32_t           m_timerClockHz; ///< Timer counter frequency (Hz).
  uint32_t           m_counterMask;  ///< 0xFFFF or 0xFFFFFFFF; set in init().

  // ISR-owned. Read by update() under a critical section.
  volatile uint32_t m_lastCapture;     ///< Counter value at the previous edge.
  volatile uint32_t m_interval;        ///< Ticks between the last two edges.
  volatile uint32_t m_pulseCount;      ///< Free-running edge count.
  volatile uint32_t m_lastCaptureMs;   ///< HAL_GetTick() at the last edge.
  volatile bool     m_haveFirstCapture;///< False until one edge has been seen.

  // Task-owned.
  int32_t  m_position;        ///< Accumulated signed edge count.
  uint32_t m_lastPulseCount;  ///< m_pulseCount as of the previous update().
  float    m_velocityRpm;     ///< Last computed velocity.
  int8_t   m_direction;       ///< Last direction supplied by the caller.
};

/* EOF -----------------------------------------------------------------------*/
