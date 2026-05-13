/*******************************************************************************
 * @file Encoder.hpp
 * @brief Quadrature encoder wrapper using STM32 TIM in encoder interface mode.
 *
 * Each encoder occupies one TIM instance configured by CubeMX in
 * TIM_ENCODERMODE_TI12 (both edges on both channels). The counter is 32-bit
 * on TIM2/TIM5 and 16-bit on TIM3/TIM4/TIM8/TIM12 — pass the correct handle.
 *
 * Pin assignments (from board_pins.h):
 *  - Left  encoder A: PA0 (TIM2 CH1), B: PA1 (TIM2 CH2)
 *  - Right encoder A: PC6 (TIM8 CH1), B: PC7 (TIM8 CH2)
 *
 * @note Call update() at a fixed rate from your control-loop tick to compute
 *       velocity. The rate passed to the constructor must match the actual call
 *       rate; mismatches produce incorrect velocity readings.
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
 * @brief Quadrature encoder interface around a HAL TIM handle.
 */
class Encoder
{
public:
  /**
   * @brief Construct an Encoder instance.
   *
   * @param htim          Pointer to the HAL TIM handle (configured in encoder mode).
   * @param countsPerRev  Encoder pulses per full shaft revolution (PPR × 4 for
   *                      quadrature). Used for velocity calculation.
   * @param updateRateHz  Frequency (Hz) at which update() will be called. Used to
   *                      convert count delta to RPM.
   */
  Encoder(TIM_HandleTypeDef* htim,
          uint32_t           countsPerRev,
          float              updateRateHz);

  /**
   * @brief Initialise the encoder and start the TIM counter.
   *
   * Call once after MX_TIMx_Init() has completed.
   *
   * @return true on success, false if HAL_TIM_Encoder_Start() fails.
   */
  bool init();

  /**
   * @brief Sample the counter and update velocity estimate.
   *
   * Must be called at a fixed period equal to 1 / updateRateHz.
   * Handles 16-bit counter overflow/underflow correctly.
   */
  void update();

  /**
   * @brief Return the cumulative position in encoder counts.
   *
   * Accumulates across counter overflows; resets only on resetCount().
   *
   * @return Signed 64-bit count value.
   */
  int64_t count() const;

  /**
   * @brief Return instantaneous velocity in RPM.
   *
   * Computed during the last update() call.
   *
   * @return Shaft velocity in revolutions per minute. Positive = forward.
   */
  float velocityRpm() const;

  /**
   * @brief Reset cumulative count to zero.
   *
   * Does not affect the underlying TIM counter; only zeroes the software
   * accumulator. Useful at the start of a motion segment.
   */
  void resetCount();

private:
  TIM_HandleTypeDef* m_htim;         ///< HAL TIM handle configured in encoder mode.
  uint32_t           m_countsPerRev; ///< Encoder counts per full shaft revolution.
  float              m_updateRateHz; ///< Rate at which update() is called (Hz).

  int64_t  m_accumulator;  ///< Software-accumulated position count.
  uint32_t m_lastRaw;      ///< Previous raw TIM counter value (for delta).
  float    m_velocityRpm;  ///< Velocity computed during last update().

  /**
   * @brief Read the raw TIM counter register.
   *
   * @return Raw 32-bit (or sign-extended 16-bit) counter value.
   */
  uint32_t readCounter() const;
};

/* EOF -----------------------------------------------------------------------*/
